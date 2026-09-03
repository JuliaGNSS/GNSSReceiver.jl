# ─────────────────────────────────────────────────────────────────────────────
# Where acquisition runs (GNSSReceiver.jl #107)
#
# One acquisition scan is a full FM-DBZP grid search over the buffered window —
# tens of milliseconds on a workstation, *seconds* on an embedded host at a few
# MS/s. Run inline (the default), it happens on the same task that tracks every
# chunk, so for its whole duration no chunk is processed. Offline that costs
# nothing: the file waits. On a live receiver it is a real-time overrun — the
# stream buffers up, and when the pipeline catches up it does so by processing
# stale chunks back to back. For a hardware-correlator receiver that is fatal
# rather than merely slow: NCO corrections computed from a stale chunk are
# scheduled at sample indices the device has long passed, so every channel
# free-runs on its last words (~0.1 chips/s of code-NCO quantisation drift) and
# a stall beyond ~10 s loses every lock.
#
# So acquisition can also run *off* the processing task: the chunk pipeline
# hands a copy of the window to a worker and keeps tracking, and the worker's
# results are merged into the receiver state on whatever chunk they come back
# on. This is opt-in (`acquire_async`) because it only makes sense in real time:
# replaying a file, the samples arrive as fast as they can be read, and a scan
# dispatched at the start of a short recording would still be running when the
# stream ends — inline acquisition is both faster and deterministic there. The
# [`receive`](@ref) method for an [`AbstractHardwareCorrelatorSDR`](@ref)
# enables it by default; the file-replay and simulated paths do not.
#
# The split is what makes it safe: the worker only ever touches its own copy of
# the samples and its own acquisition plans, and the receiver state — the
# `TrackState`, the satellite states — is still mutated only by the processing
# task, at the merge.
# ─────────────────────────────────────────────────────────────────────────────

"""
    InlineAcquisition

Run acquisition on the processing task, inside [`process`](@ref) (the default).
Deterministic and the right choice for offline replay; see
[`AsyncAcquisition`](@ref) for the real-time alternative.
"""
struct InlineAcquisition end

# One dispatched scan: an owned copy of the acquisition window, the PRNs to
# search per tracking group, and the runtime the window *starts* at — the time
# base the returned code phases refer to, and all the merge needs to advance
# them to the chunk it runs on.
struct AcquisitionRequest{T,P<:NamedTuple}
    samples::Vector{T}
    prns::P
    window_start_runtime::typeof(1.0s)
end

# What comes back: one vector of `Acquisition.AcquisitionResults` per group, the
# window's start runtime (echoed, so the response is self-describing) and the
# sample store, returned so the next dispatch can reuse it instead of allocating
# a fresh multi-megabyte window per scan.
struct AcquisitionResponse{T,R<:NamedTuple}
    results::R
    window_start_runtime::typeof(1.0s)
    samples::Vector{T}
    # Wall-clock seconds the search took — the quantity that decides whether
    # acquisition can run inline at all, so it is measured rather than assumed.
    scan_seconds::Float64
end

# One band's acquisition worker. `in_flight` is read and written only by the
# processing task (dispatch sets it, the merge clears it), so the two tasks
# communicate exclusively through the two channels.
mutable struct BandAcquisitionWorker{T,P<:NamedTuple,R<:NamedTuple}
    const requests::Channel{AcquisitionRequest{T,P}}
    const responses::Channel{AcquisitionResponse{T,R}}
    # Each group's candidate PRN list, copied out of the acquisition plans at
    # construction. The plans themselves belong to the worker task from then on,
    # so the dispatcher must not read them to decide what to search.
    const avail_prns::P
    # Returned by a scan that threw: the merge then simply has nothing to do.
    const empty_results::R
    # Recycled window store (see `AcquisitionResponse.samples`).
    spare::Vector{T}
    in_flight::Bool
    # The worker itself, so the run can wait for it to go quiescent before the
    # process tears down (see `close_acquisition!`).
    task::Union{Nothing,Task}
    # Diagnostics. A stalled or fruitless acquisition is otherwise invisible
    # once the search no longer happens on the receiver's own task, and
    # `last_scan_seconds` is the number that decides whether inline acquisition
    # was ever viable on this host.
    dispatched_scans::Int
    completed_scans::Int
    detected_prns::Int
    last_scan_seconds::Float64
end

"""
    AsyncAcquisition

Run acquisition on a worker task per RF band, off the chunk-processing task, so
a scan never stalls tracking. Built by [`receive`](@ref) when `acquire_async` is
set; see the note at the top of `async_acquisition.jl` for why this is real-time
only.
"""
struct AsyncAcquisition{W<:NamedTuple}
    workers::W
end

# The search the worker task runs, as a plain function so its return type can be
# inferred (`Base.promote_op`) without running an acquisition.
_search(acq_plan, samples, prns, interm_freq, subsample_interpolation) =
    acquire!(acq_plan, samples, prns; interm_freq, subsample_interpolation)

# Concrete result-vector type of one group's scan, derived from the plan and the
# sample type rather than by running one. Falls back to the abstract element
# type if inference cannot pin it: the merge then dispatches dynamically, which
# costs one scan's worth of dynamic calls every `acquire_every` and never
# touches the per-chunk path.
function _scan_result_type(acq_plan, ::Type{T}, interm_freq) where {T}
    result_type = Base.promote_op(
        _search,
        typeof(acq_plan),
        Vector{T},
        Vector{Int},
        typeof(interm_freq),
        Bool,
    )
    isconcretetype(result_type) ? result_type : Vector{Acquisition.AcquisitionResults}
end

# Scan one group. An empty PRN list is normal — a band's other constellation may
# be the one with satellites to search — and must not run a search.
function _scan_group(
    acq_plan,
    samples,
    prns,
    interm_freq,
    subsample_interpolation,
    ::Type{V},
) where {V}
    isempty(prns) && return V()
    _search(acq_plan, samples, prns, interm_freq, subsample_interpolation)
end

# Run a whole request: every group of the band, in the group order the results
# NamedTuple is keyed by.
function _run_scan(
    acq_plans::Tuple,
    request::AcquisitionRequest,
    interm_freq,
    subsample_interpolation,
    ::Type{R},
) where {R}
    R(
        map(
            (acq_plan, prns, V) -> _scan_group(
                acq_plan,
                request.samples,
                prns,
                interm_freq,
                subsample_interpolation,
                V,
            ),
            acq_plans,
            values(request.prns),
            fieldtypes(R),
        ),
    )
end

# Spawn one band's worker. Everything the scan needs beyond the window and the
# PRN lists is invariant for the whole run, so it is captured here rather than
# shipped with every request. The plans in particular are *only* touched by this
# task from now on (`acquire!` mutates their FFT scratch), which is what keeps
# the split race-free — the inline path is not used when a worker exists.
function _spawn_band_worker(
    ::Type{T},
    group_keys::NTuple{N,Symbol},
    acq_plans::Tuple,
    interm_freq,
    subsample_interpolation,
) where {T,N}
    prn_type = NamedTuple{group_keys,NTuple{N,Vector{Int}}}
    result_types = map(plan -> _scan_result_type(plan, T, interm_freq), acq_plans)
    result_type = NamedTuple{group_keys,Tuple{result_types...}}
    # Capacity one: `in_flight` already bounds the receiver to a single
    # outstanding scan per band, so neither `put!` can ever block.
    requests = Channel{AcquisitionRequest{T,prn_type}}(1)
    responses = Channel{AcquisitionResponse{T,result_type}}(1)
    empty_results = result_type(map(V -> V(), result_types))
    avail_prns = prn_type(map(plan -> collect(Int, plan.avail_prns), acq_plans))
    worker = BandAcquisitionWorker(
        requests,
        responses,
        avail_prns,
        empty_results,
        T[],
        false,
        nothing,
        0,
        0,
        0,
        NaN,
    )
    task = Threads.@spawn while true
        request = try
            take!(requests)
        catch e
            # The run is over (`close_acquisition!`).
            e isa InvalidStateException && break
            rethrow(e)
        end
        scan_start = time()
        results = try
            _run_scan(acq_plans, request, interm_freq, subsample_interpolation, result_type)
        catch e
            # A failed scan must not take the worker (or the receiver) down: the
            # next one runs on the next window. Reported once rather than every
            # `acquire_every` for the rest of the run.
            @error "Asynchronous acquisition failed; skipping this scan" exception =
                (e, catch_backtrace()) maxlog = 3
            empty_results
        end
        try
            put!(
                responses,
                AcquisitionResponse(
                    results,
                    request.window_start_runtime,
                    request.samples,
                    time() - scan_start,
                ),
            )
        catch e
            e isa InvalidStateException && break
            rethrow(e)
        end
    end
    Base.errormonitor(task)
    worker.task = task
    worker
end

"""
    AsyncAcquisition(band_keys, band_systems, acq_plans, sample_types, interm_freqs,
                     subsample_interpolation)

Spawn one acquisition worker per band. `sample_types` are the per-band scalar
sample element types (the acquisition buffers'), everything else is
[`receive`](@ref)'s per-band acquisition configuration. Detection itself is not
the worker's business — it hands every result back and the merge applies the
CFAR test, so the false-alarm probability stays a `process` keyword.
"""
function AsyncAcquisition(
    band_keys::NTuple{NB,Symbol},
    band_systems::Tuple,
    acq_plans,
    sample_types::Tuple,
    interm_freqs::Tuple,
    subsample_interpolation,
) where {NB}
    if Threads.nthreads() < 2
        @warn "Asynchronous acquisition needs at least two threads to overlap a scan " *
              "with tracking; with one thread a scan still blocks the chunk pipeline. " *
              "Start Julia with `-t auto`."
    end
    workers = map(
        band_systems,
        sample_types,
        interm_freqs,
    ) do systems, sample_type, interm_freq
        group_keys = map(signal_group_key, systems)
        _spawn_band_worker(
            sample_type,
            group_keys,
            values(acq_plans[group_keys]),
            interm_freq,
            subsample_interpolation,
        )
    end
    AsyncAcquisition(NamedTuple{band_keys}(workers))
end

"""
    close_acquisition!(scheduler; timeout = 10.0)

Release the scheduler's resources at the end of a run. Closing the request
channel ends a worker's loop; the inline scheduler has nothing to release.

This **waits** (up to `timeout` seconds per band) for a scan that is still
running to finish, rather than just closing the channels and returning. A search
is seconds of compute inside `Acquisition`/Polyester, and leaving one in flight
while the caller tears the process down — `exit`, or simply the end of `main` —
faults the runtime out from under it (observed as a `SIGBUS` in
`_accumulate_prn_step_tiled!` at the end of an otherwise complete hardware run).
Waiting makes the end of a run quiescent; the timeout keeps a wedged search from
hanging the shutdown.
"""
close_acquisition!(::InlineAcquisition; timeout = 10.0) = nothing
function close_acquisition!(scheduler::AsyncAcquisition; timeout = 10.0)
    # Close the request channels first, so every worker's loop exits after the
    # scan it may be running now.
    for worker in values(scheduler.workers)
        close(worker.requests)
    end
    for worker in values(scheduler.workers)
        task = worker.task
        isnothing(task) && continue
        istaskdone(task) && continue
        # `timedwait` polls, which is exactly right here: the wait happens once,
        # at the end of a run.
        timedwait(() -> istaskdone(task), timeout) === :ok || @warn(
            "an acquisition worker was still searching after $timeout s; " *
            "the run is ending without waiting for it"
        )
    end
    # Only now close the response side: while a worker was still running, its
    # result had somewhere to go.
    for worker in values(scheduler.workers)
        close(worker.responses)
    end
    nothing
end

# ─────────────────────────────────────────────────────────────────────────────
# The per-chunk step
# ─────────────────────────────────────────────────────────────────────────────

# Advance the acquisition stage by one chunk and return the (possibly updated)
# `(track_state, receiver_sat_states, acquisition_buffers, last_time_acquisition_ran)`.
# This is the seam `process` goes through; the two schedulers differ only in
# whether the search runs here or on a worker.
function advance_acquisition(
    ::InlineAcquisition,
    receiver_state,
    band_keys,
    band_systems,
    meas,
    interm_freqs,
    acq_plans,
    sampling_freq,
    invariant_acq_args,
)
    runtime, _, acquire_every = invariant_acq_args
    # Acquisition only touches state when a periodic scan is due on some band or a satellite
    # qualifies for reacquisition. On the common steady-state frame neither holds, so skip
    # `_acquire_all_bands` entirely — its per-band buffer resets and NamedTuple merges would
    # otherwise allocate every chunk. Buffers are rebuilt only if one still holds samples
    # (they must be emptied so the next scan's coherent window stays gap-free). This is a
    # pure allocation optimisation: the full path is a no-op on these frames anyway (see
    # `acquire_band`), so results are identical.
    acq_due =
        any(t -> runtime - t >= acquire_every, values(receiver_state.last_time_acquisition_ran))
    reacq_due =
        any(d -> any(should_reacquire, d), values(receiver_state.receiver_sat_states))
    acq_due || reacq_due || return (
        receiver_state.track_state,
        receiver_state.receiver_sat_states,
        _emptied_buffers(receiver_state.acquisition_buffers),
        receiver_state.last_time_acquisition_ran,
    )
    _acquire_all_bands(
        receiver_state.track_state,
        receiver_state.receiver_sat_states,
        receiver_state.acquisition_buffers,
        receiver_state.last_time_acquisition_ran,
        band_keys,
        band_systems,
        meas,
        interm_freqs,
        acq_plans,
        invariant_acq_args,
    )
end

# Reset only if some buffer still holds samples, so a steady-state frame does no
# work at all.
_emptied_buffers(acquisition_buffers) =
    all(b -> b.current_length == 0, values(acquisition_buffers)) ? acquisition_buffers :
    map(SampleBuffers.reset, acquisition_buffers)

function advance_acquisition(
    scheduler::AsyncAcquisition,
    receiver_state,
    band_keys,
    band_systems,
    meas,
    interm_freqs,
    acq_plans,
    sampling_freq,
    invariant_acq_args,
)
    runtime, _, acquire_every = invariant_acq_args
    # Steady-state frame: no scan due on any band and no worker holding a
    # finished one. Skip the per-band recursion entirely — like the inline path,
    # it is a no-op on these frames but its NamedTuple merges and buffer resets
    # would allocate on every chunk of the run.
    if !any(worker -> isready(worker.responses), values(scheduler.workers)) &&
       !any(t -> runtime - t >= acquire_every, values(receiver_state.last_time_acquisition_ran)) &&
       !any(d -> any(should_reacquire, d), values(receiver_state.receiver_sat_states))
        return (
            receiver_state.track_state,
            receiver_state.receiver_sat_states,
            _emptied_buffers(receiver_state.acquisition_buffers),
            receiver_state.last_time_acquisition_ran,
        )
    end
    _acquire_all_bands_async(
        receiver_state.track_state,
        receiver_state.receiver_sat_states,
        receiver_state.acquisition_buffers,
        receiver_state.last_time_acquisition_ran,
        band_keys,
        band_systems,
        meas,
        interm_freqs,
        scheduler.workers,
        sampling_freq,
        invariant_acq_args,
    )
end

# Type-stable recursion over the aligned per-band tuples, mirroring
# `_acquire_all_bands`.
@inline _acquire_all_bands_async(
    track_state,
    receiver_sat_states,
    acquisition_buffers,
    last_time_acquisition_ran,
    ::Tuple{},
    ::Tuple{},
    ::Tuple{},
    ::Tuple{},
    workers,
    sampling_freq,
    invariant_acq_args,
) = (track_state, receiver_sat_states, acquisition_buffers, last_time_acquisition_ran)
@inline function _acquire_all_bands_async(
    track_state,
    receiver_sat_states,
    acquisition_buffers,
    last_time_acquisition_ran,
    band_keys::Tuple,
    band_systems::Tuple,
    measurements::Tuple,
    interm_freqs::Tuple,
    workers,
    sampling_freq,
    invariant_acq_args,
)
    band_key = first(band_keys)
    systems = first(band_systems)
    group_keys = map(signal_group_key, systems)
    track_state, band_receiver_sat_states, band_buffer, band_last_time = acquire_band_async(
        workers[band_key],
        track_state,
        receiver_sat_states[group_keys],
        acquisition_buffers[band_key],
        systems,
        first(measurements),
        last_time_acquisition_ran[band_key],
        sampling_freq,
        invariant_acq_args...,
    )
    receiver_sat_states = merge(receiver_sat_states, band_receiver_sat_states)
    acquisition_buffers =
        merge(acquisition_buffers, NamedTuple{(band_key,)}((band_buffer,)))
    last_time_acquisition_ran =
        merge(last_time_acquisition_ran, NamedTuple{(band_key,)}((band_last_time,)))
    _acquire_all_bands_async(
        track_state,
        receiver_sat_states,
        acquisition_buffers,
        last_time_acquisition_ran,
        Base.tail(band_keys),
        Base.tail(band_systems),
        Base.tail(measurements),
        Base.tail(interm_freqs),
        workers,
        sampling_freq,
        invariant_acq_args,
    )
end

# One band's asynchronous acquisition step: merge whatever a worker finished,
# then buffer and (when one is due and the worker is free) dispatch the next
# scan. `subsample_interpolation` is captured by the worker, so it is accepted
# here only to keep the invariant-argument tuple identical to the inline path's.
function acquire_band_async(
    worker::BandAcquisitionWorker,
    track_state,
    receiver_sat_states,
    acquisition_buffer,
    systems,
    measurement,
    last_time_acquisition_ran,
    sampling_freq,
    runtime,
    num_ants::NumAnts,
    acquire_every,
    acq_pfa,
    code_lock_cn0_threshold,
    subsample_interpolation,
)
    # ── Merge ────────────────────────────────────────────────────────────────
    # `isready` never blocks, so a chunk that finds nothing done does no work.
    while isready(worker.responses)
        response = take!(worker.responses)
        worker.in_flight = false
        worker.completed_scans += 1
        worker.last_scan_seconds = response.scan_seconds
        # How long the worker computed for is what decides whether a scan can
        # disturb tracking at all (a `@batch` inside `acquire!` occupies every
        # default-pool thread while it runs); enable with
        # `JULIA_DEBUG=GNSSReceiver` when a run's loops look stall-prone.
        @debug "asynchronous acquisition scan merged" scan_seconds = response.scan_seconds completed_scans =
            worker.completed_scans
        # `acquire!` returns one result per PRN *searched*, so count the ones
        # that actually passed the detector — "the scans found nothing" and "the
        # scans never ran" look identical otherwise.
        worker.detected_prns += sum(
            results -> count(res -> is_detected(res; pfa = acq_pfa), results),
            values(response.results);
            init = 0,
        )
        # Reuse the window store for the next scan rather than allocating one.
        worker.spare = response.samples
        track_state, receiver_sat_states = merge_scan(
            response,
            track_state,
            receiver_sat_states,
            systems,
            runtime,
            sampling_freq,
            num_ants,
            acq_pfa,
            code_lock_cn0_threshold,
        )
    end

    # ── Dispatch ─────────────────────────────────────────────────────────────
    periodic_due = runtime - last_time_acquisition_ran >= acquire_every
    reacquire_due = any(d -> any(should_reacquire, d), values(receiver_sat_states))
    if !(periodic_due || reacquire_due)
        return track_state,
        receiver_sat_states,
        _emptied_buffer(acquisition_buffer),
        last_time_acquisition_ran
    end

    # Keep filling the window even while a scan is in flight: the buffer must
    # hold a gap-free window ending at the current frame, and the one that will
    # be dispatched when the worker frees up should be as fresh as possible.
    acquisition_buffer = buffer(acquisition_buffer, @view(measurement[:, 1]))
    (worker.in_flight && return (
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        last_time_acquisition_ran,
    ))
    SampleBuffers.isfull(acquisition_buffer) || return (
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        last_time_acquisition_ran,
    )

    prns = _prns_to_scan(worker, receiver_sat_states, periodic_due)
    if all(isempty, values(prns))
        # Every satellite of this band is tracked and locked: nothing to search.
        # Restart the periodic timer anyway, so the window is not rebuffered on
        # every chunk until the next satellite drops.
        return track_state,
        receiver_sat_states,
        SampleBuffers.reset(acquisition_buffer),
        runtime
    end

    # The window ends with this frame, so it *starts* `current_length` samples
    # before the end of it. The acquired code phases refer to that instant; the
    # merge advances them from there to whichever frame it runs on.
    window_start_runtime =
        runtime +
        (size(measurement, 1) - acquisition_buffer.current_length) / sampling_freq
    dispatch_scan!(worker, acquisition_buffer, prns, window_start_runtime)

    # Count the attempt now rather than on the (much later) merge: without this
    # a satellite due for reacquisition would re-arm a scan on every window
    # until its result came back, defeating the back-off. A successful
    # reacquisition resets the counter when it merges.
    receiver_sat_states = _count_reacquisition_attempts(receiver_sat_states)

    track_state, receiver_sat_states, SampleBuffers.reset(acquisition_buffer), runtime
end

_emptied_buffer(acquisition_buffer) =
    acquisition_buffer.current_length == 0 ? acquisition_buffer :
    SampleBuffers.reset(acquisition_buffer)

# PRNs to search per group: on a periodic scan every satellite that is not
# tracked plus every tracked one that is out of lock (exactly the inline path's
# `missing_satellites`), otherwise only those whose reacquisition back-off has
# elapsed. `avail_prns` is the worker's own copy of each plan's PRN list, so the
# processing task never reads a plan the worker is running a search on.
function _prns_to_scan(
    worker::BandAcquisitionWorker{T,P},
    receiver_sat_states,
    periodic_due,
) where {T,P}
    P(
        map(values(receiver_sat_states), values(worker.avail_prns)) do sat_states, avail
            periodic_due ?
            vcat(
                filter(prn -> !haskey(sat_states, prn), avail),
                collect(keys(filter(state -> !is_in_lock(state), sat_states))),
            ) : collect(keys(filter(should_reacquire, sat_states)))
        end,
    )
end

# Hand the window to the worker. The copy is what makes the split safe: the
# receiver reuses (and this chunk already appended to) the buffer's store, so
# the worker cannot be given a view into it. One memcpy of the window per scan —
# a couple of milliseconds against the seconds of search it overlaps with — into
# a store recycled from the previous scan.
function dispatch_scan!(worker::BandAcquisitionWorker{T}, acquisition_buffer, prns, window_start_runtime) where {T}
    samples = SampleBuffers.get_samples(acquisition_buffer)
    store = worker.spare
    worker.spare = T[]
    resize!(store, length(samples))
    copyto!(store, samples)
    put!(worker.requests, AcquisitionRequest(store, prns, window_start_runtime))
    worker.in_flight = true
    worker.dispatched_scans += 1
    worker
end

# Merge a finished scan into the receiver state. The only thing the delay
# changes is how far the code phases have to be advanced: acquisition reports
# them at the start of its window, and `advance_code_phase` propagates them at
# the acquired code rate to the frame being processed now. Over the ~1-3 s a scan
# takes, the acquisition Doppler's own error (half a bin, ~25 Hz on a 10 ms grid)
# propagates into ~0.05 chips of code-phase error — an order of magnitude below
# the DLL's half-chip pull-in, and the same order as the acquisition code-phase
# estimate itself.
function merge_scan(
    response::AcquisitionResponse,
    track_state,
    receiver_sat_states,
    systems,
    runtime,
    sampling_freq,
    num_ants,
    acq_pfa,
    code_lock_cn0_threshold,
)
    offset = round(
        Int,
        ustrip(upreferred((runtime - response.window_start_runtime) * sampling_freq)),
    )
    track_state, dicts = _merge_all_systems(
        track_state,
        systems,
        values(receiver_sat_states),
        values(response.results),
        (offset, num_ants, acq_pfa, code_lock_cn0_threshold),
    )
    track_state, NamedTuple{keys(receiver_sat_states)}(dicts)
end

# Type-stable fold over the band's constellations, mirroring
# `_acquire_all_systems`.
@inline _merge_all_systems(track_state, ::Tuple{}, ::Tuple{}, ::Tuple{}, invariant_args) =
    (track_state, ())
@inline function _merge_all_systems(
    track_state,
    systems::Tuple,
    sat_state_dicts::Tuple,
    results::Tuple,
    invariant_args,
)
    track_state, sat_state_dict = merge_scan_results(
        track_state,
        first(sat_state_dicts),
        first(systems),
        first(results),
        invariant_args...,
    )
    track_state, rest = _merge_all_systems(
        track_state,
        Base.tail(systems),
        Base.tail(sat_state_dicts),
        Base.tail(results),
        invariant_args,
    )
    (track_state, (sat_state_dict, rest...))
end

function merge_scan_results(
    track_state,
    receiver_sat_states,
    system,
    results,
    offset,
    num_ants,
    acq_pfa,
    code_lock_cn0_threshold,
)
    isempty(results) && return track_state, receiver_sat_states
    # A satellite can regain lock while the scan is running — the search list is
    # decided when the scan is dispatched, seconds earlier. Handing such a
    # detection over would replace a converged loop with a fresh acquisition
    # seed (and, on a hardware correlator, desynchronise the host state from the
    # device replica the channel is still steering), so drop it.
    fresh = filter(res -> !_is_locked_now(receiver_sat_states, res.prn), results)
    isempty(fresh) && return track_state, receiver_sat_states
    corrected = eltype(fresh)[advance_code_phase(res, offset) for res in fresh]
    track_state, receiver_sat_states, _ = update_states_from_acquisition_results(
        corrected,
        acq_pfa,
        code_lock_cn0_threshold,
        track_state,
        receiver_sat_states,
        system,
        num_ants,
    )
    track_state, receiver_sat_states
end

_is_locked_now(receiver_sat_states, prn) =
    haskey(receiver_sat_states, prn) && is_in_lock(receiver_sat_states[prn])

# Advance the reacquisition back-off of every satellite the dispatched scan is
# a reacquisition attempt for.
_count_reacquisition_attempts(receiver_sat_states) =
    map(receiver_sat_states) do sat_states
        any(should_reacquire, sat_states) || return sat_states
        map(
            state ->
                should_reacquire(state) ? increment_num_unsuccessful_reacquisition(state) :
                state,
            sat_states,
        )
    end
