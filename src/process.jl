# Acquire / reacquire the constellations of ONE band from its own sample frame,
# threading the receiver-wide `TrackState`. Only acquisition is per-band (each
# band buffers its own RF stream); tracking, decoding and PVT run once across all
# bands in `process`. `receiver_sat_states`, `acquisition_buffer` and
# `last_time_acquisition_ran` are this band's slices. Returns the threaded
# `track_state` plus this band's updated satellite-state dicts, buffer and timer.
function acquire_band(
    track_state,
    receiver_sat_states,
    acquisition_buffer,
    systems,
    acq_plans,
    measurement,
    last_time_acquisition_ran,
    interm_freq,
    runtime,
    num_ants::NumAnts,
    acquire_every,
    acq_pfa,
    code_lock_cn0_threshold,
    subsample_interpolation,
)
    num_samples = size(measurement, 1)

    # Only fill the acquisition buffer when acquisition could plausibly fire —
    # the periodic interval has elapsed, or some satellite qualifies for
    # reacquisition. This avoids a memcpy on every steady-state frame. A frame
    # that skips buffering empties the buffer instead (an O(1) length reset):
    # whatever it held is no longer contiguous with the next pushed frame, and
    # both `acquire!`'s coherent integration and `correct_code_phases` require
    # the buffered window to be gap-free and to end at the current frame.
    needs_buffering =
        runtime - last_time_acquisition_ran >= acquire_every ||
        any(sat_state_dict -> any(should_reacquire, sat_state_dict), values(receiver_sat_states))
    acquisition_buffer = if needs_buffering
        buffer(acquisition_buffer, @view(measurement[:, 1]))
    else
        SampleBuffers.reset(acquisition_buffer)
    end

    run_periodic_acquisition =
        SampleBuffers.isfull(acquisition_buffer) &&
        runtime - last_time_acquisition_ran >= acquire_every

    # Acquire / reacquire per constellation, threading the shared TrackState.
    # A type-stable fold over the systems tuple keeps `receiver_sat_states`
    # concretely typed; a plain loop collecting into a Vector{Any} would box the
    # per-system dicts and make the whole per-frame path type-unstable.
    # `receiver_sat_states` and `acq_plans` are this band's per-group slices, already
    # ordered to match `systems` by the caller (`_acquire_all_bands` selects them by
    # `map(signal_group_key, systems)`), so their `values` zip positionally with
    # `systems` inside `_acquire_all_systems`.
    track_state, new_sat_state_dicts = _acquire_all_systems(
        track_state,
        systems,
        values(receiver_sat_states),
        values(acq_plans),
        (
            acquisition_buffer,
            interm_freq,
            acq_pfa,
            code_lock_cn0_threshold,
            num_ants,
            num_samples,
            run_periodic_acquisition,
            subsample_interpolation,
        ),
    )
    receiver_sat_states = NamedTuple{keys(receiver_sat_states)}(new_sat_state_dicts)
    if run_periodic_acquisition
        last_time_acquisition_ran = runtime
        # Reset buffer after periodic acquisition so stale samples aren't reused.
        acquisition_buffer = SampleBuffers.reset(acquisition_buffer)
    end

    track_state, receiver_sat_states, acquisition_buffer, last_time_acquisition_ran
end

# Type-stable recursion over the aligned per-band tuples: acquire each band in
# turn, threading the shared `track_state` and merging each band's updated
# satellite-state dicts, acquisition buffer and acquisition timer back into the
# receiver-wide NamedTuples. `invariant_acq_args` bundles the invariant (band-independent)
# acquisition arguments. `acq_plans` is the receiver-wide plan NamedTuple; each
# band selects its own entries by group key.
@inline _acquire_all_bands(
    track_state,
    receiver_sat_states,
    acquisition_buffers,
    last_time_acquisition_ran,
    ::Tuple{},
    ::Tuple{},
    ::Tuple{},
    ::Tuple{},
    acq_plans,
    invariant_acq_args,
) = (track_state, receiver_sat_states, acquisition_buffers, last_time_acquisition_ran)
@inline function _acquire_all_bands(
    track_state,
    receiver_sat_states,
    acquisition_buffers,
    last_time_acquisition_ran,
    band_keys::Tuple,
    band_systems::Tuple,
    measurements::Tuple,
    interm_freqs::Tuple,
    acq_plans,
    invariant_acq_args,
)
    band_key = first(band_keys)
    systems = first(band_systems)
    group_keys = map(signal_group_key, systems)
    track_state, band_receiver_sat_states, band_acquisition_buffer, band_last_time_acquisition_ran = acquire_band(
        track_state,
        receiver_sat_states[group_keys],
        acquisition_buffers[band_key],
        systems,
        acq_plans[group_keys],
        first(measurements),
        last_time_acquisition_ran[band_key],
        first(interm_freqs),
        invariant_acq_args...,
    )
    receiver_sat_states = merge(receiver_sat_states, band_receiver_sat_states)
    acquisition_buffers = merge(acquisition_buffers, NamedTuple{(band_key,)}((band_acquisition_buffer,)))
    last_time_acquisition_ran =
        merge(last_time_acquisition_ran, NamedTuple{(band_key,)}((band_last_time_acquisition_ran,)))
    _acquire_all_bands(
        track_state,
        receiver_sat_states,
        acquisition_buffers,
        last_time_acquisition_ran,
        Base.tail(band_keys),
        Base.tail(band_systems),
        Base.tail(measurements),
        Base.tail(interm_freqs),
        acq_plans,
        invariant_acq_args,
    )
end

"""
    process(receiver_state, acq_plans, measurements, band_systems, sampling_freq,
            interm_freqs = map(_ -> 0.0u"Hz", band_systems); kwargs...)

Advance the receiver by one measurement chunk per band and return the next
`ReceiverState`.

A single chunk runs the whole per-cycle pipeline: it (re)acquires satellites per band via
`acq_plans` at most every `acquire_every` (buffering samples only when acquisition could
fire), tracks every band's satellites from one `TrackState`, updates their lock detectors
and, once enough have been locked for `time_in_lock_before_calculating_pvt`, recomputes
the fused multi-GNSS PVT solution every `pvt_update_interval`. Satellites that drop out of
lock are removed and reacquired with a bounded quadratic back-off. `measurements`,
`band_systems` and `interm_freqs` are tuples aligned band-by-band (`interm_freqs` defaults
to `0 Hz` for every band); `acq_plans` is one
`NamedTuple` keyed by group key across all bands. This is the function [`receive`](@ref)
calls for each chunk; see it for the meaning of the remaining keyword arguments.
"""
function process(
    receiver_state::ReceiverState,
    acq_plans,
    measurements::Tuple,
    band_systems::Tuple,
    sampling_freq,
    interm_freqs::Tuple = map(_ -> 0.0u"Hz", band_systems);
    downconvert_and_correlator = CPUThreadedDownconvertAndCorrelator(),
    num_ants::NumAnts{N} = NumAnts(1),
    acquire_every = 10u"s",
    acq_pfa = DEFAULT_ACQ_PFA,
    code_lock_cn0_threshold = nothing, # `nothing` ⇒ per-system `get_default_code_lock_cn0_threshold`
    time_in_lock_before_calculating_pvt = 2u"s",
    pvt_update_interval = 100u"ms",
    subsample_interpolation = true,
    enable_ionospheric_correction = true,
    enable_tropospheric_correction = true,
    pvt_approximate_year::Integer = year(now(UTC)),
) where {N}
    # A single-antenna band delivers an N×1 frame; `Tracking` wants a plain vector
    # for one antenna and the matrix (rows = samples) for an array.
    meas = map(m -> N == 1 ? vec(m) : m, measurements)
    # `band_keys` are exactly the acquisition-buffer NamedTuple's keys (both derive from
    # `get_band_id(system_band(first(band)))`), so read them off the type instead of
    # recomputing them from `band_systems` every chunk.
    band_keys = keys(receiver_state.acquisition_buffers)
    all_systems = _flatten_systems(band_systems)

    runtime = receiver_state.runtime
    # All bands advance from equal-length frames of the same time base, so they
    # share one runtime and signal duration.
    signal_duration = size(first(meas), 1) / sampling_freq

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
    track_state, receiver_sat_states, acquisition_buffers, last_time_acquisition_ran =
        if acq_due || reacq_due
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
                (
                    runtime,
                    num_ants,
                    acquire_every,
                    acq_pfa,
                    code_lock_cn0_threshold,
                    subsample_interpolation,
                ),
            )
        else
            buffers =
                all(b -> b.current_length == 0, values(receiver_state.acquisition_buffers)) ?
                receiver_state.acquisition_buffers :
                map(SampleBuffers.reset, receiver_state.acquisition_buffers)
            (
                receiver_state.track_state,
                receiver_state.receiver_sat_states,
                buffers,
                receiver_state.last_time_acquisition_ran,
            )
        end

    # Single multi-band tracking pass: one `BandMeasurement` per band, keyed by
    # band, fed to one `track!` call over the shared multi-band `TrackState`.
    band_measurements = NamedTuple{band_keys}(
        map(
            (m, interm_freq) -> Tracking.BandMeasurement(m, sampling_freq, interm_freq),
            meas,
            interm_freqs,
        ),
    )
    # In-place `track!` rather than the immutable `track`: the latter detaches
    # (copies) each group's satellite slot vectors and rebuilds the TrackedSat
    # wrappers every call. The receiver discards the previous `ReceiverState` each
    # chunk and reuses one hoisted correlator, so mutating in place is safe and is
    # Tracking v3's documented allocation-free real-time pattern.
    track_state = track!(band_measurements, track_state; downconvert_and_correlator)

    receiver_sat_states = update_all_receiver_sat_states(
        receiver_sat_states,
        track_state,
        all_systems,
        signal_duration,
    )

    track_state = remove_lost_satellites(receiver_sat_states, track_state)

    pvt, last_time_pvt_ran = update_pvt(
        all_systems,
        receiver_sat_states,
        track_state,
        receiver_state.pvt,
        receiver_state.pvt_sat_state_buffer,
        runtime,
        time_in_lock_before_calculating_pvt,
        receiver_state.last_time_pvt_ran,
        pvt_update_interval;
        enable_ionospheric_correction,
        enable_tropospheric_correction,
        pvt_approximate_year,
    )

    ReceiverState(
        track_state,
        receiver_sat_states,
        acquisition_buffers,
        last_time_acquisition_ran,
        pvt,
        receiver_state.pvt_sat_state_buffer,
        runtime + signal_duration,
        last_time_pvt_ran,
    )
end

function remove_lost_satellites(receiver_sat_states, track_state)
    for (group_key, group_sat_states) in pairs(receiver_sat_states)
        isempty(group_sat_states) && continue
        tracked_prns = keys(get_sat_states(track_state, group_key))
        for receiver_sat_state in group_sat_states
            if !is_in_lock(receiver_sat_state) && receiver_sat_state.prn in tracked_prns
                track_state = remove_satellite(track_state; prn = receiver_sat_state.prn, group = group_key)
            end
        end
    end
    track_state
end

# Append the PVT-ready satellites of the given `systems` (every constellation
# across all bands) to `states`. A satellite is ready once it is in lock and has
# been in lock long enough to have decoded usable data. Called by the combined
# multi-band PVT solve (`update_pvt`).
function collect_pvt_sat_states!(
    states,
    systems,
    receiver_sat_states,
    track_state,
    time_in_lock_before_calculating_pvt,
)
    for system in systems
        group_key = signal_group_key(system)
        for receiver_sat_state in receiver_sat_states[group_key]
            if is_in_lock(receiver_sat_state) && receiver_sat_state.time_in_lock > time_in_lock_before_calculating_pvt
                # Hand PVT the *ranging* signal (the pilot, for a combined spec) as
                # `system` and the *data* decoder separately: PVT derives the code /
                # carrier terms and the group-delay ISC from the ranging signal (its
                # `correct_by_group_delay` dispatches on e.g. `GPSL5Q`/`GPSL1C_P`)
                # and the TOW / bit count from the decoder's data component. The
                # combined `TrackedSat` carries the shared, pilot-driven code phase.
                sat_state = SatelliteState(
                    receiver_sat_state.decoder,
                    ranging_signal(system),
                    get_sat_state(track_state, group_key, receiver_sat_state.prn),
                )
                push!(states, sat_state)
            end
        end
    end
    states
end

# `calc_pvt` with the optional-year keyword spliced in only when supplied.
function _calc_pvt(
    pvt_satellite_states,
    pvt;
    enable_ionospheric_correction,
    enable_tropospheric_correction,
    pvt_approximate_year,
)
    length(pvt_satellite_states) >= 4 || return pvt
    calc_pvt(
        pvt_satellite_states,
        pvt;
        enable_ionospheric_correction,
        enable_tropospheric_correction,
        approximate_year = pvt_approximate_year,
    )
end

# Combined multi-band PVT over the single receiver state: pool every band's
# PVT-ready satellites into one `calc_pvt`. `all_systems` is the flat tuple of
# specs across all bands; `receiver_sat_states` and `track_state` are the
# receiver-wide, band-spanning states. The pooled vector mixes constellations and
# frequency bands, which is exactly what `calc_pvt` resolves (a clock column per
# GNSS time system and an inter-frequency-bias column per extra band).
function update_pvt(
    all_systems,
    receiver_sat_states,
    track_state,
    pvt,
    pvt_sat_state_buffer,
    runtime,
    time_in_lock_before_calculating_pvt,
    last_time_pvt_ran,
    pvt_update_interval;
    enable_ionospheric_correction = true,
    enable_tropospheric_correction = true,
    pvt_approximate_year::Integer = year(now(UTC)),
)
    runtime - last_time_pvt_ran >= pvt_update_interval || return pvt, last_time_pvt_ran

    # Reuse the buffer across PVT cycles: `collect_pvt_sat_states!` empties and refills it,
    # avoiding a fresh `Vector{SatelliteState}` allocation every cycle.
    empty!(pvt_sat_state_buffer)
    collect_pvt_sat_states!(
        pvt_sat_state_buffer,
        all_systems,
        receiver_sat_states,
        track_state,
        time_in_lock_before_calculating_pvt,
    )

    pvt = _calc_pvt(
        pvt_sat_state_buffer,
        pvt;
        enable_ionospheric_correction,
        enable_tropospheric_correction,
        pvt_approximate_year,
    )
    return pvt, runtime
end

function update_all_receiver_sat_states(receiver_sat_states, track_state, systems, signal_duration)
    group_keys = keys(receiver_sat_states)
    # Map over `systems` (aligned with `group_keys`) so each group carries its own
    # ranging/data signal selectors: CN0 and carrier lock are read from the ranging
    # signal, the navigation bits decoded from the data signal.
    #
    # The inner in-place `map!` reuses each group's dictionary storage instead of
    # allocating a fresh one every frame (the keys are unchanged; only the
    # `ReceiverSatState` values update). Safe because the emitted `sat_data` copies the
    # values it needs, so no consumer holds a reference into this dictionary.
    new_vals = map(systems) do system
        group_key = signal_group_key(system)
        data_idx = data_signal_index(system)
        group_states = receiver_sat_states[group_key]
        map!(group_states, group_states) do receiver_sat_state
            if is_in_lock(receiver_sat_state)
                prn = receiver_sat_state.prn
                ReceiverSatState(
                    prn,
                    decode(
                        receiver_sat_state.decoder,
                        get_soft_bits(track_state, group_key, prn, data_idx),
                        get_num_bits(track_state, group_key, prn, data_idx),
                    ),
                    update(
                        receiver_sat_state.code_lock_detector,
                        estimate_cn0(track_state, group_key, prn, RANGING_SIGNAL_INDEX),
                        signal_duration,
                    ),
                    update(
                        receiver_sat_state.carrier_lock_detector,
                        get_last_fully_integrated_filtered_prompt(
                            track_state,
                            group_key,
                            prn,
                            RANGING_SIGNAL_INDEX,
                        ),
                        signal_duration,
                    ),
                    receiver_sat_state.time_in_lock + signal_duration,
                    0.0s,
                    0,
                )
            else
                increase_time_out_of_lock(receiver_sat_state, signal_duration)
            end
        end
    end
    NamedTuple{group_keys}(new_vals)
end

# Build a `TrackedSat` from an acquisition result, tracking `signals` (the
# spec's `tracking_signals`: the ranging/driver signal first, the data signal
# last — a one-tuple for a data-only spec). Acquisition ran on the pilot or the
# data component (see `acquisition_signal`); either shares the spec's primary-code
# epoch, so its code phase seeds every replica. Construction goes through
# `create_tracked_sat` — the same constructor that pinned the `TrackState`'s
# satellite-slot type in `ReceiverState` — so `merge_sats` accepts the sat.
tracked_sat_from_acq(
    acq::Acquisition.AcquisitionResults,
    signals::Tuple{AbstractGNSSSignal,Vararg{AbstractGNSSSignal}},
    num_ants::NumAnts,
    doppler_estimator,
) = create_tracked_sat(
    signals,
    acq.prn,
    acq.code_phase,
    acq.carrier_doppler,
    num_ants,
    doppler_estimator,
)

# Returns `(track_state, receiver_sat_states, acquired_prns)`. `acquired_prns`
# lists the PRNs that passed the CFAR detector and were merged; the reacquisition
# path uses it to distinguish a real re-lock from a detection that was rejected.
function update_states_from_acquisition_results(
    acquisition_results,
    acq_pfa,
    code_lock_cn0_threshold,
    track_state,
    receiver_sat_states,
    system,
    num_ants,
)
    group_key = signal_group_key(system)

    # Fall back to the per-signal default code-lock threshold when the caller left
    # it unset; it comes from the ranging signal (the one the detector runs on).
    code_lock_threshold = something(
        code_lock_cn0_threshold,
        get_default_code_lock_cn0_threshold(ranging_signal(system)),
    )

    # CFAR alone decides detection: a result is accepted when its peak-to-noise ratio
    # clears the CFAR threshold for the configured false-alarm probability.
    acq_res_valids = filter(res -> is_detected(res; pfa = acq_pfa), acquisition_results)
    acquired_prns = map(res -> res.prn, acq_res_valids)
    isempty(acq_res_valids) && return track_state, receiver_sat_states, acquired_prns

    # Build the decoder from the *data* signal, never from `res.system`: with
    # pilot acquisition `res.system` is the dataless pilot, which has no decoder.
    # A re-acquired PRN keeps its existing (data) decoder state.
    data_sys = data_signal(system)
    new_receiver_sat_states = map(acq_res_valids) do res
        decoder =
            res.prn in keys(receiver_sat_states) ?
            reset_decoder_state(receiver_sat_states[res.prn].decoder) :
            GNSSDecoderState(data_sys, res.prn)
        ReceiverSatState(res, decoder, code_lock_threshold)
    end

    tracking_sigs = tracking_signals(system)
    new_sat_states = [
        tracked_sat_from_acq(res, tracking_sigs, num_ants, track_state.doppler_estimator) for res in acq_res_valids
    ]

    new_track_state = merge_sats(track_state, group_key, new_sat_states)

    new_receiver_sat_states_dictionary = merge(
        receiver_sat_states,
        Dictionary(acquired_prns, new_receiver_sat_states),
    )
    new_track_state, new_receiver_sat_states_dictionary, acquired_prns
end

# Type-stable fold over the constellations: threads the shared TrackState and
# collects each system's updated ReceiverSatState dict into a concrete tuple.
# `invariant_acq_args` bundles the invariant per-frame arguments (splatted into
# `acquire_satellites`). Recursion over the concrete `systems`/`sat_state_dicts`/`acq_plans`
# tuples unrolls at compile time, so no `Any` boxing.
@inline _acquire_all_systems(track_state, ::Tuple{}, ::Tuple{}, ::Tuple{}, invariant_acq_args) =
    (track_state, ())
@inline function _acquire_all_systems(
    track_state,
    systems::Tuple,
    sat_state_dicts::Tuple,
    acq_plans::Tuple,
    invariant_acq_args,
)
    track_state, sat_state_dict =
        acquire_satellites(track_state, first(sat_state_dicts), first(systems), first(acq_plans), invariant_acq_args...)
    track_state, rest =
        _acquire_all_systems(track_state, Base.tail(systems), Base.tail(sat_state_dicts), Base.tail(acq_plans), invariant_acq_args)
    (track_state, (sat_state_dict, rest...))
end

function acquire_satellites(
    track_state,
    receiver_sat_states,
    system,
    acq_plan,
    acquisition_buffer,
    interm_freq,
    acq_pfa,
    code_lock_cn0_threshold,
    num_ants,
    num_samples,
    run_periodic_acquisition,
    subsample_interpolation,
)
    track_state, receiver_sat_states = try_to_reacquire_lost_satellites(
        track_state,
        receiver_sat_states,
        system,
        acq_plan,
        acquisition_buffer,
        interm_freq,
        acq_pfa,
        code_lock_cn0_threshold,
        num_ants,
        num_samples,
        subsample_interpolation,
    )

    if run_periodic_acquisition
        missing_satellites = vcat(
            filter(
                prn -> !(prn in keys(receiver_sat_states)),
                collect(acq_plan.avail_prns),
            ),
            collect(keys(filter(state -> !is_in_lock(state), receiver_sat_states))),
        )
        if !isempty(missing_satellites)
            track_state, receiver_sat_states, _ = acquire_and_update_states(
                track_state,
                receiver_sat_states,
                system,
                acq_plan,
                acquisition_buffer,
                missing_satellites,
                interm_freq,
                acq_pfa,
                code_lock_cn0_threshold,
                num_ants,
                num_samples,
                subsample_interpolation,
            )
        end
    end
    track_state, receiver_sat_states
end

# One acquisition pass over `prns` on the buffered window, merged back into the
# receiver: acquire, advance the detected code phases to the current frame, then
# update the tracking and satellite states. Shared by the periodic scan and fast
# reacquisition, which differ only in how they select `prns` and what they do
# with the returned `acquired_prns`.
function acquire_and_update_states(
    track_state,
    receiver_sat_states,
    system,
    acq_plan,
    acquisition_buffer,
    prns,
    interm_freq,
    acq_pfa,
    code_lock_cn0_threshold,
    num_ants,
    num_samples,
    subsample_interpolation,
)
    acq_res = acquire!(
        acq_plan,
        SampleBuffers.get_samples(acquisition_buffer),
        prns;
        interm_freq,
        subsample_interpolation,
    )

    corrected_acq_res = correct_code_phases(acq_res, acquisition_buffer, num_samples)

    update_states_from_acquisition_results(
        corrected_acq_res,
        acq_pfa,
        code_lock_cn0_threshold,
        track_state,
        receiver_sat_states,
        system,
        num_ants,
    )
end

# Advance the acquired code phase from the start of the acquisition buffer to the
# start of the current frame, where `track` will pick these satellites up. The
# offset is purely the buffer geometry — the number of buffered samples ahead of
# the current frame — computed as `current_length - num_samples` (the current
# frame is the most recent `num_samples` samples in the buffer). Deriving it from
# a running processed-sample counter instead would drift: with the
# `needs_buffering` optimisation, frames skipped from the buffer make the buffer's
# sample counter fall behind the processed count, over-estimating the gap.
function correct_code_phases(acq_res, acquisition_buffer, num_samples)
    offset = acquisition_buffer.current_length - num_samples
    # Collect into the concrete `eltype(acq_res)`, not the abstract UnionAll
    # `Acquisition.AcquisitionResults` (it is parametric, `{S,T,D}`): `advance_code_phase`
    # preserves the type via `@set`, so this keeps the vector concretely typed and avoids
    # boxing/type-instability.
    eltype(acq_res)[advance_code_phase(res, offset) for res in acq_res]
end

function advance_code_phase(acq_res::Acquisition.AcquisitionResults, num_samples)
    advanced_code_phase = mod(
        (
            get_code_frequency(acq_res.system) +
            acq_res.carrier_doppler * get_code_center_frequency_ratio(acq_res.system)
        ) * num_samples / acq_res.sampling_frequency + acq_res.code_phase,
        get_code_length(acq_res.system),
    )
    # AcquisitionResults gained fields in Acquisition v2; copy all of them and only
    # override the advanced code phase instead of reconstructing positionally.
    @set acq_res.code_phase = advanced_code_phase
end

# Fast per-satellite reacquisition with a *bounded* attempt rate (quadratic
# back-off). `time_out_of_lock` grows every frame a sat is unlocked;
# `num_unsuccessful_reacquisition` (`n`) grows by one per failed fast attempt
# (see `try_to_reacquire_lost_satellites`). Fire the (n+1)-th attempt only once
# `time_out_of_lock` has reached the n-th back-off step, so attempts land at
# ~0.2, 0.8, 1.8, 3.2 s … out of lock — never every frame. The bound matters: an
# unthrottled full-grid `acquire!` per frame is prohibitively expensive at high
# sampling rates on churny urban signal. The optimistic lock detector bounds the
# rate further — a re-detected sat is `is_in_lock` for a ~20-update grace, so
# `time_out_of_lock` stays 0 and no new attempt fires while it is converging.
# `reacquire_backoff` sets the base step; `max_reacquire_attempts` caps total
# attempts before falling back to the periodic full scan (`acquire_every`).
function should_reacquire(state; reacquire_backoff = 200ms, max_reacquire_attempts = 5)
    n = state.num_unsuccessful_reacquisition
    !is_in_lock(state) &&
        n < max_reacquire_attempts &&
        state.time_out_of_lock >= (n + 1)^2 * reacquire_backoff
end

function try_to_reacquire_lost_satellites(
    track_state,
    receiver_sat_states,
    system,
    acq_plan,
    acquisition_buffer,
    interm_freq,
    acq_pfa,
    code_lock_cn0_threshold,
    num_ants,
    num_samples,
    subsample_interpolation,
)
    SampleBuffers.isfull(acquisition_buffer) || return track_state, receiver_sat_states

    any(should_reacquire, receiver_sat_states) || return track_state, receiver_sat_states

    # Only now allocate the filtered Dictionary (preserves type stability)
    out_of_lock_receiver_sat_states = filter(should_reacquire, receiver_sat_states)
    prns = collect(keys(out_of_lock_receiver_sat_states))

    # The FM-DBZP algorithm searches the full Doppler grid in a single pass, so
    # reacquisition just re-runs the main plan on the lost PRNs.
    track_state, receiver_sat_states, acquired_prns = acquire_and_update_states(
        track_state,
        receiver_sat_states,
        system,
        acq_plan,
        acquisition_buffer,
        prns,
        interm_freq,
        acq_pfa,
        code_lock_cn0_threshold,
        num_ants,
        num_samples,
        subsample_interpolation,
    )

    # Any attempted PRN that was not actually re-acquired counts as an
    # unsuccessful reacquisition, advancing its back-off.
    unsuccessful_prns = filter(prn -> !(prn in acquired_prns), prns)
    if !isempty(unsuccessful_prns)
        receiver_sat_states = map(receiver_sat_states) do state
            state.prn in unsuccessful_prns ?
            increment_num_unsuccessful_reacquisition(state) : state
        end
    end

    return track_state, receiver_sat_states
end
