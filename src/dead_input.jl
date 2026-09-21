# Dead-input guard (issue #142).
#
# A front end that drops a DMA buffer delivers a stretch of all-zero samples. On the
# M2SDR recording the issue was found on these came in whole 1024-sample units, 5 to
# 65 ms long. Tracking integrates such a stretch like any other: an integration window
# that lies entirely inside it produces an all-zero correlator record, and Tracking's
# discriminators turn that record into `0/0` — `pll_disc` is `atan(imag(p) / real(p))`,
# `dll_disc` is `(E - L) / (E + L)`, `fll_disc` is `atan(cross / dot)`. The NaN goes
# through the loop filters into both Dopplers, and the next correlate throws
# `InexactError: Int64(NaN)` converting the code frequency to a sample shift, which
# kills the whole processing task (JuliaGNSS/Tracking.jl#234 is the same defect, hit
# from a zero replica). That fix belongs in Tracking: a record with no energy must not
# drive the loops. Until it lands the receiver cannot let such a record be produced.
#
# What the receiver can do is see the dropout coming. Samples that are exactly zero on
# every antenna are not a signal at any gain — real front-end noise never sums to an
# exact zero over a whole code period — so a run of them at least one primary code
# period long is a dead input, and any satellite whose window falls inside it would be
# lost to the NaN. Such a band's satellites are dropped *before* `track!` through the
# receiver's normal lost-lock path: their lock detectors are forced out of lock and the
# sats are removed from tracking, so they are reacquired with the usual back-off once
# the signal returns, and the rest of the receiver — other bands, PVT, the output stream
# — keeps running. A dropout shorter than one code period cannot zero a whole window
# and is left to the tracking loops, which ride it out like any fade.
#
# The run length is carried across chunks (`ReceiverState.trailing_dead_samples`): a
# dropout that ends a chunk and continues into the next one is as dead as one inside a
# single chunk, and the windows do not line up with the chunk grid.

# A sample is dead when every antenna channel of it reads exactly zero. `Complex{Int16}`
# and the float element types alike.
@inline _dead_sample(m::AbstractVector, i) = @inbounds iszero(m[i])
@inline function _dead_sample(m::AbstractMatrix, i)
    @inbounds for j in axes(m, 2)
        iszero(m[i, j]) || return false
    end
    true
end

# Length of the run of dead samples that ends the chunk.
function _trailing_dead_samples(m)
    n = size(m, 1)
    trailing = 0
    while trailing < n && _dead_sample(m, n - trailing)
        trailing += 1
    end
    trailing
end

"""
    dead_run(m, carried, window) -> (dead::Bool, trailing::Int)

Whether the samples `m` (rows), preceded by `carried` dead samples at the end of the
previous chunk, contain a run of at least `window` consecutive dead samples — one that
could swallow a whole integration window — and the length of the dead run the chunk ends
with, to carry into the next call.

Cheap on live signal: the leading and trailing runs stop at the first live sample, and
an internal run of `window` samples or more must contain a sample at an index that is a
multiple of `window`, so only those are probed and a full run is measured only around a
probe that reads zero.
"""
function dead_run(m, carried::Int, window::Int)
    n = size(m, 1)
    leading = 0
    while leading < n && _dead_sample(m, leading + 1)
        leading += 1
    end
    # A chunk dead from end to end continues the previous chunk's run.
    leading == n && return (carried + n >= window, carried + n)
    trailing = _trailing_dead_samples(m)
    (carried + leading >= window || trailing >= window) && return (true, trailing)
    i = window
    while i <= n
        if _dead_sample(m, i)
            lo = i
            while lo > 1 && _dead_sample(m, lo - 1)
                lo -= 1
            end
            hi = i
            while hi < n && _dead_sample(m, hi + 1)
                hi += 1
            end
            hi - lo + 1 >= window && return (true, trailing)
            # Continue past this run; the next probe is the first multiple of `window`
            # beyond it.
            i = (hi ÷ window + 1) * window
        else
            i += window
        end
    end
    (false, trailing)
end

# The shortest integration window a satellite on this band can complete, in samples:
# one primary code period of the shortest-period tracked signal. This receiver never
# lengthens integrations past one primary code block (see `CodeLockDetector`), so a
# dead run shorter than this cannot produce an all-zero record. Rounded down, so a
# window shortened by a fraction of a sample through the code Doppler still counts.
function dead_input_window(systems, sampling_freq)
    minimum(systems) do system
        minimum(tracking_signals(system)) do signal
            floor(Int, upreferred(primary_code_period(signal) * sampling_freq))
        end
    end
end

# The satellite's lock detectors forced out of lock (the same primitive the vector
# loop uses to release a member for cause) and its vector-loop membership cleared: the
# sat is removed from tracking in the same step, so `update_all_receiver_sat_states`
# must count it as lost rather than look its tracking state up.
_lose_lock(state::ReceiverSatState) = ReceiverSatState(
    state.prn,
    state.decoder,
    set_out_of_lock(state.code_lock_detector),
    set_out_of_lock(state.carrier_lock_detector),
    state.time_in_lock,
    state.time_out_of_lock,
    state.num_unsuccessful_reacquisition,
    false,
)

# Drop every satellite tracked on the dead band: out of the vector loop if it was in
# one, out of lock, and out of the tracking state. Returns the updated
# `(track_state, receiver_sat_states)`. A band with nothing tracked is a no-op and
# stays silent — there is nothing to protect, and acquisition on zero samples simply
# finds nothing.
function drop_dead_band_satellites(
    track_state,
    receiver_sat_states,
    systems,
    vector_tracking::Bool,
    band_key,
    runtime,
    window,
)
    dropped = Pair{Symbol,Vector{Int}}[]
    for system in systems
        group_key = signal_group_key(system)
        prns = collect(keys(get_sat_states(track_state, group_key)))
        isempty(prns) && continue
        push!(dropped, group_key => prns)
        vector_tracking && release_from_vector_tracking!(track_state, group_key, prns)
        for prn in prns
            track_state = remove_satellite(track_state; prn, group = group_key)
        end
        group_states = map(
            state -> state.prn in prns ? _lose_lock(state) : state,
            receiver_sat_states[group_key],
        )
        receiver_sat_states =
            merge(receiver_sat_states, NamedTuple{(group_key,)}((group_states,)))
    end
    isempty(dropped) || _warn_dead_input(band_key, runtime, window, dropped)
    track_state, receiver_sat_states
end

@noinline function _warn_dead_input(band_key, runtime, window, dropped)
    @warn """
          Dead input on band `:$band_key`: the samples have read exactly zero on every \
          antenna for at least one primary code period ($window samples) — a front-end \
          or DMA dropout. Dropped its tracked satellites from tracking; they are \
          reacquired once the signal returns. Tracking.jl cannot coast through this: an \
          all-zero correlator record makes its discriminators compute 0/0, and the NaN \
          Doppler kills the processing task on the next correlate \
          (JuliaGNSS/Tracking.jl#234, GNSSReceiver.jl#142).""" runtime dropped
end

# Per-band recursion, threading the shared `track_state` and `receiver_sat_states` like
# `_acquire_all_bands` does, and collecting each band's new trailing run length.
@inline _guard_dead_bands(
    track_state,
    receiver_sat_states,
    ::Tuple{},
    ::Tuple{},
    ::Tuple{},
    ::Tuple{},
    _,
    ::Bool,
    _,
) = (track_state, receiver_sat_states, ())
@inline function _guard_dead_bands(
    track_state,
    receiver_sat_states,
    band_keys::Tuple,
    band_systems::Tuple,
    measurements::Tuple,
    carried::Tuple,
    sampling_freq,
    vector_tracking::Bool,
    runtime,
)
    band_key = first(band_keys)
    systems = first(band_systems)
    window = dead_input_window(systems, sampling_freq)
    dead, trailing = dead_run(first(measurements), first(carried), window)
    if dead
        track_state, receiver_sat_states = drop_dead_band_satellites(
            track_state,
            receiver_sat_states,
            systems,
            vector_tracking,
            band_key,
            runtime,
            window,
        )
    end
    track_state, receiver_sat_states, rest = _guard_dead_bands(
        track_state,
        receiver_sat_states,
        Base.tail(band_keys),
        Base.tail(band_systems),
        Base.tail(measurements),
        Base.tail(carried),
        sampling_freq,
        vector_tracking,
        runtime,
    )
    (track_state, receiver_sat_states, (trailing, rest...))
end

# Run the guard over every band before `track!`. Returns the updated `track_state` and
# `receiver_sat_states` plus the per-band trailing dead-run lengths to store for the
# next chunk.
function guard_dead_input(
    track_state,
    receiver_sat_states,
    trailing_dead_samples::NamedTuple,
    band_keys,
    band_systems,
    measurements,
    sampling_freq,
    vector_tracking::Bool,
    runtime,
)
    track_state, receiver_sat_states, trailing = _guard_dead_bands(
        track_state,
        receiver_sat_states,
        band_keys,
        band_systems,
        measurements,
        values(trailing_dead_samples),
        sampling_freq,
        vector_tracking,
        runtime,
    )
    track_state, receiver_sat_states, NamedTuple{band_keys}(trailing)
end
