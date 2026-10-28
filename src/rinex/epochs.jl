# Observation epochs: when one is due on the epoch grid, and the observables it carries.

"""
    ObsEpochTiming

When one observation epoch happens, in the time system of the file (`RinexLogger`'s
`time_system`).

  - `week`, `time_of_week`: the steered (nominal) epoch the record is stamped with, the week
    on the time system's own count (GPS, GST or BDT week), as RINEXParser's `epoch_time`
    takes it.
  - `receive_time`: the time of week of the measurement instant, the solution's own epoch
    carried on to it. Pseudoranges are differences against this, so they
    never touch an absolute seconds count.
  - `propagation`: how far the observables have to be carried, with their own Doppler, to
    land on the steered epoch — back over less than one chunk.
"""
struct ObsEpochTiming
    week::Int
    time_of_week::Float64
    receive_time::Float64
    propagation::Float64
end


# The receiver time [s, on the run's sample grid] the solution's epoch was measured at: the
# end of the chunk for the scalar PVT, which solves on the chunk it is handed, and the
# navigation cycle's epoch for the vector loop, whose solution — the seed fix included — is
# dated on that grid.
solution_runtime(::AbstractDopplerEstimator, runtime) = runtime
function solution_runtime(estimator::VectorPLLAndDLL, runtime)
    epoch = navigation_epoch(estimator)
    isnothing(epoch) ? runtime : ustrip(s, epoch)
end

# The timing of this chunk's observation epoch, or `nothing` when it is not due to produce
# one (see [`RinexLogger`](@ref)).
function obs_epoch_timing(logger::RinexLogger, receiver_state)
    isnothing(logger.config.obs_file) && return nothing
    steering = logger.steering
    runtime = ustrip(s, receiver_state.runtime)
    previous_runtime = steering.previous_runtime
    previous_slot = steering.previous_slot
    steering.previous_runtime = runtime
    steering.previous_slot = nothing
    interval = ustrip(s, logger.config.interval)
    isfinite(previous_runtime) &&
        runtime - previous_runtime > interval &&
        warn_interval_below_chunk(logger)
    pvt = receiver_state.pvt
    isnothing(pvt.time) && return nothing
    isempty(pvt.satellites) && return nothing
    # The solution's epoch is the receive time it solved for, clock bias applied, where the
    # solver measured (see `solution_runtime`).
    if pvt.time != steering.last_pvt_time
        steering.last_pvt_time = pvt.time
        steering.solution_runtime =
            solution_runtime(receiver_state.track_state.doppler_estimator, runtime)
    end
    # The measurement instant: the end of this chunk, where every observable below is read.
    # The solution's epoch is carried there over the span of receiver time in between, which
    # the receiver clock's drift scales onto system time. A solution that stopped coming
    # (too few satellites) is not carried on for long.
    span = runtime - steering.solution_runtime
    span > steering.max_solution_age && return nothing
    # The epoch's week and time of week on the file's time system, the whole weeks taken
    # off in integer seconds (`get_time_of_week`): the absolute count, around 1.2e9 s, would
    # leave a `Float64` only 0.2 µs, 70 m of pseudorange.
    week = get_week(logger.time_system, pvt.time)
    receive_time =
        get_time_of_week(logger.time_system, pvt.time) +
        span * (1 - relative_clock_drift(pvt))
    if receive_time >= SECONDS_PER_WEEK
        week += 1
        receive_time -= SECONDS_PER_WEEK
    end

    # The grid point at or before the measurement instant. It is due on the chunk that
    # crossed it, so the propagation back onto it is shorter than the chunk; without the
    # previous chunk's slot (the first chunk with a solution) only if it lies within this
    # chunk's span.
    index = floor(Int, receive_time / interval)
    time_of_week = index * interval
    slot = (week, index)
    steering.previous_slot = slot
    if isnothing(previous_slot)
        receive_time - time_of_week < runtime - previous_runtime || return nothing
    else
        previous_slot == slot && return nothing
    end
    # A solution update can step the receive time back across a grid point that was already
    # written; it is not written twice.
    !isnothing(steering.last_written) && slot <= steering.last_written && return nothing
    steering.last_written = slot

    ObsEpochTiming(
        week,
        time_of_week,
        receive_time,
        fold_week_crossover(time_of_week - receive_time),
    )
end

# An epoch is written on the chunk that crosses its grid point, so a chunk longer than the
# interval crosses several and writes only the last: the file has fewer epochs than its
# `INTERVAL` says.
function warn_interval_below_chunk(logger::RinexLogger)
    logger.warned_interval && return nothing
    logger.warned_interval = true
    @warn "RinexConfig.interval is $(logger.config.interval), shorter than a processed " *
          "chunk, so RINEX epochs are written once per chunk rather than once per " *
          "interval. Choose an interval longer than a chunk."
    nothing
end

# Build one observation epoch, steered onto `timing`'s nominal epoch, from the satellites of
# the PVT solution.
function build_obs_epoch(logger::RinexLogger, receiver_state, timing::ObsEpochTiming)
    track_state = receiver_state.track_state
    propagation = timing.propagation
    # Group the per-signal observations by satellite: a satellite tracked on several signals
    # contributes one RINEX record carrying each signal's observation types. `SatObs` takes
    # their placement from the header, so they are collected by descriptor and in any order.
    satellites = Dict{Tuple{Char,Int},Vector{Pair{String,ObsValue}}}()
    # The satellites of the PVT solution are exactly those with a decoded time of week, and
    # so the only ones a transmit time — and hence a pseudorange — exists for.
    for (group_key, prn) in keys(receiver_state.pvt.satellites)
        layout = get(logger.layouts, group_key, nothing)
        isnothing(layout) && continue
        sat_state = get_sat_state(track_state, group_key, prn)
        decoder = receiver_state.receiver_sat_states[group_key][prn].decoder
        # Doppler on GNSS time, the receiver oscillator's common-mode offset removed, as the
        # epochs, pseudoranges and phases are.
        doppler =
            ustrip(Hz, get_carrier_doppler(sat_state)) +
            relative_clock_drift(receiver_state.pvt) * layout.carrier_freq
        # In dB-Hz, the unit of the S observable. An estimate that has not converged is not
        # finite, which RINEXParser writes blank.
        cn0 = ustrip(estimate_cn0(sat_state, RANGING_SIGNAL_INDEX))
        ssi = signal_strength_indicator(cn0)

        # A difference of nearby times of week: one of absolute seconds would quantize the
        # range to tens of metres.
        transmit_time =
            uncorrected_transmit_time(decoder, layout.signal, sat_state, logger.time_system)
        range = fold_week_crossover(timing.receive_time - transmit_time) * SPEED_OF_LIGHT
        # Carried onto the steered epoch at its own rate, `-λ·f_d`.
        pseudorange = range - wavelength(layout) * doppler * propagation

        observations = get!(() -> Pair{String,ObsValue}[], satellites, (layout.system, prn))
        codes = layout.codes
        push!(observations, codes.C => ObsValue(pseudorange; ssi))
        # A satellite whose carrier loop measures no phase now has no arc; its phase field
        # stays blank.
        accumulator = get(logger.accumulators, (group_key, prn), nothing)
        if !isnothing(accumulator)
            phase, lli =
                carrier_phase!(accumulator, logger, layout, range, doppler, propagation)
            push!(observations, codes.L => ObsValue(phase; lli, ssi))
        end
        push!(observations, codes.D => ObsValue(doppler))
        push!(observations, codes.S => ObsValue(cn0))
    end
    isempty(satellites) && return nothing

    (; time, fractional_second) = epoch_time(
        logger.time_system;
        week = timing.week,
        time_of_week = timing.time_of_week,
    )
    # Deterministic satellite order within the epoch, as RINEX files conventionally carry.
    sats = [
        SatObs(logger.header, system, prn, satellites[(system, prn)]) for
        (system, prn) in sort!(collect(keys(satellites)))
    ]
    ObsEpoch(time, sats; fractional_second)
end

# The carrier phase [cycles] of an arc and its loss-of-lock indicator: the negated beat phase
# on system time, with a whole-cycle ambiguity near the arc's first `range`, carried onto the
# steered epoch at its own rate, `-f_d`.
function carrier_phase!(accumulator, logger, layout, range, doppler, propagation)
    beat_phase =
        carrier_beat_phase(accumulator) + layout.carrier_freq * logger.clock.phase
    isnan(accumulator.anchor) &&
        (accumulator.anchor = round(range / wavelength(layout) + beat_phase))
    carrier_phase =
        accumulator.anchor - beat_phase - doppler * propagation +
        something(layout.phase_shift, 0.0)
    # The loss-of-lock indicator flags the first epoch of an arc; after it the phase is
    # continuous with the previous epoch's. Without the carrier's polarity the phase may be
    # half a cycle off, which the half-cycle bit says.
    lli = loss_of_lock_indicator(;
        loss_of_lock = accumulator.restarted,
        half_cycle = iszero(accumulator.polarity),
    )
    accumulator.restarted = false
    carrier_phase, lli
end
