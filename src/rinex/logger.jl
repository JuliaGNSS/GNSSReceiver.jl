# `RinexLogger`: turns the receiver state of each processed chunk into the records the
# writer task puts into the files.

# One RINEX update: a finished observation epoch (`nothing` before the first fix), the
# ephemerides to offer the navigation writer (which drops the ones it has already written),
# and the header material that only exists once there is a fix. Immutable, so the writer
# task owns it outright while the tracking loop moves on.
struct RinexRecord
    epoch::Union{Nothing,ObsEpoch}
    ephemerides::Vector{Union{GPSEphemeris,GalileoEphemeris,BeiDouEphemeris}}
    nav_header::RinexNavHeader
    approx_position::NTuple{3,Float64}
    leap_seconds::Union{Nothing,Int,LeapSeconds}
end

"""
    ReceiverClockPhase

The receiver oscillator's drift integrated over receiver time since the first chunk, which
every satellite's carrier phase is corrected by (see [`update_clock_phase!`](@ref)).
"""
Base.@kwdef mutable struct ReceiverClockPhase
    # The integral [s].
    phase::Float64 = 0.0
    # The drift it was last integrated with, `NaN` before the first solution.
    drift::Float64 = NaN
    # The receiver time [s] it was integrated to, and the first chunk's.
    runtime::Float64 = NaN
    first_runtime::Float64 = NaN
end

"""
    EpochSteering

Where the observation epochs stand on the grid of `RinexConfig.interval`, for
[`obs_epoch_timing`](@ref).
"""
Base.@kwdef mutable struct EpochSteering
    # How long [s] a solution is carried on to the chunks after it.
    max_solution_age::Float64
    # Epoch of the last PVT solution, to spot the chunk a fresh fix arrived on, and the
    # receiver time [s] it was measured at: every chunk's receive time is carried on from it.
    last_pvt_time::Union{Nothing,TAITime} = nothing
    solution_runtime::Float64 = -Inf
    # The previous chunk's receiver time [s] and the grid slot (week, index) its receive time
    # fell in (`nothing` without a solution), to find the chunk that crosses each grid point.
    previous_runtime::Float64 = -Inf
    previous_slot::Union{Nothing,Tuple{Int,Int}} = nothing
    # The grid slot of the last written epoch, so each one is written exactly once.
    last_written::Union{Nothing,Tuple{Int,Int}} = nothing
end

"""
    RinexLogger(config, band_systems, interm_freqs, sampling_freq; approximate_year,
                pvt_update_interval = 100ms)

Turns the receiver state of each processed chunk into RINEX 3.05 output. [`receive`](@ref)
builds one when its `write_rinex` keyword is enabled and calls [`log_rinex!`](@ref) once
per chunk; the files are written by a task this constructor spawns, so no file I/O happens
on the tracking loop. `close` stops it and blocks until the files are complete.

Observations start at the first fix, which gives the receive time of every chunk after it
(carried on from the latest solution for at most two `pvt_update_interval`s). Each epoch is
steered onto an integer multiple of `config.interval` in the file's time system and written
for the satellites of the PVT solution; see
[Epoch times, pseudoranges and the receiver clock](@ref). Ephemerides need no fix and are
offered to the navigation writer on their own `config.interval` cadence.
"""
Base.@kwdef mutable struct RinexLogger
    config::RinexConfig
    # The observation header the file is opened with, read for its `obs_codes` by `SatObs`.
    # The writer completes a copy of it, so it never changes under this side.
    header::Union{Nothing,RinexObsHeader}
    layouts::Dict{Symbol,RinexSignalLayout}
    # Constellations the run can produce observations and ephemerides for. Taken from the
    # configuration rather than from what has been decoded so far: it pins the navigation
    # header's satellite system, and a header written before a second constellation showed
    # up would make RINEXParser reject that constellation's records.
    satellite_systems::Vector{Char}
    # The time system of the epochs, fixed for the file as its header states it (GPS for a
    # mixed file) rather than following the PVT solution's reference system.
    time_system::GNSSSignals.TimeSystem
    approximate_year::Int
    channel::Channel{RinexRecord}
    task::Task
    # The writers, which the writer task owns until [`close`](@ref).
    obs_writer::Union{Nothing,RinexObsWriter}
    nav_writer::Union{Nothing,RinexNavWriter}
    # Where the files ended up, known once `close` has named them (`nothing` for a file
    # that is not written, or that held nothing to name it by).
    obs_path::Union{Nothing,String} = nothing
    nav_path::Union{Nothing,String} = nothing
    # Continuous carrier phase per `(group key, PRN)`, i.e. per tracked signal of a
    # satellite. An entry lives as long as its tracking arc; a satellite that drops out of
    # lock loses its entry and comes back on a fresh, loss-of-lock-flagged arc.
    accumulators::Dict{Tuple{Symbol,Int},CarrierPhaseAccumulator} =
        Dict{Tuple{Symbol,Int},CarrierPhaseAccumulator}()
    # Per `(group key, PRN)`, the decoder's latest bit count and the receiver time it last
    # found a preamble at (see `note_decoder_sync!`).
    decoder_syncs::Dict{Tuple{Symbol,Int},Tuple{Int,Float64}} =
        Dict{Tuple{Symbol,Int},Tuple{Int,Float64}}()
    steering::EpochSteering
    clock::ReceiverClockPhase = ReceiverClockPhase()
    # Receiver time [s] the ephemerides were last offered at.
    last_nav_runtime::Float64 = -Inf
    # Chunks processed so far, which identifies the arcs still alive (see
    # [`CarrierPhaseAccumulator`](@ref)).
    chunk_count::Int = 0
    # Signals whose navigation message RINEX 3.05 cannot express, warned about once each,
    # and whether a chunk longer than the epoch interval was.
    warned_signals::Set{Symbol} = Set{Symbol}()
    warned_interval::Bool = false
end

function RinexLogger(
    config::RinexConfig,
    band_signals::Tuple,
    interm_freqs::Tuple,
    sampling_freq;
    approximate_year::Integer = year(now(UTC)),
    pvt_update_interval = 100ms,
)
    check_rinex_config(config)
    obs_codes, layouts = rinex_layout(band_signals, interm_freqs, sampling_freq)
    satellite_systems = unique(layout.system for layout in values(layouts))
    # The writers are opened here rather than inside the task so that an unwritable path
    # fails at the `receive` call. A file named by the convention is named when it is
    # closed, from its first epoch (see `RinexConfig`). An observation value too wide for
    # its field — a pseudorange off a wrongly decoded time of week — is written blank rather
    # than costing the epoch of every other satellite; an ephemeris with a value too wide
    # for its field is left out.
    header =
        isnothing(config.obs_file) ? nothing :
        initial_obs_header(config, obs_codes, rinex_phase_shifts(layouts))
    obs_writer =
        isnothing(header) ? nothing :
        RinexObsWriter(rinex_target(config, config.obs_file), header; overflow = :blank)
    nav_writer = try
        isnothing(config.nav_file) ? nothing :
        RinexNavWriter(
            rinex_target(
                config,
                config.nav_file;
                session = obs_writer,
                with_marker_name = isnothing(obs_writer),
            ),
            initial_nav_header(satellite_systems);
            overflow = :skip,
        )
    catch
        # A navigation file that cannot be opened leaves no observation writer open behind
        # it. Closed without an epoch, one named by the convention keeps no file; one at a
        # fixed path is removed with the empty file it began.
        if !isnothing(obs_writer)
            close(obs_writer)
            config.obs_file isa String && rm(file_path(obs_writer); force = true)
        end
        rethrow()
    end
    # Buffered, so a slow disk cannot rendezvous with — and stall — the tracking loop.
    channel = Channel{RinexRecord}(16)
    RinexLogger(;
        config,
        header,
        layouts,
        satellite_systems,
        time_system = gnss_time_system(satellite_systems),
        approximate_year,
        channel,
        task = spawn_rinex_writer(channel, obs_writer, nav_writer),
        obs_writer,
        nav_writer,
        # A solution is carried on over a couple of update intervals: a vector-loop solution
        # is dated up to one interval back, and a scalar one may come a chunk late.
        steering = EpochSteering(; max_solution_age = 2 * ustrip(s, pvt_update_interval)),
    )
end

"""
    close(logger::RinexLogger)

Stop the RINEX output and block until the files are complete on disk. Closing also flushes
the header of a file that never saw a record, so a file at a configured path is valid RINEX
even for an aborted run, and names a file named by the convention after its own data (see
[`RinexConfig`](@ref)); `obs_path` and `nav_path` then say where the files are. A writer that
failed has already reported it, so this does not raise it again.
"""
function Base.close(logger::RinexLogger)
    close(logger.channel)
    try
        wait(logger.task)
    catch e
        e isa TaskFailedException || rethrow()
    end
    isnothing(logger.obs_writer) || (logger.obs_path = file_path(logger.obs_writer))
    isnothing(logger.nav_writer) || (logger.nav_path = file_path(logger.nav_writer))
    nothing
end

"""
    log_rinex!(logger::RinexLogger, receiver_state) -> Nothing

Advance the RINEX output by one processed chunk: keep every tracked satellite's continuous
carrier phase up to date and hand the writer task a finished observation epoch whenever a
fresh PVT solution crosses the next nominal epoch, plus the current ephemerides.

The carrier phase has to be accumulated on *every* chunk — that is what keeps its
whole-cycle count exact, see [`CarrierPhaseAccumulator`](@ref) — so this is called for all
chunks, not only the ones that produce an epoch.
"""
function log_rinex!(logger::RinexLogger, receiver_state)
    runtime = ustrip(s, receiver_state.runtime)
    logger.chunk_count += 1
    update_clock_phase!(logger.clock, receiver_state.pvt, runtime)
    update_carrier_phases!(logger, receiver_state, runtime)

    timing = obs_epoch_timing(logger, receiver_state)
    # Ephemerides need no fix, so they run on their own cadence off the signal runtime.
    nav_due =
        !isnothing(logger.config.nav_file) &&
        runtime - logger.last_nav_runtime >= ustrip(s, logger.config.interval)
    (isnothing(timing) && !nav_due) && return nothing
    logger.last_nav_runtime = runtime

    epoch = isnothing(timing) ? nothing : build_obs_epoch(logger, receiver_state, timing)
    ephemerides, nav_header = build_nav_records(logger, receiver_state)
    position = receiver_state.pvt.position
    record = RinexRecord(
        epoch,
        ephemerides,
        nav_header,
        (position[1], position[2], position[3]),
        something(logger.config.leap_seconds, nav_header.leap_seconds, Some(nothing)),
    )
    # A closed channel means the writer task is gone (it only ends early on an I/O failure,
    # which it has already reported): drop the record rather than take the run down with it.
    try
        put!(logger.channel, record)
    catch e
        e isa InvalidStateException || rethrow()
    end
    nothing
end

"""
    update_clock_phase!(clock::ReceiverClockPhase, pvt, runtime)

Every satellite's phase is measured against the receiver's oscillator and so carries its
drift. It is taken out with one integral for the whole receiver from the first chunk: per
arc, each satellite would keep a different fraction of it, and two satellites' phases would
no longer differ by whole cycles. Until the first solution the drift is unknown, so the
integral is filled in with that solution's. The pseudoranges are corrected by the clock
offset instead, which differs from this integral alike for every satellite.
"""
function update_clock_phase!(clock::ReceiverClockPhase, pvt, runtime)
    isnan(clock.first_runtime) && (clock.first_runtime = runtime)
    if !isnothing(pvt.time)
        drift = relative_clock_drift(pvt)
        clock.phase +=
            isnan(clock.drift) ? drift * (runtime - clock.first_runtime) :
            0.5 * (clock.drift + drift) * (runtime - clock.runtime)
        clock.drift = drift
    end
    clock.runtime = runtime
    nothing
end
