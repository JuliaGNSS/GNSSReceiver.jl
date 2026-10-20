# ─────────────────────────────────────────────────────────────────────────────
# `receive` over a hardware correlator's loop process.
#
# Per chunk of raw samples:
#   1. acquisition — on the CPU, in a task of its own so the events keep
#      draining while it runs; a detected satellite is armed on a free hardware
#      channel with the code phase it had at the device sample the acquisition
#      window started at;
#   2. events — every channel's records feed the satellite's lock detectors,
#      its bits are queued with the device sample they completed at, its epoch
#      states are kept for the common fold boundary;
#   3. lock — a satellite whose detectors lose it is released and later
#      reacquired;
#   4. navigation — every `pvt_update_interval`, the decoders are brought up to
#      one fold boundary B that every channel has passed, and the PVT is solved
#      from the epoch states at B: code phases on one reception instant and the
#      bits that completed by it.
#
# The raw stream must be one the device names sample for sample: host sample k
# is device sample `device_sample_origin() + k` (GNSSM2SDR's raw stream fills a
# lost DMA buffer with zeros to keep it so).
# ─────────────────────────────────────────────────────────────────────────────

# How many of a satellite's epoch states are kept for the common boundary.
const EPOCH_HISTORY = 16

# How long an arm may wait for the loop's confirmation before it is given up.
const ARM_CONFIRM_TIMEOUT = 2.0s

"""
    HardwareSat

One satellite on a hardware channel, as the receiver sees it through the loop's
events: its decoder and lock detectors (a [`ReceiverSatState`](@ref)), the bits
waiting for the decoder, the latest epoch states and this chunk's records.
"""
mutable struct HardwareSat{DS<:GNSSDecoderState,S<:AbstractGNSSSignal}
    const signal::S
    const group_key::Symbol
    const prn::Int
    const channel::Int
    const arm_sequence::UInt64
    state::ReceiverSatState{DS}
    confirmed::Bool
    rejected::Bool
    armed_at::typeof(1.0s)
    # Bits not yet decoded, with the device sample their record ended at. A NaN
    # entry is a break in the bit stream — the loop restarted the bit clock, or
    # events were lost — where the decoder must start its sync over.
    const bit_samples::Vector{Int64}
    const bit_values::Vector{Float32}
    bit_clock_restarts::Int
    # The newest epoch states, a ring keyed by boundary.
    const epoch_boundaries::Vector{Int64}
    const epoch_states::Vector{HLP.EpochStateEvent}
    epoch_head::Int
    # This chunk's records.
    const prompts::Vector{ComplexF64}
    last_prompt::ComplexF64
    cn0_linear::Float64
    last_integration_samples::Int64
    records::Int
    # Device samples the records since the last lock update covered: the
    # detectors' clock is the device's, not the arrival of raw chunks, which
    # comes in bursts whenever this process has been held up.
    record_samples::Int64
    last_record_runtime::typeof(1.0s)
end

function HardwareSat(system, signal, prn::Int, channel::Int, sequence::UInt64, code_lock_threshold, runtime)
    state = ReceiverSatState(system, prn, code_lock_threshold)
    HardwareSat(
        signal, signal_group_key(system), prn, channel, sequence, state, false, false, runtime,
        Int64[], Float32[], 0,
        fill(typemin(Int64), EPOCH_HISTORY), Vector{HLP.EpochStateEvent}(undef, EPOCH_HISTORY), 0,
        ComplexF64[], complex(0.0, 0.0), 0.0, 0, 0, 0, -Inf * 1.0s,
    )
end

function _push_epoch_state!(sat::HardwareSat, boundary::Int64, ev::HLP.EpochStateEvent)
    sat.epoch_head = mod1(sat.epoch_head + 1, EPOCH_HISTORY)
    sat.epoch_boundaries[sat.epoch_head] = boundary
    sat.epoch_states[sat.epoch_head] = ev
    nothing
end

latest_boundary(sat::HardwareSat) = sat.epoch_head == 0 ? typemin(Int64) : sat.epoch_boundaries[sat.epoch_head]

function epoch_state_at(sat::HardwareSat, boundary::Int64)
    for k in eachindex(sat.epoch_boundaries)
        sat.epoch_boundaries[k] == boundary && return sat.epoch_states[k]
    end
    nothing
end

# A break in the satellite's bit stream at device sample `sample`.
function push_bit_break!(sat::HardwareSat, sample::Int64)
    push!(sat.bit_samples, sample)
    push!(sat.bit_values, NaN32)
    sat
end

# Decode every queued bit that completed at or before `boundary`, in order,
# starting the decoder's sync over at every break.
function decode_until!(sat::HardwareSat, boundary::Int64)
    n = 0
    while n < length(sat.bit_samples) && sat.bit_samples[n+1] <= boundary
        n += 1
    end
    n == 0 && return sat
    st = sat.state
    decoder = st.decoder
    first = 1
    for k = 1:n
        isnan(sat.bit_values[k]) || continue
        k > first && (decoder = decode!(decoder, view(sat.bit_values, first:(k-1)), k - first))
        decoder = reset_decoder_state!(decoder)
        first = k + 1
    end
    n >= first && (decoder = decode!(decoder, view(sat.bit_values, first:n), n - first + 1))
    sat.state = @set st.decoder = decoder
    deleteat!(sat.bit_samples, 1:n)
    deleteat!(sat.bit_values, 1:n)
    sat
end

# The records since the last update into the satellite's lock detectors, for
# the signal time they covered.
function update_lock!(sat::HardwareSat, sampling_freq)
    st = sat.state
    # Nothing measured since (the arm has not landed, or no record arrived): the
    # detectors wait — `_stale` decides when waiting has gone on too long.
    (!sat.confirmed || sat.records == 0) && return sat
    covered = uconvert(s, sat.record_samples / sampling_freq)
    cn0 = 10 * log10(max(sat.cn0_linear, 1e-3)) * dBHz
    integration_time = sat.last_integration_samples / sampling_freq
    code = update(st.code_lock_detector, cn0, integration_time, covered)
    carrier = update(st.carrier_lock_detector, sat.prompts, covered)
    sat.state = ReceiverSatState(st.prn, st.decoder, code, carrier, st.time_in_lock + covered,
                                 0.0s, 0, false)
    empty!(sat.prompts)
    sat.records = 0
    sat.record_samples = 0
    sat
end

# A confirmed satellite whose records stopped (the loop process stalled or
# dropped it) is as good as lost.
_stale(sat::HardwareSat, runtime) = sat.confirmed && runtime - sat.last_record_runtime > 1.0s

"""
    HardwareReceiverState

What `receive` keeps over a loop process between chunks: the satellites by
hardware channel, the noise reference per band, the acquisition plans and
buffer, the latest PVT solution and the clocks.
"""
mutable struct HardwareReceiverState{Sys<:Tuple,Plans<:NamedTuple,B,Sats}
    const systems::Sys
    const acq_plans::Plans
    acquisition_buffer::B
    const channels::Vector{Union{Nothing,Sats}}
    noise_channel::Int
    noise_sequence::UInt64
    noise_confirmed::Bool
    noise_armed_at::typeof(1.0s)
    pvt::PVTSolution
    runtime::typeof(1.0s)
    host_samples::Int64
    last_acquisition::typeof(1.0s)
    acquisition::Union{Nothing,Task}
    last_pvt::typeof(1.0s)
    last_output::typeof(1.0s)
    last_pvt_boundary::Int64
    arms::Int
    rejections::Int
    releases::Int
    # Outputs dropped because the consumers were behind.
    outputs_dropped::Int
end

# One satellite type for every group: the receiver's systems share one signal
# type here (several constellations would need a type per group; see the
# limitations in the docstring of `receive`).
_hardware_sat_type(system) =
    HardwareSat{decoder_state_type(system),typeof(ranging_signal(system))}

"""
    receive(loop::HardwareLoop, measurement_channel, systems, sampling_freq;
            device_sample_origin, replica_amplitude, kwargs...)

Run the receiver over a hardware correlator: acquire from the raw samples on
`measurement_channel`, track on the FPGA through `loop`'s process, decode and
solve the PVT here. Returns a channel of [`ReceiverDataOfInterest`](@ref), one
per PVT solution, like the software [`receive`](@ref). Unlike it, a consumer
that falls 16 outputs behind misses outputs rather than holding up tracking.

`device_sample_origin` is the device's sample index of the stream's first
sample: an integer, or a function returning it (called once the first chunk
has arrived, so a stream that learns it from the samples can supply it).
`replica_amplitude` is the device's carrier-table amplitude (127 for
gnss-m2sdr's TDM correlator, 1 for HardwareLoopCore's `SimulatedDevice`), so
the published prompts are on the signal's scale; it depends on the device, so
there is no default.

Keywords as the software receiver's where they apply — `acquire_every`,
`prns`, `pvt_update_interval`, `time_in_lock_before_calculating_pvt`,
`code_lock_cn0_threshold`, `enable_ionospheric_correction`,
`enable_tropospheric_correction`, `pvt_approximate_year` — plus `rf_input`
(the RF input the satellites are correlated on; the band table's by default),
`noise_prn` (the decoy the band's noise
reference starts on) and `acquisition_doppler_coverage` (the Doppler span the
acquisition searches, wide by default because a hardware front end's
oscillator offset adds to the satellites' Doppler), `acquisition_doppler_resolution`
(the Doppler bin; finer bins integrate longer and find weaker satellites; the
software receiver's by default) and `debug` (log each satellite's state and the
solve once a second).

Limitations: one band, one antenna, plain (non-combined) signals of one type,
scalar tracking.
"""
function receive(
    loop::HardwareLoop,
    measurement_channel::SignalChannel,
    systems,
    sampling_freq;
    device_sample_origin,
    acquire_every = 10s,
    prns = nothing,
    pvt_update_interval = 100ms,
    time_in_lock_before_calculating_pvt = 2s,
    code_lock_cn0_threshold = nothing,
    enable_ionospheric_correction = true,
    enable_tropospheric_correction = true,
    pvt_approximate_year::Integer = year(now(UTC)),
    rf_input::Integer = Int(first(loop.bands).rf_input),
    replica_amplitude::Real,
    noise_prn::Integer = 32,
    acquisition_doppler_coverage = 20_000.0Hz,
    acquisition_doppler_resolution = nothing,
    debug::Bool = false,
)
    systems = as_systems(systems)
    assert_single_band(systems)
    assert_decodable(systems)
    all(s -> s isa AbstractGNSSSignal, systems) ||
        throw(ArgumentError("the hardware path tracks plain signals; a CombinedSignal is not supported yet"))
    allequal(map(typeof, systems)) ||
        throw(ArgumentError("the hardware path tracks one signal type for now"))
    num_antenna_channels(measurement_channel) == 1 ||
        throw(ArgumentError("the hardware path takes one antenna's raw samples"))
    fs = Float64(ustrip(Hz, sampling_freq))
    band = first(loop.bands)
    band.sampling_freq_hz == fs ||
        throw(ArgumentError("the loop runs at $(band.sampling_freq_hz) Hz, the stream at $fs Hz"))

    # The software receiver's bin (half the loop's pull-in range) unless asked for
    # finer: a finer bin integrates longer coherently, which finds weaker
    # satellites and hands them over closer to their Doppler.
    resolutions = map(systems) do system
        something(acquisition_doppler_resolution,
                  2 * 0.5 * carrier_doppler_pull_in_range(SCALAR_DOPPLER_ESTIMATOR, system))
    end
    _, acq_plans, num_acq_samples = plan_band_acquisition(systems, sampling_freq, resolutions; prns,
                                                          min_doppler_coverage = acquisition_doppler_coverage)
    T = eltype(eltype(measurement_channel))
    Sats = _hardware_sat_type(first(systems))
    state = HardwareReceiverState(
        systems, acq_plans, SampleBuffer(T, num_acq_samples),
        Union{Nothing,Sats}[nothing for _ = 1:loop.num_channels],
        0, UInt64(0), false, -Inf * 1.0s, PVTSolution(), 0.0s, Int64(0), -Inf * 1.0s, nothing, -Inf * 1.0s, -Inf * 1.0s, typemin(Int64), 0, 0, 0, 0,
    )
    data_channel = Channel{ReceiverDataOfInterest{SatelliteDataOfInterest{ComplexF64}}}(16)
    origin_ref = Ref{Int64}(-1)
    # On the interactive threadpool when there is one (`julia -t auto,1`, one
    # more if the raw stream's reader holds one): the acquisition's FFT tasks
    # fill the default pool for seconds, and a chunk loop waiting behind them
    # lets the raw stream overflow.
    Base.errormonitor(
        Threads.@spawn :interactive try
            while true
                measurement = try
                    take!(measurement_channel)
                catch e
                    e isa InvalidStateException && break
                    rethrow(e)
                end
                if origin_ref[] < 0
                    origin_ref[] = device_sample_origin isa Integer ? Int64(device_sample_origin) :
                                   Int64(device_sample_origin())
                end
                loop.segment.open || break
                new_pvt = @lock loop.lock process_hardware_chunk!(
                    state, loop, measurement, sampling_freq, origin_ref[];
                    acquire_every, pvt_update_interval, time_in_lock_before_calculating_pvt,
                    code_lock_cn0_threshold, enable_ionospheric_correction,
                    enable_tropospheric_correction, pvt_approximate_year, rf_input,
                    replica_amplitude, noise_prn, debug,
                )
                # The loop was closed under the run: there is nothing to track with.
                loop.segment.open || break
                if new_pvt || state.runtime - state.last_output >= 3 * pvt_update_interval / 2
                    try
                        _offer!(data_channel, hardware_data_of_interest(state)) ||
                            (state.outputs_dropped += 1)
                    catch e
                        e isa InvalidStateException || rethrow(e)
                        break
                    end
                    state.last_output = state.runtime
                end
            end
        finally
            close(data_channel)
            close(measurement_channel)
        end
    )
    data_channel
end

# Hand `d` to the consumers unless they are behind: a full channel would block
# the chunk loop, and with it the heartbeat and the draining of the event rings,
# so a slow GUI would lose tracking history. A dropped output is superseded by
# the next. Throws `InvalidStateException` once the consumers have closed it.
function _offer!(ch::Channel, d)
    @lock ch begin
        isopen(ch) || throw(InvalidStateException("Channel is closed.", :closed))
        Base.n_avail(ch) < ch.sz_max || return false
        put!(ch, d)
    end
    true
end

"""
    process_hardware_chunk!(state, loop, measurement, sampling_freq, origin; kwargs...) -> Bool

One chunk: acquisition, arms, events, lock, navigation. Returns whether a new
PVT solution was computed.
"""
function process_hardware_chunk!(
    state::HardwareReceiverState,
    loop::HardwareLoop,
    measurement,
    sampling_freq,
    origin::Int64;
    acquire_every,
    pvt_update_interval,
    time_in_lock_before_calculating_pvt,
    code_lock_cn0_threshold,
    enable_ionospheric_correction,
    enable_tropospheric_correction,
    pvt_approximate_year,
    rf_input,
    replica_amplitude,
    noise_prn,
    debug = false,
)
    fs = Float64(ustrip(Hz, sampling_freq))
    num_samples = size(measurement, 1)
    signal_duration = num_samples / sampling_freq
    loop.segment.open || return false
    HLP.receiver_heartbeat!(loop.segment)
    # The loop process is the only source of tracking: without it the run is over.
    if fld(state.runtime, 1.0s) != fld(state.runtime + signal_duration, 1.0s) && !is_alive(loop)
        error("the loop process at $(loop.path) stopped beating")
    end
    state.host_samples += num_samples
    chunk_end = origin + state.host_samples          # device sample after the chunk

    # The band's noise reference: the C/N₀ of every satellite is measured
    # against it, so an arm that was rejected or never confirmed is tried again.
    if !state.noise_confirmed && state.runtime - state.noise_armed_at > ARM_CONFIRM_TIMEOUT
        ch = loop.num_channels
        system = first(state.systems)
        state.noise_sequence = send_command!(loop, HLP.COMMAND_ARM, ch, arm_command(
            system, noise_prn; carrier_doppler = 3000.0Hz, code_phase = 0.0, valid_at_sample = chunk_end,
            sampling_freq_hz = fs, signal_index = 0, rf_input, replica_amplitude))
        state.noise_channel = ch
        state.noise_armed_at = state.runtime
    end

    _acquire!(state, loop, measurement, sampling_freq, chunk_end, acquire_every; rf_input,
              replica_amplitude, code_lock_cn0_threshold)
    _drain!(state, loop)

    # Lock, and the satellites it lets go.
    for (ch, sat) in pairs(state.channels)
        isnothing(sat) && continue
        if sat.rejected
            state.channels[ch] = nothing
            continue
        end
        if !sat.confirmed && state.runtime - sat.armed_at > ARM_CONFIRM_TIMEOUT
            # The arm (or its confirmation) went missing: free the channel.
            send_command!(loop, HLP.COMMAND_RELEASE, ch, HLP.ReleaseCommand())
            state.channels[ch] = nothing
            state.releases += 1
            continue
        end
        update_lock!(sat, sampling_freq)
        if !is_in_lock(sat.state) || _stale(sat, state.runtime)
            send_command!(loop, HLP.COMMAND_RELEASE, ch, HLP.ReleaseCommand())
            state.channels[ch] = nothing
            state.releases += 1
        end
    end

    state.runtime += signal_duration
    _navigate!(state, loop; pvt_update_interval, time_in_lock_before_calculating_pvt,
               enable_ionospheric_correction, enable_tropospheric_correction, pvt_approximate_year, debug)
end

# ── Acquisition ──────────────────────────────────────────────────────────────

function _acquire!(state, loop, measurement, sampling_freq, chunk_end, acquire_every; rf_input,
                   replica_amplitude, code_lock_cn0_threshold)
    task = state.acquisition
    if !isnothing(task)
        istaskdone(task) || return nothing
        state.acquisition = nothing
        state.last_acquisition = state.runtime
        state.acquisition_buffer = SampleBuffers.reset(state.acquisition_buffer)
        if istaskfailed(task)
            # A failed search costs one interval, not the run.
            @error "acquisition failed" exception = (task.exception, task.backtrace) maxlog = 5
            return nothing
        end
        results, valid_at = fetch(task)
        _arm_acquired!(state, loop, results, valid_at, Float64(ustrip(Hz, sampling_freq)); rf_input,
                       replica_amplitude, code_lock_cn0_threshold)
        return nothing
    end
    state.runtime - state.last_acquisition >= acquire_every || return nothing
    any(isnothing, view(state.channels, 1:(loop.num_channels-1))) || return nothing
    state.acquisition_buffer = buffer(state.acquisition_buffer, @view(measurement[:, 1]))
    SampleBuffers.isfull(state.acquisition_buffer) || return nothing
    # The window's first sample, on the device's counter.
    window = copy(SampleBuffers.get_samples(state.acquisition_buffer))
    valid_at = chunk_end - length(window)
    tracked = Set((sat.group_key, sat.prn) for sat in state.channels if !isnothing(sat))
    plans = state.acq_plans
    systems = state.systems
    # The band's samples put zero Doppler at its IF; the Dopplers found are
    # relative to it, as the loop's are.
    interm_freq = first(loop.bands).intermediate_frequency_hz * Hz
    state.acquisition = loop.acquisition = Threads.@spawn begin
        results = map(systems) do system
            key = signal_group_key(system)
            plan = plans[key]
            prns = [p for p in plan.avail_prns if !((key, p) in tracked)]
            isempty(prns) ? Any[] : acquire!(plan, window, prns; interm_freq)
        end
        results, valid_at
    end
    nothing
end

function _arm_acquired!(state, loop, results, valid_at, fs; rf_input, replica_amplitude, code_lock_cn0_threshold)
    for (system, group_results) in zip(state.systems, results)
        key = signal_group_key(system)
        threshold = something(code_lock_cn0_threshold, get_default_code_lock_cn0_threshold(ranging_signal(system)))
        for res in sort(filter(r -> is_detected(r; pfa = DEFAULT_ACQ_PFA), group_results); by = r -> -r.CN0)
            any(s -> !isnothing(s) && s.group_key == key && s.prn == res.prn, state.channels) && continue
            ch = findfirst(isnothing, view(state.channels, 1:(loop.num_channels-1)))
            isnothing(ch) && return nothing
            signal = ranging_signal(system)
            seq = send_command!(loop, HLP.COMMAND_ARM, ch, arm_command(
                signal, res.prn; carrier_doppler = res.carrier_doppler, code_phase = res.code_phase,
                valid_at_sample = valid_at, sampling_freq_hz = fs, group_key = key, rf_input, replica_amplitude))
            seq == 0 && return nothing          # the command ring is full: the next search retries
            state.channels[ch] = HardwareSat(system, signal, res.prn, ch, seq, threshold, state.runtime)
            state.arms += 1
        end
    end
    nothing
end

# ── Events ───────────────────────────────────────────────────────────────────

function _drain!(state::HardwareReceiverState, loop::HardwareLoop)
    on_lost = ch -> begin
        sat = state.channels[ch]
        # Bits may be among what was lost: the decoder cannot count across it.
        isnothing(sat) || push_bit_break!(sat, isempty(sat.bit_samples) ? latest_boundary(sat) : last(sat.bit_samples))
        nothing
    end
    drain_events!(loop; on_lost) do ch, tag, ring, view
        sat = state.channels[ch]
        kind = tag.kind
        if kind == HLP.EVENT_STATUS
            ev = HLP.payload(HLP.StatusEvent, ring, view)
            isnothing(ev) && return nothing
            if ch == state.noise_channel && ev.sequence == state.noise_sequence
                if ev.code == HLP.STATUS_ARMED
                    state.noise_confirmed = true
                elseif ev.code == HLP.STATUS_ARM_REJECTED
                    @warn "the loop process rejected the noise reference; retrying" reason = Int(ev.reason) maxlog = 5
                    state.noise_armed_at = -Inf * 1.0s
                end
            end
            isnothing(sat) && return nothing
            ev.sequence == sat.arm_sequence || ev.code == HLP.STATUS_BIT_CLOCK_RESTART || return nothing
            if ev.code == HLP.STATUS_ARMED
                sat.confirmed = true
                sat.last_record_runtime = state.runtime
            elseif ev.code == HLP.STATUS_ARM_REJECTED
                sat.rejected = true
                state.rejections += 1
                @warn "the loop process rejected an arm" channel = ch prn = sat.prn reason = Int(ev.reason) maxlog = 20
            elseif ev.code == HLP.STATUS_BIT_CLOCK_RESTART
                # In the queue where it happened: the bits before it were
                # published before it.
                push_bit_break!(sat, tag.device_sample)
                sat.bit_clock_restarts += 1
            end
            return nothing
        end
        (isnothing(sat) || !sat.confirmed || Int(tag.prn) != sat.prn) && return nothing
        if kind == HLP.EVENT_RECORD
            ev = HLP.payload(HLP.RecordEvent, ring, view)
            isnothing(ev) && return nothing
            push!(sat.prompts, ev.prompt)
            sat.last_prompt = ev.prompt
            sat.last_integration_samples = ev.integrated_samples
            sat.record_samples += ev.integrated_samples
            sat.last_record_runtime = state.runtime
            (ev.flags & HLP.RECORD_HAS_CN0) != 0 && (sat.cn0_linear = ev.cn0_linear_hz)
            sat.records += 1
        elseif kind == HLP.EVENT_BIT
            ev = HLP.payload(HLP.BitEvent, ring, view)
            isnothing(ev) && return nothing
            push!(sat.bit_samples, tag.device_sample)
            push!(sat.bit_values, ev.soft_bit)
        elseif kind == HLP.EVENT_EPOCH_STATE
            ev = HLP.payload(HLP.EpochStateEvent, ring, view)
            isnothing(ev) && return nothing
            _push_epoch_state!(sat, tag.device_sample, ev)
        end
        nothing
    end
end

# ── Navigation ───────────────────────────────────────────────────────────────

# The newest fold boundary whose bits every satellite has published: each one's
# epoch state at the boundary after it has been read.
function _common_boundary(state::HardwareReceiverState)
    common = typemax(Int64)
    any_sat = false
    for sat in state.channels
        isnothing(sat) && continue
        sat.confirmed || continue
        latest = latest_boundary(sat)
        latest == typemin(Int64) && continue
        previous = sat.epoch_head == 0 ? typemin(Int64) :
                   sat.epoch_boundaries[mod1(sat.epoch_head - 1, EPOCH_HISTORY)]
        previous == typemin(Int64) && continue
        common = min(common, previous)
        any_sat = true
    end
    any_sat ? common : typemin(Int64)
end

function _navigate!(state::HardwareReceiverState, loop; pvt_update_interval, time_in_lock_before_calculating_pvt,
                    enable_ionospheric_correction, enable_tropospheric_correction, pvt_approximate_year,
                    debug = false)
    B = _common_boundary(state)
    B == typemin(Int64) && return false
    # Keep every decoder current up to B.
    for sat in state.channels
        isnothing(sat) || decode_until!(sat, B)
    end
    state.runtime - state.last_pvt >= pvt_update_interval || return false
    B > state.last_pvt_boundary || return false
    groups = NamedTuple{map(signal_group_key, state.systems)}(map(state.systems) do system
        key = signal_group_key(system)
        signal = ranging_signal(system)
        SS = PositionVelocityTime.SatelliteState{Float64,decoder_state_type(system),typeof(signal)}
        prns = Int[]
        satellites = SS[]
        for sat in state.channels
            (isnothing(sat) || sat.group_key != key) && continue
            is_pvt_ready(sat.state, time_in_lock_before_calculating_pvt) || continue
            ev = epoch_state_at(sat, B)
            isnothing(ev) && continue
            push!(prns, sat.prn)
            push!(satellites, SS(;
                decoder = sat.state.decoder,
                system = signal,
                code_phase = ev.code_phase_chips,
                carrier_doppler = ev.carrier_doppler_hz * Hz,
                carrier_phase = 2π * ev.carrier_phase_cycles,
            ))
        end
        PositionVelocityTime.SignalGroup(signal, Dictionary(prns, satellites))
    end)
    state.pvt = calc_pvt(groups, state.pvt; enable_ionospheric_correction, enable_tropospheric_correction,
                         approximate_year = pvt_approximate_year)
    if debug && fld(state.runtime, 1.0s) != fld(state.runtime - pvt_update_interval, 1.0s)
        for sat in state.channels
            isnothing(sat) && continue
            st = sat.state
            dec = st.decoder
            @info "hardware sat" prn = sat.prn channel = sat.channel confirmed = sat.confirmed in_lock = is_in_lock(st) ranging_ready = is_ranging_ready(st) time_in_lock = st.time_in_lock has_state_at_B = !isnothing(epoch_state_at(sat, B)) code_phase = (s = epoch_state_at(sat, B); isnothing(s) ? NaN : s.code_phase_chips) sync = (s = epoch_state_at(sat, B); isnothing(s) ? false : (s.flags & HLP.STATE_SYNC_FOUND) != 0) bits_after_sync = dec.num_bits_after_valid_syncro_sequence complete = GNSSDecoder.is_decoding_completed_for_positioning(dec.data) healthy = is_sat_healthy(dec) restarts = sat.bit_clock_restarts queued_bits = length(sat.bit_values)
        end
        @info "hardware pvt" B fix = !isnothing(state.pvt.time) sats = length(state.pvt.sats)
    end
    state.last_pvt = state.runtime
    state.last_pvt_boundary = B
    true
end

# ── Output ───────────────────────────────────────────────────────────────────

function hardware_data_of_interest(state::HardwareReceiverState)
    sat_data = Dictionary{Tuple{Symbol,Int},SatelliteDataOfInterest{ComplexF64}}()
    for sat in state.channels
        isnothing(sat) && continue
        sat.confirmed || continue
        cn0 = 10 * log10(max(sat.cn0_linear, 1e-3)) * dBHz
        insert!(sat_data, (sat.group_key, sat.prn), SatelliteDataOfInterest(
            cn0, sat.last_prompt, is_sat_healthy(sat.state.decoder), false,
            is_ranging_ready(sat.state), is_in_lock(sat.state)))
    end
    ReceiverDataOfInterest(sat_data, state.pvt, state.runtime, nothing)
end
