# ─────────────────────────────────────────────────────────────────────────────
# The receiver's side of the hardware-correlator loop process
# (docs/plans/2026-09-22-loop-process.md, Milestone 5).
#
# The tracking loops of a hardware correlator run in a separate,
# allocation-free process (`HardwareLoopCore.LoopCore` over a vendor driver, e.g.
# GNSSM2SDR's `gnss_loop`). This process talks to it through a
# `HardwareLoopProtocol` shared-memory segment: arm and release commands go
# out, epoch states, records, bits and status come back, and everything the
# rest of the receiver reads off the `TrackState` — Dopplers, code phases,
# soft bits, prompts, C/N₀ — is *mirrored* into it from those events. Nothing
# is correlated or folded here.
#
# `RemoteHardwareLoop` is a correlator source: `process` dispatches
# `advance_tracking!` on it instead of running the software correlator,
# and the raw stream still drives acquisition, decoding and PVT.
# ─────────────────────────────────────────────────────────────────────────────

using HardwareLoopProtocol: HardwareLoopProtocol
const HLP = HardwareLoopProtocol

"""
    device_sample_origin(sdr::AbstractHardwareCorrelatorSDR, band_id::Symbol) -> Int64

The device counter reading, on `band_id`'s counter, that corresponds to host raw
sample 0 of that band — what an acquisition's `valid_at_sample` has to be
offset by before it is handed to the loop process. The default is 0: device and
host count the same stream from the same origin (true of the simulated device).
A real device latches its counter when the raw stream starts.
"""
device_sample_origin(::AbstractHardwareCorrelatorSDR, ::Symbol) = Int64(0)

"""
    RemoteHardwareLoop(sdr; segment, spawn = nothing, kwargs...)

The receiver's handle on a loop process serving the device `sdr` streams from.

`segment` is the protocol segment's path (`/dev/shm/gnss-loop-<device>`), or an
already open `HardwareLoopProtocol.Segment` (a heap-backed one in the tests).
Given a path: a live loop found there is attached to and told to release every
channel; otherwise `spawn` (a `Cmd` starting the loop executable) is run and
waited for, or an `ArgumentError` says there is nothing to attach to. A loop
whose heartbeat stops for `heartbeat_timeout` seconds is restarted from `spawn`
and every tracked satellite re-armed from the receiver's own state.

`sdr` provides the raw streams ([`raw_sample_channel`](@ref)), the declared
[`hardware_capabilities`](@ref), the band plan and the device-counter origins
([`device_sample_origin`](@ref)).
"""
mutable struct RemoteHardwareLoop
    const sdr::AbstractHardwareCorrelatorSDR
    segment::HLP.Segment
    const segment_path::String
    const spawn::Union{Nothing,Base.AbstractCmd}
    process::Union{Nothing,Base.Process}
    const capabilities::HardwareCorrelatorCapabilities
    const band_plan::HardwareBandPlan
    const band_ids::Vector{Symbol}
    const band_sampling_freq::Vector{Float64}
    const band_origin::Vector{Int64}
    const num_channels::Int
    const num_ants::Int
    # Reference-band raw samples the receiver has processed: the host timebase.
    samples_consumed::Int64
    # ── The channel table ─────────────────────────────────────────────────────
    const assignments::Vector{Union{Nothing,HardwareChannelAssignment}}
    const channel_of::Dict{HardwareChannelAssignment,Int}
    const channel_band::Vector{Int}
    const armed::Vector{Bool}           # STATUS_ARMED received
    const arm_sequence::Vector{UInt64}
    const noise_channels::Vector{Int}   # per band; 0 for none
    next_sequence::UInt64
    # ── Per-chunk observation bookkeeping ─────────────────────────────────────
    const records_this_chunk::Vector{Int}
    const last_event_at::Vector{Int64}  # `samples_consumed` at the channel's last record
    const bit_clock_restarts::Vector{HardwareChannelAssignment}
    const max_gap_samples::Int64
    # ── Liveness ──────────────────────────────────────────────────────────────
    const heartbeat_timeout_ns::Int64
    const attach_timeout_s::Float64
    loop_alive::Bool
    restarts::Int
    # Where every mirrored epoch state is logged as CSV, or `nothing`.
    const trace::Union{Nothing,IO}
    # Ask the loop for every record's taps (diagnostics; they go to `trace`).
    const publish_taps::Bool
    # ── Counters ──────────────────────────────────────────────────────────────
    events::Int64
    lost_events::Int64
    arms_sent::Int64
    arms_rejected::Int64
    releases_sent::Int64
    commands_refused::Int64
    unassignable_signals::Int64
    unsupported_signals::Int64
end

function RemoteHardwareLoop(
    sdr::AbstractHardwareCorrelatorSDR;
    segment::Union{HLP.Segment,AbstractString},
    spawn::Union{Nothing,Base.AbstractCmd} = nothing,
    band_plan::Union{Nothing,HardwareBandPlan} = nothing,
    num_ants::Integer = 1,
    heartbeat_timeout::Real = 0.5,
    attach_timeout::Real = 10.0,
    max_dump_gap::Real = 0.5,
    trace::Union{Nothing,IO} = nothing,
    publish_taps::Bool = false,
)
    isnothing(trace) || println(trace, "host_sample,device_sample,channel,prn,signal_index,carrier_doppler_hz,code_doppler_hz,code_phase_chips,cn0_dbhz,block_count,flags,nco_carrier_hz,nco_code_hz")
    timeout_ns = round(Int64, heartbeat_timeout * 1e9)
    process = nothing
    release_all = false
    if segment isa HLP.Segment
        seg = segment
        path = seg.path
    else
        path = String(segment)
        process, seg, release_all = _attach_or_spawn(path, spawn, Float64(attach_timeout), timeout_ns)
    end
    bands = HLP.band_table(seg)
    band_ids = [Symbol(String(b.band_id)) for b in bands]
    band_fs = [b.sampling_freq_hz for b in bands]
    plan = something(band_plan, hardware_band_plan(sdr, band_ids, band_fs .* Hz))
    n = HLP.channel_count(seg)
    loop = RemoteHardwareLoop(
        sdr,
        seg,
        path,
        spawn,
        process,
        hardware_capabilities(sdr),
        plan,
        band_ids,
        band_fs,
        [device_sample_origin(sdr, id) for id in band_ids],
        n,
        Int(num_ants),
        0,
        Union{Nothing,HardwareChannelAssignment}[nothing for _ = 1:n],
        Dict{HardwareChannelAssignment,Int}(),
        zeros(Int, n),
        fill(false, n),
        zeros(UInt64, n),
        zeros(Int, length(bands)),
        UInt64(0),
        zeros(Int, n),
        fill(typemin(Int64), n),
        HardwareChannelAssignment[],
        round(Int64, max_dump_gap * first(band_fs)),
        timeout_ns,
        Float64(attach_timeout),
        true,
        0,
        trace,
        publish_taps,
        0, 0, 0, 0, 0, 0, 0, 0,
    )
    HLP.set_receiver_pid!(seg, getpid())
    HLP.receiver_heartbeat!(seg)
    # A loop that was already running holds whatever a previous receiver left
    # on it; start from a clean bank.
    release_all && _release_every_channel!(loop)
    loop
end

# ── Attaching, spawning, restarting ───────────────────────────────────────────

# Attach to a live loop at `path`, or spawn one and wait for it. Returns
# `(process, segment, attached_to_live_loop)`.
function _attach_or_spawn(path::String, spawn::Union{Nothing,Base.AbstractCmd}, attach_timeout::Float64, timeout_ns::Int64)
    if HLP.segment_exists(path)
        seg = _try_attach(path)
        if !isnothing(seg)
            HLP.heartbeat_alive(seg, :loop; stale_after_ns = timeout_ns) && return (nothing, seg, true)
            close(seg)
        end
    end
    isnothing(spawn) && throw(
        ArgumentError(
            "no live loop process serves the segment at $path, and no command to spawn one was given",
        ),
    )
    process = run(spawn; wait = false)
    seg = _wait_for_loop(path, process, attach_timeout, timeout_ns)
    (process, seg, false)
end

function _try_attach(path::String)
    try
        HLP.attach_segment(path)
    catch e
        e isa ArgumentError || e isa SystemError || rethrow()
        nothing
    end
end

# Poll until the loop process has created the segment, marked itself running
# and heartbeated — or has exited, or the timeout has passed.
function _wait_for_loop(path::String, process::Base.Process, attach_timeout::Float64, timeout_ns::Int64)
    deadline = time() + attach_timeout
    while time() < deadline
        if !process_running(process)
            throw(ErrorException("the loop process exited with code $(process.exitcode) before serving $path"))
        end
        if HLP.segment_exists(path)
            seg = _try_attach(path)
            if !isnothing(seg)
                if HLP.loop_state(seg) == HLP.LOOP_STATE_RUNNING &&
                   HLP.heartbeat_alive(seg, :loop; stale_after_ns = timeout_ns)
                    return seg
                end
                close(seg)
            end
        end
        sleep(0.02)
    end
    process_running(process) && kill(process)
    throw(ErrorException("the loop process did not start serving $path within $(attach_timeout) s"))
end

# Whether the loop is alive; when it is not and this receiver owns its command,
# restart it and forget the bank so the next pass re-arms everything.
function _check_loop!(loop::RemoteHardwareLoop)
    if HLP.heartbeat_alive(loop.segment, :loop; stale_after_ns = loop.heartbeat_timeout_ns)
        loop.loop_alive = true
        return true
    end
    if loop.loop_alive
        loop.loop_alive = false
        @warn "the loop process's heartbeat has gone stale" segment = loop.segment_path maxlog = 10
    end
    isnothing(loop.spawn) && return false
    _restart!(loop)
end

function _restart!(loop::RemoteHardwareLoop)
    process = loop.process
    if !isnothing(process) && process_running(process)
        kill(process)
        wait(process)
    end
    close(loop.segment)
    loop.process = run(loop.spawn; wait = false)
    loop.segment = _wait_for_loop(loop.segment_path, loop.process, loop.attach_timeout_s, loop.heartbeat_timeout_ns)
    HLP.set_receiver_pid!(loop.segment, getpid())
    HLP.receiver_heartbeat!(loop.segment)
    # The new loop's bank is empty; every assignment is re-armed from the
    # receiver's state on the next pass.
    fill!(loop.assignments, nothing)
    empty!(loop.channel_of)
    fill!(loop.armed, false)
    fill!(loop.arm_sequence, 0)
    fill!(loop.noise_channels, 0)
    fill!(loop.channel_band, 0)
    fill!(loop.last_event_at, typemin(Int64))
    empty!(loop.bit_clock_restarts)
    loop.restarts += 1
    loop.loop_alive = true
    @info "loop process restarted" segment = loop.segment_path restarts = loop.restarts
    true
end

"""
    close(loop::RemoteHardwareLoop; shutdown = !isnothing(loop.spawn))

Release every channel, ask the loop to shut down when this receiver spawned it,
and unmap the segment.
"""
function Base.close(loop::RemoteHardwareLoop; shutdown::Bool = !isnothing(loop.spawn))
    if loop.segment.open
        _release_every_channel!(loop)
        shutdown && _send!(loop, HLP.COMMAND_SHUTDOWN, 0, HLP.ShutdownCommand())
        close(loop.segment)
    end
    process = loop.process
    if !isnothing(process) && shutdown
        # Give the loop its exit; it may be blocked in the device.
        for _ = 1:100
            process_running(process) || break
            sleep(0.02)
        end
        if process_running(process)
            kill(process)
            wait(process)
        end
    end
    loop
end

# ── Commands ──────────────────────────────────────────────────────────────────

function _send!(loop::RemoteHardwareLoop, kind, channel::Integer, body)
    loop.next_sequence += 1
    sequence = loop.next_sequence
    ok = HLP.try_publish!(HLP.command_ring(loop.segment), HLP.CommandTag(kind, channel, sequence), body)
    ok || (loop.commands_refused += 1)
    ok ? sequence : UInt64(0)
end

function _release_every_channel!(loop::RemoteHardwareLoop)
    for ch = 1:loop.num_channels
        _send!(loop, HLP.COMMAND_RELEASE, ch, HLP.ReleaseCommand())
        loop.releases_sent += 1
    end
    fill!(loop.assignments, nothing)
    empty!(loop.channel_of)
    fill!(loop.armed, false)
    fill!(loop.noise_channels, 0)
    fill!(loop.channel_band, 0)
    loop
end

_band_index(loop::RemoteHardwareLoop, band_id::Symbol) =
    something(findfirst(==(band_id), loop.band_ids), 0)
_band_index(loop::RemoteHardwareLoop, system) = _band_index(loop, get_band_id(system_band(system)))

# Host reference-band sample → the device counter of `band_index`.
@inline function _device_sample(loop::RemoteHardwareLoop, band_index::Int, host_sample::Integer)
    scale = loop.band_sampling_freq[band_index] / first(loop.band_sampling_freq)
    loop.band_origin[band_index] + round(Int64, Int64(host_sample) * scale)
end

function _find_free_channel(loop::RemoteHardwareLoop)
    for ch = 1:loop.num_channels
        isnothing(loop.assignments[ch]) && !(ch in loop.noise_channels) && return ch
    end
    nothing
end

function _forget_channel!(loop::RemoteHardwareLoop, ch::Int)
    assignment = loop.assignments[ch]
    isnothing(assignment) || delete!(loop.channel_of, assignment)
    loop.assignments[ch] = nothing
    loop.armed[ch] = false
    loop.arm_sequence[ch] = 0
    loop.channel_band[ch] = 0
    loop.last_event_at[ch] = typemin(Int64)
    isnothing(assignment) || filter!(!=(assignment), loop.bit_clock_restarts)
    nothing
end

# Satellites the receiver dropped give their channels back.
function _remote_release_stale!(loop::RemoteHardwareLoop, track_state)
    for ch = 1:loop.num_channels
        assignment = loop.assignments[ch]
        isnothing(assignment) && continue
        _is_tracked(track_state, assignment) && continue
        _send!(loop, HLP.COMMAND_RELEASE, ch, HLP.ReleaseCommand())
        loop.releases_sent += 1
        _forget_channel!(loop, ch)
    end
    loop
end

# One open-loop noise reference per band, armed once; the loop process re-arms
# it onto fresh decoys itself.
function _remote_ensure_noise!(loop::RemoteHardwareLoop, band_systems, band_measurements)
    for systems in band_systems
        isempty(systems) && continue
        signal = _noise_reference_signal(systems)
        isnothing(signal) && continue
        band_index = _band_index(loop, first(systems))
        band_index == 0 && continue
        loop.noise_channels[band_index] == 0 || continue
        ch = _find_free_channel(loop)
        isnothing(ch) && continue
        correlator = Tracking.get_default_correlator(signal, NumAnts(loop.num_ants))
        fs = loop.band_sampling_freq[band_index]
        shifts = _tap_sample_shifts(correlator, fs, get_code_frequency(signal))
        route = _route_or_reference(loop.band_plan, loop.band_ids[band_index])
        cmd = HLP.ArmCommand(;
            signal = get_signal_id(signal),
            group_key = signal_group_key(signal),
            prn = 1,
            signal_index = 0,
            carrier_doppler_hz = rand() * 10_000 - 5_000,
            code_doppler_hz = 0.0,
            code_phase_chips = rand() * get_code_length(signal),
            valid_at_sample = _device_sample(loop, band_index, loop.samples_consumed),
            tap_sample_shifts = shifts,
            num_taps = length(shifts),
            el_sample_spacing = _el_sample_spacing(correlator, fs, signal),
            band = band_index,
            replica_amplitude = Float64(correlator_gain(loop.sdr, loop.band_ids[band_index])),
            code_amplitude = Float64(replica_code_amplitude(loop.sdr, signal)),
            carrier_phase_offset = Float64(get_carrier_phase_offset(signal)),
            sampling_freq_hz = fs,
            rf_input = route.rf_input,
            device_index = route.device_index,
        )
        sequence = _send!(loop, HLP.COMMAND_ARM, ch, cmd)
        sequence == 0 && continue
        loop.arms_sent += 1
        loop.noise_channels[band_index] = ch
        loop.channel_band[ch] = band_index
        loop.arm_sequence[ch] = sequence
    end
    loop
end

_el_sample_spacing(correlator, fs, signal) = Int(
    Tracking.get_early_late_sample_spacing(correlator, _hz(fs), _hz(get_code_frequency(signal))),
)

# This chunk's acquisitions get channels: one per tracked signal component.
function _remote_assign_new!(loop::RemoteHardwareLoop, track_state, band_systems, band_measurements)
    for systems in band_systems, system in systems
        group_key = signal_group_key(system)
        band_index = _band_index(loop, system)
        band_index == 0 && continue
        for sat in get_sat_states(track_state, group_key)
            prn = get_prn(sat)
            for (signal_index, tracked_signal) in enumerate(Tracking.get_signals(sat))
                assignment = HardwareChannelAssignment(group_key, prn, signal_index)
                haskey(loop.channel_of, assignment) && continue
                ch = _find_free_channel(loop)
                if isnothing(ch)
                    loop.unassignable_signals += 1
                    continue
                end
                _remote_arm!(loop, ch, band_index, assignment, sat, tracked_signal)
            end
        end
    end
    loop
end

function _remote_arm!(loop::RemoteHardwareLoop, ch::Int, band_index::Int, assignment, sat, tracked_signal)
    signal = Tracking.get_signal(tracked_signal)
    correlator = Tracking.get_correlator(tracked_signal)
    fs = loop.band_sampling_freq[band_index]
    unsupported = hardware_support_error(
        loop.capabilities,
        signal,
        correlator,
        fs;
        num_ants = Tracking.get_num_ants(correlator),
        dump_tap_slots = HLP.MAX_TAP_VALUES ÷ max(1, loop.num_ants),
    )
    if !isnothing(unsupported)
        loop.unsupported_signals += 1
        @warn "loop process $unsupported" prn = assignment.prn maxlog = 10
        return loop
    end
    shifts = _tap_sample_shifts(correlator, fs, get_code_frequency(signal))
    band_id = loop.band_ids[band_index]
    route = _route_or_reference(loop.band_plan, band_id)
    cmd = HLP.ArmCommand(;
        signal = get_signal_id(signal),
        group_key = assignment.group_key,
        prn = assignment.prn,
        signal_index = assignment.signal_index,
        carrier_doppler_hz = ustrip(Hz, uconvert(Hz, get_carrier_doppler(sat))),
        code_doppler_hz = ustrip(Hz, uconvert(Hz, get_code_doppler(sat))),
        code_phase_chips = get_code_phase(sat),
        valid_at_sample = _device_sample(loop, band_index, loop.samples_consumed),
        tap_sample_shifts = shifts,
        num_taps = length(shifts),
        el_sample_spacing = _el_sample_spacing(correlator, fs, signal),
        band = band_index,
        replica_amplitude = Float64(correlator_gain(loop.sdr, band_id)),
        code_amplitude = Float64(replica_code_amplitude(loop.sdr, signal)),
        carrier_phase_offset = Float64(get_carrier_phase_offset(signal)),
        sampling_freq_hz = fs,
        rf_input = route.rf_input,
        device_index = route.device_index,
        want_taps = loop.publish_taps,
    )
    sequence = _send!(loop, HLP.COMMAND_ARM, ch, cmd)
    sequence == 0 && return loop
    loop.arms_sent += 1
    loop.assignments[ch] = assignment
    loop.channel_of[assignment] = ch
    loop.channel_band[ch] = band_index
    loop.armed[ch] = false
    loop.arm_sequence[ch] = sequence
    loop.last_event_at[ch] = loop.samples_consumed
    loop
end

# ── The chunk: commands out, events in, tracking state mirrored ──────────────

function advance_tracking!(loop::RemoteHardwareLoop, band_measurements, track_state, band_systems)
    # `track!`'s per-chunk contract: the bit store and prompt buffers are the
    # chunk's own, consumed by the decoder and the lock detectors after it.
    Tracking.reset_start_sample_and_bit_buffer!(track_state)
    # A closed handle (the receiver is shutting down while a chunk is still in
    # flight) has no segment to touch.
    loop.segment.open || return track_state
    HLP.receiver_heartbeat!(loop.segment)
    _check_loop!(loop)
    fill!(loop.records_this_chunk, 0)
    # Reconcile the bank with the tracking state before this chunk is counted:
    # an acquisition's code phase is valid at the chunk's first sample.
    _remote_release_stale!(loop, track_state)
    _remote_ensure_noise!(loop, band_systems, band_measurements)
    _remote_assign_new!(loop, track_state, band_systems, band_measurements)
    loop.samples_consumed += _chunk_num_samples(band_measurements)
    _drain_events!(loop, track_state)
    track_state
end

function _drain_events!(loop::RemoteHardwareLoop, track_state)
    for ch = 1:loop.num_channels
        ring = HLP.event_ring(loop.segment, ch)
        while true
            status, view, lost = HLP.peek!(ring, HLP.EventTag)
            status === :empty && break
            status === :lost && (loop.lost_events += lost)
            tag = view.tag
            _apply_event!(loop, track_state, ch, ring, tag, view)
            HLP.commit!(ring, view)
            loop.events += 1
        end
    end
    loop
end

function _apply_event!(loop::RemoteHardwareLoop, track_state, ch::Int, ring, tag::HLP.EventTag, view)
    kind = tag.kind
    if kind == HLP.EVENT_STATUS
        ev = HLP.payload(HLP.StatusEvent, ring, view)
        isnothing(ev) || _apply_status!(loop, ch, ev)
        return nothing
    end
    assignment = loop.assignments[ch]
    isnothing(assignment) && return nothing
    sats = get_sat_states(track_state, assignment.group_key)
    haskey(sats, assignment.prn) || return nothing
    sat = sats[assignment.prn]
    signal_index = assignment.signal_index
    signal_index in eachindex(Tracking.get_signals(sat)) || return nothing
    if kind == HLP.EVENT_RECORD
        ev = HLP.payload(HLP.RecordEvent, ring, view)
        isnothing(ev) && return nothing
        isnothing(loop.trace) || println(loop.trace, "R,", loop.samples_consumed, ",", tag.device_sample, ",", ch, ",", assignment.prn, ",",
            signal_index, ",", ev.integrated_samples, ",", ev.block_credit, ",", 10log10(max(ev.cn0_linear_hz, 1e-9)), ",", ev.flags, ",",
            abs(ev.prompt), ",", ev.applied_carrier_hz, ",", ev.applied_code_hz)
        sats[assignment.prn] = _mirror_record(sat, signal_index, ev)
        loop.records_this_chunk[ch] += 1
        loop.last_event_at[ch] = loop.samples_consumed
    elseif kind == HLP.EVENT_TAPS
        ev = HLP.payload(HLP.TapsEvent, ring, view)
        isnothing(ev) && return nothing
        if !isnothing(loop.trace)
            n = Int(tag.num_taps)
            print(loop.trace, "T,", loop.samples_consumed, ",", tag.device_sample, ",", ch, ",", assignment.prn, ",", ev.integrated_samples)
            for i = 1:min(n, length(ev.taps))
                print(loop.trace, ",", real(ev.taps[i]), ",", imag(ev.taps[i]))
            end
            println(loop.trace)
        end
    elseif kind == HLP.EVENT_BIT
        ev = HLP.payload(HLP.BitEvent, ring, view)
        isnothing(ev) && return nothing
        push!(Tracking.get_soft_bits(Tracking.get_signals(sat)[signal_index]), ev.soft_bit)
    elseif kind == HLP.EVENT_EPOCH_STATE
        ev = HLP.payload(HLP.EpochStateEvent, ring, view)
        isnothing(ev) && return nothing
        band_index = loop.channel_band[ch]
        band_index == 0 && return nothing
        isnothing(loop.trace) || println(loop.trace, loop.samples_consumed, ",", tag.device_sample, ",", ch, ",", assignment.prn, ",", signal_index, ",",
            ev.carrier_doppler_hz, ",", ev.code_doppler_hz, ",", ev.code_phase_chips, ",", 10log10(max(ev.cn0_linear_hz, 1e-9)), ",",
            ev.block_count, ",", ev.flags, ",", ev.nco_carrier_hz, ",", ev.nco_code_hz)
        sats[assignment.prn] = _mirror_epoch_state(loop, sat, signal_index, band_index, tag, ev)
    end
    nothing
end

function _apply_status!(loop::RemoteHardwareLoop, ch::Int, ev::HLP.StatusEvent)
    code = ev.code
    if code == HLP.STATUS_ARMED
        ev.sequence == loop.arm_sequence[ch] && (loop.armed[ch] = true)
    elseif code == HLP.STATUS_ARM_REJECTED
        ev.sequence == loop.arm_sequence[ch] || return nothing
        loop.arms_rejected += 1
        band = findfirst(==(ch), loop.noise_channels)
        isnothing(band) || (loop.noise_channels[band] = 0)
        assignment = loop.assignments[ch]
        @warn "the loop process rejected an arm" channel = ch reason = Int(ev.reason) prn =
            isnothing(assignment) ? 0 : assignment.prn maxlog = 20
        _forget_channel!(loop, ch)
    elseif code == HLP.STATUS_BIT_CLOCK_RESTART
        assignment = loop.assignments[ch]
        isnothing(assignment) || assignment in loop.bit_clock_restarts ||
            push!(loop.bit_clock_restarts, assignment)
    end
    nothing
end

# A record: the prompt for the lock detectors, the record's length for the
# C/N₀ normalisation and the C/N₀ itself into the signal's estimator.
function _mirror_record(sat, signal_index::Int, ev::HLP.RecordEvent)
    signals = Tracking.get_signals(sat)
    sig = signals[signal_index]
    push!(Tracking.get_filtered_prompts(sig), ev.prompt)
    has_cn0 = (ev.flags & HLP.RECORD_HAS_CN0) != 0
    new_sig = Tracking.TrackedSignal(
        sig;
        last_fully_integrated_correlator = _with_prompt(Tracking.get_last_fully_integrated_correlator(sig), ev.prompt),
        last_fully_integrated_filtered_prompt = ev.prompt,
        last_fully_integrated_num_code_blocks = max(1, Int(ev.block_credit)),
        cn0_estimator = has_cn0 ? _mirror_cn0(Tracking.get_cn0_estimator(sig), ev.cn0_linear_hz) :
                        Tracking.get_cn0_estimator(sig),
    )
    Tracking.TrackedSat(sat; signals = Base.setindex(signals, new_sig, signal_index))
end

# The record's prompt into the correlator the receiver's outputs read the
# prompt from (`_ranging_prompt`); the other taps are not carried by the event.
function _with_prompt(correlator::Tracking.AbstractCorrelator, prompt::ComplexF64)
    accumulators = Tracking.get_accumulators(correlator)
    eltype(accumulators) === ComplexF64 || return correlator
    Tracking.update_accumulator(
        correlator,
        Base.setindex(accumulators, prompt, Tracking.get_prompt_index(correlator)),
    )
end

# The loop's per-record C/N₀ takes the place of the term the estimator would
# have computed from a prompt and a noise density.
function _mirror_cn0(estimator::Tracking.NoiseRefCN0Estimator, cn0_linear_hz::Float64)
    buffer = estimator.buffered_cn0
    n = length(buffer)
    next_index = mod(estimator.current_index, n) + 1
    buffer[next_index] = cn0_linear_hz
    Tracking.NoiseRefCN0Estimator(estimator.num_records, buffer, next_index, min(estimator.filled_length + 1, n))
end
_mirror_cn0(estimator, ::Float64) = estimator

# The epoch state: the satellite's Dopplers, its code phase extrapolated to the
# end of this chunk on the common reception instant, its carrier phase, and the
# signal's sync flags.
function _mirror_epoch_state(loop::RemoteHardwareLoop, sat, signal_index::Int, band_index::Int, tag::HLP.EventTag, ev::HLP.EpochStateEvent)
    signals = Tracking.get_signals(sat)
    sig = signals[signal_index]
    bb = Tracking.get_bit_buffer(sig)
    found = (ev.flags & HLP.STATE_SYNC_FOUND) != 0
    B = typeof(bb.code_block_buffer)
    new_bb = Tracking.BitBuffer{B}(
        bb.code_block_buffer,
        bb.code_block_buffer_length,
        found,
        Int(ev.secondary_phase),
        ev.polarity,
        bb.prompt_accumulator,
        Int(ev.block_count),
        bb.soft_bits,
        bb.phase_acc,
    )
    new_sig = Tracking.TrackedSignal(sig; bit_buffer = new_bb)
    new_signals = Base.setindex(signals, new_sig, signal_index)
    signal_index == RANGING_SIGNAL_INDEX || return Tracking.TrackedSat(sat; signals = new_signals)
    signal = Tracking.get_signal(sig)
    fs = loop.band_sampling_freq[band_index]
    chips_per_sample = (_hz(get_code_frequency(signal)) + ev.code_doppler_hz) / fs
    # From the fold boundary on the band's counter to the end of this chunk.
    chunk_end = _device_sample(loop, band_index, loop.samples_consumed)
    code_phase = ev.code_phase_chips + (chunk_end - Int64(tag.device_sample)) * chips_per_sample
    code_phase = mod(code_phase, Tracking.current_code_wrap(new_signals))
    Tracking.TrackedSat(
        sat;
        carrier_doppler = ev.carrier_doppler_hz * Hz,
        code_doppler = ev.code_doppler_hz * Hz,
        code_phase,
        carrier_phase = ev.carrier_phase_cycles,
        signals = new_signals,
    )
end

# ── What `process` asks a correlator source ───────────────────────────────────

function is_observation_gap(loop::RemoteHardwareLoop, track_state, group_key, prn)
    ch = get(loop.channel_of, HardwareChannelAssignment(group_key, prn, RANGING_SIGNAL_INDEX), 0)
    ch == 0 && return false
    loop.records_this_chunk[ch] == 0 || return false
    last = loop.last_event_at[ch]
    last == typemin(Int64) && return false
    loop.samples_consumed - last <= loop.max_gap_samples
end

function has_current_observations(loop::RemoteHardwareLoop, track_state, system, prn)
    group_key = signal_group_key(system)
    haskey(get_sat_states(track_state, group_key), prn) || return false
    all((RANGING_SIGNAL_INDEX, data_signal_index(system))) do signal_index
        ch = get(loop.channel_of, HardwareChannelAssignment(group_key, prn, signal_index), 0)
        ch != 0 && loop.records_this_chunk[ch] > 0
    end
end

function take_bit_clock_restart!(loop::RemoteHardwareLoop, group_key, prn)
    isempty(loop.bit_clock_restarts) && return false
    before = length(loop.bit_clock_restarts)
    filter!(a -> !(a.group_key == group_key && a.prn == prn), loop.bit_clock_restarts)
    length(loop.bit_clock_restarts) < before
end

# ── receive ───────────────────────────────────────────────────────────────────

"""
    receive(loop::RemoteHardwareLoop, systems, sampling_freq; kwargs...)

Run the pipeline with the tracking loops in the loop process `loop` talks to.
Acquisition, decoding and PVT run here off the device's raw stream(s); the
loops' Dopplers, code phases, bits and C/N₀ are mirrored from the loop's
events. Vector tracking closes the loops on the host and is refused. Every
other keyword is [`receive`](@ref)'s.
"""
function receive(
    loop::RemoteHardwareLoop,
    systems,
    sampling_freq;
    acquire_async::Bool = true,
    processing_threadpool::Symbol = :interactive,
    vector_tracking::Union{Bool,VectorTracking} = false,
    interm_freqs = nothing,
    interm_freq = nothing,
    kwargs...,
)
    vt_enabled(vector_tracking) && throw(
        ArgumentError(
            "vector tracking closes the tracking loops on the host; a loop process cannot serve it",
        ),
    )
    band_systems = _band_system_groups(systems)
    band_keys = map(s -> get_band_id(system_band(first(s))), band_systems)
    band_sampling_freqs = _per_band_values(sampling_freq, band_systems)
    for (band_id, fs) in zip(band_keys, band_sampling_freqs)
        index = _band_index(loop, band_id)
        index == 0 && throw(ArgumentError("the loop process serves no band $band_id (it has $(loop.band_ids))"))
        isapprox(loop.band_sampling_freq[index], _hz(fs); rtol = 1e-9) || throw(
            ArgumentError(
                "band $band_id runs at $(loop.band_sampling_freq[index]) Hz in the loop process, not $(_hz(fs)) Hz",
            ),
        )
    end
    validate_hardware_configuration(
        loop.sdr,
        band_systems,
        band_sampling_freqs;
        num_ants = get(kwargs, :num_ants, NumAnts(1)),
        band_plan = loop.band_plan,
    )
    raw_channels = map(band_id -> raw_sample_channel(loop.sdr, band_id), band_keys)
    receive(
        raw_channels,
        band_systems,
        band_sampling_freqs;
        interm_freqs = _per_band_values(something(interm_freqs, interm_freq, 0.0Hz), band_systems),
        correlator_source = loop,
        acquire_async,
        processing_threadpool,
        # The estimator only types the tracking state here: the loop process
        # runs the very same one.
        doppler_estimator = NCOReferencedPLLAndDLL(),
        vector_tracking = false,
        kwargs...,
    )
end
