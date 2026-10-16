# ─────────────────────────────────────────────────────────────────────────────
# The receiver's side of a hardware correlator whose tracking loops run in a
# separate, allocation-free loop process (HardwareLoopCore). The two processes
# share one HardwareLoopProtocol segment: this side sends arm / release
# commands on its command ring and reads, per hardware channel, the records,
# navigation bits, epoch states and status events the loop publishes.
#
# The receiver keeps everything that may allocate — acquisition, decoding, lock
# detection, PVT — and the loop keeps everything that has a deadline.
# ─────────────────────────────────────────────────────────────────────────────

const HLP = HardwareLoopProtocol

"""
    HardwareLoop(segment_path; spawn = nothing, attach_timeout = 30)

The receiver's handle on a hardware correlator's loop process, through the
HardwareLoopProtocol segment at `segment_path` (a file under `/dev/shm`).
With `spawn` (a `Cmd`), the loop process is started first and shut down again
by `close`; otherwise a running one is attached to. Attaching releases every
channel, so a loop left armed by an earlier receiver starts clean.

Hand it to [`receive`](@ref) together with the raw sample stream the
acquisition runs on.
"""
mutable struct HardwareLoop
    const path::String
    segment::HLP.Segment
    process::Union{Nothing,Base.Process}
    const num_channels::Int
    const bands::Vector{HLP.BandEntry}
    sequence::UInt64
    events::Int
    lost_events::Int
    commands_dropped::Int
    # Held by `receive`'s task for each chunk and by `close`, so the segment is
    # never unmapped under a chunk that is using it.
    const lock::ReentrantLock
end

function HardwareLoop(segment_path::AbstractString; spawn::Union{Nothing,Base.AbstractCmd} = nothing,
                      attach_timeout::Real = 30)
    path = String(segment_path)
    process = nothing
    if !isnothing(spawn)
        # A stale segment from an earlier run would be attached to before the
        # new loop has created its own.
        isfile(path) && rm(path)
        process = run(spawn; wait = false)
    end
    segment = _wait_for_segment(path, process, Float64(attach_timeout))
    loop = HardwareLoop(path, segment, process, HLP.channel_count(segment), HLP.band_table(segment), 0, 0, 0, 0,
                        ReentrantLock())
    HLP.set_receiver_pid!(segment, getpid())
    HLP.receiver_heartbeat!(segment)
    for ch = 1:loop.num_channels
        send_command!(loop, HLP.COMMAND_RELEASE, ch, HLP.ReleaseCommand())
    end
    loop
end

function _wait_for_segment(path::String, process, timeout::Float64)
    t0 = time()
    while true
        if isfile(path)
            seg = try
                HLP.attach_segment(path)
            catch
                nothing
            end
            if !isnothing(seg)
                HLP.heartbeat_alive(seg, :loop) && return seg
                close(seg)
            end
        end
        !isnothing(process) && !process_running(process) &&
            error("the loop process exited ($(process.exitcode)) before its segment was ready")
        time() - t0 > timeout && error("no live loop process at $path after $(timeout) s")
        sleep(0.05)
    end
end

"Whether the loop process is still beating."
is_alive(loop::HardwareLoop) = loop.segment.open && HLP.heartbeat_alive(loop.segment, :loop)

function Base.close(loop::HardwareLoop; shutdown::Bool = !isnothing(loop.process))
    @lock loop.lock _close(loop, shutdown)
end

function _close(loop::HardwareLoop, shutdown::Bool)
    loop.segment.open || return loop
    if shutdown
        send_command!(loop, HLP.COMMAND_SHUTDOWN, 0, HLP.ShutdownCommand())
        p = loop.process
        if !isnothing(p)
            t0 = time()
            while process_running(p) && time() - t0 < 5
                sleep(0.05)
            end
            process_running(p) && kill(p)
        end
    else
        for ch = 1:loop.num_channels
            send_command!(loop, HLP.COMMAND_RELEASE, ch, HLP.ReleaseCommand())
        end
    end
    close(loop.segment)
    loop
end

"""
    send_command!(loop, kind, channel, body) -> UInt64

Publish a command on the loop's command ring and return its sequence number,
the one the acknowledging status event echoes — or `0` if the ring was full
and the command was not published.
"""
function send_command!(loop::HardwareLoop, kind, channel::Integer, body)
    loop.sequence += 1
    ok = HLP.try_publish!(HLP.command_ring(loop.segment), HLP.CommandTag(kind, channel, loop.sequence), body)
    ok || (loop.commands_dropped += 1; return UInt64(0))
    loop.sequence
end

"""
    drain_events!(f, loop; on_lost = ch -> nothing)

Call `f(channel, tag, ring, view)` for every event the loop has published since
the last drain, channel by channel in publication order, then commit it.
History the receiver was too slow to read is counted in `loop.lost_events`,
and `on_lost(channel)` is called where it is missing, before the events after
the hole.
"""
function drain_events!(f::F, loop::HardwareLoop; on_lost::L = ch -> nothing) where {F,L}
    seg = loop.segment
    for ch = 1:loop.num_channels
        ring = HLP.event_ring(seg, ch)
        while true
            status, view, lost = HLP.peek!(ring, HLP.EventTag)
            status === :empty && break
            if status === :lost
                loop.lost_events += lost
                on_lost(ch)
            end
            f(ch, view.tag, ring, view)
            HLP.commit!(ring, view)
            loop.events += 1
        end
    end
    loop
end

# ── Arm commands ─────────────────────────────────────────────────────────────

# The tap offsets the loop's correlator for `signal` expects, in samples,
# latest tap first, as a fixed tuple.
function hardware_tap_shifts(signal::AbstractGNSSSignal, sampling_freq_hz::Float64, num_ants::NumAnts)
    template = TrackingLoops.get_default_correlator(signal, num_ants)
    shifts = TrackingLoops.get_correlator_sample_shifts(
        template,
        sampling_freq_hz,
        ustrip(Hz, get_code_frequency(signal)),
    )
    ntuple(i -> i <= length(shifts) ? Int32(shifts[i]) : Int32(0), Val(5)), length(shifts)
end

"""
    arm_command(signal, prn; carrier_doppler, code_phase, valid_at_sample,
                sampling_freq_hz, group_key, signal_index = 1, rf_input = 1,
                num_ants = NumAnts(1), replica_amplitude = 1.0)

The `ArmCommand` for one satellite signal: the Dopplers and the code phase
the acquisition measured at device sample `valid_at_sample`, the loop's own
tap layout for the signal, and the RF input to correlate on.
"""
function arm_command(
    signal::AbstractGNSSSignal,
    prn::Integer;
    carrier_doppler,
    code_phase::Real,
    valid_at_sample::Integer,
    sampling_freq_hz::Float64,
    group_key::Symbol = get_signal_id(signal),
    signal_index::Integer = 1,
    rf_input::Integer = 1,
    num_ants::NumAnts = NumAnts(1),
    replica_amplitude::Real = 1.0,
)
    shifts, num_taps = hardware_tap_shifts(signal, sampling_freq_hz, num_ants)
    carrier_hz = Float64(ustrip(Hz, carrier_doppler))
    HLP.ArmCommand(;
        signal = get_signal_id(signal),
        group_key,
        prn,
        signal_index,
        carrier_doppler_hz = carrier_hz,
        code_doppler_hz = carrier_hz * get_code_center_frequency_ratio(signal),
        code_phase_chips = Float64(code_phase),
        valid_at_sample,
        tap_sample_shifts = shifts,
        num_taps,
        sampling_freq_hz,
        rf_input,
        replica_amplitude,
        code_amplitude = Float64(get_code_amplitude(signal)),
        carrier_phase_offset = Float64(get_carrier_phase_offset(signal)),
    )
end
