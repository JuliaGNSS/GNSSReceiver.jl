# ─────────────────────────────────────────────────────────────────────────────
# The receiver over a loop process (docs/plans/2026-09-22-loop-process.md,
# Milestone 5), closed in-process: the loop core runs over the simulated device
# on a heap-backed protocol segment, serviced from the producer task after every
# chunk it correlates, and the receiver attaches to that segment exactly as it
# would to a real loop process's. Nothing here is a mock — the same
# `RemoteHardwareLoop` arms the channels, mirrors the events and answers
# `process`'s questions as against `gnss_loop` on the board.
# ─────────────────────────────────────────────────────────────────────────────

using GNSSReceiver: RemoteHardwareLoop, device_sample_origin
# Qualified rather than imported, to keep the test's `Main` namespace small.
using HardwareLoopCore: HardwareLoopCore, SimulatedDevice, LoopCore
const TL = HardwareLoopCore
using HardwareLoopProtocol
using HardwareLoopProtocol: create_segment, SegmentConfig, BandEntry

# The receiver-side device: the raw stream, the declared capabilities and the
# loop core it drives in-process.
struct SimulatedLoopSDR{R,D,C} <: AbstractHardwareCorrelatorSDR
    raw::R
    device::D
    core::C
    segment::HardwareLoopProtocol.Segment
    system::GPSL1CA
    fs::Float64
end

GNSSReceiver.raw_sample_channel(sdr::SimulatedLoopSDR) = sdr.raw
GNSSReceiver.num_hardware_channels(sdr::SimulatedLoopSDR) = length(sdr.device.channels)
function GNSSReceiver.hardware_capabilities(sdr::SimulatedLoopSDR)
    code_freq = ustrip(Hz, uconvert(Hz, get_code_frequency(sdr.system)))
    HardwareCorrelatorCapabilities(;
        signals = [get_signal_id(sdr.system)],
        modulations = [nameof(typeof(get_modulation(sdr.system)))],
        max_primary_code_length = get_code_length(sdr.system),
        code_frequency_limits = (code_freq, code_freq),
        tap_layouts = [3],
        max_tap_offset_chips = 1.0,
        num_antennas = 1,
        bands = [get_band_id(get_band(sdr.system))],
        num_rf_inputs = 1,
        max_secondary_code_length = 1,
        reports_code_phase = true,
    )
end

function simulated_loop_sdr(; chunk, n_channels = 4, fs = 4e6, handover_code_phase_error = 0.25, record_delay = 0)
    system = GPSL1CA()
    device = SimulatedDevice(
        system;
        sampling_freq = fs,
        num_channels = n_channels,
        handover_code_phase_error,
        record_delay_samples = record_delay,
    )
    segment = create_segment(
        nothing,
        SegmentConfig(; channel_count = n_channels, bands = [BandEntry(get_band_id(get_band(system)), fs)]),
    )
    core = LoopCore(device, (system,), segment)
    raw = GNSSReceiver.SignalChannel{ComplexF64,1}(chunk, 4)
    SimulatedLoopSDR(raw, device, core, segment, system, fs)
end

# The whole receiver against the in-process loop on `seconds` of a synthetic
# GPS L1 C/A signal.
function run_remote_closed_loop(;
    chunk = 4000,
    seconds = 1.4,
    prn = 11,
    true_doppler = 1200.0,
    initial_code_phase = 137.4,
    amplitude = 0.126,
    handover_code_phase_error = 0.25,
    record_delay = 0,
)
    sdr = simulated_loop_sdr(; chunk, handover_code_phase_error, record_delay)
    fs = sdr.fs
    system = sdr.system
    num_chunks = cld(round(Int, seconds * fs), chunk)
    true_code_freq = ustrip(uconvert(Hz, get_code_frequency(system))) +
                     true_doppler * get_code_center_frequency_ratio(system)
    producer = Threads.@spawn begin
        rng = Random.Xoshiro(0xC0FFEE)
        buf = Matrix{ComplexF64}(undef, chunk, 1)
        try
            for c = 0:(num_chunks-1)
                n0 = c * chunk
                for k = 1:chunk
                    t = (n0 + k - 1) / fs
                    code = get_code(system, initial_code_phase + true_code_freq * t, prn)
                    buf[k, 1] = amplitude * code * cis(2π * true_doppler * t) + randn(rng, ComplexF64)
                end
                # The device sees the chunk first, and the loop process folds
                # it, before the receiver gets the raw samples — the order a
                # live board imposes.
                TL.correlate_chunk!(sdr.device, view(buf, :, 1))
                TL.service_pass!(sdr.core; wait_ms = 0)
                put!(sdr.raw, copy(buf))
            end
        finally
            close(sdr.raw)
        end
    end
    Base.errormonitor(producer)
    loop = RemoteHardwareLoop(sdr; segment = sdr.segment)
    data_channel = receive(
        loop,
        system,
        fs * Hz;
        acquire_async = false,
        acquire_every = 20ms,
        prns = [prn],
        time_in_lock_before_calculating_pvt = 1000u"s",
    )
    results = collect_data(data_channel)
    wait(producer)
    code_length = get_code_length(system)
    tracked = sdr.device.channels[findfirst(c -> c.active && c.prn == prn, sdr.device.channels)]
    true_code_phase = mod(initial_code_phase + true_code_freq * sdr.device.sample_count / fs, code_length)
    code_error = mod(tracked.code_phase - true_code_phase + code_length / 2, code_length) - code_length / 2
    (; sdr, loop, results, system, prn, true_doppler, handover_code_phase_error, code_error,
       device_doppler = tracked.carrier_doppler)
end

@testset "The receiver closes the loop through a loop process ($delay-epoch record delay)" for delay in (0, 2)
    r = run_remote_closed_loop(; record_delay = delay * 4000)
    key = (get_signal_id(r.system), r.prn)
    @test length(r.results) > 0
    # The noise reference and the satellite were armed, nothing was refused.
    @test r.loop.arms_sent >= 2
    @test r.loop.arms_rejected == 0
    @test r.loop.commands_refused == 0
    @test r.loop.unsupported_signals == 0
    @test r.loop.lost_events == 0
    @test r.loop.restarts == 0
    # It ended the run tracked and in lock, with the C/N₀ the loop measured
    # against its noise reference (0.126² · 4 MS/s ≈ 48 dBHz).
    final = last(r.results)
    @test haskey(final.sat_data, key)
    sat = final.sat_data[key]
    @test sat.is_in_lock
    @test 40dBHz < sat.cn0 < 52dBHz
    # The loop converged on the true Doppler (within the tolerance the
    # in-process link's closed-loop test uses: acquisition hands over late in
    # this short run) and pulled the quarter-chip handover error in.
    @test abs(r.device_doppler - r.true_doppler) < 15.0
    @test abs(r.code_error) < 0.5 * r.handover_code_phase_error
    # Its events reached the tracking state: the receiver's prompt is the
    # loop's, and the loop's words really drove the device.
    @test abs(sat.prompt) > 0
    @test r.sdr.core.words_committed > 200
end

@testset "A remote loop refuses vector tracking and unknown bands" begin
    sdr = simulated_loop_sdr(; chunk = 4000)
    loop = RemoteHardwareLoop(sdr; segment = sdr.segment)
    @test_throws ArgumentError receive(loop, sdr.system, 4e6Hz; vector_tracking = true)
    @test_throws ArgumentError receive(loop, sdr.system, 5e6Hz)
    @test_throws ArgumentError receive(loop, GPSL5I(), 4e6Hz)
end

# A stand-in loop process: creates the segment, marks itself running and
# heartbeats until killed — enough to exercise spawning, attaching, the
# clean-slate release and the restart after a lost heartbeat.
const HEARTBEAT_STUB = """
using HardwareLoopProtocol
seg = create_segment(ARGS[1], SegmentConfig(; channel_count = 4, bands = [BandEntry("L1", 4e6)]))
set_loop_pid!(seg, getpid())
set_loop_state!(seg, HardwareLoopProtocol.LOOP_STATE_RUNNING)
while true
    loop_heartbeat!(seg)
    sleep(0.02)
end
"""

@testset "A loop process is spawned, attached to and restarted from its command" begin
    path = joinpath(mktempdir(), "gnss-loop-test")
    sdr = simulated_loop_sdr(; chunk = 4000)
    cmd = `$(Base.julia_cmd()) --startup-file=no --project=$(Base.active_project()) -e $HEARTBEAT_STUB $path`
    loop = RemoteHardwareLoop(sdr; segment = path, spawn = cmd, heartbeat_timeout = 0.3, attach_timeout = 120.0)
    try
        @test !isnothing(loop.process) && process_running(loop.process)
        @test HardwareLoopProtocol.heartbeat_alive(loop.segment, :loop; stale_after_ns = 300_000_000)
        @test loop.num_channels == 4 && loop.band_ids == [:L1]
        # A second receiver attaching to the live loop releases every channel.
        other = RemoteHardwareLoop(sdr; segment = path)
        @test isnothing(other.process)
        @test other.releases_sent == 4
        close(other; shutdown = false)
        # The loop dies: the next chunk's liveness check respawns it.
        kill(loop.process)
        wait(loop.process)
        sleep(0.4)
        @test GNSSReceiver._check_loop!(loop)
        @test loop.restarts == 1
        @test process_running(loop.process)
        @test HardwareLoopProtocol.heartbeat_alive(loop.segment, :loop; stale_after_ns = 300_000_000)
    finally
        close(loop)
    end
    @test !process_running(loop.process)
end
