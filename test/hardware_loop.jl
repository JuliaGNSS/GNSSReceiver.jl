# The hardware-correlator path end to end, without hardware: HardwareLoopCore's
# `SimulatedDevice` correlates the ION recording, its `LoopCore` closes the loops
# and publishes into a file-backed HardwareLoopProtocol segment, and
# `receive(::HardwareLoop, …)` acquires, decodes and solves the PVT from what
# the segment carries — the same two halves, and the same wire, a board uses.
#
# Note: included from runtests.jl after the ION integration test, which downloads
# the recording into the same scratch space.

using HardwareLoopCore
using HardwareLoopProtocol: create_segment, SegmentConfig, loop_heartbeat!
import HardwareLoopProtocol as HLP

# A segment whose loop side has just beaten, and the receiver's handle on it: for
# the tests of the receiver's own bookkeeping, which command a loop that never
# answers unless the test answers for it.
function with_test_loop(f; num_channels = 4, fs = 2.048e6)
    path = joinpath(mktempdir(), "gnss-loop-unit")
    dev = SimulatedDevice(GPSL1CA(); sampling_freq = fs, num_channels)
    seg = create_segment(path, SegmentConfig(;
        channel_count = num_channels,
        bands = driver_capabilities(dev).bands,
        event_capacity = 1 << 10,
    ))
    loop_heartbeat!(seg)
    loop = HardwareLoop(path)
    try
        f(loop, seg)
    finally
        close(loop)
        close(seg)
    end
end

# An acquisition result the receiver takes as a detection.
detected(system, prn; fs = 2.048e6Hz) = Acquisition.AcquisitionResults(
    system, prn, fs, 1000.0Hz, 100.0, nothing, 45.0, 1.0, 1e6, 1, nothing, -5000.0Hz:100.0Hz:5000.0Hz, 1, 2048, 1)

@testset "hardware loop: the ION recording through a simulated correlator" begin
    dat_file = joinpath(@get_scratch!("rtl_sdr_test_data"), "RTLSDR_Bands-L1.uint8")
    if !isfile(dat_file)
        @warn "ION recording not available; skipping the hardware-loop integration test"
        return
    end
    fs = 2.048e6
    chunk = 8192                       # 4 ms
    seconds = 45
    num_channels = 12
    path = joinpath(mktempdir(), "gnss-loop-test")
    dev = SimulatedDevice(GPSL1CA(); sampling_freq = fs, num_channels)
    seg = create_segment(path, SegmentConfig(;
        channel_count = num_channels,
        bands = driver_capabilities(dev).bands,
        event_capacity = 1 << 16,
    ))
    core = LoopCore(dev, (GPSL1CA(),), seg)
    loop_heartbeat!(seg)
    loop = HardwareLoop(path)
    @test loop.num_channels == num_channels

    raw = GNSSReceiver.SignalChannel{Complex{Int16},1}(chunk, 64)
    stop = Threads.Atomic{Bool}(false)
    producer = Threads.@spawn begin
        io = open(dat_file)
        bytes = Vector{UInt8}(undef, 2chunk)
        samples = Vector{ComplexF64}(undef, chunk)
        try
            for _ = 1:round(Int, seconds * fs / chunk)
                stop[] && break
                readbytes!(io, bytes) < 2chunk && break
                frame = Matrix{Complex{Int16}}(undef, chunk, 1)
                @inbounds for i = 1:chunk
                    frame[i, 1] = complex(Int16(bytes[2i-1]) - Int16(128), Int16(bytes[2i]) - Int16(128))
                    samples[i] = frame[i, 1]
                end
                # A search in flight takes wall time the recording does not:
                # on a board the samples arrive in real time and an arm hands
                # over from a window at most a search old. Hold the recording
                # while one runs, so the test does not depend on how fast this
                # machine searches.
                while (task = loop.acquisition; !isnothing(task) && !istaskdone(task))
                    sleep(0.001)
                end
                # The device first, then the receiver: like a board, whose
                # correlator sees a sample before the host's raw stream does.
                correlate_chunk!(dev, samples)
                service_pass!(core; wait_ms = 0)
                put!(raw, frame)
            end
        finally
            close(io)
            close(raw)
        end
    end
    data = receive(loop, raw, GPSL1CA(), fs * Hz; device_sample_origin = 0, pvt_approximate_year = 2017,
                   replica_amplitude = 1.0)
    fixes = PVTSolution[]
    tracked = Set{Int}()
    first_fix = nothing
    for d in data
        foreach(((_, prn),) -> push!(tracked, prn), keys(d.sat_data))
        if !isnothing(d.pvt.time)
            push!(fixes, d.pvt)
            isnothing(first_fix) && (first_fix = d.runtime)
        end
    end
    stop[] = true
    wait(producer)
    close(loop)
    # `close` waited for the search in flight, so none outlives the loop.
    @test isnothing(loop.acquisition) || istaskdone(loop.acquisition)
    close(seg)

    @info "hardware loop" tracked = sort(collect(tracked)) fixes = length(fixes) first_fix commands = core.commands_handled lost = loop.lost_events
    @test length(tracked) >= 8
    @test !isempty(fixes)
    # The software receiver's fix on this recording (ion_rtlsdr_integration.jl).
    expected = [3.9074087926e6, 3.0683836901e5, 5.0149608655e6]
    final = last(fixes)
    @test norm([final.position.x, final.position.y, final.position.z] - expected) < 15
    @test length(final.sats) >= 6
    @test loop.lost_events == 0
end

@testset "hardware loop: bits are decoded in order across a break" begin
    system = GPSL1CA()
    sat = GNSSReceiver.HardwareSat(system, system, 7, 1, UInt64(1), 30.0dBHz, 0.0u"s")
    for k = 1:10
        push!(sat.bit_samples, 1000k)
        push!(sat.bit_values, isodd(k) ? 1.0f0 : -1.0f0)
    end
    GNSSReceiver.push_bit_break!(sat, 10_000)
    for k = 11:15
        push!(sat.bit_samples, 1000k)
        push!(sat.bit_values, 1.0f0)
    end
    # Up to the break and three bits past it: the rest stays queued.
    GNSSReceiver.decode_until!(sat, 13_000)
    @test sat.bit_samples == [14_000, 15_000]
    @test all(!isnan, sat.bit_values)
    # The decoder saw no sync in this noise, and starts over at the break.
    @test isnothing(sat.state.decoder.num_bits_after_valid_syncro_sequence)
end

@testset "hardware loop: a burst of records is judged record by record" begin
    system = GPSL1CA()
    fs = 4e6u"Hz"
    linear(cn0_dBHz) = 10^(cn0_dBHz / 10)
    prompt = complex(1.0, 0.1)                    # phase-locked
    function settled_sat()
        sat = GNSSReceiver.HardwareSat(system, system, 7, 1, UInt64(1), 30.0dBHz, 0.0u"s")
        sat.confirmed = true
        # 3 s of 1 ms records at 33 dB-Hz, one lock update each: past the pull-in
        # stage, so 200 ms out of lock loses the satellite.
        for _ = 1:3000
            GNSSReceiver.push_record!(sat, prompt, 4000, linear(33))
            GNSSReceiver.update_lock!(sat, fs)
        end
        @test GNSSReceiver.has_pulled_in(sat.state.code_lock_detector)
        sat
    end
    # 300 ms of records, as a chunk drains them after this process was held up:
    # all at 33 dB-Hz but the last, at 29.
    burst = [(prompt, 4000, linear(k == 300 ? 29 : 33)) for k = 1:300]

    batched = settled_sat()
    foreach(r -> GNSSReceiver.push_record!(batched, r...), burst)
    GNSSReceiver.update_lock!(batched, fs)
    # One weak record is 1 ms out of lock, not the burst's 300 ms.
    @test GNSSReceiver.is_in_lock(batched.state)

    one_by_one = settled_sat()
    for r in burst
        GNSSReceiver.push_record!(one_by_one, r...)
        GNSSReceiver.update_lock!(one_by_one, fs)
    end
    @test batched.state.code_lock_detector === one_by_one.state.code_lock_detector
    @test batched.state.carrier_lock_detector === one_by_one.state.carrier_lock_detector
    @test batched.state.time_in_lock ≈ one_by_one.state.time_in_lock
    @test isempty(batched.prompts) && isempty(batched.record_cn0s) && isempty(batched.record_samples)

    # A record without a C/N₀ (the noise reference not measured yet) carries the
    # latest one rather than reading as no signal.
    GNSSReceiver.push_record!(batched, prompt, 4000, nothing)
    @test only(batched.record_cn0s) == linear(29)
end

@testset "hardware loop: a satellite acquired again keeps what it decoded" begin
    with_test_loop() do loop, _
        system = GPSL1CA()
        key = GNSSReceiver.signal_group_key(system)
        fs = 2.048e6
        state = GNSSReceiver.HardwareReceiverState((system,), NamedTuple(), nothing, loop.num_channels)
        arm!(prn) = GNSSReceiver._arm_acquired!(state, loop, ([detected(system, prn)],), 0, fs;
                                               rf_input = 1, replica_amplitude = 1.0, code_lock_cn0_threshold = nothing)
        arm!(7)
        ch = findfirst(s -> !isnothing(s) && s.prn == 7, state.channels)
        sat = state.channels[ch]
        # Its ephemeris decoded, and the time of week.
        decoder = sat.state.decoder
        decoder = GNSSDecoderState(decoder;
            raw_data = GNSSDecoder.GPSL1CAData(decoder.raw_data; WN = 2391, sqrt_A = 5153.6, TOW = 345_600))
        st = sat.state
        sat.state = @set st.decoder = decoder

        GNSSReceiver._free_channel!(state, loop, ch)
        @test all(isnothing, state.channels)
        @test state.releases == 1
        @test haskey(state.decoders, (key, 7))

        arm!(7)
        again = state.channels[findfirst(!isnothing, state.channels)]
        @test again.prn == 7
        @test again.state.decoder.raw_data.WN == 2391
        @test again.state.decoder.raw_data.sqrt_A == 5153.6
        # Only the sync starts over: the time of week comes with the next subframe.
        @test isnothing(again.state.decoder.raw_data.TOW)
        @test isnothing(again.state.decoder.num_bits_after_valid_syncro_sequence)
        @test isempty(state.decoders)

        # A satellite never tracked before starts from scratch.
        arm!(9)
        fresh = state.channels[findfirst(s -> !isnothing(s) && s.prn == 9, state.channels)]
        @test isnothing(fresh.state.decoder.raw_data.WN)
    end
end

@testset "hardware loop: a noise reference the loop rejects later is armed again" begin
    with_test_loop() do loop, seg
        system = GPSL1CA()
        state = GNSSReceiver.HardwareReceiverState((system,), NamedTuple(), nothing, loop.num_channels)
        arm!() = GNSSReceiver._arm_noise_reference!(state, loop, 0, 2.048e6;
                                                   rf_input = 1, replica_amplitude = 1.0, noise_prn = 32)
        ch = loop.num_channels
        answer!(code, reason, sequence) = HLP.publish!(HLP.event_ring(seg, ch), HLP.EventTag(HLP.EVENT_STATUS, ch, 0),
                                                       HLP.StatusEvent(code, reason, 0, sequence))
        arm!()
        armed = state.noise_sequence
        @test state.noise_channel == ch && armed != 0

        answer!(HLP.STATUS_ARMED, HLP.REJECT_NONE, armed)
        GNSSReceiver._drain!(state, loop)
        @test state.noise_confirmed
        arm!()
        @test state.noise_sequence == armed            # confirmed: nothing to do

        # The loop re-arms the reference onto fresh decoys by itself, under the
        # receiver's sequence, and answers one the device could not commit with a
        # rejection, freeing the channel.
        answer!(HLP.STATUS_ARM_REJECTED, HLP.REJECT_DEVICE_ERROR, armed)
        Base.CoreLogging.with_logger(Base.CoreLogging.NullLogger()) do
            GNSSReceiver._drain!(state, loop)
        end
        @test !state.noise_confirmed
        arm!()
        @test state.noise_sequence > armed
    end
end

@testset "hardware loop: an output the consumers are behind on is dropped" begin
    ch = Channel{Int}(2)
    @test GNSSReceiver._offer!(ch, 1) && GNSSReceiver._offer!(ch, 2)
    @test !GNSSReceiver._offer!(ch, 3)             # full: dropped, not blocked
    @test take!(ch) == 1 && GNSSReceiver._offer!(ch, 4)
    @test collect(Iterators.take(ch, 2)) == [2, 4]
    close(ch)
    @test_throws InvalidStateException GNSSReceiver._offer!(ch, 5)
end
