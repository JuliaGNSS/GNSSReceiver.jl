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
