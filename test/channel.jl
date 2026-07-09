using GNSSReceiver:
    PipeChannel,
    SignalChannel,
    FixedSizeMatrixDefault,
    consume_channel,
    tee,
    rechunk,
    membuffer,
    spawn_signal_channel_thread,
    num_antenna_channels,
    write_to_file,
    read_from_file

@testset "PipeChannel scalar put!/take!/close" begin
    ch = PipeChannel{Int}(4)
    put!(ch, 1)
    put!(ch, 2)
    @test isready(ch)
    @test take!(ch) == 1
    @test take!(ch) == 2
    @test isempty(ch)
    close(ch)
    @test !isopen(ch)
    @test_throws InvalidStateException take!(ch)
end

@testset "PipeChannel batch put!/take! and iteration" begin
    ch = PipeChannel{Int}(8)
    task = Threads.@spawn begin
        # more items than capacity forces the batch put! to wave
        put!(ch, collect(1:50))
        close(ch)
    end
    # batch take of a fixed count
    first_ten = take!(ch, 10)
    @test first_ten == collect(1:10)
    # drain the rest by iteration
    rest = Int[]
    for x in ch
        push!(rest, x)
    end
    wait(task)
    @test rest == collect(11:50)
end

@testset "SignalChannel dimensions and validation" begin
    ch = SignalChannel{ComplexF32,2}(8)
    @test num_antenna_channels(ch) == 2
    @test eltype(typeof(ch)) == FixedSizeMatrixDefault{ComplexF32}
    buf = FixedSizeMatrixDefault{ComplexF32}(undef, 8, 2)
    fill!(buf, ComplexF32(1))
    put!(ch, buf)
    @test size(take!(ch)) == (8, 2)
    # plain Matrix rejected for performance
    @test_throws ArgumentError put!(ch, ones(ComplexF32, 8, 2))
    # wrong dimensions rejected
    @test_throws ArgumentError put!(ch, FixedSizeMatrixDefault{ComplexF32}(undef, 4, 2))
    close(ch)
end

@testset "spawn_signal_channel_thread + consume_channel" begin
    ch = spawn_signal_channel_thread(;
        T = ComplexF32,
        num_samples = 4,
        num_antenna_channels = 2,
    ) do out
        for i = 1:5
            b = FixedSizeMatrixDefault{ComplexF32}(undef, 4, 2)
            fill!(b, ComplexF32(i))
            put!(out, b)
        end
    end
    n = 0
    consume_channel(ch) do data
        n += 1
        @test all(==(ComplexF32(n)), data)
    end
    @test n == 5
end

@testset "rechunk merges and splits chunks" begin
    # 5 input chunks of 3 samples -> rechunk to 5 -> three 5-sample chunks (15 total)
    src = spawn_signal_channel_thread(; T = Float32, num_samples = 3) do out
        for i = 1:5
            b = FixedSizeMatrixDefault{Float32}(undef, 3, 1)
            b .= Float32.((i - 1) * 3 .+ (1:3))
            put!(out, b)
        end
    end
    out = rechunk(src, 5)
    @test out.num_samples == 5
    collected = Float32[]
    consume_channel(out) do data
        @test size(data) == (5, 1)
        append!(collected, vec(collect(data)))
    end
    @test collected == Float32.(1:15)
end

@testset "tee duplicates and membuffer forwards" begin
    src = spawn_signal_channel_thread(; T = Float32, num_samples = 2) do out
        for i = 1:4
            b = FixedSizeMatrixDefault{Float32}(undef, 2, 1)
            fill!(b, Float32(i))
            put!(out, b)
        end
    end
    buffered = membuffer(src, 8)
    o1, o2 = tee(buffered)
    c1 = Float32[]
    c2 = Float32[]
    t = Threads.@spawn consume_channel(d -> append!(c2, vec(collect(d))), o2)
    consume_channel(d -> append!(c1, vec(collect(d))), o1)
    wait(t)
    @test c1 == Float32[1, 1, 2, 2, 3, 3, 4, 4]
    @test c2 == c1
end

@testset "write_to_file / read_from_file roundtrip" begin
    dir = mktempdir()
    path = joinpath(dir, "sig")
    src = spawn_signal_channel_thread(;
        T = ComplexF32,
        num_samples = 4,
        num_antenna_channels = 2,
    ) do out
        for i = 1:3
            b = FixedSizeMatrixDefault{ComplexF32}(undef, 4, 2)
            b .= ComplexF32(i)
            put!(out, b)
        end
    end
    wait(write_to_file(src, path))
    rd = read_from_file(path, 4, 2; T = ComplexF32)
    n = 0
    consume_channel(rd) do data
        n += 1
        @test all(==(ComplexF32(n)), data)
    end
    @test n == 3
end
