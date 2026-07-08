@testset "Save data" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{4,ComplexF64}}
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{sat_data_type}}() do ch
        foreach(1:20) do i
            data = GNSSReceiver.ReceiverDataOfInterest{sat_data_type}(
                Dictionary{Tuple{Symbol,Int},sat_data_type}(),
                GNSSReceiver.PVTSolution(),
                (i - 1) * 1ms,
            )
            put!(ch, data)
        end
    end

    filename = joinpath(mktempdir(), "data.jld2")
    # `save_data` returns its spawned writer task; wait on it so the JLD2 file is
    # fully written before we read it back.
    wait(save_data(data_channel; filename))

    data_over_time = load(filename, "data_over_time")
    @test length(data_over_time) == 20
    @test length(last(data_over_time).sat_data) == 0
    @test data_over_time[1].runtime == 0ms
    @test data_over_time[end].runtime == 19ms
end

@testset "Save data with a custom extract payload" begin
    # A channel of a custom (non-ReceiverDataOfInterest) payload, as produced via
    # `receive`'s `extract` keyword. `save_data` must handle it just like `collect_data`.
    data_channel = Channel{@NamedTuple{runtime::typeof(1.0ms)}}() do ch
        foreach(i -> put!(ch, (; runtime = (i - 1) * 1.0ms)), 1:20)
    end

    filename = joinpath(mktempdir(), "custom.jld2")
    wait(save_data(data_channel; filename))

    data_over_time = load(filename, "data_over_time")
    @test length(data_over_time) == 20
    @test data_over_time[end].runtime == 19ms
end

@testset "Collect data" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{4,ComplexF64}}
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{sat_data_type}}() do ch
        foreach(1:20) do i
            put!(
                ch,
                GNSSReceiver.ReceiverDataOfInterest{sat_data_type}(
                    Dictionary{Tuple{Symbol,Int},sat_data_type}(),
                    GNSSReceiver.PVTSolution(),
                    (i - 1) * 1ms,
                ),
            )
        end
    end

    data = collect_data(data_channel)
    @test length(data) == 20
    @test data[1].runtime == 0ms
    @test data[end].runtime == 19ms
end