@testset "Save data" begin
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{4}}() do ch
        foreach(1:20) do i
            data = GNSSReceiver.ReceiverDataOfInterest{4}(
                Dict{Int, Vector{GNSSReceiver.SatelliteDataOfInterest{4}}}(),
                GNSSReceiver.PVTSolution(),
                (i - 1) * 1ms
            )
            put!(ch, data)
        end
    end

    @sync begin
        save_data(data_channel, filename = "./data.jld2")
    end
    data_over_time = load("./data.jld2", "data_over_time")

    @test length(data_over_time) == 20
    @test length(last(data_over_time).sat_data) == 0
end