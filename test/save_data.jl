@testset "Save data" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{4,ComplexF64}}
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{sat_data_type}}() do ch
        foreach(1:20) do i
            data = GNSSReceiver.ReceiverDataOfInterest{sat_data_type}(
                Dict{Int,Vector{sat_data_type}}(),
                GNSSReceiver.PVTSolution(),
                (i - 1) * 1ms,
            )
            put!(ch, data)
        end
    end

    @sync begin
        save_data(data_channel; filename = "/tmp/data.jld2")
    end
    #    sleep(1.0)
    #    data_over_time = load("/tmp/data.jld2", "data_over_time")

    #    @test length(data_over_time) == 20
    #    @test length(last(data_over_time).sat_data) == 0
end