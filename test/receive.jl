@testset "Receive signal matrix of type $(type)" for type in [ComplexF64, ComplexF32, Complex{Int16}]

    sampling_freq = 5e6Hz
    gpsl1 = GPSL1()
    
    measurement_channel = Channel{Matrix{type}}() do ch
        foreach(i -> put!(ch, rand(type, 20000, 4)), 1:20)
    end

    data_channel = receive(measurement_channel, gpsl1, sampling_freq, num_ants = NumAnts(4)) 

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 0
        @test isnothing(data.pvt.time)
    end

end

@testset "Receive signal vector of type $(type)" for type in [ComplexF64, ComplexF32, Complex{Int16}]

    sampling_freq = 5e6Hz
    gpsl1 = GPSL1()
    
    measurement_channel = Channel{Vector{type}}() do ch
        foreach(i -> put!(ch, rand(type, 20000)), 1:20)
    end

    data_channel = receive(measurement_channel, gpsl1, sampling_freq, num_ants = NumAnts(1)) 

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 0
        @test isnothing(data.pvt.time)
    end

end