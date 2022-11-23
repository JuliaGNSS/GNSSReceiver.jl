@testset "Receive signal matrix of type $(type)" for type in [ComplexF64, ComplexF32, Complex{Int16}]

    sampling_freq = 5e6Hz
    gpsl1 = GPSL1()
    
    measurement_channel = Channel{Matrix{type}}() do ch
        if type <: Complex{Int16}
            foreach(i -> put!(ch, type.(round.(randn(ComplexF32, 20000, 4) * 512))), 1:20)
        else
            foreach(i -> put!(ch, randn(type, 20000, 4) * 512), 1:20)
        end
    end

    data_channel = receive(measurement_channel, gpsl1, sampling_freq, num_ants = NumAnts(4), num_samples = 20000) 

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 0
        @test isnothing(data.pvt.time)
    end

    sat_state = GNSSReceiver.SatelliteChannelState(
        TrackingState(1, gpsl1, 1200.0Hz, 123.0, num_ants = NumAnts(4)),
        GNSSDecoder.GPSL1DecoderState(1),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        0ms
    )

    rec_state = GNSSReceiver.ReceiverState(
        Dict(1 => sat_state),
        PositionVelocityTime.PVTSolution(),
        0ms
    )

    measurement_channel = Channel{Matrix{type}}() do ch
        if type <: Complex{Int16}
            foreach(i -> put!(ch, type.(round.(randn(ComplexF32, 20000, 4) * 512))), 1:20)
        else
            foreach(i -> put!(ch, randn(type, 20000, 4) * 512), 1:20)
        end
    end

    data_channel = receive(measurement_channel, gpsl1, sampling_freq, num_ants = NumAnts(4), receiver_state = rec_state, num_samples = 20000) 

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 1
        @test isnothing(data.pvt.time)
    end

end

@testset "Receive signal vector of type $(type)" for type in [ComplexF64, ComplexF32, Complex{Int16}]

    sampling_freq = 5e6Hz
    gpsl1 = GPSL1()
    
    measurement_channel = Channel{Vector{type}}() do ch
        if type <: Complex{Int16}
            foreach(i -> put!(ch, type.(round.(randn(ComplexF32, 20000) * 512))), 1:20)
        else
            foreach(i -> put!(ch, randn(type, 20000) * 512), 1:20)
        end
    end

    data_channel = receive(measurement_channel, gpsl1, sampling_freq, num_ants = NumAnts(1), num_samples = 20000) 

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 0
        @test isnothing(data.pvt.time)
    end

    sat_state = GNSSReceiver.SatelliteChannelState(
        TrackingState(1, gpsl1, 1200.0Hz, 123.0),
        GNSSDecoder.GPSL1DecoderState(1),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        0ms
    )

    rec_state = GNSSReceiver.ReceiverState(
        Dict(1 => sat_state),
        PositionVelocityTime.PVTSolution(),
        0ms
    )

    measurement_channel = Channel{Vector{type}}() do ch
        if type <: Complex{Int16}
            foreach(i -> put!(ch, type.(round.(randn(ComplexF32, 20000) * 512))), 1:20)
        else
            foreach(i -> put!(ch, randn(type, 20000) * 512), 1:20)
        end
    end

    data_channel = receive(measurement_channel, gpsl1, sampling_freq, num_ants = NumAnts(1), receiver_state = rec_state, num_samples = 20000) 

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 1
        @test isnothing(data.pvt.time)
    end

end