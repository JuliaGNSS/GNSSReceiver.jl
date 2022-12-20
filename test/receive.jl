@testset "Receive signal matrix of type $(type)" for type in [
    ComplexF64,
    ComplexF32,
    Complex{Int16},
]
    sampling_freq = 5e6Hz
    gpsl1 = GPSL1()
    num_samples = 5000
    num_ants = 4

    measurement_channel = GNSSReceiver.MatrixSizedChannel{type}(num_samples, num_ants) do ch
        if type <: Complex{Int16}
            foreach(i -> put!(ch, type.(round.(randn(ComplexF32, num_samples, num_ants) * 512))), 1:20)
        else
            foreach(i -> put!(ch, randn(type, num_samples, num_ants) * 512), 1:20)
        end
    end

    data_channel = receive(
        measurement_channel,
        gpsl1,
        sampling_freq;
        num_ants = NumAnts(num_ants),
    )

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 0
        @test isnothing(data.pvt.time)
    end

    sat_state = GNSSReceiver.SatelliteChannelState(
        TrackingState(1, gpsl1, 1200.0Hz, 123.0; num_ants = NumAnts(4)),
        GNSSDecoder.GPSL1DecoderState(1),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        0.0u"s",
        0,
    )

    acq_buffer = GNSSReceiver.AcquisitionBuffer(type, num_samples, num_ants)
    rec_state = GNSSReceiver.ReceiverState(
        Dict(1 => sat_state),
        PositionVelocityTime.PVTSolution(),
        0.0u"s",
        0,
        acq_buffer
    )

    measurement_channel = GNSSReceiver.MatrixSizedChannel{type}(num_samples, num_ants) do ch
        if type <: Complex{Int16}
            foreach(i -> put!(ch, type.(round.(randn(ComplexF32, num_samples, num_ants) * 512))), 1:20)
        else
            foreach(i -> put!(ch, randn(type, num_samples, num_ants) * 512), 1:20)
        end
    end

    data_channel = receive(
        measurement_channel,
        gpsl1,
        sampling_freq;
        num_ants = NumAnts(num_ants),
        receiver_state = rec_state,
    )

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 1
        @test isnothing(data.pvt.time)
    end
end