@testset "Receive signal matrix of type $(type)" for type in [
    ComplexF64,
    ComplexF32,
    Complex{Int16},
]
    sampling_freq = 5e6Hz
    system = GPSL1()
    num_samples = 20000
    num_ants = 4

    measurement_channel = GNSSReceiver.MatrixSizedChannel{type}(num_samples, num_ants) do ch
        if type <: Complex{Int16}
            foreach(
                i -> put!(
                    ch,
                    type.(round.(randn(ComplexF32, num_samples, num_ants) * 512)),
                ),
                1:20,
            )
        else
            foreach(i -> put!(ch, randn(type, num_samples, num_ants) * 512), 1:20)
        end
    end

    data_channel =
        receive(measurement_channel, system, sampling_freq; num_ants = NumAnts(num_ants))

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 0
        @test isnothing(data.pvt.time)
    end

    receiver_sat_states = (Dictionary([1], [GNSSReceiver.ReceiverSatState(system, 1)]),)

    track_state =
        TrackState(system, [SatState(system, 1, 0.0, 20u"Hz"; num_ants = NumAnts(4))])

    acquisition_buffer = GNSSReceiver.SampleBuffer(ComplexF64, 20000)

    pvt = PositionVelocityTime.PVTSolution()

    receiver_state = ReceiverState(
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        pvt,
        0.0u"s",
        -Inf * 1.0u"s",
        -Inf * 1.0u"s",
        0,
    )

    measurement_channel = GNSSReceiver.MatrixSizedChannel{type}(num_samples, num_ants) do ch
        if type <: Complex{Int16}
            foreach(
                i -> put!(
                    ch,
                    type.(round.(randn(ComplexF32, num_samples, num_ants) * 512)),
                ),
                1:20,
            )
        else
            foreach(i -> put!(ch, randn(type, num_samples, num_ants) * 512), 1:20)
        end
    end

    data_channel = receive(
        measurement_channel,
        system,
        sampling_freq;
        num_ants = NumAnts(num_ants),
        receiver_state,
    )

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 1
        @test isnothing(data.pvt.time)
    end
end