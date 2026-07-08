@testset "Process measurement with number of antennas $i" for i in [1, 4]
    measurement = randn(ComplexF64, 20000, i)
    system = GPSL1()
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(i),
    )
    sampling_freq = 5e6Hz

    acq_plan = plan_acquire(system, float(sampling_freq), collect(1:32))

    next_receiver_state = GNSSReceiver.process(
        receiver_state,
        acq_plan,
        measurement,
        system,
        sampling_freq;
        num_ants = NumAnts(i),
    )

    @test length(get_sat_states(next_receiver_state.track_state)) == 0

    receiver_sat_states = (Dictionary([1], [GNSSReceiver.ReceiverSatState(system, 1)]),)

    track_state =
        TrackState(system, [SatState(system, 1, 0.0, 20u"Hz"; num_ants = NumAnts(i))])

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

    next_receiver_state = GNSSReceiver.process(
        receiver_state,
        acq_plan,
        measurement,
        system,
        sampling_freq;
        num_ants = NumAnts(i),
    )

    @test length(get_sat_states(next_receiver_state.track_state)) == 1
end