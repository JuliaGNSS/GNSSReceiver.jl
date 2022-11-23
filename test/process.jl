@testset "Process measurement" begin
    receiver_state = GNSSReceiver.ReceiverState()
    measurement = randn(ComplexF64, 20000, 4)
    system = GPSL1()
    sampling_freq = 5e6Hz

    acq_plan = CoarseFineAcquisitionPlan(system, size(measurement, 1), sampling_freq)

    next_receiver_state, track_results =
        GNSSReceiver.process(receiver_state, acq_plan, measurement, system, sampling_freq)

    @test length(track_results) == 0

    sat_state = GNSSReceiver.SatelliteChannelState(
        TrackingState(1, system, 1202.0Hz, 123.2; num_ants = NumAnts(4)),
        GNSSReceiver.GNSSDecoderState(system, 1),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        0ms,
    )

    receiver_state =
        GNSSReceiver.ReceiverState(Dict(1 => sat_state), GNSSReceiver.PVTSolution(), 0ms)

    next_receiver_state, track_results =
        GNSSReceiver.process(receiver_state, acq_plan, measurement, system, sampling_freq)

    @test length(track_results) == 1
end