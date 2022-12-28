@testset "Process measurement" begin
    measurement = randn(ComplexF64, 20000, 4)
    system = GPSL1()
    receiver_state = GNSSReceiver.ReceiverState(ComplexF64, size(measurement, 1), 4ms, system, NumAnts(4))
    sampling_freq = 5e6Hz

    acq_plan = CoarseFineAcquisitionPlan(system, size(measurement, 1), sampling_freq)
    coarse_step = 1 / 3 / (size(measurement, 1) / sampling_freq)
    fine_step = 1 / 12 / (size(measurement, 1) / sampling_freq)
    fine_doppler_range = -2*coarse_step:fine_step:2*coarse_step
    fast_re_acq_plan = AcquisitionPlan(
        system,
        size(measurement, 1),
        sampling_freq,
        dopplers = fine_doppler_range
    )

    next_receiver_state, track_results =
        GNSSReceiver.process(receiver_state, acq_plan, fast_re_acq_plan, measurement, system, sampling_freq)

    @test length(track_results) == 0

    sat_state = GNSSReceiver.SatelliteChannelState(
        TrackingState(1, system, 1202.0Hz, 123.2; num_ants = NumAnts(4)),
        GNSSReceiver.GNSSDecoderState(system, 1),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        0.0u"s",
        0,
    )

    acq_buffer = GNSSReceiver.AcquisitionBuffer(ComplexF64, size(measurement, 1), 4)
    receiver_state =
        GNSSReceiver.ReceiverState(Dict(1 => sat_state), GNSSReceiver.PVTSolution(), 0.0u"s", 0, acq_buffer)

    next_receiver_state, track_results =
        GNSSReceiver.process(receiver_state, acq_plan, fast_re_acq_plan, measurement, system, sampling_freq)

    @test length(track_results) == 1
end