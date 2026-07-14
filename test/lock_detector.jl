@testset "CodeLockDetector accumulates and pays back out-of-lock time" begin
    # Warm up past the wait-time threshold with a healthy CN0 so the detector
    # starts arming its out-of-lock timer.
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        out_of_lock_time_threshold = 200u"ms",
        wait_time_threshold = 80u"ms",
    )
    for _ = 1:20
        detector = GNSSReceiver.update(detector, 45u"dBHz", 4u"ms")
    end
    @test GNSSReceiver.is_in_lock(detector)
    @test detector.out_of_lock_time == 0u"s"

    # CN0 below threshold accumulates out-of-lock time until lock is declared lost.
    for _ = 1:60
        detector = GNSSReceiver.update(detector, 10u"dBHz", 4u"ms")
    end
    @test !GNSSReceiver.is_in_lock(detector)
    @test detector.out_of_lock_time >= 200u"ms"

    # A healthy CN0 again pays the accumulated out-of-lock time back down.
    accumulated = detector.out_of_lock_time
    detector = GNSSReceiver.update(detector, 45u"dBHz", 4u"ms")
    @test detector.out_of_lock_time < accumulated
    @test detector.out_of_lock_time == accumulated - 4u"ms"
end

@testset "CodeLockDetector stays neutral before the wait time elapses" begin
    # Before `wait_time_threshold` is reached a bad CN0 must not accumulate any
    # out-of-lock time (the detector is still warming up).
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        wait_time_threshold = 80u"ms",
    )
    detector = GNSSReceiver.update(detector, 5u"dBHz", 4u"ms")
    @test detector.out_of_lock_time == 0u"s"
    @test GNSSReceiver.is_in_lock(detector)
end

@testset "CarrierLockDetector accumulates out-of-lock on weak in-phase power" begin
    detector = GNSSReceiver.CarrierLockDetector(;
        out_of_lock_time_threshold = 200u"ms",
        wait_time_threshold = 80u"ms",
        integration_time_threshold = 80u"ms",
    )
    # A prompt dominated by its quadrature component fails the in-phase dominance
    # test each integration block, so out-of-lock time accumulates and lock is lost.
    weak_inphase = complex(0.1, 10.0)
    for _ = 1:80
        detector = GNSSReceiver.update(detector, weak_inphase, 4u"ms")
    end
    @test detector.out_of_lock_time > 0u"s"
    @test !GNSSReceiver.is_in_lock(detector)

    # A prompt dominated by its in-phase component resets the accumulated time.
    strong_inphase = complex(10.0, 0.1)
    for _ = 1:40
        detector = GNSSReceiver.update(detector, strong_inphase, 4u"ms")
    end
    @test detector.out_of_lock_time == 0u"s"
    @test GNSSReceiver.is_in_lock(detector)
end
