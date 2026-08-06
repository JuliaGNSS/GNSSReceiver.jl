# One primary code period: the reference integration time the CN0 threshold is anchored
# to, so a record of this length is credited exactly `cn0_threshold` (no relaxation).
const REFERENCE_T = 1u"ms"

@testset "CodeLockDetector accumulates and pays back out-of-lock time" begin
    # Warm up past the wait-time threshold with a healthy CN0 so the detector
    # starts arming its out-of-lock timer.
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        reference_integration_time = REFERENCE_T,
        out_of_lock_time_threshold = 200u"ms",
        wait_time_threshold = 80u"ms",
    )
    for _ = 1:20
        detector = GNSSReceiver.update(detector, 45u"dBHz", REFERENCE_T, 4u"ms")
    end
    @test GNSSReceiver.is_in_lock(detector)
    @test detector.out_of_lock_time == 0u"s"

    # CN0 below threshold accumulates out-of-lock time until lock is declared lost.
    for _ = 1:60
        detector = GNSSReceiver.update(detector, 10u"dBHz", REFERENCE_T, 4u"ms")
    end
    @test !GNSSReceiver.is_in_lock(detector)
    @test detector.out_of_lock_time >= 200u"ms"

    # A healthy CN0 again pays the accumulated out-of-lock time back down.
    accumulated = detector.out_of_lock_time
    detector = GNSSReceiver.update(detector, 45u"dBHz", REFERENCE_T, 4u"ms")
    @test detector.out_of_lock_time < accumulated
    @test detector.out_of_lock_time == accumulated - 4u"ms"
end

@testset "CodeLockDetector caps the out-of-lock dwell so recovery is bounded" begin
    # The dwell is paid back at the rate it is spent, so an uncapped accumulation would make
    # re-declaring lock take as long as the outage before it — minutes for a vector-tracking
    # member carried through a tunnel, which is exactly the case the bridging is for.
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        reference_integration_time = REFERENCE_T,
        out_of_lock_time_threshold = 200u"ms",
        wait_time_threshold = 80u"ms",
    )
    for _ = 1:20
        detector = GNSSReceiver.update(detector, 45u"dBHz", REFERENCE_T, 4u"ms")
    end
    # A 60 s outage at 4 ms per update.
    for _ = 1:15_000
        detector = GNSSReceiver.update(detector, 10u"dBHz", REFERENCE_T, 4u"ms")
    end
    @test !GNSSReceiver.is_in_lock(detector)
    @test detector.out_of_lock_time ==
          GNSSReceiver.OUT_OF_LOCK_TIME_CAP_FACTOR * detector.out_of_lock_time_threshold

    # One threshold's worth of good signal therefore brings it back, not sixty seconds':
    # 51 updates of 4 ms pay 204 ms off the 400 ms cap, crossing back under the threshold.
    for _ = 1:51
        detector = GNSSReceiver.update(detector, 45u"dBHz", REFERENCE_T, 4u"ms")
    end
    @test GNSSReceiver.is_in_lock(detector)
    @test detector.out_of_lock_time < detector.out_of_lock_time_threshold
end

@testset "CodeLockDetector stays neutral before the wait time elapses" begin
    # Before `wait_time_threshold` is reached a bad CN0 must not accumulate any
    # out-of-lock time (the detector is still warming up).
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        reference_integration_time = REFERENCE_T,
        wait_time_threshold = 80u"ms",
    )
    detector = GNSSReceiver.update(detector, 5u"dBHz", REFERENCE_T, 4u"ms")
    @test detector.out_of_lock_time == 0u"s"
    @test GNSSReceiver.is_in_lock(detector)
end

@testset "CodeLockDetector credits the record's integration time" begin
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        reference_integration_time = REFERENCE_T,
    )
    # The test is on `CN0 · T`, so an N-block record clears the detector at
    # 10·log10(N) dB less CN0 — 3.01 dB at the two-block records flown on sky.
    below = 30u"dBHz" - 2u"dB"
    @test GNSSReceiver.is_below_cn0_threshold(detector, below, REFERENCE_T)
    @test !GNSSReceiver.is_below_cn0_threshold(detector, below, 2 * REFERENCE_T)

    # The relaxation is 10·log10(N) and no more, to within 0.01 dB. Probed just off the
    # boundary rather than on it: the exact boundary lands one ulp low through the dB
    # round trip (250.0 Hz · 4 ms == 0.9999999999999998).
    boundary = 30u"dBHz" - 10 * log10(4) * u"dB"
    four_blocks = 4 * REFERENCE_T
    @test !GNSSReceiver.is_below_cn0_threshold(detector, boundary + 0.01u"dB", four_blocks)
    @test GNSSReceiver.is_below_cn0_threshold(detector, boundary - 0.01u"dB", four_blocks)

    # Tracking reports the integration time in `Hz^-1` (code blocks · chips / chip rate),
    # which must compare the same as the equivalent `s`.
    in_inverse_hz = uconvert(u"Hz^-1", REFERENCE_T)
    @test GNSSReceiver.is_below_cn0_threshold(detector, below, in_inverse_hz)
    @test !GNSSReceiver.is_below_cn0_threshold(detector, below, 2 * in_inverse_hz)
end

@testset "CodeLockDetector clamps the credit at the coherence limit" begin
    # With the credit capped at one code period, a longer record buys nothing.
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        reference_integration_time = REFERENCE_T,
        coherence_limit = REFERENCE_T,
    )
    below = 30u"dBHz" - 2u"dB"
    @test GNSSReceiver.is_below_cn0_threshold(detector, below, REFERENCE_T)
    @test GNSSReceiver.is_below_cn0_threshold(detector, below, 20 * REFERENCE_T)
end

@testset "CodeLockDetector treats a non-finite CN0 as out of lock" begin
    # The moments CN0 estimator degenerates to `NaN` for an all-zero prompt buffer (a
    # dead correlator channel) and to `Inf` when it sees no noise at all. `NaN` must
    # count as out of lock rather than hold lock forever through a failed comparison.
    # `-Inf dBHz` is the default NWPR estimator's "no detectable signal", so it is the
    # value a noise-only channel now actually reports.
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        reference_integration_time = REFERENCE_T,
    )
    nan_cn0 = uconvert(u"dBHz", NaN * u"Hz")
    inf_cn0 = uconvert(u"dBHz", Inf * u"Hz")
    minus_inf_cn0 = uconvert(u"dBHz", 0.0u"Hz")
    @test ustrip(minus_inf_cn0) == -Inf
    @test GNSSReceiver.is_below_cn0_threshold(detector, nan_cn0, REFERENCE_T)
    @test !GNSSReceiver.is_below_cn0_threshold(detector, inf_cn0, REFERENCE_T)
    @test GNSSReceiver.is_below_cn0_threshold(detector, minus_inf_cn0, REFERENCE_T)
    # Not even a record long enough to credit `10·log10(N)` dB rescues it.
    @test GNSSReceiver.is_below_cn0_threshold(detector, minus_inf_cn0, 20 * REFERENCE_T)
end

@testset "ReceiverSatState anchors the threshold to the ranging signal's code period" begin
    # The reference must come from the signal the detector actually runs on, not from a
    # hard-coded 1 ms: GPS L1 C/A's code period is 1 ms, Galileo E1B's is 4 ms.
    @test GNSSReceiver.primary_code_period(GPSL1CA()) ≈ 1u"ms"
    @test GNSSReceiver.primary_code_period(GalileoE1B()) ≈ 4u"ms"
    # Also defined for a pilot, which has no data rate to derive a symbol period from.
    @test GNSSReceiver.primary_code_period(GPSL5Q()) ≈ 1u"ms"

    state = GNSSReceiver.ReceiverSatState(GPSL1CA(), 1)
    @test state.code_lock_detector.reference_integration_time ==
          GNSSReceiver.primary_code_period(GPSL1CA())
end

# ---------------------------------------------------------------------------------------
# Synthetic-signal tests. The testsets above feed the detector fabricated CN0 numbers; the
# ones below drive it through Tracking's real CN0 estimators and the two accessors
# `process` actually calls (`estimate_cn0` and `get_last_fully_integrated_integration_time`),
# so the seam between Tracking's normalization and the detector's `CN0 · T` test is covered
# rather than assumed.
#
# Prompt model: a coherent prompt correlator output for a signal at carrier-to-noise density
# ratio `cn0` integrated over `T` is a constant signal amplitude in complex Gaussian noise.
# The estimators recover `SNR = A²/σ²`, and `randn(ComplexF64)` has unit total power, so
# `σ² = 1` and `A = √(CN0·T)`. The amplitude is constant — a perfectly phase-locked loop —
# which is what lets NWPR's coherent narrowband sum reach the truth here; in a real loop its
# residual phase noise costs a dB or two near threshold (see `CodeLockDetector`).
#
# Two estimators are exercised: Tracking's per-signal default (`NWPRCN0Estimator` as of
# Tracking 6 — what the receiver actually runs on), and the `MomentsCN0Estimator` that used
# to be the default. Fed the identical prompt stream, so anything that holds for both is a
# property of the detector rather than of one estimator's normalization. Passed as a
# zero-argument factory because both buffer into a mutable array that must not be shared
# between realizations.
# ---------------------------------------------------------------------------------------

const NUM_PROMPTS = 100
default_estimator() = Tracking.default_cn0_estimator(GPSL1CA(), NUM_PROMPTS)
moments_estimator() = MomentsCN0Estimator(NUM_PROMPTS)

function synthetic_tracked_signal(
    cn0,
    num_code_blocks,
    seed;
    num_prompts = NUM_PROMPTS,
    make_estimator = moments_estimator,
)
    signal = GPSL1CA()
    base = TrackedSignal(signal; num_prompts_for_cn0_estimation = num_prompts)
    integration_time = num_code_blocks * GNSSReceiver.primary_code_period(signal)
    amplitude = sqrt(ustrip(uconvert(NoUnits, Unitful.linear(cn0) * integration_time)))
    rng = Random.Xoshiro(seed)
    estimator = make_estimator()
    for _ = 1:num_prompts
        estimator = Tracking.update(estimator, amplitude + randn(rng, ComplexF64))
    end
    TrackedSignal(
        base;
        cn0_estimator = estimator,
        last_fully_integrated_num_code_blocks = num_code_blocks,
    )
end

# The decision `process` makes, read off a tracked signal exactly as `process` reads it.
function synthetic_is_below(detector, tracked_signal)
    GNSSReceiver.is_below_cn0_threshold(
        detector,
        estimate_cn0(tracked_signal),
        get_last_fully_integrated_integration_time(tracked_signal),
    )
end

synthetic_detector() = GNSSReceiver.CodeLockDetector(;
    cn0_threshold = 30u"dBHz",
    reference_integration_time = GNSSReceiver.primary_code_period(GPSL1CA()),
)

# Fraction of independent noise realizations the detector would call in-lock. Both
# estimators have several dB of spread at 100 prompts, so near-threshold behaviour is only
# meaningful as a rate — no single realization proves anything.
synthetic_hold_rate(detector, cn0, num_code_blocks; trials = 200, kwargs...) =
    count(
        !synthetic_is_below(
            detector,
            synthetic_tracked_signal(cn0, num_code_blocks, s; kwargs...),
        ) for s = 1:trials
    ) / trials

# Drive `update` over `seeds` at a fixed CN0 and record length, as `process` would.
function synthetic_drive(
    detector,
    cn0,
    num_code_blocks,
    seeds;
    signal_duration = 4u"ms",
    kwargs...,
)
    for seed in seeds
        tracked = synthetic_tracked_signal(cn0, num_code_blocks, seed; kwargs...)
        detector = GNSSReceiver.update(
            detector,
            estimate_cn0(tracked),
            get_last_fully_integrated_integration_time(tracked),
            signal_duration,
        )
    end
    detector
end

@testset "Detector statistic is immune to Tracking's CN0 normalization ($name)" for (
    name,
    make_estimator,
) in (("default estimator", default_estimator), ("moments", moments_estimator))
    # One prompt buffer, presented as an N-block record for several N. `estimate_cn0`
    # divides by the record's integration time, so it reports 10·log10(N) dB lower — but
    # the detector multiplies the same time back in, so its statistic, and therefore its
    # decision, must be bit-identical. This is what makes the threshold independent of how
    # Tracking chooses to normalize — and of which estimator it swaps in as its default.
    detector = synthetic_detector()
    one_block = synthetic_tracked_signal(30.0dBHz, 1, 7; make_estimator)
    reference_cn0 = estimate_cn0(one_block)
    reference_snr = uconvert(
        NoUnits,
        Unitful.linear(reference_cn0) *
        get_last_fully_integrated_integration_time(one_block),
    )

    for num_blocks in (2, 4, 20)
        relabelled =
            TrackedSignal(one_block; last_fully_integrated_num_code_blocks = num_blocks)
        cn0 = estimate_cn0(relabelled)
        # Exactly 10·log10(N) dB lower — the only thing that changed is the divisor.
        @test ustrip(reference_cn0) - ustrip(cn0) ≈ 10 * log10(num_blocks)
        snr = uconvert(
            NoUnits,
            Unitful.linear(cn0) *
            get_last_fully_integrated_integration_time(relabelled),
        )
        @test snr == reference_snr
        @test synthetic_is_below(detector, relabelled) ==
              synthetic_is_below(detector, one_block)
    end
end

@testset "An N-block record buys exactly 10·log10(N) dB of CN0 ($name)" for (
    name,
    make_estimator,
) in (("default estimator", default_estimator), ("moments", moments_estimator))
    # At a fixed CN0 the prompt amplitude grows as √(N·T), so an N-block record at CN0 `x`
    # produces the identical prompt buffer to a one-block record at `x + 10·log10(N)` dB.
    # The detector must therefore make the identical decision — per realization, not just
    # on average. This is the property the fix exists to provide.
    detector = synthetic_detector()
    for num_blocks in (2, 4, 20), cn0 in (18.0dBHz, 25.0dBHz, 31.0dBHz)
        equivalent = cn0 + 10 * log10(num_blocks) * u"dB"
        for seed = 1:25
            @test synthetic_is_below(
                detector,
                synthetic_tracked_signal(cn0, num_blocks, seed; make_estimator),
            ) == synthetic_is_below(
                detector,
                synthetic_tracked_signal(equivalent, 1, seed; make_estimator),
            )
        end
    end
end

@testset "Longer records hold lock on signals a one-block record drops ($name)" for (
    name,
    make_estimator,
) in (("default estimator", default_estimator), ("moments", moments_estimator))
    # The statistical consequence of the above, over many noise realizations, with wide
    # margins rather than any single realization's outcome. The margins are wide enough to
    # hold for both estimators even though they disagree by ~3 dB near threshold (the moment
    # ratio's noise floor reads 27.9 dBHz where NWPR reads a true 25.0).
    detector = synthetic_detector()

    # 22 dBHz is ~8 dB under the threshold: a one-block record almost always drops it,
    # while 20 blocks (13 dB of credit) puts it comfortably above.
    @test synthetic_hold_rate(detector, 22.0dBHz, 1; make_estimator) < 0.4
    @test synthetic_hold_rate(detector, 22.0dBHz, 20; make_estimator) > 0.95
    # Monotone in the number of blocks credited.
    rates =
        [synthetic_hold_rate(detector, 25.0dBHz, n; make_estimator) for n in (1, 2, 4, 20)]
    @test issorted(rates)
    @test rates[1] < 0.4
    @test rates[end] > 0.95

    # A strong signal is held whatever the record length.
    @test synthetic_hold_rate(detector, 45.0dBHz, 1; make_estimator) == 1.0
    @test synthetic_hold_rate(detector, 45.0dBHz, 20; make_estimator) == 1.0
end

@testset "Crediting integration time does not raise the noise-only false-alarm rate" begin
    # On pure noise the detector's statistic reduces to the estimator's raw SNR whatever `N`
    # is — the estimator divides the record's integration time out and the detector
    # multiplies it straight back in — so crediting a longer record must not make a
    # noise-only channel any more likely to be called locked. Checked on the moment ratio,
    # which is the estimator that actually has a noise-only false-alarm rate to raise.
    detector = synthetic_detector()
    false_alarm_one_block = synthetic_hold_rate(detector, 0.0dBHz, 1)
    false_alarm_twenty_blocks = synthetic_hold_rate(detector, 0.0dBHz, 20)
    @test isapprox(false_alarm_one_block, false_alarm_twenty_blocks; atol = 0.05)

    # That rate is high: `cn0_threshold` is a fixed number, not a CFAR threshold calibrated
    # against the estimator's noise distribution, and `MomentsCN0Estimator` reports a median
    # of ~27.6 dBHz on pure noise at 100 prompts. Pinned here so a future move to a
    # false-alarm-parameterized threshold shows up as a deliberate change.
    @test 0.1 < false_alarm_one_block < 0.35

    # The out-of-lock dwell is what actually suppresses those transients: end to end, pure
    # noise loses lock for every record length.
    for num_code_blocks in (1, 20)
        driven = synthetic_drive(synthetic_detector(), 0.0dBHz, num_code_blocks, 1:200)
        @test !GNSSReceiver.is_in_lock(driven)
        @test driven.out_of_lock_time >= driven.out_of_lock_time_threshold
    end
end

@testset "Tracking's default estimator has no noise-only false alarms" begin
    # What Tracking 6's default estimator changes for the detector. NWPR reports
    # `-Inf dBHz` on pure noise rather than the moment ratio's ~27.6 dBHz floor, so a dead
    # or falsely acquired channel clears no threshold at all — at any record length, since
    # the credit multiplies a linear CN0 of zero.
    detector = synthetic_detector()
    for num_code_blocks in (1, 20)
        @test synthetic_hold_rate(
            detector,
            0.0dBHz,
            num_code_blocks;
            make_estimator = default_estimator,
        ) == 0.0
    end

    # The consequence end to end: the out-of-lock dwell is spent monotonically instead of
    # being paid back on the realizations the moment ratio let through, so lock is lost in
    # exactly `wait_time_threshold + out_of_lock_time_threshold` of signal — 20 + 50 updates
    # of 4 ms — rather than the ~85 the moment ratio needs.
    duration = 4u"ms"
    for num_updates in (69, 70)
        driven = synthetic_drive(
            synthetic_detector(),
            0.0dBHz,
            1,
            1:num_updates;
            signal_duration = duration,
            make_estimator = default_estimator,
        )
        @test GNSSReceiver.is_in_lock(driven) == (num_updates < 70)
        @test driven.out_of_lock_time ≈ (num_updates - 20) * duration
    end
end

@testset "Synthetic CN0 drives the detector's lock transitions ($name)" for (
    name,
    make_estimator,
) in (("default estimator", default_estimator), ("moments", moments_estimator))
    # End to end through `update`: the timers and the lock decision, fed by CN0 estimates
    # that came out of Tracking's estimator rather than being fabricated.

    # Note on drive lengths: they are sized for the moment ratio, which clears the threshold
    # on ~20% of noise realizations, so a failing signal accumulates out-of-lock time at
    # only ~0.6·signal_duration per update (the detector pays time back on the realizations
    # that pass) and needs ~85 updates to cross a 200 ms threshold. 200 updates leaves
    # comfortable margin; NWPR gets there in 70.

    # A healthy signal warms up and stays locked.
    healthy = synthetic_drive(synthetic_detector(), 45.0dBHz, 1, 1:200; make_estimator)
    @test GNSSReceiver.is_in_lock(healthy)
    @test healthy.out_of_lock_time == 0u"s"

    # Then it collapses: lock is lost once `out_of_lock_time_threshold` accumulates.
    collapsed = synthetic_drive(healthy, 5.0dBHz, 1, 201:400; make_estimator)
    @test !GNSSReceiver.is_in_lock(collapsed)
    @test collapsed.out_of_lock_time >= collapsed.out_of_lock_time_threshold

    # A 20 dBHz signal is 10 dB under the threshold, so one-block records drop it — but
    # 20-block records hold it without ever accumulating any out-of-lock time at all,
    # because the 13 dB of credited integration time covers the shortfall. Same CN0, same
    # code path; only the record length differs. This is the fix in one comparison.
    on_one_block = synthetic_drive(synthetic_detector(), 20.0dBHz, 1, 1:200; make_estimator)
    on_twenty_blocks =
        synthetic_drive(synthetic_detector(), 20.0dBHz, 20, 1:200; make_estimator)
    @test !GNSSReceiver.is_in_lock(on_one_block)
    @test GNSSReceiver.is_in_lock(on_twenty_blocks)
    @test on_twenty_blocks.out_of_lock_time == 0u"s"
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
