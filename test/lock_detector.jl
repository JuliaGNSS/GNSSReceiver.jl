# One primary code period: the reference integration time the CN0 threshold is anchored
# to, so a record of this length is credited exactly `cn0_threshold` (no relaxation).
const REFERENCE_T = 1u"ms"

@testset "Detector timings scale with the primary code period" begin
    # This is the headline fix: the same code-period counts must become proportionally
    # longer absolute times on a longer code, because Tracking's loop bandwidths are
    # `0.018 / T_code` and so every loop time constant scales with `T_code`.
    for (code_period, factor) in ((1u"ms", 1), (4u"ms", 4), (10u"ms", 10))
        code = GNSSReceiver.CodeLockDetector(; reference_integration_time = code_period)
        @test code.reference_integration_time ≈ code_period
        @test code.out_of_lock_time_threshold ≈ factor * 200u"ms"
        @test code.wait_time_threshold ≈ factor * 80u"ms"

        carrier = GNSSReceiver.CarrierLockDetector(; reference_integration_time = code_period)
        @test carrier.out_of_lock_time_threshold ≈ factor * 4u"s"
        @test carrier.wait_time_threshold ≈ factor * 80u"ms"
    end

    # On GPS L1 C/A the defaults reproduce the historical absolute timings exactly, so the
    # signal the receiver was tuned against sees no change at all.
    l1ca = GNSSReceiver.primary_code_period(GPSL1CA())
    @test GNSSReceiver.CodeLockDetector(;
        reference_integration_time = l1ca,
    ).out_of_lock_time_threshold ≈ 200u"ms"
    @test GNSSReceiver.CarrierLockDetector(;
        reference_integration_time = l1ca,
    ).out_of_lock_time_threshold ≈ 4u"s"

    # Tracking reports integration times in `Hz^-1`; that must configure identically, so the
    # constructors have to `uconvert` rather than assume seconds.
    in_inverse_hz = uconvert(u"Hz^-1", 4u"ms")
    @test GNSSReceiver.CodeLockDetector(;
        reference_integration_time = in_inverse_hz,
    ).out_of_lock_time_threshold ≈ 800u"ms"
    @test GNSSReceiver.CarrierLockDetector(;
        reference_integration_time = in_inverse_hz,
    ).wait_time_threshold ≈ 320u"ms"

    # A count that cannot describe a timing is rejected rather than silently scaled into a
    # negative threshold, which would read as "out of lock" from the very first update.
    @test_throws ArgumentError GNSSReceiver.CodeLockDetector(; out_of_lock_code_periods = 0)
    @test_throws ArgumentError GNSSReceiver.CodeLockDetector(; warm_up_code_periods = -1)
    @test_throws ArgumentError GNSSReceiver.CarrierLockDetector(;
        out_of_lock_code_periods = NaN,
    )
    @test_throws ArgumentError GNSSReceiver.CodeLockDetector(;
        reference_integration_time = 0u"ms",
    )
end

@testset "CodeLockDetector accumulates and pays back out-of-lock time" begin
    # Warm up past the wait-time threshold with a healthy CN0 so the detector
    # starts arming its out-of-lock timer.
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        reference_integration_time = REFERENCE_T,
        out_of_lock_code_periods = 200,
        warm_up_code_periods = 80,
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
        out_of_lock_code_periods = 200,
        warm_up_code_periods = 80,
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
    # Before the warm-up elapses a bad CN0 must not accumulate any
    # out-of-lock time (the detector is still warming up).
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30u"dBHz",
        reference_integration_time = REFERENCE_T,
        warm_up_code_periods = 80,
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
    # `-Inf dBHz` is the default estimator's "no detectable signal" — an unmeasured
    # channel, or a noise reference whose per-record terms have not cleared zero — so it is
    # a value a channel actually reports.
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
    # Both detectors must be anchored, not just the code one.
    @test state.code_lock_detector.out_of_lock_time_threshold ≈ 200u"ms"
    @test state.carrier_lock_detector.out_of_lock_time_threshold ≈ 4u"s"

    # Galileo E1B's 4 ms code period gives timings 4× longer, matching its 4× slower loops —
    # the property whose absence made lock detection fail on the slower signals. Built from
    # the code period directly rather than through `ReceiverSatState(GalileoE1B(), 1)`: that
    # would construct a `GNSSDecoderState`, pulling in Galileo's FEC decoder (Aff3ct), whose
    # binary does not load on Windows. The `ReceiverSatState` wiring is already covered by
    # the GPS L1 C/A case above; what is E1B-specific here is only the code period.
    e1b_period = GNSSReceiver.primary_code_period(GalileoE1B())
    @test GNSSReceiver.CodeLockDetector(;
        reference_integration_time = e1b_period,
    ).out_of_lock_time_threshold ≈ 800u"ms"
    @test GNSSReceiver.CarrierLockDetector(;
        reference_integration_time = e1b_period,
    ).out_of_lock_time_threshold ≈ 16u"s"
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
# so both estimators reach the truth here; in a real loop the residual phase noise shrinks
# the prompt and costs a fraction of a dB near threshold (see `CodeLockDetector`).
#
# Two estimators are exercised: Tracking's per-signal default (`NoiseRefCN0Estimator` as of
# Tracking 7 — what the receiver actually runs on), and the `MomentsCN0Estimator` that used
# to be the default. Fed the identical prompt stream, so anything that holds for both is a
# property of the detector rather than of one estimator's normalization. Passed as a
# zero-argument factory because both buffer into a mutable array that must not be shared
# between realizations.
#
# The two are fed through different `update` arities, and that difference is what Tracking 7
# changes for this file: the moment ratio infers its noise floor from the prompt stream and
# takes a bare prompt, while the noise reference divides by a *measured* density and needs
# the record's `CN0UpdateContext` — that density and the record's own integration time. In
# this model the noise is `randn(ComplexF64)`, i.e. unit power per record, so the density
# that reproduces it is `N₀ = σ²·T = T`: `|P|²/N₀ - 1/T` then has expectation
# `(CN0·T + 1)/T - 1/T = CN0`, the same truth the moment ratio recovers from `A²/σ²`.
# ---------------------------------------------------------------------------------------

const NUM_PROMPTS = 100
default_estimator() = Tracking.default_cn0_estimator(GPSL1CA(), NUM_PROMPTS)
moments_estimator() = MomentsCN0Estimator(NUM_PROMPTS)

# Fold one record's prompt in. The moment ratio (like any estimator that reads the prompt
# stream alone) takes the two-argument form; everything else gets the context the tracking
# loop would have built for this record.
fold_prompt(estimator::MomentsCN0Estimator, prompt, context) =
    Tracking.update(estimator, prompt)
fold_prompt(estimator, prompt, context) = Tracking.update(estimator, prompt, context)

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
    # Unit noise power per record ⇒ a measured density of `T` (see above). Only a
    # noise-referenced estimator reads either field.
    context = Tracking.CN0UpdateContext(
        signal,
        Tracking.get_bit_buffer(base),
        num_code_blocks;
        noise_density = integration_time,
        integration_time,
    )
    rng = Random.Xoshiro(seed)
    estimator = make_estimator()
    for _ = 1:num_prompts
        estimator = fold_prompt(estimator, amplitude + randn(rng, ComplexF64), context)
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

@testset "Detector statistic is immune to Tracking's CN0 normalization (moments)" begin
    # One prompt buffer, presented as an N-block record for several N. The moment ratio
    # divides by the record's integration time when the estimate is *read*, so it reports
    # 10·log10(N) dB lower — but the detector multiplies the same time back in, so its
    # statistic, and therefore its decision, must be bit-identical. This is what makes the
    # threshold independent of how Tracking chooses to normalize. Tracking's default
    # divides per record instead, which the testset below covers.
    make_estimator = moments_estimator
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

@testset "The noise reference applies the integration time per record" begin
    # The other half of the same seam, for the estimator Tracking 7 defaults to. It divides
    # by `T` as it folds each record — where that record's own length is known — so
    # `estimate_cn0` reports the true CN0 whatever the record length, and relabelling a
    # buffer after the fact cannot move it. The detector needs no case distinction either
    # way: what it thresholds is the record's post-integration SNR, and it multiplies the
    # record's own `T` back in regardless of when the estimator divided it out.
    one_block = synthetic_tracked_signal(30.0dBHz, 1, 7; make_estimator = default_estimator)
    @test isapprox(ustrip(estimate_cn0(one_block)), 30.0, atol = 2.0)

    for num_blocks in (2, 4, 20)
        relabelled =
            TrackedSignal(one_block; last_fully_integrated_num_code_blocks = num_blocks)
        # Bit-identical, not merely close: `estimate_cn0` never reads the argument.
        @test estimate_cn0(relabelled) == estimate_cn0(one_block)

        # A genuine N-block record at the same CN0 — the amplitude grows with the record,
        # and the per-record divisor grows with it — lands on the same truth rather than on
        # N times it. That is what makes the number the detector credits `T` to a real CN0
        # under the new default as much as under the old one.
        folded = synthetic_tracked_signal(
            30.0dBHz,
            num_blocks,
            7;
            make_estimator = default_estimator,
        )
        @test isapprox(ustrip(estimate_cn0(folded)), 30.0, atol = 2.0)
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
    # hold for both estimators even though they disagree by ~3 dB near threshold: at a true
    # 25 dBHz the noise reference's median is 25.1, while the moment ratio's noise floor
    # puts it at 27.9.
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
    # What Tracking's default estimator changes for the detector. The moment ratio
    # manufactures ~27.6 dBHz out of pure noise; the noise reference divides by a floor it
    # measured rather than by one it inferred from the same prompts, so on pure noise its
    # per-record terms average to about zero — `-Inf dBHz` on about half the realizations,
    # single digits on the rest. Either way a dead or falsely acquired channel clears no
    # threshold at all, at any record length.
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
    # exactly the warm-up plus the out-of-lock dwell — 20 + 50 updates
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
    # comfortable margin; the noise reference gets there in 70.

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

# ---------------------------------------------------------------------------------------
# CarrierLockDetector — the Van Dierendonck NBD/NBP phase-lock indicator
# ---------------------------------------------------------------------------------------

carrier_detector(; kwargs...) =
    GNSSReceiver.CarrierLockDetector(; reference_integration_time = REFERENCE_T, kwargs...)

# Prompts at a known post-integration SNR and carrier phase error, matching the model the
# detector is derived against: a constant signal amplitude `√ρ` in unit-total-variance
# complex Gaussian noise.
function synthetic_prompts(snr, phase_error_deg, count, seed)
    rng = Random.Xoshiro(seed)
    amplitude = sqrt(snr)
    [amplitude * cis(deg2rad(phase_error_deg)) + randn(rng, ComplexF64) for _ = 1:count]
end

# Hand-rolled rather than pulling `Statistics` into the test target for three one-liners.
# `spread_of` is the population RMS about the mean, the same pattern
# `ion_rtlsdr_integration.jl` uses for residual scatter; `percentile_of` is the nearest-rank
# percentile, since the interpolation `Statistics.quantile` adds is irrelevant to a
# threshold comparison.
mean_of(x) = sum(x) / length(x)
spread_of(x) = sqrt(sum(abs2, x .- mean_of(x)) / length(x))
percentile_of(x, q) = sort(x)[clamp(ceil(Int, q * length(x)), 1, length(x))]

@testset "CarrierLockDetector's indicator matches cos(2φ)·ρ/(ρ+1)" begin
    # The statistic's whole justification is this expectation. Drive the real detector with
    # prompts at known SNR and phase error and check it converges to the analytic value.
    for snr in (1.0, 3.162, 10.0), phase in (0.0, 15.0, 30.0, 45.0)
        detector = carrier_detector(; smoothing_records = 2000)
        detector =
            GNSSReceiver.update(detector, synthetic_prompts(snr, phase, 40000, 42), 40u"s")
        expected = cosd(2 * phase) * snr / (snr + 1)
        @test GNSSReceiver.phase_lock_indicator(detector) ≈ expected atol = 0.02
    end
end

@testset "CarrierLockDetector's indicator is unbiased on noise" begin
    # `E[PLI] = cos(2φ)·ρ/(ρ+1)` goes to zero as the signal power does, so a dead channel
    # sits at zero and the distance from there to the threshold is a real detection margin
    # that can be quoted (median 0.015, p99 0.14 at a 200-record window).
    indicators = map(1:40) do seed
        detector = carrier_detector(; smoothing_records = 200)
        detector =
            GNSSReceiver.update(detector, synthetic_prompts(0.0, 0.0, 4000, seed), 4u"s")
        GNSSReceiver.phase_lock_indicator(detector)
    end
    @test abs(mean_of(indicators)) < 0.05
    # And it stays clear of the default threshold, so noise alone does not hold lock.
    @test maximum(indicators) < 0.25
end

@testset "CarrierLockDetector rejects a quadrature-dominated prompt" begin
    detector = carrier_detector()
    # A prompt dominated by its quadrature component is 90° off: cos(2φ) = −1.
    weak_inphase = complex(0.1, 10.0)
    for _ = 1:200
        detector = GNSSReceiver.update(detector, weak_inphase, 4u"ms")
    end
    @test GNSSReceiver.phase_lock_indicator(detector) < 0
    @test GNSSReceiver.is_below_phase_lock_threshold(detector)
    @test detector.out_of_lock_time > 0u"s"

    # Sustained, it loses lock once the 4 s dwell is exhausted.
    for _ = 1:1000
        detector = GNSSReceiver.update(detector, weak_inphase, 4u"ms")
    end
    @test !GNSSReceiver.is_in_lock(detector)

    # Recovery. The indicator is a smoothed quantity, so swinging it from cos(2φ) = −1 back
    # over the threshold takes on the order of one `smoothing_records` window (200 prompts
    # by default) — a single good prompt cannot and should not do it. That is the deliberate
    # asymmetry: the detector decides quickly and changes its mind slowly.
    strong_inphase = complex(10.0, 0.1)
    detector = GNSSReceiver.update(detector, strong_inphase, 4u"ms")
    @test GNSSReceiver.is_below_phase_lock_threshold(detector)

    for _ = 1:250
        detector = GNSSReceiver.update(detector, strong_inphase, 4u"ms")
    end
    @test GNSSReceiver.phase_lock_indicator(detector) > 0.25
    # Once the indicator is back above threshold the accumulated time clears outright —
    # the "optimistic" credit, so a *consecutive* run of failures is needed to declare loss.
    @test detector.out_of_lock_time == 0u"s"
    @test GNSSReceiver.is_in_lock(detector)
end

@testset "CarrierLockDetector needs a consecutive run of failures" begin
    # Intermittent dips must not accumulate: a marginal satellite's indicator crosses the
    # threshold in both directions, and a paying-back accumulator would random-walk into a
    # spurious loss.
    detector = carrier_detector(; smoothing_records = 1)
    for i = 1:2000
        prompt = isodd(i) ? complex(10.0, 0.1) : complex(0.1, 10.0)
        detector = GNSSReceiver.update(detector, prompt, 4u"ms")
    end
    @test GNSSReceiver.is_in_lock(detector)
    @test detector.out_of_lock_time <= 4.001u"ms"
end

@testset "CarrierLockDetector uses every prompt of the chunk" begin
    # `process` hands the detector the chunk's whole prompt sequence. Folding four prompts
    # in one update must advance the moving averages exactly as four single-prompt updates
    # would — otherwise the smoothing window would silently depend on the chunk length.
    prompts = synthetic_prompts(10.0, 20.0, 4, 7)
    batched = GNSSReceiver.update(carrier_detector(), prompts, 4u"ms")
    one_by_one = carrier_detector()
    for prompt in prompts
        one_by_one = GNSSReceiver.update(one_by_one, prompt, 1u"ms")
    end
    @test GNSSReceiver.phase_lock_indicator(batched) ≈
          GNSSReceiver.phase_lock_indicator(one_by_one)

    # Reading only the last prompt of each chunk — what the receiver used to do — throws
    # three quarters of the evidence away, so the indicator is measurably noisier.
    spread(step) = spread_of(
        map(1:60) do seed
            detector = carrier_detector(; smoothing_records = 200)
            all_prompts = synthetic_prompts(1.0, 0.0, 800, seed)
            for i = 1:step:length(all_prompts)
                detector = GNSSReceiver.update(detector, all_prompts[i], 4u"ms")
            end
            GNSSReceiver.phase_lock_indicator(detector)
        end,
    )
    @test spread(1) < spread(4)
end

@testset "CarrierLockDetector leaves the code detector as the binding limit" begin
    # The indicator saturates at ρ/(ρ+1), so the threshold has to be read together with the
    # C/N0 one. At the code-lock threshold (30 dBHz over a 1 ms record, ρ = 1) even perfect
    # phase lock reads only 0.5, so the carrier threshold must sit below that or it would
    # reject satellites the code detector accepts.
    detector = carrier_detector()
    @test detector.phase_lock_threshold < 0.5

    # A perfectly phase-locked signal exactly at the code-lock threshold must pass.
    at_code_threshold = carrier_detector(; smoothing_records = 2000)
    at_code_threshold = GNSSReceiver.update(
        at_code_threshold,
        synthetic_prompts(1.0, 0.0, 40000, 3),
        40u"s",
    )
    @test !GNSSReceiver.is_below_phase_lock_threshold(at_code_threshold)

    # Read as a phase test at that SNR, the default demands |φ| ≤ 30°.
    @test cosd(2 * 30.0) * 1.0 / (1.0 + 1.0) ≈ 0.25
    # Read as a pure sensitivity floor it is ρ ≥ 1/3 — 25.2 dBHz at a 1 ms record, i.e.
    # below the code detector's 30 dBHz, so the code detector stays the binding limit.
    floor_snr = detector.phase_lock_threshold / (1 - detector.phase_lock_threshold)
    @test floor_snr ≈ 1 / 3
    @test 10 * log10(floor_snr / 1e-3) < 30
end

@testset "CarrierLockDetector treats an unmeasured or non-finite indicator safely" begin
    # Before any prompt has been folded in there is nothing to decide on; the warm-up owns
    # that window, so the detector must not report itself out of lock.
    fresh = carrier_detector()
    @test isnan(GNSSReceiver.phase_lock_indicator(fresh))
    @test !GNSSReceiver.is_below_phase_lock_threshold(fresh)
    @test GNSSReceiver.is_in_lock(fresh)

    # An all-zero prompt (a dead correlator channel) leaves the total power at zero, so the
    # indicator stays unmeasured rather than becoming a NaN that holds lock forever.
    dead = GNSSReceiver.update(fresh, complex(0.0, 0.0), 4u"ms")
    @test GNSSReceiver.is_in_lock(dead)
    # A NaN prompt must count as out of lock: it poisons both averages permanently, and
    # `NaN > 0` is `false`, so it would otherwise read as "nothing measured yet" forever.
    nan_fed = GNSSReceiver.update(fresh, complex(NaN, NaN), 4u"ms")
    @test GNSSReceiver.is_below_phase_lock_threshold(nan_fed)

    # An empty prompt sequence (no record completed this chunk) still advances the timers
    # without inventing evidence.
    empty_chunk = GNSSReceiver.update(fresh, ComplexF64[], 4u"ms")
    @test empty_chunk.wait_time ≈ 4u"ms"
    @test isnan(GNSSReceiver.phase_lock_indicator(empty_chunk))
    @test GNSSReceiver.is_in_lock(empty_chunk)
end

@testset "CarrierLockDetector's smoothing window is counted in records" begin
    # The gain is applied once per prompt, and a prompt is one record spanning
    # `get_last_fully_integrated_num_code_blocks` code blocks — so this window is NOT a
    # count of code periods, unlike the detector's two timings (those are driven by
    # `signal_duration`, which is real elapsed time). They coincide only while Tracking's
    # `preferred_num_code_blocks_to_integrate` stays at 1. The name has to say which it is.
    @test carrier_detector(; smoothing_records = 200).smoothing_gain ≈ 1 / 200
    # Folding N prompts advances the average by N gain steps regardless of the
    # `signal_duration` the chunk claims — which is exactly why the window cannot be
    # expressed in code periods.
    prompts = synthetic_prompts(10.0, 0.0, 50, 11)
    over_one_chunk = GNSSReceiver.update(carrier_detector(), prompts, 4u"ms")
    over_fifty_chunks = carrier_detector()
    for prompt in prompts
        over_fifty_chunks = GNSSReceiver.update(over_fifty_chunks, prompt, 40u"ms")
    end
    @test GNSSReceiver.phase_lock_indicator(over_one_chunk) ≈
          GNSSReceiver.phase_lock_indicator(over_fifty_chunks)
    # 50 chunks of 40 ms against one of 4 ms: 500× the signal time, the same 50 gain steps.
    # The timers, driven by `signal_duration`, do see the difference — the second detector
    # has run its warm-up out while the first has barely started it.
    @test over_one_chunk.wait_time ≈ 4u"ms"
    @test over_fifty_chunks.wait_time ≈ over_fifty_chunks.wait_time_threshold
end

@testset "CarrierLockDetector rejects a configuration that would silently kill the channel" begin
    # `smoothing_records = 0` gives an infinite gain, so both moving averages become `NaN`
    # on the first prompt; `is_below_phase_lock_threshold` then reads that as a dead
    # correlator and holds the channel permanently out of lock. A window below one record
    # gives a gain above 1, which makes the average overshoot every sample instead of
    # smoothing it. Neither is detectable downstream, so both have to fail at construction.
    @test isnan(
        GNSSReceiver.phase_lock_indicator(
            GNSSReceiver.update(
                GNSSReceiver.CarrierLockDetector(
                    0.0,
                    0.0,
                    Inf,
                    0.25,
                    0.0u"s",
                    4u"s",
                    80u"ms",
                    80u"ms",
                ),
                complex(1.0, 0.0),
                4u"ms",
            ),
        ),
    )
    @test_throws ArgumentError carrier_detector(; smoothing_records = 0)
    @test_throws ArgumentError carrier_detector(; smoothing_records = -200)
    @test_throws ArgumentError carrier_detector(; smoothing_records = 0.5)
    @test_throws ArgumentError carrier_detector(; smoothing_records = NaN)
    @test_throws ArgumentError carrier_detector(; smoothing_records = Inf)
    @test carrier_detector(; smoothing_records = 1).smoothing_gain == 1.0

    # The indicator is `cos(2φ)·ρ/(ρ+1) ∈ [-1, 1]`, so a threshold at or above 1 is
    # unreachable at any C/N0 (permanently out of lock) and one at or below −1 is met by
    # anything (loss never reported). The practical ceiling is lower — the indicator
    # saturates at ρ/(ρ+1) = 0.5 at the 30 dBHz the code detector accepts over a one-block
    # record — but that bound moves with the record length, so only the mathematical one is
    # enforced.
    @test_throws ArgumentError carrier_detector(; phase_lock_threshold = 1.0)
    @test_throws ArgumentError carrier_detector(; phase_lock_threshold = 1.5)
    @test_throws ArgumentError carrier_detector(; phase_lock_threshold = -1.0)
    @test_throws ArgumentError carrier_detector(; phase_lock_threshold = NaN)
    @test carrier_detector(; phase_lock_threshold = 0.9) isa GNSSReceiver.CarrierLockDetector
end

@testset "CarrierLockDetector's quoted noise figures are asymptotic" begin
    # The 80-code-period warm-up is shorter than the 200-record smoothing window, so the
    # first judgement is made on an average that has not converged and the docstring's
    # noise-only figures do not yet apply. Measured here so the docstring's claim is checked
    # rather than asserted: the p99 falls monotonically as the window fills, and at the
    # warm-up boundary it is still above the 0.25 threshold.
    noise_indicators(nprompts) = map(1:2000) do seed
        detector = carrier_detector(; smoothing_records = 200)
        GNSSReceiver.phase_lock_indicator(
            GNSSReceiver.update(
                detector,
                synthetic_prompts(0.0, 0.0, nprompts, seed),
                4u"ms",
            ),
        )
    end
    p99(nprompts) = percentile_of(noise_indicators(nprompts), 0.99)
    early, window, converged = p99(80), p99(200), p99(2000)
    @test early > 0.25          # above threshold ~1% of the time when the warm-up ends
    @test window < early        # and the spread shrinks as the average fills
    @test converged < window
    @test converged < 0.15      # the asymptotic figure the docstring quotes

    # The warm-up is deliberately not lengthened to the window: clearing the dwell on any
    # favourable chunk, together with the 4000-code-period dwell, means loss needs a
    # *consecutive* run of failures, so an early excursion in either direction is absorbed.
    @test carrier_detector().wait_time_threshold ≈ 80u"ms"
    # In the false-drop direction the early window is already tight enough: at the code-lock
    # threshold (ρ = 1, perfect phase lock) the indicator is essentially never below 0.25.
    at_threshold = map(1:1000) do seed
        detector = carrier_detector(; smoothing_records = 200)
        GNSSReceiver.phase_lock_indicator(
            GNSSReceiver.update(detector, synthetic_prompts(1.0, 0.0, 80, seed), 4u"ms"),
        )
    end
    @test count(<(0.25), at_threshold) / length(at_threshold) < 0.01
end
