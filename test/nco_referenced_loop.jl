# Tests for the delay-aware hardware loop (issue #107,
# docs/plans/2026-09-14-delay-aware-hardware-loop.md).
#
# The estimator is validated on a noise-free model of the loop it closes: a
# carrier whose phase error advances by `2π (f_true − w) Δt` per record under
# the replica word `w`, with the word the estimator commands landing `d`
# records after the record that produced it. That isolates the one thing this
# estimator changes — how the delay is handled — from everything the loop
# core's simulated device (HardwareLoopCore's tests and
# test/remote_hardware_loop.jl) exercises as well: handover, code loop, bit
# sync, C/N₀.

using GNSSReceiver:
    NCOReferencedPLLAndDLL,
    NCOTimeline,
    FixedNCOWord,
    reset_timeline!,
    schedule_word!,
    promote_words!,
    mean_nco_word,
    nco_word_at,
    word_changes_within,
    scheduled_words

@testset "An NCO timeline averages the words that ran over a span" begin
    tl = NCOTimeline()
    reset_timeline!(tl, 100.0, 0.1)
    @test mean_nco_word(tl, 0, 4000) == (100.0, 0.1)
    @test nco_word_at(tl, 12345) == (100.0, 0.1)
    @test !word_changes_within(tl, 0, 10_000)

    schedule_word!(tl, 1000, 110.0, 0.11)
    schedule_word!(tl, 2000, 120.0, 0.12)
    # Time-weighted: half the span on each word.
    @test mean_nco_word(tl, 500, 1500) == (105.0, 0.105)
    @test mean_nco_word(tl, 0, 4000) == (112.5, 0.1125)
    # An empty span is the word in effect at its start.
    @test mean_nco_word(tl, 1000, 1000) == (110.0, 0.11)
    @test mean_nco_word(tl, 999, 999) == (100.0, 0.1)
    @test nco_word_at(tl, 2000) == (120.0, 0.12)
    # Half-sample bounds, as record centres are.
    @test all(mean_nco_word(tl, 999.5, 1000.5) .≈ (105.0, 0.105))
    # A switch strictly inside `(lo, hi]` separates two records' words.
    @test word_changes_within(tl, 0, 1000)
    @test !word_changes_within(tl, 1000, 2000 - 1)
    @test word_changes_within(tl, 1999, 2000)

    # A newer command for the same or an earlier sample supersedes: the device
    # keeps the newest word it was given for a sample.
    schedule_word!(tl, 2000, 130.0, 0.13)
    @test mean_nco_word(tl, 2000, 3000) == (130.0, 0.13)
    @test length(scheduled_words(tl)) == 2
    schedule_word!(tl, 1500, 140.0, 0.14)
    @test [w.sample for w in scheduled_words(tl)] == [1000, 1500]

    # Promotion folds landed words into the applied one and forgets them.
    promote_words!(tl, 1200)
    @test tl.applied_carrier_doppler == 110.0
    @test [w.sample for w in scheduled_words(tl)] == [1500]
    @test mean_nco_word(tl, 1000, 2000) == (125.0, 0.125)
    promote_words!(tl, 10_000)
    @test isempty(scheduled_words(tl))
    @test mean_nco_word(tl, 0, 1) == (140.0, 0.14)

    # A handover starts over.
    reset_timeline!(tl, 7.0, 0.007)
    @test isempty(scheduled_words(tl))
    @test mean_nco_word(tl, 0, 1) == (7.0, 0.007)

    @test mean_nco_word(FixedNCOWord(3.0, 0.3), 0, 100) == (3.0, 0.3)
end

# ── The loop model ───────────────────────────────────────────────────────────

const LOOP_SYSTEM = GPSL1CA()
const LOOP_FS = 4e6Hz
const LOOP_N = 4000   # samples per record: one code period at 4 MS/s

loop_epl(late, prompt, early) =
    EarlyPromptLateCorrelator(SVector{3,ComplexF64}(late, prompt, early), 1)

# A BPSK loop locks the prompt onto either half of the real axis; the phase
# error that matters is modulo π.
wrap_half_cycle(phase) = rem(phase, π, RoundNearest)

# Close the loop on the model for `steps` records. The replica runs on the words
# the estimator commands, each landing `d` records after the record it was
# computed from; a record's prompt is `cis` of its mean phase error. Returns the
# per-record phase error (mod π) and the word each record ran on.
function simulate_delayed_loop(
    estimator,
    d;
    f_true = 130.0,      # Hz
    handover = 100.0,    # Hz, the acquisition's Doppler estimate
    phi0 = 0.8,          # rad
    steps = 600,
    prn = 7,
)
    track_state = TrackState(
        LOOP_SYSTEM,
        [TrackedSat(LOOP_SYSTEM, prn, 0.0, handover * Hz; doppler_estimator = estimator)];
        doppler_estimator = estimator,
    )
    noise = Tracking._signal_noise_densities(
        track_state.noise_estimators,
        eltype(track_state.groups[1].satellites),
    )
    timeline = NCOTimeline()
    reset_timeline!(timeline, handover, 0.0)
    dt = LOOP_N / ustrip(Hz, LOOP_FS)
    phase = phi0
    phases = Float64[]
    words = Float64[]
    for k = 0:steps-1
        a = k * LOOP_N
        b = a + LOOP_N
        w, _ = nco_word_at(timeline, a)           # constant over the record
        mean_phase = phase + π * (f_true - w) * dt
        phase += 2π * (f_true - w) * dt
        push!(phases, wrap_half_cycle(mean_phase))
        push!(words, w)
        p = cis(mean_phase)
        sat = get_sat_state(track_state, prn)
        push!(
            Tracking.get_correlator_outputs(sat, 1),
            CorrelatorOutput(loop_epl(0.5p, p, 0.5p), LOOP_N, b),
        )
        landing = Int64(b + d * LOOP_N)
        new_sat =
            estimator isa NCOReferencedPLLAndDLL ?
            GNSSReceiver._nco_update_tracked_sat(sat, estimator, LOOP_FS, noise, timeline, landing) :
            Tracking._update_tracked_sat_doppler(sat, LOOP_FS, noise)
        Tracking.get_sat_states(track_state)[prn] = new_sat
        schedule_word!(
            timeline,
            landing,
            ustrip(Hz, get_carrier_doppler(new_sat)),
            ustrip(Hz, get_code_doppler(new_sat)),
        )
        promote_words!(timeline, b - LOOP_N)
    end
    phases, words
end

@testset "With no delay the NCO-referenced loop is the conventional loop" begin
    conventional = simulate_delayed_loop(ConventionalAssistedPLLAndDLL(), 0)
    referenced = simulate_delayed_loop(NCOReferencedPLLAndDLL(), 0)
    # Bit-identical, not approximately equal: the software receiver's tuning
    # is inherited, not re-derived.
    @test referenced[1] == conventional[1]
    @test referenced[2] == conventional[2]
    control = simulate_delayed_loop(NCOReferencedPLLAndDLL(; predict_landing = false), 0)
    @test control[2] == conventional[2]
    # And it is a loop that locks.
    @test all(abs.(referenced[1][400:end]) .< 0.2)
    @test all(abs.(referenced[2][400:end] .- 130.0) .< 1.0)
end

@testset "The NCO-referenced loop holds lock through $d records of feedback delay" for d in 1:6
    phases, words = simulate_delayed_loop(NCOReferencedPLLAndDLL(), d; steps = 1200)
    # Settled after a second: the residual phase and frequency errors of the
    # conventional loop's own (slow, FLL-assisted) settling, not a limit cycle.
    @test all(abs.(phases[1000:end]) .< 0.05)
    @test all(abs.(words[1000:end] .- 130.0) .< 0.3)
    # The transient is the delay-free loop's, only somewhat larger: a 30 Hz
    # step with 0.8 rad of phase error never wraps the discriminator.
    @test maximum(abs.(phases[100:end])) < 1.0
end

@testset "The conventional loop and the negative control limit-cycle at four records of delay" begin
    for estimator in (
        ConventionalAssistedPLLAndDLL(),
        NCOReferencedPLLAndDLL(; predict_landing = false),
    )
        phases, words = simulate_delayed_loop(estimator, 4; steps = 1200)
        tail = words[600:end] .- 130.0
        # The ±80 Hz carrier swing measured on the board (issue #107): the
        # command never settles and the phase error is uniformly wrong.
        @test sqrt(sum(abs2, tail) / length(tail)) > 20
        @test maximum(abs.(phases[600:end])) > 1.0
    end
end

@testset "The NCO-referenced loop pulls in an acquisition-sized offset with delay" begin
    for d in (2, 3, 6), (offset, phi0) in ((100.0, 0.0), (-120.0, 1.2))
        phases, words = simulate_delayed_loop(
            NCOReferencedPLLAndDLL(),
            d;
            f_true = 100.0 + offset,
            phi0,
            steps = 1500,
        )
        @test all(abs.(words[1300:end] .- (100.0 + offset)) .< 0.5)
        @test all(abs.(phases[1300:end]) .< 0.05)
    end
end

@testset "NCO-referenced estimator state and pull-in range" begin
    estimator = NCOReferencedPLLAndDLL()
    @test GNSSReceiver.carrier_doppler_pull_in_range(estimator, GPSL1CA()) == 250.0Hz
    sat = TrackedSat(GPSL1CA(), 3, 0.0, 1234.0Hz; doppler_estimator = estimator)
    state = get_doppler_estimator_state(sat)
    @test state isa GNSSReceiver.SatNCOReferencedPLLAndDLL
    @test state.init_carrier_doppler == 1234.0Hz
    @test state.carrier_loop_filter_bandwidth == 18.0Hz   # GPS L1 C/A reference
    @test isnan(state.previous_record_center)
    narrow = NCOReferencedPLLAndDLL(; carrier_loop_filter_bandwidth = 12.0Hz)
    @test get_doppler_estimator_state(
        TrackedSat(GPSL1CA(), 3, 0.0, 0.0Hz; doppler_estimator = narrow),
    ).carrier_loop_filter_bandwidth == 12.0Hz
    # `reset_loop_filters!` re-seeds from the converged Dopplers and keeps the
    # per-satellite bandwidth.
    track_state = TrackState(GPSL1CA(), [TrackedSat(GPSL1CA(), 3, 0.0, 50.0Hz; doppler_estimator = narrow)];
        doppler_estimator = narrow)
    reset_loop_filters!(track_state)
    reset_state = get_doppler_estimator_state(get_sat_state(track_state, 3))
    @test reset_state.init_carrier_doppler == 50.0Hz
    @test reset_state.carrier_loop_filter_bandwidth == 12.0Hz
    @test reset_state.carrier_loop_filter.x1 == 0.0Hz
end
