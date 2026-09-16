# ─────────────────────────────────────────────────────────────────────────────
# Step 9's reference harness, applied: the per-signal software checks, and the
# integrity tests that bind the support matrix to them.
#
# Three layers, in the order they run:
#
#   1. The harness itself (test/reference_harness.jl) — determinism, chunk
#      independence, the antenna and tap layouts, the quantiser, and the
#      tolerance derivations. A harness whose own reference is wrong would make
#      every claim below meaningless, so it is tested first and directly.
#   2. The per-signal software checks — replica correctness for every signal in
#      the parent issue's scope, an acquisition handover for every signal that
#      can carry one, and a full `receive` run for the GPS L1 C/A baseline. Each
#      passing check records its evidence with `SignalSupport.record_evidence!`.
#   3. The matrix integrity tests — that every `:supported` cell of
#      test/signal_support.jl names evidence that actually exists, that every
#      recorded piece of evidence is claimed by a cell, that the `:not_applicable`
#      cells agree with what GNSSSignals says about the signal, and that the
#      matrix in docs/src/signal_support.md is the one that was just tested.
#
# What this deliberately does *not* do is claim more than it ran. Most cells are
# `untested` or `blocked`, because steps 1-8 of issue #130 have not landed and
# there is nothing yet to have tested. That is the honest state of the matrix and
# the reason it is worth maintaining.
#
# Note: this file is included from runtests.jl, which provides the `using`
# statements for GNSSReceiver, GNSSSignals, Acquisition, Unitful and Test.
# ─────────────────────────────────────────────────────────────────────────────

include("reference_harness.jl")
include("signal_support.jl")

using .ReferenceHarness
using .ReferenceHarness: cn0_dbhz, code_phase_resolution, data_bit, noiseless, reset!
using .SignalSupport
using .SignalSupport: entry, normalize_markdown, reason_keys, render_markdown

# `secondary_value(code, prn, index)` is GNSSSignals' own accessor for a secondary
# (overlay) chip and is deliberately unexported — an internal of the code-generation
# path rather than part of everyone's vocabulary. The secondary-code half of the
# replica check has to compare against *that* table and not against a copy, so it is
# bound here, at one site, the way runtests.jl binds PositionVelocityTime's solver
# internals.
using GNSSSignals: secondary_value

# ─────────────────────────────────────────────────────────────────────────────
# Per-signal test parameters
# ─────────────────────────────────────────────────────────────────────────────

# A PRN that actually broadcasts `signal`. Not cosmetic: GNSSSignals bakes an
# all-zero ranging code for a PRN outside a signal's allocated range (BeiDou B2b
# is defined for PRN 6-58 only), and a case built on one generates a signal of
# exactly zero power — which would fail every check below for a reason that has
# nothing to do with the receiver. `broadcasting_prns` is the receiver's own answer
# to "can a satellite at this PRN carry this signal at all".
function harness_prn(signal::AbstractGNSSSignal, offset::Integer = 0)
    capable = GNSSReceiver.broadcasting_prns(signal)
    prns = isnothing(capable) ? (1:32) : collect(capable)
    prns[mod(2+offset, length(prns))+1]
end

# The sampling frequency the replica checks run at: four samples per sub-carrier
# half-cycle, so a BOC signal's modulation is resolved rather than aliased away.
replica_sampling_freq(signal) = ReferenceHarness.default_sampling_freq(signal)

# The sampling frequency the acquisition checks run at. Four samples per chip is
# what a front end is actually built for; the second term is the floor GNSSSignals'
# `gen_code!` imposes on a BOC replica (see `min_sampling_freq`), doubled so the
# sub-carrier is not sampled exactly at its own transitions. For GPS L1 C/A this is
# 4.092 MHz and for Galileo E1B 24.55 MHz — the cost the BOC(6, 1) component of a
# CBOC signal imposes on any receiver that acquires it.
acquisition_sampling_freq(signal) = max(
    4 * ReferenceHarness._hz(get_code_frequency(signal)),
    2 * ReferenceHarness.min_sampling_freq(signal),
)

# Samples per replica check. Two thousand chips is long enough for the code's
# autocorrelation properties to show (a GPS L1 C/A sidelobe sits 65/1023 below the
# peak and a shorter window would not resolve it), and the cap keeps a 49 MHz
# Galileo E1 case from turning a correlation sweep into a benchmark.
const REPLICA_WINDOW_CHIPS = 2046
const REPLICA_MAX_SAMPLES = 40_000

# Secondary-code chips compared per signal. GPS L1C-P's overlay is 1800 chips long
# at 10 ms each — 18 s of signal — so the check reads a prefix rather than the whole
# period. Sixteen chips is enough that a constant, reversed or off-by-one overlay
# cannot survive, and short enough to cost nothing.
const SECONDARY_CHIPS_CHECKED = 16

# Samples correlated per secondary-code chip. A short window inside each primary
# period: all it has to do is resolve the sign of the accumulator, and on a
# noise-free case a hundred chips does that with no margin to spare at all.
const SECONDARY_WINDOW_SAMPLES = 4_000

# ─────────────────────────────────────────────────────────────────────────────
# The per-signal replica check
# ─────────────────────────────────────────────────────────────────────────────

# Averages here are hand-rolled rather than pulling `Statistics` into the test target
# for two call sites, the same trade test/lock_detector.jl makes.
#
# The code's DC balance over one primary period, in units of the σ a balanced
# random ±1 code of that length would show. A Gold code is balanced by construction
# (GPS L1 C/A comes out at 1/1023), and every code in the scope lands under 2σ; the
# bound below is 6σ, so this catches a table that is constant, truncated or padded
# without ever tripping on a legitimately unbalanced short code.
function code_balance_sigma(signal::AbstractGNSSSignal, prn::Integer)
    code_length = get_code_length(signal)
    total = 0.0
    for chip = 0:(code_length-1)
        total += Float64(get_code(signal, chip + 0.5, prn))
    end
    abs(total) / sqrt(code_length)
end

# The largest correlation magnitude found at least `min_offset` chips away from the
# peak, relative to the peak. A BOC modulation puts real side peaks inside ±1 chip —
# that is what its autocorrelation looks like and not a defect — so the sweep starts
# beyond them and measures the code's own sidelobes.
function max_sidelobe_relative(
    samples,
    case,
    peak;
    min_offset = 1.5,
    max_offset = 4.0,
    step = 0.25,
)
    layout = TapLayout((:prompt,), (0.0,))
    worst = 0.0
    for magnitude in (-1, 1), offset = min_offset:step:max_offset
        value = abs(
            correlate(samples, case, layout; code_phase_offset = magnitude * offset)[1, 1],
        )
        worst = max(worst, value)
    end
    worst / peak
end

# Number of secondary-code chips whose sign disagrees with `get_secondary_code`.
#
# Each primary period is correlated against a replica pulled back by whole code
# periods, so the replica sits in period 0 while the signal is in period k: the sign
# of the accumulator is then `secondary[k] · secondary[0]`, which is a statement
# about the overlay and not about the primary code. Pulling back by a whole number
# of code lengths is exact regardless of Doppler — the fractional phase is untouched
# — so this measures the overlay alone.
function secondary_code_mismatches(case::ReferenceCase; num_chips = SECONDARY_CHIPS_CHECKED)
    signal = case.signal
    secondary = get_secondary_code(signal)
    length = get_secondary_code_length(signal)
    length > 1 || return 0
    primary_length = get_code_length(signal)
    period_samples = samples_per_code_period(case)
    window = min(period_samples, SECONDARY_WINDOW_SAMPLES)
    layout = TapLayout((:prompt,), (0.0,))
    reference_chip = Int(secondary_value(secondary, case.prn, 0))
    mismatches = 0
    for k = 0:(min(num_chips, length)-1)
        first_sample = k * period_samples
        samples = generate_samples(case, window; first_sample)
        accumulator = correlate(
            samples,
            case,
            layout;
            first_sample,
            code_phase_offset = -k * primary_length,
        )[
            1,
            1,
        ]
        expected = Int(secondary_value(secondary, case.prn, k)) * reference_chip
        sign(real(accumulator)) == sign(expected) || (mismatches += 1)
    end
    mismatches
end

"""
    replica_comparison(signal; prn, tolerances) -> Comparison

The replica-correctness check for one signal, as a single [`Comparison`](@ref) so a
failure names the property that broke.

It runs noise-free (an explicit amplitude rather than a C/N₀), at a fractional code
phase and a non-zero Doppler, and measures:

  - `code_power_relative` — the sampled code's mean square against 1. Every code in
    the scope is unit-RMS, including the multi-level CBOC approximation, which is
    what lets one amplitude budget cover every modulation.
  - `code_balance_sigma` — the code's DC balance over one primary period, in σ.
  - `code_phase_chips` — where the correlation peak actually sits against where the
    case says it does, at a fractional phase no sample lands on.
  - `carrier_phase_cycles` — the prompt's residual phase.
  - `prompt_amplitude_relative`, `prompt_phase_cycles`, `max_tap_relative` — the
    three-tap accumulator against [`reference_correlation`](@ref). These three pin
    the reference *helper* against the explicit generate-then-correlate pair rather
    than testing the code table; the entries that carry this check are the ones
    below, which are properties of the code itself and hold no matter how the
    reference is computed.
  - `sidelobe_relative` — the worst correlation beyond ±1.5 chips.
  - `cross_prn_relative` — the same samples against another PRN's replica.
  - `secondary_code_mismatches` — the overlay's chips against `get_secondary_code`,
    zero tolerated.
"""
function replica_comparison(
    signal::AbstractGNSSSignal;
    prn::Integer = harness_prn(signal),
    tolerances::Tolerances = software_tolerances(),
)
    sampling_freq = replica_sampling_freq(signal)
    code_freq = ReferenceHarness._hz(get_code_frequency(signal))
    code_length = get_code_length(signal)
    # A code phase inside the first primary period (so the secondary index is 0) at a
    # fraction of a chip no sampling instant can land on.
    code_phase = floor(0.31 * code_length) + 0.4271
    case = ReferenceCase(
        signal;
        prn,
        sampling_freq,
        carrier_doppler = 873.0,
        code_phase,
        carrier_phase = 0.1234,
        amplitude = 1.0,
        noise_power = 0.0,
    )
    num_samples = min(
        round(Int, REPLICA_WINDOW_CHIPS * sampling_freq / code_freq),
        REPLICA_MAX_SAMPLES,
    )
    samples = generate_samples(case, num_samples)
    layout = epl_taps()
    reference = reference_correlation(case, layout, num_samples)
    measured = correlate(samples, case, layout)
    peak = abs(reference[2, 1])
    other_prn = ReferenceCase(
        signal;
        prn = harness_prn(signal, 7),
        sampling_freq,
        carrier_doppler = 873.0,
        code_phase,
        carrier_phase = 0.1234,
        amplitude = 1.0,
        noise_power = 0.0,
    )
    correlation = compare_correlations("replica", measured, reference, tolerances)
    Comparison(
        "replica $(get_signal_id(signal)) PRN $prn",
        [
            Deviation(
                :code_power_relative,
                sum(abs2, samples) / length(samples) - 1,
                tolerances.amplitude_relative,
            ),
            Deviation(:code_balance_sigma, code_balance_sigma(signal, prn), 6.0),
            Deviation(
                :code_phase_chips,
                estimate_code_phase_error(samples, case),
                tolerances.code_phase_chips + code_phase_resolution(case),
            ),
            Deviation(
                :carrier_phase_cycles,
                estimate_carrier_phase_error(samples, case),
                tolerances.carrier_phase_cycles,
            ),
            deviations(correlation)...,
            Deviation(
                :sidelobe_relative,
                max_sidelobe_relative(samples, case, peak),
                tolerances.cross_correlation_relative,
            ),
            Deviation(
                :cross_prn_relative,
                abs(correlate(samples, other_prn, layout)[2, 1]) / peak,
                tolerances.cross_correlation_relative,
            ),
            Deviation(:secondary_code_mismatches, secondary_code_mismatches(case), 0.0),
        ],
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# The per-signal acquisition-handover check
# ─────────────────────────────────────────────────────────────────────────────

"""
    acquisition_comparison(signal; prn, cn0_dbhz, tolerances) -> Comparison

A cold acquisition of one signal over harness samples, through the receiver's own
`plan_band_acquisition` — so the plan, the PRN restriction and the coherent length
are the ones `receive` would build, not a hand-rolled search.

Measured:

  - `not_detected` — 0 when `Acquisition.is_detected` says the satellite is there,
    1 when it does not. A zero tolerance, because a handover that did not happen is
    not a handover with a large error.
  - `code_phase_chips` — the handed-over code phase against the truth, budgeted at
    two samples. The acquisition grid *is* one sample, so this is the grid's own
    resolution and not a claim about the search's quality.
  - `doppler_hz` — the handed-over Doppler against the truth, budgeted at one and a
    half Doppler bins of the plan's own grid (read off the result rather than
    assumed): the peak can land on the bin next to the truth's, and half a bin of
    margin covers the split-peak case.

The coherent integration time is pinned to 1 ms rather than left to the plan's
default of a single code period. Only one signal in the scope is affected — Galileo
E5a-QP, whose 330-chip approximation code is a 64.5 µs window, too short to detect
anything at a realistic C/N₀ — but pinning it makes the detection margin comparable
across the scope instead of varying by a factor of 300 with the code period.
"""
function acquisition_comparison(
    signal::AbstractGNSSSignal;
    prn::Integer = harness_prn(signal),
    cn0_dbhz::Real = 48.0,
    tolerances::Tolerances = software_tolerances(),
    carrier_doppler::Real = 1234.0,
    seed::Integer = 0x5eed_ac_01,
)
    sampling_freq = acquisition_sampling_freq(signal)
    _, acq_plans, num_samples = GNSSReceiver.plan_band_acquisition(
        (signal,),
        sampling_freq * Hz,
        (nothing,);
        prns = [prn],
        acq_coherent_integration_time = 1u"ms",
    )
    plan = acq_plans[GNSSReceiver.signal_group_key(signal)]
    code_length = get_code_length(signal)
    code_phase = floor(0.37 * code_length) + 0.4271
    case = ReferenceCase(
        signal;
        prn,
        sampling_freq,
        carrier_doppler,
        code_phase,
        cn0_dbhz,
        seed,
    )
    result = only(
        acquire!(
            plan,
            vec(generate_samples(case, num_samples)),
            [prn];
            interm_freq = 0.0Hz,
        ),
    )
    dopplers = ustrip.(Hz, collect(result.dopplers))
    doppler_bin = length(dopplers) > 1 ? abs(dopplers[2] - dopplers[1]) : 0.0
    code_phase_error =
        mod(result.code_phase - code_phase + code_length / 2, code_length) - code_length / 2
    Comparison(
        "acquisition $(get_signal_id(signal)) PRN $prn",
        [
            Deviation(:not_detected, is_detected(result) ? 0.0 : 1.0, 0.0),
            Deviation(
                :code_phase_chips,
                code_phase_error,
                tolerances.code_phase_chips + 4 * code_phase_resolution(case),
            ),
            Deviation(
                :doppler_hz,
                ustrip(Hz, result.carrier_doppler) - carrier_doppler,
                max(tolerances.doppler_hz, 1.5 * doppler_bin),
            ),
        ],
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# The GPS L1 C/A baseline: a full software receive run over harness samples
# ─────────────────────────────────────────────────────────────────────────────

"""
    baseline_receive_comparison(; kwargs...) -> Comparison

The GPS L1 C/A regression baseline: `receive` over a harness stream, end to end.
The satellite has to be acquired from cold, handed over, tracked, and reported in
lock with a C/N₀ close to the one the case specifies.

This is the check that makes `tracking` a supported cell for GPS L1 C/A. It is the
whole receive path, over a stream that is reproducible to the sample, so a
regression in acquisition, in the handover or in the loops all land here — and the
same case can later be replayed through a device and compared to this result rather
than to a fresh guess.

Measured: `never_locked` (zero tolerated), `cn0_db` (the reported C/N₀ against the
case's, at the thermal budget the case and the tracking integration imply) and
`locked_fraction_shortfall` (how far short of `min_locked_fraction` of the run the
satellite stayed locked).
"""
function baseline_receive_comparison(;
    prn::Integer = 11,
    sampling_freq::Real = 4e6,
    seconds::Real = 0.8,
    chunk::Integer = 4000,
    cn0_dbhz::Real = 45.0,
    carrier_doppler::Real = 1200.0,
    code_phase::Real = 137.4,
    seed::Integer = 0x00C0FFEE,
    min_locked_fraction::Real = 0.2,
)
    system = GPSL1CA()
    case = ReferenceCase(
        system;
        prn,
        sampling_freq,
        carrier_doppler,
        code_phase,
        cn0_dbhz,
        seed,
    )
    num_chunks = cld(round(Int, seconds * sampling_freq), chunk)
    channel = GNSSReceiver.spawn_signal_channel_thread(;
        T = ComplexF64,
        num_samples = chunk,
        num_antenna_channels = 1,
    ) do ch
        source = SampleSource(case)
        for _ = 1:num_chunks
            put!(ch, next_samples!(source, chunk))
        end
    end
    data = collect_data(
        receive(
            channel,
            system,
            sampling_freq * Hz,
            ;
            prns = [prn],
            # A finite synthetic stream stays deterministic on a single-threaded
            # runner only with the scan inline: an asynchronous one can finish after
            # the producer has closed its stream, leaving no chunk to merge into.
            acquire_async = false,
            acquire_every = 20ms,
            # Synthetic samples carry no navigation message, so PVT can never
            # converge and its solver must not run.
            time_in_lock_before_calculating_pvt = 1000u"s",
        ),
    )
    key = (get_signal_id(system), prn)
    tracked = [d.sat_data[key] for d in data if haskey(d.sat_data, key)]
    locked = filter(sd -> sd.is_in_lock, tracked)
    locked_fraction = isempty(data) ? 0.0 : length(locked) / length(data)
    # The C/N₀ the tracking loops report, averaged over the locked part of the run.
    # Its budget is the thermal one for the detector's own integration window — one
    # code period — at the case's C/N₀.
    # `ustrip` has no method for a logarithmic `Level`, so the dB figure comes back
    # through the linear ratio, the same way the dashboard reads a C/N₀ bar.
    cn0_db(cn0) = 10 * log10(Unitful.linear(cn0) / Hz)
    reported_cn0 =
        isempty(locked) ? -Inf : sum(cn0_db(sd.cn0) for sd in locked) / length(locked)
    code_period_samples = samples_per_code_period(case)
    cn0_budget = statistical_tolerances(case, code_period_samples).cn0_db
    Comparison(
        "receive GPSL1CA PRN $prn",
        [
            Deviation(:never_locked, isempty(locked) ? 1.0 : 0.0, 0.0),
            Deviation(
                :locked_fraction_shortfall,
                max(0.0, min_locked_fraction - locked_fraction),
                0.0,
            ),
            Deviation(:cn0_db, isempty(locked) ? Inf : reported_cn0 - cn0_dbhz, cn0_budget),
        ],
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# 1. The harness itself
# ─────────────────────────────────────────────────────────────────────────────

@testset "Reference harness: a case is its samples" begin
    case = ReferenceCase(GPSL1CA(); prn = 11, carrier_doppler = 1234.5, code_phase = 137.4)
    num_samples = 3 * samples_per_code_period(case)

    # Determinism: the same case, twice, sample for sample.
    @test generate_samples(case, num_samples) == generate_samples(case, num_samples)

    # Chunk independence — the property the whole comparison rests on. A device
    # replaying 4000-sample chunks and a correlation over the whole window have to
    # see one stream, or "the two agree" means nothing.
    source = SampleSource(case)
    pieces = vcat(next_samples!(source, 1000), next_samples!(source, num_samples - 1000))
    @test pieces == generate_samples(case, num_samples)

    # And rewinding gives the same stream again.
    @test next_samples!(reset!(source), num_samples) == generate_samples(case, num_samples)

    # A different seed is a different noise realisation, but the *signal* underneath
    # is untouched: the noise-free versions are identical.
    other_seed = ReferenceCase(
        GPSL1CA();
        prn = 11,
        carrier_doppler = 1234.5,
        code_phase = 137.4,
        seed = 0xDEADBEEF,
    )
    @test generate_samples(other_seed, 1000) != generate_samples(case, 1000)
    @test generate_samples(noiseless(other_seed), 1000) ==
          generate_samples(noiseless(case), 1000)

    # Value equality: two cases built from the same parameters are the same case.
    @test ReferenceCase(
        GPSL1CA();
        prn = 11,
        carrier_doppler = 1234.5,
        code_phase = 137.4,
    ) == case
    @test ReferenceCase(
        GPSL1CA();
        prn = 12,
        carrier_doppler = 1234.5,
        code_phase = 137.4,
    ) != case

    # C/N₀ and amplitude are two views of one number.
    @test cn0_dbhz(case) ≈ 45.0
    @test signal_amplitude(case) ≈ sqrt(10^4.5 * case.noise_power / case.sampling_freq)
    # A noise-free case has no finite C/N₀ and has to be given an amplitude.
    @test_throws ArgumentError ReferenceCase(GPSL1CA(); noise_power = 0)
    @test cn0_dbhz(ReferenceCase(GPSL1CA(); amplitude = 1.0, noise_power = 0.0)) == Inf
end

@testset "Reference harness: the reference model" begin
    signal = GPSL1CA()
    case = ReferenceCase(
        signal;
        prn = 11,
        carrier_doppler = 700.0,
        code_phase = 512.25,
        carrier_phase = 0.3,
        amplitude = 1.0,
        noise_power = 0.0,
    )
    num_samples = samples_per_code_period(case)
    layout = epl_taps()
    reference = reference_correlation(case, layout, num_samples)

    # The prompt of a noise-free, unit-RMS, aligned correlation is A·N at the case's
    # carrier phase. Nothing in `reference_correlation` assumes this — it integrates
    # — so agreement is a statement about the generator and the correlator together.
    @test abs(reference[2, 1]) ≈ num_samples rtol = 1e-3
    @test angle(reference[2, 1]) / 2π ≈ 0 atol = 1e-3

    # The early and late taps are symmetric about the prompt and below it.
    @test abs(reference[1, 1]) ≈ abs(reference[3, 1]) rtol = 0.02
    @test abs(reference[1, 1]) < abs(reference[2, 1])

    # A block from the middle of the stream correlates to the same thing as long as
    # its absolute position is declared; forgetting `first_sample` is a code-phase
    # error of that many samples and has to look like one. The offset is deliberately
    # not a whole number of code periods — one that was would wrap back onto the same
    # phase and hide the mistake.
    offset = 7 * num_samples + 731
    middle = generate_samples(case, num_samples; first_sample = offset)
    @test abs(correlate(middle, case, layout; first_sample = offset)[2, 1]) ≈ num_samples rtol =
        1e-3
    @test abs(correlate(middle, case, layout)[2, 1]) < 0.2 * num_samples

    # Five taps, on a layout the sampling grid can actually realise: the outer pair
    # sits further down the correlation peak than the inner pair.
    code_freq = ReferenceHarness._hz(get_code_frequency(signal))
    realisable = quantize_taps(vepl_taps(), case.sampling_freq, code_freq)
    five = reference_correlation(case, realisable, num_samples)
    @test size(five) == (5, 1)
    @test abs(five[1, 1]) < abs(five[2, 1]) < abs(five[3, 1])
    @test abs(five[5, 1]) < abs(five[4, 1]) < abs(five[3, 1])

    # Which is exactly why `quantize_taps` exists. `vepl_taps`' 0.15-chip inner pair
    # is below one sample at four samples per chip, and a shift smaller than the
    # sample spacing is not a shift of 0.15 chips — it is whatever the alignment
    # happens to give. Here it gives nothing at all on the early side: not one sample
    # crosses a chip boundary, so that tap returns the prompt's own value and the
    # discriminator it feeds is identically zero. A device cannot realise it either —
    # its replica taps are register delays, which is what `quantize_taps` models.
    ideal = reference_correlation(case, vepl_taps(), num_samples)
    @test ideal[3, 1] == ideal[4, 1]
    @test realisable.offsets[4] ≈ 1 * code_freq / case.sampling_freq
    @test abs(five[4, 1]) < abs(five[3, 1])

    # Taps are addressable by name, in the accumulator order a correlator uses.
    @test tap_index(epl_taps(), :prompt) == 2
    @test tap_index(vepl_taps(), :very_early) == 5
    @test_throws ArgumentError tap_index(epl_taps(), :very_early)

    # The quantised layout rounds onto the sample grid and never collapses a tap onto
    # the prompt, however small the requested shift.
    quantized = quantize_taps(epl_taps(; early_late_to_prompt_shift = 0.5), 4e6, code_freq)
    @test quantized.offsets[2] == 0
    @test quantized.offsets[3] ≈ round(0.5 * 4e6 / code_freq) * code_freq / 4e6
    @test all(
        !=(0),
        quantize_taps(epl_taps(; early_late_to_prompt_shift = 0.01), 4e6, code_freq).offsets[[
            1,
            3,
        ]],
    )
end

@testset "Reference harness: antennas, Doppler and data bits" begin
    signal = GPSL1CA()
    steering = ComplexF64[1.0, 0.5im]
    case = ReferenceCase(
        signal;
        prn = 11,
        num_ants = 2,
        steering,
        amplitude = 1.0,
        noise_power = 0.0,
    )
    num_samples = samples_per_code_period(case)
    reference = reference_correlation(case, epl_taps(), num_samples)
    # Each antenna carries its own complex gain, and the reference reproduces it.
    @test size(reference) == (3, 2)
    @test reference[2, 2] / reference[2, 1] ≈ steering[2] / steering[1] rtol = 1e-6

    # A mismatched steering vector is an argument error, not a silent broadcast.
    @test_throws ArgumentError ReferenceCase(signal; num_ants = 2, steering = [1.0])

    # Doppler moves the code as well as the carrier: correlating a Doppler-bearing
    # signal with a zero-Doppler replica loses the peak over a long enough window.
    doppler_case = ReferenceCase(
        signal;
        prn = 11,
        carrier_doppler = 3000.0,
        amplitude = 1.0,
        noise_power = 0.0,
    )
    samples = generate_samples(doppler_case, num_samples)
    aligned = abs(correlate(samples, doppler_case, epl_taps())[2, 1])
    mismatched =
        abs(correlate(samples, doppler_case, epl_taps(); doppler_offset = -3000.0)[1+1, 1])
    @test aligned ≈ num_samples rtol = 1e-3
    @test mismatched < 0.1 * aligned

    # Data bits are deterministic, ±1, and actually modulate the signal: a window
    # that spans a bit boundary loses coherence a dataless one keeps.
    bit_case = ReferenceCase(
        signal;
        prn = 11,
        data_bits = true,
        amplitude = 1.0,
        noise_power = 0.0,
    )
    @test all(b -> b in (-1, 1), (data_bit(bit_case, k) for k = 0:99))
    @test data_bit(bit_case, 7) == data_bit(bit_case, 7)
    @test any(k -> data_bit(bit_case, k) != data_bit(bit_case, 0), 0:99)
    # And the modulation really is on the samples: integrated across the first bit
    # transition the stream contains, the coherent sum is smaller than the same window
    # of the same signal without data. The span is derived from where the transition
    # actually is rather than assumed — GPS L1 C/A's 20 ms bit means a fixed 30-period
    # window may well fall inside one bit, which would prove nothing.
    bit_samples = round(Int, case.sampling_freq / ustrip(Hz, get_data_frequency(signal)))
    first_flip = findfirst(k -> data_bit(bit_case, k) != data_bit(bit_case, 0), 1:50)
    @test !isnothing(first_flip)
    span = (first_flip + 1) * bit_samples
    dataless = ReferenceCase(signal; prn = 11, amplitude = 1.0, noise_power = 0.0)
    with_bits = abs(correlate(generate_samples(bit_case, span), bit_case, epl_taps())[2, 1])
    without = abs(correlate(generate_samples(dataless, span), dataless, epl_taps())[2, 1])
    @test with_bits < without
end

@testset "Reference harness: tolerances and quantisation" begin
    case = ReferenceCase(GPSL1CA(); prn = 11, cn0_dbhz = 45.0)
    num_samples = samples_per_code_period(case)

    # The accumulator noise is 1/sqrt(C/N₀·T) and falls as 1/sqrt(N) — the reason a
    # tight budget is a statement about the window, not about intent.
    @test accumulator_noise_relative(case, num_samples) ≈
          1 / sqrt(10^4.5 * num_samples / case.sampling_freq) rtol = 1e-6
    @test accumulator_noise_relative(case, 4 * num_samples) ≈
          accumulator_noise_relative(case, num_samples) / 2 rtol = 1e-6
    @test accumulator_noise_relative(noiseless(case), num_samples) == 0

    # A statistical budget is wider than the deterministic one, and narrows as the
    # window grows.
    wide = statistical_tolerances(case, num_samples)
    narrow = statistical_tolerances(case, 100 * num_samples)
    @test wide.amplitude_relative >
          narrow.amplitude_relative >
          software_tolerances().amplitude_relative
    @test wide.cn0_db > narrow.cn0_db > software_tolerances().cn0_db
    # The code-phase budget never drops below half a sample however long the window.
    @test narrow.code_phase_chips > code_phase_resolution(case)

    # And a noisy measurement of the case lands inside its own budget: the C/N₀ the
    # harness reads back from the samples matches the one that was asked for.
    samples = generate_samples(case, 20 * num_samples)
    budget = statistical_tolerances(case, 20 * num_samples)
    @test abs(measure_cn0(samples, case) - 45.0) <= budget.cn0_db
    @test abs(estimate_code_phase_error(samples, case)) <= budget.code_phase_chips

    # Quantisation: an 8-bit path is close to the float one, a 1-bit path is not —
    # and each is inside the budget its own bit depth buys.
    noise_free = ReferenceCase(GPSL1CA(); prn = 11, amplitude = 1.0, noise_power = 0.0)
    clean = generate_samples(noise_free, num_samples)
    reference = reference_correlation(noise_free, epl_taps(), num_samples)
    for bits in (8, 4, 2, 1)
        measured = correlate(quantize(clean, bits), noise_free, epl_taps())
        @test passed(
            compare_correlations(
                "quantized $bits bit",
                measured,
                reference,
                quantization_tolerances(bits),
            ),
        )
    end
    @test !passed(
        compare_correlations(
            "1 bit against the float budget",
            correlate(quantize(clean, 1), noise_free, epl_taps()),
            reference,
            software_tolerances(),
        ),
    )
    # More bits is a tighter budget, floored at the software one.
    @test quantization_tolerances(2).amplitude_relative >
          quantization_tolerances(10).amplitude_relative >=
          software_tolerances().amplitude_relative

    # A comparison reports the quantity that moved, not just a verdict.
    wrong = compare_correlations(
        "wrong replica",
        correlate(
            clean,
            ReferenceCase(GPSL1CA(); prn = 12, amplitude = 1.0, noise_power = 0.0),
            epl_taps(),
        ),
        reference,
    )
    @test !passed(wrong)
    @test !passed(ReferenceHarness.deviation(wrong, :prompt_amplitude_relative))
    @test occursin("OUT OF TOLERANCE", sprint(show, MIME"text/plain"(), wrong))
end

# ─────────────────────────────────────────────────────────────────────────────
# 2. The per-signal software checks
# ─────────────────────────────────────────────────────────────────────────────

@testset "Replica: $(get_signal_id(signal))" for signal in scope_signals()
    comparison = replica_comparison(signal)
    passed(comparison) || show(stdout, MIME"text/plain"(), comparison)
    @test passed(comparison)
    passed(comparison) &&
        record_evidence!(get_signal_id(signal), :replica, :harness_replica)
end

@testset "Acquisition handover: $(get_signal_id(signal))" for signal in filter(
    s -> entry(s, :acquisition_handover).status === :supported,
    scope_signals(),
)
    comparison = acquisition_comparison(signal)
    passed(comparison) || show(stdout, MIME"text/plain"(), comparison)
    @test passed(comparison)
    passed(comparison) &&
        record_evidence!(get_signal_id(signal), :acquisition_handover, :harness_acquisition)
end

@testset "GPS L1 C/A baseline: acquire, track and hold lock" begin
    comparison = baseline_receive_comparison()
    passed(comparison) || show(stdout, MIME"text/plain"(), comparison)
    @test passed(comparison)
    passed(comparison) && record_evidence!(:GPSL1CA, :tracking, :harness_receive)
end

# ─────────────────────────────────────────────────────────────────────────────
# 3. Matrix integrity
# ─────────────────────────────────────────────────────────────────────────────

# Every concrete signal GNSSSignals exports. The matrix has to cover exactly these.
function exported_signals()
    types = filter(names(GNSSSignals)) do name
        value = getfield(GNSSSignals, name)
        value isa Type && value <: AbstractGNSSSignal && !isabstracttype(value)
    end
    Set(get_signal_id(getfield(GNSSSignals, name)()) for name in types)
end

@testset "Support matrix: scope" begin
    # A GNSSSignals release that adds a signal fails here until the matrix grows a
    # row for it — which is what "extend this matrix when the supported GNSSSignals
    # release adds signals" (issue #130) means when it is not left to goodwill.
    @test Set(get_signal_id(s) for s in scope_signals()) == exported_signals()
    @test Set(keys(MATRIX)) == exported_signals()
    # No signal listed twice across the families.
    @test length(scope_signals()) == length(unique(get_signal_id.(scope_signals())))
end

@testset "Support matrix: well-formed cells" begin
    for signal in scope_signals()
        signal_id = get_signal_id(signal)
        for role in ROLES
            cell = entry(signal_id, role)
            label = "$signal_id/$role"
            @test cell.status in STATUSES
            @test cell.evidence in EVIDENCE_LEVELS
            if cell.status === :supported
                # A supported cell names its level and its evidence. This is the rule
                # that stops the matrix from being metadata: there is no way to write
                # a supported cell without pointing at something.
                @test cell.evidence !== :none
                @test haskey(SOURCES, cell.source)
                @test cell.reason === Symbol("")
            else
                @test cell.evidence === :none
                @test cell.source === Symbol("")
                # And an unsupported cell says why, in a reason the legend explains.
                @test cell.reason in reason_keys()
            end
        end
    end
end

@testset "Support matrix: `n/a` agrees with the signal model" begin
    for signal in scope_signals()
        signal_id = get_signal_id(signal)
        is_pilot = ustrip(Hz, get_data_frequency(signal)) == 0
        has_secondary = get_secondary_code_length(signal) > 1
        # A pilot carries no navigation data, and only a pilot may claim that.
        @test (entry(signal_id, :data_decode).status === :not_applicable) == is_pilot
        # Secondary synchronisation is meaningless without a secondary code — and
        # mandatory to have an answer for when there is one.
        @test (entry(signal_id, :secondary_sync).status === :not_applicable) ==
              !has_secondary
        # Nothing is ever `n/a` for the roles every signal has to fulfil.
        @test entry(signal_id, :replica).status !== :not_applicable
        @test entry(signal_id, :tracking).status !== :not_applicable
        @test entry(signal_id, :pvt).status !== :not_applicable
    end
end

@testset "Support matrix: every claim has evidence, every evidence a claim" begin
    recorded = recorded_evidence()
    claimed = Set{Tuple{Symbol,Symbol,Symbol}}()
    for signal in scope_signals(), role in ROLES
        cell = entry(signal, role)
        cell.status === :supported || continue
        push!(claimed, (get_signal_id(signal), role, cell.source))
    end

    # Claims backed by a check in this file: the check must have run *and passed*
    # this session. A failing check records nothing, so the claim shows up here.
    harness_claims = filter(c -> startswith(String(c[3]), "harness_"), claimed)
    @test setdiff(harness_claims, recorded) == Set()
    # And nothing runs unclaimed: a check whose cell was quietly downgraded is a
    # stale matrix, which is the other way this artifact rots.
    @test setdiff(recorded, claimed) == Set()

    # Claims backed by another test file: the file has to exist and be wired into
    # the suite, or the evidence is a citation of something that never runs.
    runtests = read(joinpath(@__DIR__, "runtests.jl"), String)
    for (_, _, source) in setdiff(claimed, harness_claims)
        file = SOURCES[source].file
        @test isfile(joinpath(@__DIR__, "..", file))
        @test occursin(basename(file), runtests)
    end

    # The legend carries no stale entries and no missing ones.
    used_reasons = Set(
        entry(signal, role).reason for signal in scope_signals() for
        role in ROLES if entry(signal, role).status !== :supported
    )
    @test used_reasons == Set(reason_keys())
    used_sources = Set(c[3] for c in claimed)
    @test used_sources == Set(keys(SOURCES))
end

@testset "Support matrix: the documented table is the tested one" begin
    path = joinpath(@__DIR__, "..", "docs", "src", "signal_support.md")
    @test isfile(path)
    document = read(path, String)
    opening = "<!-- BEGIN GENERATED MATRIX -->"
    closing = "<!-- END GENERATED MATRIX -->"
    @test occursin(opening, document)
    @test occursin(closing, document)
    documented =
        document[(findfirst(opening, document).stop+1):(findfirst(closing, document).start-1)]

    # Compared line by line after normalisation, and asserted on the *list of
    # differing lines* rather than on the two documents: a whole-document `==` that
    # fails prints both copies of a 25-row table, which is unreadable exactly when it
    # is needed. `normalize_markdown` absorbs the formatter's table alignment and
    # bullet markers, so what is left is a genuine content difference.
    from_docs = split(normalize_markdown(documented), '\n')
    from_code = split(normalize_markdown(render_markdown()), '\n')
    differences = [
        (line, get(from_docs, line, "<missing>"), get(from_code, line, "<missing>")) for
        line = 1:max(length(from_docs), length(from_code)) if
        get(from_docs, line, "") != get(from_code, line, "")
    ]
    if !isempty(differences)
        @warn "docs/src/signal_support.md is stale: regenerate the block between the " *
              "generated-matrix markers from `SignalSupport.render_markdown()`. " *
              "$(length(differences)) line(s) differ; the first few follow."
        for (line, documented_line, generated_line) in first(differences, 5)
            println("  line $line\n    docs: $documented_line\n    code: $generated_line")
        end
    end
    # Asserted on the *count*: `@test` prints the expression it evaluated, and a
    # predicate over the list would print the list. The lines themselves were just
    # printed above, where they can be read.
    @test length(differences) == 0
end
