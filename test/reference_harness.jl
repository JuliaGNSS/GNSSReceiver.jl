# ─────────────────────────────────────────────────────────────────────────────
# The reference harness: deterministic synthetic signals, a noise-free reference
# model, and the comparison plumbing every signal-validation step compares its
# own results against.
#
# Issue #130 asks for a per-signal support/evidence matrix, and #135 (step 9) for
# the evidence behind it. The evidence is worthless if every step invents its own
# generator: "the software path and the gateware agree" only means something when
# both were handed the *same* samples and both were measured against the *same*
# reference with the *same* stated tolerance. So the generator, the reference and
# the tolerances live here, once, and the per-step checks plug into them.
#
# Three layers, each usable on its own:
#
#   1. `ReferenceCase` + `SampleSource` — a signal described entirely by its
#      parameters (signal variant, PRN, signal power, Doppler, fractional code
#      phase, antenna layout, noise, seed) and the sample stream that follows
#      from them. Two runs of the same case produce bit-identical samples, and
#      the stream does not depend on how it is chunked, so a replay through a
#      device and a direct software correlation see the very same signal.
#   2. `reference_correlation` — what an ideal correlator, aligned on the truth,
#      would accumulate for a given tap layout. This is the *reference*: the
#      quantity a software correlator, a simulated gateware and a hardware
#      replay are all compared to, rather than to each other.
#   3. `Tolerances` + `compare` — named, documented deviations with an explicit
#      budget per quantity, so a comparison reports *which* quantity moved and
#      by how much instead of a bare `false`.
#
# Nothing in here knows about the receiver: it depends on GNSSSignals alone. That
# is deliberate — the same case that drives a `receive` run drives a bare
# correlation, and later a device.
# ─────────────────────────────────────────────────────────────────────────────

module ReferenceHarness

using GNSSSignals
using Random: Xoshiro, randn, seed!
using Unitful
using Unitful: Hz, ustrip, uconvert

export ReferenceCase,
    SampleSource,
    TapLayout,
    tap_index,
    Tolerances,
    Deviation,
    Comparison,
    epl_taps,
    vepl_taps,
    quantize_taps,
    generate_samples,
    next_samples!,
    reference_correlation,
    correlate,
    estimate_code_phase_error,
    code_phase_resolution,
    estimate_carrier_phase_error,
    measure_cn0,
    code_frequency,
    carrier_frequency,
    signal_amplitude,
    samples_per_code_period,
    min_sampling_freq,
    default_sampling_freq,
    subcarrier_transitions_per_chip,
    quantize,
    software_tolerances,
    quantization_tolerances,
    statistical_tolerances,
    accumulator_noise_relative,
    compare,
    compare_correlations,
    passed,
    deviations

# ─────────────────────────────────────────────────────────────────────────────
# Tap layouts
# ─────────────────────────────────────────────────────────────────────────────

"""
    TapLayout(names, offsets)

A correlator's taps: their `names` and their code-phase `offsets` from the prompt,
in chips, ordered latest-first — `Tracking`'s accumulator order, so a layout maps
element-wise onto `get_accumulators(correlator)`.

Offsets are held in chips rather than samples because a chip offset is a property
of the *layout* while a sample offset is a property of a particular sampling
frequency; [`quantize_taps`](@ref) converts one to the other the way a device has to.
"""
struct TapLayout{N}
    names::NTuple{N,Symbol}
    offsets::NTuple{N,Float64}
end

TapLayout(names, offsets) = TapLayout(Tuple(Symbol.(names)), Tuple(Float64.(offsets)))

num_taps(::TapLayout{N}) where {N} = N
Base.length(layout::TapLayout) = num_taps(layout)

"""
    tap_index(layout, name) -> Int

Index of the tap called `name` in `layout`'s accumulator order.
"""
function tap_index(layout::TapLayout, name::Symbol)
    index = findfirst(==(name), layout.names)
    isnothing(index) &&
        throw(ArgumentError("tap $name is not part of the layout $(layout.names)"))
    index
end

"""
    epl_taps(; early_late_to_prompt_shift = 0.5) -> TapLayout

The three-tap early/prompt/late layout, defaulting to `Tracking`'s
`EarlyPromptLateCorrelator` shift (±0.5 chips about the prompt).
"""
epl_taps(; early_late_to_prompt_shift::Real = 0.5) = TapLayout(
    (:late, :prompt, :early),
    (-Float64(early_late_to_prompt_shift), 0.0, Float64(early_late_to_prompt_shift)),
)

"""
    vepl_taps(; early_late_to_prompt_shift = 0.15,
                very_early_late_to_prompt_shift = 0.6) -> TapLayout

The five-tap very-early/early/prompt/late/very-late layout, defaulting to
`Tracking`'s `VeryEarlyPromptLateCorrelator` shifts. This is the layout a BOC
signal needs, where the narrow inner pair rides the main peak and the outer pair
watches the side peaks the modulation puts there — and the layout the hardware
correlator of issue #130 step 5 has to grow to before any BOC signal can be
validated on it.
"""
vepl_taps(;
    early_late_to_prompt_shift::Real = 0.15,
    very_early_late_to_prompt_shift::Real = 0.6,
) = TapLayout(
    (:very_late, :late, :prompt, :early, :very_early),
    (
        -Float64(very_early_late_to_prompt_shift),
        -Float64(early_late_to_prompt_shift),
        0.0,
        Float64(early_late_to_prompt_shift),
        Float64(very_early_late_to_prompt_shift),
    ),
)

"""
    quantize_taps(layout, sampling_freq, code_freq) -> TapLayout

The layout a correlator sampling at `sampling_freq` can actually realise: every
offset rounded to a whole number of samples (never to zero, so a tap never
collapses onto the prompt), converted back to chips. Matches `Tracking`'s
`calc_preferred_code_shift_to_sample_shift`, which is also what a gateware
correlator is limited to — its replica taps are register delays, not phases.

Comparing a measurement against a reference built from the *unquantized* layout
charges the tap quantisation to the device; comparing against the quantized one
isolates everything else. Which is why this is a function and not a hidden step.
"""
function quantize_taps(layout::TapLayout, sampling_freq, code_freq)
    fs = _hz(sampling_freq)
    fc = _hz(code_freq)
    offsets = map(layout.offsets) do offset
        offset == 0 && return 0.0
        shift = max(1, round(Int, abs(offset) * fs / fc))
        sign(offset) * shift * fc / fs
    end
    TapLayout(layout.names, offsets)
end

# Plain `Float64` Hz from either a `Real` or a Unitful frequency, so every entry
# point takes both and the inner loops only ever see a `Float64`.
_hz(x::Real) = Float64(x)
_hz(x) = Float64(ustrip(Hz, uconvert(Hz, x)))

# ─────────────────────────────────────────────────────────────────────────────
# The case
# ─────────────────────────────────────────────────────────────────────────────

"""
    ReferenceCase(signal; kwargs...)

One reproducible synthetic signal, described entirely by its parameters. Two
`ReferenceCase`s that compare equal generate the same samples, on any machine,
in any order, in any chunking.

Keywords (all with defaults, so a case is as short as `ReferenceCase(GPSL1CA())`):

  - `prn = 1` — the satellite's PRN.
  - `sampling_freq` — sampling frequency in Hz (plain or Unitful). Defaults to
    [`default_sampling_freq`](@ref), which keeps the sub-carrier of a BOC signal
    resolved rather than aliased.
  - `intermediate_freq = 0` — the front-end IF the signal sits at, in Hz.
  - `carrier_doppler = 0` — carrier Doppler in Hz. The code Doppler follows from
    it through `get_code_center_frequency_ratio`, as it does on the sky.
  - `code_phase = 0` — code phase of the *first* sample, in chips. Fractional
    values are the point: a device that rounds its replica to a whole chip, or a
    reference that is off by half a sample, shows up nowhere else.
  - `carrier_phase = 0` — carrier phase of the first sample, in cycles.
  - `cn0_dbhz = 45` — carrier-to-noise density. Together with `noise_power` and
    the sampling frequency this fixes the signal amplitude
    (`A = sqrt(10^(C/N₀/10) · N₀)`, `N₀ = noise_power / sampling_freq`).
  - `amplitude = nothing` — set it to override `cn0_dbhz` with an explicit
    amplitude (the only way to describe a noise-free signal, where C/N₀ is
    infinite).
  - `noise_power = 1` — total power of the complex noise per antenna (`E|n|²`).
    Zero gives a noise-free stream, which is what a replica or a gateware
    bit-exactness check wants.
  - `num_ants = 1`, `steering = nothing` — the antenna layout. `steering` is one
    complex gain per antenna (default: all ones); it is what makes a multi-antenna
    case more than N copies of the same samples.
  - `data_bits = false` — modulate the deterministic ±1 bit stream of
    [`data_bit`](@ref) onto the signal at `get_data_frequency`. Off by default:
    a bit transition inside an integration is a real effect worth testing, but it
    is not what most checks are measuring. Ignored for a pilot (no data rate).
  - `seed = 0x9e3779b97f4a7c15` — seeds both the noise and the data bits.

Everything downstream reads the case through [`code_frequency`](@ref),
[`carrier_frequency`](@ref), [`signal_amplitude`](@ref), [`code_phase_at`](@ref)
and [`carrier_phase_at`](@ref), so the truth is stated once.
"""
struct ReferenceCase{S<:AbstractGNSSSignal}
    signal::S
    prn::Int
    sampling_freq::Float64
    intermediate_freq::Float64
    carrier_doppler::Float64
    code_phase::Float64
    carrier_phase::Float64
    amplitude::Float64
    noise_power::Float64
    num_ants::Int
    steering::Vector{ComplexF64}
    data_bits::Bool
    seed::UInt64
end

"""
    min_sampling_freq(signal) -> Float64

The lowest sampling frequency at which `signal`'s replica can be generated at all:
`code_frequency · subcarrier_transitions_per_chip`, one sample per sub-carrier
half-cycle. GNSSSignals' `gen_code!` — which `Acquisition` builds its replicas
with — refuses anything below it outright, so this is a hard floor for any check
that runs a real acquisition and not just a correlation.

It is why a Galileo E1B or a GPS L1C-P case cannot be run at the 4 MHz a GPS L1
C/A front end gets away with: their BOC(6, 1) component puts the floor at
12.276 MHz.
"""
min_sampling_freq(signal::AbstractGNSSSignal) =
    _hz(get_code_frequency(signal)) * subcarrier_transitions_per_chip(signal)

"""
    default_sampling_freq(signal) -> Float64

A sampling frequency that represents `signal` faithfully: four samples per chip,
raised to four samples per sub-carrier half-cycle for a BOC-family modulation
(four times [`min_sampling_freq`](@ref)).

A BOC(m, n) sub-carrier flips `2m/n` times per chip; sampling a BOC(6, 1) signal
at four samples per chip aliases the sub-carrier away entirely. Since the replica
is evaluated at the same instants, the correlation peak survives — which is
exactly why a too-low rate hides a modulation bug instead of exposing one.

Note what this models and what it does not: the generator evaluates the ideal code
at the sampling instants, i.e. a front end of infinite bandwidth followed by an
ideal sampler. A real front end band-limits first, which rounds the chip
transitions off and costs correlation power. Comparisons that care about absolute
amplitude against real RF have to account for that; comparisons between the
software path, a simulated gateware and a replay of these very samples do not,
because all three see the same stream.
"""
default_sampling_freq(signal::AbstractGNSSSignal) = 4.0 * min_sampling_freq(signal)

"""
    subcarrier_transitions_per_chip(signal) -> Int

Sub-carrier half-cycles per code chip: `1` for a plain BPSK (`LOC`) code, `2m/n`
for a BOC(m, n), and the *highest* component of a composite (CBOC, TMBOC) —
Galileo E1B's BOC(6, 1) component, GPS L1C-P's TMBOC pockets — because that is
what sets the bandwidth the signal has to be sampled at.
"""
subcarrier_transitions_per_chip(signal::AbstractGNSSSignal) =
    _transitions_per_chip(get_modulation(signal))

_transitions_per_chip(::GNSSSignals.Modulation) = 1
# BOC(m, n): the sub-carrier runs at m·1.023 MHz against an n·1.023 MHz chip rate,
# so it flips 2m/n times per chip.
_transitions_per_chip(boc::Union{GNSSSignals.BOCsin,GNSSSignals.BOCcos}) =
    max(1, ceil(Int, 2 * boc.m / boc.n))
# CBOC and TMBOC carry two BOC components; the wider one sets the bandwidth.
_transitions_per_chip(m::Union{GNSSSignals.CBOC,TMBOC}) =
    max(_transitions_per_chip(m.boc1), _transitions_per_chip(m.boc2))

function ReferenceCase(
    signal::AbstractGNSSSignal;
    prn::Integer = 1,
    sampling_freq = default_sampling_freq(signal),
    intermediate_freq = 0.0,
    carrier_doppler = 0.0,
    code_phase::Real = 0.0,
    carrier_phase::Real = 0.0,
    cn0_dbhz::Real = 45.0,
    amplitude::Union{Nothing,Real} = nothing,
    noise_power::Real = 1.0,
    num_ants::Integer = 1,
    steering = nothing,
    data_bits::Bool = false,
    seed::Integer = 0x9e3779b97f4a7c15,
)
    fs = _hz(sampling_freq)
    noise = Float64(noise_power)
    amp = if !isnothing(amplitude)
        Float64(amplitude)
    else
        noise > 0 || throw(
            ArgumentError(
                "a noise-free case (noise_power = 0) has no finite C/N₀; pass `amplitude` instead",
            ),
        )
        sqrt(10^(Float64(cn0_dbhz) / 10) * noise / fs)
    end
    gains =
        isnothing(steering) ? ones(ComplexF64, num_ants) : ComplexF64.(collect(steering))
    length(gains) == num_ants || throw(
        ArgumentError(
            "steering has $(length(gains)) gains but the case has $num_ants antennas",
        ),
    )
    ReferenceCase(
        signal,
        Int(prn),
        fs,
        _hz(intermediate_freq),
        _hz(carrier_doppler),
        Float64(code_phase),
        Float64(carrier_phase),
        amp,
        noise,
        Int(num_ants),
        gains,
        data_bits,
        UInt64(seed),
    )
end

# Value equality (and a matching hash) so a case can be used as a dictionary key and
# so "the same case" is a statement about parameters, not about identity. The signal
# is compared by `get_signal_id` rather than by `==`: a signal object carries its
# baked code table, and Julia's default struct equality on that is identity, so two
# separately constructed `GPSL1CA()`s would otherwise describe different cases while
# generating the same samples.
_case_fields(case::ReferenceCase) =
    ntuple(i -> getfield(case, i + 1), fieldcount(ReferenceCase) - 1)

Base.:(==)(a::ReferenceCase, b::ReferenceCase) =
    get_signal_id(a.signal) == get_signal_id(b.signal) && _case_fields(a) == _case_fields(b)
Base.hash(case::ReferenceCase, h::UInt) =
    hash(_case_fields(case), hash(get_signal_id(case.signal), h))

function Base.show(io::IO, case::ReferenceCase)
    print(
        io,
        "ReferenceCase(",
        get_signal_name(case.signal),
        ", prn = ",
        case.prn,
        ", fs = ",
        round(case.sampling_freq / 1e6; digits = 3),
        " MHz, doppler = ",
        round(case.carrier_doppler; digits = 1),
        " Hz, code phase = ",
        round(case.code_phase; digits = 4),
        " chips, C/N₀ = ",
        round(cn0_dbhz(case); digits = 1),
        " dBHz, ants = ",
        case.num_ants,
        ")",
    )
end

"""
    code_frequency(case) -> Float64

Chipping rate seen at the antenna: the nominal rate plus the code Doppler the
carrier Doppler implies (`get_code_center_frequency_ratio`).
"""
code_frequency(case::ReferenceCase) =
    _hz(get_code_frequency(case.signal)) +
    case.carrier_doppler * get_code_center_frequency_ratio(case.signal)

"""
    carrier_frequency(case) -> Float64

Carrier frequency at baseband: the front-end intermediate frequency plus Doppler.
"""
carrier_frequency(case::ReferenceCase) = case.intermediate_freq + case.carrier_doppler

"""
    signal_amplitude(case) -> Float64

Amplitude of the (unit-RMS) code the case transmits — `A` in
`A · code · exp(i2πφ)`.
"""
signal_amplitude(case::ReferenceCase) = case.amplitude

"""
    cn0_dbhz(case) -> Float64

The case's carrier-to-noise density in dBHz, `Inf` for a noise-free case. The
inverse of the `cn0_dbhz` keyword, so a case built from an explicit amplitude can
still report one.
"""
cn0_dbhz(case::ReferenceCase) =
    case.noise_power > 0 ?
    10 * log10(case.amplitude^2 * case.sampling_freq / case.noise_power) : Inf

"""
    samples_per_code_period(case) -> Int

Samples in one primary code period at the case's sampling frequency, with the
`ceil` convention the receiver's own `samples_per_code` uses.
"""
samples_per_code_period(case::ReferenceCase) = ceil(
    Int,
    get_code_length(case.signal) * case.sampling_freq /
    _hz(get_code_frequency(case.signal)),
)

"""
    code_phase_at(case, n) -> Float64

Absolute code phase in chips at sample `n` (0-based, counted from the start of the
stream). Absolute, not wrapped: `get_code` reads the secondary-code chip out of the
integer part, so wrapping it would strip the secondary code.
"""
code_phase_at(case::ReferenceCase, n::Integer) =
    case.code_phase + code_frequency(case) * n / case.sampling_freq

"""
    carrier_phase_at(case, n) -> Float64

Absolute carrier phase in cycles at sample `n` (0-based).
"""
carrier_phase_at(case::ReferenceCase, n::Integer) =
    case.carrier_phase + carrier_frequency(case) * n / case.sampling_freq

"""
    data_bit(case, bit_index) -> Int

The deterministic ±1 navigation bit at `bit_index`. Not a real navigation
message — no preamble, no parity, nothing a decoder could lock onto — just a
reproducible bit stream, which is all a *tracking* check needs from data
modulation: something that flips the prompt's sign at the symbol rate.
"""
data_bit(case::ReferenceCase, bit_index::Integer) =
    isodd(hash((case.seed, :data, Int(bit_index)))) ? 1 : -1

# The data bit multiplying sample `n`, or `1` when the case carries no data (a
# pilot, or `data_bits = false`).
@inline function data_bit_at(case::ReferenceCase, n::Integer)
    case.data_bits || return 1
    data_freq = _hz(get_data_frequency(case.signal))
    (isfinite(data_freq) && data_freq > 0) || return 1
    data_bit(case, floor(Int, n * data_freq / case.sampling_freq))
end

"""
    noiseless(case) -> ReferenceCase

The same case with its noise removed and its amplitude kept. The reference model
is built from this, so the reference is the noise-free truth rather than one
particular noise realisation.
"""
noiseless(case::ReferenceCase) = ReferenceCase(
    case.signal,
    case.prn,
    case.sampling_freq,
    case.intermediate_freq,
    case.carrier_doppler,
    case.code_phase,
    case.carrier_phase,
    case.amplitude,
    0.0,
    case.num_ants,
    case.steering,
    case.data_bits,
    case.seed,
)

# ─────────────────────────────────────────────────────────────────────────────
# Sample generation
# ─────────────────────────────────────────────────────────────────────────────

"""
    SampleSource(case)

A stateful producer of `case`'s samples. [`next_samples!`](@ref) hands out the
next block; the block boundaries do not change the stream, because the noise is
drawn one sample at a time from a single seeded generator and the signal is a
closed-form function of the absolute sample index. So a run that reads 4000-sample
chunks and a run that reads the whole second at once see identical samples — the
property that lets a device replay and a software correlation be compared at all.
"""
mutable struct SampleSource{S<:AbstractGNSSSignal}
    const case::ReferenceCase{S}
    const rng::Xoshiro
    next_sample::Int
end

SampleSource(case::ReferenceCase) = SampleSource(case, Xoshiro(case.seed), 0)

"""
    reset!(source) -> SampleSource

Rewind the source to the start of the stream, reseeding the noise.
"""
function reset!(source::SampleSource)
    seed!(source.rng, source.case.seed)
    source.next_sample = 0
    source
end

"""
    next_samples!(source, num_samples) -> Matrix{ComplexF64}

The next `num_samples` samples, as a `num_samples × num_ants` matrix — the
sample-major, antenna-minor layout the receiver's channels carry.
"""
function next_samples!(source::SampleSource, num_samples::Integer)
    samples = Matrix{ComplexF64}(undef, num_samples, source.case.num_ants)
    fill_samples!(samples, source)
    samples
end

"""
    fill_samples!(samples, source) -> Matrix

Fill an existing `num_samples × num_ants` matrix from `source` and advance it.
"""
function fill_samples!(samples::AbstractMatrix{ComplexF64}, source::SampleSource)
    case = source.case
    size(samples, 2) == case.num_ants || throw(
        DimensionMismatch(
            "the buffer has $(size(samples, 2)) antennas, the case $(case.num_ants)",
        ),
    )
    _fill_samples!(
        samples,
        case.signal,
        case.prn,
        source.rng,
        source.next_sample,
        case.code_phase,
        code_frequency(case) / case.sampling_freq,
        case.carrier_phase,
        carrier_frequency(case) / case.sampling_freq,
        case.amplitude,
        sqrt(case.noise_power / 2),
        case.steering,
        case,
    )
    source.next_sample += size(samples, 1)
    samples
end

# The inner loop, behind a function barrier so the signal type (and with it the
# `get_code` lookup) is concrete: `get_code` on an abstractly-typed signal boxes
# its result, which costs more than the arithmetic (see the note on
# `SimulatedFPGA`'s fields in test/simulated_fpga.jl).
function _fill_samples!(
    samples,
    signal::S,
    prn,
    rng,
    first_sample,
    code_phase,
    chips_per_sample,
    carrier_phase,
    cycles_per_sample,
    amplitude,
    noise_sigma,
    steering,
    case,
) where {S<:AbstractGNSSSignal}
    num_samples, num_ants = size(samples)
    @inbounds for k = 1:num_samples
        n = first_sample + k - 1
        code = get_code(signal, code_phase + chips_per_sample * n, prn)
        carrier = cis(2π * (carrier_phase + cycles_per_sample * n))
        modulated = amplitude * code * data_bit_at(case, n) * carrier
        for a = 1:num_ants
            # One noise draw per (sample, antenna) in a fixed order, so the stream
            # is a function of the absolute sample index alone.
            noise =
                noise_sigma > 0 ?
                noise_sigma * complex(randn(rng, Float64), randn(rng, Float64)) :
                zero(ComplexF64)
            samples[k, a] = steering[a] * modulated + noise
        end
    end
    samples
end

"""
    generate_samples(case, num_samples; first_sample = 0) -> Matrix{ComplexF64}

`num_samples` samples of `case` starting at absolute sample `first_sample`.

Convenience over [`SampleSource`](@ref) for a one-shot block. Note that the noise
is drawn from the head of the stream regardless of `first_sample` — it is a fresh
realisation, not a window into the same one. Use a `SampleSource` (and read it in
order) when the *same* stream has to be seen twice.
"""
function generate_samples(
    case::ReferenceCase,
    num_samples::Integer;
    first_sample::Integer = 0,
)
    source = SampleSource(case)
    source.next_sample = Int(first_sample)
    next_samples!(source, num_samples)
end

"""
    quantize(samples, num_bits; full_scale_sigma = 4) -> Matrix

`samples` through a mid-tread uniform quantiser of `num_bits` bits per component,
scaled so `full_scale_sigma` standard deviations of the input fit in the range and
clipped beyond it — the front-end (or DMA) quantisation a hardware path applies
before its correlator ever sees a sample.

Here so that a comparison against a quantized path states its quantisation, and so
[`quantization_tolerances`](@ref) can be checked against a real quantiser rather
than a guessed one.
"""
function quantize(
    samples::AbstractArray{<:Complex},
    num_bits::Integer;
    full_scale_sigma::Real = 4,
)
    num_bits >= 1 || throw(ArgumentError("num_bits must be at least 1"))
    scale = full_scale_sigma * sqrt(sum(abs2, samples) / (2 * length(samples)))
    scale > 0 || return copy(samples)
    levels = 2^(num_bits - 1)
    step = scale / levels
    q(x) = clamp(round(x / step), -levels, levels - 1) * step
    map(z -> complex(q(real(z)), q(imag(z))), samples)
end

# ─────────────────────────────────────────────────────────────────────────────
# Correlation and the reference model
# ─────────────────────────────────────────────────────────────────────────────

"""
    correlate(samples, case, layout; kwargs...) -> Matrix{ComplexF64}

Correlate `samples` against `case`'s replica for every tap in `layout`, returning
a `num_taps × num_ants` matrix of accumulators in the layout's (latest-first) order.

The replica follows the case's own trajectory unless an offset moves it:

  - `first_sample = 0` — the absolute sample index `samples[1, :]` sits at, so a
    block taken from the middle of a stream correlates against the right phase.
  - `code_phase_offset = 0` — chips added to the replica's code phase.
  - `carrier_phase_offset = 0` — cycles added to the replica's carrier phase.
  - `doppler_offset = 0` — Hz added to the replica's Doppler, propagating into
    both the carrier and (through the code/carrier ratio) the code rate, exactly
    as a real replica drifts.

The offsets are how a *mis*-aligned correlator is described: a peak search sweeps
`code_phase_offset`, a Doppler search sweeps `doppler_offset`.
"""
function correlate(
    samples::AbstractMatrix,
    case::ReferenceCase,
    layout::TapLayout;
    first_sample::Integer = 0,
    code_phase_offset::Real = 0.0,
    carrier_phase_offset::Real = 0.0,
    doppler_offset::Real = 0.0,
)
    out = zeros(ComplexF64, num_taps(layout), size(samples, 2))
    code_freq =
        code_frequency(case) + doppler_offset * get_code_center_frequency_ratio(case.signal)
    carrier_freq = carrier_frequency(case) + doppler_offset
    _correlate!(
        out,
        samples,
        case.signal,
        case.prn,
        Int(first_sample),
        case.code_phase + code_phase_offset,
        code_freq / case.sampling_freq,
        case.carrier_phase + carrier_phase_offset,
        carrier_freq / case.sampling_freq,
        layout.offsets,
    )
    out
end

function _correlate!(
    out,
    samples,
    signal::S,
    prn,
    first_sample,
    code_phase,
    chips_per_sample,
    carrier_phase,
    cycles_per_sample,
    offsets::NTuple{N,Float64},
) where {S<:AbstractGNSSSignal,N}
    num_samples, num_ants = size(samples)
    @inbounds for k = 1:num_samples
        n = first_sample + k - 1
        phase = code_phase + chips_per_sample * n
        wipeoff = cis(-2π * (carrier_phase + cycles_per_sample * n))
        for t = 1:N
            code = get_code(signal, phase + offsets[t], prn)
            weight = code * wipeoff
            for a = 1:num_ants
                out[t, a] += samples[k, a] * weight
            end
        end
    end
    out
end

"""
    reference_correlation(case, layout, num_samples; first_sample = 0) -> Matrix{ComplexF64}

What an ideal correlator locked on the truth accumulates over `num_samples`
samples of `case`: the noise-free signal correlated against its own replica.

This is *the* reference of the harness. A software correlator, a simulated
gateware and a hardware replay each produce their own version of this matrix, and
each is compared to this one — never to each other, so a shared bug cannot pass
as agreement.

For a unit-RMS code with no data transition inside the window and the prompt tap
aligned, the prompt entry is `A · N · steering[a] · exp(i2π·carrier_phase)`; the
off-prompt taps fall off with the code's autocorrelation, which for a BOC
modulation is not a triangle. Nothing here assumes any of that — it integrates.
"""
function reference_correlation(
    case::ReferenceCase,
    layout::TapLayout,
    num_samples::Integer;
    first_sample::Integer = 0,
)
    samples = generate_samples(noiseless(case), num_samples; first_sample)
    correlate(samples, noiseless(case), layout; first_sample)
end

"""
    code_phase_resolution(case) -> Float64

Half a sample, in chips: `0.5 · code_frequency / sampling_freq`. The floor on how
sharply *any* code-phase estimate can be read off a sampled signal.

A sampled code is piecewise constant, so the correlation against a shifted replica
does not change at all until a shift moves some sample across a chip boundary: the
peak is a plateau one sample wide, not a point. Every code-phase budget in the
harness carries this term, and it is the reason a comparison run at four samples
per chip cannot hold 0.01 chips however clean the signal is.
"""
code_phase_resolution(case::ReferenceCase) = 0.5 * code_frequency(case) / case.sampling_freq

"""
    estimate_code_phase_error(samples, case; kwargs...) -> Float64

The code-phase offset, in chips, at which `samples` correlate most strongly —
i.e. the *error* of the case's stated code phase against what the samples
actually carry. Zero (to within [`code_phase_resolution`](@ref)) for samples the
case generated itself; a handover error, a replay misalignment or an off-by-one
replica shows up as a non-zero value.

A coarse sweep over `± search_span` chips in `coarse_step` steps, then a fine
sweep in `fine_step` steps around the coarse winner, wide enough to contain the
whole plateau. The answer is the *centre* of the plateau rather than its first
point, which is what keeps the estimate unbiased on a noise-free signal.

Keywords: `search_span = 1.5`, `coarse_step = 1 / 8`, `fine_step = 1 / 128`, plus
`first_sample` and `doppler_offset` as in [`correlate`](@ref).
"""
function estimate_code_phase_error(
    samples::AbstractMatrix,
    case::ReferenceCase;
    search_span::Real = 1.5,
    coarse_step::Real = 1 / 8,
    fine_step::Real = 1 / 128,
    first_sample::Integer = 0,
    doppler_offset::Real = 0.0,
)
    prompt = TapLayout((:prompt,), (0.0,))
    power(offset) = sum(
        abs2,
        correlate(
            samples,
            case,
            prompt;
            first_sample,
            code_phase_offset = offset,
            doppler_offset,
        ),
    )
    coarse = _argmax_on_grid(power, -search_span, search_span, coarse_step)
    # Two coarse steps either side: the plateau is one sample wide, which at a low
    # samples-per-chip rate is wider than a single coarse step, and a fine sweep
    # that stopped at the coarse winner ± one step would cut it off on one side and
    # report the bias that asymmetry creates.
    _argmax_on_grid(power, coarse - 2 * coarse_step, coarse + 2 * coarse_step, fine_step)
end

# The centre of the maximising plateau of `f` on the inclusive, `step`-spaced grid
# over `[lo, hi]`. Ties are taken within a relative epsilon, so the flat top of a
# sampled correlation counts as one plateau rather than as float noise.
function _argmax_on_grid(f, lo, hi, step)
    first_best, last_best, best_value = lo, lo, -Inf
    x = lo
    while x <= hi + step / 2
        value = f(x)
        if value > best_value * (1 + 1e-9)
            first_best, last_best, best_value = x, x, value
        elseif value >= best_value * (1 - 1e-9)
            last_best = x
        end
        x += step
    end
    (first_best + last_best) / 2
end

"""
    estimate_carrier_phase_error(samples, case; kwargs...) -> Float64

Carrier-phase error in cycles, wrapped to `(-0.5, 0.5]`: the argument of the
prompt accumulator, which is zero when the replica's phase matches the samples'.

Read modulo a cycle, and modulo *half* a cycle for a data-bearing case — a
navigation bit inverts the prompt, and this cannot tell that from half a cycle of
phase error.
"""
function estimate_carrier_phase_error(
    samples::AbstractMatrix,
    case::ReferenceCase;
    first_sample::Integer = 0,
    code_phase_offset::Real = 0.0,
    doppler_offset::Real = 0.0,
    antenna::Integer = 1,
)
    prompt = TapLayout((:prompt,), (0.0,))
    accumulator =
        correlate(samples, case, prompt; first_sample, code_phase_offset, doppler_offset)[
            1,
            antenna,
        ]
    _wrap_cycles(angle(accumulator) / 2π)
end

_wrap_cycles(x) = x - round(x)

"""
    measure_cn0(samples, case; kwargs...) -> Float64

Carrier-to-noise density in dBHz measured from the prompt accumulator, with the
noise's own contribution removed:

    E|P|² = A²N² + N₀·fs·N   ⇒   Â² = max(0, (|P|² − noise_power·N) / N²)

and `C/N₀ = 10·log₁₀(Â²·fs / noise_power)`. The noise power is the case's, so this
measures the *signal*, not the noise — a deliberate choice, because a device that
scales its output changes `Â` and nothing else, and that is precisely what a
replica-amplitude comparison has to see.

`-Inf` when the estimate collapses to zero (no signal), and `Inf` for a noise-free
case, mirroring [`cn0_dbhz`](@ref). Named `measure_cn0` rather than `estimate_cn0`
because `Tracking` exports a function of the latter name: this one reads a C/N₀ off a
block of samples with the truth in hand, which is a different job from a tracking
loop's running estimator.
"""
function measure_cn0(
    samples::AbstractMatrix,
    case::ReferenceCase;
    first_sample::Integer = 0,
    code_phase_offset::Real = 0.0,
    doppler_offset::Real = 0.0,
    antenna::Integer = 1,
)
    case.noise_power > 0 || return Inf
    prompt = TapLayout((:prompt,), (0.0,))
    accumulator =
        correlate(samples, case, prompt; first_sample, code_phase_offset, doppler_offset)[
            1,
            antenna,
        ]
    n = size(samples, 1)
    power = max(0.0, (abs2(accumulator) - case.noise_power * n) / n^2)
    power > 0 || return -Inf
    10 * log10(power * case.sampling_freq / case.noise_power)
end

# ─────────────────────────────────────────────────────────────────────────────
# Tolerances and comparison
# ─────────────────────────────────────────────────────────────────────────────

"""
    Tolerances(; kwargs...)

The comparison budget, one documented number per quantity. Every check states the
tolerance set it used, so "within tolerance" is a claim that can be read back.

  - `code_phase_chips = 0.01` — code-phase agreement. A hundredth of a chip is
    ~3 m on GPS L1 C/A and far below any tracking loop's own jitter, while being
    reachable by a peak search on a fraction of a code period.
  - `carrier_phase_cycles = 0.02` — carrier-phase agreement, ≈ 7°.
  - `doppler_hz = 1.0` — Doppler agreement, below one acquisition bin at any
    coherent length the receiver uses.
  - `amplitude_relative = 0.05` — relative agreement of the correlation
    magnitude. This is the *replica amplitude* budget the parent issue asks to be
    written down: a device whose replica is scaled differently (a CBOC integer
    replica, a gateware that rounds its code amplitude) lands here rather than
    silently in the code-phase number.
  - `cn0_db = 1.0` — C/N₀ agreement in dB.
  - `correlation_relative = 0.05` — per-tap complex agreement,
    `|measured − reference| / |reference prompt|`. Normalised by the *prompt* so an
    outer tap, whose own magnitude may be near zero, is not held to an unreachable
    relative error.
  - `cross_correlation_relative = 0.15` — the ceiling a *wrong* PRN's correlation
    must stay under, relative to the right PRN's prompt. Generous on purpose: the
    worst cross-correlation of a Gold code family is only ~24 dB down, and a short
    integration window does not reach the asymptotic figure.

Use [`software_tolerances`](@ref) for the pure-software path and
[`quantization_tolerances`](@ref) for a path that quantises.
"""
Base.@kwdef struct Tolerances
    code_phase_chips::Float64 = 0.01
    carrier_phase_cycles::Float64 = 0.02
    doppler_hz::Float64 = 1.0
    amplitude_relative::Float64 = 0.05
    cn0_db::Float64 = 1.0
    correlation_relative::Float64 = 0.05
    cross_correlation_relative::Float64 = 0.15
end

"""
    software_tolerances(; kwargs...) -> Tolerances

The default budget: what a floating-point software correlator handed the harness's
own samples has to meet. Keywords override individual entries.
"""
software_tolerances(; kwargs...) = Tolerances(; kwargs...)

"""
    quantization_tolerances(num_bits; kwargs...) -> Tolerances

The budget for a path that quantises its samples (or its replica) to `num_bits`
bits per component: the software budget with the amplitude- and correlation-related
entries widened by the quantiser's own loss.

A uniform quantiser at `b` bits over ±4σ has a signal-to-quantisation-noise ratio
of roughly `6.02·b − 7.3` dB relative to the input noise, so its fractional
amplitude error is about `10^(−(6.02b − 7.3) / 20)`. That figure is added to the
software budget rather than replacing it — the float path's own error does not go
away — and is floored at the software value, so asking for many bits never
*tightens* the budget below what the float path meets.

At 1 bit this widens the amplitude budget to ~1.2 — a hard limiter's output bears
almost no amplitude information, which is the honest budget — at 4 bits to ~0.2, and
at 8 bits to ~0.06, where the float path's own 0.05 dominates again. Quantisation
does not bias the code phase, so the phase entries are untouched.
"""
function quantization_tolerances(num_bits::Integer; kwargs...)
    base = software_tolerances(; kwargs...)
    relative = 10^(-(6.02 * num_bits - 7.3) / 20)
    Tolerances(;
        code_phase_chips = base.code_phase_chips,
        carrier_phase_cycles = base.carrier_phase_cycles,
        doppler_hz = base.doppler_hz,
        amplitude_relative = base.amplitude_relative + relative,
        cn0_db = base.cn0_db + 20 * log10(1 + relative),
        correlation_relative = base.correlation_relative + relative,
        cross_correlation_relative = base.cross_correlation_relative + relative,
    )
end

"""
    accumulator_noise_relative(case, num_samples) -> Float64

The 1σ magnitude of the noise on a correlation accumulator, relative to the
noise-free prompt: `sqrt(noise_power · N) / (A · N)`, which reduces to
`1 / sqrt(C/N₀ · T)` for an integration of `T = N / fs` seconds. Zero for a
noise-free case.

This is the number that decides whether a comparison budget is reachable at all.
A 45 dBHz signal integrated over one GPS L1 C/A code period gives 0.18 — so no
correlation comparison at that C/N₀ and that window can hold 5 % without being a
coin flip, however tight the intent. [`statistical_tolerances`](@ref) turns it
into a budget; [`noiseless`](@ref) removes the need for one.
"""
accumulator_noise_relative(case::ReferenceCase, num_samples::Integer) =
    case.noise_power > 0 ? sqrt(case.noise_power / (case.amplitude^2 * num_samples)) : 0.0

"""
    statistical_tolerances(case, num_samples; sigma = 5, base = software_tolerances(),
                           early_late_spacing = 0.5) -> Tolerances

The budget for comparing a *noisy* measurement of `case` over `num_samples`
samples: the deterministic `base` budget widened by `sigma` standard deviations of
the thermal noise the case itself specifies.

  - the amplitude, correlation and C/N₀ entries are widened by
    `sigma · accumulator_noise_relative` (the C/N₀ one in dB,
    `20·log₁₀(1 + …)`), and the carrier-phase entry by the same figure converted
    to cycles — for a small error the phase of `1 + ε` is `ε` radians;
  - the code-phase entry is widened by the coherent early–late DLL jitter
    `sqrt(d / (2 · C/N₀ · T))` chips for an early-to-late spacing `d`
    (`early_late_spacing`), the standard yardstick for any code-phase estimate off
    a correlation peak, *plus* [`code_phase_resolution`](@ref) — the half-sample
    floor that survives however strong the signal is.

`sigma = 5` because these checks run in CI on every commit: a 1-in-3.5-million
false failure is the price of not having a flaky test, and a bug that moves a
quantity by less than five standard deviations of its own noise was never going to
be caught by *this* window anyway — it is caught by lengthening it, which lowers
the budget with `1/sqrt(N)`.
"""
function statistical_tolerances(
    case::ReferenceCase,
    num_samples::Integer;
    sigma::Real = 5,
    base::Tolerances = software_tolerances(),
    early_late_spacing::Real = 0.5,
)
    relative = sigma * accumulator_noise_relative(case, num_samples)
    integration_time = num_samples / case.sampling_freq
    cn0_linear =
        case.noise_power > 0 ? case.amplitude^2 * case.sampling_freq / case.noise_power :
        Inf
    code_jitter =
        isfinite(cn0_linear) ?
        sigma * sqrt(early_late_spacing / (2 * cn0_linear * integration_time)) : 0.0
    Tolerances(;
        code_phase_chips = base.code_phase_chips +
                           code_jitter +
                           code_phase_resolution(case),
        carrier_phase_cycles = base.carrier_phase_cycles + relative / 2π,
        doppler_hz = base.doppler_hz,
        amplitude_relative = base.amplitude_relative + relative,
        cn0_db = base.cn0_db + 20 * log10(1 + relative),
        correlation_relative = base.correlation_relative + relative,
        cross_correlation_relative = base.cross_correlation_relative + relative,
    )
end

"""
    Deviation(name, value, tolerance)

One measured deviation and the budget it was held to. `value` is signed where a
sign is meaningful (a phase error) and non-negative where it is not (a relative
magnitude error); [`passed`](@ref) compares `abs(value)` either way.
"""
struct Deviation
    name::Symbol
    value::Float64
    tolerance::Float64
end

passed(d::Deviation) = abs(d.value) <= d.tolerance

"""
    Comparison(label, deviations)

The result of one comparison: every [`Deviation`](@ref) it measured, in the order
it measured them. `passed(comparison)` is true when all of them are within budget;
printing one shows each entry with its budget and a marker, so a failing `@test`
shows *which* quantity moved.
"""
struct Comparison
    label::String
    deviations::Vector{Deviation}
end

passed(c::Comparison) = all(passed, c.deviations)
deviations(c::Comparison) = c.deviations

"""
    deviation(comparison, name) -> Deviation

The named deviation of `comparison`.
"""
function deviation(c::Comparison, name::Symbol)
    index = findfirst(d -> d.name === name, c.deviations)
    isnothing(index) &&
        throw(ArgumentError("comparison $(c.label) has no deviation called $name"))
    c.deviations[index]
end

function Base.show(io::IO, ::MIME"text/plain", c::Comparison)
    println(
        io,
        "Comparison(",
        c.label,
        "): ",
        passed(c) ? "within tolerance" : "OUT OF TOLERANCE",
    )
    for d in c.deviations
        println(
            io,
            "  ",
            passed(d) ? "ok   " : "FAIL ",
            rpad(String(d.name), 26),
            " ",
            _fmt(d.value),
            "  (budget ",
            _fmt(d.tolerance),
            ")",
        )
    end
end
Base.show(io::IO, c::Comparison) =
    print(io, "Comparison(", c.label, ", ", passed(c) ? "ok" : "FAILED", ")")

_fmt(x) =
    abs(x) >= 1e-3 || x == 0 ? string(round(x; digits = 6)) :
    string(round(x; sigdigits = 3))

"""
    compare(label, pairs...) -> Comparison

Build a comparison from `name => (value, tolerance)` pairs. The building block the
per-signal checks use when their quantities are not a correlation matrix.
"""
compare(label::AbstractString, pairs::Pair{Symbol,<:Tuple}...) = Comparison(
    String(label),
    [Deviation(name, Float64(v), Float64(t)) for (name, (v, t)) in pairs],
)

"""
    compare_correlations(label, measured, reference, tolerances) -> Comparison

Compare a measured `num_taps × num_ants` correlation matrix against the
[`reference_correlation`](@ref) of the same window, reporting:

  - `prompt_amplitude_relative` — `| |measured prompt| / |reference prompt| − 1 |`,
    the replica-amplitude figure, against `amplitude_relative`;
  - `prompt_phase_cycles` — the prompt's phase difference in cycles, against
    `carrier_phase_cycles`;
  - `max_tap_relative` — the largest per-tap complex deviation normalised by the
    reference prompt's magnitude, against `correlation_relative`.

`prompt_index` selects the prompt row (the middle row by default, which is where
both [`epl_taps`](@ref) and [`vepl_taps`](@ref) put it).
"""
function compare_correlations(
    label::AbstractString,
    measured::AbstractMatrix,
    reference::AbstractMatrix,
    tolerances::Tolerances = software_tolerances();
    prompt_index::Integer = (size(reference, 1) + 1) ÷ 2,
)
    size(measured) == size(reference) || throw(
        DimensionMismatch(
            "measured $(size(measured)) and reference $(size(reference)) differ in shape",
        ),
    )
    reference_prompt = reference[prompt_index, 1]
    scale = abs(reference_prompt)
    scale > 0 ||
        throw(ArgumentError("the reference prompt is zero; nothing to normalise by"))
    amplitude_error = abs(measured[prompt_index, 1]) / scale - 1
    phase_error = _wrap_cycles(angle(measured[prompt_index, 1] / reference_prompt) / 2π)
    max_tap = maximum(abs.(measured .- reference)) / scale
    Comparison(
        String(label),
        [
            Deviation(
                :prompt_amplitude_relative,
                amplitude_error,
                tolerances.amplitude_relative,
            ),
            Deviation(:prompt_phase_cycles, phase_error, tolerances.carrier_phase_cycles),
            Deviation(:max_tap_relative, max_tap, tolerances.correlation_relative),
        ],
    )
end

end # module ReferenceHarness
