# ─────────────────────────────────────────────────────────────────────────────
# What a hardware correlator can do, and what it is asked to do
# (GNSSReceiver.jl #131)
#
# The hardware path in `hardware_correlator.jl` was written against one device
# and one signal: GPS L1 C/A, a 1023-chip BPSK code, three taps. Every one of
# those was an assumption rather than a statement — a request for Galileo E1B
# reached the gateware unchallenged and failed later, as a `DimensionMismatch`
# deep in the ingest path where five accumulators met a three-accumulator
# record.
#
# This file turns those assumptions into a declaration and a gate:
#
#   * [`HardwareCorrelatorCapabilities`](@ref) — what the device's gateware can
#     replicate and correlate, asked for with [`hardware_capabilities`](@ref).
#     A device that declares nothing *is* the legacy GPS L1 C/A device, so the
#     boards that shipped before this interface keep working and keep being
#     described truthfully.
#   * [`validate_hardware_configuration`](@ref) — run once, before the receiver
#     starts, so an unserviceable request is an actionable error rather than a
#     channel that never locks.
#   * [`HardwareChannelConfig`](@ref) — the complete description of what one
#     hardware channel must do: every quantised tap offset, the replica
#     amplitude and code normalisation, the overlay handling, the component's
#     identity and its carrier-phase reference.
#
# Together they are the adapter/gateware contract; the prose version lives in
# `docs/src/hardware_contract.md`.
# ─────────────────────────────────────────────────────────────────────────────

"""
    HardwareCorrelatorCapabilities

What a hardware correlator's gateware can replicate and correlate — the
vendor-neutral answer to "may this device be asked to track this signal?".

Every field is a *limit*, not a promise about a particular channel, and every
one of them can be left unrestricted. Build one with keywords:

```julia
HardwareCorrelatorCapabilities(;
    signals = [:GPSL1CA, :GalileoE1B],      # `GNSSSignals.get_signal_id`s, or `nothing` for any
    modulations = [:LOC, :CBOC],            # `nameof(typeof(get_modulation(signal)))`, or `nothing`
    max_primary_code_length = 4092,         # chips the code memory holds
    code_frequency_limits = (1.023e6, 1.023e6),  # Hz, inclusive
    tap_layouts = [3, 5],                   # accumulator counts the correlator bank can produce
    max_tap_offset_chips = 1.0,             # how far from prompt a replica can be placed
    num_antennas = 1,
    bands = [:L1],                          # `GNSSSignals.get_band_id`s, or `nothing` for any
    num_rf_inputs = 1,                      # bands that can be received at once
    max_secondary_code_length = 1,          # longest overlay the gateway can wipe off; 1 = none
    reports_code_phase = true,              # does it latch `CorrelatorDump.code_phase`?
)
```

Fields:

  - `signals` — the `get_signal_id` symbols the gateware has code generators
    for, or `nothing` when it can replicate any signal GNSSSignals describes
    (a device that loads code tables from the host, say).
  - `modulations` — the modulation type names it can synthesise
    (`:LOC`, `:BOCsin`, `:BOCcos`, `:CBOC`, `:TMBOC`), or `nothing` for any.
    Declaring this separately from `signals` is what lets a device that has a
    generic BPSK generator say so without enumerating every BPSK signal.
  - `max_primary_code_length` — chips of primary code the channel's code memory
    holds. GPS L2 CL's 767 250-chip code is the one that fails this on almost
    every device; see issue #133 for what a device short of it can still do.
  - `code_frequency_limits` — inclusive `(min, max)` chip rate in Hz the code
    NCO covers.
  - `tap_layouts` — the numbers of accumulators one channel can produce. `[3]`
    is an E/P/L bank, `[3, 5]` one that can also do VE/E/P/L/VL. A signal
    whose `Tracking` correlator has a tap count that is not in this list cannot
    be tracked on this device: the missing taps cannot be invented host-side.
  - `max_tap_offset_chips` — how far either side of prompt a replica can be
    placed, in chips. The LiteX-M2SDR's code RAM reaches ±1 chip and rejects
    anything wider.
  - `num_antennas` — accumulators per tap, i.e. how many antenna chains the
    correlator bank despreads in parallel. Beamforming is post-correlation on
    the host, so this is what caps [`EigenBeamformer`](@ref)'s array.
  - `bands` — the `get_band_id` symbols the RF front end can tune, or `nothing`
    for any.
  - `num_rf_inputs` — how many of those bands can be received *at the same
    time*. Simultaneous all-band reception is an RF-capacity constraint, not a
    correlator one, and this is where it is declared.
  - `max_secondary_code_length` — the longest secondary (overlay) code the
    gateware can wipe off itself, or `1` for "primary code only". Purely a
    declaration: the host removes the overlay from the dumps itself once the
    sync detector has found its phase (see
    [`GNSSReceiver.requested_secondary_code_mode`](@ref)), so nothing asks a
    device to do it and `1` costs a device nothing. It is what
    [`supports_secondary_code_wipeoff`](@ref) reads, and what a future
    *scheduled* wipeoff contract — which would let a device pre-accumulate
    across code periods — would gate on.
  - `reports_code_phase` — whether the device latches the replica's code phase
    alongside the accumulators (`CorrelatorDump.code_phase`). Informational:
    without it pseudoranges are dead-reckoned from the handover seed rather
    than anchored to the replica the DLL steers.

A device declares its own with [`hardware_capabilities`](@ref); one that does
not is taken to be [`LEGACY_GPS_L1CA_CAPABILITIES`](@ref).
"""
struct HardwareCorrelatorCapabilities
    signals::Union{Nothing,Vector{Symbol}}
    modulations::Union{Nothing,Vector{Symbol}}
    max_primary_code_length::Int
    code_frequency_limits::Tuple{Float64,Float64}
    tap_layouts::Vector{Int}
    max_tap_offset_chips::Float64
    num_antennas::Int
    bands::Union{Nothing,Vector{Symbol}}
    num_rf_inputs::Int
    max_secondary_code_length::Int
    reports_code_phase::Bool
end

function HardwareCorrelatorCapabilities(;
    signals = nothing,
    modulations = nothing,
    max_primary_code_length::Integer = typemax(Int),
    code_frequency_limits = (0.0, Inf),
    tap_layouts = [3, 5],
    max_tap_offset_chips::Real = Inf,
    num_antennas::Integer = 1,
    bands = nothing,
    num_rf_inputs::Integer = 1,
    max_secondary_code_length::Integer = 1,
    reports_code_phase::Bool = false,
)
    layouts = collect(Int, tap_layouts)
    isempty(layouts) &&
        throw(ArgumentError("tap_layouts must name at least one accumulator count"))
    all(>(0), layouts) ||
        throw(ArgumentError("tap_layouts must be positive accumulator counts"))
    num_antennas >= 1 ||
        throw(ArgumentError("num_antennas must be at least 1 (got $num_antennas)"))
    num_rf_inputs >= 1 ||
        throw(ArgumentError("num_rf_inputs must be at least 1 (got $num_rf_inputs)"))
    lo, hi = Float64(first(code_frequency_limits)), Float64(last(code_frequency_limits))
    lo <= hi || throw(
        ArgumentError("code_frequency_limits must be (min, max) in Hz (got ($lo, $hi))"),
    )
    HardwareCorrelatorCapabilities(
        isnothing(signals) ? nothing : collect(Symbol, signals),
        isnothing(modulations) ? nothing : collect(Symbol, modulations),
        Int(max_primary_code_length),
        (lo, hi),
        layouts,
        Float64(max_tap_offset_chips),
        Int(num_antennas),
        isnothing(bands) ? nothing : collect(Symbol, bands),
        Int(num_rf_inputs),
        Int(max_secondary_code_length),
        reports_code_phase,
    )
end

"""
    LEGACY_GPS_L1CA_CAPABILITIES

The profile of a device that predates [`hardware_capabilities`](@ref): the GPS
L1 C/A correlator the hardware path was written against (issues #107, #129).
One RF input on L1, a 1023-chip BPSK code at 1.023 Mcps, a three-tap E/P/L bank
reaching ±1 chip, one antenna, no secondary-code wipeoff.

This is what [`hardware_capabilities`](@ref) returns for a device that declares
nothing, so an existing adapter keeps working and keeps being described
truthfully — and a request it cannot serve is refused with an actionable error
instead of reaching the gateware.
"""
const LEGACY_GPS_L1CA_CAPABILITIES = HardwareCorrelatorCapabilities(;
    signals = [:GPSL1CA],
    modulations = [:LOC],
    max_primary_code_length = 1023,
    code_frequency_limits = (1.023e6, 1.023e6),
    tap_layouts = [3],
    max_tap_offset_chips = 1.0,
    num_antennas = 1,
    bands = [:L1],
    num_rf_inputs = 1,
    max_secondary_code_length = 1,
    reports_code_phase = true,
)

"""
    supports_secondary_code_wipeoff(capabilities, signal) -> Bool

Whether the device can wipe `signal`'s secondary (overlay) code off in the
gateware, so consecutive dumps can be summed without the overlay cancelling
them.

`false` — the default for every device — is not an error, and `true` changes
nothing today: the host removes the overlay from each primary-period dump once
it knows its phase, so [`HardwareChannelConfig`](@ref) asks for `:primary_only`
whatever a device declares. See
[`GNSSReceiver.requested_secondary_code_mode`](@ref) for why ownership sits
there, and [`coherent_integration_blocks`](@ref) for what the removal unlocks.
"""
supports_secondary_code_wipeoff(
    capabilities::HardwareCorrelatorCapabilities,
    signal::AbstractGNSSSignal,
) =
    get_secondary_code_length(signal) > 1 &&
    get_secondary_code_length(signal) <= capabilities.max_secondary_code_length

# ─────────────────────────────────────────────────────────────────────────────
# Validation, before anything is armed
# ─────────────────────────────────────────────────────────────────────────────

_hz(x::Real) = Float64(x)
_hz(x) = Float64(ustrip(uconvert(Hz, x)))

_modulation_name(signal) = nameof(typeof(get_modulation(signal)))

# Quantised replica offsets for `correlator`, latest first and prompt at zero —
# exactly what `Tracking`'s discriminators recover from the correlator they are
# handed, and therefore exactly what the device has to program.
_tap_sample_shifts(correlator, sampling_freq, code_frequency) = Vector{Int}(
    Tracking.get_correlator_sample_shifts(
        correlator,
        _hz(sampling_freq),
        _hz(code_frequency),
    ),
)

"""
    hardware_support_error(capabilities, signal, correlator, sampling_freq;
                           num_ants, dump_tap_slots) -> Union{Nothing,String}

Every reason `capabilities` cannot serve `signal` tracked with `correlator` at
`sampling_freq`, as one message — or `nothing` when it can.

`correlator` is the host-side `Tracking` correlator the signal is tracked with
(`Tracking.get_default_correlator(signal, num_ants)` unless overridden): its tap
count and preferred shifts are what the device must reproduce, so they are what
is checked. `dump_tap_slots`, when given, is how many accumulator slots the
device's dump record carries — a wire too narrow for the correlator cannot
transport it, which is the same refusal one step earlier in the path.

All reasons are collected rather than reported one at a time: a message that
stops at the first one sends the reader round the loop for each of the rest.
"""
function hardware_support_error(
    capabilities::HardwareCorrelatorCapabilities,
    signal::AbstractGNSSSignal,
    correlator::Tracking.AbstractCorrelator,
    sampling_freq;
    num_ants::Integer = Tracking.get_num_ants(correlator),
    dump_tap_slots::Union{Nothing,Integer} = nothing,
)
    reasons = String[]
    signal_id = get_signal_id(signal)
    if !isnothing(capabilities.signals) && !(signal_id in capabilities.signals)
        push!(
            reasons,
            "the device has no code generator for $signal_id (it declares " *
            "$(join(capabilities.signals, ", ")))",
        )
    end
    modulation = _modulation_name(signal)
    if !isnothing(capabilities.modulations) && !(modulation in capabilities.modulations)
        push!(
            reasons,
            "the device cannot synthesise $modulation modulation (it declares " *
            "$(join(capabilities.modulations, ", ")))",
        )
    end
    code_length = get_code_length(signal)
    if code_length > capabilities.max_primary_code_length
        push!(
            reasons,
            "the primary code is $code_length chips, past the device's " *
            "$(capabilities.max_primary_code_length)-chip code memory",
        )
    end
    code_frequency = _hz(get_code_frequency(signal))
    lo, hi = capabilities.code_frequency_limits
    if code_frequency < lo || code_frequency > hi
        push!(
            reasons,
            "the chip rate $(code_frequency / 1e6) Mcps is outside the device's " *
            "$(lo / 1e6)–$(hi / 1e6) Mcps code NCO range",
        )
    end
    taps = Tracking.get_num_accumulators(correlator)
    if !(taps in capabilities.tap_layouts)
        push!(
            reasons,
            "$signal_id is tracked with a $taps-tap $(nameof(typeof(correlator))), and " *
            "the device's correlator bank produces " *
            "$(join(capabilities.tap_layouts, "/"))-tap layouts",
        )
    end
    if !isnothing(dump_tap_slots) && dump_tap_slots < taps
        push!(
            reasons,
            "the device's dump record carries $dump_tap_slots accumulator slots, too " *
            "few for a $taps-tap correlator — the missing taps cannot be invented on " *
            "the host",
        )
    end
    shifts = _tap_sample_shifts(correlator, sampling_freq, get_code_frequency(signal))
    offset_chips = maximum(abs, shifts) * code_frequency / _hz(sampling_freq)
    if offset_chips > capabilities.max_tap_offset_chips
        push!(
            reasons,
            "the outermost replica sits $(round(offset_chips; digits = 3)) chips from " *
            "prompt, past the device's ±$(capabilities.max_tap_offset_chips)-chip reach",
        )
    end
    if num_ants > capabilities.num_antennas
        push!(
            reasons,
            "$num_ants antennas were requested and the correlator bank despreads " *
            "$(capabilities.num_antennas)",
        )
    end
    band_id = get_band_id(get_band(signal))
    if !isnothing(capabilities.bands) && !(band_id in capabilities.bands)
        push!(
            reasons,
            "the front end cannot tune band $band_id (it declares " *
            "$(join(capabilities.bands, ", ")))",
        )
    end
    isempty(reasons) && return nothing
    "cannot track $signal_id on this hardware correlator:\n" *
    join(map(r -> "  - " * r, reasons), "\n")
end

const _CONTRACT_POINTER =
    "Query `GNSSReceiver.hardware_capabilities(sdr)` for the device's full profile, or " *
    "implement it for this device if it is more capable than the default GPS L1 C/A " *
    "profile. See the \"Hardware-correlator contract\" section of the manual."

"""
    wire_tap_slots(::Type{<:Tracking.AbstractCorrelator}) -> Union{Nothing,Int}

How many accumulator slots a dump record built on this correlator type carries,
known from the type alone — before any dump has arrived, and without an
instance to take a `length` of. `nothing` for a correlator type this package
does not know the width of, which simply skips the wire-width check.

A device's [`correlator_dump_channel`](@ref) fixes one such type for the whole
run, and it has to be wide enough for every correlator the receiver tracks
with: a three-slot record cannot carry a five-tap correlator, and the host
will not invent the missing taps.
"""
wire_tap_slots(::Type{<:Tracking.AbstractCorrelator}) = nothing
wire_tap_slots(::Type{<:Tracking.EarlyPromptLateCorrelator}) = 3
wire_tap_slots(::Type{<:Tracking.VeryEarlyPromptLateCorrelator}) = 5

# ─────────────────────────────────────────────────────────────────────────────
# The complete channel configuration
# ─────────────────────────────────────────────────────────────────────────────

"""
    HardwareChannelConfig

Everything one hardware channel needs to replicate and correlate one signal
component of one satellite — the argument of the modern
[`assign_channel!`](@ref).

The legacy call passed a PRN, three Dopplers and an Early-to-Late spacing, and
left everything else to a shared assumption. This carries the lot:

  - `signal` — the `AbstractGNSSSignal` to replicate, and `signal_index` /
    `group_key` / `prn`, which together identify *which component of which
    satellite* the channel serves. A pilot/data pair occupies two channels that
    differ only in `signal` and `signal_index`.
  - `carrier_doppler` / `code_doppler` (Hz) and `code_phase` (chips) describe
    the satellite at `valid_at_sample`, a count of raw samples the host has
    consumed since the run began — unchanged from the legacy handover contract.
  - `tap_sample_shifts` — **all** quantised replica offsets in whole input
    samples, latest first, prompt at zero: `[-2, 0, 2]` for a three-tap bank,
    `[-2, -1, 0, 1, 2]` for a five-tap one. Program exactly these. They are
    what `Tracking` recovers from the correlator it is handed and normalises
    the discriminators by, so a device that re-derives its own from the
    preferred chip shift introduces a loop-gain error (~2.3 % at 4 MHz and
    0.5 chips) or, for a five-tap bank, an outright wrong VE/VL distance.
  - `el_sample_spacing` — the Early-to-Late distance in samples, i.e.
    `tap_sample_shifts[early] - tap_sample_shifts[late]`. Redundant with the
    shifts and kept because it is the one number the legacy interface carried.
  - `replica_amplitude` — the amplitude of the carrier replica the device
    wipes off with, relative to the unit-amplitude replica a host correlator
    would use, for this channel's band (see [`correlator_gain`](@ref)). The
    ingest divides it out.
  - `code_amplitude` — the RMS amplitude of the *code* replica the device
    correlates with (see [`replica_code_amplitude`](@ref)). The ingest rescales
    to `GNSSSignals.get_code_amplitude(signal)`, so a gateware approximation
    does not move the satellite's C/N₀.
  - `secondary_code_mode` — `:primary_only` (the device replicates the primary
    code and the host removes the overlay from each dump) or `:wipeoff` (the
    device removes it, so its dumps carry none). The link asks for
    `:primary_only` for every device and every signal — an overlay's phase is
    not known when a channel is armed, so the host owns the removal; see
    [`GNSSReceiver.requested_secondary_code_mode`](@ref). What `:primary_only`
    obliges a device to is one record per primary code period, which is the
    dump contract anyway: a record spanning several code periods has summed
    their overlay chips inside the accumulator, where no single sign takes them
    off again.
  - `carrier_phase_offset` — the component's carrier phase against its band's
    in-phase reference, in radians (`GNSSSignals.get_carrier_phase_offset`).
    The device **must not** apply it: it mixes every component of a band
    against one common in-phase carrier, so the ICD phase relationship survives
    into the accumulators, which is what lets the host lock the driver
    component on the real axis and de-rotate the others onto it. A device that
    cannot help rotating per component must remove exactly this value again.
  - `band_id` / `sampling_freq` — which RF band the channel lives on and the
    sample rate its `tap_sample_shifts` and `valid_at_sample` are counted in.
    Per-band, because gain, sample rate and replica offsets are all per-band.

Built by the link from the tracking state; a vendor package only reads it.
"""
struct HardwareChannelConfig{S<:AbstractGNSSSignal}
    signal::S
    signal_index::Int
    group_key::Symbol
    prn::Int
    carrier_doppler::Float64
    code_doppler::Float64
    code_phase::Float64
    valid_at_sample::Int64
    tap_sample_shifts::Vector{Int}
    el_sample_spacing::Int
    replica_amplitude::Float64
    code_amplitude::Float64
    secondary_code_mode::Symbol
    carrier_phase_offset::Float64
    band_id::Symbol
    sampling_freq::Float64
end

function HardwareChannelConfig(
    signal::AbstractGNSSSignal,
    correlator::Tracking.AbstractCorrelator;
    signal_index::Integer,
    group_key::Symbol,
    prn::Integer,
    carrier_doppler,
    code_doppler,
    code_phase,
    valid_at_sample::Integer,
    sampling_freq,
    replica_amplitude::Real = 1.0,
    code_amplitude::Real = get_code_amplitude(signal),
    secondary_code_mode::Symbol = :primary_only,
)
    secondary_code_mode in (:primary_only, :wipeoff) || throw(
        ArgumentError(
            "secondary_code_mode must be :primary_only or :wipeoff " *
            "(got $secondary_code_mode)",
        ),
    )
    code_frequency = get_code_frequency(signal)
    shifts = _tap_sample_shifts(correlator, sampling_freq, code_frequency)
    HardwareChannelConfig(
        signal,
        Int(signal_index),
        group_key,
        Int(prn),
        _hz(carrier_doppler),
        _hz(code_doppler),
        Float64(code_phase),
        Int64(valid_at_sample),
        shifts,
        Int(
            Tracking.get_early_late_sample_spacing(
                correlator,
                _hz(sampling_freq),
                _hz(code_frequency),
            ),
        ),
        Float64(replica_amplitude),
        Float64(code_amplitude),
        secondary_code_mode,
        Float64(get_carrier_phase_offset(signal)),
        get_band_id(get_band(signal)),
        _hz(sampling_freq),
    )
end

# The amplitude scale the ingest divides a channel's accumulators by, so that a
# record off this device lands where the host's own correlator would have put
# it: the carrier replica's amplitude out, and the device's code table rescaled
# to the one GNSSSignals models. See `replica_code_amplitude`.
correlator_output_scale(config::HardwareChannelConfig) =
    config.replica_amplitude * config.code_amplitude / get_code_amplitude(config.signal)
