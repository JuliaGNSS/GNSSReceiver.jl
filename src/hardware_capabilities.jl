# ─────────────────────────────────────────────────────────────────────────────
# What a hardware correlator can do, and what it is asked to do
# (GNSSReceiver.jl #131)
#
# The hardware path (`hardware_device.jl`) was written against one device
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
#   * the arm command a [`RemoteHardwareLoop`](@ref) sends — the complete description of what one
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
    reports_code_phase = true,              # does it latch the replica code phase per record?
    supports_partial_code_dumps = false,    # can it dump *inside* a code period?
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
    declaration: the loop process removes the overlay from the records itself
    once the sync detector has found its phase, so nothing asks a device to do
    it and `1` costs a device nothing. It is what
    [`supports_secondary_code_wipeoff`](@ref) reads, and what a future
    *scheduled* wipeoff contract — which would let a device pre-accumulate
    across code periods — would gate on.
  - `reports_code_phase` — whether the device latches the replica's code phase
    alongside the accumulators (the record's code phase). Informational:
    without it pseudoranges are dead-reckoned from the handover seed rather
    than anchored to the replica the DLL steers, and a partial-primary record
    stream has to dead-reckon its place on the code-block grid from the
    handover instead of reading it off every record.
  - `supports_partial_code_dumps` — whether a channel can be told to dump
    *inside* a primary code period rather than only on the code wrap. `false`
    is the historical contract (one record per code period) and is what every
    device declares until issue #133's steps 2 and 3 land.

    This is a hard requirement rather than a nicety for a long code: GPS L2CL's
    767 250 chips at 511.5 kcps are a 1.5 s period, so a device that only dumps
    on the wrap hands the tracking loops one record and one NCO correction every
    1.5 s. [`hardware_support_error`](@ref) refuses that combination before a
    channel is armed — see its `max_integration_time` keyword for where the
    line is drawn. What the loop process does with a device that can dump inside
    a code period is HardwareLoopCore's record accounting.

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
    supports_partial_code_dumps::Bool
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
    supports_partial_code_dumps::Bool = false,
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
        supports_partial_code_dumps,
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
    supports_partial_code_dumps = false,
)

"""
    DEFAULT_MAX_INTEGRATION_TIME

The longest span of signal the hardware path folds into one correlator record by
default, in seconds: 20 ms, one GPS navigation bit.

It is the line between a signal whose *primary code period* can be the unit of
integration and one whose cannot. Every signal in issue #130's scope but GPS
L2CL has a code period at or below it — 1 ms for GPS L1 C/A and L5, 4 ms for
Galileo E1, 20 ms for GPS L2CM, 64.5 µs for Galileo E5a-QP — and for those the
records the loops are handed are whole code blocks, which is what keeps the
navigation bit grid and the overlay counter exact. L2CL's 1.5 s period is three
orders of magnitude past it, so its records are *partial*: cut inside a code
period, counted as the fraction of one they are, and never as a completed
period.

Both the loop process (its `LoopConfig.max_integration_time`) and
the pre-arm gate ([`hardware_support_error`](@ref)) read it, so a device is
refused for exactly the signals the configured integration length cannot serve.
"""
const DEFAULT_MAX_INTEGRATION_TIME = 20e-3

# One primary code period of `signal` in plain seconds — the receiver's own
# `primary_code_period` with the unit stripped, because the record accounting
# multiplies and compares it on every dump. For a long code it is also the
# quantity that makes "one record per code period" unusable as a
# tracking-update interval: 1 ms for GPS L1 C/A, 1.5 s for GPS L2CL, 64.5 µs
# for Galileo E5a-QP.
code_period_seconds(signal::AbstractGNSSSignal) =
    get_code_length(signal) / _hz(get_code_frequency(signal))

"""
    supports_secondary_code_wipeoff(capabilities, signal) -> Bool

Whether the device can wipe `signal`'s secondary (overlay) code off in the
gateware, so consecutive dumps can be summed without the overlay cancelling
them.

`false` — the default for every device — is not an error, and `true` changes
nothing today: the loop process removes the overlay from each primary-period
record once it knows its phase, so every arm command asks for `:primary_only`
whatever a device declares. The "Secondary codes" section of the
[hardware-correlator contract](@ref "Hardware-correlator contract") says why
ownership sits there, and what the removal unlocks.
"""
supports_secondary_code_wipeoff(
    capabilities::HardwareCorrelatorCapabilities,
    signal::AbstractGNSSSignal,
) =
    get_secondary_code_length(signal) > 1 &&
    get_secondary_code_length(signal) <= capabilities.max_secondary_code_length

# ─────────────────────────────────────────────────────────────────────────────
# RF bands: where each one arrives, and what clock it is counted on (#134)
#
# "Every signal is supported" and "every band is received at once" are different
# claims, and the second one is an RF-capacity statement about a particular
# front end. A [`HardwareBandPlan`](@ref) is where that statement is written
# down: which physical input each requested band comes in on, at what sample
# rate, on which device — and therefore how the device's several free-running
# counters relate to the one timebase the receiver folds, ranges and fixes on.
# ─────────────────────────────────────────────────────────────────────────────

_hz(x::Real) = Float64(x)
_hz(x) = Float64(ustrip(uconvert(Hz, x)))

"""
    HardwareBandRoute(band_id; rf_input, device_index, sampling_freq)

Where one RF band physically arrives and how fast it is counted.

  - `band_id` — the `GNSSSignals.get_band_id` symbol (`:L1`, `:L5`, `:B1I`, …).
    One route per band; a band is what a front end *tunes*.
  - `rf_input` — the device's RF input (tuner / downconversion chain) this band
    comes in on, 1-based. Two independently tuned bands need two inputs, and
    the pre-arm validation refuses a plan that puts two bands on one.
  - `device_index` — which physical device, 1-based, for a multi-device array.
    Anything past `1` needs a declared clock relationship (see
    [`HardwareBandPlan`](@ref)).
  - `sampling_freq` — the rate, in Hz, this band's raw samples *and* its
    correlator dumps' `sample_index` are counted at. It need not equal any other
    band's.

!!! note "An RF input is not an antenna"

    `rf_input` selects a *band*; `HardwareCorrelatorCapabilities.num_antennas`
    counts the coherent chains that band is received on. An N-antenna L1 front
    end is one route with `num_antennas = N`, not N routes: the antennas share
    one LO and one sample clock, and the correlator bank despreads all of them
    into one `SVector{N,Complex}` accumulator per tap so that the host can
    beamform from the prompt covariance. Two *bands* share nothing but the
    board.
"""
struct HardwareBandRoute
    band_id::Symbol
    rf_input::Int
    device_index::Int
    sampling_freq::Float64
end

HardwareBandRoute(
    band_id::Symbol;
    rf_input::Integer = 1,
    device_index::Integer = 1,
    sampling_freq,
) = HardwareBandRoute(band_id, Int(rf_input), Int(device_index), _hz(sampling_freq))

"""
    HardwareBandPlan(routes; clock_synchronization = :single_device)

The receiver's RF configuration: one [`HardwareBandRoute`](@ref) per band, in
order, plus the clock relationship between the devices they live on.

**The first route's band is the reference band, and its sample counter is the
receiver timebase.** Every epoch boundary, every fold, every NCO landing sample
and every code-phase reference epoch is expressed in reference-band samples;
a record arriving on another band is mapped onto that axis by the exact ratio of
the two rates ([`to_receiver_samples`](@ref)), and a command going the other way
is mapped back ([`to_band_samples`](@ref)). Since the ratio is exact and both
counters are derived from one hardware clock, the mapping is a scaling and not
an estimate — which is what lets two bands at 4 and 5 MS/s close the *same*
epoch at the same instant, and what makes the pseudoranges of both bands
referenced to one common reception time.

`clock_synchronization` says what makes that true across `device_index`es:

  - `:single_device` (the default) — one device, one sample clock. Nothing to
    synchronise.
  - `:shared_clock` — several devices driven from one reference and one sample
    clock (a common 10 MHz plus a distributed sample-clock/PPS), with their
    sample counters aligned at start. This is the only multi-device
    configuration the receiver supports, because the timebase mapping above is
    a ratio with no offset term.
  - `:independent` — devices on free-running clocks. Their counters drift
    against each other by parts per million, which is metres of pseudorange per
    second and no bounded relationship for the fold grid at all. The pre-arm
    validation refuses it rather than producing a fix nothing can vouch for; a
    receiver that needs it has to estimate and steer the inter-device offset
    first, which this package does not do.
"""
struct HardwareBandPlan
    routes::Vector{HardwareBandRoute}
    clock_synchronization::Symbol
end

const CLOCK_SYNCHRONIZATIONS = (:single_device, :shared_clock, :independent)

function HardwareBandPlan(routes; clock_synchronization::Symbol = :single_device)
    rs = collect(HardwareBandRoute, routes)
    isempty(rs) && throw(ArgumentError("a band plan needs at least one band route"))
    allunique(map(r -> r.band_id, rs)) || throw(
        ArgumentError(
            "a band plan carries one route per band; got " *
            join(map(r -> r.band_id, rs), ", "),
        ),
    )
    all(r -> r.sampling_freq > 0, rs) ||
        throw(ArgumentError("every band route needs a positive sampling frequency"))
    all(r -> r.rf_input >= 1 && r.device_index >= 1, rs) ||
        throw(ArgumentError("rf_input and device_index are 1-based"))
    clock_synchronization in CLOCK_SYNCHRONIZATIONS || throw(
        ArgumentError(
            "clock_synchronization must be one of " *
            join(CLOCK_SYNCHRONIZATIONS, ", ") *
            " (got $clock_synchronization)",
        ),
    )
    HardwareBandPlan(rs, clock_synchronization)
end

"""
    reference_band(plan) -> Symbol

The band whose sample counter *is* the receiver timebase: the plan's first.
"""
reference_band(plan::HardwareBandPlan) = first(plan.routes).band_id

"""
    reference_sampling_frequency(plan) -> Float64

The receiver timebase's rate in Hz — the reference band's sampling frequency.
"""
reference_sampling_frequency(plan::HardwareBandPlan) = first(plan.routes).sampling_freq

"""
    band_ids(plan) -> Vector{Symbol}

Every band the plan routes, in order, the reference band first.
"""
band_ids(plan::HardwareBandPlan) = map(r -> r.band_id, plan.routes)

"""
    band_route(plan, band_id) -> Union{Nothing,HardwareBandRoute}

The route for `band_id`, or `nothing` when the plan does not carry that band.
"""
function band_route(plan::HardwareBandPlan, band_id::Symbol)
    for route in plan.routes
        route.band_id === band_id && return route
    end
    nothing
end

# The route for `band_id`, falling back to the reference band's. A band the plan
# does not know about is a configuration error the pre-arm gate has already
# refused; on the chunk path the reference route is the conservative answer
# (it is what a single-band receiver has always used) rather than a throw that
# would cost every other satellite its lock.
_route_or_reference(plan::HardwareBandPlan, band_id::Symbol) =
    something(band_route(plan, band_id), first(plan.routes))

"""
    band_sampling_frequency(plan, band_id) -> Float64

The rate in Hz that `band_id`'s samples, dump `sample_index`es and replica tap
offsets are counted at.
"""
band_sampling_frequency(plan::HardwareBandPlan, band_id::Symbol) =
    _route_or_reference(plan, band_id).sampling_freq

"""
    receiver_timebase_scale(plan, band_id) -> Float64

How many receiver-timebase samples one of `band_id`'s samples is worth:
`reference_sampling_frequency(plan) / band_sampling_frequency(plan, band_id)`.
Exactly `1.0` for the reference band, and for every band of a device that
samples all of them at one rate — which is why a single-band receiver's
arithmetic is untouched by any of this.
"""
receiver_timebase_scale(plan::HardwareBandPlan, band_id::Symbol) =
    reference_sampling_frequency(plan) / band_sampling_frequency(plan, band_id)

"""
    to_receiver_samples(plan, band_id, sample) -> Int64

Map a count on `band_id`'s device counter onto the receiver timebase.
"""
to_receiver_samples(plan::HardwareBandPlan, band_id::Symbol, sample) =
    _scale_sample(sample, receiver_timebase_scale(plan, band_id))

"""
    to_band_samples(plan, band_id, sample) -> Int64

Map a count on the receiver timebase onto `band_id`'s own device counter — the
inverse of [`to_receiver_samples`](@ref), and what a handover time or an
an NCO word's landing sample is expressed in before it is handed to
the device.
"""
to_band_samples(plan::HardwareBandPlan, band_id::Symbol, sample) =
    _scale_sample(sample, 1 / receiver_timebase_scale(plan, band_id))

# Scale a sample count, leaving the sentinels alone: `typemin`/`typemax` mark
# "no sample" rather than an instant, and scaling them would turn a sentinel
# into an ordinary (and wrong) index.
@inline function _scale_sample(sample, scale::Float64)
    n = Int64(sample)
    (scale == 1.0 || n == typemin(Int64) || n == typemax(Int64)) && return n
    round(Int64, n * scale)
end

"""
    band_plan_error(capabilities, plan) -> Union{Nothing,String}

Every reason a device with `capabilities` cannot receive `plan`'s bands *at the
same time*, as one message — or `nothing` when it can.

This is the RF-capacity gate, and it is deliberately separate from the
per-signal one: a device may be perfectly able to replicate and correlate every
signal it is asked for and still have exactly one tuner. Supporting every signal
individually is not the same as receiving every band simultaneously, and the
receiver never quietly configures a subset.

The checks:

  - every band is one the front end declares it can tune;
  - no more bands than the device has RF inputs — per device, since that is what
    the declaration counts;
  - no two bands on one RF input of one device;
  - a multi-device plan has a declared clock relationship that makes its
    counters comparable (see [`HardwareBandPlan`](@ref)).

There is no automatic fallback to *sequential retuning* — receiving band A for a
while, then retuning to band B. It would silently turn a simultaneous request
into a time-multiplexed one, which changes the C/N₀, the measurement epochs and
the fix rate of every band involved, and leaves no band continuously tracked.
A receiver that wants it runs one [`receive`](@ref) per band configuration and
retunes between them, which is explicit and is what the error below says.
"""
function band_plan_error(
    capabilities::HardwareCorrelatorCapabilities,
    plan::HardwareBandPlan,
)
    problems = String[]
    bands = band_ids(plan)
    if !isnothing(capabilities.bands)
        unknown = filter(b -> !(b in capabilities.bands), bands)
        isempty(unknown) || push!(
            problems,
            "the front end cannot tune band(s) $(join(unknown, ", ")) (it declares " *
            "$(join(capabilities.bands, ", ")))",
        )
    end
    devices = unique(map(r -> r.device_index, plan.routes))
    for device in devices
        on_device = filter(r -> r.device_index == device, plan.routes)
        where_ = length(devices) == 1 ? "" : " on device $device"
        if length(on_device) > capabilities.num_rf_inputs
            push!(
                problems,
                "cannot receive bands $(join(map(r -> r.band_id, on_device), ", "))" *
                "$where_ at once: the device has $(capabilities.num_rf_inputs) RF " *
                "input(s), which is what caps *simultaneously tuned bands* — " *
                "`num_antennas` ($(capabilities.num_antennas)) counts the coherent " *
                "antenna chains of one band and cannot stand in for it. Sequential " *
                "retuning is not performed automatically: run one receiver per band " *
                "configuration and retune between them",
            )
        end
        inputs = map(r -> r.rf_input, on_device)
        if !allunique(inputs)
            clashing = [
                "$(r.band_id)→input $(r.rf_input)" for
                r in on_device if count(==(r.rf_input), inputs) > 1
            ]
            push!(
                problems,
                "two bands share one RF input$where_ ($(join(clashing, ", "))): an RF " *
                "input is tuned to one band at a time. Declare " *
                "`GNSSReceiver.band_rf_input(sdr, band_id)` for this device",
            )
        end
    end
    if length(devices) > 1 && plan.clock_synchronization === :independent
        push!(
            problems,
            "bands are spread over devices $(join(devices, ", ")) whose clocks are " *
            "declared `:independent`: their sample counters drift against each other, " *
            "so nothing maps them onto one receiver timebase and no common reception " *
            "epoch exists. Drive the devices from one reference and one sample clock " *
            "and declare `GNSSReceiver.clock_synchronization(sdr) = :shared_clock`",
        )
    end
    isempty(problems) && return nothing
    join(problems, "\n")
end

# ─────────────────────────────────────────────────────────────────────────────
# Validation, before anything is armed
# ─────────────────────────────────────────────────────────────────────────────

# Plain seconds from either a `Real` or a Unitful time, so a caller may write
# `20u"ms"` or `0.02` and mean the same thing.
_seconds(x::Real) = Float64(x)
_seconds(x) = Float64(ustrip(uconvert(s, x)))

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

`max_integration_time` is the longest span of signal the receiver will fold into
one record (the loop process's `max_integration_time`, and
[`DEFAULT_MAX_INTEGRATION_TIME`](@ref) here). It is what decides whether the
signal's *primary code period* can be the unit of integration: past it a record
has to be cut inside a code period, which a device that only dumps on the code
wrap cannot do. That is the one check here about timing rather than replicas,
and it is the reason a 1.5 s GPS L2CL code is refused on a device that has the
code memory for it but dumps once per period.

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
    max_integration_time = DEFAULT_MAX_INTEGRATION_TIME,
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
    # Timing, not replicas: one code period is the shortest record a device that
    # dumps only on the code wrap can produce, so for a long code that period is
    # also the loops' update interval. Past the configured integration length
    # that is not tracking, it is an open loop with a heartbeat — refuse it here
    # rather than let a satellite "track" at one correction per 1.5 s.
    period = code_period_seconds(signal)
    max_time = _seconds(max_integration_time)
    if period > max_time * (1 + 1e-9) && !capabilities.supports_partial_code_dumps
        push!(
            reasons,
            "one primary code period of $signal_id is $(round(period; sigdigits = 4)) s, " *
            "past the $(round(max_time * 1e3; sigdigits = 4)) ms this receiver folds " *
            "into one record, and the device dumps only once per code period — so " *
            "every tracking-loop update would wait a whole code period. The device " *
            "has to be able to dump inside one " *
            "(`HardwareCorrelatorCapabilities.supports_partial_code_dumps`)",
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

A device's record format fixes one such width for the whole run, and it has
to be wide enough for every correlator the receiver tracks with: a three-slot
record cannot carry a five-tap correlator, and the loop will not invent the
missing taps.
"""
wire_tap_slots(::Type{<:Tracking.AbstractCorrelator}) = nothing
wire_tap_slots(::Type{<:Tracking.EarlyPromptLateCorrelator}) = 3
wire_tap_slots(::Type{<:Tracking.VeryEarlyPromptLateCorrelator}) = 5
