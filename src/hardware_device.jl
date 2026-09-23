# ─────────────────────────────────────────────────────────────────────────────
# Hardware-correlator devices (GNSSReceiver.jl #107)
#
# Some SDRs downconvert and correlate on their FPGA. Their tracking loops are
# closed by a separate, allocation-free loop process (HardwareLoopCore.jl over
# a vendor driver, e.g. GNSSM2SDR's `gnss_loop`), and this receiver talks to
# it through a `HardwareLoopProtocol` segment — see `remote_hardware_loop.jl`.
# What a vendor package still describes to the receiver is the *device*: its
# raw sample streams (acquisition, decoding and PVT run on them here), its
# declared capabilities and RF band plan, its replica amplitudes, and the
# origin of its sample counter. That device-side contract is this file.
#
# The raw sample stream is not replaced: it drives acquisition, the receiver's
# clock, decoding and PVT exactly as in the software receiver. Only the
# correlation and the loop closure live on the device and in its loop process.
# ─────────────────────────────────────────────────────────────────────────────

"""
    AbstractHardwareCorrelatorSDR

A software-defined radio whose FPGA correlates and whose tracking loops are
closed by a separate loop process. Subtype it to describe such a device to the
receiver, then hand it to [`RemoteHardwareLoop`](@ref) and
`receive(loop, systems, sampling_freq)`.

Required: [`raw_sample_channel`](@ref) (the raw I/Q stream acquisition runs on;
per band for a multi-band device) and [`num_hardware_channels`](@ref). Declare
what the gateware can do with [`hardware_capabilities`](@ref) — every
configured signal is validated against it before anything is armed — and, for
a multi-band device, the RF routing through [`band_rf_input`](@ref),
[`band_hardware_channels`](@ref), [`band_device_index`](@ref) and
[`clock_synchronization`](@ref). [`correlator_gain`](@ref) and
[`replica_code_amplitude`](@ref) state the replica amplitudes the loop divides
out of every accumulator, and [`device_sample_origin`](@ref) maps host raw
sample 0 onto the device counter, so an acquisition's code phase can be armed
at the right device sample.

The device's records and NCO words never pass through this process: the loop
process reads and writes them and publishes what the receiver needs — epoch
states, records, bits, status — through the protocol segment.
"""
abstract type AbstractHardwareCorrelatorSDR end

_not_implemented(f, sdr) = throw(
    ArgumentError(
        "$(typeof(sdr)) is an AbstractHardwareCorrelatorSDR but does not implement " *
        "GNSSReceiver.$f. See `AbstractHardwareCorrelatorSDR` for the required interface.",
    ),
)

"""
    raw_sample_channel(sdr::AbstractHardwareCorrelatorSDR) -> SignalChannel

The device's raw sample stream. Required; see
[`AbstractHardwareCorrelatorSDR`](@ref).
"""
raw_sample_channel(sdr::AbstractHardwareCorrelatorSDR) =
    _not_implemented("raw_sample_channel", sdr)

"""
    raw_sample_channel(sdr, band_id::Symbol) -> SignalChannel

One RF band's raw sample stream. The default is the device's single stream, so
a one-band device implements nothing; a device that receives several bands at
once returns a separate stream per band, each counted at that band's own
[`band_sampling_frequency`](@ref) and each driving its own band's acquisition.

The streams stay separate rather than being interleaved for the same reason the
dumps and the raw samples do: they have different rates, and the receiver's
acquisition, buffering and code-phase correction are all per band already.
"""
raw_sample_channel(sdr::AbstractHardwareCorrelatorSDR, ::Symbol) = raw_sample_channel(sdr)

"""
    band_rf_input(sdr, band_id) -> Int

Which RF input (tuner / downconversion chain) of the device `band_id` arrives
on, 1-based. The default is `1`, which is right for every single-band device.

A device that can receive several bands at once **must** implement this: the
receiver will not guess which tuner a band lands on, and refuses a plan that
puts two bands on one input rather than configuring a subset in silence. An RF
input is not an antenna — see [`HardwareBandRoute`](@ref).
"""
band_rf_input(::AbstractHardwareCorrelatorSDR, ::Symbol) = 1

"""
    band_hardware_channels(sdr, band_id) -> AbstractVector{Int}

Which of the device's hardware channels can correlate `band_id` — its
*correlator bank* for that band, as 1-based indices into
[`num_hardware_channels`](@ref).

The default is every channel, which is right for a single-band device and for a
multi-band one whose bank can be pointed at any input. A device whose replica
sets are wired to one downconversion chain each returns that chain's slice, and
the loop then only ever hands a band's satellites a channel that can actually see
it — the raw acquisition stream and the correlator bank that serves it are the
same front end.

Getting this wrong is not a subtle failure: a channel of the wrong bank
correlates the *other* band's samples with this band's replica, produces records
that never rise above the noise, and the satellite is dropped as if it had faded.

The returned ranges must not overlap between bands.
"""
band_hardware_channels(sdr::AbstractHardwareCorrelatorSDR, ::Symbol) =
    Base.OneTo(Int(num_hardware_channels(sdr)))

"""
    band_device_index(sdr, band_id) -> Int

Which physical device of a multi-device array `band_id` arrives on, 1-based.
The default is `1`. Anything else needs a declared
[`clock_synchronization`](@ref).
"""
band_device_index(::AbstractHardwareCorrelatorSDR, ::Symbol) = 1

"""
    clock_synchronization(sdr) -> Symbol

How the sample clocks of the devices this adapter drives relate to each other:
`:single_device` (the default), `:shared_clock` or `:independent`. See
[`HardwareBandPlan`](@ref) for what each obliges and why `:independent` is
refused for a multi-device plan.
"""
clock_synchronization(::AbstractHardwareCorrelatorSDR) = :single_device

"""
    band_bank_error(sdr, plan) -> Union{Nothing,String}

Every reason `sdr`'s correlator banks cannot serve `plan`'s bands, as one
message — or `nothing` when they can.

A band whose bank ([`band_hardware_channels`](@ref)) holds no channel in range
is a band the receiver can tune and then never track anything on: its satellites
are acquired, find no channel of their own bank, and wait unassigned while
their lock detectors quietly release them. Two
bands sharing a channel is the same failure wearing the other hat — whichever
band claims it first leaves the other one short, and which one that is depends
on acquisition order.

Both are exactly the silent partial configuration this gate exists to prevent,
so they are refused before anything is armed rather than discovered as a band
that never locks.
"""
function band_bank_error(sdr::AbstractHardwareCorrelatorSDR, plan::HardwareBandPlan)
    total = Int(num_hardware_channels(sdr))
    problems = String[]
    banks = map(plan.routes) do route
        channels = Int[
            hw_channel for
            hw_channel in Base.invokelatest(band_hardware_channels, sdr, route.band_id) if
            1 <= hw_channel <= total
        ]
        isempty(channels) && push!(
            problems,
            "band $(route.band_id) has no correlator channel: " *
            "`GNSSReceiver.band_hardware_channels(sdr, :$(route.band_id))` names none of " *
            "the device's $total channels, so nothing could ever be armed on that band",
        )
        route.band_id => channels
    end
    for i in eachindex(banks), j = (i+1):lastindex(banks)
        shared = intersect(last(banks[i]), last(banks[j]))
        isempty(shared) || push!(
            problems,
            "bands $(first(banks[i])) and $(first(banks[j])) claim the same correlator " *
            "channel(s) $(join(shared, ", ")): a channel belongs to one band's bank, and " *
            "whichever band took it first would leave the other short",
        )
    end
    isempty(problems) && return nothing
    join(problems, "\n")
end

"""
    hardware_band_plan(sdr, band_ids, sampling_freqs) -> HardwareBandPlan

The device's RF configuration for the requested bands: one
[`HardwareBandRoute`](@ref) per band, in the order given — so the **first band
is the reference band and its counter is the receiver timebase** — plus the
device's declared [`clock_synchronization`](@ref).

The default builds it from [`band_rf_input`](@ref) and
[`band_device_index`](@ref), which is all a device usually has to declare.
Override the whole function only where the routing cannot be expressed per band
(a device that swaps inputs depending on the combination requested, say).

`sampling_freqs` is aligned with `band_ids`; a single frequency applies to every
band, which is the common case of one sample clock feeding several tuners.
"""
function hardware_band_plan(sdr::AbstractHardwareCorrelatorSDR, band_ids, sampling_freqs)
    ids = collect(Symbol, band_ids)
    freqs =
        sampling_freqs isa Union{Tuple,AbstractVector} ? collect(map(_hz, sampling_freqs)) :
        fill(_hz(sampling_freqs), length(ids))
    length(freqs) == length(ids) || throw(
        ArgumentError(
            "hardware_band_plan needs one sampling frequency per band (got " *
            "$(length(freqs)) for $(length(ids)) bands)",
        ),
    )
    HardwareBandPlan(
        map(ids, freqs) do band_id, sampling_freq
            HardwareBandRoute(
                band_id,
                Int(Base.invokelatest(band_rf_input, sdr, band_id)),
                Int(Base.invokelatest(band_device_index, sdr, band_id)),
                sampling_freq,
            )
        end;
        clock_synchronization = Base.invokelatest(clock_synchronization, sdr),
    )
end

"""
    num_hardware_channels(sdr::AbstractHardwareCorrelatorSDR) -> Int

How many hardware tracking channels (replica sets) the gateware provides.
Required; see [`AbstractHardwareCorrelatorSDR`](@ref).
"""
num_hardware_channels(sdr::AbstractHardwareCorrelatorSDR) =
    _not_implemented("num_hardware_channels", sdr)

"""
    correlator_gain(sdr) -> Real

Amplitude of the replica the device wipes the carrier off with, relative to the
unit-amplitude replica a host correlator would use on the same samples. The
ingest divides it out, so a satellite's prompt lands on the same scale as the
raw samples it was correlated from.

A device that mixes with a `±127` sine/cosine table returns `127`; one that
normalises in the gateware returns the default `1`. It is a pure scale, so the
discriminators and any moment-ratio C/N₀ estimator cannot see it — but
`Tracking`'s noise-referenced C/N₀ divides the prompt power by a floor measured
from the raw samples, and there a wrong gain is a `20·log10(g)` dB offset on
every satellite. Getting it wrong is therefore visible only as a uniform C/N₀
bias, which is exactly the kind of error a lock-detector threshold silently
absorbs, so declare it rather than leaving it at the default.

    correlator_gain(sdr, band_id) -> Real

The same thing for one RF band (`GNSSSignals.get_band_id`: `:L1`, `:L5`, …),
defaulting to the device-wide value. A multi-band front end rarely has one
scale: each band has its own gain chain and its own replica table, and a C/N₀
referenced to raw-sample power is biased by `20·log10(g)` per band. Declare it
per band and every band's satellites land on the same scale.

Read once per assignment — the arm command carries the result as its replica
amplitude — so a device may compute it rather than store it.
"""
correlator_gain(::AbstractHardwareCorrelatorSDR) = 1
correlator_gain(sdr::AbstractHardwareCorrelatorSDR, ::Symbol) = correlator_gain(sdr)

"""
    hardware_capabilities(sdr) -> HardwareCorrelatorCapabilities

What `sdr`'s gateware can replicate and correlate. Optional; the default is
[`LEGACY_GPS_L1CA_CAPABILITIES`](@ref), i.e. "the GPS L1 C/A device this
interface was written against".

Declare it as soon as a device does anything else — the receiver validates
every configured signal against it before it arms a channel
([`validate_hardware_configuration`](@ref)), so an undeclared capability is a
capability the receiver will refuse to use, and an over-declared one is a
channel that never locks.
"""
hardware_capabilities(::AbstractHardwareCorrelatorSDR) = LEGACY_GPS_L1CA_CAPABILITIES

supports_secondary_code_wipeoff(sdr::AbstractHardwareCorrelatorSDR, signal) =
    supports_secondary_code_wipeoff(hardware_capabilities(sdr), signal)

"""
    replica_code_amplitude(sdr, signal) -> Real

Per-sample RMS amplitude of the *code* replica `sdr`'s gateware correlates
`signal` with, on the same scale `GNSSSignals.get_code_amplitude` reports for
the host's own table.

The default is `get_code_amplitude(signal)`: the device reproduces the modelled
code exactly, which is true for every ±1 code (BPSK, BOC, TMBOC) and is the only
case the legacy L1 C/A path had. It is *not* true where the gateware
approximates — a device that replicates Galileo E1B with a plain ±1 BOC(1,1)
replica has a code amplitude of `1` where GNSSSignals' multi-level CBOC table
has ≈ 19.92 — and there the ingest has to rescale, or the same satellite reads
~26 dB apart depending on which correlator produced it.

This is a pure amplitude convention: the ingest divides every accumulator by
`replica_code_amplitude(sdr, signal) / get_code_amplitude(signal)`, so the
prompt reaching `Tracking` is always on the host table's scale and
`Tracking.normalize`'s own division by `get_code_amplitude` lands on a
modulation-independent, unit-power amplitude. It says nothing about the *shape*
of an approximated correlation function; which approximations are usable at all
is issue #135's matrix.
"""
replica_code_amplitude(::AbstractHardwareCorrelatorSDR, signal::AbstractGNSSSignal) =
    get_code_amplitude(signal)

"""
    check_hardware_support(sdr, signal, sampling_freq; correlator, num_ants, dump_tap_slots) -> nothing

Throw an `ArgumentError` naming `sdr` and every reason it cannot track `signal`
at `sampling_freq`, or return `nothing` when it can. The single-signal form of
[`validate_hardware_configuration`](@ref).

`correlator` defaults to the one `Tracking` tracks `signal` with; pass it when
the receiver is configured with another.
"""
function check_hardware_support(
    sdr::AbstractHardwareCorrelatorSDR,
    signal::AbstractGNSSSignal,
    sampling_freq;
    correlator::Tracking.AbstractCorrelator = Tracking.get_default_correlator(
        signal,
        NumAnts(1),
    ),
    num_ants::Integer = Tracking.get_num_ants(correlator),
    dump_tap_slots::Union{Nothing,Integer} = nothing,
    max_integration_time = DEFAULT_MAX_INTEGRATION_TIME,
)
    message = hardware_support_error(
        hardware_capabilities(sdr),
        signal,
        correlator,
        sampling_freq;
        num_ants,
        dump_tap_slots,
        max_integration_time,
    )
    isnothing(message) && return nothing
    throw(ArgumentError("$(nameof(typeof(sdr))) $message\n" * _CONTRACT_POINTER))
end

"""
    validate_hardware_configuration(sdr, systems, sampling_freq; num_ants) -> nothing

Check every signal the receiver would track against `sdr`'s declared
[`hardware_capabilities`](@ref) and throw one `ArgumentError` listing every
problem — *before* a channel is armed, before a single CSR is written.

`systems` is what [`receive`](@ref) was given (a signal, a
[`CombinedSignal`](@ref), a tuple of them sharing one band, or a tuple of such
tuples, one per band); each system's components are checked one by one against
its **own band's** sampling frequency, so a pilot/data pair is accepted only if
the device can serve both. The device's dump record is checked too: a three-slot
record cannot carry a five-tap correlator, which is the failure this validation
exists to replace — at PR #129's head that combination reached the ingest path
and died there as a `DimensionMismatch` (issue #131).

`sampling_freq` is one frequency for every band, or a tuple aligned with the
band groups. `band_plan` is the RF configuration to check
([`hardware_band_plan`](@ref) builds the device's own); its bands, RF inputs,
devices and clock relationship are validated as a whole, because receiving every
requested band *at once* is an RF-capacity question the per-signal checks cannot
answer (see [`band_plan_error`](@ref)).

This is the pre-arm gate; [`receive`](@ref)`(::AbstractHardwareCorrelatorSDR, …)`
calls it for you. Call it directly when building a link by hand.
"""
function validate_hardware_configuration(
    sdr::AbstractHardwareCorrelatorSDR,
    systems,
    sampling_freq;
    num_ants::NumAnts{N} = NumAnts(1),
    max_integration_time = DEFAULT_MAX_INTEGRATION_TIME,
    band_plan::Union{Nothing,HardwareBandPlan} = nothing,
) where {N}
    capabilities = hardware_capabilities(sdr)
    band_systems = _band_system_groups(systems)
    plan = something(
        band_plan,
        hardware_band_plan(
            sdr,
            map(systems -> get_band_id(system_band(first(systems))), band_systems),
            _per_band_values(sampling_freq, band_systems),
        ),
    )
    dump_tap_slots = nothing
    problems = String[]
    for systems in band_systems, system in systems
        band_freq = band_sampling_frequency(plan, get_band_id(system_band(system)))
        for signal in tracking_signals(system)
            message = hardware_support_error(
                capabilities,
                signal,
                Tracking.get_default_correlator(signal, num_ants),
                band_freq;
                num_ants = N,
                dump_tap_slots,
                max_integration_time,
            )
            isnothing(message) || push!(problems, message)
        end
    end
    rf_problem = band_plan_error(capabilities, plan)
    isnothing(rf_problem) || push!(problems, rf_problem)
    bank_problem = band_bank_error(sdr, plan)
    isnothing(bank_problem) || push!(problems, bank_problem)
    isempty(problems) && return nothing
    throw(
        ArgumentError(
            "$(nameof(typeof(sdr))) " * join(problems, "\n") * "\n" * _CONTRACT_POINTER,
        ),
    )
end

# One hardware channel's current occupant. `signal_index` addresses the
# component within the satellite's `tracking_signals` tuple, so a pilot/data
# pair simply occupies two hardware channels.
struct HardwareChannelAssignment
    group_key::Symbol
    prn::Int
    signal_index::Int
end

# Whether the satellite a channel was armed for is still in the tracking state.
# A channel whose occupant the receiver has dropped is released.
function _is_tracked(track_state, assignment::HardwareChannelAssignment)
    sat_states = get_sat_states(track_state, assignment.group_key)
    haskey(sat_states, assignment.prn)
end

# ─────────────────────────────────────────────────────────────────────────────
# The dispatch seam: how one chunk advances the tracking state
# ─────────────────────────────────────────────────────────────────────────────

"""
    advance_tracking!(correlator_source, band_measurements, track_state, band_systems) -> TrackState

Advance `track_state` by one processing chunk and return it. This is the single
point where the software and hardware-correlator receivers differ; everything
around it — acquisition, lock detection, decoding, PVT — is shared.

The software method takes any `Tracking` downconvert-and-correlator backend and
simply calls `track!`, which correlates the raw chunk itself. The
[`RemoteHardwareLoop`](@ref) method ignores the samples for tracking purposes
(the FPGA already correlated them and the loop process closed the loops) and
instead mirrors the loop process's events into the tracking state.
"""
advance_tracking!(
    downconvert_and_correlator,
    band_measurements,
    track_state,
    band_systems,
) = track!(band_measurements, track_state; downconvert_and_correlator)

"""
    is_observation_gap(correlator_source, track_state, group_key, prn) -> Bool

Whether this chunk delivered **no measurement at all** for the satellite, in a
way that says nothing about the signal — so the receiver freezes its lock
detectors instead of letting them decay through it.

`false` for a software correlator: there the chunk's samples *are* the
measurement, and a chunk that produces no record produced none because the
signal was not there. A [`RemoteHardwareLoop`](@ref) answers `true` for a
satellite whose records were delayed on their way from the device, within a
bounded gap — the device correlates whether or not the host is listening, and
its own logs showed the FPGA holding satellites through gaps of seconds on the
last NCO word (issue #107). Past the gap budget the path is not late but
broken, and the detectors are allowed to decay so the satellite is eventually
released rather than held in a lock nothing confirms.

!!! warning "Wrappers must forward this"
    A correlator source that *wraps* a loop — an instrumented source that
    forwards `advance_tracking!`, say — matches the fallback below, not the
    loop's method, and its satellites silently lose the protection. Forward
    `is_observation_gap`, `has_current_observations` and
    `take_bit_clock_restart!` explicitly.
"""
is_observation_gap(correlator_source, track_state, group_key, prn) = false

# Lock detectors may coast through a transport gap, but navigation must not use
# the frozen decoder bit count with a code phase that keeps wrapping: a source
# says whether it delivered records on both the ranging and the data component
# this chunk. The software path retains its normal measurement cadence.
has_current_observations(source, track_state, system, prn) = true

"""
    take_bit_clock_restart!(correlator_source, group_key, prn) -> Bool

Whether the source restarted this satellite's bit clock since the receiver last
asked, clearing the flag. The receiver restarts the satellite's decoder in
response, because the bit stream it was decoding has been cut. `false` for every
source but a [`RemoteHardwareLoop`](@ref) whose loop process lost records.
"""
take_bit_clock_restart!(correlator_source, group_key, prn) = false

# Samples in this chunk, counted on the **receiver timebase** — so, on the
# reference band's frame. Every band advances from frames of one duration on one
# time base, but not necessarily of one length: a band sampled faster delivers
# proportionally more samples for the same span. The reference band is the one
# `samples_consumed` (and with it every handover time) is counted in, so it is
# the one that speaks here.
_chunk_num_samples(band_measurements::NamedTuple) =
    _chunk_num_samples(first(values(band_measurements)))
_chunk_num_samples(m::Tracking.BandMeasurement) = size(Tracking.get_samples(m), 1)

# The reference despreads one signal of *this band*, and it is the same one the
# band's handovers are referenced to: the ranging signal of its first system.
# Per band, because a noise density belongs to one front end — see the
# `noise_channels` field.
function _noise_reference_signal(systems)
    for system in systems
        for signal in tracking_signals(system)
            return signal
        end
    end
    nothing
end

# Per-band sampling frequency for a system, read off the `BandMeasurement` the
# chunk was built with so the ingest path and the estimator can never disagree.
_band_sampling_frequency(band_measurements::NamedTuple, system) =
    get_sampling_frequency(band_measurements[get_band_id(system_band(system))])
