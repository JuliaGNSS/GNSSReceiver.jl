# ─────────────────────────────────────────────────────────────────────────────
# Hardware-correlator SDRs (GNSSReceiver.jl #107)
#
# Some SDRs do the downconversion and correlation on the FPGA and stream
# *correlator dumps* to the host, which then runs only the tracking loop filters
# and pushes NCO updates back. This file holds the vendor-agnostic half of that
# split: the record types, the abstract SDR type its accessors, and the
# dump-driven tracking step. The link to a concrete device — DMA drain, CSR
# writes, gateware channel setup — lives in a separate vendor package.
#
# The hardware correlator is an *addition* to the raw sample stream, not a
# replacement for it. Raw samples keep flowing on the existing `SignalChannel`
# and keep driving acquisition, decoding, PVT and the receiver's runtime clock
# exactly as in the software receiver; the only thing that changes is where a
# chunk's correlator outputs come from. That is not just a compatibility
# choice — on a device that taps its own RX datapath, the correlators only see
# samples while the raw DMA is draining, so the raw stream has to keep running
# for the hardware to correlate at all.
#
# The swap is one dispatch: `process` asks a *correlator source* to advance the
# tracking state by one chunk (`advance_tracking!`). A `Tracking`
# downconvert-and-correlator backend correlates the raw chunk itself; a
# [`HardwareCorrelatorLink`](@ref) instead ingests the dumps the FPGA already
# produced and folds them. Nothing else in the pipeline knows the difference.
# ─────────────────────────────────────────────────────────────────────────────

"""
    CorrelatorDump{C}

One correlator dump streamed from a hardware correlator: Tracking's
`CorrelatorOutput` plus the routing needed
to get it back to the right satellite.

  - `channel` — the hardware channel that produced it (1-based), or
    [`EPOCH_STROBE_CHANNEL`](@ref) for a timebase marker.
  - `prn` — the PRN the channel was correlating. Carried for validation: a
    dump that arrives after its channel was reassigned is stale and dropped.
  - `output` — `(correlator, integrated_samples, sample_index)`. Its
    `correlator` is an `EarlyPromptLateCorrelator` whose accumulators are
    ordered *latest first* — `[late, prompt, early]` — because
    `get_prompt_index` is 2. Building it in E/P/L order inverts the sign of the
    DLL discriminator and the loop never converges. `sample_index` is the
    device's free-running sample counter at the end of the integration; it is
    the epoch tag the whole ingest path is clocked on.

Only the accumulators and the two counts are read off the wire. The
correlator's *spacing* metadata is replaced on ingest with the tracked
satellite's, i.e. with the spacing `assign_channel!` programmed — so a vendor
does not have to reproduce `preferred_early_late_to_prompt_code_shift` and
cannot silently mis-normalise the DLL by getting it wrong. Construct the
correlator however is convenient; only the values and their order matter.

  - `code_phase` — the device replica's code phase in chips (modulo the primary
    code length) at `output.sample_index`, or `NaN` when the device does not
    report it. A device that dumps on the sample completing a code period
    reports a value just below the code length (e.g. `1022 + frac` for GPS
    L1 C/A). This is the *absolute pseudorange anchor*: with it the host's
    per-satellite code-phase bookkeeping is re-anchored to the replica the DLL
    actually steers on every dump, so neither the handover seed error nor the
    device NCO's fixed-point quantisation can drift the host's absolute code
    phase — and PVT stays honest. Without it (`NaN`) the host can only dead
    reckon from the acquisition seed, which is fine for tracking but degrades
    the pseudoranges; report it if the hardware can.

  - `num_taps` — how many of the wire correlator's accumulator slots this
    record actually filled, counted from the first. Defaults to all of them,
    which is what a device serving one tap layout wants.

    This is what lets **one** link carry both E/P/L and VE/E/P/L/VL records: a
    device that serves both fixes its stream's correlator type at the *widest*
    layout it produces, and a three-tap channel fills the leading three slots
    and says `num_taps = 3`. The meaningful taps are ordered latest-first with
    prompt at `div(num_taps - 1, 2) + 1` — `Tracking`'s own rule — so a
    three-tap record reads `[late, prompt, early]` and a five-tap one
    `[very late, late, prompt, early, very early]`, whatever the wire is wide
    enough for. The trailing slots are not read and need not be zeroed.

    The host does *not* invent what is missing in either direction: a record
    whose `num_taps` does not match the tap count of the correlator its
    satellite is tracked with is dropped and counted, because a three-tap
    record reshaped into a five-tap correlator would hand `dll_disc` two
    accumulators that never saw a replica. Which layouts a device may produce
    at all is declared through
    [`HardwareCorrelatorCapabilities`](@ref)`.tap_layouts` and checked before
    anything is armed.

`isbits` (so a `PipeChannel{CorrelatorDump{C}}` ring stays allocation-free)
provided `C` is. Use integer accumulators off the FPGA and let
`integrated_samples` do the float normalisation on the host.

For N antennas the accumulator element type is `SVector{N,Complex}` — the device
must stream *per-antenna* accumulators, because beamforming
([`EigenBeamformer`](@ref)) is post-correlation on the CPU and adapts from the
per-antenna prompt covariance. Pre-combining in hardware would kill it.
"""
struct CorrelatorDump{C<:Tracking.AbstractCorrelator}
    channel::Int32
    prn::Int32
    output::Tracking.CorrelatorOutput{C}
    code_phase::Float64
    num_taps::Int32
end

CorrelatorDump(
    channel::Integer,
    prn::Integer,
    output::Tracking.CorrelatorOutput,
    code_phase::Real = NaN,
    num_taps::Integer = Tracking.get_num_accumulators(output.correlator),
) = CorrelatorDump(Int32(channel), Int32(prn), output, Float64(code_phase), Int32(num_taps))

"""
    num_correlator_taps(dump::CorrelatorDump) -> Int

How many of `dump`'s accumulator slots carry a correlation, counted from the
first. See [`CorrelatorDump`](@ref) for the ordering and for why the rest are
neither read nor invented.
"""
num_correlator_taps(dump::CorrelatorDump) = Int(dump.num_taps)

"""
    EPOCH_STROBE_CHANNEL

Sentinel `channel` marking a [`CorrelatorDump`](@ref) as an *epoch strobe*: a
periodic timebase marker on the device's shared sample counter rather than a
real correlation result. See [`epoch_strobe`](@ref).
"""
const EPOCH_STROBE_CHANNEL = Int32(-1)

"""
    is_epoch_strobe(dump::CorrelatorDump) -> Bool

Whether `dump` is a timebase marker rather than a correlation result. Strobes
advance the host's epoch clock and are never appended to a satellite.
"""
is_epoch_strobe(dump::CorrelatorDump) = dump.channel == EPOCH_STROBE_CHANNEL

"""
    epoch_strobe(correlator_prototype, sample_index) -> CorrelatorDump

Build an epoch strobe carrying `sample_index` on the device's free-running
sample counter. `correlator_prototype` only fixes the record's type parameter —
its accumulators are zeroed and never read.

A device should emit these at a fixed period regardless of what its channels are
doing. Without them the host's epoch clock stalls whenever every channel falls
silent (nothing locked yet, or every satellite just lost), because epochs are
closed by *observing* a record past the boundary. With them, epochs keep closing
on the device's own timebase.
"""
epoch_strobe(correlator_prototype::Tracking.AbstractCorrelator, sample_index::Integer) =
    CorrelatorDump(
        EPOCH_STROBE_CHANNEL,
        Int32(0),
        Tracking.CorrelatorOutput(zero(correlator_prototype), 0, Int(sample_index)),
        NaN,
    )

"""
    NCOUpdate

One host → device NCO correction for a hardware channel.

`carrier_doppler` and `code_doppler` are plain `Float64` in **Hz** (not Unitful
quantities) so the record stays `isbits`; the constructor accepts Unitful
frequencies and converts. `apply_at_sample` is the device sample index at which
the update takes effect — the start of the target epoch on the same
free-running counter that tags every [`CorrelatorDump`](@ref).

Scheduling the update at a *named* sample rather than "as soon as it arrives"
is what makes the feedback delay deterministic: the correction computed from
epoch `k` lands at `k + n` for a fixed `n` chosen by the host, instead of
whenever PCIe happens to deliver it. The loop filter can then account for `n`.

!!! warning "Updates overlap: a device must never let one cancel another"
    One update per assigned channel is pushed per folded epoch, so at a 1 kHz
    fold rate the next update reaches the device ~1 ms after the last while
    `apply_at_sample` is one or more epochs ahead — the successor **always**
    arrives before its predecessor is due. A device that stages a commit in a
    single register set (a common gateware shape, e.g. the LiteX-M2SDR's
    `apply_at`/`arm` pair) must therefore not treat arming as a queue: if a new
    arm replaces a pending commit, no NCO word ever reaches the replicas and
    every channel silently free-runs on its handover values while the loops look
    healthy on the host. That failure cost a season of "tracks, then walks off
    and never decodes" in issue #107.

    A vendor package has three sound options, in order of preference: apply the
    words at `apply_at_sample` from a real queue; apply them *immediately* on
    arrival and accept the transport jitter (correct for a rate-only correction —
    a small unknown delay beats a correction that never lands, and this is what
    GNSSM2SDR does, reserving the scheduled path for sample-exact handovers); or
    reject the update and report it, so the receiver sees the loop is open.
    Silently dropping either the new or the pending update is the one
    unacceptable choice.
"""
struct NCOUpdate
    channel::Int32
    prn::Int32
    carrier_doppler::Float64
    code_doppler::Float64
    apply_at_sample::Int64
end

NCOUpdate(channel::Integer, prn::Integer, carrier_doppler, code_doppler, apply_at_sample) =
    NCOUpdate(
        Int32(channel),
        Int32(prn),
        Float64(ustrip(uconvert(Hz, carrier_doppler))),
        Float64(ustrip(uconvert(Hz, code_doppler))),
        Int64(apply_at_sample),
    )

# ─────────────────────────────────────────────────────────────────────────────
# The vendor extension point
# ─────────────────────────────────────────────────────────────────────────────

"""
    AbstractHardwareCorrelatorSDR

Supertype for an SDR whose FPGA downconverts and correlates on-device. A vendor
package subtypes this and implements the interface below; nothing
device-specific belongs in GNSSReceiver.

Required:

  - [`raw_sample_channel`](@ref)`(sdr)` — the `SignalChannel` of raw samples.
    Acquisition, decoding, PVT and the runtime clock all still run off this, so
    it must keep streaming for the whole run.
  - [`correlator_dump_channel`](@ref)`(sdr)` — `PipeChannel{CorrelatorDump{C}}`,
    device → host.
  - [`nco_update_channel`](@ref)`(sdr)` — `PipeChannel{NCOUpdate}`, host →
    device. The same SPSC shape reversed: the tracking loop produces, the
    vendor's writer task consumes.
  - [`num_hardware_channels`](@ref)`(sdr)` — how many replica sets the gateware
    has.
  - [`assign_channel!`](@ref) / [`release_channel!`](@ref) — acquisition
    handover and loss of lock.

Optional:

  - [`dropped_dump_count!`](@ref)`(sdr)` — surface (and clear) a ring overflow.
    Defaults to `0`, i.e. "this device cannot tell"; implement it if it can,
    because a silently dropped dump is a silently corrupted loop.

A vendor package needs no `PrecompileTools` workload of its own for the receive
pipeline. Nothing GNSSReceiver compiles for a hardware receiver depends on the
device's type — [`HardwareCorrelatorLink`](@ref) carries the correlator type
only — and every call into this interface goes through `invokelatest`, so
defining these methods does not invalidate the pipeline either. The package
precompiles the whole hardware path against an internal stub device, and that is
what a real device runs.

Dumps and raw samples are deliberately *separate* streams rather than one fused
element type: they have very different rates and lifecycles (tiny-continuous vs
huge-periodic), and fusing them would force the full-rate raw stream to ride the
tracking cadence — wasting exactly the PCIe bandwidth that correlating on the
FPGA is meant to save. They are aligned by the shared free-running sample
counter, not by bundling.
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
the link then only ever hands a band's satellites a channel that can actually see
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
    correlator_dump_channel(sdr::AbstractHardwareCorrelatorSDR) -> PipeChannel{<:CorrelatorDump}

The device → host stream of correlator dumps. Required; see
[`AbstractHardwareCorrelatorSDR`](@ref).
"""
correlator_dump_channel(sdr::AbstractHardwareCorrelatorSDR) =
    _not_implemented("correlator_dump_channel", sdr)

"""
    nco_update_channel(sdr::AbstractHardwareCorrelatorSDR) -> PipeChannel{NCOUpdate}

The host → device stream of NCO corrections. Required; see
[`AbstractHardwareCorrelatorSDR`](@ref).
"""
nco_update_channel(sdr::AbstractHardwareCorrelatorSDR) =
    _not_implemented("nco_update_channel", sdr)

"""
    num_hardware_channels(sdr::AbstractHardwareCorrelatorSDR) -> Int

How many hardware tracking channels (replica sets) the gateware provides.
Required; see [`AbstractHardwareCorrelatorSDR`](@ref).
"""
num_hardware_channels(sdr::AbstractHardwareCorrelatorSDR) =
    _not_implemented("num_hardware_channels", sdr)

"""
    assign_channel!(sdr, hw_channel, prn, carrier_doppler, code_doppler, code_phase,
                    valid_at_sample; el_sample_spacing, signal)

Hand a freshly acquired satellite over to hardware channel `hw_channel`
(1-based). The vendor package programs the gateware's carrier and code NCOs,
loads the PRN code and starts correlating.

`carrier_doppler` and `code_doppler` are Unitful frequencies and `code_phase` is
in chips. All three describe the satellite **at `valid_at_sample`**, a count of
raw samples the host has consumed from [`raw_sample_channel`](@ref) since the
run began. Because both streams come off one device, the vendor package knows
the constant offset between that host count and its own free-running sample
counter, so it can propagate the phase to whatever sample it actually starts on.
This is the only handover timing contract — GNSSReceiver never sees the device's
counter directly.

`el_sample_spacing` is the Early-to-Late spacing **in whole input samples**,
already quantised the way `Tracking`'s `get_correlator_sample_shifts` quantises
it. Program exactly this: `dll_disc` normalises with the quantised spacing, so a
device that uses the raw preferred chip shift instead introduces a DLL loop-gain
error (~2.3 % at 4 MHz and 0.5 chips).

`signal` is the `AbstractGNSSSignal` the channel must replicate (e.g. which
component of a pilot/data pair).

# The configuration form

    assign_channel!(sdr, hw_channel, config::HardwareChannelConfig)

is what the link actually calls, and what a device beyond GPS L1 C/A should
implement. [`HardwareChannelConfig`](@ref) carries everything above *and*
everything the positional form left to a shared assumption: all the quantised
tap offsets rather than only the Early-to-Late distance, the band's replica
amplitude and the code replica's amplitude, whether the overlay code is to be
wiped off, the component's identity within its satellite and its carrier-phase
reference.

A device that implements only the positional form keeps working: the default
method below unpacks the configuration into it, which is exactly the L1 C/A
compatibility path — three taps, one antenna, one band, primary code only, all
of which the configuration's extra fields then restate rather than change.

!!! warning "Give `assign_channel!` a concrete signature"

    Define the arguments your device takes, never a
    `assign_channel!(::MyDevice, args...; kwargs...)` catch-all. Such a method
    is neither more nor less specific than the configuration shim below — it
    wins on the device argument and loses on the configuration one — so the
    call becomes an `ambiguous` `MethodError` at the first handover.
"""
assign_channel!(sdr::AbstractHardwareCorrelatorSDR, args...; kwargs...) =
    _not_implemented("assign_channel!", sdr)

# The compatibility path: a device that only implements the positional
# handover gets the fields it knows about. Everything the configuration adds
# describes what that interface already implied — a single three-tap E/P/L
# bank on one band replicating the primary code at unit amplitude — so
# dropping it here changes nothing for such a device, and
# `validate_hardware_configuration` has already refused anything that would.
assign_channel!(
    sdr::AbstractHardwareCorrelatorSDR,
    hw_channel,
    config::HardwareChannelConfig,
) = assign_channel!(
    sdr,
    hw_channel,
    config.prn,
    config.carrier_doppler * Hz,
    config.code_doppler * Hz,
    config.code_phase,
    config.valid_at_sample;
    el_sample_spacing = config.el_sample_spacing,
    signal = config.signal,
)

"""
    release_channel!(sdr, hw_channel)

Stop correlating on `hw_channel` and return it to the free pool. Called when a
satellite loses lock or is otherwise dropped from the tracking state.
"""
release_channel!(sdr::AbstractHardwareCorrelatorSDR, hw_channel) =
    _not_implemented("release_channel!", sdr)

"""
    dropped_dump_count!(sdr) -> Int

Number of dumps the device dropped since the last call, then clear the counter
(the gateware's sticky, write-1-to-clear overflow status).

The dump ring is bounded. If the host stalls, the vendor's producer cannot push
and records are lost — a missed epoch on some channel, which silently corrupts
that satellite's loop. Surfacing it lets the receiver flag or reset the affected
channels instead. The default returns `0`, meaning "this device cannot report
it"; implement it if yours can.
"""
dropped_dump_count!(::AbstractHardwareCorrelatorSDR) = 0

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

Read once per assignment — [`HardwareChannelConfig`](@ref) carries the result
as its `replica_amplitude` — so a device may compute it rather than store it.
"""
correlator_gain(::AbstractHardwareCorrelatorSDR) = 1
correlator_gain(sdr::AbstractHardwareCorrelatorSDR, ::Symbol) = correlator_gain(sdr)

"""
    assignment_start_sample(sdr, hw_channel) -> Int64

Earliest device sample belonging to the channel's confirmed current assignment.
Return `typemax(Int64)` while an asynchronous arm is pending or failed, and
publish its effective sample only after confirming it applied. Revoke the
previous boundary before starting any new assignment, including the same PRN.
The Receiver rejects integrations starting before this boundary, including
already queued dumps. Synchronous producers default to no additional cutoff.
"""
assignment_start_sample(::AbstractHardwareCorrelatorSDR, hw_channel) = typemin(Int64)

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
    dump_tap_slots::Union{Nothing,Integer} = _dump_tap_slots(sdr),
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
    dump_tap_slots = _dump_tap_slots(sdr)
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
    isempty(problems) && return nothing
    throw(
        ArgumentError(
            "$(nameof(typeof(sdr))) " * join(problems, "\n") * "\n" * _CONTRACT_POINTER,
        ),
    )
end

# Accumulator slots one of the device's dump records carries, read off its
# stream's element type. `nothing` when the device exposes neither the stream
# nor a wire correlator this package knows the width of — nothing to check
# against, rather than a failure.
function _dump_tap_slots(sdr::AbstractHardwareCorrelatorSDR)
    dumps = try
        correlator_dump_channel(sdr)
    catch
        return nothing
    end
    dump_type = eltype(dumps)
    dump_type <: CorrelatorDump || return nothing
    wire_tap_slots(_correlator_type(dump_type))
end

# ─────────────────────────────────────────────────────────────────────────────
# Host-side ingest state
# ─────────────────────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────────────────────
# What the device NCO ran: per-channel word timelines
# ─────────────────────────────────────────────────────────────────────────────

# One NCO word the host has scheduled at a named device sample (`apply_at_sample`
# of the `NCOUpdate` that carried it). Plain `Float64` Hz, like `NCOUpdate`.
struct ScheduledNCOWord
    sample::Int64
    carrier_doppler::Float64
    code_doppler::Float64
end

"""
    NCOTimeline

The carrier and code words one hardware channel's NCOs ran and will run: the
word in effect now and the words already scheduled at named device samples.

A hardware loop is closed through a device that holds each word until the next
one lands, milliseconds after the record that motivated it ended. The loop
filter therefore cannot assume that the word it last computed is the one a
record was integrated under — during a pull-in the two differ by tens of Hz —
and a correction computed against the wrong word restates an error the device
is already about to remove. The timeline is the link's record of what the NCO
actually did, so the estimator can attribute every record to the word that
really ran ([`mean_nco_word`](@ref)) and size its correction for the moment it
will land ([`NCOReferencedPLLAndDLL`](@ref)).

Fed by [`push_nco_updates!`](@ref) with every update the device accepted, and
reset at every handover to the words `assign_channel!` loaded. Only words the
device has actually been sent are entered — an update the feedback ring
refused never reaches the NCO, so it never reaches the timeline either.
"""
mutable struct NCOTimeline
    applied_carrier_doppler::Float64
    applied_code_doppler::Float64
    # Ascending in `sample`; every entry lands strictly after the applied word.
    const scheduled::Vector{ScheduledNCOWord}
end

NCOTimeline() = NCOTimeline(0.0, 0.0, ScheduledNCOWord[])

# A handover: the device starts on these words and nothing is in flight.
function reset_timeline!(timeline::NCOTimeline, carrier_doppler_hz, code_doppler_hz)
    timeline.applied_carrier_doppler = Float64(carrier_doppler_hz)
    timeline.applied_code_doppler = Float64(code_doppler_hz)
    empty!(timeline.scheduled)
    timeline
end

# Record a word the device has accepted for `sample`. A device keeps the newest
# command for a given sample, and the link never schedules a later command for
# an earlier sample, so anything queued at or past `sample` is superseded.
function schedule_word!(timeline::NCOTimeline, sample, carrier_doppler_hz, code_doppler_hz)
    while !isempty(timeline.scheduled) && last(timeline.scheduled).sample >= sample
        pop!(timeline.scheduled)
    end
    push!(
        timeline.scheduled,
        ScheduledNCOWord(Int64(sample), Float64(carrier_doppler_hz), Float64(code_doppler_hz)),
    )
    timeline
end

# Everything scheduled at or before `sample` has landed: fold it into the applied
# word. Only call this once no query will start before `sample` again.
function promote_words!(timeline::NCOTimeline, sample)
    n = 0
    for word in timeline.scheduled
        word.sample <= sample || break
        timeline.applied_carrier_doppler = word.carrier_doppler
        timeline.applied_code_doppler = word.code_doppler
        n += 1
    end
    n == 0 || deleteat!(timeline.scheduled, 1:n)
    timeline
end

# Whether a scheduled word takes effect in `(lo, hi]`, i.e. whether records
# ending at `lo` and starting at `hi` ran on different words.
word_changes_within(timeline::NCOTimeline, lo, hi) =
    any(word -> lo < word.sample <= hi, timeline.scheduled)

# The word in effect at device sample `sample`.
function nco_word_at(timeline::NCOTimeline, sample)
    carrier, code = timeline.applied_carrier_doppler, timeline.applied_code_doppler
    for word in timeline.scheduled
        word.sample <= sample || break
        carrier, code = word.carrier_doppler, word.code_doppler
    end
    carrier, code
end

"""
    mean_nco_word(words, a, b) -> (carrier_doppler_hz, code_doppler_hz)

Time-weighted mean of the carrier and code words in effect over the device
samples `[a, b)` — the replica frequencies a record integrated over that span
was really correlated with. For `b <= a` the word in effect at `a`.

`words` is an [`NCOTimeline`](@ref) for a hardware channel, or a
[`FixedNCOWord`](@ref) where the replica ran on one known word (the software
receiver regenerates its replicas from the satellite's Doppler every chunk).
"""
function mean_nco_word(timeline::NCOTimeline, a::Real, b::Real)
    total = b - a
    total > 0 || return nco_word_at(timeline, a)
    carrier, code = timeline.applied_carrier_doppler, timeline.applied_code_doppler
    carrier_sum = 0.0
    code_sum = 0.0
    t = a
    for word in timeline.scheduled
        word.sample >= b && break
        if word.sample > t
            carrier_sum += carrier * (word.sample - t)
            code_sum += code * (word.sample - t)
            t = word.sample
        end
        carrier, code = word.carrier_doppler, word.code_doppler
    end
    carrier_sum += carrier * (b - t)
    code_sum += code * (b - t)
    carrier_sum / total, code_sum / total
end

"""
    FixedNCOWord(carrier_doppler_hz, code_doppler_hz)

A replica that ran on one known word for every span asked about — what the
software receiver's replicas do within a chunk. See [`mean_nco_word`](@ref).
"""
struct FixedNCOWord
    carrier_doppler::Float64
    code_doppler::Float64
end

mean_nco_word(word::FixedNCOWord, a::Real, b::Real) = word.carrier_doppler, word.code_doppler

# One hardware channel's current occupant. `signal_index` addresses the
# component within the satellite's `tracking_signals` tuple, so a pilot/data
# pair simply occupies two hardware channels.
struct HardwareChannelAssignment
    group_key::Symbol
    prn::Int
    signal_index::Int
end

"""
    HardwareCorrelatorLink(sdr; doppler_update_interval, sampling_freq, kwargs...)

Host-side state for driving an [`AbstractHardwareCorrelatorSDR`](@ref): which
satellite occupies which hardware channel, the epoch clock the dumps are folded
on, and the scratch buffers that keep the ingest path allocation-free.

This is the object [`receive`](@ref) hands to [`process`](@ref) as its
correlator source; passing it instead of a `Tracking` downconvert-and-correlator
backend is the whole hardware/software switch.

Keywords:

  - `doppler_update_interval` — the fixed processing epoch. Dumps are collected
    until a record crosses the boundary, then the estimator folds every
    satellite's collected outputs and updates each NCO once. Defaults to one
    primary code period of `reference_signal`, or to `max_integration_time`
    where that is shorter — a 1.5 s GPS L2CL code period is not an update
    interval any tracking loop survives.
  - `feedback_delay_epochs` — how many epochs ahead an [`NCOUpdate`](@ref) is
    scheduled, i.e. the `n` in "the correction from epoch `k` applies at
    `k + n`". Must be large enough to cover the PCIe round trip. The link
    records every accepted update in the channel's [`NCOTimeline`](@ref), so a
    delay-aware estimator ([`NCOReferencedPLLAndDLL`](@ref), the hardware
    receiver's default) sees both the word each record ran on and the sample
    its own correction will land at; the delay is then a known constant the
    loop compensates rather than a lag it has to be de-tuned for.
  - `max_dumps_per_drain` — cap on records pulled from the ring per chunk, so a
    backlog cannot monopolise one call.
  - `max_catchup_epochs` — how far the fold loop replays before treating the
    shortfall as a stream gap and resynchronising the epoch grid.
  - `coherent_code_blocks` — how many primary-code blocks the link sums into one
    record before handing it to the tracking loops, once bit/secondary sync has
    landed. `nothing` (the default) means *one full symbol*: a whole navigation
    bit for a data-bearing signal, one secondary-code period for a pilot. `1`
    restores the old behaviour of folding every dump on its own. See
    [`coherent_integration_blocks`](@ref) for why this is not optional in
    practice.
  - `max_integration_time` — the longest span of signal folded into one record,
    whatever `coherent_code_blocks` asks for
    (default [`DEFAULT_MAX_INTEGRATION_TIME`](@ref), 20 ms). This is the knob
    that separates the *tracking-update cadence* from the *primary-code period*:
    for every signal whose code period fits inside it a record is still a whole
    number of code blocks, and for one whose does not — GPS L2CL's 1.5 s — the
    record is cut inside a code period and counted as the fraction of one it is.
    See [`coherent_integration_periods`](@ref).
  - `max_epoch_clock_advance` — how far ahead of everything seen so far a
    *single* record may place the epoch clock (default 1 s). A record beyond it
    is held back until a second record corroborates the jump, and counted in
    `implausible_dumps`. One nonsense index is otherwise permanent: the clock
    only moves forward and the grid resynchronises onto it.
  - `band_plan` — the RF configuration ([`HardwareBandPlan`](@ref)): which band
    arrives on which input of which device, at what rate, and what makes their
    counters comparable. `nothing` (the default) builds the single-band plan the
    link has always assumed — `reference_signal`'s band at `sampling_freq` —
    so nothing about a one-band receiver changes. With several bands the
    **first** route's band is the reference band and its counter is the receiver
    timebase: `sampling_freq` must be that band's rate, every epoch boundary and
    fold is counted on it, and a record or a command on another band is mapped
    across by the exact ratio of the two rates.
  - `max_dump_gap` — how long a hardware channel's records may be missing before
    the receiver stops protecting its satellite (default 5 s). Within it a
    satellite that receives no record is *frozen* rather than decayed — a gap in
    the dump stream is a host fault and says nothing about the signal, and the
    device goes on tracking through it. Past it the dump path is broken rather
    than late, and the lock detectors are allowed to release the satellite. See
    [`is_observation_gap`](@ref).
"""
mutable struct HardwareCorrelatorLink{C<:Tracking.AbstractCorrelator}
    # Deliberately abstract, and the one field of this struct that is: the
    # device is *not* a type parameter, so every link over the same correlator
    # type is the same type and the pipeline compiled for one device serves
    # every other. That matters because the pipeline is large — `receive`'s
    # processing closure alone costs 1.7 s to compile on an Orin — and it is
    # specialised on the correlator source. With the device in the type, a
    # warm-up against a stub SDR compiled a specialisation the live device
    # could not use, and the whole pipeline recompiled *inside* the first live
    # chunk: 1.7 s in which no dump is drained and the ring overflows (issue
    # #107). Erasing it costs one dynamic dispatch per device call — see
    # `_call_device` — and every one of those is per chunk
    # (`dropped_dump_count!`) or rarer (`assign_channel!`, `release_channel!`);
    # the two streams the ingest path really uses are cached below, so neither
    # the drain nor the feedback push touches the device at all.
    const sdr::AbstractHardwareCorrelatorSDR
    # The device's two record streams, resolved once here rather than asked for
    # per chunk. Caching them keeps the drain and the feedback push on concrete
    # types — but the reason it matters is invalidation, not dispatch cost: code
    # compiled against `correlator_dump_channel`'s method table is thrown away
    # the moment a vendor package adds its own method to it, which is exactly
    # when a hardware receiver starts, and the pipeline then recompiles inside
    # the first live chunk. Reading the channels out of the link touches no
    # generic function at all.
    const dumps::PipeChannel{CorrelatorDump{C}}
    const ncos::PipeChannel{NCOUpdate}
    # hw channel (1-based) → its occupant, or `nothing` when free.
    const assignments::Vector{Union{Nothing,HardwareChannelAssignment}}
    # Reverse index, so the per-chunk sync is a lookup rather than a scan.
    const channel_of::Dict{HardwareChannelAssignment,Int}
    # Records pulled from the ring but not yet folded: they belong to an epoch
    # that has not closed. Reused across chunks.
    const pending::Vector{CorrelatorDump{C}}
    # Scratch for the batch `take!` and the batch NCO `put!`.
    const drain_buffer::Vector{CorrelatorDump{C}}
    const nco_buffer::Vector{NCOUpdate}
    # Epoch grid on the *sample-index* axis (Δ = interval × fs), not wall clock,
    # so it is deterministic and replayable. Counted in *reference-band* samples
    # — the receiver timebase — so one boundary is one instant across every
    # band, whatever rate each one is counted at (see `band_plan`).
    const epoch_length::Int
    # Which RF band arrives on which input of which device, at what rate, and
    # what makes their counters comparable. The first route's band is the
    # reference band; everything above is expressed on its counter. See
    # `HardwareBandPlan`.
    const band_plan::HardwareBandPlan
    # Per hardware channel: the band its occupant lives on, that band's sample
    # rate, and how many receiver-timebase samples one of its device samples is
    # worth. A single-band receiver has scale 1.0 everywhere and the rate is the
    # reference rate, so none of the arithmetic below changes for it.
    const channel_band::Vector{Symbol}
    const channel_sampling_freq::Vector{Float64}
    const channel_timebase_scale::Vector{Float64}
    # The correlator bank each band may draw channels from
    # (`band_hardware_channels`), indexed by the band's position in the plan.
    # A satellite is only ever armed on a channel of its own band's bank: a
    # channel wired to another downconversion chain would correlate the wrong
    # band's samples with this band's replica and never rise above the noise.
    const band_channels::Vector{Vector{Int}}
    # Amplitude of the replica the device wipes off with, relative to the unit
    # replica the host's own correlator would use. Divided out of every
    # accumulator on ingest — see `_retag_spacing`. Device-wide; a per-band
    # declaration (`correlator_gain(sdr, band_id)`) overrides it per channel
    # when `gain_is_per_band`.
    const correlator_gain::Float64
    # What the device says it can replicate and correlate, resolved once here
    # rather than asked per assignment: `_assign!` runs on the chunk path and
    # `hardware_capabilities` is a device call, i.e. an `invokelatest`.
    const capabilities::HardwareCorrelatorCapabilities
    # Whether each channel's replica amplitude is asked of the device per band.
    # False when the caller declared one gain for the whole device, and false
    # for a `:channel` noise reference, where the amplitude divides out of the
    # C/N₀ ratio and the declared gain is neither needed nor wanted.
    const gain_is_per_band::Bool
    const feedback_delay_epochs::Int
    const max_dumps_per_drain::Int
    # ── Absolute code-phase bookkeeping (pseudoranges) ────────────────────────
    # The estimator fold updates Dopplers but never advances a satellite's
    # `code_phase`; in the software receiver the correlate phase does that. Here
    # the FPGA is the correlate phase, so the link dead-reckons each assigned
    # satellite's code phase to every fold boundary and — when the device
    # reports `CorrelatorDump.code_phase` — re-anchors it to the replica the
    # DLL actually steers. All three vectors are indexed by hardware channel and
    # only the estimator-driver signal's channel (signal_index 1) participates.
    #
    # Device sample the sat's `code_phase` currently refers to (`typemin` until
    # the first anchor: before that the acquisition seed is left untouched,
    # because the host cannot place it on the device's counter axis).
    const phase_ref_sample::Vector{Int64}
    # Freshest anchor collected while appending this epoch's dumps (`typemin`
    # sample = none).
    const anchor_sample::Vector{Int64}
    const anchor_code_phase::Vector{Float64}
    # Whether the primary-code count has been tied to the decoded symbol grid.
    const bit_phase_anchored::Vector{Bool}
    # ── Record continuity, per hardware channel ───────────────────────────────
    # A channel's records tile the sample axis: each one covers
    # `[sample_index - integrated_samples, sample_index)`, so the next one must
    # start exactly where this one ended. This vector holds that expected start
    # (`typemin` = no record folded on the channel yet, e.g. right after an
    # assignment). Anything else is a discontinuity, and the two counters below
    # record it — see `_account_record_continuity!` for why the bit clock, not
    # the loop filters, is what a lost record damages.
    const last_record_end::Vector{Int64}
    # Length of the newest record folded on this channel, i.e. how long one of
    # its records currently is. The device's record length is its *code* epoch,
    # which need not equal the fold epoch, so this is what a hole is measured
    # against. `typemin` = no record folded yet.
    const last_record_samples::Vector{Int64}
    # Device samples this channel's record stream is missing because whole
    # records never reached the host — the ring overran, or a batch was dropped
    # host-side. A missing record necessarily costs at least one whole epoch of
    # span, which is exactly how these are told apart from the re-arm holes
    # below. Reset when the channel is (re)assigned.
    const lost_record_samples::Vector{Int64}
    # Device samples the channel did not integrate into any record at all,
    # because it was being re-armed. Nothing was lost in transit: the device
    # stops correlating while `assign_channel!`'s sample-exact phase load takes
    # effect, then resumes on the new replica, so the hole is followed by a
    # *short* first record running to the next epoch boundary. Such a hole is
    # always less than one epoch — it is the remainder of one — which is the
    # discriminator used below.
    #
    # Measured on the board (issue #107): the open-loop noise channel, re-armed
    # once a second onto a fresh decoy, produced 252 short records in 259
    # re-arms over a 258 s run, while every satellite channel produced exactly
    # as many as it had assignments, and the two channels never re-armed
    # produced none. Charging these as lost data made a clean run look like it
    # was dropping correlator output.
    const rearm_dead_samples::Vector{Int64}
    # How far this channel's records have overlapped (a record starting before
    # the previous one ended: a duplicate or a device counter step back).
    const overlapping_record_samples::Vector{Int64}
    # Amplitude scale each channel's accumulators are divided by on ingest: the
    # band's replica amplitude, times the ratio of the code amplitude the
    # device's replica actually has to the one GNSSSignals models. Fixed at the
    # assignment (`correlator_output_scale`), because both factors are
    # properties of the signal and band the channel was armed for, not of the
    # device as a whole. 1.0 for an unassigned channel.
    const channel_scale::Vector{Float64}
    # How many epochs the fold loop will replay in one chunk before treating the
    # shortfall as a gap and resynchronising the grid (see `fold_closed_epochs!`).
    const max_catchup_epochs::Int
    # ── Coherent pre-accumulation, per hardware channel ───────────────────────
    # Requested coherent integration length in primary-code blocks, or 0 for
    # "one full symbol" (see `coherent_integration_blocks`).
    const coherent_code_blocks::Int
    # Longest span of signal, in seconds, that may be folded into one record —
    # the ceiling `coherent_code_blocks` is clamped against, and the line above
    # which a signal's primary-code period stops being a usable integration unit
    # (see `coherent_integration_periods`).
    const max_integration_time::Float64
    # The partially accumulated record: summed accumulators, the samples and
    # (fractional) primary-code periods they span, and the `sample_index` of the
    # newest dump in it. `partial_samples == 0` means "nothing accumulated", in
    # which case `partial_correlator` is undefined rather than zero — a
    # correlator type has no zero without an instance to take it from.
    #
    # `partial_periods` is a `Float64` rather than a block count, and that is the
    # whole of issue #133 in one field: a record spanning a quarter of a code
    # period spans a quarter of one, not "at least one block".
    const partial_correlator::Vector{C}
    const partial_samples::Vector{Int64}
    const partial_periods::Vector{Float64}
    # Primary-code wraps the open record has crossed. Not `round(partial_periods)`:
    # the short record a channel produces between its sample-exact phase load and
    # the next code wrap covers a *fraction* of a block and completes one, and
    # that is the count the coherent accumulation is sized in.
    const partial_wraps::Vector{Int}
    const partial_end::Vector{Int64}
    # Where this channel's record stream currently stands on the primary-code
    # block grid: the fraction of a code period at `last_record_end`, i.e. where
    # the next record begins inside its block. Exactly `0.0` on a block
    # boundary (`_snap_block_fraction` forces it there, so "is this on a
    # boundary?" is an equality rather than a tolerance at every call site), and
    # `NaN` while it is unknown — a channel with no record folded yet.
    #
    # Re-anchored to `CorrelatorDump.code_phase` on every record that reports
    # one, which is what keeps a partial-primary stream on the grid instead of
    # dead-reckoning a 1.5 s code period from the handover for the whole run.
    const block_phase::Vector{Float64}
    # Primary-code wraps this channel's record stream has completed since it was
    # assigned — the count the code *did*, as opposed to the number of records
    # the device produced. Read through `primary_code_wraps`.
    const primary_wraps::Vector{Int64}
    # The longest record seen on this channel since it was assigned, i.e. how
    # long one of its records nominally is. `_account_record_continuity!` tells a
    # re-arm hole from a lost record by comparing the hole to it: with a record
    # per code period the code period served, but a device dumping four times per
    # period leaves holes a quarter that size and every one of them would read as
    # a re-arm. `typemin` = nothing seen yet.
    const nominal_record_samples::Vector{Int64}
    # Primary-code blocks handed to the estimator since it last ran, per channel.
    # `coherent_integration_blocks` sizes a record against the bit buffer's
    # progress through the current symbol, but the bit buffer only advances when
    # the estimator consumes the records — once per chunk. Records emitted
    # earlier in the same chunk are invisible to it, so without this count a
    # host that fell a symbol behind sized every record from the same stale
    # block count, the third one straddled a bit edge, and every later one sat
    # off the grid — the bit stream stalled while bit sync stayed "found".
    const pending_blocks::Vector{Int}
    # Whether the channel's record stream lost records since the estimator last
    # ran (see `_account_record_continuity!`). Consumed by
    # `restart_lost_bit_clocks!` after the estimator, so the bit buffer that
    # counted blocks up to the hole finishes the records it was given before it
    # is replaced.
    const bit_clock_lost::Vector{Bool}
    # Satellites whose bit clock was just restarted, waiting for the receiver to
    # restart the matching decoder (`take_bit_clock_restart!`).
    const bit_clock_restarts::Vector{HardwareChannelAssignment}
    # ── Secondary-code (overlay) removal, per hardware channel ────────────────
    # Whether the *host* takes this channel's overlay off every dump, i.e. the
    # channel's signal has one and `requested_secondary_code_mode` left the job
    # with the host rather than the device. False for a signal without an
    # overlay, which is what keeps GPS L1 C/A's ingest path exactly as it was.
    const secondary_wipe::Vector{Bool}
    # The overlay chip index carried by the primary-code block that *starts* at
    # `secondary_phase_sample`, or `-1` while it is unknown — before
    # secondary/bit sync, and after anything that broke the record stream the
    # counter rides. `anchor_secondary_phases!` seeds it from the bit buffer
    # once per sync; every wiped dump advances it over the blocks it covered.
    const secondary_phase::Vector{Int}
    const secondary_phase_sample::Vector{Int64}
    # ── Noise reference, per RF band ──────────────────────────────────────────
    # Where the C/N₀ estimator's noise density comes from: `:channel` spends a
    # hardware channel on an open-loop despread (the documented FPGA recipe),
    # `:samples` meters Σ|x|² off the raw stream. See `append_noise_observations!`.
    const noise_source::Symbol
    # **One reference per band**, all five vectors indexed by the band's position
    # in `band_plan.routes`. A noise density is a property of one front end's
    # gain chain, one antenna, one filter and one modulation — pooling an L1
    # floor with an L5 one describes neither, and a C/N₀ referenced to the pooled
    # value is wrong on both bands by whatever they differ by. So each band gets
    # its own open-loop despread, on its own signal, at its own sample rate, and
    # its observation reaches only that band's signals.
    #
    # The channel each band's reference occupies, or 0 while it has none. It is
    # never handed to a satellite and never receives an `NCOUpdate`.
    const noise_channels::Vector{Int}
    # The decoy PRN each one currently replicates, and how many epochs since it
    # was last re-armed onto a fresh PRN / phase / carrier offset.
    const noise_prns::Vector{Int32}
    const noise_epochs_since_rearm::Vector{Int}
    const noise_rearm_epochs::Int
    # This chunk's pooled accumulation *per band*: `Σ b·bᴴ` over every tap of
    # every dump that band's noise channel produced (a 1×1 matrix for one
    # antenna), the number of independent looks that pooled, and the samples one
    # look spans.
    const noise_accumulators::Vector{Matrix{ComplexF64}}
    const noise_looks::Vector{Int}
    const noise_samples_per_look::Vector{Int}
    # Reverse index: hardware channel → the band whose noise reference it
    # carries, or 0 for an ordinary (or free) channel. A dump's routing is then
    # an O(1) lookup rather than a scan over the bands.
    const noise_band_of_channel::Vector{Int}
    # Sample index at which the currently open epoch closes. `typemin` until the
    # first record arrives and anchors the grid (see `_anchor_epoch_grid!`).
    next_epoch_boundary::Int
    # Highest `sample_index` seen so far; a record at or past the boundary is
    # what closes the open epoch.
    latest_sample_index::Int
    # ── Epoch-clock plausibility ──────────────────────────────────────────────
    # The furthest ahead of `latest_sample_index` a *single* record may place
    # the epoch clock. The clock only ever moves forward, so one record
    # carrying a nonsense index moves it somewhere no genuine record will ever
    # reach again and every real dump is then "in the past" for the rest of the
    # run — measured on the board as an index of 717 259 801 450 on a counter
    # sitting at 29 × 10⁹, after which the grid resynchronised onto it and
    # nothing was ever tracked again (issue #107). Beyond this bound the record
    # is held back rather than trusted; a device that genuinely jumped (a
    # restart, a re-armed counter) corroborates the jump with its very next
    # record, which is what `implausible_index_candidate` waits for.
    const max_index_advance::Int64
    # The last rejected index, or `typemin` for none. Cleared by any plausible
    # record, so only *consecutive* implausible records — a real jump — are
    # ever accepted.
    implausible_index_candidate::Int64
    # Raw samples consumed from `raw_sample_channel` since the run began — the
    # time base `assign_channel!` hands over on.
    samples_consumed::Int
    # ── Dump-stream gaps ──────────────────────────────────────────────────────
    # `samples_consumed` when this hardware channel last contributed a record,
    # i.e. how long its dumps have been missing measured on the host's own
    # clock. A gap in the dump stream is a *host* fault — the device keeps
    # correlating, and the board's own logs show a channel still on the peak
    # after 2.5 s of missing records — so the receiver freezes the satellite's
    # lock detectors rather than letting them decay through it
    # (`is_within_dump_gap_budget`). `typemin` = nothing folded since the
    # channel was assigned.
    const last_record_at_samples::Vector{Int64}
    # How long that freeze may last before the detectors are allowed to decay
    # again, in raw samples. Without a bound a permanently dead dump path would
    # hold every satellite in lock for ever.
    const max_dump_gap_samples::Int64
    # ── NCO word timelines, per hardware channel ─────────────────────────────
    # What each channel's NCOs ran and will run (see `NCOTimeline`): reset to
    # the handover words by `_assign!`, extended by `push_nco_updates!` with
    # every update the device accepted, and folded forward by
    # `promote_applied_words!` once the records that ran on a word have been
    # folded. This is what lets the estimator attribute a record to the word
    # that really ran under it, and what lets the record accumulation cut on a
    # word boundary the way it cuts on a bit edge.
    const nco_timelines::Vector{NCOTimeline}
    # Stand-in for a satellite the link holds no channel for; it never carries
    # records, so its words are never read.
    const unassigned_timeline::NCOTimeline
    # Device sample the updates computed from the fold in progress will land
    # at (`push_nco_updates!` schedules every channel's update there). Set
    # before the estimator runs so a delay-aware estimator can size its
    # correction for that moment; `typemin` until the first fold.
    scheduled_apply_at_sample::Int64
    # Diagnostics.
    dropped_dumps::Int
    # `NCOUpdate`s the feedback ring refused because the device's writer was not
    # keeping up. Each one is a chunk in which every channel free-ran on its
    # previous word — the loop was open, and the estimator has to know it.
    dropped_nco_updates::Int
    stale_dumps::Int
    unassignable_signals::Int
    skipped_epochs::Int
    # Records refused by the epoch-clock plausibility bound above.
    implausible_dumps::Int
    # Number of forward gaps seen across all channels (the *samples* they cost
    # are per channel, above), split the same way.
    lost_record_gaps::Int
    rearm_gaps::Int
    # Records whose `num_taps` did not match the tap count of the correlator
    # their satellite is tracked with. Dropped rather than reshaped — see
    # `CorrelatorDump`.
    tap_layout_mismatches::Int
    # Satellites the device's declared capabilities cannot serve, so no channel
    # was armed for them. Validated before the run starts, so a non-zero count
    # here means the tracking state grew a signal the configuration did not
    # declare.
    unsupported_signals::Int
    # Times a known overlay phase was dropped because the record it was about to
    # wipe did not start where the counter stood, or covered more than one
    # primary-code block. Removal then stops until the next sync re-seeds it
    # rather than wiping at a phase the host can no longer vouch for — a wrong
    # sign is worse than no wipe, because nothing downstream can see it.
    secondary_phase_losses::Int
    # Records handed to the loops that spanned less than one primary-code
    # period. Zero for every signal whose code period fits inside
    # `max_integration_time`, and one per record for GPS L2CL — which is the
    # point, not a fault: it is what "the loop is updated without waiting 1.5 s"
    # looks like from the accounting's side.
    partial_primary_records::Int
    # Records emitted past their target length because the device's dump grid
    # does not divide its primary-code period, so no dump ever ended on a block
    # boundary to cut on. A device producing partial dumps has to align them to
    # the code-block boundary; this counts the times it did not.
    misaligned_dump_boundaries::Int
end

function HardwareCorrelatorLink(
    sdr::AbstractHardwareCorrelatorSDR;
    sampling_freq,
    reference_signal,
    doppler_update_interval = nothing,
    feedback_delay_epochs::Integer = 2,
    max_dumps_per_drain::Integer = 4096,
    max_catchup_epochs::Integer = 64,
    coherent_code_blocks::Union{Nothing,Integer} = nothing,
    max_integration_time = DEFAULT_MAX_INTEGRATION_TIME,
    correlator_gain = nothing,
    noise_source::Symbol = :channel,
    noise_rearm_interval = 1u"s",
    max_epoch_clock_advance = 1u"s",
    max_dump_gap = 5u"s",
    band_plan::Union{Nothing,HardwareBandPlan} = nothing,
)
    # The RF plan. A link built without one is the single-band receiver it has
    # always been: one band — the reference signal's — at `sampling_freq`, on
    # the device's first input, so every scale below is exactly 1.0.
    plan = something(
        band_plan,
        hardware_band_plan(
            sdr,
            (get_band_id(get_band(reference_signal)),),
            (sampling_freq,),
        ),
    )
    _hz(sampling_freq) ≈ reference_sampling_frequency(plan) || throw(
        ArgumentError(
            "`sampling_freq` ($(_hz(sampling_freq)) Hz) must be the band plan's " *
            "reference band $(reference_band(plan)) rate " *
            "($(reference_sampling_frequency(plan)) Hz): the reference band's counter " *
            "is the receiver timebase",
        ),
    )
    max_integration_seconds = _seconds(max_integration_time)
    max_integration_seconds > 0 || throw(
        ArgumentError("max_integration_time must be positive (got $max_integration_time)"),
    )
    # The processing epoch is a *time*, not a property of the reference signal's
    # code. One primary code period is the natural default and stays the default
    # for every signal whose code period is a usable update interval — but GPS
    # L2CL's is 1.5 s, and an epoch grid on that would fold the loops and push an
    # NCO correction twice per three seconds, which is the conflation issue #133
    # is about. Past `max_integration_time` the epoch is the integration length.
    interval =
        isnothing(doppler_update_interval) ?
        min(code_period_seconds(reference_signal), max_integration_seconds) :
        _seconds(doppler_update_interval)
    epoch_length = round(Int, interval * _hz(sampling_freq))
    epoch_length > 0 || throw(
        ArgumentError(
            "doppler_update_interval $(interval) s is shorter than one sample period at " *
            "$sampling_freq",
        ),
    )
    feedback_delay_epochs >= 1 || throw(
        ArgumentError("feedback_delay_epochs must be at least 1 (got $feedback_delay_epochs)"),
    )
    max_catchup_epochs >= 1 || throw(
        ArgumentError("max_catchup_epochs must be at least 1 (got $max_catchup_epochs)"),
    )
    noise_source in (:channel, :samples) || throw(
        ArgumentError("noise_source must be :channel or :samples (got $noise_source)"),
    )
    # A `:channel` reference rides the device's own datapath, so the replica
    # amplitude divides out of the C/N₀ ratio and the declared gain is neither
    # needed nor wanted; a `:samples` reference does not, and needs it exactly.
    gain =
        noise_source === :channel ? 1.0 :
        Float64(something(correlator_gain, GNSSReceiver.correlator_gain(sdr)))
    gain > 0 ||
        throw(ArgumentError("correlator_gain must be positive (got $gain)"))
    # Ask the device per band only where the answer can matter: an explicit
    # `correlator_gain` is the caller overriding every band at once, and a
    # `:channel` noise reference has already forced the scale to 1.
    gain_is_per_band = noise_source === :samples && isnothing(correlator_gain)
    noise_rearm_epochs =
        max(1, round(Int, upreferred(noise_rearm_interval * sampling_freq) / epoch_length))
    isnothing(coherent_code_blocks) ||
        coherent_code_blocks >= 1 ||
        throw(
            ArgumentError(
                "coherent_code_blocks must be at least 1 or nothing (got $coherent_code_blocks)",
            ),
        )
    max_index_advance = round(Int64, upreferred(max_epoch_clock_advance * sampling_freq))
    max_index_advance >= epoch_length || throw(
        ArgumentError(
            "max_epoch_clock_advance $max_epoch_clock_advance is shorter than one epoch " *
            "at $sampling_freq",
        ),
    )
    max_dump_gap_samples = round(Int64, upreferred(max_dump_gap * sampling_freq))
    max_dump_gap_samples >= 0 || throw(
        ArgumentError("max_dump_gap must not be negative (got $max_dump_gap)"),
    )

    dumps = correlator_dump_channel(sdr)
    dump_type = eltype(dumps)
    dump_type <: CorrelatorDump || throw(
        ArgumentError(
            "correlator_dump_channel(::$(typeof(sdr))) must have eltype <: CorrelatorDump, " *
            "got $dump_type",
        ),
    )
    ncos = nco_update_channel(sdr)
    n = num_hardware_channels(sdr)
    num_bands = length(plan.routes)

    HardwareCorrelatorLink{_correlator_type(dump_type)}(
        sdr,
        dumps,
        ncos,
        Union{Nothing,HardwareChannelAssignment}[nothing for _ = 1:n],
        Dict{HardwareChannelAssignment,Int}(),
        dump_type[],
        dump_type[],
        NCOUpdate[],
        epoch_length,
        plan,
        fill(reference_band(plan), n),
        fill(reference_sampling_frequency(plan), n),
        ones(Float64, n),
        [
            [
                hw_channel for
                hw_channel in Base.invokelatest(band_hardware_channels, sdr, route.band_id) if
                1 <= hw_channel <= n
            ] for route in plan.routes
        ],
        Float64(gain),
        hardware_capabilities(sdr),
        gain_is_per_band,
        Int(feedback_delay_epochs),
        Int(max_dumps_per_drain),
        fill(typemin(Int64), n),
        fill(typemin(Int64), n),
        fill(NaN, n),
        fill(false, n),
        fill(typemin(Int64), n),
        fill(typemin(Int64), n),
        zeros(Int64, n),
        zeros(Int64, n),
        zeros(Int64, n),
        ones(Float64, n),
        Int(max_catchup_epochs),
        isnothing(coherent_code_blocks) ? 0 : Int(coherent_code_blocks),
        max_integration_seconds,
        Vector{_correlator_type(dump_type)}(undef, n),
        zeros(Int64, n),
        zeros(Float64, n),
        zeros(Int, n),
        fill(typemin(Int64), n),
        fill(NaN, n),
        zeros(Int64, n),
        fill(typemin(Int64), n),
        zeros(Int, n),
        fill(false, n),
        HardwareChannelAssignment[],
        fill(false, n),
        fill(-1, n),
        fill(typemin(Int64), n),
        noise_source,
        zeros(Int, num_bands),
        zeros(Int32, num_bands),
        zeros(Int, num_bands),
        noise_rearm_epochs,
        [
            zeros(
                ComplexF64,
                _num_ants(_correlator_type(dump_type)),
                _num_ants(_correlator_type(dump_type)),
            ) for _ = 1:num_bands
        ],
        zeros(Int, num_bands),
        zeros(Int, num_bands),
        zeros(Int, n),
        typemin(Int),
        typemin(Int),
        max_index_advance,
        typemin(Int64),
        0,
        fill(typemin(Int64), n),
        max_dump_gap_samples,
        [NCOTimeline() for _ = 1:n],
        NCOTimeline(),
        typemin(Int64),
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
end

_correlator_type(::Type{CorrelatorDump{C}}) where {C} = C

"""
    get_sdr(link::HardwareCorrelatorLink)

The device behind a link.
"""
get_sdr(link::HardwareCorrelatorLink) = link.sdr

# Call one of the device interface's functions on the link's device.
#
# `invokelatest`, not a plain call, and that is the whole point of this helper.
# `link.sdr` is abstract, so inference cannot resolve `dropped_dump_count!` on
# it; what it does instead is record a backedge on the *method table*, so that
# a package adding a method to it invalidates the caller. A vendor package
# defining the interface for its device is precisely that, and it invalidates
# everything the call is compiled into: the ingest path, `process`, and
# `receive`'s processing closure. Which means the pipeline this package
# precompiles would be recompiled anyway — inside the first live chunk, with the
# device's dump ring unattended for as long as it takes (issue #107).
# `invokelatest` resolves in the current world at run time and leaves no
# backedge to invalidate. Assignment-boundary reads also use this barrier
# when consuming records, so loading a vendor cannot invalidate the fold.
_call_device(f::F, link::HardwareCorrelatorLink, args...; kwargs...) where {F} =
    Base.invokelatest(f, link.sdr, args...; kwargs...)

# ─────────────────────────────────────────────────────────────────────────────
# The receiver timebase (#134)
#
# Every band the device receives counts its own samples at its own rate, so a
# dump's `sample_index` means "so many samples of *that* band". The link folds,
# ranges and schedules on one axis instead — the reference band's counter, which
# is the receiver timebase — and the two conversions below are the only place
# the two axes meet. Both are exact scalings: the rates are ratios of one
# hardware clock (`HardwareBandPlan` refuses a configuration where they are
# not), so there is no offset to estimate and no drift to track.
#
# For a single-band receiver every scale is exactly `1.0` and both functions are
# the identity, which is why none of the arithmetic downstream had to change.
# ─────────────────────────────────────────────────────────────────────────────

# Position of `band_id` in the plan, or 0 for a band it does not route.
function _band_index(link::HardwareCorrelatorLink, band_id::Symbol)
    routes = link.band_plan.routes
    for index in eachindex(routes)
        routes[index].band_id === band_id && return index
    end
    0
end

_band_index(link::HardwareCorrelatorLink, system) =
    _band_index(link, get_band_id(system_band(system)))

# One of this channel's device samples on the receiver timebase, and back.
_receiver_sample(link::HardwareCorrelatorLink, hw_channel::Integer, sample) =
    _scale_sample(sample, link.channel_timebase_scale[hw_channel])

_band_sample(link::HardwareCorrelatorLink, hw_channel::Integer, sample) =
    _scale_sample(sample, 1 / link.channel_timebase_scale[hw_channel])

# A record's place on the receiver timebase. An epoch strobe carries no channel
# (`EPOCH_STROBE_CHANNEL`), and a device emits it on the *reference* band's
# counter — it is the timebase marker, so it is stated in the timebase — which
# is what the out-of-range fallback says.
@inline function _epoch_sample(link::HardwareCorrelatorLink, dump::CorrelatorDump)
    hw_channel = Int(dump.channel)
    checkbounds(Bool, link.channel_timebase_scale, hw_channel) ||
        return Int64(dump.output.sample_index)
    _receiver_sample(link, hw_channel, dump.output.sample_index)
end

# Point one hardware channel at a band: the rate its records, replica offsets and
# handover times are counted at, and the scale onto the receiver timebase.
function _route_channel!(link::HardwareCorrelatorLink, hw_channel::Integer, band_id::Symbol)
    plan = link.band_plan
    link.channel_band[hw_channel] = band_id
    link.channel_sampling_freq[hw_channel] = band_sampling_frequency(plan, band_id)
    link.channel_timebase_scale[hw_channel] = receiver_timebase_scale(plan, band_id)
    link
end

# …and back to the reference band, for a channel that has been released.
_unroute_channel!(link::HardwareCorrelatorLink, hw_channel::Integer) =
    _route_channel!(link, hw_channel, reference_band(link.band_plan))

"""
    receiver_sampling_frequency(link) -> Float64

The rate, in Hz, of the receiver timebase — the reference band's sampling
frequency. Every epoch boundary, `latest_sample_index`, `samples_consumed` and
fold boundary is counted at it; a channel's own band rate is
[`channel_sampling_frequency`](@ref).
"""
receiver_sampling_frequency(link::HardwareCorrelatorLink) =
    reference_sampling_frequency(link.band_plan)

"""
    channel_band_id(link, hw_channel) -> Symbol

The RF band the channel's current occupant lives on — the band its dumps'
`sample_index`, its replica offsets and its [`NCOUpdate`](@ref)s are all counted
on. The reference band for a channel that holds nothing.
"""
channel_band_id(link::HardwareCorrelatorLink, hw_channel::Integer) =
    link.channel_band[hw_channel]

"""
    channel_sampling_frequency(link, hw_channel) -> Float64

The sample rate, in Hz, of the band the channel's occupant lives on.
"""
channel_sampling_frequency(link::HardwareCorrelatorLink, hw_channel::Integer) =
    link.channel_sampling_freq[hw_channel]

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
[`HardwareCorrelatorLink`](@ref) method ignores the samples for tracking
purposes (the FPGA already correlated them) and instead ingests the dumps that
arrived, folding each completed epoch.
"""
advance_tracking!(
    downconvert_and_correlator,
    band_measurements,
    track_state,
    band_systems,
) = track!(band_measurements, track_state; downconvert_and_correlator)

function advance_tracking!(
    link::HardwareCorrelatorLink,
    band_measurements,
    track_state,
    band_systems,
)
    # Honour `track!`'s per-chunk contract: the navigation-bit store is consumed
    # by the decoder after each chunk, so it must be reset at the start of the
    # next one. `track!` does this itself (track.jl); without it every chunk
    # re-feeds the whole accumulated history to `decode` as "new" bits.
    Tracking.reset_start_sample_and_bit_buffer!(track_state)

    # A dump only makes sense for a satellite the device is actually
    # correlating, so reconcile the channel table with the tracking state first:
    # this chunk's acquisitions get hardware channels, and satellites the
    # receiver has dropped give theirs back. This runs *before* the chunk is
    # counted: an acquired satellite's code phase refers to the first sample of
    # the chunk being processed (see `correct_code_phases` / `merge_scan`), so
    # that is the sample the handover has to be declared valid at.
    sync_hardware_channels!(link, track_state, band_systems, band_measurements)

    # The raw stream is still the receiver's clock: count what this chunk
    # delivered so the handover time base stays aligned with it.
    link.samples_consumed += _chunk_num_samples(band_measurements)

    link.dropped_dumps += _call_device(dropped_dump_count!, link)

    drain_dumps!(link)
    fold_closed_epochs!(link, track_state, band_measurements, band_systems)

    track_state
end

"""
    is_observation_gap(correlator_source, track_state, group_key, prn) -> Bool

Whether this chunk delivered **no measurement at all** for the satellite, in a
way that says nothing about the signal — so the receiver should freeze its lock
detectors rather than let them decay through it.

`false` for a software correlator: there the chunk's samples *are* the
measurement, and a chunk that produces no record produced none because the
signal was not there.

A hardware-correlator receiver is the opposite case. The device correlates
whether or not the host is listening, and the dumps reach the host over a DMA
ring that a late host overruns: a stall of a few hundred milliseconds — one JIT
compilation is enough — costs every record produced during it. The board's own
logs are unambiguous about what that gap is *not*: when the records resumed
after 2.5 s of silence the prompt was still at its handover power with
`|P|/|E,L| = 1.44`, i.e. the FPGA had held the satellite throughout on its last
NCO word (issue #107). Decaying the detectors through such a gap released five
satellites the device had never lost, and cost a rescan (two minutes) to get
them back.

So a hardware-tracked satellite that contributed no fully integrated record
this chunk is frozen: its detectors, its dwell and its `time_in_lock` are left
exactly as the last real measurement left them. The freeze is bounded by the
link's `max_dump_gap` — past that the dump path is not late but broken, and the
detectors are allowed to decay so the satellite is eventually released rather
than held in a lock nothing is confirming.

!!! warning "Wrappers must forward this"
    A correlator source that *wraps* a link — an instrumented source that
    forwards [`advance_tracking!`](@ref), say — matches the fallback below, not
    the link's method, and its satellites silently lose the protection. Forward
    it explicitly:

    ```julia
    GNSSReceiver.is_observation_gap(w::MyWrapper, track_state, group_key, prn) =
        GNSSReceiver.is_observation_gap(w.link, track_state, group_key, prn)
    GNSSReceiver.has_current_observations(w::MyWrapper, track_state, system, prn) =
        GNSSReceiver.has_current_observations(w.link, track_state, system, prn)
    ```

    The second method keeps stale bit counts out of navigation while the first
    preserves lock through a bounded gap.
"""
is_observation_gap(correlator_source, track_state, group_key, prn) = false

function is_observation_gap(
    link::HardwareCorrelatorLink,
    track_state,
    group_key,
    prn,
)
    # Tracking clears `filtered_prompts` at the start of every chunk and pushes
    # one per completed record, so "empty" is exactly "no record this chunk".
    isempty(get_filtered_prompts(track_state, group_key, prn, RANGING_SIGNAL_INDEX)) &&
        is_within_dump_gap_budget(link, group_key, prn)
end

# Lock detectors may coast through a transport gap, but navigation must not use
# the frozen decoder bit count with a code phase that keeps wrapping. Require
# records on both the ranging and data components, regardless of the lock-gap
# budget. The software path retains its normal measurement cadence.
has_current_observations(source, track_state, system, prn) = true
function has_current_observations(link::HardwareCorrelatorLink, track_state, system, prn)
    group_key = signal_group_key(system)
    sats = get_sat_states(track_state, group_key)
    haskey(sats, prn) || return false
    all((RANGING_SIGNAL_INDEX, data_signal_index(system))) do signal_index
        !isempty(get_filtered_prompts(track_state, group_key, prn, signal_index))
    end
end

# Whether the satellite's hardware channel has been silent for less than the
# link's `max_dump_gap`. A satellite the link holds no channel for is not
# hardware-tracked at all, so nothing about it is a dump-stream gap.
function is_within_dump_gap_budget(link::HardwareCorrelatorLink, group_key, prn)
    hw_channel = get(
        link.channel_of,
        HardwareChannelAssignment(group_key, prn, RANGING_SIGNAL_INDEX),
        0,
    )
    hw_channel == 0 && return false
    last_seen = link.last_record_at_samples[hw_channel]
    last_seen == typemin(Int64) && return false
    link.samples_consumed - last_seen <= link.max_dump_gap_samples
end

"""
    append_noise_observations!(link, track_state, band_systems, band_measurements)

Give every signal's noise estimator this chunk's measured noise density.

`Tracking`'s default C/N₀ estimator divides a record's prompt power by a
*measured* density rather than inferring a floor from the prompt's own moments,
and on this path nothing else fills it: without this every satellite reports
`-Inf dBHz` and the code lock detector drops it on the first chunk that looks.

Two sources, chosen by the link's `noise_source`:

  - `:channel` (the default, and the one `Tracking`'s FPGA recipe prescribes)
    spends one hardware channel on an **open-loop despread** — see
    [`ensure_noise_channel!`](@ref). Its taps ride the device's own quantise →
    despread → accumulate datapath, so the replica amplitude, the input scaling
    and the code amplitude are common to numerator and denominator and divide
    out of the ratio. That is also what makes the floor the *post-correlation*
    one, `N₀ + ∫S_I(f)·|G(f)|²df`, measured through the consumer's own code
    rather than modelled.
  - `:samples` meters `Σ|x|²` over the chunk's raw samples instead. It is the
    documented power-monitor builder and it reduces to the same `N₀` on white
    input, so on a thermal-dominated band the two agree — but it weights every
    frequency flatly, so a coloured interferer moves it by an amount that has
    nothing to do with what the despreading modulation would actually collect.
    It also needs the device's replica amplitude declared by hand
    ([`correlator_gain`](@ref)), because nothing cancels. Kept as a control and
    as a fallback for a device that cannot spare a channel.

A noise *density* is a property of the band and the modulation, not of a
satellite, so one observation serves every satellite tracking that signal — and
only those. Both sources are **per band**: `:samples` meters each band's own raw
frame, and `:channel` keeps one open-loop reference per band
([`ensure_noise_channel!`](@ref)). Unrelated RF floors are never pooled, because
a pooled floor is a C/N₀ bias on every satellite of every band involved and
nothing downstream can see it.
"""
function append_noise_observations!(link, track_state, band_systems, band_measurements)
    isempty(track_state.noise_estimators) && return track_state
    if link.noise_source === :channel
        _flush_channel_noise!(link, track_state, band_systems, band_measurements)
    else
        _append_band_noise!(
            track_state,
            Tuple(band_measurements),
            band_systems,
            keys(track_state.noise_estimators),
        )
    end
    track_state
end

# Walk the bands as a tuple recursion rather than a `map`/`zip`: the signal ids
# a band contributes are a compile-time property of its system tuple, and this
# keeps the whole walk inferable from the `TrackState`'s type.
_append_band_noise!(track_state, ::Tuple{}, ::Tuple{}, configured) = track_state
function _append_band_noise!(track_state, measurements::Tuple, systems::Tuple, configured)
    measurement = first(measurements)
    observation = Tracking.noise_observation_from_samples(
        _accumulated_power(Tracking.get_samples(measurement)),
        size(Tracking.get_samples(measurement), 1),
        Tracking.get_sampling_frequency(measurement),
    )
    _append_signal_noise!(
        track_state,
        observation,
        _flatten_systems(map(tracking_signals, first(systems))),
        configured,
    )
    _append_band_noise!(track_state, Base.tail(measurements), Base.tail(systems), configured)
end

_append_signal_noise!(track_state, observation, ::Tuple{}, configured) = track_state
function _append_signal_noise!(track_state, observation, signals::Tuple, configured)
    signal_id = get_signal_id(first(signals))
    # A signal only has an estimator if its C/N₀ estimator reads a density;
    # appending to one that has none is an error, not a no-op.
    signal_id in configured &&
        Tracking.append_noise_observation!(track_state, observation, signal_id)
    _append_signal_noise!(track_state, observation, Base.tail(signals), configured)
end

# One antenna: the scalar Σ|x|². An array: the raw spatial covariance Σ x·xᴴ,
# which is what a beamformer's weights reduce to that satellite's own floor.
#
# Every sample is widened before it is squared. Integer sample types are the
# normal case for a front end (`Complex{Int16}` here), and Julia's integer
# arithmetic does not widen: `abs2` on a `Complex{Int16}` whose magnitude
# exceeds 181 wraps *inside the element* — `sum` then adds up already-corrupted
# terms in `Int64` and returns a plausible-looking number that is too small by
# a random factor. A noise floor too small by 5x is a C/N₀ too high by 7 dB, on
# every satellite, with nothing else looking wrong.
_accumulated_power(samples::AbstractVector) = sum(x -> abs2(ComplexF64(x)), samples)
function _accumulated_power(samples::AbstractMatrix)
    n = size(samples, 2)
    acc = zero(SMatrix{n,n,ComplexF64})
    for i in axes(samples, 1)
        x = SVector{n,ComplexF64}(view(samples, i, :))
        acc += x * x'
    end
    acc
end

# Samples in this chunk, counted on the **receiver timebase** — so, on the
# reference band's frame. Every band advances from frames of one duration on one
# time base, but not necessarily of one length: a band sampled faster delivers
# proportionally more samples for the same span. The reference band is the one
# `samples_consumed` (and with it every handover time) is counted in, so it is
# the one that speaks here.
_chunk_num_samples(band_measurements::NamedTuple) =
    _chunk_num_samples(first(values(band_measurements)))
_chunk_num_samples(m::Tracking.BandMeasurement) = size(Tracking.get_samples(m), 1)

# ─────────────────────────────────────────────────────────────────────────────
# Channel management
# ─────────────────────────────────────────────────────────────────────────────

"""
    sync_hardware_channels!(link, track_state, band_systems, band_measurements)

Reconcile the device's channel table with `track_state`: release channels whose
satellite is no longer tracked, then assign a free channel to every tracked
(satellite, signal) that does not have one.

Releases run first so a satellite dropped this chunk frees its channel for one
acquired in the same chunk.
"""
function sync_hardware_channels!(link, track_state, band_systems, band_measurements)
    release_stale_channels!(link, track_state)
    # Before the satellites, so the reference is not the thing that loses the
    # last free channel: with no density every satellite's C/N₀ reads
    # `-Inf dBHz` and they are all dropped, which costs far more than the one
    # channel.
    ensure_noise_channel!(link, track_state, band_systems, band_measurements)
    assign_new_channels!(link, track_state, band_systems, band_measurements)
    link
end

"""
    ensure_noise_channel!(link, track_state, band_systems, band_measurements)

Keep **one hardware channel per RF band** running an open-loop despread as that
band's C/N₀ noise reference, and re-arm each of them periodically.

One per band, not one for the receiver: a noise density is a property of a front
end's gain chain, antenna, filter and interference environment, and of the
modulation that despreads it. Two bands share none of those. Pooling their
floors into one number describes neither band, and the error lands on every
satellite of both as a C/N₀ bias — the quantity the code lock detector thresholds
on, and the one thing in the receiver with nothing to contradict it. So each
band's reference is armed on a signal of that band, at that band's sample rate,
with that band's replica gain, and its observation reaches only that band's
signals ([`append_noise_observations!`](@ref)).

The channel is an ordinary tracking channel programmed with a *decoy* PRN: same
code generator, same carrier NCO, same quantisation, same accumulators as every
satellite. That is the whole point — the reference is then model-free by
construction, because the replica amplitude, the input scaling and the code
amplitude are common to it and to the taps it is divided into, and cancel. It
also makes the measured floor the post-correlation one, weighted by the
despreading modulation's own spectrum, rather than flat received power.

It is open loop: no discriminator, no loop filter, and `push_nco_updates!` never
sends it an `NCOUpdate`. Its code Doppler is left at zero while the sky's is
not, so the relative code phase slides several chips a second and any chance
alignment decays on its own; every `noise_rearm_epochs` it is additionally
re-armed onto the next PRN of the family, a fresh uniform code phase and a
carrier offset drawn from ±5 kHz. Randomising is what keeps a chance alignment
from becoming a permanent bias, and it is why the reference needs to know
nothing about which satellites are tracked.
"""
function ensure_noise_channel!(link, track_state, band_systems, band_measurements)
    link.noise_source === :channel || return link
    isempty(track_state.noise_estimators) && return link
    for systems in band_systems
        isempty(systems) && continue
        signal = _noise_reference_signal(systems)
        isnothing(signal) && continue
        band_index = _band_index(link, first(systems))
        band_index == 0 && continue
        if link.noise_channels[band_index] == 0
            hw_channel = _find_free_channel(link, band_index)
            isnothing(hw_channel) && continue
            link.noise_channels[band_index] = hw_channel
            link.noise_band_of_channel[hw_channel] = band_index
            link.noise_epochs_since_rearm[band_index] = link.noise_rearm_epochs
        end
        link.noise_epochs_since_rearm[band_index] >= link.noise_rearm_epochs || continue
        _arm_noise_channel!(
            link,
            band_index,
            signal,
            _band_sampling_frequency(band_measurements, signal),
        )
    end
    link
end

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

function _arm_noise_channel!(link, band_index, signal, sampling_freq)
    hw_channel = link.noise_channels[band_index]
    band_id = get_band_id(get_band(signal))
    _route_channel!(link, hw_channel, band_id)
    route = _route_or_reference(link.band_plan, band_id)
    # Rotate through the family rather than picking one and staying: a PRN whose
    # cross-correlation with a strong satellite happens to be unusually high is
    # then one observation in the window, not the window.
    link.noise_prns[band_index] = Int32(mod(Int(link.noise_prns[band_index]), 32) + 1)
    # The same quantised spacing a tracked satellite gets, asked of `Tracking` the
    # same way `_assign!` asks. Wider taps would be better — at a whole chip the
    # three of them are three *independent* looks, which is what pooling them
    # assumes — but a device's code RAM reaches a bounded number of chips either
    # side of prompt (the M2SDR's is ±1, and it rejects anything further), and a
    # spacing it refuses is a channel that never starts. At half a chip the taps
    # are correlated, so the window's variance improves a little more slowly
    # than `1/M`; each tap on its own is still an unbiased look at the noise
    # power, so the density itself is unaffected.
    correlator = Tracking.EarlyPromptLateCorrelator(
        num_ants = Tracking.NumAnts(size(link.noise_accumulators[band_index], 1)),
    )
    # `signal_index = 0`: the reference belongs to no satellite's component
    # list. Everything else is an ordinary assignment, which is the point — the
    # floor it measures is only model-free because it goes through the same
    # replica, the same quantisation and the same accumulators as a satellite.
    config = HardwareChannelConfig(
        signal,
        correlator;
        signal_index = 0,
        group_key = signal_group_key(signal),
        prn = Int(link.noise_prns[band_index]),
        carrier_doppler = (rand() * 10_000 - 5_000) * Hz,  # carrier dither, ±5 kHz
        code_doppler = 0.0Hz,                              # open loop: it free-runs
        code_phase = rand() * get_code_length(signal),     # uniform code phase
        valid_at_sample = _band_sample(link, hw_channel, link.samples_consumed),
        sampling_freq,
        replica_amplitude = _replica_amplitude(link, band_id),
        code_amplitude = Float64(_call_device(replica_code_amplitude, link, signal)),
        secondary_code_mode = requested_secondary_code_mode(link, signal),
        rf_input = route.rf_input,
        device_index = route.device_index,
    )
    _call_device(assign_channel!, link, hw_channel, config)
    link.noise_epochs_since_rearm[band_index] = 0
    # A re-arm invalidates whatever was part-accumulated against the old PRN.
    _reset_noise_accumulator!(link, band_index)
    link
end

function _reset_noise_accumulator!(link, band_index)
    fill!(link.noise_accumulators[band_index], zero(ComplexF64))
    link.noise_looks[band_index] = 0
    link.noise_samples_per_look[band_index] = 0
    link
end

# Pool one noise dump: `Σ b·bᴴ` over its taps. The taps are kept apart for a
# satellite because their differences are the discriminants; here they are three
# independent looks and nothing about their relative values means anything, so
# they are summed. For an antenna array the pooled payload is the array's
# spatial covariance, whose diagonal is each antenna's own floor.
function _accumulate_noise_dump!(link, band_index, output, num_taps)
    accumulators = get_accumulators(output.correlator)
    for index = 1:min(num_taps, length(accumulators))
        _add_outer!(link.noise_accumulators[band_index], accumulators[index])
        link.noise_looks[band_index] += 1
    end
    # Every tap of one dump integrates the same samples, so the span of a look
    # is the dump's own length, counted once.
    link.noise_samples_per_look[band_index] = output.integrated_samples
    link
end

_add_outer!(acc::Matrix{ComplexF64}, tap::Number) = (acc[1, 1] += abs2(tap); acc)
function _add_outer!(acc::Matrix{ComplexF64}, tap)
    for j in eachindex(tap), i in eachindex(tap)
        acc[i, j] += tap[i] * conj(tap[j])
    end
    acc
end

# Hand the chunk's pooled accumulation to every signal that asked for a density,
# then start a fresh one. `M` is the number of independent looks rather than the
# sample count: it is what makes observations from producers of different
# granularity combinable, and what the sliding window weights by.
function _flush_channel_noise!(link, track_state, band_systems, band_measurements)
    configured = keys(track_state.noise_estimators)
    for systems in band_systems
        isempty(systems) && continue
        band_index = _band_index(link, first(systems))
        band_index == 0 && continue
        link.noise_looks[band_index] == 0 && continue
        signal = _noise_reference_signal(systems)
        isnothing(signal) && continue
        sampling_freq = _band_sampling_frequency(band_measurements, signal)
        samples_per_look = link.noise_samples_per_look[band_index]
        observation = Tracking.noise_observation_from_correlator(
            _pooled_noise(link, band_index),
            link.noise_looks[band_index],
            link.noise_looks[band_index] * samples_per_look,
            sampling_freq;
            prn = Int(link.noise_prns[band_index]),
            duration = samples_per_look / sampling_freq,
        )
        # Only *this band's* signals. A density measured through an L1 front
        # end says nothing about an L5 one, and handing it to L5's estimator
        # biases every L5 satellite's C/N₀ by whatever the two gain chains,
        # filters and interference environments differ by — silently, because
        # a C/N₀ has no second opinion to disagree with.
        _append_signal_noise!(
            track_state,
            observation,
            _flatten_systems(map(tracking_signals, systems)),
            configured,
        )
        _reset_noise_accumulator!(link, band_index)
    end
    track_state
end

function _pooled_noise(link, band_index)
    accumulator = link.noise_accumulators[band_index]
    size(accumulator, 1) == 1 ? real(accumulator[1, 1]) :
    SMatrix{size(accumulator, 1),size(accumulator, 2),ComplexF64}(accumulator)
end

function release_stale_channels!(link, track_state)
    for hw_channel in eachindex(link.assignments)
        assignment = link.assignments[hw_channel]
        isnothing(assignment) && continue
        _is_tracked(track_state, assignment) && continue
        _call_device(release_channel!, link, hw_channel)
        link.assignments[hw_channel] = nothing
        delete!(link.channel_of, assignment)
        link.phase_ref_sample[hw_channel] = typemin(Int64)
        link.anchor_sample[hw_channel] = typemin(Int64)
        link.anchor_code_phase[hw_channel] = NaN
        link.bit_phase_anchored[hw_channel] = false
        link.last_record_end[hw_channel] = typemin(Int64)
        link.last_record_samples[hw_channel] = typemin(Int64)
        link.nominal_record_samples[hw_channel] = typemin(Int64)
        link.block_phase[hw_channel] = NaN
        link.primary_wraps[hw_channel] = 0
        link.lost_record_samples[hw_channel] = 0
        link.rearm_dead_samples[hw_channel] = 0
        link.overlapping_record_samples[hw_channel] = 0
        link.channel_scale[hw_channel] = 1.0
        link.pending_blocks[hw_channel] = 0
        link.bit_clock_lost[hw_channel] = false
        link.secondary_wipe[hw_channel] = false
        _unroute_channel!(link, hw_channel)
        forget_secondary_phase!(link, hw_channel)
        filter!(!=(assignment), link.bit_clock_restarts)
        reset_timeline!(link.nco_timelines[hw_channel], 0.0, 0.0)
        # A part-accumulated record belongs to the satellite that just left; it
        # can neither be finished nor handed to anyone else.
        _discard_partial!(link, hw_channel)
        # Any dump still in flight for this channel now refers to a satellite
        # that is gone; `fold_closed_epochs!` drops it as stale.
    end
    link
end

function _is_tracked(track_state, assignment::HardwareChannelAssignment)
    sat_states = get_sat_states(track_state, assignment.group_key)
    haskey(sat_states, assignment.prn)
end

function assign_new_channels!(link, track_state, band_systems, band_measurements)
    for systems in band_systems, system in systems
        group_key = signal_group_key(system)
        sampling_freq = _band_sampling_frequency(band_measurements, system)
        # The correlator bank that sees this system's band — the same front end
        # its raw acquisition stream comes from.
        band_index = _band_index(link, system)
        for sat_state in get_sat_states(track_state, group_key)
            prn = get_prn(sat_state)
            for (signal_index, tracked_signal) in enumerate(get_signals(sat_state))
                assignment = HardwareChannelAssignment(group_key, prn, signal_index)
                haskey(link.channel_of, assignment) && continue
                hw_channel = _find_free_channel(link, band_index)
                if isnothing(hw_channel)
                    # More tracked signals than this band's bank has replica sets.
                    # The unassigned ones simply get no correlator outputs, so
                    # their lock detectors decay and the receiver drops them —
                    # the same path as a satellite that faded.
                    link.unassignable_signals += 1
                    continue
                end
                _assign!(
                    link,
                    hw_channel,
                    assignment,
                    sat_state,
                    tracked_signal,
                    sampling_freq,
                )
            end
        end
    end
    link
end

# A free channel of `band_index`'s own correlator bank, or `nothing` when that
# bank is full. Scoped to the band rather than to the device: a channel wired to
# another RF input sees another band's samples, so handing it this band's
# satellite would arm a replica against the wrong spectrum.
function _find_free_channel(link, band_index::Integer)
    band_index in eachindex(link.band_channels) || return nothing
    for hw_channel in link.band_channels[band_index]
        # Every band's noise reference holds its channel for the whole run.
        link.noise_band_of_channel[hw_channel] == 0 || continue
        isnothing(link.assignments[hw_channel]) && return hw_channel
    end
    nothing
end

function _assign!(link, hw_channel, assignment, sat_state, tracked_signal, sampling_freq)
    signal = get_signal(tracked_signal)
    correlator = get_correlator(tracked_signal)
    # Last line of defence before a device write. `receive` validated the whole
    # configuration before the run started, so reaching this is a tracking state
    # that grew a signal the configuration never declared; refuse the channel
    # rather than throwing, because this runs on the chunk path and taking the
    # receiver down would cost every other satellite its lock. Unarmed, the
    # satellite simply receives no correlator output and its lock detectors
    # release it — the same path as one that faded.
    unsupported = hardware_support_error(
        link.capabilities,
        signal,
        correlator,
        sampling_freq;
        num_ants = Tracking.get_num_ants(correlator),
        dump_tap_slots = wire_tap_slots(_link_correlator_type(link)),
        max_integration_time = link.max_integration_time,
    )
    if !isnothing(unsupported)
        link.unsupported_signals += 1
        @warn "hardware correlator $unsupported" prn = assignment.prn maxlog = 10
        return link
    end
    # Route the channel to its band *before* the handover is built: the band's
    # own sample rate, RF input and device are what the configuration states,
    # and the handover instant is stated on that band's counter rather than on
    # the receiver timebase the host counts `samples_consumed` in.
    band_id = get_band_id(get_band(signal))
    _route_channel!(link, hw_channel, band_id)
    route = _route_or_reference(link.band_plan, band_id)
    config = HardwareChannelConfig(
        signal,
        correlator;
        signal_index = assignment.signal_index,
        group_key = assignment.group_key,
        prn = assignment.prn,
        carrier_doppler = get_carrier_doppler(sat_state),
        code_doppler = get_code_doppler(sat_state),
        code_phase = get_code_phase(sat_state),
        valid_at_sample = _band_sample(link, hw_channel, link.samples_consumed),
        sampling_freq,
        replica_amplitude = _replica_amplitude(link, band_id),
        code_amplitude = Float64(_call_device(replica_code_amplitude, link, signal)),
        secondary_code_mode = requested_secondary_code_mode(link, signal),
        rf_input = route.rf_input,
        device_index = route.device_index,
    )
    _call_device(assign_channel!, link, hw_channel, config)
    link.channel_scale[hw_channel] = correlator_output_scale(config)
    link.assignments[hw_channel] = assignment
    link.channel_of[assignment] = hw_channel
    # The sat's `code_phase` is the acquisition seed, which lives on the host's
    # raw-sample axis; it cannot be placed on the device counter until the first
    # anchored dump arrives, so the reference starts unknown.
    link.phase_ref_sample[hw_channel] = typemin(Int64)
    link.anchor_sample[hw_channel] = typemin(Int64)
    link.anchor_code_phase[hw_channel] = NaN
    link.bit_phase_anchored[hw_channel] = false
    # A fresh occupant starts a fresh record stream: the previous satellite's
    # end sample says nothing about where this one's first record begins.
    link.last_record_end[hw_channel] = typemin(Int64)
    link.last_record_samples[hw_channel] = typemin(Int64)
    link.nominal_record_samples[hw_channel] = typemin(Int64)
    # A fresh occupant's replica starts wherever the handover put it, so the
    # channel's place on the primary-code block grid is unknown until its first
    # record re-establishes it.
    link.block_phase[hw_channel] = NaN
    link.primary_wraps[hw_channel] = 0
    link.lost_record_samples[hw_channel] = 0
    link.rearm_dead_samples[hw_channel] = 0
    link.overlapping_record_samples[hw_channel] = 0
    link.pending_blocks[hw_channel] = 0
    link.bit_clock_lost[hw_channel] = false
    # Who takes this channel's overlay off, decided once here from what the
    # device was asked for, and an overlay phase that is not yet known: the new
    # occupant's secondary/bit sync has not been found on this channel.
    link.secondary_wipe[hw_channel] =
        get_secondary_code_length(signal) > 1 &&
        requested_secondary_code_mode(link, signal) === :primary_only
    forget_secondary_phase!(link, hw_channel)
    # The handover itself counts as the channel's last sign of life, so the
    # dump-gap budget is spent from here rather than from whatever the previous
    # occupant left behind.
    link.last_record_at_samples[hw_channel] = link.samples_consumed
    # The device starts on exactly the words just handed to `assign_channel!`,
    # and nothing scheduled for the previous occupant applies to this one.
    reset_timeline!(
        link.nco_timelines[hw_channel],
        ustrip(Hz, uconvert(Hz, get_carrier_doppler(sat_state))),
        ustrip(Hz, uconvert(Hz, get_code_doppler(sat_state))),
    )
    _discard_partial!(link, hw_channel)
    link
end

# The correlator type a link's records are carried in — its own type parameter,
# so no instance and no device call is needed.
_link_correlator_type(::HardwareCorrelatorLink{C}) where {C} = C

# Replica amplitude for one band: the device's per-band declaration where the
# link is set up to ask for it, and the single device-wide value otherwise.
_replica_amplitude(link::HardwareCorrelatorLink, band_id::Symbol) =
    link.gain_is_per_band ? Float64(_call_device(correlator_gain, link, band_id)) :
    link.correlator_gain

"""
    requested_secondary_code_mode(link, signal) -> Symbol

Which side of the link removes `signal`'s secondary (overlay) code:
`:primary_only` — the device replicates the primary code and the **host** takes
the overlay off each dump — or `:wipeoff`, the device doing it in the gateware.

`:primary_only` for every device and every signal, deliberately, and this is the
one place that decision is written down. The host-side removal keys off it —
[`GNSSReceiver.is_secondary_code_removed`](@ref) is false for a channel whose
device was asked to wipe — so a future `:wipeoff` turns the host's removal off
in the same step it turns the device's on: the overlay comes off once, or not at
all, never twice.

**Why the host owns it.** The overlay's phase is not known when a channel is
armed. It is recovered by the bit/secondary-code sync detector, some way into
tracking, from the prompts themselves — and a device asked to wipe an overlay at
the wrong phase cancels the signal instead of accumulating it. Handing the job
to the gateware therefore needs more than a flag in
[`HardwareChannelConfig`](@ref): it needs a *scheduled* command — "from device
sample `n`, overlay chip `k`" — on the same sample-exact footing as
[`NCOUpdate`](@ref), plus a way for the device to report the overlay counter
back so a lost record cannot leave the two ends disagreeing silently. None of
that exists yet, and the host can do the whole job with a per-dump sign, which
is exact rather than approximate: the device's replicas run continuously, so
multiplying a primary-period dump by its overlay chip *is* the correlation the
device would have produced with the overlay baked in.

What the host's ownership costs is that the overlay cannot be removed before
the accumulators leave the device, so a dump must stay one primary-code period
long for its single overlay chip to be separable — which is the device contract
anyway (see [`CorrelatorDump`](@ref)) — and pre-accumulation in the gateware
across code periods stays unavailable for overlaid signals. That is what a
scheduled `:wipeoff` contract would buy, and issue #133 is where dumps shorter
or longer than a primary period are dealt with.

[`supports_secondary_code_wipeoff`](@ref) reports what a device *could* do, and
is left as the declaration it is: nothing requests it yet.
[`coherent_integration_blocks`](@ref) is what the removal unlocks either way —
records may span several code blocks once the overlay is off them.
"""
requested_secondary_code_mode(::HardwareCorrelatorLink, ::AbstractGNSSSignal) =
    :primary_only

# Per-band sampling frequency for a system, read off the `BandMeasurement` the
# chunk was built with so the ingest path and the estimator can never disagree.
_band_sampling_frequency(band_measurements::NamedTuple, system) =
    get_sampling_frequency(band_measurements[get_band_id(system_band(system))])

# ─────────────────────────────────────────────────────────────────────────────
# Dump ingest and the epoch fold
# ─────────────────────────────────────────────────────────────────────────────

"""
    drain_dumps!(link) -> Int

Move every dump currently in the ring into `link.pending` and return how many
were accepted (records the epoch-clock plausibility bound refuses are dropped
and counted in `link.implausible_dumps`; see `_is_plausible_index!`).

Non-blocking by construction: it takes exactly `n_avail` records (capped by
`max_dumps_per_drain`), so a chunk that finds the ring empty does nothing rather
than parking the receiver's processing task. Pacing comes from the raw stream,
which is the receiver's clock; the dump ring only has to be drained faster than
the device fills it.
"""
function drain_dumps!(link::HardwareCorrelatorLink)
    channel = link.dumps
    available = min(Base.n_avail(channel), link.max_dumps_per_drain)
    available == 0 && return 0
    resize!(link.drain_buffer, available)
    take!(channel, link.drain_buffer)
    taken = 0
    for dump in link.drain_buffer
        # Every comparison the epoch clock makes is on the receiver timebase, so
        # the record's own band counter is mapped onto it first. Without that a
        # band sampled 25 % faster would drive the clock 25 % fast and every
        # record of every slower band would be permanently "in the past".
        epoch_sample = _epoch_sample(link, dump)
        _is_plausible_index!(link, epoch_sample) || continue
        push!(link.pending, dump)
        # The epoch clock advances on *any* record, strobe or not: that is what
        # lets a silent channel stall the loop only when the device also stops
        # strobing.
        link.latest_sample_index = max(link.latest_sample_index, epoch_sample)
        taken += 1
    end
    _anchor_epoch_grid!(link)
    taken
end

# Whether a record's `sample_index` may be believed, i.e. whether it may move
# the epoch clock. The clock is monotone and the grid resynchronises onto it
# (`fold_closed_epochs!`), so a single nonsense index is not a transient: it
# permanently strands the grid ahead of every genuine record. The rule is
# therefore "one record cannot move the clock further than `max_index_advance`
# on its own" — a jump that large has to be *corroborated* by a second record
# landing near the first, which a device that really did restart its counter
# supplies immediately while a torn CSR read or a framing slip does not.
#
# The first record of a run anchors the clock and is always believed: there is
# nothing yet to be implausible against.
function _is_plausible_index!(link::HardwareCorrelatorLink, sample_index)
    if link.latest_sample_index == typemin(Int) ||
       sample_index - link.latest_sample_index <= link.max_index_advance
        # Any believable record clears a pending candidate: a genuine jump
        # arrives as a *run* of records, so a candidate that the next record
        # does not confirm was noise.
        link.implausible_index_candidate = typemin(Int64)
        return true
    end
    if link.implausible_index_candidate != typemin(Int64) &&
       abs(sample_index - link.implausible_index_candidate) <= link.max_index_advance
        link.implausible_index_candidate = typemin(Int64)
        return true
    end
    link.implausible_index_candidate = sample_index
    link.implausible_dumps += 1
    false
end

# Anchor the epoch grid to the first record ever seen. The grid is defined on the
# device's own counter, whose origin the host does not know a priori, so the
# first record's sample index defines epoch 0 and every boundary follows from Δ.
function _anchor_epoch_grid!(link)
    link.next_epoch_boundary == typemin(Int) || return link
    isempty(link.pending) && return link
    first_index = minimum(dump -> _epoch_sample(link, dump), link.pending)
    link.next_epoch_boundary = first_index + link.epoch_length
    link
end

"""
    fold_closed_epochs!(link, track_state, band_measurements) -> Int

Fold every epoch that has closed and return how many folds ran.

An epoch closes when a record with `sample_index >= boundary` has been seen —
including an [epoch strobe](@ref epoch_strobe), which is why a momentarily
silent channel does not stall the loop. Closing it appends every collected
output that belongs to the epoch (in `sample_index` order, since `dll_disc` and
the CN0 estimator see them in the order they are appended), runs the estimator
once so each satellite's NCO is updated exactly once for the epoch, and pushes
the resulting [`NCOUpdate`](@ref)s.

Nothing is dropped for pacing reasons: a satellite contributing zero, one or two
outputs to an epoch is expected and the estimator handles it.

A gap in the record stream is bounded rather than replayed. The loop normally
closes one or two epochs per chunk, but if the host stalls — or the device
stops and restarts — `sample_index` can jump by far more than one epoch. Folding
every skipped epoch would run thousands of estimator passes over empty buffers
in a single chunk and flood the feedback ring with one `NCOUpdate` per skipped
epoch, turning a transient stall into a much longer one. Past
`max_catchup_epochs` the grid is instead **resynchronised** onto the epoch
containing the newest record: the skipped epochs carried no data, so nothing is
lost by not folding them, and the loop resumes in real time.
"""
function fold_closed_epochs!(
    link::HardwareCorrelatorLink,
    track_state,
    band_measurements,
    band_systems,
)
    link.next_epoch_boundary == typemin(Int) && return 0

    # A jump this large is a gap, not a backlog: skip to the current epoch
    # rather than grinding through every boundary in between.
    behind = link.latest_sample_index - link.next_epoch_boundary
    if behind >= link.max_catchup_epochs * link.epoch_length
        skipped = behind ÷ link.epoch_length
        link.skipped_epochs += skipped
        link.next_epoch_boundary += skipped * link.epoch_length
    end

    folds = 0
    boundary = link.next_epoch_boundary
    while link.latest_sample_index >= link.next_epoch_boundary
        boundary = link.next_epoch_boundary
        append_epoch_outputs!(link, track_state, boundary)
        # Phase bookkeeping runs before the estimator so a bit-sync phase snap
        # sees the anchored, boundary-referenced code phase — and it runs per
        # epoch so each boundary's anchor is absorbed where it belongs.
        advance_code_phases!(link, track_state, boundary)
        link.next_epoch_boundary = boundary + link.epoch_length
        folds += 1
        link.noise_epochs_since_rearm .+= 1
    end
    folds == 0 && return 0

    # One estimator pass and one NCO update per *chunk*, not per epoch.
    #
    # The host can only get a correction to the device once per chunk, so that is
    # the interval the loop's output has to be sized for. Running the estimator
    # per epoch instead pushes one update per epoch, of which the device only
    # ever keeps the last — and that last one carries a proportional term
    # computed for a single epoch while the device then holds it for the whole
    # chunk (or for however long the host stays behind). Coalescing the epochs'
    # records into one per channel first (`flush_partial_records!`) makes the
    # record's `integrated_samples`, and therefore the loop's `Δt` and its `1/n`
    # bandwidth scaling, equal to the interval that actually elapses. See
    # `coherent_integration_blocks` for the measurement that forced this.
    flush_partial_records!(link, track_state)
    # The fold's C/N₀ estimator reads a measured noise density and nothing else
    # on this path fills it, so the reference has to land before the estimator
    # runs, not after it.
    append_noise_observations!(link, track_state, band_systems, band_measurements)
    # Where this fold's corrections will land, decided before the estimator runs
    # so it can size them for that moment (`push_nco_updates!` schedules them
    # at exactly this sample).
    link.scheduled_apply_at_sample = nco_apply_at_sample(link, boundary)
    estimate_dopplers!(link, track_state, band_measurements)
    # The estimator consumed every record emitted this chunk, so the bit buffers
    # are current again and nothing is pending against them.
    fill!(link.pending_blocks, 0)
    anchor_bit_phases!(link, track_state, boundary)
    restart_lost_bit_clocks!(link, track_state)
    # After the restart, so a bit clock that was just thrown away cannot hand
    # the overlay counter a phase derived from the sync that went with it.
    anchor_secondary_phases!(link, track_state)
    push_nco_updates!(link, track_state, boundary)
    promote_applied_words!(link)
    folds
end

"""
    estimate_dopplers!(link, track_state, band_measurements) -> TrackState

Run the tracking state's Doppler estimator over the records the link folded
this chunk. `Tracking`'s estimators are called as they are; an
[`NCOReferencedPLLAndDLL`](@ref) is additionally handed the link's per-channel
[`NCOTimeline`](@ref)s and the sample its corrections will land at.
"""
estimate_dopplers!(link::HardwareCorrelatorLink, track_state, band_measurements) =
    Tracking.estimate_dopplers_and_filter_prompt!(track_state, band_measurements)

# The device sample at which the updates from the fold closing `boundary` are
# scheduled: `feedback_delay_epochs` past the newest record the host has seen.
# See `push_nco_updates!` for why the newest record rather than the boundary.
nco_apply_at_sample(link::HardwareCorrelatorLink, boundary) =
    Int64(max(boundary, link.latest_sample_index)) +
    Int64(link.feedback_delay_epochs) * Int64(link.epoch_length)

# Fold each channel's timeline forward over the words its folded records ran
# on. Everything scheduled up to the start of the newest folded record has
# landed and can be absorbed into the applied word; no later query starts
# before that record's centre, so nothing the estimator will ask about is lost.
function promote_applied_words!(link::HardwareCorrelatorLink)
    for hw_channel in eachindex(link.assignments)
        isnothing(link.assignments[hw_channel]) && continue
        last_end = link.last_record_end[hw_channel]
        last_end == typemin(Int64) && continue
        promote_words!(
            link.nco_timelines[hw_channel],
            last_end - link.last_record_samples[hw_channel],
        )
    end
    link
end

"""
    restart_lost_bit_clocks!(link, track_state) -> link

Give every satellite whose hardware channel lost records this chunk a fresh,
unsynchronised bit buffer, and queue it for the receiver to restart its decoder
(see [`take_bit_clock_restart!`](@ref)).

A lost record is a hole in the block count the navigation bit clock is built on:
tracking, C/N₀ and bit sync all look healthy afterwards while every bit sits off
its grid and the pseudorange's bit count is wrong by the hole. Nothing about the
hole can be measured — the records are gone — so the state that depended on
continuity is thrown away and rebuilt from the signal: bit sync is found again
within a second or two, and the decoder re-synchronises on the next preamble.
The tracking loops, which need no continuity, keep running on the device
throughout. Runs after the estimator, so the records folded up to the hole have
been credited to the buffer that was counting them.

The absolute code phase is left alone: [`advance_code_phases!`](@ref)
dead-reckons it across the hole and re-anchors it to the device replica on the
next dump. Only its integer code-period count is re-tied to the bit grid, by
[`anchor_bit_phases!`](@ref), once bit sync is back.
"""
function restart_lost_bit_clocks!(link::HardwareCorrelatorLink, track_state)
    for hw_channel in eachindex(link.assignments)
        link.bit_clock_lost[hw_channel] || continue
        link.bit_clock_lost[hw_channel] = false
        assignment = link.assignments[hw_channel]
        isnothing(assignment) && continue
        sat_states = get_sat_states(track_state, assignment.group_key)
        haskey(sat_states, assignment.prn) || continue
        sat_state = sat_states[assignment.prn]
        signals = Tracking.get_signals(sat_state)
        tracked = signals[assignment.signal_index]
        restarted = Tracking.TrackedSignal(
            tracked;
            bit_buffer = typeof(Tracking.get_bit_buffer(tracked))(),
        )
        sat_states[assignment.prn] = Tracking.TrackedSat(
            sat_state;
            signals = Base.setindex(signals, restarted, assignment.signal_index),
        )
        link.bit_phase_anchored[hw_channel] = false
        # The overlay phase was derived from the sync that just went with the
        # bit buffer; the fresh one carries a frozen zero that means nothing.
        forget_secondary_phase!(link, hw_channel)
        assignment in link.bit_clock_restarts || push!(link.bit_clock_restarts, assignment)
    end
    link
end

"""
    take_bit_clock_restart!(correlator_source, group_key, prn) -> Bool

Whether the source restarted this satellite's bit clock since the receiver last
asked, clearing the flag. The receiver restarts the satellite's decoder in
response, because the bit stream it was decoding has been cut. `false` for every
source but a [`HardwareCorrelatorLink`](@ref) that lost records — see
[`restart_lost_bit_clocks!`](@ref).
"""
take_bit_clock_restart!(correlator_source, group_key, prn) = false

function take_bit_clock_restart!(link::HardwareCorrelatorLink, group_key, prn)
    isempty(link.bit_clock_restarts) && return false
    before = length(link.bit_clock_restarts)
    filter!(a -> !(a.group_key == group_key && a.prn == prn), link.bit_clock_restarts)
    length(link.bit_clock_restarts) < before
end

"""
    flush_partial_records!(link, track_state) -> link

Hand every hardware channel's part-accumulated record to the estimator, so a
chunk's worth of dumps reaches the loop filters as one record spanning the whole
chunk. Called once per chunk, immediately before the estimator runs.

Channels with nothing accumulated — and channels whose satellite the receiver
has already dropped — are skipped.

A part-record that does not yet end on a primary-code block boundary is *not*
flushed where the signal's code period is the integration unit
([`allows_partial_primary_records`](@ref)): `Tracking` credits a record to its
bit clock in whole blocks, so handing it one cut inside a block moves the
navigation bit grid by the cut for the rest of the run. Such a record waits for
the dump that completes its block, which is at most one dump away — the cost is
one update interval, once, against a bit stream that never comes back.
"""
function flush_partial_records!(link::HardwareCorrelatorLink, track_state)
    for hw_channel in eachindex(link.assignments)
        link.partial_samples[hw_channel] == 0 && continue
        assignment = link.assignments[hw_channel]
        if isnothing(assignment)
            _discard_partial!(link, hw_channel)
            continue
        end
        sat_states = get_sat_states(track_state, assignment.group_key)
        if !haskey(sat_states, assignment.prn)
            _discard_partial!(link, hw_channel)
            continue
        end
        sat_state = sat_states[assignment.prn]
        signal = get_signal(Tracking.get_signals(sat_state)[assignment.signal_index])
        _record_is_emittable(
            link,
            signal,
            hw_channel,
            _block_boundary_tolerance(link, signal, hw_channel),
        ) || continue
        _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
    end
    link
end

"""
    advance_code_phases!(link, track_state, boundary)

Advance every assigned satellite's absolute `code_phase` to the fold boundary,
re-anchoring it to the device replica wherever this epoch's dumps carried a
[`CorrelatorDump`](@ref) `code_phase`.

The estimator fold updates Dopplers but never moves `code_phase` — in the
software receiver the correlate phase advances it sample by sample. Here the
FPGA is the correlate phase, so the host mirrors it: dead-reckon by
`Δsamples × code_frequency / fs` on the device's sample axis, then absorb the
(wrapped, ±half a code length) difference to the reported replica phase. The
anchor is what keeps the *absolute* phase — and with it the pseudorange —
honest: it erases the handover seed error once the DLL has pulled in, and it
cancels the drift between the host's float bookkeeping and the device NCO's
fixed-point steps, neither of which any tracking loop would otherwise ever see.

Referencing every satellite to the same boundary is what makes the code phases
comparable across satellites — the common-reception-time assumption PVT's
pseudoranges are built on. `boundary` is on the **receiver timebase**, so that
holds across RF bands too: it is converted to each channel's own band counter
before the dead reckoning, and a satellite on a 5 MS/s band and one on a 4 MS/s
band are both extrapolated to the same instant. Satellites without an anchor yet (assigned, but no
dump seen) keep their acquisition seed: it lives on the host's raw-sample axis,
which the link cannot place on the device counter, and the DLL pull-in doesn't
need it to be moved.

The whole-code-period count picked up before bit sync is arbitrary.
[`anchor_bit_phases!`](@ref) ties it to the bit buffer after the estimator
first finds synchronization. Tracking's secondary-code snap does not perform
this operation for signals without a secondary code, such as GPS L1 C/A.
"""
function advance_code_phases!(link::HardwareCorrelatorLink, track_state, epoch_boundary)
    for hw_channel in eachindex(link.assignments)
        assignment = link.assignments[hw_channel]
        (isnothing(assignment) || assignment.signal_index != 1) && continue
        sat_states = get_sat_states(track_state, assignment.group_key)
        haskey(sat_states, assignment.prn) || continue
        sat_state = sat_states[assignment.prn]

        signals = Tracking.get_signals(sat_state)
        signal = get_signal(first(signals))
        code_length = get_code_length(signal)
        # Chips per sample *of this channel's band*: the dumps, the anchors and
        # the reference below are all counted on that band's counter, and only
        # the fold boundary arrives on the receiver timebase.
        chips_per_sample =
            (ustrip(Hz, get_code_frequency(signal)) +
             ustrip(Hz, uconvert(Hz, get_code_doppler(sat_state)))) /
            link.channel_sampling_freq[hw_channel]
        # The common reception epoch, expressed on this band's counter. Every
        # band's satellites are extrapolated to the *same instant*, which is the
        # common-reception-time assumption PVT's pseudoranges rest on — it just
        # happens to be a different integer on each band's counter.
        boundary = _band_sample(link, hw_channel, epoch_boundary)

        reference = link.phase_ref_sample[hw_channel]
        anchor = link.anchor_sample[hw_channel]
        code_phase = get_code_phase(sat_state)
        if anchor != typemin(Int64)
            # Dead-reckon to the anchor (a no-op on the very first one, whose
            # window is arbitrary until the bit-sync snap), absorb the wrapped
            # difference to the reported replica phase, then extrapolate the
            # short hop to the boundary.
            predicted = reference == typemin(Int64) ? code_phase :
                        code_phase + (anchor - reference) * chips_per_sample
            correction = rem(
                link.anchor_code_phase[hw_channel] - mod(predicted, code_length),
                code_length,
                RoundNearest,
            )
            code_phase = predicted + correction + (boundary - anchor) * chips_per_sample
            link.anchor_sample[hw_channel] = typemin(Int64)
            link.anchor_code_phase[hw_channel] = NaN
        elseif reference != typemin(Int64)
            # No dump this epoch (dropped or momentarily silent): keep the phase
            # moving so it stays comparable with the other satellites'.
            code_phase += (boundary - reference) * chips_per_sample
        else
            continue
        end

        code_phase = mod(code_phase, Tracking.current_code_wrap(signals))
        sat_states[assignment.prn] = Tracking.TrackedSat(sat_state; code_phase)
        link.phase_ref_sample[hw_channel] = boundary
    end
    link
end

# The hardware phase is referenced to the common fold boundary, whereas the
# bit buffer counts completed records through this channel's last record end.
# At first bit sync those two clocks must be joined explicitly: pre-sync phase
# wraps every primary period, and a chunk can contain more records after the
# one that found the bit edge. Losing that count introduces an integer-ms
# pseudorange error even with perfectly continuous records and valid decoding.
"""
    anchor_bit_phases!(link, track_state, boundary)

Tie a newly synchronized data signal's integer code-period count to its bit
buffer, retaining the replica phase at the common reception boundary. Applied
once per channel assignment, after the estimator folds the chunk's records.
"""
function anchor_bit_phases!(link::HardwareCorrelatorLink, track_state, epoch_boundary)
    for ch in eachindex(link.assignments)
        assignment = link.assignments[ch]
        (isnothing(assignment) || assignment.signal_index != 1) && continue
        link.bit_phase_anchored[ch] && continue
        states = get_sat_states(track_state, assignment.group_key)
        haskey(states, assignment.prn) || continue
        sat = states[assignment.prn]
        signals = Tracking.get_signals(sat)
        signal = get_signal(first(signals))
        bb = Tracking.get_bit_buffer(first(signals))
        # Secondary-code signals have their own Tracking phase snap. This
        # anchor is for a single data signal with a primary-only replica.
        length(signals) == 1 || continue
        get_secondary_code_length(signal) == 1 || continue
        iszero(get_data_frequency(signal)) && continue
        bb.found || continue
        # Both the reference and the record end live on this channel's band
        # counter, so the fold boundary is converted onto it first.
        boundary = _band_sample(link, ch, epoch_boundary)
        link.phase_ref_sample[ch] == boundary || continue
        last = link.last_record_end[ch]
        last == typemin(Int64) && continue
        primary = get_code_length(signal)
        rate =
            (ustrip(Hz, get_code_frequency(signal)) + ustrip(Hz, get_code_doppler(sat))) /
            link.channel_sampling_freq[ch]
        elapsed = (boundary - last) * rate
        # A dump may report the last sample before wrap (near `primary`) or
        # the first after it (near zero). Recover its signed residual about
        # the completed code boundary, then retain the full extrapolation to
        # the common reception epoch, including any whole periods.
        residual = rem(get_code_phase(sat) - elapsed, primary, RoundNearest)
        code_phase =
            bb.prompt_accumulator_integrated_code_blocks * primary + residual + elapsed
        states[assignment.prn] = Tracking.TrackedSat(sat; code_phase)
        link.bit_phase_anchored[ch] = true
    end
    link
end

# Append every pending output that ended before `boundary`, oldest first, and
# drop it from `pending`. Records at or past the boundary belong to the next
# epoch and stay.
function append_epoch_outputs!(link, track_state, boundary)
    # Ordered — and cut — on the receiver timebase, so records from bands
    # counted at different rates interleave by the instant they were produced
    # rather than by the integer their own counter happens to be at.
    sort!(link.pending; by = dump -> _epoch_sample(link, dump))
    keep = 0
    for dump in link.pending
        if _epoch_sample(link, dump) >= boundary
            keep += 1
            link.pending[keep] = dump
            continue
        end
        is_epoch_strobe(dump) && continue
        _append_dump!(link, track_state, dump)
    end
    resize!(link.pending, keep)
    link
end

# A channel's records tile the sample axis: record `k` covers
# `[sample_index - integrated_samples, sample_index)`, so record `k+1` must
# start exactly where record `k` ended. Measure the discontinuity.
#
# This matters far more than it looks. The loop filters key off each record's
# own `integrated_samples`, so a lost record costs them nothing but one missed
# update — but the *navigation bit clock* counts code blocks as they are folded
# (`Tracking.buffer` completes a bit once the accumulated block count reaches
# `num_code_blocks_that_form_a_bit`), so a record the host never sees moves that
# satellite's bit boundary permanently by the record's length. Tracking, C/N0
# and bit sync all stay healthy while the bit stream sits off its 20 ms grid;
# only the decoder notices, and only as "a valid preamble never appears again" —
# and until it does, the bit count its pseudorange is built on is wrong by the
# hole. So a hole is not compensated for, it is declared: the satellite's bit
# clock and decoder are restarted (`restart_lost_bit_clocks!`), and the host
# fault that caused it is what has to be fixed. A dump stream that is drained
# without stalls loses nothing.
function _account_record_continuity!(
    link,
    track_state,
    assignment,
    sat_state,
    hw_channel,
    output,
    span,
)
    expected_start = link.last_record_end[hw_channel]
    record_start = output.sample_index - output.integrated_samples
    if expected_start != typemin(Int64)
        gap = record_start - expected_start
        if gap > 0
            # A re-arm leaves a hole shorter than one of the channel's records,
            # followed by a short record running to the next boundary; a lost
            # record leaves a hole of at least one whole record. The measure is
            # therefore *the channel's own record length*, not the signal's code
            # period: with a record per code period the two are the same number,
            # but a device dumping four times per period leaves lost-record holes
            # a quarter of a code period long and every one of them would read as
            # a re-arm — which leaves the bit clock standing across a hole that
            # cut it (#133).
            #
            # The nominal length is the longest record seen since the channel was
            # assigned rather than the previous one: adjacent full records can
            # differ by a sample, and the previous record can itself be a short
            # arm record.
            _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
            nominal = _nominal_record_samples(link, sat_state, assignment, hw_channel)
            if gap < nominal && output.integrated_samples < nominal
                link.rearm_dead_samples[hw_channel] += gap
                link.rearm_gaps += 1
            else
                link.lost_record_samples[hw_channel] += gap
                link.lost_record_gaps += 1
                # From here on the channel's records are folded one code block
                # at a time, as for a fresh satellite, until the bit clock has
                # been restarted after this chunk's estimator pass.
                link.bit_clock_lost[hw_channel] = true
                @warn "hardware correlator records lost; restarting the satellite's bit sync and decoder" prn =
                    assignment.prn hw_channel lost_samples = gap
            end
        elseif gap < 0
            link.overlapping_record_samples[hw_channel] -= gap
        end
    end
    link.last_record_end[hw_channel] = output.sample_index
    link.last_record_samples[hw_channel] = output.integrated_samples
    link.nominal_record_samples[hw_channel] =
        max(link.nominal_record_samples[hw_channel], output.integrated_samples)
    # The record stream's place on the primary-code block grid moves with the
    # record, and only with it: `span.wraps` is what the *code* did, however many
    # records the device produced while doing it.
    link.block_phase[hw_channel] = span.block_phase
    link.primary_wraps[hw_channel] += span.wraps
    link
end

# How long one of this channel's records nominally is, in device samples: the
# longest seen since the channel was assigned, capped at one primary-code period
# because a record spanning several of them says nothing about the device's dump
# cadence. Falls back to the code period while nothing has been seen.
function _nominal_record_samples(link, sat_state, assignment, hw_channel)
    signal = get_signal(Tracking.get_signals(sat_state)[assignment.signal_index])
    code_rate =
        ustrip(Hz, get_code_frequency(signal)) +
        ustrip(Hz, uconvert(Hz, get_code_doppler(sat_state)))
    period = floor(
        Int64,
        get_code_length(signal) * link.channel_sampling_freq[hw_channel] / code_rate,
    )
    seen = link.nominal_record_samples[hw_channel]
    seen <= 0 ? period : min(period, seen)
end

function _append_dump!(link, track_state, dump)
    hw_channel = Int(dump.channel)
    checkbounds(Bool, link.assignments, hw_channel) || (link.stale_dumps += 1; return link)
    start = _call_device(assignment_start_sample, link, hw_channel)::Int64
    if start == typemax(Int64) ||
       dump.output.sample_index - dump.output.integrated_samples < start
        link.stale_dumps += 1
        return link
    end
    noise_band = link.noise_band_of_channel[hw_channel]
    if noise_band != 0
        # A dump still carrying the previous decoy PRN was produced before the
        # re-arm took effect; pooling it would credit the window a look at a
        # replica the accumulator is no longer about. The band index is what
        # keeps one front end's floor out of another's estimate.
        dump.prn == link.noise_prns[noise_band] ?
        _accumulate_noise_dump!(link, noise_band, dump.output, num_correlator_taps(dump)) :
        (link.stale_dumps += 1)
        return link
    end
    assignment = link.assignments[hw_channel]
    # A dump whose channel is free, or whose PRN no longer matches the channel's
    # occupant, was produced before a reassignment took effect. Folding it into
    # whoever holds the channel now would corrupt that satellite's loop.
    if isnothing(assignment) || assignment.prn != Int(dump.prn)
        link.stale_dumps += 1
        return link
    end
    sat_state = get_sat_state(track_state, assignment.group_key, assignment.prn)
    # The record has to describe the correlator its satellite is tracked with.
    # Taking the first three of five accumulators for a five-tap signal would
    # hand `dll_disc` two taps that never saw a replica, and padding a three-tap
    # record out to five would invent them outright — so a mismatch is dropped
    # and counted. `validate_hardware_configuration` refuses the configurations
    # that make this reachable before the run starts; reaching it means the
    # gateware is producing a layout it was not asked for.
    expected_taps =
        Tracking.get_num_accumulators(get_correlator(sat_state, assignment.signal_index))
    if num_correlator_taps(dump) != expected_taps
        link.tap_layout_mismatches += 1
        @warn "hardware correlator dump carries the wrong tap layout; dropping it" prn =
            assignment.prn hw_channel dump_taps = num_correlator_taps(dump) expected_taps maxlog =
            10
        return link
    end
    # Where this record sits on the primary-code block grid, measured once and
    # handed to everything downstream: the overlay removal needs to know which
    # block's chip it carries, the continuity accounting moves the grid by it,
    # and the coherent accumulation counts in it. Measured before any of them so
    # the three cannot disagree about the same record (#133).
    span = _record_block_span(
        link,
        sat_state,
        assignment.signal_index,
        hw_channel,
        dump.output,
        dump.code_phase,
    )
    # Take the overlay chip off before anything reads the accumulators, so the
    # coherent pre-accumulation, the discriminators, the C/N0 estimator and the
    # navigation bit accumulation all see the same wiped record.
    output = _wipe_secondary_code!(
        link,
        track_state,
        assignment,
        sat_state,
        hw_channel,
        dump.output,
        span,
    )
    _account_record_continuity!(
        link,
        track_state,
        assignment,
        sat_state,
        hw_channel,
        output,
        span,
    )
    _accumulate_dump!(link, track_state, assignment, sat_state, hw_channel, output, span)
    # The channel is alive: the dump-gap budget starts again from here.
    link.last_record_at_samples[hw_channel] = link.samples_consumed
    # Collect the code-phase anchor for the phase bookkeeping. Only the
    # estimator-driver signal carries the sat-shared code phase; dumps are
    # appended in `sample_index` order, so the epoch's freshest anchor wins.
    # Anchors are collected per *dump*, not per emitted record: the code phase
    # bookkeeping is about where the device's replica is, which every dump
    # reports regardless of how many of them the loops get to see at once.
    if assignment.signal_index == 1 && !isnan(dump.code_phase)
        link.anchor_sample[hw_channel] = dump.output.sample_index
        link.anchor_code_phase[hw_channel] = dump.code_phase
    end
    link
end

# ─────────────────────────────────────────────────────────────────────────────
# Secondary (overlay) codes: taking them off the dumps, host-side (#132)
#
# A device replicates the primary code only (see
# [`requested_secondary_code_mode`](@ref)), so each dump of an overlaid signal
# carries that primary-code block's overlay chip as a ±1 sign on every one of
# its accumulators. `Tracking` assumes the opposite: once bit/secondary sync is
# found its software replica bakes the overlay into the code, and the prompts
# reaching the post-sync bit accumulation are already wiped.
#
# Handing it un-wiped prompts does not break *tracking* — every discriminator is
# a ratio of taps that all carry the same sign — but it cancels the navigation
# symbol. Ten GPS L5I dumps of a constant +1 symbol sum to the NH10 code's own
# sum, 2, where ten wiped ones sum to 10: 14 dB of the symbol thrown away, and a
# soft bit whose sign is the overlay's rather than the satellite's.
#
# So the host does the wipe the device was not asked for, one primary-code block
# at a time, at the ingest — ahead of the coherent pre-accumulation, and
# therefore ahead of the discriminators, the C/N0 estimator and the bit buffer
# alike, all of which read the same record.
#
# What it costs is a *phase*: which overlay chip a given block carries. That is
# exactly what the sync detector recovers, and `anchor_secondary_phases!` reads
# it out of the bit buffer on the fold that finds it. From there the link's own
# counter rides the record stream, which tiles the sample axis one overlay chip
# per primary-code block. A record that does not start where the counter stands
# — a lost record, an overlap, a dump spanning more than one block — is a
# counter that can no longer be vouched for, and the removal stops until the
# next sync re-seeds it. Nothing is ever wiped at a guessed phase: a wrong sign
# is worse than no wipe, because every consumer downstream is blind to it.
# ─────────────────────────────────────────────────────────────────────────────

"""
    is_secondary_code_removed(link, hw_channel) -> Bool

Whether the host is removing this hardware channel's secondary (overlay) code
from every dump right now — i.e. the channel carries an overlay the device was
not asked to wipe *and* the link knows the overlay's phase.

This is the condition under which consecutive dumps may be summed:
[`coherent_integration_blocks`](@ref) holds an overlaid signal at one
primary-code block per record until it is true.
"""
is_secondary_code_removed(link::HardwareCorrelatorLink, hw_channel::Integer) =
    link.secondary_wipe[hw_channel] && link.secondary_phase[hw_channel] >= 0

# Drop a channel's overlay phase: the removal stops, and the next
# `anchor_secondary_phases!` re-seeds it from the bit buffer if sync still
# stands. Cheap and idempotent, so every path that can invalidate the counter
# calls it rather than reasoning about whether it has to.
function forget_secondary_phase!(link::HardwareCorrelatorLink, hw_channel::Integer)
    link.secondary_phase[hw_channel] = -1
    link.secondary_phase_sample[hw_channel] = typemin(Int64)
    link
end

"""
    anchor_secondary_phases!(link, track_state) -> link

Seed every overlaid channel's secondary-code phase from its bit buffer, and drop
it again wherever synchronization no longer stands.

Run once per chunk, after the estimator has folded the chunk's records and after
[`restart_lost_bit_clocks!`](@ref). The sync detector reports the overlay chip
the *upcoming* integration aligns to, and `Tracking` walks that anchor along
every further record folded in the same chunk (the ones it marks
`correlated_pre_sync` and keeps out of the coherent bit sum, because they were
correlated against the pre-sync replica — here, un-wiped). So by the time the
fold returns, the reported phase belongs to the block starting at
`last_record_end`: exactly where the channel's next dump begins.

Only the *seed* comes from the bit buffer. `Tracking` freezes
`BitBuffer.secondary_phase` after sync — it reads it once, at the code-phase
snap — so from there the link's own counter is what the removal runs on, and a
channel that is already counting is left alone.
"""
function anchor_secondary_phases!(link::HardwareCorrelatorLink, track_state)
    for hw_channel in eachindex(link.assignments)
        link.secondary_wipe[hw_channel] || continue
        assignment = link.assignments[hw_channel]
        isnothing(assignment) && continue
        sat_states = get_sat_states(track_state, assignment.group_key)
        if !haskey(sat_states, assignment.prn)
            forget_secondary_phase!(link, hw_channel)
            continue
        end
        tracked_signal =
            Tracking.get_signals(sat_states[assignment.prn])[assignment.signal_index]
        bit_buffer = Tracking.get_bit_buffer(tracked_signal)
        if !has_bit_or_secondary_code_been_found(bit_buffer)
            # Sync has not been found yet, or it was thrown away with the bit
            # clock. Either way the phase is not knowable from here.
            forget_secondary_phase!(link, hw_channel)
            continue
        end
        # Already counting: the link's counter has moved past the frozen value
        # in the bit buffer, and re-seeding from it would step the overlay back.
        link.secondary_phase[hw_channel] >= 0 && continue
        last_end = link.last_record_end[hw_channel]
        # Sync without a record on this channel is a satellite that synced
        # elsewhere (a re-assignment inheriting a synced bit buffer); there is
        # no sample to hang the phase on until its first record arrives.
        last_end == typemin(Int64) && continue
        link.secondary_phase[hw_channel] = mod(
            bit_buffer.secondary_phase,
            get_secondary_code_length(get_signal(tracked_signal)),
        )
        link.secondary_phase_sample[hw_channel] = last_end
    end
    link
end

# Take the overlay chip off one record and advance the channel's overlay counter
# over it. Returns what the rest of the ingest path sees: the record unchanged
# on a channel that is not being wiped, and one whose accumulators have all been
# multiplied by the same ±1 where it is.
#
# The sign comes from `GNSSSignals.secondary_value`, the very lookup the
# software replica is built from, so a shared overlay (GPS L5's NH10/NH20), a
# per-PRN one (the 1800-chip GPS L1C / BeiDou B1C overlays) and any
# PRN-dependent exception a signal model carries are all handled by the model
# rather than restated here.
function _wipe_secondary_code!(
    link,
    track_state,
    assignment,
    sat_state,
    hw_channel,
    output,
    span,
)
    is_secondary_code_removed(link, hw_channel) || return output
    tracked_signal = Tracking.get_signals(sat_state)[assignment.signal_index]
    signal = get_signal(tracked_signal)
    # The counter rides a record stream that tiles the sample axis, one overlay
    # chip per primary-code block. A record starting anywhere else, or crossing a
    # block boundary inside itself — its blocks' chips are already summed inside
    # the accumulator, where no single sign can separate them again — leaves the
    # host unable to say which chip this record carries.
    #
    # What a record must *not* do is cross a boundary; how long it is does not
    # matter. A record shorter than a code period lies wholly inside one block
    # and carries that block's chip exactly as a whole-period record does — which
    # is what makes the removal work on a device that dumps inside a code period
    # at all (#133). `wraps == 1` is admissible only when the record ends
    # precisely on the boundary it crossed.
    within_one_block = span.wraps == 0 || (span.wraps == 1 && span.block_phase == 0.0)
    if output.sample_index - output.integrated_samples !=
       link.secondary_phase_sample[hw_channel] || !within_one_block
        # Cut the record here: what has accumulated so far was wiped and what
        # follows will not be, and one record cannot be both.
        _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
        forget_secondary_phase!(link, hw_channel)
        link.secondary_phase_losses += 1
        return output
    end
    chip = GNSSSignals.secondary_value(
        get_secondary_code(signal),
        assignment.prn,
        link.secondary_phase[hw_channel],
    )
    # The counter advances over the code *wraps* the record completed, not over
    # the record: two half-period records of one block carry the same chip and
    # move the counter once, between them.
    link.secondary_phase[hw_channel] = mod(
        link.secondary_phase[hw_channel] + span.wraps,
        get_secondary_code_length(signal),
    )
    link.secondary_phase_sample[hw_channel] = output.sample_index
    chip > 0 && return output
    Tracking.CorrelatorOutput(
        _negate_accumulators(output.correlator),
        output.integrated_samples,
        output.sample_index,
    )
end

# Flip the sign of every accumulator, keeping everything else from the
# correlator — the overlay chip is one sign for the whole primary-code period,
# so every tap carries it and every tap loses it together.
_negate_accumulators(correlator::Tracking.AbstractCorrelator) =
    @set correlator.accumulators = -get_accumulators(correlator)

"""
    coherent_integration_blocks(link, sat_state, signal_index, hw_channel) -> Int

How many primary-code blocks the link should sum into one record for this
signal, right now.

This is a **ceiling**, not the length actually used: the fold flushes whatever
has accumulated at the end of every processing chunk (see
[`flush_partial_records!`](@ref)), so a record normally spans exactly the chunk.
The ceiling only bites when a chunk covers more signal time than one symbol.

**Why records are combined at all.** A loop filter's output is a *frequency*
that is meant to act for exactly one update interval: the proportional term of
`filter_loop` is sized to remove a fraction of the measured phase error over
`Δt`. A hardware correlator's NCO, though, holds whatever word it was last given
until the next one arrives — so if a correction computed for a 1 ms interval is
left in place for 20 ms, it over-corrects by twentyfold and *injects* the phase
error it was meant to remove.

That is not hypothetical. Measured on sky over 200 s (issue #107), the rate at
which a satellite's 20 ms navigation bit lands past 90° of carrier phase — i.e.
comes out inverted — tracks how far behind the host was, at constant signal
strength:

| records the fold delivered in one 2 ms chunk | bits | bit past 90° |
|---:|---:|---:|
| 0-2 (host keeping up)  |  362 |  0.8 % |
| 3-4                    | 5215 |  1.4 % |
| 5-8                    | 3204 |  3.8 % |
| 9-16                   | 1020 | 11.0 % |
| 17-32                  |  302 | 15.9 % |
| 33+                    |   83 | 26.5 % |

Splitting the same table by bit amplitude keeps the trend (0.9 % → 8.2 % across
the same backlog range among *strong* bits only), so it is the staleness of the
feedback and not the C/N₀. One inverted bit spoils one 30-bit word, and
`GNSSDecoder` needs subframes 1, 2 and 3 to arrive clean *and* mutually
consistent — 18 s of unbroken words. At one spoiled word per second no
ephemeris ever completes, which is exactly what issue #107 saw while every
individual measurement (prompt SNR, code phase, bit clock, C/N₀) looked healthy.

Summing `n` consecutive dumps' accumulators is exactly the correlation the
device would have produced had it integrated `n` blocks, because its replicas
run continuously across a dump boundary. Handing the estimator that one record
makes `Δt` the *real* elapsed interval, and `Tracking` derives everything else
from the record's own `integrated_samples` — the loop bandwidth scaling by `1/n`
(so the proportional term shrinks to match), the FLL's integration time, and the
bit clock's block credit. The correction the device is then left holding is one
sized for the interval it will actually hold it for.

**The ceiling.** One symbol — a navigation bit for a data-bearing signal, one
secondary-code period for a pilot — because past it the data flips sign inside
the integration. Before bit/secondary sync the length is forced to 1: the sync
detectors consume exactly one prompt per code block and `Tracking`'s
`_buffer_find_bit` rejects anything else outright.

**Do not raise this to a full symbol by default.** Lengthening the integration
shrinks the carrier discriminators' unambiguous range to `±1/(4·n·T_block)`
(`atan` sees phase modulo π, `fll_disc` divides by the integration time), and a
loop whose residual frequency error is outside that range aliases and runs away
rather than pulling in. Jumping straight from 1 ms to a 20 ms integration while
the loop still carries the ±22 Hz of jitter a 1 ms update rate produces was
measured in the closed-loop reproduction to false-lock and diverge by ~1000 Hz.
A chunk-length record (2 ms ⇒ ±125 Hz) is comfortably inside; anything longer
needs the length to be ramped up as the loop settles, which this does not yet do.

The count is measured against the bit buffer's progress through the current
symbol — its own block count plus the blocks of the records emitted since the
estimator last advanced it (`pending_blocks`) — so a record truncated by the
chunk boundary lands the *next* one back on the symbol edge, and a chunk that
holds several symbols' worth of dumps is cut on every edge rather than at one
stale offset.
"""
function coherent_integration_blocks(
    link::HardwareCorrelatorLink,
    sat_state,
    signal_index,
    hw_channel,
)
    tracked_signal = Tracking.get_signals(sat_state)[signal_index]
    bit_buffer = Tracking.get_bit_buffer(tracked_signal)
    # Pre-sync the detectors need one prompt per code block, and Tracking
    # enforces it. A channel whose bit clock is about to be restarted is folded
    # the same way: its records will meet a fresh, unsynchronised bit buffer.
    has_bit_or_secondary_code_been_found(bit_buffer) || return 1
    link.bit_clock_lost[hw_channel] && return 1
    signal = get_signal(tracked_signal)
    # Consecutive dumps of an overlaid signal carry different overlay chips, so
    # summing them cancels the signal rather than accumulating it — unless the
    # overlay has already been taken off them. The host does that from the
    # moment it knows the phase ([`is_secondary_code_removed`](@ref)); until
    # then such a signal stays at one block per record.
    get_secondary_code_length(signal) == 1 ||
        is_secondary_code_removed(link, hw_channel) ||
        return 1
    # `Tracking`'s own structural ceiling, rather than this package's reading of
    # the signal's metadata: one navigation bit where there is data, one
    # secondary-code period for a pilot — and whatever a signal states for
    # itself. Galileo E5a-QP is why that last clause matters: it has neither
    # data nor an overlay, so the metadata says "one block", and one block is
    # 64.5 µs. `Tracking` states 31 (one 2 ms code cycle) for exactly that
    # reason, and taking the ceiling from there is what keeps the loops from
    # being handed fifteen thousand records a second (#133).
    blocks_per_symbol = Tracking.max_num_code_blocks_to_integrate(signal)
    blocks_per_symbol <= 1 && return 1
    requested =
        link.coherent_code_blocks == 0 ? blocks_per_symbol :
        min(link.coherent_code_blocks, blocks_per_symbol)
    # Land on the symbol boundary the bit buffer is counting toward, so a
    # truncated record is absorbed once instead of shifting every later one.
    counted =
        bit_buffer.prompt_accumulator_integrated_code_blocks +
        link.pending_blocks[hw_channel]
    remaining = blocks_per_symbol - mod(counted, blocks_per_symbol)
    max(1, min(requested, remaining))
end

"""
    coherent_integration_periods(link, sat_state, signal_index, hw_channel) -> Float64

How much signal, in primary-code periods, the link sums into one record for this
signal right now — [`coherent_integration_blocks`](@ref) capped by the link's
`max_integration_time`.

The two differ by exactly the thing issue #133 is about.
`coherent_integration_blocks` answers in whole code blocks, which is the right
unit for every signal whose code period is short enough to *be* a unit of
integration: GPS L1 C/A's 1 ms, GPS L2CM's 20 ms, Galileo E5a-QP's 64.5 µs. GPS
L2CL's code period is 1.5 s, and one block of it is not an integration length, it
is an outage — the loop filters would be handed one record and the device one NCO
correction every one and a half seconds, which no tracking loop survives.

So the answer is a `Float64`: for L2CL at the default 20 ms it is `0.0133`, and a
record is cut after 20 ms of a code period with the fraction of a period it
covers recorded rather than rounded up to one. That is the separation the issue
asks for — the tracking-update cadence is a *time*, the code wrap is a property
of the code, and neither is the other.

`max_integration_time` bites only where it has to: on a signal whose code period
is longer than it ([`allows_partial_primary_records`](@ref)). It is not a general
ceiling on the coherent integration — a receiver that asks for twenty 10 ms GPS
L1C-P blocks gets twenty, and the per-chunk flush is what decides the length in
practice (see [`coherent_integration_blocks`](@ref)). Capping a *short* code here
would silently shorten every deliberately long integration, which is a different
decision from the one this is for.
"""
function coherent_integration_periods(
    link::HardwareCorrelatorLink,
    sat_state,
    signal_index,
    hw_channel,
)
    signal = get_signal(Tracking.get_signals(sat_state)[signal_index])
    blocks = coherent_integration_blocks(link, sat_state, signal_index, hw_channel)
    allows_partial_primary_records(link, signal) || return Float64(blocks)
    min(Float64(blocks), _max_record_periods(link, signal))
end

# Add one dump to this channel's partial record and, once it spans the coherent
# integration length, hand the sum to the estimator as a single record.
#
# Summing accumulators is the whole trick: the device's replicas run
# continuously across a dump boundary, so `Σ dumps` IS the accumulator a device
# that had integrated over the whole span would have produced. The emitted
# record carries the summed `integrated_samples` and the *last* dump's
# `sample_index`, which is what makes `Tracking` treat it as one long
# integration ending there — the loop bandwidth scaling, the integration time
# the FLL divides by, and the bit clock's block credit all follow from those two
# numbers.
function _accumulate_dump!(
    link,
    track_state,
    assignment,
    sat_state,
    hw_channel,
    output,
    span,
)
    signal = get_signal(Tracking.get_signals(sat_state)[assignment.signal_index])
    target =
        coherent_integration_periods(link, sat_state, assignment.signal_index, hw_channel)
    tolerance = _block_boundary_tolerance(link, signal, hw_channel)
    # A record is cut where the NCO word changed, the way it is cut on a bit
    # edge: the dumps accumulated so far ran on one word and this one starts on
    # another, and a record straddling the switch could be attributed to
    # neither. (A switch *inside* a dump cannot be cut; `mean_nco_word` weights
    # the two words by the samples each ran for.)
    if link.partial_samples[hw_channel] > 0 && word_changes_within(
        link.nco_timelines[hw_channel],
        link.partial_end[hw_channel] - link.partial_samples[hw_channel],
        output.sample_index - output.integrated_samples,
    )
        _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
    end
    if link.partial_samples[hw_channel] == 0
        link.partial_correlator[hw_channel] = output.correlator
        link.partial_samples[hw_channel] = output.integrated_samples
        link.partial_periods[hw_channel] = span.periods
        link.partial_wraps[hw_channel] = span.wraps
    else
        link.partial_correlator[hw_channel] =
            _add_accumulators(link.partial_correlator[hw_channel], output.correlator)
        link.partial_samples[hw_channel] += output.integrated_samples
        link.partial_periods[hw_channel] += span.periods
        link.partial_wraps[hw_channel] += span.wraps
    end
    link.partial_end[hw_channel] = output.sample_index
    # Two measures of "long enough", because the two cases count in different
    # units. Where the code period is the integration unit the target is a block
    # count and the accumulation is sized in the *wraps* it has crossed, which is
    # what lands it on the bit grid; where a record is cut inside a code period
    # the target is a fraction of one and the periods are what there is to count.
    long_enough =
        allows_partial_primary_records(link, signal) ?
        link.partial_periods[hw_channel] >= target - tolerance :
        link.partial_wraps[hw_channel] >= max(1, floor(Int, target + tolerance))
    if long_enough && _record_is_emittable(link, signal, hw_channel, tolerance)
        _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
    elseif link.partial_periods[hw_channel] >= target + 1
        # The record is a whole code period past its target and still has not met
        # a block boundary to be cut on, so the device's dump grid does not
        # divide its code period. Hand over what there is rather than accumulate
        # for ever, and say so: a stream like that cannot keep the navigation bit
        # grid whatever the host does with it.
        link.misaligned_dump_boundaries += 1
        @warn "hardware correlator dumps do not align with the primary-code block " *
              "boundary; the coherent accumulation cannot be cut on it" prn =
            assignment.prn hw_channel maxlog = 10
        _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
    end
    link
end

# May this channel's part-accumulated record be handed to the tracking loops as
# it stands?
#
# For a signal whose code period is a usable integration unit: only where the
# accumulation ends on a code block boundary. `Tracking` credits its bit clock in
# whole blocks recovered from the record's own sample count, so a record cut
# inside a block is credited a block it did not cover — which slides the
# navigation bit grid by the cut, for the rest of the run. Carrying the
# part-record into the next chunk costs one update interval once; emitting it
# costs the bit stream.
#
# The boundary is the whole of the condition, and the length deliberately is not:
# the short first record after a channel is armed runs from the sample-exact
# phase load to the next code wrap, so it is a *fraction* of a block that ends on
# a boundary — and `Tracking` expects exactly that, crediting the first
# integration one block whatever it covered. Everything after it starts on a
# boundary, so ending on one means a whole number of blocks.
#
# For a long code (`allows_partial_primary_records`) there is no grid to keep and
# waiting for the boundary means waiting a code period, so anything goes.
function _record_is_emittable(link, signal, hw_channel, tolerance)
    link.partial_samples[hw_channel] > 0 || return false
    allows_partial_primary_records(link, signal) && return true
    phase = link.block_phase[hw_channel]
    isnan(phase) || phase == 0.0
end

# ─────────────────────────────────────────────────────────────────────────────
# Where a record sits on the primary-code block grid (#133)
#
# The old accounting asked one question — "how many whole code blocks is this?"
# — and answered it with `max(1, round(…))`. That conflates four separate
# quantities, and the rounding and the floor each break a different one:
#
#   * the *dump integration duration* is whatever the device accumulated, which
#     may be a fraction of a code period;
#   * the *primary-code wraps* a record covers is how often the replica ran
#     through its code inside it, which for a short dump is usually zero;
#   * the *navigation/secondary-code block count* rides those wraps, because one
#     overlay chip and one bit-clock credit belong to one code period;
#   * the *tracking-update cadence* is how often the loops are handed a record,
#     which for a 1.5 s code has to be far shorter than a code period.
#
# So a record is measured as a `Float64` count of code periods and placed on the
# block grid by a running fraction, `link.block_phase`. A record that covers a
# quarter of a period advances the fraction by a quarter and completes no wrap;
# four of them complete one. Nothing is rounded up, and nothing is invented.
# ─────────────────────────────────────────────────────────────────────────────

# Primary-code periods a record spans — exactly, as a fraction. Doppler-adjusted,
# because the fraction accumulates over a whole run where a rounded block count
# never did: the nominal rate is what `Tracking` rounds with, and the two agree
# to far better than the half period that rounding cares about.
function _record_code_periods(link, sat_state, signal, output, hw_channel)
    code_rate =
        ustrip(Hz, get_code_frequency(signal)) +
        ustrip(Hz, uconvert(Hz, get_code_doppler(sat_state)))
    output.integrated_samples * code_rate /
    (get_code_length(signal) * link.channel_sampling_freq[hw_channel])
end

# How close to a block boundary still counts as being on it, as a fraction of a
# code period: one sample, and never less than a chip and a half.
#
# Both ends of that matter. A sampled code is piecewise constant, so a phase is
# only ever known to within the sample it was latched on; and a device reporting
# the replica phase at the sample that *completes* a code period reports a value
# just below the code length rather than zero (see `CorrelatorDump.code_phase`),
# which is the same boundary written the other way round. Snapping both to
# exactly `0.0` is what lets every later test be an equality.
_block_boundary_tolerance(link, signal, hw_channel) =
    max(
        1.5,
        ustrip(Hz, get_code_frequency(signal)) / link.channel_sampling_freq[hw_channel],
    ) / get_code_length(signal)

# A block fraction with the boundary snapped to exactly zero.
_snap_block_fraction(fraction, tolerance) =
    (fraction < tolerance || fraction > 1 - tolerance) ? 0.0 : fraction

"""
    RecordBlockSpan

Where one record sits on its signal's primary-code block grid: the code periods
it spans (`periods`, fractional), the code wraps it completed (`wraps`), and the
fraction of a code period its end falls at (`block_phase`, exactly `0.0` on a
boundary).

Measured once per dump as each record is appended, and handed to everything that
needs it, so the overlay removal, the continuity accounting and the coherent
accumulation cannot come to three different answers about the same record.
"""
struct RecordBlockSpan
    periods::Float64
    wraps::Int
    block_phase::Float64
end

# Where a record *starts* inside its code block. Read off the channel's running
# fraction where there is one — advanced over any hole between the previous
# record and this one, so a gap does not silently shift the grid — and otherwise
# recovered from the device's own reported replica phase, which is the only
# thing that can place the first record of a channel that dumps inside a code
# period.
#
# Failing both, the record is taken to *end* on a block boundary, because a
# device on the historical contract dumps on the code wrap and nowhere else.
# That covers the one record where it matters: the short first record after a
# channel is armed, which runs from the sample-exact phase load to the next code
# wrap and is a fraction of a block rather than one. Assuming it *started* on a
# boundary instead would leave the channel's whole grid a fraction of a block
# out for as long as the assignment lasts.
function _record_start_fraction(link, hw_channel, output, periods, reported, tolerance)
    previous_end = link.last_record_end[hw_channel]
    standing = link.block_phase[hw_channel]
    if previous_end != typemin(Int64) && !isnan(standing)
        record_start = output.sample_index - output.integrated_samples
        periods_per_sample = periods / max(1, output.integrated_samples)
        hole = (record_start - previous_end) * periods_per_sample
        return _snap_block_fraction(mod(standing + hole, 1.0), tolerance)
    end
    ends_at = isnan(reported) ? 0.0 : reported
    _snap_block_fraction(mod(ends_at - periods, 1.0), tolerance)
end

# Measure one record against the block grid. Pure: it reads the channel's
# standing fraction but moves nothing, so the ingest path can consult the answer
# before deciding whether the record is usable at all.
function _record_block_span(link, sat_state, signal_index, hw_channel, output, code_phase)
    signal = get_signal(Tracking.get_signals(sat_state)[signal_index])
    code_length = get_code_length(signal)
    tolerance = _block_boundary_tolerance(link, signal, hw_channel)
    periods = _record_code_periods(link, sat_state, signal, output, hw_channel)
    reported = isnan(code_phase) ? NaN : mod(code_phase / code_length, 1.0)
    start = _record_start_fraction(link, hw_channel, output, periods, reported, tolerance)
    wraps = max(0, floor(Int, start + periods + tolerance))
    block_phase = _snap_block_fraction(
        isnan(reported) ? clamp(start + periods - wraps, 0.0, 1.0) : reported,
        tolerance,
    )
    RecordBlockSpan(periods, wraps, block_phase)
end

"""
    primary_code_wraps(link, hw_channel) -> Int

Primary-code periods this hardware channel's record stream has completed since
the channel was assigned.

The count the *code* did, which for a device dumping inside a code period is not
the number of records it produced: 1500 one-millisecond records of GPS L2CL
complete one wrap, and 1499 of them complete none. Nothing here is rounded up —
a record shorter than a code period never completes one.
"""
primary_code_wraps(link::HardwareCorrelatorLink, hw_channel::Integer) =
    Int(link.primary_wraps[hw_channel])

"""
    primary_code_block_phase(link, hw_channel) -> Float64

Where this hardware channel's next record begins inside its primary-code block,
as a fraction of a code period in `[0, 1)` — exactly `0.0` on a block boundary,
and `NaN` before the channel has folded a record.

This is the partial-primary metadata the rest of the accounting hangs on: the
overlay chip a record carries, whether a record may be handed to the loops
(`Tracking` counts whole blocks), and how far through a 1.5 s code the receiver
currently is, are all read off it. Re-anchored to
[`CorrelatorDump`](@ref)`.code_phase` on every record that reports one.
"""
primary_code_block_phase(link::HardwareCorrelatorLink, hw_channel::Integer) =
    link.block_phase[hw_channel]

"""
    record_integration_periods(link, hw_channel) -> Float64

Primary-code periods accumulated into this channel's open record so far, as a
fraction — `0.0` when nothing is accumulated.
"""
record_integration_periods(link::HardwareCorrelatorLink, hw_channel::Integer) =
    link.partial_periods[hw_channel]

"""
    allows_partial_primary_records(link, signal) -> Bool

Whether the link may hand the tracking loops a record spanning *less* than one
primary-code period of `signal`.

`false` for every signal whose code period fits inside the link's
`max_integration_time`, and that is the safe answer: `Tracking` credits its bit
clock and its overlay counter in whole code blocks, so a record cut inside a
block would slide the navigation bit grid by the cut. Records are then whole
blocks and a part-accumulated one is carried across the chunk boundary rather
than emitted short.

`true` for a code period longer than the integration length — GPS L2CL's 1.5 s
against a 20 ms default — where the alternative is one loop update per code
period. Such a signal has no navigation data and no overlay to keep a grid for
(`Tracking`'s sync detector reports nothing to find), so the cut costs nothing
that a wait of 1.5 s would not cost far more of.
"""
allows_partial_primary_records(link::HardwareCorrelatorLink, signal) =
    code_period_seconds(signal) > link.max_integration_time * (1 + 1e-9)

# Code periods `max_integration_time` is worth for this signal — the ceiling on
# one record, in the units the accumulation counts in.
_max_record_periods(link, signal) = link.max_integration_time / code_period_seconds(signal)

# Hand the accumulated record to the estimator and start a fresh one. A no-op
# when nothing is accumulated, so it is safe to call as a flush.
function _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
    link.partial_samples[hw_channel] == 0 && return link
    append_correlator_output!(
        track_state,
        _retag_spacing(
            get_correlator(sat_state, assignment.signal_index),
            Tracking.CorrelatorOutput(
                link.partial_correlator[hw_channel],
                link.partial_samples[hw_channel],
                link.partial_end[hw_channel],
            ),
            link.channel_scale[hw_channel],
        ),
        assignment.group_key,
        assignment.prn,
        assignment.signal_index,
    )
    # What `Tracking` will credit this record to the bit clock: the whole code
    # blocks its sample count rounds to, which is *its* arithmetic, so the two
    # cannot come to different answers about the same record. A partial-primary
    # record rounds to none, and is counted as the partial record it is instead.
    blocks = round(Int, link.partial_periods[hw_channel])
    link.pending_blocks[hw_channel] += blocks
    blocks == 0 && (link.partial_primary_records += 1)
    link.partial_samples[hw_channel] = 0
    link.partial_periods[hw_channel] = 0.0
    link.partial_wraps[hw_channel] = 0
    link.partial_end[hw_channel] = typemin(Int64)
    link
end

# Throw away a partial record without emitting it. Used where the accumulation
# cannot be completed or attributed: a channel changing occupant.
function _discard_partial!(link, hw_channel)
    link.partial_samples[hw_channel] = 0
    link.partial_periods[hw_channel] = 0.0
    link.partial_wraps[hw_channel] = 0
    link.partial_end[hw_channel] = typemin(Int64)
    link
end

# Sum two correlators' accumulators, keeping everything else from the first.
_add_accumulators(a::Tracking.AbstractCorrelator, b::Tracking.AbstractCorrelator) =
    @set a.accumulators = get_accumulators(a) .+ get_accumulators(b)

# Take the dump's accumulators, but the *host's* correlator spacing metadata.
#
# `dll_disc` does not use the spacing we programmed the device with: it recovers
# it from the correlator handed to it, via
# `get_early_late_sample_spacing(correlator, …)` → the correlator's own
# `preferred_early_late_to_prompt_code_shift`. So if a vendor builds the dump's
# correlator with a different preferred shift than the tracked satellite's, the
# discriminator normalises by a spacing the device never used. That is not a
# small gain error: the normalisation factor is `(2 - distance_in_chips) / 2`,
# which goes *negative* once the assumed distance exceeds two chips, inverting
# the DLL so the loop drives the code phase away from the peak.
#
# Rather than leave that as a contract a vendor has to get right (and a silent,
# hard-to-attribute failure when they do not), the host substitutes its own
# correlator as the template: same type, same preferred shift as the satellite
# being tracked — which is by construction the spacing `assign_channel!` handed
# the device — with only the accumulators taken from the wire. The vendor is
# then responsible for exactly one thing, the accumulator values and their
# `[late, prompt, early]` order.
#
# The accumulators are also brought onto the host's amplitude scale here, by
# dividing out the channel's `channel_scale` — the amplitude of the replica the
# device wipes off with (where the host's own correlator uses unit amplitude),
# times the ratio of the code amplitude the device's replica actually has to the
# one GNSSSignals models (see `replica_code_amplitude`). It is a pure scale
# factor, so every discriminator (a ratio) and the old moment-ratio C/N₀
# estimators are blind to it. `NoiseRefCN0Estimator` is not: it divides the
# prompt power by a noise density measured somewhere else, so a device reporting
# `g ×` the host's prompt reads `20·log10(g)` dB too high — 42 dB for a replica
# of amplitude 127. See `append_noise_observations!` for where the floor comes
# from.
#
# Only the record's *meaningful* taps are read: the template's own accumulator
# count decides how many leading slots of the wire correlator are taken, so one
# link can carry a wide record for a five-tap satellite and a narrow one for a
# three-tap satellite (see `CorrelatorDump`). `_append_dump!` has already
# refused any record whose declared tap count is not the template's, so the
# slots taken here are exactly the ones the device filled.
# Antenna count off the correlator *type*, so the noise accumulator can be sized
# before any dump has arrived.
_num_ants(::Type{<:Tracking.AbstractCorrelator{M}}) where {M} = M

_retag_spacing(
    template::Tracking.AbstractCorrelator,
    output::Tracking.CorrelatorOutput,
    scale::Float64,
) = Tracking.CorrelatorOutput(
    @set(
        template.accumulators = _leading_taps(
            get_accumulators(template),
            get_accumulators(output.correlator),
            scale,
        )
    ),
    output.integrated_samples,
    output.sample_index,
)

# The first `N` accumulators of `wire`, scaled — `N` taken from the template's
# own `SVector` type, so the result is the exact type the template's field
# wants and the whole thing stays allocation-free.
@inline _leading_taps(::SVector{N,T}, wire::AbstractVector, scale::Float64) where {N,T} =
    SVector{N,T}(ntuple(index -> wire[index] / scale, Val(N)))

"""
    push_nco_updates!(link, track_state, boundary) -> Int

Push one [`NCOUpdate`](@ref) per assigned hardware channel and return how many
were sent.

Called once per chunk, right after the estimator folded it, so the Dopplers read
back are the newest. All updates are scheduled at the same future *instant* —
`feedback_delay_epochs × Δ` past the *newest record the host has seen* — which is
what keeps the loop delay a known constant instead of PCIe jitter. That instant
is computed on the receiver timebase and then written on each channel's **own
band counter**, so a multi-band device reads every `apply_at_sample` in the
units its band is clocked in.

The reference is `latest_sample_index` rather than the epoch `boundary` because
the two part company exactly when it matters. `boundary` is where the fold grid
has got to; when the host is behind, that is in the past, and scheduling a
correction at a sample the device passed milliseconds ago asks it to apply the
update late by however far behind the host is — or, on a device that honours the
schedule strictly, to discard it. Anchoring to the newest record keeps the
correction a fixed, small distance in the *device's* future either way.

Every update the device accepts is entered in its channel's
[`NCOTimeline`](@ref), so the estimator can later attribute records to the word
that ran under them.

A full ring means the device's writer is not keeping up; the updates are
dropped rather than blocking the receiver, counted in `link.dropped_nco_updates`
and kept out of the timelines — the device never saw them, so every channel
runs on its previous word for another chunk.
"""
function push_nco_updates!(link::HardwareCorrelatorLink, track_state, boundary)
    channel = link.ncos
    apply_at_sample = nco_apply_at_sample(link, boundary)
    empty!(link.nco_buffer)
    for hw_channel in eachindex(link.assignments)
        assignment = link.assignments[hw_channel]
        isnothing(assignment) && continue
        # Only the driver signal's channel carries the loop; a passenger
        # component shares the satellite's Doppler, so it gets the same numbers.
        sat_state = get_sat_state(track_state, assignment.group_key, assignment.prn)
        push!(
            link.nco_buffer,
            NCOUpdate(
                hw_channel,
                assignment.prn,
                get_carrier_doppler(sat_state),
                get_code_doppler(sat_state),
                # One landing *instant* for the whole fold, stated on each
                # channel's own band counter — the same moment is a different
                # integer on a 4 MS/s band and a 5 MS/s one, and a device
                # applying the reference band's integer on another band would
                # land the correction a quarter of the feedback delay early or
                # late.
                _band_sample(link, hw_channel, apply_at_sample),
            ),
        )
    end
    isempty(link.nco_buffer) && return 0
    if length(link.nco_buffer) > n_avail_space(channel)
        link.dropped_nco_updates += length(link.nco_buffer)
        @warn "NCO feedback ring full; this chunk's corrections were dropped and every " *
              "hardware channel free-runs on its previous word" dropped_total =
            link.dropped_nco_updates maxlog = 10
        return 0
    end
    put!(channel, link.nco_buffer)
    for update in link.nco_buffer
        schedule_word!(
            link.nco_timelines[update.channel],
            update.apply_at_sample,
            update.carrier_doppler,
            update.code_doppler,
        )
    end
    length(link.nco_buffer)
end

# Free slots in a `PipeChannel`. `n_avail` counts queued items, so the space left
# is the (usable) capacity minus that.
n_avail_space(channel::PipeChannel) = channel.capacity - 1 - Base.n_avail(channel)
