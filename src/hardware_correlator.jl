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

One correlator dump streamed from a hardware correlator: `Tracking`'s
[`CorrelatorOutput`](@extref Tracking.CorrelatorOutput) plus the routing needed
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
end

CorrelatorDump(
    channel::Integer,
    prn::Integer,
    output::Tracking.CorrelatorOutput,
    code_phase::Real = NaN,
) = CorrelatorDump(Int32(channel), Int32(prn), output, Float64(code_phase))

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
"""
assign_channel!(sdr::AbstractHardwareCorrelatorSDR, args...; kwargs...) =
    _not_implemented("assign_channel!", sdr)

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
"""
correlator_gain(::AbstractHardwareCorrelatorSDR) = 1

# ─────────────────────────────────────────────────────────────────────────────
# Host-side ingest state
# ─────────────────────────────────────────────────────────────────────────────

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
    primary code period of `reference_signal`.
  - `feedback_delay_epochs` — how many epochs ahead an [`NCOUpdate`](@ref) is
    scheduled, i.e. the `n` in "the correction from epoch `k` applies at
    `k + n`". Must be large enough to cover the PCIe round trip.
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
"""
mutable struct HardwareCorrelatorLink{
    S<:AbstractHardwareCorrelatorSDR,
    C<:Tracking.AbstractCorrelator,
}
    const sdr::S
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
    # so it is deterministic and replayable.
    const epoch_length::Int
    # Sampling frequency (Hz) of the band the epoch grid lives on. Used to turn
    # device-sample spans into chips for the code-phase bookkeeping.
    const sampling_freq_hz::Float64
    # Amplitude of the replica the device wipes off with, relative to the unit
    # replica the host's own correlator would use. Divided out of every
    # accumulator on ingest — see `_retag_spacing`.
    const correlator_gain::Float64
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
    # ── Record continuity, per hardware channel ───────────────────────────────
    # A channel's records tile the sample axis: each one covers
    # `[sample_index - integrated_samples, sample_index)`, so the next one must
    # start exactly where this one ended. This vector holds that expected start
    # (`typemin` = no record folded on the channel yet, e.g. right after an
    # assignment). Anything else is a discontinuity, and the two counters below
    # record it — see `_account_record_continuity!` for why the bit clock, not
    # the loop filters, is what a lost record damages.
    const last_record_end::Vector{Int64}
    # Device samples this channel's record stream is missing, i.e. summed
    # forward gaps. Reset when the channel is (re)assigned.
    const lost_record_samples::Vector{Int64}
    # How far this channel's records have overlapped (a record starting before
    # the previous one ended: a duplicate or a device counter step back).
    const overlapping_record_samples::Vector{Int64}
    # How many epochs the fold loop will replay in one chunk before treating the
    # shortfall as a gap and resynchronising the grid (see `fold_closed_epochs!`).
    const max_catchup_epochs::Int
    # ── Coherent pre-accumulation, per hardware channel ───────────────────────
    # Requested coherent integration length in primary-code blocks, or 0 for
    # "one full symbol" (see `coherent_integration_blocks`).
    const coherent_code_blocks::Int
    # The partially accumulated record: summed accumulators, the samples and
    # whole code blocks they span, and the `sample_index` of the newest dump in
    # it. `partial_blocks == 0` means "nothing accumulated", in which case
    # `partial_correlator` is undefined rather than zero — a correlator type has
    # no zero without an instance to take it from.
    const partial_correlator::Vector{C}
    const partial_samples::Vector{Int64}
    const partial_blocks::Vector{Int}
    const partial_end::Vector{Int64}
    # ── Noise reference ───────────────────────────────────────────────────────
    # Where the C/N₀ estimator's noise density comes from: `:channel` spends a
    # hardware channel on an open-loop despread (the documented FPGA recipe),
    # `:samples` meters Σ|x|² off the raw stream. See `append_noise_observations!`.
    const noise_source::Symbol
    # The channel the open-loop reference occupies, or 0 while it has none. It
    # is never handed to a satellite and never receives an `NCOUpdate`.
    noise_channel::Int
    # The decoy PRN it currently replicates, and how many epochs since it was
    # last re-armed onto a fresh PRN / phase / carrier offset.
    noise_prn::Int32
    noise_epochs_since_rearm::Int
    const noise_rearm_epochs::Int
    # This chunk's pooled accumulation: `Σ b·bᴴ` over every tap of every dump
    # the noise channel produced (a 1×1 matrix for one antenna), the number of
    # independent looks that pooled, and the samples one look spans.
    const noise_accumulator::Matrix{ComplexF64}
    noise_looks::Int
    noise_samples_per_look::Int
    # Sample index at which the currently open epoch closes. `typemin` until the
    # first record arrives and anchors the grid (see `_anchor_epoch_grid!`).
    next_epoch_boundary::Int
    # Highest `sample_index` seen so far; a record at or past the boundary is
    # what closes the open epoch.
    latest_sample_index::Int
    # Raw samples consumed from `raw_sample_channel` since the run began — the
    # time base `assign_channel!` hands over on.
    samples_consumed::Int
    # Diagnostics.
    dropped_dumps::Int
    stale_dumps::Int
    unassignable_signals::Int
    skipped_epochs::Int
    # Number of forward gaps seen across all channels (the *samples* they cost
    # are per channel, above).
    lost_record_gaps::Int
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
    correlator_gain = nothing,
    noise_source::Symbol = :channel,
    noise_rearm_interval = 1u"s",
)
    interval = something(
        doppler_update_interval,
        get_code_length(reference_signal) / get_code_frequency(reference_signal),
    )
    epoch_length = round(Int, upreferred(interval * sampling_freq))
    epoch_length > 0 || throw(
        ArgumentError(
            "doppler_update_interval $interval is shorter than one sample period at " *
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
    noise_rearm_epochs =
        max(1, round(Int, upreferred(noise_rearm_interval * sampling_freq) / epoch_length))
    isnothing(coherent_code_blocks) ||
        coherent_code_blocks >= 1 ||
        throw(
            ArgumentError(
                "coherent_code_blocks must be at least 1 or nothing (got $coherent_code_blocks)",
            ),
        )

    dump_type = eltype(correlator_dump_channel(sdr))
    dump_type <: CorrelatorDump || throw(
        ArgumentError(
            "correlator_dump_channel(::$(typeof(sdr))) must have eltype <: CorrelatorDump, " *
            "got $dump_type",
        ),
    )
    n = num_hardware_channels(sdr)

    HardwareCorrelatorLink{typeof(sdr),_correlator_type(dump_type)}(
        sdr,
        Union{Nothing,HardwareChannelAssignment}[nothing for _ = 1:n],
        Dict{HardwareChannelAssignment,Int}(),
        dump_type[],
        dump_type[],
        NCOUpdate[],
        epoch_length,
        Float64(ustrip(uconvert(Hz, sampling_freq))),
        Float64(gain),
        Int(feedback_delay_epochs),
        Int(max_dumps_per_drain),
        fill(typemin(Int64), n),
        fill(typemin(Int64), n),
        fill(NaN, n),
        fill(typemin(Int64), n),
        zeros(Int64, n),
        zeros(Int64, n),
        Int(max_catchup_epochs),
        isnothing(coherent_code_blocks) ? 0 : Int(coherent_code_blocks),
        Vector{_correlator_type(dump_type)}(undef, n),
        zeros(Int64, n),
        zeros(Int, n),
        fill(typemin(Int64), n),
        noise_source,
        0,
        Int32(0),
        0,
        noise_rearm_epochs,
        zeros(ComplexF64, _num_ants(_correlator_type(dump_type)), _num_ants(_correlator_type(dump_type))),
        0,
        0,
        typemin(Int),
        typemin(Int),
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

    # The raw stream is still the receiver's clock: count what this chunk
    # delivered so the handover time base stays aligned with it.
    link.samples_consumed += _chunk_num_samples(band_measurements)

    # A dump only makes sense for a satellite the device is actually
    # correlating, so reconcile the channel table with the tracking state first:
    # this chunk's acquisitions get hardware channels, and satellites the
    # receiver has dropped give theirs back.
    sync_hardware_channels!(link, track_state, band_systems, band_measurements)

    link.dropped_dumps += dropped_dump_count!(link.sdr)

    drain_dumps!(link)
    fold_closed_epochs!(link, track_state, band_measurements, band_systems)

    track_state
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
satellite, so one observation serves every satellite tracking that signal.
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

# Samples in this chunk. Every band advances from equal-length frames of one
# time base (`receive` enforces it), so the first band speaks for all of them.
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

Keep one hardware channel running an **open-loop despread** as the C/N₀
estimator's noise reference, and re-arm it periodically.

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
    signal = _noise_reference_signal(band_systems)
    isnothing(signal) && return link
    if link.noise_channel == 0
        hw_channel = _find_free_channel(link)
        isnothing(hw_channel) && return link
        link.noise_channel = hw_channel
        link.noise_epochs_since_rearm = link.noise_rearm_epochs
    end
    link.noise_epochs_since_rearm >= link.noise_rearm_epochs || return link
    _arm_noise_channel!(link, signal, _band_sampling_frequency(band_measurements, signal))
end

# The reference despreads one signal, and it is the same one the epoch grid and
# the handovers are referenced to: the ranging signal of the first system.
function _noise_reference_signal(band_systems)
    for systems in band_systems, system in systems
        for signal in tracking_signals(system)
            return signal
        end
    end
    nothing
end

function _arm_noise_channel!(link, signal, sampling_freq)
    code_frequency = get_code_frequency(signal)
    # Rotate through the family rather than picking one and staying: a PRN whose
    # cross-correlation with a strong satellite happens to be unusually high is
    # then one observation in the window, not the window.
    link.noise_prn = Int32(mod(Int(link.noise_prn), 32) + 1)
    # Taps a whole chip apart, so the three of them are three independent looks
    # at the same noise — which is exactly what pooling them assumes.
    el_sample_spacing = max(1, round(Int, 2 * link.sampling_freq_hz / ustrip(Hz, code_frequency)))
    assign_channel!(
        link.sdr,
        link.noise_channel,
        Int(link.noise_prn),
        (rand() * 10_000 - 5_000) * Hz,   # carrier dither, ±5 kHz
        0.0Hz,                            # open loop: the replica free-runs
        rand() * get_code_length(signal), # uniform code phase
        link.samples_consumed;
        el_sample_spacing,
        signal,
    )
    link.noise_epochs_since_rearm = 0
    # A re-arm invalidates whatever was part-accumulated against the old PRN.
    _reset_noise_accumulator!(link)
    link
end

function _reset_noise_accumulator!(link)
    fill!(link.noise_accumulator, zero(ComplexF64))
    link.noise_looks = 0
    link.noise_samples_per_look = 0
    link
end

# Pool one noise dump: `Σ b·bᴴ` over its taps. The taps are kept apart for a
# satellite because their differences are the discriminants; here they are three
# independent looks and nothing about their relative values means anything, so
# they are summed. For an antenna array the pooled payload is the array's
# spatial covariance, whose diagonal is each antenna's own floor.
function _accumulate_noise_dump!(link, output)
    accumulators = get_accumulators(output.correlator)
    for tap in accumulators
        _add_outer!(link.noise_accumulator, tap)
        link.noise_looks += 1
    end
    # Every tap of one dump integrates the same samples, so the span of a look
    # is the dump's own length, counted once.
    link.noise_samples_per_look = output.integrated_samples
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
    link.noise_looks == 0 && return track_state
    signal = _noise_reference_signal(band_systems)
    isnothing(signal) && return track_state
    sampling_freq = _band_sampling_frequency(band_measurements, signal)
    observation = Tracking.noise_observation_from_correlator(
        _pooled_noise(link),
        link.noise_looks,
        link.noise_looks * link.noise_samples_per_look,
        sampling_freq;
        prn = Int(link.noise_prn),
        duration = link.noise_samples_per_look / sampling_freq,
    )
    _append_signal_noise!(
        track_state,
        observation,
        _flatten_systems(map(tracking_signals, _flatten_systems(band_systems))),
        keys(track_state.noise_estimators),
    )
    _reset_noise_accumulator!(link)
    track_state
end

_pooled_noise(link) =
    size(link.noise_accumulator, 1) == 1 ? real(link.noise_accumulator[1, 1]) :
    SMatrix{size(link.noise_accumulator, 1),size(link.noise_accumulator, 2),ComplexF64}(
        link.noise_accumulator,
    )

function release_stale_channels!(link, track_state)
    for hw_channel in eachindex(link.assignments)
        assignment = link.assignments[hw_channel]
        isnothing(assignment) && continue
        _is_tracked(track_state, assignment) && continue
        release_channel!(link.sdr, hw_channel)
        link.assignments[hw_channel] = nothing
        delete!(link.channel_of, assignment)
        link.phase_ref_sample[hw_channel] = typemin(Int64)
        link.anchor_sample[hw_channel] = typemin(Int64)
        link.anchor_code_phase[hw_channel] = NaN
        link.last_record_end[hw_channel] = typemin(Int64)
        link.lost_record_samples[hw_channel] = 0
        link.overlapping_record_samples[hw_channel] = 0
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
        for sat_state in get_sat_states(track_state, group_key)
            prn = get_prn(sat_state)
            for (signal_index, tracked_signal) in enumerate(get_signals(sat_state))
                assignment = HardwareChannelAssignment(group_key, prn, signal_index)
                haskey(link.channel_of, assignment) && continue
                hw_channel = _find_free_channel(link)
                if isnothing(hw_channel)
                    # More tracked signals than the gateware has replica sets.
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

function _find_free_channel(link)
    for hw_channel in eachindex(link.assignments)
        hw_channel == link.noise_channel && continue
        isnothing(link.assignments[hw_channel]) && return hw_channel
    end
    nothing
end

function _assign!(link, hw_channel, assignment, sat_state, tracked_signal, sampling_freq)
    signal = get_signal(tracked_signal)
    correlator = get_correlator(tracked_signal)
    code_frequency = get_code_frequency(signal)
    # Quantise exactly the way Tracking does, and hand the device the whole
    # number of samples it must offset the Early and Late replicas by — see
    # `assign_channel!`.
    el_sample_spacing =
        Tracking.get_early_late_sample_spacing(correlator, sampling_freq, code_frequency)
    assign_channel!(
        link.sdr,
        hw_channel,
        assignment.prn,
        get_carrier_doppler(sat_state),
        get_code_doppler(sat_state),
        get_code_phase(sat_state),
        link.samples_consumed;
        el_sample_spacing,
        signal,
    )
    link.assignments[hw_channel] = assignment
    link.channel_of[assignment] = hw_channel
    # The sat's `code_phase` is the acquisition seed, which lives on the host's
    # raw-sample axis; it cannot be placed on the device counter until the first
    # anchored dump arrives, so the reference starts unknown.
    link.phase_ref_sample[hw_channel] = typemin(Int64)
    link.anchor_sample[hw_channel] = typemin(Int64)
    link.anchor_code_phase[hw_channel] = NaN
    # A fresh occupant starts a fresh record stream: the previous satellite's
    # end sample says nothing about where this one's first record begins.
    link.last_record_end[hw_channel] = typemin(Int64)
    link.lost_record_samples[hw_channel] = 0
    link.overlapping_record_samples[hw_channel] = 0
    _discard_partial!(link, hw_channel)
    link
end

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
were taken.

Non-blocking by construction: it takes exactly `n_avail` records (capped by
`max_dumps_per_drain`), so a chunk that finds the ring empty does nothing rather
than parking the receiver's processing task. Pacing comes from the raw stream,
which is the receiver's clock; the dump ring only has to be drained faster than
the device fills it.
"""
function drain_dumps!(link::HardwareCorrelatorLink)
    channel = correlator_dump_channel(link.sdr)
    available = min(Base.n_avail(channel), link.max_dumps_per_drain)
    available == 0 && return 0
    resize!(link.drain_buffer, available)
    take!(channel, link.drain_buffer)
    for dump in link.drain_buffer
        push!(link.pending, dump)
        # The epoch clock advances on *any* record, strobe or not: that is what
        # lets a silent channel stall the loop only when the device also stops
        # strobing.
        link.latest_sample_index = max(link.latest_sample_index, dump.output.sample_index)
    end
    _anchor_epoch_grid!(link)
    available
end

# Anchor the epoch grid to the first record ever seen. The grid is defined on the
# device's own counter, whose origin the host does not know a priori, so the
# first record's sample index defines epoch 0 and every boundary follows from Δ.
function _anchor_epoch_grid!(link)
    link.next_epoch_boundary == typemin(Int) || return link
    isempty(link.pending) && return link
    first_index = minimum(dump -> dump.output.sample_index, link.pending)
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
        link.noise_epochs_since_rearm += 1
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
    Tracking.estimate_dopplers_and_filter_prompt!(track_state, band_measurements)
    push_nco_updates!(link, track_state, boundary)
    folds
end

"""
    flush_partial_records!(link, track_state) -> link

Hand every hardware channel's part-accumulated record to the estimator, so a
chunk's worth of dumps reaches the loop filters as one record spanning the whole
chunk. Called once per chunk, immediately before the estimator runs.

Channels with nothing accumulated — and channels whose satellite the receiver
has already dropped — are skipped.
"""
function flush_partial_records!(link::HardwareCorrelatorLink, track_state)
    for hw_channel in eachindex(link.assignments)
        link.partial_blocks[hw_channel] == 0 && continue
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
        _emit_partial!(
            link,
            track_state,
            assignment,
            sat_states[assignment.prn],
            hw_channel,
        )
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
pseudoranges are built on. Satellites without an anchor yet (assigned, but no
dump seen) keep their acquisition seed: it lives on the host's raw-sample axis,
which the link cannot place on the device counter, and the DLL pull-in doesn't
need it to be moved.

The whole-code-period count picked up while unanchored is arbitrary; that is
fine, because the bit-sync phase snap (which runs *after* this in the same
fold) re-windows `code_phase` from the bit buffer and preserves only the
within-code-period part — exactly the part the anchor makes exact.
"""
function advance_code_phases!(link::HardwareCorrelatorLink, track_state, boundary)
    for hw_channel in eachindex(link.assignments)
        assignment = link.assignments[hw_channel]
        (isnothing(assignment) || assignment.signal_index != 1) && continue
        sat_states = get_sat_states(track_state, assignment.group_key)
        haskey(sat_states, assignment.prn) || continue
        sat_state = sat_states[assignment.prn]

        signals = Tracking.get_signals(sat_state)
        signal = get_signal(first(signals))
        code_length = get_code_length(signal)
        chips_per_sample =
            (ustrip(Hz, get_code_frequency(signal)) +
             ustrip(Hz, uconvert(Hz, get_code_doppler(sat_state)))) / link.sampling_freq_hz

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

# Append every pending output that ended before `boundary`, oldest first, and
# drop it from `pending`. Records at or past the boundary belong to the next
# epoch and stay.
function append_epoch_outputs!(link, track_state, boundary)
    sort!(link.pending; by = dump -> dump.output.sample_index)
    keep = 0
    for dump in link.pending
        if dump.output.sample_index >= boundary
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
# only the decoder notices, and only as "a valid preamble never appears again".
# `Tracking.advance_bit_clock` is what puts the clock back, and the gap record
# appended below is how this path asks for it.
function _account_record_continuity!(
    link,
    track_state,
    assignment,
    sat_state,
    hw_channel,
    output,
)
    expected_start = link.last_record_end[hw_channel]
    record_start = output.sample_index - output.integrated_samples
    if expected_start != typemin(Int64)
        gap = record_start - expected_start
        if gap > 0
            link.lost_record_samples[hw_channel] += gap
            link.lost_record_gaps += 1
            # Close the coherent accumulation *before* the hole: the records
            # after it are a different stretch of signal, and the gap record has
            # to reach the bit clock between the two. The partial comes out short
            # (weak, like the bit straddling the gap) and the next accumulation
            # re-aligns to the symbol boundary, which is exactly what
            # `coherent_integration_blocks` measures against the bit buffer.
            _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
            # Hand the fold a record covering exactly the missing span with a
            # zeroed correlator. `Tracking` reads that as "this much time passed
            # and nothing is known about it": the loop filters, the prompt
            # filter and the C/N0 estimator skip it (they would only be poisoned
            # by a zero prompt), while the bit clock is credited the blocks it
            # covers. The bit straddling the gap comes out weak, and every bit
            # after it stays on the 20 ms grid.
            append_correlator_output!(
                track_state,
                Tracking.CorrelatorOutput(
                    zero(get_correlator(sat_state, assignment.signal_index)),
                    gap,
                    record_start,
                ),
                assignment.group_key,
                assignment.prn,
                assignment.signal_index,
            )
        elseif gap < 0
            link.overlapping_record_samples[hw_channel] -= gap
        end
    end
    link.last_record_end[hw_channel] = output.sample_index
    link
end

function _append_dump!(link, track_state, dump)
    hw_channel = Int(dump.channel)
    checkbounds(Bool, link.assignments, hw_channel) || (link.stale_dumps += 1; return link)
    if hw_channel == link.noise_channel
        # A dump still carrying the previous decoy PRN was produced before the
        # re-arm took effect; pooling it would credit the window a look at a
        # replica the accumulator is no longer about.
        dump.prn == link.noise_prn ? _accumulate_noise_dump!(link, dump.output) :
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
    _account_record_continuity!(
        link,
        track_state,
        assignment,
        sat_state,
        hw_channel,
        dump.output,
    )
    _accumulate_dump!(link, track_state, assignment, sat_state, hw_channel, dump.output)
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

"""
    coherent_integration_blocks(link, sat_state, signal_index) -> Int

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

The count is measured against the bit buffer's own progress through the current
symbol, so a partial record — one truncated by a stream gap — lands the *next*
one back on the symbol boundary instead of straddling it for the rest of the lock.
"""
function coherent_integration_blocks(link::HardwareCorrelatorLink, sat_state, signal_index)
    tracked_signal = Tracking.get_signals(sat_state)[signal_index]
    bit_buffer = Tracking.get_bit_buffer(tracked_signal)
    # Pre-sync the detectors need one prompt per code block, and Tracking
    # enforces it.
    has_bit_or_secondary_code_been_found(bit_buffer) || return 1
    signal = get_signal(tracked_signal)
    # A device replica reproduces the primary code only — nothing in
    # `assign_channel!` asks it to wipe off a secondary/overlay code — so
    # consecutive dumps of an overlaid signal carry alternating overlay chips.
    # Summing across them would cancel the signal rather than accumulate it, so
    # such signals stay at one block per record until the interface can tell a
    # device to apply the overlay itself.
    get_secondary_code_length(signal) == 1 || return 1
    blocks_per_symbol = _code_blocks_per_symbol(signal)
    blocks_per_symbol <= 1 && return 1
    requested =
        link.coherent_code_blocks == 0 ? blocks_per_symbol :
        min(link.coherent_code_blocks, blocks_per_symbol)
    # Land on the symbol boundary the bit buffer is counting toward, so a
    # truncated record is absorbed once instead of shifting every later one.
    remaining =
        blocks_per_symbol -
        mod(bit_buffer.prompt_accumulator_integrated_code_blocks, blocks_per_symbol)
    max(1, min(requested, remaining))
end

# Primary-code blocks in one symbol: a navigation bit where there is data, one
# secondary-code period for a pilot, and 1 when neither applies (a signal whose
# symbol *is* the code block, e.g. Galileo E1B).
function _code_blocks_per_symbol(signal)
    data_frequency = get_data_frequency(signal)
    iszero(data_frequency) && return get_secondary_code_length(signal)
    round(
        Int,
        upreferred(
            get_code_frequency(signal) / (get_code_length(signal) * data_frequency),
        ),
    )
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
function _accumulate_dump!(link, track_state, assignment, sat_state, hw_channel, output)
    target = coherent_integration_blocks(link, sat_state, assignment.signal_index)
    blocks = _record_code_blocks(link, sat_state, assignment.signal_index, output)
    if link.partial_blocks[hw_channel] == 0
        link.partial_correlator[hw_channel] = output.correlator
        link.partial_samples[hw_channel] = output.integrated_samples
        link.partial_blocks[hw_channel] = blocks
    else
        link.partial_correlator[hw_channel] = _add_accumulators(
            link.partial_correlator[hw_channel],
            output.correlator,
        )
        link.partial_samples[hw_channel] += output.integrated_samples
        link.partial_blocks[hw_channel] += blocks
    end
    link.partial_end[hw_channel] = output.sample_index
    link.partial_blocks[hw_channel] >= target &&
        _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
    link
end

# Whole primary-code blocks a record spans, recovered from its sample count the
# same way `Tracking` recovers it for the bit clock, so the two cannot disagree
# about how much signal time a record represents.
function _record_code_blocks(link, sat_state, signal_index, output)
    signal = get_signal(Tracking.get_signals(sat_state)[signal_index])
    max(
        1,
        round(
            Int,
            output.integrated_samples * ustrip(Hz, get_code_frequency(signal)) /
            (get_code_length(signal) * link.sampling_freq_hz),
        ),
    )
end

# Hand the accumulated record to the estimator and start a fresh one. A no-op
# when nothing is accumulated, so it is safe to call as a flush.
function _emit_partial!(link, track_state, assignment, sat_state, hw_channel)
    link.partial_blocks[hw_channel] == 0 && return link
    append_correlator_output!(
        track_state,
        _retag_spacing(
            get_correlator(sat_state, assignment.signal_index),
            Tracking.CorrelatorOutput(
                link.partial_correlator[hw_channel],
                link.partial_samples[hw_channel],
                link.partial_end[hw_channel],
            ),
            link.correlator_gain,
        ),
        assignment.group_key,
        assignment.prn,
        assignment.signal_index,
    )
    link.partial_samples[hw_channel] = 0
    link.partial_blocks[hw_channel] = 0
    link.partial_end[hw_channel] = typemin(Int64)
    link
end

# Throw away a partial record without emitting it. Used where the accumulation
# cannot be completed or attributed: a channel changing occupant.
function _discard_partial!(link, hw_channel)
    link.partial_samples[hw_channel] = 0
    link.partial_blocks[hw_channel] = 0
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
# dividing out `correlator_gain` — the amplitude of the replica the device wipes
# off with, where the host's own correlator uses unit amplitude. It is a pure
# scale factor, so every discriminator (a ratio) and the old moment-ratio C/N₀
# estimators are blind to it. `NoiseRefCN0Estimator` is not: it divides the
# prompt power by a noise density measured somewhere else, so a device reporting
# `g ×` the host's prompt reads `20·log10(g)` dB too high — 42 dB for a replica
# of amplitude 127. See `append_noise_observations!` for where the floor comes
# from.
# Antenna count off the correlator *type*, so the noise accumulator can be sized
# before any dump has arrived.
_num_ants(::Type{<:Tracking.AbstractCorrelator{M}}) where {M} = M

_retag_spacing(
    template::Tracking.AbstractCorrelator,
    output::Tracking.CorrelatorOutput,
    gain::Float64,
) = Tracking.CorrelatorOutput(
    @set(template.accumulators = get_accumulators(output.correlator) ./ gain),
    output.integrated_samples,
    output.sample_index,
)

"""
    push_nco_updates!(link, track_state, boundary) -> Int

Push one [`NCOUpdate`](@ref) per assigned hardware channel and return how many
were sent.

Called once per chunk, right after the estimator folded it, so the Dopplers read
back are the newest. All updates are scheduled at the same future sample —
`feedback_delay_epochs × Δ` past the *newest record the host has seen* — which is
what keeps the loop delay a known constant instead of PCIe jitter.

The reference is `latest_sample_index` rather than the epoch `boundary` because
the two part company exactly when it matters. `boundary` is where the fold grid
has got to; when the host is behind, that is in the past, and scheduling a
correction at a sample the device passed milliseconds ago asks it to apply the
update late by however far behind the host is — or, on a device that honours the
schedule strictly, to discard it. Anchoring to the newest record keeps the
correction a fixed, small distance in the *device's* future either way.

A full ring means the device's writer is not keeping up; the updates are
dropped rather than blocking the receiver, and counted in `link.dropped_dumps`'
sibling diagnostics.
"""
function push_nco_updates!(link::HardwareCorrelatorLink, track_state, boundary)
    channel = nco_update_channel(link.sdr)
    apply_at_sample =
        max(boundary, link.latest_sample_index) +
        link.feedback_delay_epochs * link.epoch_length
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
                apply_at_sample,
            ),
        )
    end
    isempty(link.nco_buffer) && return 0
    length(link.nco_buffer) <= n_avail_space(channel) || return 0
    put!(channel, link.nco_buffer)
    length(link.nco_buffer)
end

# Free slots in a `PipeChannel`. `n_avail` counts queued items, so the space left
# is the (usable) capacity minus that.
n_avail_space(channel::PipeChannel) = channel.capacity - 1 - Base.n_avail(channel)
