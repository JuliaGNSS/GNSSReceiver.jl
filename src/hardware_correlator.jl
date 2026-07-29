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
end

CorrelatorDump(channel::Integer, prn::Integer, output::Tracking.CorrelatorOutput) =
    CorrelatorDump(Int32(channel), Int32(prn), output)

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
    const feedback_delay_epochs::Int
    const max_dumps_per_drain::Int
    # How many epochs the fold loop will replay in one chunk before treating the
    # shortfall as a gap and resynchronising the grid (see `fold_closed_epochs!`).
    const max_catchup_epochs::Int
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
end

function HardwareCorrelatorLink(
    sdr::AbstractHardwareCorrelatorSDR;
    sampling_freq,
    reference_signal,
    doppler_update_interval = nothing,
    feedback_delay_epochs::Integer = 2,
    max_dumps_per_drain::Integer = 4096,
    max_catchup_epochs::Integer = 64,
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
        Int(feedback_delay_epochs),
        Int(max_dumps_per_drain),
        Int(max_catchup_epochs),
        typemin(Int),
        typemin(Int),
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
    # `track!` resets the per-signal bit buffers at the start of every call and
    # this method replaces `track!`, so it inherits that duty. The pipeline read
    # last chunk's bits out right after the previous `advance_tracking!`; without
    # the reset the hard-bit buffer fills 128 bits after bit sync (2.56 s for GPS
    # L1 C/A) and throws from inside the estimator — measured on hardware, not
    # hypothetical. The reset keeps the bit-edge sync and the partial-bit
    # accumulator; only the already-consumed bits are dropped.
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
    fold_closed_epochs!(link, track_state, band_measurements)

    track_state
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
    assign_new_channels!(link, track_state, band_systems, band_measurements)
    link
end

function release_stale_channels!(link, track_state)
    for hw_channel in eachindex(link.assignments)
        assignment = link.assignments[hw_channel]
        isnothing(assignment) && continue
        _is_tracked(track_state, assignment) && continue
        release_channel!(link.sdr, hw_channel)
        link.assignments[hw_channel] = nothing
        delete!(link.channel_of, assignment)
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
function fold_closed_epochs!(link::HardwareCorrelatorLink, track_state, band_measurements)
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
    while link.latest_sample_index >= link.next_epoch_boundary
        boundary = link.next_epoch_boundary
        append_epoch_outputs!(link, track_state, boundary)
        Tracking.estimate_dopplers_and_filter_prompt!(track_state, band_measurements)
        push_nco_updates!(link, track_state, boundary)
        link.next_epoch_boundary = boundary + link.epoch_length
        folds += 1
    end
    folds
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

function _append_dump!(link, track_state, dump)
    hw_channel = Int(dump.channel)
    checkbounds(Bool, link.assignments, hw_channel) || (link.stale_dumps += 1; return link)
    assignment = link.assignments[hw_channel]
    # A dump whose channel is free, or whose PRN no longer matches the channel's
    # occupant, was produced before a reassignment took effect. Folding it into
    # whoever holds the channel now would corrupt that satellite's loop.
    if isnothing(assignment) || assignment.prn != Int(dump.prn)
        link.stale_dumps += 1
        return link
    end
    sat_state = get_sat_state(track_state, assignment.group_key, assignment.prn)
    append_correlator_output!(
        track_state,
        _retag_spacing(get_correlator(sat_state, assignment.signal_index), dump.output),
        assignment.group_key,
        assignment.prn,
        assignment.signal_index,
    )
    link
end

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
_retag_spacing(template::Tracking.AbstractCorrelator, output::Tracking.CorrelatorOutput) =
    Tracking.CorrelatorOutput(
        @set(template.accumulators = get_accumulators(output.correlator)),
        output.integrated_samples,
        output.sample_index,
    )

"""
    push_nco_updates!(link, track_state, boundary) -> Int

Push one [`NCOUpdate`](@ref) per assigned hardware channel and return how many
were sent.

Called right after the estimator folded an epoch, so the Dopplers read back are
this epoch's. All updates are scheduled at the same future sample —
`boundary + feedback_delay_epochs × Δ` — which is what keeps the loop delay a
known constant instead of PCIe jitter.

A full ring means the device's writer is not keeping up; the updates are
dropped rather than blocking the receiver, and counted in `link.dropped_dumps`'
sibling diagnostics.
"""
function push_nco_updates!(link::HardwareCorrelatorLink, track_state, boundary)
    channel = nco_update_channel(link.sdr)
    apply_at_sample = boundary + link.feedback_delay_epochs * link.epoch_length
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
