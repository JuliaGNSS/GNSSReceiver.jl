# ─────────────────────────────────────────────────────────────────────────────
# Host-side secondary-code (overlay) removal on the hardware path (issue #132).
#
# A hardware correlator replicates the primary code only, so every dump of an
# overlaid signal carries that primary-code block's overlay chip as a ±1 sign on
# every accumulator. `Tracking` assumes the opposite — once bit/secondary sync is
# found its software replica bakes the overlay into the code — so un-wiped
# prompts do not cancel the overlay, they cancel the *symbol*. The first testset
# is the issue's own measurement of that: ten GPS L5I dumps of a constant +1
# symbol sum to 2 rather than 10.
#
# The signals come from the reference harness (test/reference_harness.jl), so the
# dumps under test are correlations of the same synthetic stream every other
# per-signal check is measured against, and the wiped records are compared to the
# harness's own noise-free reference rather than to a hand-written number. The
# awkward cases below — sync arriving mid-chunk, a nonzero rotation, a per-PRN
# 1800-chip overlay, a lost record, reassignment, a data/pilot pair — are
# bookkeeping rather than signal fidelity, so they drive the ingest path with
# synthetic dumps directly, which is what pins the arithmetic.
#
# The last testset closes the loop: GPS L5I through the simulated device of
# test/simulated_fpga.jl, which really does correlate with a primary-only
# replica, with the overlay found by `Tracking`'s own detector rather than
# injected.
#
# `RecordingSDR`, `CapableSDR`, `SimulatedFPGA`, `epl`, `EPL` and `dump_at` come
# from test/hardware_correlator.jl and test/hardware_capabilities.jl;
# `ReferenceHarness` and `SignalSupport` from runtests.jl.
# ─────────────────────────────────────────────────────────────────────────────

using GNSSReceiver:
    HardwareChannelAssignment,
    advance_tracking!,
    anchor_secondary_phases!,
    coherent_integration_blocks,
    flush_partial_records!,
    is_secondary_code_removed,
    requested_secondary_code_mode

using GNSSSignals: get_secondary_code, secondary_value

using .ReferenceHarness
using .ReferenceHarness: data_bit
using .SignalSupport: record_evidence!

# A device that can replicate the overlaid signals these tests track, and — like
# every device the contract describes today — cannot wipe an overlay off itself.
const OVERLAY_CAPABILITIES = HardwareCorrelatorCapabilities(;
    signals = [:GPSL5I, :GPSL5Q, :GPSL1C_P, :GalileoE1C, :GPSL1CA],
    modulations = [:LOC, :BOCsin, :BOCcos, :CBOC, :TMBOC],
    max_primary_code_length = 10_230,
    code_frequency_limits = (1.023e6, 10.23e6),
    tap_layouts = [3, 5],
    max_tap_offset_chips = 1.0,
    bands = [:L1, :L5],
    num_rf_inputs = 2,
    max_secondary_code_length = 1,
)

# Two samples per chip, so `Tracking`'s ±0.5-chip early/late shift quantises onto
# the sample grid exactly and one primary-code block is a whole number of
# samples — neither is essential to the removal, and both keep the arithmetic
# below readable.
overlay_sampling_freq(signal) = 2 * uconvert(Hz, get_code_frequency(signal))

# Samples one primary-code block of `signal` spans at `sampling_freq`.
block_samples(signal, sampling_freq) = round(
    Int,
    get_code_length(signal) * ustrip(Hz, sampling_freq) /
    ustrip(Hz, get_code_frequency(signal)),
)

# The overlay chip of `signal`'s `index`-th primary-code block for `prn`.
overlay_chip(signal, prn, index) = secondary_value(get_secondary_code(signal), prn, index)

# The raw (un-normalized) prompt a device dumps for one block of a signal of unit
# normalized amplitude: `Tracking.normalize` divides an accumulator by its sample
# count and by the code amplitude, so this is what comes back as ±1.
raw_prompt(signal, block, amplitude = 1) = block * get_code_amplitude(signal) * amplitude

# The E/P/L correlator a device replicating the *primary* code only would dump
# for one block, built from the harness's `num_taps × 1` correlation of that
# block: over one primary period the two replicas differ by exactly the block's
# overlay chip, so the device's accumulators are the reference's times that sign.
device_epl(taps, chip) = epl(chip * taps[1, 1], chip * taps[2, 1], chip * taps[3, 1])

# One `CorrelatorDump` on `hw_channel` carrying `correlator`'s accumulators and
# ending at `sample_index`.
overlay_dump(hw_channel, prn, correlator, samples, sample_index) = CorrelatorDump(
    hw_channel,
    prn,
    Tracking.CorrelatorOutput(correlator, samples, sample_index),
    NaN,
)

# A link over `sdr` whose channels hold `prn`'s `signals`, one per component, with
# bit buffers reporting secondary sync at `secondary_phase` and `blocks` code
# blocks already accumulated toward the current symbol. Returns
# `(link, track_state)`.
function synced_overlay_link(
    sdr,
    signals::Tuple,
    prn;
    sampling_freq,
    secondary_phase = 0,
    blocks = 0,
    found = true,
    coherent_code_blocks = 1,
)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq,
        reference_signal = first(signals),
        coherent_code_blocks,
        noise_source = :samples,
    )
    group = GNSSReceiver.signal_group_key(first(signals))
    track_state = TrackState(; signals = NamedTuple{(group,)}((signals,)))
    track_state =
        add_satellite!(track_state; prn, group, code_phase = 0.0, carrier_doppler = 0.0Hz)
    for signal_index in eachindex(signals)
        assignment = HardwareChannelAssignment(group, prn, signal_index)
        link.assignments[signal_index] = assignment
        link.channel_of[assignment] = signal_index
        link.secondary_wipe[signal_index] =
            get_secondary_code_length(signals[signal_index]) > 1
    end
    sat = get_sat_state(track_state, prn)
    tracked = map(Tracking.get_signals(sat)) do tracked_signal
        buffer = Tracking.get_bit_buffer(tracked_signal)
        get_secondary_code_length(get_signal(tracked_signal)) == 1 &&
            return tracked_signal
        Tracking.TrackedSignal(
            tracked_signal;
            bit_buffer = typeof(buffer)(
                buffer.code_block_buffer,
                buffer.code_block_buffer_length,
                found,
                secondary_phase,
                Int8(1),
                complex(0.0, 0.0),
                blocks,
                buffer.soft_bits,
                buffer.phase_acc,
            ),
        )
    end
    Tracking.get_sat_states(track_state)[prn] = Tracking.TrackedSat(sat; signals = tracked)
    link, track_state
end

synced_overlay_link(sdr, signal::AbstractGNSSSignal, prn; kwargs...) =
    synced_overlay_link(sdr, (signal,), prn; kwargs...)

# Feed `n` consecutive primary-period dumps of a constant `amplitude` symbol,
# overlay chips and all, exactly as a device replicating the primary code would
# produce them.
function append_overlaid_dumps!(
    link,
    track_state,
    signal,
    prn,
    hw_channel,
    start_sample,
    n;
    sampling_freq,
    amplitude = 1,
    first_chip_index = 0,
    skip = Int[],
    # How a prompt becomes a correlator: three taps for a BPSK signal's
    # E/P/L bank, five for the VE/E/P/L/VL bank a BOC-family signal is tracked
    # with. Only the prompt carries the symbol here; the rest of the bank is
    # what the tap-layout check reads.
    correlator = prompt -> epl(0, prompt, 0),
)
    block = block_samples(signal, sampling_freq)
    for k = 0:(n-1)
        k in skip && continue
        chip = overlay_chip(signal, prn, first_chip_index + k)
        GNSSReceiver._append_dump!(
            link,
            track_state,
            overlay_dump(
                hw_channel,
                prn,
                correlator(raw_prompt(signal, block, amplitude) * chip),
                block,
                start_sample + block * (k + 1),
            ),
        )
    end
    link
end

# The soft bits the estimator has decoded for one component so far.
overlay_soft_bits(track_state, prn, signal_index = 1) = Tracking.get_soft_bits(
    Tracking.get_bit_buffer(
        Tracking.get_signals(get_sat_state(track_state, prn))[signal_index],
    ),
)

# Fold whatever has been appended through the estimator, in the order
# `fold_closed_epochs!` runs it.
function fold_overlay_records!(link, track_state, band_id, sampling_freq, block)
    flush_partial_records!(link, track_state)
    band_measurements = NamedTuple{(band_id,)}((
        Tracking.BandMeasurement(zeros(ComplexF64, block), sampling_freq, 0.0Hz),
    ))
    Tracking.estimate_dopplers_and_filter_prompt!(track_state, band_measurements)
    fill!(link.pending_blocks, 0)
    anchor_secondary_phases!(link, track_state)
    track_state
end

@testset "The GPS L5I reproducer: a constant symbol survives its overlay" begin
    # The issue's measurement, over the harness's own GPS L5I signal: a constant
    # +1 symbol across the ten primary-code blocks of one data bit. NH10 is
    # (1,1,1,1,-1,-1,1,-1,1,-1) and sums to 2, so without removal the decoded
    # soft bit is the overlay's own sum rather than the symbol's energy — 14 dB
    # thrown away, and a bit whose sign is NH10's rather than the satellite's.
    signal = GPSL5I()
    prn = 1
    sampling_freq = overlay_sampling_freq(signal)
    block = block_samples(signal, sampling_freq)
    @test get_secondary_code_length(signal) == 10
    @test sum(overlay_chip(signal, prn, k) for k = 0:9) == 2

    # Noise-free and at unit amplitude, so a normalized prompt is exactly 1 and
    # the decoded soft bit is exactly the number of blocks that summed.
    case = ReferenceCase(
        signal;
        prn,
        sampling_freq,
        amplitude = 1.0,
        noise_power = 0.0,
        code_phase = 0.0,
    )
    samples = generate_samples(case, 10block)
    # What a device replicating the primary code only dumps for block `k`: the
    # correlation an overlay-carrying replica would have produced, times that
    # block's overlay chip. Over one primary period the two replicas differ by
    # exactly that constant sign.
    device_dumps = map(0:9) do k
        taps = correlate(
            view(samples, (k*block+1):((k+1)*block), :),
            case,
            epl_taps();
            first_sample = k * block,
        )
        device_epl(taps, overlay_chip(signal, prn, k))
    end
    # The harness's own reference for one block, so the assertions below are
    # against the truth and not against the dumps' own arithmetic.
    reference_block = reference_correlation(case, epl_taps(), block)
    @test Tracking.get_prompt(first(device_dumps)) ≈ reference_block[2, 1]

    @testset "$label" for (label, (wipe, expected)) in (
        "the host removes the overlay" => (true, 10.0),
        "the overlay is left on" => (false, 2.0),
    )
        sdr = CapableSDR(EPL, 2, OVERLAY_CAPABILITIES)
        link, track_state = synced_overlay_link(sdr, signal, prn; sampling_freq)
        link.secondary_wipe[1] = wipe
        # The device's first record starts where the fold that found sync left off.
        link.last_record_end[1] = 0
        anchor_secondary_phases!(link, track_state)
        @test is_secondary_code_removed(link, 1) == wipe

        for (k, correlator) in enumerate(device_dumps)
            GNSSReceiver._append_dump!(
                link,
                track_state,
                overlay_dump(1, prn, correlator, block, k * block),
            )
        end
        fold_overlay_records!(link, track_state, :L5, sampling_freq, block)
        @test only(overlay_soft_bits(track_state, prn)) ≈ expected
    end
end

@testset "GPS L1 C/A is untouched by the removal path" begin
    # The regression baseline for the whole hardware chain has no secondary code,
    # so every part of this must be a no-op for it — including the record length,
    # which has to stay at the chunk-spanning behaviour issue #107 measured.
    signal = GPSL1CA()
    prn = 9
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(sdr; sampling_freq = 4e6Hz, reference_signal = signal)
    track_state = TrackState(signal, [TrackedSat(signal, prn, 0.0, 0.0Hz)])
    assignment = HardwareChannelAssignment(:default, prn, 1)
    link.assignments[1] = assignment
    link.channel_of[assignment] = 1

    # The link never arms the removal for a signal without an overlay.
    @test !link.secondary_wipe[1]
    @test !is_secondary_code_removed(link, 1)
    @test requested_secondary_code_mode(link, signal) === :primary_only

    # Pre-sync, four dumps reach the loops one per code block, with their
    # accumulators exactly as dumped.
    outputs = Tracking.get_correlator_outputs(get_sat_state(track_state, prn), 1)
    for k = 1:4
        GNSSReceiver._append_dump!(
            link,
            track_state,
            dump_at(1, prn, 4000k; prompt = 3 + 4im),
        )
    end
    flush_partial_records!(link, track_state)
    @test length(outputs) == 4
    @test all(o -> Tracking.get_prompt(o.correlator) ≈ 3 + 4im, outputs)
    @test all(o -> o.integrated_samples == 4000, outputs)
    @test link.secondary_phase_losses == 0
    empty!(outputs)
    fill!(link.pending_blocks, 0)

    # And a synced L1 C/A channel still folds a whole navigation bit's worth,
    # summing its dumps rather than being pinned to one block by an overlay it
    # does not have.
    sat = get_sat_state(track_state, prn)
    tracked = Tracking.get_signals(sat)[1]
    buffer = Tracking.get_bit_buffer(tracked)
    Tracking.get_sat_states(track_state)[prn] = Tracking.TrackedSat(
        sat;
        signals = (
            Tracking.TrackedSignal(
                tracked;
                bit_buffer = typeof(buffer)(
                    buffer.code_block_buffer,
                    buffer.code_block_buffer_length,
                    true,
                    0,
                    Int8(1),
                    complex(0.0, 0.0),
                    0,
                    buffer.soft_bits,
                    buffer.phase_acc,
                ),
            ),
        ),
    )
    @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) == 20
    for k = 5:8
        GNSSReceiver._append_dump!(
            link,
            track_state,
            dump_at(1, prn, 4000k; prompt = 3 + 4im),
        )
    end
    flush_partial_records!(link, track_state)
    record = only(outputs)
    @test Tracking.get_prompt(record.correlator) ≈ 12 + 16im
    @test record.integrated_samples == 16_000
    @test link.secondary_phase_losses == 0
end

@testset "Removal waits for sync and starts on the record after it" begin
    # Before sync the host cannot know which overlay chip a dump carries, and
    # `Tracking`'s own rotation search needs the prompts *un*-wiped to find it.
    # So the records already queued when sync lands — and the record that found
    # it — are folded as they arrived; the counter is seeded for the block that
    # follows the last of them.
    signal = GPSL5I()
    prn = 3
    sampling_freq = overlay_sampling_freq(signal)
    block = block_samples(signal, sampling_freq)
    sdr = CapableSDR(EPL, 2, OVERLAY_CAPABILITIES)
    link, track_state = synced_overlay_link(sdr, signal, prn; sampling_freq, found = false)

    @test !is_secondary_code_removed(link, 1)
    @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) == 1
    append_overlaid_dumps!(link, track_state, signal, prn, 1, 500_000, 3; sampling_freq)
    records = Tracking.get_correlator_outputs(get_sat_state(track_state, prn), 1)
    # Pre-sync the overlay is still on the prompts: the sync detector needs it.
    @test length(records) == 3
    @test Tracking.get_prompt(records[1].correlator) ≈
          raw_prompt(signal, block) * overlay_chip(signal, prn, 0)
    empty!(records)
    fill!(link.pending_blocks, 0)

    # Sync lands on this fold, reporting the chip of the block that follows the
    # three records just folded.
    sat = get_sat_state(track_state, prn)
    tracked = Tracking.get_signals(sat)[1]
    buffer = Tracking.get_bit_buffer(tracked)
    Tracking.get_sat_states(track_state)[prn] = Tracking.TrackedSat(
        sat;
        signals = (
            Tracking.TrackedSignal(
                tracked;
                bit_buffer = typeof(buffer)(
                    buffer.code_block_buffer,
                    buffer.code_block_buffer_length,
                    true,
                    3,
                    Int8(1),
                    complex(0.0, 0.0),
                    3,
                    buffer.soft_bits,
                    buffer.phase_acc,
                ),
            ),
        ),
    )
    anchor_secondary_phases!(link, track_state)
    @test is_secondary_code_removed(link, 1)
    @test link.secondary_phase[1] == 3
    @test link.secondary_phase_sample[1] == 500_000 + 3block

    # The remaining seven blocks of the symbol now arrive wiped, and complete the
    # bit the pre-sync blocks were already credited to.
    append_overlaid_dumps!(
        link,
        track_state,
        signal,
        prn,
        1,
        500_000 + 3block,
        7;
        sampling_freq,
        first_chip_index = 3,
    )
    fold_overlay_records!(link, track_state, :L5, sampling_freq, block)
    @test only(overlay_soft_bits(track_state, prn)) ≈ 7
end

@testset "A nonzero sync rotation wipes the right chips" begin
    # The rotation search locks at *any* overlay chip, so the seeded phase is
    # usually not 0. Every rotation must reconstruct the same constant symbol.
    signal = GPSL5I()
    prn = 5
    sampling_freq = overlay_sampling_freq(signal)
    block = block_samples(signal, sampling_freq)
    for rotation = 0:9
        sdr = CapableSDR(EPL, 2, OVERLAY_CAPABILITIES)
        link, track_state = synced_overlay_link(
            sdr,
            signal,
            prn;
            sampling_freq,
            secondary_phase = rotation,
            blocks = rotation,
        )
        link.last_record_end[1] = 800_000
        anchor_secondary_phases!(link, track_state)
        @test link.secondary_phase[1] == rotation
        append_overlaid_dumps!(
            link,
            track_state,
            signal,
            prn,
            1,
            800_000,
            10 - rotation;
            sampling_freq,
            first_chip_index = rotation,
            amplitude = -1,
        )
        fold_overlay_records!(link, track_state, :L5, sampling_freq, block)
        # The first emitted bit completes on the data-bit boundary, i.e.
        # `10 - rotation` blocks after the lock (issue #125's seeding).
        @test only(overlay_soft_bits(track_state, prn)) ≈ -(10 - rotation)
    end
end

@testset "A per-PRN 1800-chip overlay is removed from its own table" begin
    # GPS L1C-P's overlay is a per-PRN 1800-chip code — 18 s of it — so the chip a
    # block carries cannot be read off a shared table. The removal reads
    # `GNSSSignals.secondary_value`, the same lookup the software replica is built
    # from, so a per-PRN table and a shared one take the same path.
    signal = GPSL1C_P()
    sampling_freq = overlay_sampling_freq(signal)
    block = block_samples(signal, sampling_freq)
    @test get_secondary_code_length(signal) == 1800
    @test get_secondary_code(signal) isa GNSSSignals.PerPRNSecondaryCode
    # Two PRNs whose overlays differ over the chips under test, so a removal that
    # ignored the PRN would show up here.
    prns = (1, 2)
    @test any(
        overlay_chip(signal, prns[1], k) != overlay_chip(signal, prns[2], k) for
        k = 1790:1809
    )
    for prn in prns
        # A TMBOC pilot is tracked with the five-tap VE/E/P/L/VL bank, so the
        # device's records have to be five slots wide too.
        sdr = CapableSDR(VEPL, 2, OVERLAY_CAPABILITIES)
        # A rotation near the end of the 1800-chip period, so the counter also has
        # to wrap correctly.
        link, track_state = synced_overlay_link(
            sdr,
            signal,
            prn;
            sampling_freq,
            secondary_phase = 1790,
            coherent_code_blocks = 20,
        )
        link.last_record_end[1] = 4_000_000
        anchor_secondary_phases!(link, track_state)
        append_overlaid_dumps!(
            link,
            track_state,
            signal,
            prn,
            1,
            4_000_000,
            20;
            sampling_freq,
            first_chip_index = 1790,
            correlator = prompt -> vepl(0, 0, prompt, 0, 0),
        )
        flush_partial_records!(link, track_state)
        # A dataless pilot with its overlay off is coherent over any length:
        # twenty wiped blocks sum, they do not cancel.
        record = only(Tracking.get_correlator_outputs(get_sat_state(track_state, prn), 1))
        @test record.integrated_samples == 20block
        @test Tracking.get_prompt(record.correlator) ≈ 20 * raw_prompt(signal, block)
        # The counter wrapped through the end of the period.
        @test link.secondary_phase[1] == mod(1790 + 20, 1800)
    end
end

@testset "A hole in the record stream stops the removal instead of guessing" begin
    # The overlay counter rides a record stream that tiles the sample axis. A
    # record that never reached the host moves every later block's chip, and a
    # counter that kept counting would then wipe with the wrong sign for the rest
    # of the run — worse than not wiping at all, because nothing downstream can
    # see a sign error. So the phase is dropped, the partial record is cut where
    # the removal stopped, and only a fresh sync re-seeds it.
    signal = GPSL5I()
    prn = 7
    sampling_freq = overlay_sampling_freq(signal)
    block = block_samples(signal, sampling_freq)
    sdr = CapableSDR(EPL, 2, OVERLAY_CAPABILITIES)
    link, track_state =
        synced_overlay_link(sdr, signal, prn; sampling_freq, coherent_code_blocks = 10)
    link.last_record_end[1] = 2_000_000
    anchor_secondary_phases!(link, track_state)

    # Blocks 0 and 1 arrive, block 2 never does, blocks 3 and 4 arrive.
    append_overlaid_dumps!(
        link,
        track_state,
        signal,
        prn,
        1,
        2_000_000,
        5;
        sampling_freq,
        skip = [2],
    )
    @test link.secondary_phase_losses == 1
    @test !is_secondary_code_removed(link, 1)
    @test link.lost_record_gaps == 1
    # The two wiped blocks were cut into their own record rather than summed with
    # the un-wiped ones that followed.
    records = Tracking.get_correlator_outputs(get_sat_state(track_state, prn), 1)
    @test Tracking.get_prompt(first(records).correlator) ≈ 2 * raw_prompt(signal, block)
    @test first(records).integrated_samples == 2block

    # A restarted bit clock leaves the phase unknown, and no later fold may
    # resurrect it from the frozen zero in the fresh bit buffer.
    GNSSReceiver.restart_lost_bit_clocks!(link, track_state)
    anchor_secondary_phases!(link, track_state)
    @test !is_secondary_code_removed(link, 1)
    @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) == 1
end

@testset "Longer coherent integration only across wiped records" begin
    # Summing dumps is what makes the loop's Δt the interval the device really
    # holds a correction for (issue #107), and an overlaid signal was pinned to
    # one block per record because summing across overlay chips cancels the
    # signal. With the overlay off that restriction lifts — but only then, and
    # only up to the symbol the data flips on.
    signal = GPSL5I()
    prn = 11
    sampling_freq = overlay_sampling_freq(signal)
    block = block_samples(signal, sampling_freq)
    case = ReferenceCase(
        signal;
        prn,
        sampling_freq,
        amplitude = 1.0,
        noise_power = 0.0,
        code_phase = 0.0,
    )
    samples = generate_samples(case, 10block)
    sdr = CapableSDR(EPL, 2, OVERLAY_CAPABILITIES)
    link, track_state =
        synced_overlay_link(sdr, signal, prn; sampling_freq, coherent_code_blocks = nothing)

    # Synced, but the phase has not been seeded yet: still one block per record.
    @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) == 1
    link.last_record_end[1] = 0
    anchor_secondary_phases!(link, track_state)
    @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) == 10

    for k = 0:9
        taps = correlate(
            view(samples, (k*block+1):((k+1)*block), :),
            case,
            epl_taps();
            first_sample = k * block,
        )
        GNSSReceiver._append_dump!(
            link,
            track_state,
            overlay_dump(
                1,
                prn,
                device_epl(taps, overlay_chip(signal, prn, k)),
                block,
                (k + 1) * block,
            ),
        )
    end
    # Ten wiped blocks become one 10 ms record — the coherent sum the overlay was
    # hiding — and it is the one an ideal correlator carrying the overlay in its
    # replica would have accumulated over the same window.
    record = only(Tracking.get_correlator_outputs(get_sat_state(track_state, prn), 1))
    @test record.integrated_samples == 10block
    comparison = compare_correlations(
        "ten wiped GPS L5I blocks",
        reshape(collect(get_accumulators(record.correlator)), 3, 1),
        reference_correlation(case, epl_taps(), 10block),
        software_tolerances(),
    )
    passed(comparison) || show(stdout, MIME"text/plain"(), comparison)
    @test passed(comparison)
    # …and it stops at the symbol: the eleventh block would start a new record,
    # because past the data-bit boundary the symbol can flip sign.
    @test link.partial_blocks[1] == 0
end

@testset "Reassignment and release leave no stale overlay phase" begin
    # A channel that changes occupant must not wipe the newcomer's dumps with the
    # previous satellite's overlay phase.
    signal = GPSL5I()
    prn = 13
    sampling_freq = overlay_sampling_freq(signal)
    sdr = CapableSDR(EPL, 2, OVERLAY_CAPABILITIES)
    link, track_state = synced_overlay_link(sdr, signal, prn; sampling_freq)
    link.last_record_end[1] = 5_000_000
    anchor_secondary_phases!(link, track_state)
    @test is_secondary_code_removed(link, 1)

    group = GNSSReceiver.signal_group_key(signal)
    empty_state = TrackState(; signals = NamedTuple{(group,)}(((signal,),)))
    GNSSReceiver.release_stale_channels!(link, empty_state)
    @test !link.secondary_wipe[1]
    @test !is_secondary_code_removed(link, 1)
    @test link.secondary_phase_sample[1] == typemin(Int64)

    # The fresh occupant is armed for removal by the assignment itself, with no
    # phase until its own sync is found.
    band_systems = ((signal,),)
    band_measurements =
        (; L5 = Tracking.BandMeasurement(zeros(ComplexF64, 2000), sampling_freq, 0.0Hz))
    fresh = add_satellite!(
        empty_state;
        prn = prn + 1,
        group,
        code_phase = 0.0,
        carrier_doppler = 0.0Hz,
    )
    GNSSReceiver.sync_hardware_channels!(link, fresh, band_systems, band_measurements)
    hw_channel = link.channel_of[HardwareChannelAssignment(group, prn + 1, 1)]
    @test link.secondary_wipe[hw_channel]
    @test !is_secondary_code_removed(link, hw_channel)
    @test link.secondary_phase[hw_channel] == -1
    # The device is still asked to replicate the primary code only: the host owns
    # the removal, and asking for both would remove the overlay twice.
    @test only(sdr.assigned).secondary_code_mode === :primary_only
end

@testset "Data and pilot components keep their own overlays and phase relation" begin
    # A pilot/data pair occupies two channels whose overlays differ, and the ICD
    # carrier-phase relationship between them is what lets the host lock one on
    # the real axis and de-rotate the other onto it. Removing each component's own
    # overlay restores that relationship rather than disturbing it: the removal is
    # a ±1 the transmitter applied, taken off again.
    data, pilot = GPSL5I(), GPSL5Q()
    prn = 17
    sampling_freq = overlay_sampling_freq(data)
    block = block_samples(data, sampling_freq)
    @test get_secondary_code_length(data) == 10
    @test get_secondary_code_length(pilot) == 20
    sdr = CapableSDR(EPL, 4, OVERLAY_CAPABILITIES)
    link, track_state = synced_overlay_link(
        sdr,
        (data, pilot),
        prn;
        sampling_freq,
        coherent_code_blocks = 10,
    )
    link.last_record_end[1] = 6_000_000
    link.last_record_end[2] = 6_000_000
    anchor_secondary_phases!(link, track_state)
    @test is_secondary_code_removed(link, 1)
    @test is_secondary_code_removed(link, 2)

    # The pair is transmitted in quadrature: the data component on the band's
    # in-phase reference, the pilot a quarter cycle from it. Both dumps carry
    # their own overlay chip on top of that.
    @test !(get_carrier_phase_offset(pilot) ≈ get_carrier_phase_offset(data))
    for (hw_channel, signal) in ((1, data), (2, pilot))
        append_overlaid_dumps!(
            link,
            track_state,
            signal,
            prn,
            hw_channel,
            6_000_000,
            10;
            sampling_freq,
            amplitude = cis(get_carrier_phase_offset(signal)),
        )
    end
    flush_partial_records!(link, track_state)
    outputs(index) = Tracking.get_correlator_outputs(get_sat_state(track_state, prn), index)
    data_record = only(outputs(1))
    pilot_record = only(outputs(2))
    # Each component's overlay is gone, and the phase between them survived it.
    @test Tracking.get_prompt(data_record.correlator) ≈
          10 * raw_prompt(data, block) * cis(get_carrier_phase_offset(data))
    @test Tracking.get_prompt(pilot_record.correlator) ≈
          10 * raw_prompt(pilot, block) * cis(get_carrier_phase_offset(pilot))
    @test angle(
        Tracking.get_prompt(pilot_record.correlator) /
        Tracking.get_prompt(data_record.correlator),
    ) ≈ get_carrier_phase_offset(pilot) - get_carrier_phase_offset(data)
end

@testset "A multi-block dump cannot be wiped and is not guessed at" begin
    # The device's contract is one record per primary-code period, which is what
    # makes "one overlay chip per record" true. A record covering more than one
    # block has already summed its blocks' chips inside the accumulator, so there
    # is no single sign left to take off — the removal stops rather than
    # inventing one.
    signal = GPSL5I()
    prn = 19
    sampling_freq = overlay_sampling_freq(signal)
    block = block_samples(signal, sampling_freq)
    sdr = CapableSDR(EPL, 2, OVERLAY_CAPABILITIES)
    link, track_state = synced_overlay_link(sdr, signal, prn; sampling_freq)
    link.last_record_end[1] = 7_000_000
    anchor_secondary_phases!(link, track_state)

    GNSSReceiver._append_dump!(
        link,
        track_state,
        overlay_dump(
            1,
            prn,
            epl(0, raw_prompt(signal, 3block), 0),
            3block,
            7_000_000 + 3block,
        ),
    )
    @test !is_secondary_code_removed(link, 1)
    @test link.secondary_phase_losses == 1
end

@testset "Simulated device: GPS L5I synchronises, wipes and decodes its symbols" begin
    # The whole path, end to end, with nothing injected: the simulated correlator
    # of test/simulated_fpga.jl really does replicate the primary code only, so
    # its dumps carry NH10 the way a gateware's do; `Tracking`'s own detector
    # finds the overlay; and the link then takes it off every record. What is
    # asserted is the thing the receiver is for — that the decoded symbols are the
    # ones the harness transmitted.
    signal = GPSL5I()
    prn = 6
    sampling_freq = overlay_sampling_freq(signal)
    block = block_samples(signal, sampling_freq)
    blocks_per_symbol = get_secondary_code_length(signal)
    num_blocks = 160
    case = ReferenceCase(
        signal;
        prn,
        sampling_freq,
        cn0_dbhz = 48.0,
        code_phase = 0.0,
        data_bits = true,
    )
    source = SampleSource(case)
    sdr = SimulatedFPGA(
        signal;
        sampling_freq,
        chunk = block,
        n_channels = 3,
        epoch_length = block,
    )
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq,
        reference_signal = signal,
        # The documented FPGA recipe: a hardware channel spent on an open-loop
        # despread of a decoy PRN, which the simulated device serves the same way
        # it serves a satellite.
        noise_source = :channel,
    )
    group = GNSSReceiver.signal_group_key(signal)
    track_state = TrackState(; signals = NamedTuple{(group,)}(((signal,),)))
    track_state = add_satellite!(
        track_state;
        prn,
        group,
        code_phase = case.code_phase,
        carrier_doppler = 0.0Hz,
    )
    band_systems = ((signal,),)

    # (symbol index, soft bit), collected per chunk because `advance_tracking!`
    # resets the bit store at the top of every chunk the way `track!` does.
    decoded = Tuple{Int,Float32}[]
    synced_at = 0
    assignment = HardwareChannelAssignment(group, prn, 1)
    # Which hardware channel the satellite ends up on is the link's business —
    # the open-loop noise reference takes one of its own.
    satellite_channel() = get(link.channel_of, assignment, 0)
    for index = 1:num_blocks
        samples = next_samples!(source, block)
        correlate_chunk!(sdr, view(samples, :, 1))
        band_measurements = (; L5 = Tracking.BandMeasurement(samples, sampling_freq, 0.0Hz))
        track_state = advance_tracking!(link, band_measurements, track_state, band_systems)
        if iszero(synced_at) &&
           satellite_channel() > 0 &&
           is_secondary_code_removed(link, satellite_channel())
            synced_at = index
        end
        for bit in overlay_soft_bits(track_state, prn)
            # One chunk is one primary-code block and a symbol is ten of them, so
            # a bit completing in this chunk is the symbol that ended with it.
            push!(decoded, (div(index, blocks_per_symbol) - 1, bit))
        end
    end

    # The overlay was found on the hardware path, and the link took it from there.
    hw_channel = satellite_channel()
    @test hw_channel > 0
    @test synced_at > 0
    removing = is_secondary_code_removed(link, hw_channel)
    @test removing
    @test link.secondary_phase_losses == 0
    # Whole symbols only reach the decoder once the overlay is off, so there are
    # as many of them as the run had room for.
    enough_symbols = length(decoded) >= 8
    @test enough_symbols
    # A negative-polarity lock inverts every bit consistently; the decoder
    # resolves that from the preamble, so the comparison is up to one sign.
    polarity = sign(first(decoded)[2] * data_bit(case, first(decoded)[1]))
    symbols_match =
        all(sign(bit) == polarity * data_bit(case, symbol) for (symbol, bit) in decoded)
    @test symbols_match
    # And each one carries close to the full ten blocks of energy rather than the
    # overlay's own sum of two.
    full_energy =
        minimum(abs(bit) for (_, bit) in decoded) >
        0.6 * blocks_per_symbol * signal_amplitude(case)
    @test full_energy

    # The matrix's claim must not outlive the check that backs it, so the
    # evidence is recorded only when every assertion above held — the same rule
    # the per-signal checks in test/signal_validation.jl follow.
    hw_channel > 0 &&
        synced_at > 0 &&
        removing &&
        iszero(link.secondary_phase_losses) &&
        enough_symbols &&
        symbols_match &&
        full_energy &&
        record_evidence!(:GPSL5I, :secondary_sync, :harness_hardware_overlay)
end
