# ─────────────────────────────────────────────────────────────────────────────
# Long codes and dumps shorter than a primary code period (issue #133).
#
# Four different things used to be one number. A hardware channel has a
# *tracking-update cadence* (how often the loops are handed a record and the NCO
# a correction), a *dump integration duration* (how long the device accumulated
# before it streamed a record), a count of *primary-code wraps* (how often the
# replica ran through its code), and a *navigation/secondary-code block count*
# (what the bit clock and the overlay counter ride on). The ingest path measured
# all four with one expression — `max(1, round(samples · f_code / (L · fs)))` —
# which rounds a record to whole primary-code blocks and never reports fewer than
# one. A record covering a quarter of a code period therefore *was* a code
# period, as far as the bit clock, the overlay counter and the coherent
# accumulation were concerned.
#
# That is unworkable at both ends of the signal scope. GPS L2CL's primary code is
# 767250 chips at 511.5 kcps — one period is 1.5 s, so a receiver that only ever
# closes its loop on a code wrap updates its NCO twice per three seconds; and a
# receiver that dumps faster has every one of those short records counted as a
# completed 1.5 s period. Galileo E5a-QP is the opposite extreme: a 330-chip
# primary code is 64.5 µs, and folding one record per period hands the loops
# fifteen thousand records a second.
#
# These tests pin the separation. The accounting is exercised with hand-built
# dumps, because it is arithmetic and the arithmetic is the point; the last
# testsets run the simulated correlator of test/simulated_fpga.jl over
# reference-harness samples, so a partial-dump stream really is produced by a
# device and really is measured against the harness's own noise-free reference.
#
# `RecordingSDR`, `CapableSDR`, `SimulatedFPGA`, `epl`, `EPL` and `dump_at` come
# from test/hardware_correlator.jl and test/hardware_capabilities.jl;
# `ReferenceHarness` from runtests.jl.
# ─────────────────────────────────────────────────────────────────────────────

using GNSSReceiver:
    HardwareChannelAssignment,
    allows_partial_primary_records,
    anchor_secondary_phases!,
    coherent_integration_blocks,
    coherent_integration_periods,
    flush_partial_records!,
    is_secondary_code_removed,
    primary_code_block_phase,
    primary_code_wraps,
    record_integration_periods

using GNSSSignals: get_secondary_code, secondary_value

using .ReferenceHarness

# A device with no signal restrictions at all, so a test can hand it whatever
# signal it is about. `supports_partial_code_dumps` is what a gateware declares
# once it can be told to dump inside a code period (step 2/3 of the roadmap);
# everything about the host-side accounting is testable without it, and the
# pre-arm gate below is what the declaration actually gates.
partial_capabilities(; partial_dumps::Bool = true, max_code_length = 1_000_000) =
    HardwareCorrelatorCapabilities(;
        max_primary_code_length = max_code_length,
        code_frequency_limits = (100e3, 20e6),
        tap_layouts = [3, 5],
        max_tap_offset_chips = 1.0,
        bands = nothing,
        num_rf_inputs = 4,
        reports_code_phase = true,
        supports_partial_code_dumps = partial_dumps,
    )

# GPS L2CL's 767250-chip code table costs half a gigabyte to build, so the one
# instance every L2CL testset below shares is built once here.
const L2CL = GPSL2CL()

# Samples one primary-code block of `signal` spans at `sampling_freq`.
primary_block_samples(signal, sampling_freq) = round(
    Int,
    get_code_length(signal) * ustrip(Hz, sampling_freq) /
    ustrip(Hz, get_code_frequency(signal)),
)

# A link over a fully capable device with `prn` occupying hardware channel 1, and
# a track state to fold its records into. `synced` puts the signal's bit buffer
# past bit/secondary sync with `blocks` blocks already counted toward the current
# symbol, which is what the post-sync sizing rules are measured against.
function partial_link(
    signal,
    prn;
    sampling_freq,
    synced::Bool = false,
    secondary_phase::Integer = 0,
    blocks::Integer = 0,
    capabilities = partial_capabilities(),
    kwargs...,
)
    sdr = CapableSDR(EPL, 4, capabilities)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq,
        reference_signal = signal,
        noise_source = :samples,
        kwargs...,
    )
    group = GNSSReceiver.signal_group_key(signal)
    track_state = TrackState(; signals = NamedTuple{(group,)}(((signal,),)))
    track_state =
        add_satellite!(track_state; prn, group, code_phase = 0.0, carrier_doppler = 0.0Hz)
    assignment = HardwareChannelAssignment(group, prn, 1)
    link.assignments[1] = assignment
    link.channel_of[assignment] = 1
    link.secondary_wipe[1] = get_secondary_code_length(signal) > 1
    if synced
        sat = get_sat_state(track_state, prn)
        tracked = only(Tracking.get_signals(sat))
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
                        secondary_phase,
                        Int8(1),
                        complex(0.0, 0.0),
                        blocks,
                        buffer.soft_bits,
                        buffer.phase_acc,
                    ),
                ),
            ),
        )
    end
    link, track_state
end

# One record on channel `hw_channel`, `samples` long, ending at `sample_index`,
# optionally carrying the device's absolute replica code phase.
partial_dump(hw_channel, prn, samples, sample_index; prompt = 1.0 + 0im, code_phase = NaN) =
    CorrelatorDump(
        hw_channel,
        prn,
        Tracking.CorrelatorOutput(epl(0, prompt, 0), samples, sample_index),
        code_phase,
    )

# Feed `n` consecutive records of `samples` samples each, tiling the sample axis
# from `start_sample`, reporting the replica code phase they end at unless
# `report_code_phase = false`. `skip` names records (0-based) the host never
# sees, which is how a lost record is described.
function append_tiling_dumps!(
    link,
    track_state,
    signal,
    prn,
    hw_channel,
    start_sample,
    n,
    samples;
    sampling_freq,
    prompt = k -> 1.0 + 0im,
    skip = Int[],
    report_code_phase::Bool = true,
    code_phase_at_zero::Real = 0.0,
)
    code_length = get_code_length(signal)
    chips_per_sample =
        ustrip(Hz, get_code_frequency(signal)) / ustrip(Hz, uconvert(Hz, sampling_freq))
    for k = 0:(n-1)
        k in skip && continue
        ending = start_sample + samples * (k + 1)
        # The replica's absolute phase at the record's end, as a device latches
        # it: a replica that stood at `code_phase_at_zero` at device sample 0.
        phase = mod(code_phase_at_zero + ending * chips_per_sample, code_length)
        GNSSReceiver._append_dump!(
            link,
            track_state,
            partial_dump(
                hw_channel,
                prn,
                samples,
                ending;
                prompt = prompt(k),
                code_phase = report_code_phase ? phase : NaN,
            ),
        )
    end
    link
end

records(track_state, prn, signal_index = 1) =
    Tracking.get_correlator_outputs(get_sat_state(track_state, prn), signal_index)

# ─────────────────────────────────────────────────────────────────────────────

@testset "A dump shorter than a primary code period is not a primary code period" begin
    # The defect in one measurement. Four records tile one GPS L1 C/A code block
    # exactly; between them they complete one code wrap and carry one code
    # block's worth of signal. Rounding each to "at least one block" made them
    # four wraps, four blocks and four separate records handed to the loops.
    signal = GPSL1CA()
    prn = 1
    sampling_freq = 4 * uconvert(Hz, get_code_frequency(signal))
    block = primary_block_samples(signal, sampling_freq)   # 4092 samples
    quarter = block ÷ 4
    link, track_state = partial_link(signal, prn; sampling_freq)

    append_tiling_dumps!(link, track_state, signal, prn, 1, 0, 3, quarter; sampling_freq)

    # Three of the four records sit inside the block, so no code wrap has
    # completed — and nothing has reached the loops, because a record shorter
    # than a code block cannot be handed to `Tracking`, whose pre-sync bit search
    # takes exactly one prompt per block.
    @test primary_code_wraps(link, 1) == 0
    @test isempty(records(track_state, prn))
    # A flush at a chunk boundary does not force one out either: the part-record
    # waits for the dump that completes its block.
    flush_partial_records!(link, track_state)
    @test isempty(records(track_state, prn))

    append_tiling_dumps!(
        link,
        track_state,
        signal,
        prn,
        1,
        3 * quarter,
        1,
        quarter;
        sampling_freq,
    )
    @test primary_code_wraps(link, 1) == 1
    record = only(records(track_state, prn))
    @test record.integrated_samples == block
    @test record.sample_index == block
    @test Tracking.get_prompt(record.correlator) ≈ 4 + 0im
    # And exactly one block is credited against the navigation bit grid.
    @test link.pending_blocks[1] == 1
end

@testset "Partial records land on the primary-code block grid" begin
    # Quarter-block records, one code wrap at a time: the wrap count has to track
    # the code and not the record stream, and the block phase has to say where
    # inside a block the next record will start.
    signal = GPSL1CA()
    prn = 4
    sampling_freq = 4 * uconvert(Hz, get_code_frequency(signal))
    block = primary_block_samples(signal, sampling_freq)
    quarter = block ÷ 4
    link, track_state = partial_link(signal, prn; sampling_freq)

    for k = 1:11
        append_tiling_dumps!(
            link,
            track_state,
            signal,
            prn,
            1,
            quarter * (k - 1),
            1,
            quarter;
            sampling_freq,
        )
        @test primary_code_wraps(link, 1) == k ÷ 4
        @test primary_code_block_phase(link, 1) ≈ mod(k, 4) / 4 atol = 1e-9
        @test record_integration_periods(link, 1) ≈ mod(k, 4) / 4 atol = 1e-9
    end
    # Two whole blocks emitted, the third still accumulating.
    @test length(records(track_state, prn)) == 2
    @test all(o -> o.integrated_samples == block, records(track_state, prn))
    @test link.pending_blocks[1] == 2
end

@testset "Partial records keep the navigation bit grid" begin
    # Post-sync a record may span a whole navigation bit, and the bit grid is
    # what makes the decoded stream mean anything. Eighty quarter-block records
    # are twenty code blocks: one bit, one record, and the bit clock credited
    # exactly once.
    signal = GPSL1CA()
    prn = 6
    sampling_freq = 4 * uconvert(Hz, get_code_frequency(signal))
    block = primary_block_samples(signal, sampling_freq)
    quarter = block ÷ 4
    link, track_state = partial_link(signal, prn; sampling_freq, synced = true)

    @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) == 20
    @test coherent_integration_periods(link, get_sat_state(track_state, prn), 1, 1) ≈ 20.0

    append_tiling_dumps!(
        link,
        track_state,
        signal,
        prn,
        1,
        0,
        4 * 20,
        quarter;
        sampling_freq,
    )
    record = only(records(track_state, prn))
    @test record.integrated_samples == 20 * block
    @test Tracking.get_prompt(record.correlator) ≈ 80 + 0im
    @test link.pending_blocks[1] == 20
    @test primary_code_wraps(link, 1) == 20
    # The next bit starts on the grid, not a quarter block into it.
    @test primary_code_block_phase(link, 1) == 0.0
end

@testset "A hole in a partial-dump stream is a lost record, not a re-arm hole" begin
    # A re-arm hole is shorter than one of the channel's records and is followed
    # by a short one; a lost record is a whole record missing. With a record per
    # code period the two were told apart by comparing the hole to one *code
    # period*, which classifies every hole in a partial-dump stream as a re-arm —
    # and a re-arm hole leaves the bit clock alone, so a genuine loss would have
    # slid the bit grid permanently with nothing said about it.
    signal = GPSL1CA()
    prn = 8
    sampling_freq = 4 * uconvert(Hz, get_code_frequency(signal))
    block = primary_block_samples(signal, sampling_freq)
    quarter = block ÷ 4
    link, track_state = partial_link(signal, prn; sampling_freq, synced = true)

    @test_logs (:warn, r"records lost") append_tiling_dumps!(
        link,
        track_state,
        signal,
        prn,
        1,
        0,
        8,
        quarter;
        sampling_freq,
        skip = [4],
    )
    @test link.lost_record_gaps == 1
    @test link.lost_record_samples[1] == quarter
    @test link.rearm_gaps == 0
    @test link.bit_clock_lost[1]

    # A hole shorter than one record — the remainder a re-arm leaves — still
    # reads as a re-arm, and still leaves the bit clock alone.
    link.bit_clock_lost[1] = false
    GNSSReceiver._append_dump!(
        link,
        track_state,
        partial_dump(1, prn, quarter ÷ 2, 8 * quarter + quarter ÷ 2 + 100),
    )
    @test link.rearm_gaps == 1
    @test link.lost_record_gaps == 1
    @test !link.bit_clock_lost[1]
end

@testset "A secondary code is removed per code block, not per record" begin
    # The overlay chip is one sign for a whole primary-code period, so two
    # half-block records of the same block carry the *same* chip. Advancing the
    # overlay counter once per record instead of once per code wrap put every
    # second record on the wrong chip — and a wrong sign is invisible to every
    # discriminator downstream.
    signal = GPSL5I()
    prn = 11
    sampling_freq = 2 * uconvert(Hz, get_code_frequency(signal))
    block = primary_block_samples(signal, sampling_freq)
    half = block ÷ 2
    overlay = get_secondary_code(signal)
    link, track_state = partial_link(
        signal,
        prn;
        sampling_freq,
        synced = true,
        capabilities = partial_capabilities(; max_code_length = 10_230),
    )
    link.last_record_end[1] = 0
    anchor_secondary_phases!(link, track_state)
    @test is_secondary_code_removed(link, 1)

    # A constant +1 symbol as the device dumps it: the block's overlay chip on
    # every half-block record of that block.
    append_tiling_dumps!(
        link,
        track_state,
        signal,
        prn,
        1,
        0,
        2 * get_secondary_code_length(signal),
        half;
        sampling_freq,
        prompt = k -> complex(Float64(secondary_value(overlay, prn, k ÷ 2))),
    )
    @test link.secondary_phase_losses == 0
    @test primary_code_wraps(link, 1) == get_secondary_code_length(signal)
    record = only(records(track_state, prn))
    # Every record wiped with its own block's chip: the symbol adds up rather
    # than the overlay.
    @test Tracking.get_prompt(record.correlator) ≈
          2 * get_secondary_code_length(signal) + 0im
end

@testset "A long primary code gets timely loop updates without inventing wraps" begin
    # GPS L2CL: 767250 chips at 511.5 kcps, one code period every 1.5 s. Waiting
    # for a code wrap would update the loop filters twice per three seconds; the
    # link instead cuts a record at `max_integration_time` and says so, without
    # any of those records being counted as a completed code period.
    signal = L2CL
    prn = 15
    sampling_freq = 2 * uconvert(Hz, get_code_frequency(signal))
    block = primary_block_samples(signal, sampling_freq)
    dump = round(Int, ustrip(Hz, sampling_freq) * 1e-3)     # a 1 ms dump
    link, track_state =
        partial_link(signal, prn; sampling_freq, max_integration_time = 20u"ms")

    @test allows_partial_primary_records(link, signal)
    @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) == 1
    # One code block is 1.5 s; the record the loops get is 20 ms of it.
    @test coherent_integration_periods(link, get_sat_state(track_state, prn), 1, 1) ≈
          0.02 / 1.5

    append_tiling_dumps!(link, track_state, signal, prn, 1, 0, 100, dump; sampling_freq)
    out = records(track_state, prn)
    @test length(out) == 5                       # 100 ms of 1 ms dumps, cut at 20 ms
    @test all(o -> o.integrated_samples == 20 * dump, out)
    # Not one of them completed a code period, and none was counted as one.
    @test primary_code_wraps(link, 1) == 0
    @test link.pending_blocks[1] == 0
    @test link.partial_primary_records == 5
    @test primary_code_block_phase(link, 1) ≈ 100 * dump / block atol = 1e-9
end

@testset "A very short primary code does not thrash the loops" begin
    # Galileo E5a-QP's primary code is 330 chips at 5.115 Mcps — 64.5 µs. One
    # record per code period is fifteen thousand records a second, each with a
    # sixteenth of the energy. `Tracking` states the ceiling for exactly this
    # signal (one 2 ms, 31-block code cycle); the link folds to it.
    signal = GalileoE5aQP()
    prn = 3
    sampling_freq = 2 * uconvert(Hz, get_code_frequency(signal))
    block = primary_block_samples(signal, sampling_freq)
    link, track_state = partial_link(
        signal,
        prn;
        sampling_freq,
        synced = true,
        capabilities = partial_capabilities(; max_code_length = 10_230),
    )

    @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) == 31
    @test !allows_partial_primary_records(link, signal)
    append_tiling_dumps!(link, track_state, signal, prn, 1, 0, 62, block; sampling_freq)
    out = records(track_state, prn)
    @test length(out) == 2
    @test all(o -> o.integrated_samples == 31 * block, out)
    @test primary_code_wraps(link, 1) == 62
end

@testset "Timing and accounting across L1 C/A, L2CM, L2CL and a short code" begin
    # The four corners of the scope, measured the same way: what one code period
    # is, whether a record may be shorter than one, and how many code periods the
    # loops are handed at a time. The numbers are the signals' own — nothing here
    # is a receiver setting except `max_integration_time`.
    cases = (
        (GPSL1CA(), 1e-3, false, 20, 1),
        (GPSL2CM(), 20e-3, false, 1, 1),
        (L2CL, 1.5, true, 1, 4),
        (GalileoE5aQP(), 330 / 5.115e6, false, 31, 1),
    )
    for (signal, period, partial, blocks, dumps_per_block) in cases
        prn = 1
        sampling_freq = 2 * uconvert(Hz, get_code_frequency(signal))
        block = primary_block_samples(signal, sampling_freq)
        link, track_state = partial_link(
            signal,
            prn;
            sampling_freq,
            synced = true,
            max_integration_time = 20u"ms",
            capabilities = partial_capabilities(),
        )
        @test block / ustrip(Hz, sampling_freq) ≈ period rtol = 1e-9
        @test allows_partial_primary_records(link, signal) == partial
        @test coherent_integration_blocks(link, get_sat_state(track_state, prn), 1, 1) ==
              blocks
        # A record is capped at 20 ms of signal time however long a code period
        # is, and never asks for less than the dumps it is fed.
        target = coherent_integration_periods(link, get_sat_state(track_state, prn), 1, 1)
        @test target * period <= 20e-3 * (1 + 1e-9)
        @test target > 0

        # Records tiling four code periods: the wrap count is the code's, whatever
        # the dump length is.
        @test block % dumps_per_block == 0
        dump = block ÷ dumps_per_block
        append_tiling_dumps!(
            link,
            track_state,
            signal,
            prn,
            1,
            0,
            4 * dumps_per_block,
            dump;
            sampling_freq,
        )
        @test primary_code_wraps(link, 1) == 4
        @test primary_code_block_phase(link, 1) == 0.0
    end
end

@testset "A device that cannot dump inside a code period is refused before arming" begin
    # The pre-arm gate issue #130 asks for: a device whose dumps are one per code
    # period cannot serve a 1.5 s code, and saying so before a channel is armed
    # is the difference between an actionable error and a satellite that tracks
    # at 0.67 Hz of loop bandwidth and is never understood.
    signal = L2CL
    correlator = Tracking.get_default_correlator(signal, NumAnts(1))
    sampling_freq = 2 * uconvert(Hz, get_code_frequency(signal))

    message = GNSSReceiver.hardware_support_error(
        partial_capabilities(; partial_dumps = false),
        signal,
        correlator,
        sampling_freq,
    )
    @test !isnothing(message)
    @test occursin("1.5", message)
    @test occursin("supports_partial_code_dumps", message)

    # The same device with the declaration serves it.
    @test isnothing(
        GNSSReceiver.hardware_support_error(
            partial_capabilities(; partial_dumps = true),
            signal,
            correlator,
            sampling_freq,
        ),
    )
    # And a short code is unaffected either way.
    for partial_dumps in (false, true)
        @test isnothing(
            GNSSReceiver.hardware_support_error(
                partial_capabilities(; partial_dumps),
                GPSL1CA(),
                Tracking.get_default_correlator(GPSL1CA(), NumAnts(1)),
                4e6Hz,
            ),
        )
    end

    # `validate_hardware_configuration` reports it the same way, and the message
    # names the signal.
    sdr = CapableSDR(EPL, 4, partial_capabilities(; partial_dumps = false))
    err = try
        GNSSReceiver.validate_hardware_configuration(sdr, signal, sampling_freq)
        nothing
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("GPSL2CL", err.msg)
end

@testset "GPS L1 C/A is unchanged by a device that dumps once per code period" begin
    # The regression baseline. A device on the old contract — one record per code
    # period, no reported code phase — must fold exactly as it always did.
    signal = GPSL1CA()
    prn = 9
    sampling_freq = 4e6Hz
    link, track_state = partial_link(signal, prn; sampling_freq, synced = true)
    for k = 1:20
        GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 4000k))
    end
    record = only(records(track_state, prn))
    @test record.integrated_samples == 80_000
    @test record.sample_index == 80_000
    @test Tracking.get_prompt(record.correlator) ≈ 20 + 0im
    @test link.pending_blocks[1] == 20
    @test primary_code_wraps(link, 1) == 20
    @test link.partial_primary_records == 0
end

@testset "Simulated device: partial dumps sum to the reference correlation" begin
    # A partial record is a real integration or it is nothing. The simulated
    # correlator dumps every millisecond inside GPS L2CL's 1.5 s code period,
    # armed on the harness's own truth, and the sum of its dumps over a span has
    # to be what an ideal correlator locked on that truth would have accumulated
    # over the same span — the harness's `reference_correlation`, not another
    # path's answer.
    signal = L2CL
    prn = 7
    sampling_freq = 2 * uconvert(Hz, get_code_frequency(signal))
    fs = ustrip(Hz, sampling_freq)
    dump_samples = round(Int, fs * 1e-3)              # a 1 ms dump
    num_dumps = 5
    case = ReferenceCase(
        signal;
        prn,
        sampling_freq,
        code_phase = 0.0,
        noise_power = 0.0,
        amplitude = 1.0,
    )
    sdr = SimulatedFPGA(
        signal;
        sampling_freq,
        chunk = num_dumps * dump_samples,
        n_channels = 1,
        dump_interval_samples = dump_samples,
    )
    # The device declares that it can be told to dump inside a code period —
    # which is exactly what makes it able to serve this signal at all.
    @test GNSSReceiver.hardware_capabilities(sdr).supports_partial_code_dumps
    GNSSReceiver.assign_channel!(
        sdr,
        1,
        prn,
        0.0Hz,
        0.0Hz,
        case.code_phase,
        0;
        el_sample_spacing = 2,
        signal,
    )
    samples = generate_samples(case, num_dumps * dump_samples)
    dumps = correlate_chunk!(sdr, view(samples, :, 1))
    produced = filter(d -> !GNSSReceiver.is_epoch_strobe(d), dumps)

    @test length(produced) == num_dumps
    @test all(d -> d.output.integrated_samples == dump_samples, produced)
    # Each record carries the replica's absolute code phase, which is what lets
    # the host place a sub-period record on the code-block grid at all.
    @test all(d -> !isnan(d.code_phase), produced)
    @test last(produced).code_phase ≈
          num_dumps * dump_samples * ustrip(Hz, get_code_frequency(signal)) / fs rtol = 1e-9
    # No dump completed a code period: a 5 ms slice of a 1.5 s code.
    @test all(d -> d.code_phase < get_code_length(signal), produced)

    layout = quantize_taps(epl_taps(), sampling_freq, code_frequency(case))
    expected = reference_correlation(case, layout, num_dumps * dump_samples)
    summed = sum(
        SVector{3,ComplexF64}(Tracking.get_accumulators(d.output.correlator)) for
        d in produced
    )
    @test summed ≈ SVector{3,ComplexF64}(expected[:, 1]) rtol = 1e-9
end

@testset "Simulated device: GPS L2CL is folded from partial dumps" begin
    # The link over the same device, chunk by chunk, with the replica handed over
    # half a chip off so the code loop has something to do: the loops are handed
    # a record every `max_integration_time` of a 1.5 s code period, the device
    # receives a correction per chunk rather than one per code wrap, the DLL
    # pulls the replica in — and across two seconds of replay the code completes
    # exactly one wrap, counted once, out of nearly two hundred records not one
    # of which was a code period.
    signal = L2CL
    prn = 12
    sampling_freq = 2 * uconvert(Hz, get_code_frequency(signal))
    fs = ustrip(Hz, sampling_freq)
    dump_samples = round(Int, fs * 1e-3)
    chunk = 20 * dump_samples                        # 20 ms of signal per chunk
    num_chunks = 100                                 # 2 s: past the 1.5 s code wrap
    handover_error = 0.5                             # chips the DLL has to pull in
    case = ReferenceCase(signal; prn, sampling_freq, cn0_dbhz = 48.0, code_phase = 0.0)
    source = SampleSource(case)
    sdr = SimulatedFPGA(
        signal;
        sampling_freq,
        chunk,
        n_channels = 2,
        epoch_length = chunk,
        dump_interval_samples = dump_samples,
        handover_code_phase_error = handover_error,
    )
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq,
        reference_signal = signal,
        doppler_update_interval = chunk / sampling_freq,
        noise_source = :channel,
        max_integration_time = 20u"ms",
    )
    group = GNSSReceiver.signal_group_key(signal)
    track_state = TrackState(;
        signals = NamedTuple{(group,)}(((signal,),)),
        doppler_estimator = GNSSReceiver.NCOReferencedPLLAndDLL(),
    )
    track_state = add_satellite!(
        track_state;
        prn,
        group,
        code_phase = case.code_phase,
        carrier_doppler = 0.0Hz,
    )
    band_systems = ((signal,),)

    code_length = get_code_length(signal)
    # Where the truth's code phase stands at a device sample.
    truth_phase(n) = mod(case.code_phase + n * code_frequency(case) / fs, code_length)
    # The estimator consumes each chunk's records inside `advance_tracking!`, so
    # what a chunk delivered is counted as it goes: one partial-primary record
    # per chunk is at least one loop update per 20 ms of a 1.5 s code period.
    emitted = Int[]
    errors = Float64[]
    for _ = 1:num_chunks
        samples = next_samples!(source, chunk)
        correlate_chunk!(sdr, view(samples, :, 1))
        band_measurements = (; L2 = Tracking.BandMeasurement(samples, sampling_freq, 0.0Hz))
        track_state = advance_tracking!(link, band_measurements, track_state, band_systems)
        push!(emitted, link.partial_primary_records)
        channel = get(link.channel_of, HardwareChannelAssignment(group, prn, 1), 0)
        channel > 0 &&
            sdr.channels[channel].active &&
            push!(
                errors,
                rem(
                    sdr.channels[channel].code_phase - truth_phase(sdr.sample_count),
                    code_length,
                    RoundNearest,
                ),
            )
    end

    hw_channel = link.channel_of[HardwareChannelAssignment(group, prn, 1)]
    # At least a record per chunk once the channel is armed — the loops are
    # updated ~75 times per code period rather than waiting 1.5 s for the wrap.
    @test last(emitted) >= num_chunks - 1
    @test issorted(emitted)
    # And the device really did receive those corrections: a loop, not a
    # heartbeat. One per chunk, against the one per 1.5 s a code-wrap dump
    # contract would have allowed.
    @test length(sdr.applied) >= num_chunks - 5
    # The code loop pulls the replica in from the handover error, driven entirely
    # by records shorter than a code period.
    @test first(errors) ≈ handover_error atol = 0.05
    @test abs(last(errors)) < 0.75 * handover_error
    @test abs(last(errors)) < abs(errors[num_chunks÷2])
    # Every record the loops were handed was a partial one — that is what a 1.5 s
    # code period looks like from the accounting's side — and the code's one wrap
    # in two seconds is counted once, not once per record.
    @test link.partial_primary_records == last(emitted)
    @test link.partial_primary_records > 100
    @test primary_code_wraps(link, hw_channel) == 1
    @test link.pending_blocks[hw_channel] == 0
    @test link.misaligned_dump_boundaries == 0
    # The block phase is where the replayed span lands inside the 1.5 s code,
    # to within the chunk the fold grid lags by.
    block = primary_block_samples(signal, sampling_freq)
    @test primary_code_block_phase(link, hw_channel) ≈ mod(num_chunks * chunk / block, 1.0) atol =
        2 * chunk / block
    # Nothing was lost and nothing overlapped: the records tile the sample axis
    # across every cut, including the one the code wrap put in the middle.
    @test link.lost_record_gaps == 0
    @test link.overlapping_record_samples[hw_channel] == 0
    @test link.rearm_gaps == 0
end
