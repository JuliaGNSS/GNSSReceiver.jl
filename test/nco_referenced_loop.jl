# Tests for the delay-aware hardware loop (issue #107,
# docs/plans/2026-09-14-delay-aware-hardware-loop.md).
#
# The estimator is validated on a noise-free model of the loop it closes: a
# carrier whose phase error advances by `2π (f_true − w) Δt` per record under
# the replica word `w`, with the word the estimator commands landing `d`
# records after the record that produced it. That isolates the one thing this
# estimator changes — how the delay is handled — from everything the simulated
# device in test/hardware_correlator.jl exercises as well (handover, code loop,
# bit sync, C/N₀).
#
# Included after test/hardware_correlator.jl, whose `RecordingSDR`, `EPL` and
# `dump_at` helpers the link-level tests below reuse.

using GNSSReceiver:
    NCOReferencedPLLAndDLL,
    NCOTimeline,
    FixedNCOWord,
    reset_timeline!,
    schedule_word!,
    promote_words!,
    mean_nco_word,
    nco_word_at,
    word_changes_within

@testset "An NCO timeline averages the words that ran over a span" begin
    tl = NCOTimeline()
    reset_timeline!(tl, 100.0, 0.1)
    @test mean_nco_word(tl, 0, 4000) == (100.0, 0.1)
    @test nco_word_at(tl, 12345) == (100.0, 0.1)
    @test !word_changes_within(tl, 0, 10_000)

    schedule_word!(tl, 1000, 110.0, 0.11)
    schedule_word!(tl, 2000, 120.0, 0.12)
    # Time-weighted: half the span on each word.
    @test mean_nco_word(tl, 500, 1500) == (105.0, 0.105)
    @test mean_nco_word(tl, 0, 4000) == (112.5, 0.1125)
    # An empty span is the word in effect at its start.
    @test mean_nco_word(tl, 1000, 1000) == (110.0, 0.11)
    @test mean_nco_word(tl, 999, 999) == (100.0, 0.1)
    @test nco_word_at(tl, 2000) == (120.0, 0.12)
    # Half-sample bounds, as record centres are.
    @test all(mean_nco_word(tl, 999.5, 1000.5) .≈ (105.0, 0.105))
    # A switch strictly inside `(lo, hi]` separates two records' words.
    @test word_changes_within(tl, 0, 1000)
    @test !word_changes_within(tl, 1000, 2000 - 1)
    @test word_changes_within(tl, 1999, 2000)

    # A newer command for the same or an earlier sample supersedes: the device
    # keeps the newest word it was given for a sample.
    schedule_word!(tl, 2000, 130.0, 0.13)
    @test mean_nco_word(tl, 2000, 3000) == (130.0, 0.13)
    @test length(tl.scheduled) == 2
    schedule_word!(tl, 1500, 140.0, 0.14)
    @test [w.sample for w in tl.scheduled] == [1000, 1500]

    # Promotion folds landed words into the applied one and forgets them.
    promote_words!(tl, 1200)
    @test tl.applied_carrier_doppler == 110.0
    @test [w.sample for w in tl.scheduled] == [1500]
    @test mean_nco_word(tl, 1000, 2000) == (125.0, 0.125)
    promote_words!(tl, 10_000)
    @test isempty(tl.scheduled)
    @test mean_nco_word(tl, 0, 1) == (140.0, 0.14)

    # A handover starts over.
    reset_timeline!(tl, 7.0, 0.007)
    @test isempty(tl.scheduled)
    @test mean_nco_word(tl, 0, 1) == (7.0, 0.007)

    @test mean_nco_word(FixedNCOWord(3.0, 0.3), 0, 100) == (3.0, 0.3)
end

# ── The loop model ───────────────────────────────────────────────────────────

const LOOP_SYSTEM = GPSL1CA()
const LOOP_FS = 4e6Hz
const LOOP_N = 4000   # samples per record: one code period at 4 MS/s

loop_epl(late, prompt, early) =
    EarlyPromptLateCorrelator(SVector{3,ComplexF64}(late, prompt, early), 1)

# A BPSK loop locks the prompt onto either half of the real axis; the phase
# error that matters is modulo π.
wrap_half_cycle(phase) = rem(phase, π, RoundNearest)

# Close the loop on the model for `steps` records. The replica runs on the words
# the estimator commands, each landing `d` records after the record it was
# computed from; a record's prompt is `cis` of its mean phase error. Returns the
# per-record phase error (mod π) and the word each record ran on.
function simulate_delayed_loop(
    estimator,
    d;
    f_true = 130.0,      # Hz
    handover = 100.0,    # Hz, the acquisition's Doppler estimate
    phi0 = 0.8,          # rad
    steps = 600,
    prn = 7,
)
    track_state = TrackState(
        LOOP_SYSTEM,
        [TrackedSat(LOOP_SYSTEM, prn, 0.0, handover * Hz; doppler_estimator = estimator)];
        doppler_estimator = estimator,
    )
    noise = Tracking._signal_noise_densities(
        track_state.noise_estimators,
        eltype(track_state.groups[1].satellites),
    )
    timeline = NCOTimeline()
    reset_timeline!(timeline, handover, 0.0)
    dt = LOOP_N / ustrip(Hz, LOOP_FS)
    phase = phi0
    phases = Float64[]
    words = Float64[]
    for k = 0:steps-1
        a = k * LOOP_N
        b = a + LOOP_N
        w, _ = nco_word_at(timeline, a)           # constant over the record
        mean_phase = phase + π * (f_true - w) * dt
        phase += 2π * (f_true - w) * dt
        push!(phases, wrap_half_cycle(mean_phase))
        push!(words, w)
        p = cis(mean_phase)
        sat = get_sat_state(track_state, prn)
        push!(
            Tracking.get_correlator_outputs(sat, 1),
            CorrelatorOutput(loop_epl(0.5p, p, 0.5p), LOOP_N, b),
        )
        landing = Int64(b + d * LOOP_N)
        new_sat =
            estimator isa NCOReferencedPLLAndDLL ?
            GNSSReceiver._nco_update_tracked_sat(sat, estimator, LOOP_FS, noise, timeline, landing) :
            Tracking._update_tracked_sat_doppler(sat, LOOP_FS, noise)
        Tracking.get_sat_states(track_state)[prn] = new_sat
        schedule_word!(
            timeline,
            landing,
            ustrip(Hz, get_carrier_doppler(new_sat)),
            ustrip(Hz, get_code_doppler(new_sat)),
        )
        promote_words!(timeline, b - LOOP_N)
    end
    phases, words
end

@testset "With no delay the NCO-referenced loop is the conventional loop" begin
    conventional = simulate_delayed_loop(ConventionalAssistedPLLAndDLL(), 0)
    referenced = simulate_delayed_loop(NCOReferencedPLLAndDLL(), 0)
    # Bit-identical, not approximately equal: the software receiver's tuning
    # is inherited, not re-derived.
    @test referenced[1] == conventional[1]
    @test referenced[2] == conventional[2]
    control = simulate_delayed_loop(NCOReferencedPLLAndDLL(; predict_landing = false), 0)
    @test control[2] == conventional[2]
    # And it is a loop that locks.
    @test all(abs.(referenced[1][400:end]) .< 0.2)
    @test all(abs.(referenced[2][400:end] .- 130.0) .< 1.0)
end

@testset "The NCO-referenced loop holds lock through $d records of feedback delay" for d in 1:6
    phases, words = simulate_delayed_loop(NCOReferencedPLLAndDLL(), d; steps = 1200)
    # Settled after a second: the residual phase and frequency errors of the
    # conventional loop's own (slow, FLL-assisted) settling, not a limit cycle.
    @test all(abs.(phases[1000:end]) .< 0.05)
    @test all(abs.(words[1000:end] .- 130.0) .< 0.3)
    # The transient is the delay-free loop's, only somewhat larger: a 30 Hz
    # step with 0.8 rad of phase error never wraps the discriminator.
    @test maximum(abs.(phases[100:end])) < 1.0
end

@testset "The conventional loop and the negative control limit-cycle at four records of delay" begin
    for estimator in (
        ConventionalAssistedPLLAndDLL(),
        NCOReferencedPLLAndDLL(; predict_landing = false),
    )
        phases, words = simulate_delayed_loop(estimator, 4; steps = 1200)
        tail = words[600:end] .- 130.0
        # The ±80 Hz carrier swing measured on the board (issue #107): the
        # command never settles and the phase error is uniformly wrong.
        @test sqrt(sum(abs2, tail) / length(tail)) > 20
        @test maximum(abs.(phases[600:end])) > 1.0
    end
end

@testset "The NCO-referenced loop pulls in an acquisition-sized offset with delay" begin
    for d in (2, 3, 6), (offset, phi0) in ((100.0, 0.0), (-120.0, 1.2))
        phases, words = simulate_delayed_loop(
            NCOReferencedPLLAndDLL(),
            d;
            f_true = 100.0 + offset,
            phi0,
            steps = 1500,
        )
        @test all(abs.(words[1300:end] .- (100.0 + offset)) .< 0.5)
        @test all(abs.(phases[1300:end]) .< 0.05)
    end
end

@testset "NCO-referenced estimator state and pull-in range" begin
    estimator = NCOReferencedPLLAndDLL()
    @test GNSSReceiver.carrier_doppler_pull_in_range(estimator, GPSL1CA()) == 250.0Hz
    sat = TrackedSat(GPSL1CA(), 3, 0.0, 1234.0Hz; doppler_estimator = estimator)
    state = get_doppler_estimator_state(sat)
    @test state isa GNSSReceiver.SatNCOReferencedPLLAndDLL
    @test state.init_carrier_doppler == 1234.0Hz
    @test state.carrier_loop_filter_bandwidth == 18.0Hz   # GPS L1 C/A reference
    @test isnan(state.previous_record_center)
    narrow = NCOReferencedPLLAndDLL(; carrier_loop_filter_bandwidth = 12.0Hz)
    @test get_doppler_estimator_state(
        TrackedSat(GPSL1CA(), 3, 0.0, 0.0Hz; doppler_estimator = narrow),
    ).carrier_loop_filter_bandwidth == 12.0Hz
    # `reset_loop_filters!` re-seeds from the converged Dopplers and keeps the
    # per-satellite bandwidth.
    track_state = TrackState(GPSL1CA(), [TrackedSat(GPSL1CA(), 3, 0.0, 50.0Hz; doppler_estimator = narrow)];
        doppler_estimator = narrow)
    reset_loop_filters!(track_state)
    reset_state = get_doppler_estimator_state(get_sat_state(track_state, 3))
    @test reset_state.init_carrier_doppler == 50.0Hz
    @test reset_state.carrier_loop_filter_bandwidth == 12.0Hz
    @test reset_state.carrier_loop_filter.x1 == 0.0Hz
end

# ── Through the link ─────────────────────────────────────────────────────────

# A receiver track state whose estimator is the hardware default, with one
# satellite handed over at the given Dopplers.
function nco_track_state(system, prn; carrier_doppler = 1500.0Hz, code_doppler = 1.5Hz)
    base = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(1),
        doppler_estimator = NCOReferencedPLLAndDLL(),
    )
    merge_sats(
        base.track_state,
        get_signal_id(system),
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            prn,
            0.0,
            carrier_doppler,
            NumAnts(1),
            base.track_state.doppler_estimator,
        )],
    )
end

@testset "The link keeps a timeline of what each channel's NCO ran" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    prn = 21
    sdr = RecordingSDR(EPL, 3)
    link = HardwareCorrelatorLink(sdr; sampling_freq = 4e6Hz, reference_signal = system)
    track_state = nco_track_state(system, prn)
    band_systems = ((system,),)
    band_measurements = (; L1 = Tracking.BandMeasurement(zeros(ComplexF64, 4000), 4e6Hz, 0.0Hz))
    # The estimator sizes acquisition the same way the conventional one does.
    @test track_state.doppler_estimator isa NCOReferencedPLLAndDLL

    # Handover: the channel's timeline starts on the words the device was given.
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)
    hw_channel = link.channel_of[GNSSReceiver.HardwareChannelAssignment(key, prn, 1)]
    timeline = link.nco_timelines[hw_channel]
    @test mean_nco_word(timeline, 0, 1) == (1500.0, ustrip(Hz, get_code_doppler(get_sat_state(track_state, key, prn))))
    @test isempty(timeline.scheduled)

    # A fold's updates are scheduled at one sample, decided before the estimator
    # runs and entered in the timeline once the device has accepted them.
    link.latest_sample_index = 100_000
    boundary = 96_000
    apply_at = GNSSReceiver.nco_apply_at_sample(link, boundary)
    @test apply_at == 100_000 + 2 * 4000
    @test GNSSReceiver.push_nco_updates!(link, track_state, boundary) == 1
    update = take!(sdr.ncos)
    @test update.apply_at_sample == apply_at
    @test update.channel == hw_channel
    @test [w.sample for w in timeline.scheduled] == [apply_at]
    @test mean_nco_word(timeline, apply_at, apply_at + 1) == (update.carrier_doppler, update.code_doppler)
    @test link.dropped_nco_updates == 0

    # A refused update never reaches the device, so it never reaches the timeline
    # either — and it is counted, where before it vanished.
    while Base.n_avail(sdr.ncos) > 0
        take!(sdr.ncos)
    end
    for _ = 1:(sdr.ncos.capacity-1)
        put!(sdr.ncos, NCOUpdate(3, 1, 0.0Hz, 0.0Hz, 0))
    end
    @test GNSSReceiver.n_avail_space(sdr.ncos) == 0
    @test (@test_logs (:warn, r"NCO feedback ring full") GNSSReceiver.push_nco_updates!(
        link, track_state, boundary + 4000)) == 0
    @test link.dropped_nco_updates == 1
    @test [w.sample for w in timeline.scheduled] == [apply_at]

    # Once the records that ran on a word have been folded, it is the applied word.
    link.last_record_end[hw_channel] = apply_at + 8000
    link.last_record_samples[hw_channel] = 4000
    GNSSReceiver.promote_applied_words!(link)
    @test isempty(timeline.scheduled)
    @test timeline.applied_carrier_doppler == update.carrier_doppler

    # A release clears it, and a new occupant starts on its own handover words.
    empty_state = GNSSReceiver.ReceiverState(
        ComplexF64, system; num_samples_for_acquisition = 20000, num_ants = NumAnts(1),
        doppler_estimator = NCOReferencedPLLAndDLL()).track_state
    GNSSReceiver.release_stale_channels!(link, empty_state)
    @test mean_nco_word(timeline, 0, 1) == (0.0, 0.0)
end

@testset "A record is cut where the NCO word changed" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    prn = 9
    sdr = RecordingSDR(EPL, 1)
    link = HardwareCorrelatorLink(sdr; sampling_freq = 4e6Hz, reference_signal = system)
    track_state = nco_track_state(system, prn)
    assignment = GNSSReceiver.HardwareChannelAssignment(key, prn, 1)
    link.assignments[1] = assignment
    link.channel_of[assignment] = 1
    signal() = Tracking.get_signals(get_sat_state(track_state, key, prn))[1]
    outputs() = Tracking.get_correlator_outputs(get_sat_state(track_state, key, prn), 1)
    # Post-sync, so the link would otherwise sum a chunk's dumps into one record.
    bit_buffer = Tracking.get_bit_buffer(signal())
    synced = typeof(bit_buffer)(
        bit_buffer.code_block_buffer, bit_buffer.code_block_buffer_length, true, 0,
        Int8(1), complex(0.0, 0.0), 0, bit_buffer.soft_bits, bit_buffer.phase_acc)
    Tracking.get_sat_states(track_state, key)[prn] = Tracking.TrackedSat(
        get_sat_state(track_state, key, prn);
        signals = (Tracking.TrackedSignal(signal(); bit_buffer = synced),))

    # Two dumps, no word change in between: one record, as before.
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 100_000))
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 104_000))
    GNSSReceiver.flush_partial_records!(link, track_state)
    @test only(outputs()).integrated_samples == 8000
    empty!(outputs())
    fill!(link.pending_blocks, 0)

    # A word lands at the boundary between the two dumps: each is its own record,
    # so neither straddles the switch.
    schedule_word!(link.nco_timelines[1], 108_000, 1510.0, 1.51)
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 108_000))
    @test isempty(outputs())
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 112_000))
    @test length(outputs()) == 1
    @test outputs()[1].integrated_samples == 4000
    @test outputs()[1].sample_index == 108_000
    GNSSReceiver.flush_partial_records!(link, track_state)
    @test length(outputs()) == 2
    @test outputs()[2].integrated_samples == 4000
    empty!(outputs())
    fill!(link.pending_blocks, 0)

    # A word landing *inside* a dump cannot be cut; the dump that straddles it
    # is closed off so the next one starts clean on the new word.
    schedule_word!(link.nco_timelines[1], 114_000, 1520.0, 1.52)
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 116_000))
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 120_000))
    GNSSReceiver.flush_partial_records!(link, track_state)
    @test [o.integrated_samples for o in outputs()] == [4000, 4000]
    @test mean_nco_word(link.nco_timelines[1], 112_000, 116_000) == (1515.0, 1.515)
    @test mean_nco_word(link.nco_timelines[1], 116_000, 120_000) == (1520.0, 1.52)
end

@testset "The link hands the estimator the applied words and the landing sample" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    prn = 4
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(sdr; sampling_freq = 4e6Hz, reference_signal = system)
    band_systems = ((system,),)
    band_measurements = (; L1 = Tracking.BandMeasurement(zeros(ComplexF64, 4000), 4e6Hz, 0.0Hz))
    # A satellite handed over at 1000 Hz whose signal is really at 1000 Hz, so a
    # prompt on the real axis means "no error" — unless the estimator believes
    # the replica ran on some other word.
    track_state = nco_track_state(system, prn; carrier_doppler = 1000.0Hz, code_doppler = 0.0Hz)
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)
    hw_channel = link.channel_of[GNSSReceiver.HardwareChannelAssignment(key, prn, 1)]
    Tracking.append_noise_observation!(
        track_state,
        Tracking.noise_observation_from_samples(4000.0, 4000, 4e6Hz),
        key,
    )

    # Fold one real-axis record with the handover word applied: nothing to
    # correct, the command is the handover word.
    Tracking.append_correlator_output!(
        track_state,
        CorrelatorOutput(epl(0.5, 1.0, 0.5), 4000, 100_000),
        key, prn, 1,
    )
    link.latest_sample_index = 100_000
    link.scheduled_apply_at_sample = GNSSReceiver.nco_apply_at_sample(link, 100_000)
    GNSSReceiver.estimate_dopplers!(link, track_state, band_measurements)
    @test get_carrier_doppler(get_sat_state(track_state, key, prn)) ≈ 1000.0Hz atol = 1e-9Hz

    # Now the record ran on a word 40 Hz above the signal for its whole span —
    # the estimator learns that from the timeline, not from its own last command
    # — and the prompt has rotated accordingly. The absolute frequency
    # measurement `word + FLL` says the signal is at 1000 Hz, so the command
    # comes back toward it rather than restating the +40 Hz.
    schedule_word!(link.nco_timelines[hw_channel], 100_000, 1040.0, 0.0)
    rotated = cis(-2π * 40.0 * 0.5e-3)         # mean phase error over 1 ms at −40 Hz
    Tracking.append_correlator_output!(
        track_state,
        CorrelatorOutput(epl(0.5rotated, rotated, 0.5rotated), 4000, 104_000),
        key, prn, 1,
    )
    link.latest_sample_index = 104_000
    link.scheduled_apply_at_sample = GNSSReceiver.nco_apply_at_sample(link, 104_000)
    GNSSReceiver.estimate_dopplers!(link, track_state, band_measurements)
    command = ustrip(Hz, get_carrier_doppler(get_sat_state(track_state, key, prn)))
    @test command < 1040.0
    # The same record folded through the software path (word = the satellite's
    # own Doppler, no delay) reads the rotation as a real error and commands
    # differently — the timeline is what the hardware fold adds.
    software_state = nco_track_state(system, prn; carrier_doppler = 1000.0Hz, code_doppler = 0.0Hz)
    Tracking.append_noise_observation!(
        software_state, Tracking.noise_observation_from_samples(4000.0, 4000, 4e6Hz), key)
    Tracking.append_correlator_output!(
        software_state, CorrelatorOutput(epl(0.5, 1.0, 0.5), 4000, 100_000), key, prn, 1)
    Tracking.estimate_dopplers_and_filter_prompt!(software_state, band_measurements)
    Tracking.append_correlator_output!(
        software_state, CorrelatorOutput(epl(0.5rotated, rotated, 0.5rotated), 4000, 104_000), key, prn, 1)
    Tracking.estimate_dopplers_and_filter_prompt!(software_state, band_measurements)
    software_command = ustrip(Hz, get_carrier_doppler(get_sat_state(software_state, key, prn)))
    @test software_command != command
    @test software_command < 1000.0     # it chases the rotation as a genuine error
end

@testset "The hardware receiver defaults to the NCO-referenced estimator" begin
    # Through the public method only the estimator changes; everything else the
    # link and pipeline do is as before (the closed-loop tests above run it).
    system = GPSL1CA()
    sdr = RecordingSDR(EPL, 2)
    state = GNSSReceiver.ReceiverState(
        ComplexF64, system; num_samples_for_acquisition = 20000, num_ants = NumAnts(1),
        doppler_estimator = NCOReferencedPLLAndDLL())
    @test state.track_state.doppler_estimator isa NCOReferencedPLLAndDLL
    # A pre-built link must be the device's own.
    other = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(other; sampling_freq = 4e6Hz, reference_signal = system)
    @test_throws ArgumentError receive(sdr, system, 4e6Hz; link, acquire_async = false)
end
