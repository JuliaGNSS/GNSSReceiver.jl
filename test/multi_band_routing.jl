# ─────────────────────────────────────────────────────────────────────────────
# Routing hardware channels and noise estimates across RF bands (issue #134).
#
# Supporting every signal individually is not the same as receiving every RF
# band simultaneously, and this file is where the difference is pinned down:
#
#   * two bands whose device counters tick at *different* rates are mapped onto
#     one documented receiver timebase, so an epoch closes at one instant in
#     time rather than at one integer on whichever counter happens to run
#     fastest;
#   * a channel is armed on, and fed back to, the RF input its band actually
#     arrives on;
#   * each band keeps its own noise reference and its own replica gain — the
#     floors of two unrelated front ends are never pooled;
#   * antenna inputs stay distinct from independently tuned bands;
#   * a request past the front end's capacity is refused before anything is
#     armed, rather than half-configured in silence;
#   * and a fault on one band — lost records, a full feedback ring — does not
#     reach the other.
# ─────────────────────────────────────────────────────────────────────────────

using GNSSReceiver:
    HardwareBandPlan,
    HardwareBandRoute,
    HardwareCorrelatorLink,
    HardwareCorrelatorCapabilities,
    HardwareChannelAssignment,
    AbstractHardwareCorrelatorSDR,
    CorrelatorDump,
    NCOUpdate,
    band_sampling_frequency,
    receiver_timebase_scale,
    to_receiver_samples,
    to_band_samples,
    band_rf_input,
    band_device_index,
    clock_synchronization,
    reference_band,
    validate_hardware_configuration,
    hardware_band_plan,
    epoch_strobe

using PipeChannels: PipeChannel
using Tracking: CorrelatorOutput, EarlyPromptLateCorrelator

# ─────────────────────────────────────────────────────────────────────────────
# A two-band recording device. It correlates nothing: the tests hand it exactly
# the dumps they want to see routed, which is what makes the timebase algebra
# checkable to the sample.
# ─────────────────────────────────────────────────────────────────────────────

const MB_BANDS = (:L1, :B1I)

mutable struct TwoBandSDR{C} <: AbstractHardwareCorrelatorSDR
    const dumps::PipeChannel{CorrelatorDump{C}}
    const ncos::PipeChannel{NCOUpdate}
    const n_channels::Int
    const assigned::Vector{Any}
    const released::Vector{Int}
    const capabilities::HardwareCorrelatorCapabilities
    const band_gains::Dict{Symbol,Float64}
    const clock_sync::Symbol
    const inputs::Dict{Symbol,Int}
    const devices::Dict{Symbol,Int}
end

function TwoBandSDR(
    ::Type{C},
    n_channels;
    capacity = 4096,
    num_rf_inputs = 2,
    num_antennas = 1,
    bands = collect(MB_BANDS),
    band_gains = Dict(:L1 => 1.0, :B1I => 1.0),
    clock_sync = :single_device,
    inputs = Dict(:L1 => 1, :B1I => 2),
    devices = Dict(:L1 => 1, :B1I => 1),
) where {C}
    capabilities = HardwareCorrelatorCapabilities(;
        signals = [:GPSL1CA, :BeiDouB1I],
        modulations = [:LOC],
        max_primary_code_length = 4096,
        code_frequency_limits = (0.5e6, 3.0e6),
        tap_layouts = [3],
        max_tap_offset_chips = 1.0,
        num_antennas,
        bands,
        num_rf_inputs,
        reports_code_phase = true,
    )
    TwoBandSDR{C}(
        PipeChannel{CorrelatorDump{C}}(capacity),
        PipeChannel{NCOUpdate}(capacity),
        n_channels,
        Any[],
        Int[],
        capabilities,
        band_gains,
        clock_sync,
        inputs,
        devices,
    )
end

GNSSReceiver.correlator_dump_channel(sdr::TwoBandSDR) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::TwoBandSDR) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::TwoBandSDR) = sdr.n_channels
GNSSReceiver.hardware_capabilities(sdr::TwoBandSDR) = sdr.capabilities
GNSSReceiver.correlator_gain(sdr::TwoBandSDR, band_id::Symbol) =
    get(sdr.band_gains, band_id, 1.0)
GNSSReceiver.band_rf_input(sdr::TwoBandSDR, band_id::Symbol) = sdr.inputs[band_id]
GNSSReceiver.band_device_index(sdr::TwoBandSDR, band_id::Symbol) = sdr.devices[band_id]
GNSSReceiver.clock_synchronization(sdr::TwoBandSDR) = sdr.clock_sync
GNSSReceiver.release_channel!(sdr::TwoBandSDR, hw_channel) = push!(sdr.released, hw_channel)
GNSSReceiver.assign_channel!(
    sdr::TwoBandSDR,
    hw_channel,
    config::GNSSReceiver.HardwareChannelConfig,
) = (push!(sdr.assigned, (; hw_channel, config)); nothing)

mb_epl(late, prompt, early) =
    EarlyPromptLateCorrelator(SVector{3,ComplexF64}(late, prompt, early), 1)
const MBEPL = typeof(mb_epl(0, 0, 0))

mb_dump(
    channel,
    prn,
    sample_index;
    prompt = 100.0 + 0im,
    integrated_samples = 4000,
    late = 40.0 + 0im,
    early = 40.0 + 0im,
    code_phase = NaN,
) = CorrelatorDump(
    channel,
    prn,
    CorrelatorOutput(mb_epl(late, prompt, early), integrated_samples, sample_index),
    code_phase,
)

# The two-band plan every test below runs on: L1 at 4 MS/s (the reference band,
# and therefore the receiver timebase) and BeiDou B1I at 5 MS/s.
const MB_FS = (; L1 = 4e6, B1I = 5e6)

# How much louder the B1I front end's noise floor is than L1's, in amplitude.
# Nothing about the two bands' gain chains, filters or interference environments
# is shared, and pooling their floors would describe neither.
const MB_B1I_NOISE_AMPLITUDE = sqrt(8.0)   # ≈ 9 dB in power

# The one-sided noise densities, in s (= 1/Hz), the two front ends are given.
const MB_N0_L1 = 1e-9
const MB_N0_B1I = 8e-9

mb_plan(sdr) = hardware_band_plan(sdr, MB_BANDS, (MB_FS.L1, MB_FS.B1I))

mb_link(sdr; kwargs...) = HardwareCorrelatorLink(
    sdr;
    sampling_freq = MB_FS.L1 * Hz,
    reference_signal = GPSL1CA(),
    band_plan = mb_plan(sdr),
    kwargs...,
)

# The two-band tracking state, band measurements and systems the link is driven
# with. `prn_l1` tracks GPS L1 C/A on :L1, `prn_b1` BeiDou B1I on :B1I.
function mb_track_state(; prn_l1 = 1, prn_b1 = 2, samples = 4000)
    track_state =
        TrackState(; signals = (GPSL1CA = (GPSL1CA(),), BeiDouB1I = (BeiDouB1I(),)))
    track_state = add_satellite!(
        track_state;
        prn = prn_l1,
        group = :GPSL1CA,
        code_phase = 0.0,
        carrier_doppler = 0.0Hz,
    )
    track_state = add_satellite!(
        track_state;
        prn = prn_b1,
        group = :BeiDouB1I,
        code_phase = 0.0,
        carrier_doppler = 0.0Hz,
    )
    band_systems = ((GPSL1CA(),), (BeiDouB1I(),))
    # Each band's frame carries its own noise floor — B1I's is 9 dB above L1's,
    # which is what the per-band noise reference has to keep apart.
    rng = Random.Xoshiro(0xB1B1)
    band_measurements = (;
        L1 = Tracking.BandMeasurement(
            randn(rng, ComplexF64, samples),
            MB_FS.L1 * Hz,
            0.0Hz,
        ),
        B1I = Tracking.BandMeasurement(
            MB_B1I_NOISE_AMPLITUDE .*
            randn(rng, ComplexF64, round(Int, samples * MB_FS.B1I / MB_FS.L1)),
            MB_FS.B1I * Hz,
            0.0Hz,
        ),
    )
    track_state, band_systems, band_measurements
end

@testset "A band plan maps every band's counter onto one receiver timebase" begin
    sdr = TwoBandSDR(MBEPL, 8)
    plan = mb_plan(sdr)

    # The first band is the reference: its counter *is* the receiver timebase.
    @test reference_band(plan) === :L1
    @test band_sampling_frequency(plan, :L1) == 4e6
    @test band_sampling_frequency(plan, :B1I) == 5e6
    @test receiver_timebase_scale(plan, :L1) == 1.0
    @test receiver_timebase_scale(plan, :B1I) == 4e6 / 5e6

    # One millisecond is 4000 counts on L1 and 5000 on B1I, and both map onto
    # the same instant of the receiver timebase.
    @test to_receiver_samples(plan, :B1I, 5000) == 4000
    @test to_receiver_samples(plan, :L1, 4000) == 4000
    @test to_band_samples(plan, :B1I, 4000) == 5000
    # Round trip, so a handover time and a feedback time cannot drift apart.
    @test to_band_samples(plan, :B1I, to_receiver_samples(plan, :B1I, 123_455)) == 123_455

    # Routing: each band names the RF input and device it arrives on.
    @test band_rf_input(sdr, :L1) == 1
    @test band_rf_input(sdr, :B1I) == 2
    @test band_device_index(sdr, :B1I) == 1
    @test clock_synchronization(sdr) === :single_device
end

@testset "Epochs close on the receiver timebase, not on the fastest counter" begin
    sdr = TwoBandSDR(MBEPL, 8)
    link = mb_link(sdr)
    track_state, band_systems, band_measurements = mb_track_state()
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)

    l1 = link.channel_of[HardwareChannelAssignment(:GPSL1CA, 1, 1)]
    b1 = link.channel_of[HardwareChannelAssignment(:BeiDouB1I, 2, 1)]

    # 100 ms of both bands, each on its own counter, drained a millisecond at a
    # time as a live run does. The B1I counter reaches 500 000 where L1's
    # reaches 400 000; if the grid rode the raw indices the clock would run
    # 25 % fast and every L1 record would be permanently "in the past".
    for ms = 1:100
        put!(
            sdr.dumps,
            [
                mb_dump(l1, 1, 4000 * ms; integrated_samples = 4000),
                mb_dump(b1, 2, 5000 * ms; integrated_samples = 5000),
            ],
        )
        GNSSReceiver.drain_dumps!(link)
        GNSSReceiver.fold_closed_epochs!(link, track_state, band_measurements, band_systems)
    end

    # The clock sits at 100 ms of the receiver timebase, whichever band's
    # counter got there, and the grid advanced one epoch per millisecond — not
    # 125, and with nothing skipped.
    @test link.latest_sample_index == 400_000
    @test link.next_epoch_boundary == 404_000
    @test link.skipped_epochs == 0
    @test link.stale_dumps == 0
    # Both bands' records reached their satellites, and neither band's records
    # were counted as lost or overlapping because of the other band's counter.
    @test link.lost_record_gaps == 0
    @test link.overlapping_record_samples[l1] == 0
    @test link.overlapping_record_samples[b1] == 0
    # A B1I record spans one primary code period at 5 MS/s (2046 chips at
    # 2.046 Mcps = 1 ms = 5000 samples); measured with the reference band's rate
    # it would read 0.8 of one and the block grid would never wrap. Both bands
    # completed 99 of their 100 records — the hundredth sits exactly on the open
    # boundary and belongs to the epoch that has not closed.
    @test GNSSReceiver.primary_code_wraps(link, b1) == 99
    @test GNSSReceiver.primary_code_wraps(link, l1) == 99
end

@testset "NCO feedback lands on the channel's own counter" begin
    sdr = TwoBandSDR(MBEPL, 8)
    link = mb_link(sdr; feedback_delay_epochs = 2)
    track_state, band_systems, band_measurements = mb_track_state()
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)

    l1 = link.channel_of[HardwareChannelAssignment(:GPSL1CA, 1, 1)]
    b1 = link.channel_of[HardwareChannelAssignment(:BeiDouB1I, 2, 1)]

    for ms = 1:3
        put!(
            sdr.dumps,
            [
                mb_dump(l1, 1, 4000 * ms; integrated_samples = 4000),
                mb_dump(b1, 2, 5000 * ms; integrated_samples = 5000),
            ],
        )
        GNSSReceiver.drain_dumps!(link)
        GNSSReceiver.fold_closed_epochs!(link, track_state, band_measurements, band_systems)
    end

    @test Base.n_avail(sdr.ncos) > 0
    updates = Vector{NCOUpdate}(undef, Base.n_avail(sdr.ncos))
    take!(sdr.ncos, updates)
    by_channel = Dict(Int(u.channel) => u for u in updates)  # last fold wins
    @test haskey(by_channel, l1) && haskey(by_channel, b1)
    # Both corrections apply at the *same instant*, which is a different integer
    # on each band's counter: 8000 more L1 counts and 10 000 more B1I counts.
    @test by_channel[b1].apply_at_sample ==
          round(Int64, by_channel[l1].apply_at_sample * 5 / 4)
    # The fold's landing instant is decided once, on the receiver timebase — so
    # it *is* the reference band's integer, and any other band's channel has to
    # be given the converted one. A delay-aware estimator reads it the same way
    # (`GNSSReceiver._band_sample`), because it compares it against records whose
    # `sample_index` is on that band's counter.
    @test link.scheduled_apply_at_sample == by_channel[l1].apply_at_sample
    @test GNSSReceiver._band_sample(link, b1, link.scheduled_apply_at_sample) ==
          by_channel[b1].apply_at_sample
end

@testset "Handovers carry the band, its RF input and its own sample rate" begin
    sdr = TwoBandSDR(MBEPL, 8)
    link = mb_link(sdr)
    track_state, band_systems, band_measurements = mb_track_state()
    # A non-zero host sample count, so a converted handover time is visible.
    link.samples_consumed = 40_000    # 10 ms on the receiver timebase
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)

    # `signal_index == 0` is a band's open-loop noise reference, not a satellite.
    sats = filter(a -> a.config.signal_index == 1, sdr.assigned)
    l1 = only(filter(a -> a.config.prn == 1, sats)).config
    b1 = only(filter(a -> a.config.prn == 2, sats)).config

    @test l1.band_id === :L1
    @test b1.band_id === :B1I
    @test l1.sampling_freq == 4e6
    @test b1.sampling_freq == 5e6
    @test l1.rf_input == 1
    @test b1.rf_input == 2
    @test l1.device_index == 1 && b1.device_index == 1
    # The handover instant is one instant, expressed on each band's own counter.
    @test l1.valid_at_sample == 40_000
    @test b1.valid_at_sample == 50_000
    # …and the replica offsets are quantised at that band's *own* rate: half a
    # chip of GPS L1 C/A is 1.955 samples at 4 MS/s and half a chip of B1I is
    # 1.222 at 5 MS/s, so each channel's Early-to-Late distance follows from the
    # rate the band is really counted at rather than from the reference band's.
    @test l1.el_sample_spacing == Int(
        Tracking.get_early_late_sample_spacing(
            get_default_correlator(GPSL1CA()),
            4e6,
            ustrip(Hz, get_code_frequency(GPSL1CA())),
        ),
    )
    @test b1.el_sample_spacing == Int(
        Tracking.get_early_late_sample_spacing(
            get_default_correlator(BeiDouB1I()),
            5e6,
            ustrip(Hz, get_code_frequency(BeiDouB1I())),
        ),
    )
    @test l1.tap_sample_shifts[begin] == -l1.tap_sample_shifts[end]
    @test b1.tap_sample_shifts[begin] == -b1.tap_sample_shifts[end]
end

@testset "Each band keeps its own noise reference — floors are never pooled" begin
    # The two front ends differ by 9 dB in noise power. One reference per band
    # has to report that difference; one pooled reference reports a number that
    # is wrong on both bands, and a C/N₀ has nothing downstream to contradict it.
    sdr = TwoBandSDR(MBEPL, 8)
    link = mb_link(sdr)                     # `noise_source = :channel`, the default
    track_state, band_systems, band_measurements = mb_track_state()
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)

    # One open-loop reference per band, on distinct channels, each routed to its
    # own band's rate.
    @test length(link.noise_channels) == 2
    @test all(>(0), link.noise_channels)
    @test allunique(link.noise_channels)
    l1_ref, b1_ref = link.noise_channels
    @test GNSSReceiver.channel_band_id(link, l1_ref) === :L1
    @test GNSSReceiver.channel_band_id(link, b1_ref) === :B1I
    @test GNSSReceiver.channel_sampling_frequency(link, b1_ref) == 5e6
    # …armed on a signal of its own band.
    refs = filter(a -> a.config.signal_index == 0, sdr.assigned)
    @test Set(r.config.band_id for r in refs) == Set((:L1, :B1I))
    @test only(filter(r -> r.config.band_id === :B1I, refs)).config.rf_input == 2

    # Feed each reference a floor of its own. A correlator accumulator over `N`
    # samples of white noise at rate `fs` and one-sided density `N₀` has expected
    # power `N·fs·N₀`, so the tap amplitude that *means* a given density differs
    # between the bands — which is exactly why a floor cannot be carried across
    # one.
    tap_amplitude(density, samples, fs) = sqrt(density * samples * fs)
    l1_amp = tap_amplitude(MB_N0_L1, 4000, MB_FS.L1)
    b1_amp = tap_amplitude(MB_N0_B1I, 5000, MB_FS.B1I)
    for ms = 1:3
        put!(
            sdr.dumps,
            [
                mb_dump(
                    l1_ref,
                    link.noise_prns[1],
                    4000 * ms;
                    integrated_samples = 4000,
                    prompt = complex(l1_amp),
                    late = complex(l1_amp),
                    early = complex(l1_amp),
                ),
                mb_dump(
                    b1_ref,
                    link.noise_prns[2],
                    5000 * ms;
                    integrated_samples = 5000,
                    prompt = complex(b1_amp),
                    late = complex(b1_amp),
                    early = complex(b1_amp),
                ),
            ],
        )
        GNSSReceiver.drain_dumps!(link)
        GNSSReceiver.fold_closed_epochs!(link, track_state, band_measurements, band_systems)
    end

    l1_density = Tracking.get_noise_density(track_state.noise_estimators[:GPSL1CA])
    b1_density = Tracking.get_noise_density(track_state.noise_estimators[:BeiDouB1I])
    @test !isnothing(l1_density)
    @test !isnothing(b1_density)
    # Each band recovers *its own* floor, to the sample: `N₀ = Σ|b|² / (M·N·fs)`
    # with that band's record length and sample rate. A single pooled reference
    # would hand both bands one number somewhere between the two, and every
    # satellite's C/N₀ on both bands would be wrong by the difference with
    # nothing downstream able to see it.
    @test ustrip(u"s", l1_density) ≈ MB_N0_L1 rtol = 1e-9
    @test ustrip(u"s", b1_density) ≈ MB_N0_B1I rtol = 1e-9
    @test ustrip(u"s", b1_density) / ustrip(u"s", l1_density) ≈ 8 rtol = 1e-9
end

@testset "An antenna array is not a second band" begin
    # A device with four coherent antenna chains on one RF input can serve a
    # four-antenna L1 receiver and still not receive a second band; the two
    # declarations are independent and neither stands in for the other.
    sdr = TwoBandSDR(MBEPL, 8; num_rf_inputs = 1, num_antennas = 4)
    single = hardware_band_plan(sdr, (:L1,), (4e6,))
    @test isnothing(GNSSReceiver.band_plan_error(sdr.capabilities, single))

    both = hardware_band_plan(sdr, MB_BANDS, (MB_FS.L1, MB_FS.B1I))
    message = GNSSReceiver.band_plan_error(sdr.capabilities, both)
    @test !isnothing(message)
    @test occursin("1 RF input", message)
    @test occursin("num_antennas", message)          # named, and refused as a stand-in
    @test occursin("Sequential retuning", message)   # and the alternative is spelled out

    # A one-input device that does declare two bands is still refused, because
    # nothing tells the receiver which tuner each band lands on.
    one_input = TwoBandSDR(MBEPL, 8; num_rf_inputs = 2, inputs = Dict(:L1 => 1, :B1I => 1))
    clash = GNSSReceiver.band_plan_error(
        one_input.capabilities,
        hardware_band_plan(one_input, MB_BANDS, (MB_FS.L1, MB_FS.B1I)),
    )
    @test !isnothing(clash)
    @test occursin("share one RF input", clash)
    @test occursin("band_rf_input", clash)
end

@testset "An unsupported RF configuration is refused before anything is armed" begin
    # More bands than tuners.
    narrow = TwoBandSDR(MBEPL, 8; num_rf_inputs = 1)
    err = try
        validate_hardware_configuration(
            narrow,
            ((GPSL1CA(),), (BeiDouB1I(),)),
            (MB_FS.L1, MB_FS.B1I);
            band_plan = hardware_band_plan(narrow, MB_BANDS, (MB_FS.L1, MB_FS.B1I)),
        )
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("RF input", err.msg)
    @test isempty(narrow.assigned)

    # A band the front end cannot tune at all.
    l1_only = TwoBandSDR(MBEPL, 8; bands = [:L1])
    err = try
        validate_hardware_configuration(
            l1_only,
            ((GPSL1CA(),), (BeiDouB1I(),)),
            (MB_FS.L1, MB_FS.B1I);
            band_plan = hardware_band_plan(l1_only, MB_BANDS, (MB_FS.L1, MB_FS.B1I)),
        )
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("cannot tune band", err.msg)

    # Two devices on free-running clocks: no common timebase exists, so there is
    # no common reception epoch and the configuration is refused rather than
    # producing a fix nothing can vouch for.
    split =
        TwoBandSDR(MBEPL, 8; devices = Dict(:L1 => 1, :B1I => 2), clock_sync = :independent)
    err = try
        validate_hardware_configuration(
            split,
            ((GPSL1CA(),), (BeiDouB1I(),)),
            (MB_FS.L1, MB_FS.B1I);
            band_plan = hardware_band_plan(split, MB_BANDS, (MB_FS.L1, MB_FS.B1I)),
        )
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin(":independent", err.msg)
    @test occursin(":shared_clock", err.msg)

    # …and the same two devices on one distributed sample clock are accepted.
    shared = TwoBandSDR(
        MBEPL,
        8;
        devices = Dict(:L1 => 1, :B1I => 2),
        clock_sync = :shared_clock,
    )
    @test isnothing(
        validate_hardware_configuration(
            shared,
            ((GPSL1CA(),), (BeiDouB1I(),)),
            (MB_FS.L1, MB_FS.B1I);
            band_plan = hardware_band_plan(shared, MB_BANDS, (MB_FS.L1, MB_FS.B1I)),
        ),
    )

    # A link whose `sampling_freq` is not the reference band's rate has no
    # timebase at all, and says so rather than scaling everything by 1.25.
    @test_throws ArgumentError HardwareCorrelatorLink(
        TwoBandSDR(MBEPL, 8);
        sampling_freq = MB_FS.B1I * Hz,
        reference_signal = GPSL1CA(),
        band_plan = hardware_band_plan(
            TwoBandSDR(MBEPL, 8),
            MB_BANDS,
            (MB_FS.L1, MB_FS.B1I),
        ),
    )
end

@testset "Both bands' satellites are referenced to one reception epoch" begin
    # The pseudoranges of a multi-band fix are differences of code phases taken
    # at *one* instant. With two counters that instant is two integers, so the
    # extrapolation has to be done on each band's own axis — otherwise the B1I
    # satellite's phase is advanced by 25 % too little and its pseudorange is
    # metres out with nothing else looking wrong.
    sdr = TwoBandSDR(MBEPL, 8)
    link = mb_link(sdr)
    track_state, band_systems, band_measurements = mb_track_state()
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)
    l1 = link.channel_of[HardwareChannelAssignment(:GPSL1CA, 1, 1)]
    b1 = link.channel_of[HardwareChannelAssignment(:BeiDouB1I, 2, 1)]

    # Both channels report a replica phase of exactly zero one millisecond in,
    # then go quiet: the phase bookkeeping has to dead reckon both to the common
    # boundary from there.
    put!(
        sdr.dumps,
        [
            mb_dump(l1, 1, 4000; integrated_samples = 4000, code_phase = 0.0),
            mb_dump(b1, 2, 5000; integrated_samples = 5000, code_phase = 0.0),
        ],
    )
    GNSSReceiver.drain_dumps!(link)
    put!(sdr.dumps, [epoch_strobe(mb_epl(0, 0, 0), 8000)])
    GNSSReceiver.drain_dumps!(link)
    GNSSReceiver.fold_closed_epochs!(link, track_state, band_measurements, band_systems)

    # Each channel's phase reference is the same instant on its own counter.
    @test link.phase_ref_sample[l1] == 8000
    @test link.phase_ref_sample[b1] == 10_000
    # And each satellite's code phase is exactly one code period past its
    # anchor: 1 ms of GPS L1 C/A is 1023 chips, 1 ms of B1I is 2046.
    l1_phase = get_code_phase(get_sat_state(track_state, :GPSL1CA, 1))
    b1_phase = get_code_phase(get_sat_state(track_state, :BeiDouB1I, 2))
    # (`rem(…, RoundNearest)` because a phase on the wrap reads either as zero or
    # as just under the code length.)
    @test rem(l1_phase, get_code_length(GPSL1CA()), RoundNearest) ≈ 0.0 atol = 1e-6
    @test rem(b1_phase, get_code_length(BeiDouB1I()), RoundNearest) ≈ 0.0 atol = 1e-6
end

@testset "A fault on one band does not reach the other" begin
    sdr = TwoBandSDR(MBEPL, 8)
    link = mb_link(sdr)
    track_state, band_systems, band_measurements = mb_track_state()
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)
    l1 = link.channel_of[HardwareChannelAssignment(:GPSL1CA, 1, 1)]
    b1 = link.channel_of[HardwareChannelAssignment(:BeiDouB1I, 2, 1)]

    # Three clean milliseconds on both bands.
    for ms = 1:3
        put!(
            sdr.dumps,
            [
                mb_dump(l1, 1, 4000 * ms; integrated_samples = 4000),
                mb_dump(b1, 2, 5000 * ms; integrated_samples = 5000),
            ],
        )
        GNSSReceiver.drain_dumps!(link)
        GNSSReceiver.fold_closed_epochs!(link, track_state, band_measurements, band_systems)
    end

    # Now the B1I band's ring overruns: its records for milliseconds 4 and 5
    # never reach the host. L1 keeps streaming.
    for ms = 4:8
        records = [mb_dump(l1, 1, 4000 * ms; integrated_samples = 4000)]
        ms in (4, 5) || push!(records, mb_dump(b1, 2, 5000 * ms; integrated_samples = 5000))
        put!(sdr.dumps, records)
        GNSSReceiver.drain_dumps!(link)
        GNSSReceiver.fold_closed_epochs!(link, track_state, band_measurements, band_systems)
    end

    # The hole is charged to B1I's channel alone, and B1I's bit clock — not
    # L1's — is the one restarted.
    @test link.lost_record_samples[b1] == 2 * 5000
    @test link.lost_record_samples[l1] == 0
    @test link.rearm_dead_samples[l1] == 0
    @test any(a -> a.group_key === :BeiDouB1I, link.bit_clock_restarts)
    @test !any(a -> a.group_key === :GPSL1CA, link.bit_clock_restarts)
    # L1's record stream is intact: every millisecond it produced completed a
    # code period, and none of them overlapped.
    @test GNSSReceiver.primary_code_wraps(link, l1) == 7
    @test link.overlapping_record_samples[l1] == 0
    @test link.stale_dumps == 0
end

@testset "Backpressure on the feedback ring is reported, not silently absorbed" begin
    # A device whose NCO writer is not keeping up: the ring holds one update and
    # two channels want one each. The chunk's corrections are dropped as a whole
    # and counted — never half-applied, which would leave one band steered and
    # the other free-running with nothing recording the difference.
    sdr = TwoBandSDR(MBEPL, 8; capacity = 2)
    link = mb_link(sdr)
    track_state, band_systems, band_measurements = mb_track_state()
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)
    l1 = link.channel_of[HardwareChannelAssignment(:GPSL1CA, 1, 1)]
    b1 = link.channel_of[HardwareChannelAssignment(:BeiDouB1I, 2, 1)]
    for ms = 1:3
        put!(
            sdr.dumps,
            [
                mb_dump(l1, 1, 4000 * ms; integrated_samples = 4000),
                mb_dump(b1, 2, 5000 * ms; integrated_samples = 5000),
            ],
        )
        GNSSReceiver.drain_dumps!(link)
        GNSSReceiver.fold_closed_epochs!(link, track_state, band_measurements, band_systems)
    end
    @test link.dropped_nco_updates > 0
    # Both channels kept folding records through it — the loop was open, not
    # broken, and the accounting says which.
    @test GNSSReceiver.primary_code_wraps(link, l1) == 2
    @test GNSSReceiver.primary_code_wraps(link, b1) == 2
end

# ─────────────────────────────────────────────────────────────────────────────
# A two-band simulated hardware correlator, and the whole receiver over it.
#
# Two independent front ends at *different* sample rates, each correlating its
# own band's samples with its own replicas on its own free-running counter, and
# one host folding both onto one timebase. Built by composing two of
# test/simulated_fpga.jl's `SimulatedFPGA`s rather than writing a second
# simulator: each one is a band, and the outer device is only the routing —
# which is precisely what this step adds.
#
# `SimulatedFPGA`, `sim_epl` and `SimEPL` come from test/hardware_correlator.jl,
# which runtests.jl includes first; the guard below lets this file be run on its
# own.
# ─────────────────────────────────────────────────────────────────────────────

isdefined(@__MODULE__, :SimulatedFPGA) || include("simulated_fpga.jl")

using .ReferenceHarness: ReferenceCase, SampleSource, next_samples!

mutable struct TwoBandSimulatedFPGA <: AbstractHardwareCorrelatorSDR
    const band_ids::Vector{Symbol}
    const engines::Vector{Any}          # one `SimulatedFPGA` per band
    const offsets::Vector{Int}          # global channel index = local + offset
    const dumps::PipeChannel{CorrelatorDump{SimEPL}}
    const ncos::PipeChannel{NCOUpdate}
    const configs::Vector{Any}          # every `HardwareChannelConfig` armed
    const scratch::Vector{CorrelatorDump{SimEPL}}
end

function TwoBandSimulatedFPGA(engines, band_ids; capacity = 1 << 16)
    offsets = cumsum([0; [length(e.channels) for e in engines[1:(end-1)]]])
    TwoBandSimulatedFPGA(
        collect(Symbol, band_ids),
        collect(Any, engines),
        offsets,
        PipeChannel{CorrelatorDump{SimEPL}}(capacity),
        PipeChannel{NCOUpdate}(1 << 12),
        Any[],
        CorrelatorDump{SimEPL}[],
    )
end

mb_band_index(sdr::TwoBandSimulatedFPGA, band_id::Symbol) =
    findfirst(==(band_id), sdr.band_ids)
# Which band a global channel index belongs to, and its index inside that band.
function mb_local_channel(sdr::TwoBandSimulatedFPGA, hw_channel::Integer)
    band = searchsortedlast(sdr.offsets, hw_channel - 1)
    band, hw_channel - sdr.offsets[band]
end

GNSSReceiver.correlator_dump_channel(sdr::TwoBandSimulatedFPGA) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::TwoBandSimulatedFPGA) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::TwoBandSimulatedFPGA) =
    sum(e -> length(e.channels), sdr.engines)
GNSSReceiver.raw_sample_channel(sdr::TwoBandSimulatedFPGA) = first(sdr.engines).raw
GNSSReceiver.raw_sample_channel(sdr::TwoBandSimulatedFPGA, band_id::Symbol) =
    sdr.engines[mb_band_index(sdr, band_id)].raw
GNSSReceiver.band_rf_input(sdr::TwoBandSimulatedFPGA, band_id::Symbol) =
    mb_band_index(sdr, band_id)
# Each band's replica sets are the engine's own: a channel of one band's bank
# cannot see the other band's samples at all.
function GNSSReceiver.band_hardware_channels(sdr::TwoBandSimulatedFPGA, band_id::Symbol)
    band = mb_band_index(sdr, band_id)
    sdr.offsets[band] .+ (1:length(sdr.engines[band].channels))
end
GNSSReceiver.clock_synchronization(::TwoBandSimulatedFPGA) = :single_device

function GNSSReceiver.hardware_capabilities(sdr::TwoBandSimulatedFPGA)
    signals = [get_signal_id(e.system) for e in sdr.engines]
    freqs = [ustrip(Hz, uconvert(Hz, get_code_frequency(e.system))) for e in sdr.engines]
    HardwareCorrelatorCapabilities(;
        signals,
        modulations = unique(nameof(typeof(get_modulation(e.system))) for e in sdr.engines),
        max_primary_code_length = maximum(get_code_length(e.system) for e in sdr.engines),
        code_frequency_limits = (minimum(freqs), maximum(freqs)),
        tap_layouts = [3],
        max_tap_offset_chips = 1.0,
        num_antennas = 1,
        bands = copy(sdr.band_ids),
        num_rf_inputs = length(sdr.engines),
        reports_code_phase = true,
    )
end

function GNSSReceiver.assign_channel!(
    sdr::TwoBandSimulatedFPGA,
    hw_channel,
    config::GNSSReceiver.HardwareChannelConfig,
)
    band, local_channel = mb_local_channel(sdr, hw_channel)
    push!(sdr.configs, config)
    GNSSReceiver.assign_channel!(
        sdr.engines[band],
        local_channel,
        config.prn,
        config.carrier_doppler * Hz,
        config.code_doppler * Hz,
        config.code_phase,
        config.valid_at_sample;
        el_sample_spacing = config.el_sample_spacing,
        signal = config.signal,
    )
end

function GNSSReceiver.release_channel!(sdr::TwoBandSimulatedFPGA, hw_channel)
    band, local_channel = mb_local_channel(sdr, hw_channel)
    GNSSReceiver.release_channel!(sdr.engines[band], local_channel)
end

"""
    mb_pump!(sdr, band, samples)

Correlate one band's chunk on its own engine, republish its records on the
shared dump stream under their *global* channel numbers, and hand the samples to
that band's raw stream.

Two contract points are enforced here rather than assumed. NCO updates are
demultiplexed to the engine that owns the channel — each one's
`apply_at_sample` is already stated on that band's counter, which is what the
engine compares against its own. And only the **reference** band's epoch strobes
are forwarded: a strobe is the timebase marker, so it is stated in the timebase.
"""
function mb_pump!(sdr::TwoBandSimulatedFPGA, band::Integer, samples)
    engine = sdr.engines[band]
    # Demultiplex the host's feedback to the engines.
    while Base.n_avail(sdr.ncos) > 0
        update = take!(sdr.ncos)
        target, local_channel = mb_local_channel(sdr, Int(update.channel))
        target == band || continue
        put!(
            engine.ncos,
            NCOUpdate(
                local_channel,
                update.prn,
                update.carrier_doppler * Hz,
                update.code_doppler * Hz,
                update.apply_at_sample,
            ),
        )
    end
    produced = correlate_chunk!(engine, samples)
    # `correlate_chunk!` also pushes onto the engine's own ring; drain it so it
    # cannot fill, and republish under global channel numbers instead.
    while Base.n_avail(engine.dumps) > 0
        take!(engine.dumps)
    end
    empty!(sdr.scratch)
    offset = sdr.offsets[band]
    for dump in produced
        if is_epoch_strobe(dump)
            band == 1 && push!(sdr.scratch, dump)
            continue
        end
        push!(
            sdr.scratch,
            CorrelatorDump(
                Int(dump.channel) + offset,
                dump.prn,
                dump.output,
                dump.code_phase,
                dump.num_taps,
            ),
        )
    end
    isempty(sdr.scratch) || put!(sdr.dumps, copy(sdr.scratch))
    sdr
end

@testset "Two bands at different sample rates through the whole receiver" begin
    # GPS L1 C/A on :L1 at 4 MS/s — the regression baseline — alongside BeiDou
    # B1I on :B1I at 5 MS/s. Two front ends, two sample clocks' worth of
    # counters, two noise floors, one receiver.
    l1_fs, b1_fs = 4e6, 5e6
    l1_chunk, b1_chunk = 4000, 5000            # 1 ms of each band
    seconds = 1.2
    num_chunks = round(Int, seconds * 1000)
    l1_prn, b1_prn = 11, 6

    l1_case = ReferenceCase(
        GPSL1CA();
        prn = l1_prn,
        sampling_freq = l1_fs,
        carrier_doppler = 1200.0,
        code_phase = 137.4,
        cn0_dbhz = 48,
        seed = 0xC0FFEE,
    )
    # A louder front end on the second band: 6 dB more noise power, which is
    # what the per-band noise references have to keep apart.
    b1_case = ReferenceCase(
        BeiDouB1I();
        prn = b1_prn,
        sampling_freq = b1_fs,
        carrier_doppler = -800.0,
        code_phase = 512.7,
        cn0_dbhz = 50,
        noise_power = 4.0,
        seed = 0xBEEF,
    )

    engines = [
        SimulatedFPGA(
            GPSL1CA();
            sampling_freq = l1_fs,
            chunk = l1_chunk,
            n_channels = 6,
            epoch_length = l1_chunk,
            handover_code_phase_error = 0.25,
        ),
        SimulatedFPGA(
            BeiDouB1I();
            sampling_freq = b1_fs,
            chunk = b1_chunk,
            n_channels = 6,
            epoch_length = b1_chunk,
        ),
    ]
    sdr = TwoBandSimulatedFPGA(engines, (:L1, :B1I))

    producer = Threads.@spawn begin
        l1_source = SampleSource(l1_case)
        b1_source = SampleSource(b1_case)
        try
            for _ = 1:num_chunks
                l1 = next_samples!(l1_source, l1_chunk)
                b1 = next_samples!(b1_source, b1_chunk)
                # Tap first, exactly as the gateware's observer sees a word only
                # once DMA accepted it, then hand each band to its own host
                # stream.
                mb_pump!(sdr, 1, view(l1, :, 1))
                mb_pump!(sdr, 2, view(b1, :, 1))
                put!(engines[1].raw, l1)
                put!(engines[2].raw, b1)
            end
        finally
            close(engines[1].raw)
            close(engines[2].raw)
        end
    end
    Base.errormonitor(producer)

    plan = hardware_band_plan(sdr, (:L1, :B1I), (l1_fs, b1_fs))
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = l1_fs * Hz,
        reference_signal = GPSL1CA(),
        band_plan = plan,
    )
    data_channel = receive(
        sdr,
        ((GPSL1CA(),), (BeiDouB1I(),)),
        (l1_fs * Hz, b1_fs * Hz);
        link,
        band_plan = plan,
        acquire_async = false,
        acquire_every = 20ms,
        prns = [l1_prn, b1_prn],
        # Synthetic signals carry no navigation message, so a fix can never
        # converge; the routing is what this test is about.
        time_in_lock_before_calculating_pvt = 1000u"s",
    )
    results = collect_data(data_channel)
    wait(producer)

    @test !isempty(results)

    # 1. Routing. Each band's satellite was armed on that band's RF input, at
    #    that band's sample rate, with that band's Early-to-Late quantisation.
    sats = filter(c -> c.signal_index == 1, sdr.configs)
    l1_cfg = first(filter(c -> c.band_id === :L1, sats))
    b1_cfg = first(filter(c -> c.band_id === :B1I, sats))
    @test l1_cfg.prn == l1_prn
    @test b1_cfg.prn == b1_prn
    @test l1_cfg.sampling_freq == l1_fs
    @test b1_cfg.sampling_freq == b1_fs
    @test l1_cfg.rf_input == 1
    @test b1_cfg.rf_input == 2
    @test l1_cfg.el_sample_spacing == Int(
        get_early_late_sample_spacing(
            EarlyPromptLateCorrelator(num_ants = NumAnts(1)),
            l1_fs * Hz,
            get_code_frequency(GPSL1CA()),
        ),
    )
    @test b1_cfg.el_sample_spacing == Int(
        get_early_late_sample_spacing(
            EarlyPromptLateCorrelator(num_ants = NumAnts(1)),
            b1_fs * Hz,
            get_code_frequency(BeiDouB1I()),
        ),
    )
    # …and each handover reached the engine that owns that band.
    @test any(h -> h.prn == l1_prn, engines[1].handovers)
    @test any(h -> h.prn == b1_prn, engines[2].handovers)

    # 2. Timestamps. Both bands' records folded onto one grid: the epoch clock
    #    tracks the *reference* band's counter and the run's epochs are the run's
    #    milliseconds, not the faster band's.
    @test link.skipped_epochs == 0
    @test abs(link.latest_sample_index - engines[1].sample_count) <= link.epoch_length
    @test engines[2].sample_count > engines[1].sample_count   # 5 MS/s vs 4 MS/s

    # 3. Both bands' channels really produced folded records.
    l1_channel = link.channel_of[HardwareChannelAssignment(:GPSL1CA, l1_prn, 1)]
    b1_channel = link.channel_of[HardwareChannelAssignment(:BeiDouB1I, b1_prn, 1)]
    @test GNSSReceiver.channel_band_id(link, l1_channel) === :L1
    @test GNSSReceiver.channel_band_id(link, b1_channel) === :B1I
    @test GNSSReceiver.primary_code_wraps(link, l1_channel) > 100
    @test GNSSReceiver.primary_code_wraps(link, b1_channel) > 100

    # 4. Noise. Each band spends a hardware channel of its *own* front end on an
    #    open-loop despread, armed on a signal of that band at that band's rate.
    #    (What the two references then measure is pinned to the sample in the
    #    per-band noise testset above.)
    @test length(link.noise_channels) == 2
    @test allunique(link.noise_channels)
    refs = filter(c -> c.signal_index == 0, sdr.configs)
    @test Set(r.band_id for r in refs) == Set((:L1, :B1I))
    @test all(r -> r.sampling_freq == (r.band_id === :L1 ? l1_fs : b1_fs), refs)
    @test all(r -> r.rf_input == (r.band_id === :L1 ? 1 : 2), refs)

    # 5. GPS L1 C/A stays the regression baseline: a second band at another
    #    sample rate alongside it must not cost it its lock.
    key = (:GPSL1CA, l1_prn)
    final = last(results)
    @test haskey(final.sat_data, key)
    @test final.sat_data[key].cn0 > 35dBHz
    @test !isempty(engines[1].applied)
    @test last(engines[1].applied).carrier_doppler ≈ 1200.0 atol = 15.0
    @test link.dropped_nco_updates == 0
end
