# Tests for the hardware-correlator capability query, pre-arm validation and the
# complete correlator configuration (issue #131).
#
# The hardware path shipped with #107/#129 assumed every device was the one it
# was written against: GPS L1 C/A, three taps, a 1023-chip code. These tests pin
# what a device is now allowed to be, what it has to declare, and what the link
# does with the declaration — most importantly that a request the device cannot
# serve is refused *before* a channel is armed, and that a link can carry
# three-tap and five-tap records at the same time without inventing taps.
#
# `RecordingSDR`, `epl` and `EPL` come from test/hardware_correlator.jl, which
# runtests.jl includes first.

using GNSSReceiver:
    HardwareCorrelatorCapabilities,
    HardwareChannelConfig,
    LEGACY_GPS_L1CA_CAPABILITIES,
    check_hardware_support,
    hardware_capabilities,
    hardware_support_error,
    num_correlator_taps,
    replica_code_amplitude,
    supports_secondary_code_wipeoff,
    validate_hardware_configuration

using Tracking:
    VeryEarlyPromptLateCorrelator,
    get_correlator_outputs,
    get_default_correlator,
    get_num_accumulators

# The five-slot wire record a device that also serves BOC signals dumps.
vepl(very_late, late, prompt, early, very_early) = VeryEarlyPromptLateCorrelator(
    SVector{5,ComplexF64}(very_late, late, prompt, early, very_early),
    0.15,
    0.6,
)
const VEPL = typeof(vepl(0, 0, 0, 0, 0))

# A device that declares what it can do, implements the *new* assignment form
# directly (no legacy shim), and can be given per-band replica gains and
# per-signal replica code amplitudes.
struct CapableSDR{C} <: AbstractHardwareCorrelatorSDR
    dumps::PipeChannel{CorrelatorDump{C}}
    ncos::PipeChannel{NCOUpdate}
    n_channels::Int
    caps::HardwareCorrelatorCapabilities
    band_gains::Dict{Symbol,Float64}
    code_amplitudes::Dict{Symbol,Float64}
    assigned::Vector{HardwareChannelConfig}
    released::Vector{Int}
end

CapableSDR(
    ::Type{C},
    n_channels,
    caps;
    band_gains = Dict{Symbol,Float64}(),
    code_amplitudes = Dict{Symbol,Float64}(),
    capacity = 256,
) where {C} = CapableSDR{C}(
    PipeChannel{CorrelatorDump{C}}(capacity),
    PipeChannel{NCOUpdate}(capacity),
    n_channels,
    caps,
    band_gains,
    code_amplitudes,
    HardwareChannelConfig[],
    Int[],
)

GNSSReceiver.correlator_dump_channel(sdr::CapableSDR) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::CapableSDR) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::CapableSDR) = sdr.n_channels
GNSSReceiver.hardware_capabilities(sdr::CapableSDR) = sdr.caps
GNSSReceiver.release_channel!(sdr::CapableSDR, hw_channel) = push!(sdr.released, hw_channel)
GNSSReceiver.correlator_gain(sdr::CapableSDR, band_id::Symbol) =
    get(sdr.band_gains, band_id, 1.0)
GNSSReceiver.replica_code_amplitude(sdr::CapableSDR, signal::AbstractGNSSSignal) =
    get(sdr.code_amplitudes, get_signal_id(signal), get_code_amplitude(signal))
function GNSSReceiver.assign_channel!(
    sdr::CapableSDR,
    hw_channel,
    config::HardwareChannelConfig,
)
    push!(sdr.assigned, config)
    nothing
end

# An L1 device that serves both the three-tap BPSK and the five-tap BOC families.
const L1_WIDE_CAPABILITIES = HardwareCorrelatorCapabilities(;
    signals = [:GPSL1CA, :GalileoE1B],
    modulations = [:LOC, :CBOC],
    max_primary_code_length = 4092,
    code_frequency_limits = (1.023e6, 1.023e6),
    tap_layouts = [3, 5],
    max_tap_offset_chips = 1.0,
    num_antennas = 1,
    bands = [:L1],
    num_rf_inputs = 1,
    max_secondary_code_length = 1,
)

@testset "A device that declares nothing is the legacy GPS L1 C/A device" begin
    sdr = RecordingSDR(EPL, 4)
    caps = hardware_capabilities(sdr)
    @test caps === LEGACY_GPS_L1CA_CAPABILITIES
    @test caps.signals == [:GPSL1CA]
    @test caps.tap_layouts == [3]
    @test caps.max_primary_code_length == 1023
    @test caps.num_antennas == 1
    @test caps.bands == [:L1]
    @test !supports_secondary_code_wipeoff(caps, GPSL5I())

    # Its own signal is accepted.
    @test isnothing(
        hardware_support_error(caps, GPSL1CA(), get_default_correlator(GPSL1CA()), 4e6Hz),
    )
    @test isnothing(check_hardware_support(sdr, GPSL1CA(), 4e6Hz))
end

@testset "An unsupported signal is refused with an actionable error" begin
    sdr = RecordingSDR(EPL, 4)
    msg = hardware_support_error(
        LEGACY_GPS_L1CA_CAPABILITIES,
        GalileoE1B(),
        get_default_correlator(GalileoE1B()),
        4e6Hz,
    )
    @test !isnothing(msg)
    # It names the signal asked for, what the device does support, and every
    # separate reason — a message that stops at the first one sends the reader
    # round the loop again.
    @test occursin("GalileoE1B", msg)
    @test occursin("GPSL1CA", msg)
    @test occursin("CBOC", msg)
    @test occursin("4092", msg)
    @test occursin("tap", msg)

    err = try
        check_hardware_support(sdr, GalileoE1B(), 4e6Hz)
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("GalileoE1B", err.msg)
    @test occursin("RecordingSDR", err.msg)
end

@testset "Configurations are validated before any channel is armed" begin
    sdr = RecordingSDR(EPL, 4)
    # The regression baseline passes.
    @test isnothing(validate_hardware_configuration(sdr, (GPSL1CA(),), 4e6Hz))

    err = try
        validate_hardware_configuration(sdr, (GalileoE1B(),), 4e6Hz)
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("GalileoE1B", err.msg)
    # Nothing reached the device.
    @test isempty(sdr.assigned)

    # A CombinedSignal is validated component by component: the pilot is fine
    # for a three-tap L5 device, the data component too, but neither is for an
    # L1 C/A-only one.
    @test_throws ArgumentError validate_hardware_configuration(
        sdr,
        (CombinedSignal(GPSL5Q(), GPSL5I()),),
        4e6Hz,
    )
end

@testset "receive refuses an unsupported signal before the device is touched" begin
    sdr = SimulatedFPGA(GPSL1CA(); sampling_freq = 4e6, chunk = 4000)
    err = try
        receive(sdr, GalileoE1B(), 4e6Hz; acquire_async = false)
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("GalileoE1B", err.msg)
    @test isempty(sdr.handovers)
end

@testset "A dump record too narrow for the tracked correlator is caught up front" begin
    # The device says it serves the five-tap family, but its dump ring carries
    # only three accumulator slots: the record cannot transport the correlator.
    sdr = CapableSDR(EPL, 4, L1_WIDE_CAPABILITIES)
    err = try
        validate_hardware_configuration(sdr, (GalileoE1B(),), 4e6Hz)
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("GalileoE1B", err.msg)
    @test occursin("3", err.msg)
    @test occursin("slot", err.msg)
    # The same device is fine for the three-tap signal it can carry.
    @test isnothing(validate_hardware_configuration(sdr, (GPSL1CA(),), 4e6Hz))
end

@testset "The assignment describes the whole correlator configuration" begin
    sdr = CapableSDR(VEPL, 4, L1_WIDE_CAPABILITIES)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
        noise_source = :samples,
    )
    track_state =
        TrackState(; signals = (GPSL1CA = (GPSL1CA(),), GalileoE1B = (GalileoE1B(),)))
    track_state = add_satellite!(
        track_state;
        prn = 1,
        group = :GPSL1CA,
        code_phase = 0.0,
        carrier_doppler = 0.0Hz,
    )
    track_state = add_satellite!(
        track_state;
        prn = 2,
        group = :GalileoE1B,
        code_phase = 0.0,
        carrier_doppler = 0.0Hz,
    )
    band_systems = ((GPSL1CA(), GalileoE1B()),)
    band_measurements =
        (; L1 = Tracking.BandMeasurement(zeros(ComplexF64, 4000), 4e6Hz, 0.0Hz))
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)

    @test length(sdr.assigned) == 2
    l1ca = sdr.assigned[findfirst(c -> c.prn == 1, sdr.assigned)]
    e1b = sdr.assigned[findfirst(c -> c.prn == 2, sdr.assigned)]

    # Every quantised tap offset, latest first and prompt at zero — not just the
    # Early-to-Late distance the old interface handed over.
    @test l1ca.tap_sample_shifts == [-2, 0, 2]
    @test l1ca.el_sample_spacing == 4
    @test e1b.tap_sample_shifts == [-2, -1, 0, 1, 2]
    @test e1b.el_sample_spacing == 2
    # …and they agree with what Tracking would quantise for the tracked
    # correlator, which is what the DLL normalises by.
    @test e1b.el_sample_spacing == Tracking.get_early_late_sample_spacing(
        get_default_correlator(GalileoE1B()),
        4e6Hz,
        get_code_frequency(GalileoE1B()),
    )

    # Signal/component identity, so a pilot/data pair is distinguishable.
    @test l1ca.signal isa GPSL1CA
    @test l1ca.signal_index == 1
    @test l1ca.group_key == :GPSL1CA
    @test e1b.signal isa GalileoE1B

    # Replica amplitude / normalisation and the overlay contract.
    @test l1ca.replica_amplitude == 1.0
    @test l1ca.code_amplitude == get_code_amplitude(GPSL1CA())
    @test e1b.code_amplitude == get_code_amplitude(GalileoE1B())
    @test l1ca.secondary_code_mode === :primary_only
    @test e1b.secondary_code_mode === :primary_only

    # The component carrier-phase convention: the assignment states the
    # component's ICD phase against the band's in-phase reference, which the
    # device must *not* fold into the accumulators.
    @test l1ca.carrier_phase_offset == get_carrier_phase_offset(GPSL1CA())
    @test e1b.carrier_phase_offset == get_carrier_phase_offset(GalileoE1B())

    @test l1ca.band_id === :L1
    @test l1ca.sampling_freq == 4e6
end

@testset "Mixed three- and five-tap dumps reach the right tracked signals" begin
    # A five-slot wire carrying both layouts: the L1 C/A channel fills three
    # slots and says so, the Galileo E1B channel fills five.
    sdr = CapableSDR(
        VEPL,
        4,
        L1_WIDE_CAPABILITIES;
        band_gains = Dict(:L1 => 4.0),
        # This device replicates E1B with a plain ±1 BOC(1,1) replica, so its
        # code amplitude is 1 where GNSSSignals' CBOC table's is ~19.9.
        code_amplitudes = Dict(:GalileoE1B => 1.0),
    )
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
        noise_source = :samples,
    )
    track_state =
        TrackState(; signals = (GPSL1CA = (GPSL1CA(),), GalileoE1B = (GalileoE1B(),)))
    track_state = add_satellite!(
        track_state;
        prn = 1,
        group = :GPSL1CA,
        code_phase = 0.0,
        carrier_doppler = 0.0Hz,
    )
    track_state = add_satellite!(
        track_state;
        prn = 2,
        group = :GalileoE1B,
        code_phase = 0.0,
        carrier_doppler = 0.0Hz,
    )
    band_systems = ((GPSL1CA(), GalileoE1B()),)
    band_measurements =
        (; L1 = Tracking.BandMeasurement(zeros(ComplexF64, 4000), 4e6Hz, 0.0Hz))
    GNSSReceiver.sync_hardware_channels!(link, track_state, band_systems, band_measurements)

    l1ca_channel = link.channel_of[GNSSReceiver.HardwareChannelAssignment(:GPSL1CA, 1, 1)]
    e1b_channel = link.channel_of[GNSSReceiver.HardwareChannelAssignment(:GalileoE1B, 2, 1)]

    # Three meaningful taps in the leading slots of the five-slot wire record.
    three_tap = CorrelatorDump(
        l1ca_channel,
        1,
        CorrelatorOutput(vepl(40, 100, 40, 0, 0), 4000, 4000),
        NaN,
        3,
    )
    five_tap = CorrelatorDump(
        e1b_channel,
        2,
        CorrelatorOutput(vepl(10, 40, 100, 40, 10), 4000, 4000),
        NaN,
        5,
    )
    @test num_correlator_taps(three_tap) == 3
    @test num_correlator_taps(five_tap) == 5

    GNSSReceiver._append_dump!(link, track_state, three_tap)
    GNSSReceiver._append_dump!(link, track_state, five_tap)
    GNSSReceiver.flush_partial_records!(link, track_state)

    l1ca_out = only(get_correlator_outputs(get_sat_state(track_state, :GPSL1CA, 1), 1))
    e1b_out = only(get_correlator_outputs(get_sat_state(track_state, :GalileoE1B, 2), 1))

    # The record is retagged with the *tracked* correlator's type and spacing —
    # no invented taps in either direction.
    @test l1ca_out.correlator isa EarlyPromptLateCorrelator
    @test get_num_accumulators(l1ca_out.correlator) == 3
    @test e1b_out.correlator isa VeryEarlyPromptLateCorrelator
    @test get_num_accumulators(e1b_out.correlator) == 5
    @test l1ca_out.correlator.preferred_early_late_to_prompt_code_shift == 0.5
    @test e1b_out.correlator.preferred_early_late_to_prompt_code_shift == 0.15
    @test e1b_out.correlator.preferred_very_early_late_to_prompt_code_shift == 0.6

    # Normalisation: the band's replica gain divides out, and the device's own
    # code amplitude is rescaled to the one GNSSSignals reports, so a prompt
    # lands on the same amplitude scale whatever the gateware replicated.
    @test get_accumulators(l1ca_out.correlator) ≈ SVector{3,ComplexF64}(40, 100, 40) ./ 4.0
    e1b_scale = 4.0 * 1.0 / get_code_amplitude(GalileoE1B())
    @test get_accumulators(e1b_out.correlator) ≈
          SVector{5,ComplexF64}(10, 40, 100, 40, 10) ./ e1b_scale

    # A record whose tap count does not match the tracked correlator is refused
    # rather than reshaped into it.
    mismatched = CorrelatorDump(
        e1b_channel,
        2,
        CorrelatorOutput(vepl(40, 100, 40, 0, 0), 4000, 8000),
        NaN,
        3,
    )
    GNSSReceiver._append_dump!(link, track_state, mismatched)
    GNSSReceiver.flush_partial_records!(link, track_state)
    @test link.tap_layout_mismatches == 1
    @test length(get_correlator_outputs(get_sat_state(track_state, :GalileoE1B, 2), 1)) == 1
end

@testset "A dump defaults to the tap count its wire record carries" begin
    d = CorrelatorDump(1, 5, CorrelatorOutput(epl(0.4, 1.0, 0.4), 4000, 4000), NaN)
    @test num_correlator_taps(d) == 3
    @test isbitstype(typeof(d))
    wide = CorrelatorDump(1, 5, CorrelatorOutput(vepl(1, 2, 3, 2, 1), 4000, 4000))
    @test num_correlator_taps(wide) == 5
end
