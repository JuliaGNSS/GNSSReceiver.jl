# Tests for the hardware-correlator capability query and the pre-arm validation
# (issue #131).
#
# The hardware path shipped with #107/#129 assumed every device was the one it
# was written against: GPS L1 C/A, three taps, a 1023-chip code. These tests pin
# what a device is now allowed to be, what it has to declare, and — most
# importantly — that a request the device cannot serve is refused *before* a
# channel is armed. What the loop process then does with an arm is
# HardwareLoopCore's to test; the receiver's side of it is in
# test/remote_hardware_loop.jl.

using GNSSReceiver:
    HardwareCorrelatorCapabilities,
    LEGACY_GPS_L1CA_CAPABILITIES,
    check_hardware_support,
    hardware_capabilities,
    hardware_support_error,
    replica_code_amplitude,
    supports_secondary_code_wipeoff,
    validate_hardware_configuration,
    wire_tap_slots

using Tracking: EarlyPromptLateCorrelator, VeryEarlyPromptLateCorrelator, get_default_correlator

# A device that declares nothing beyond the two required methods: the legacy
# GPS L1 C/A device every adapter written before the capability query is.
struct LegacySDR <: AbstractHardwareCorrelatorSDR
    raw::GNSSReceiver.SignalChannel{ComplexF64,1}
    n_channels::Int
end
LegacySDR(n_channels) = LegacySDR(GNSSReceiver.SignalChannel{ComplexF64,1}(4000, 4), n_channels)
GNSSReceiver.raw_sample_channel(sdr::LegacySDR) = sdr.raw
GNSSReceiver.num_hardware_channels(sdr::LegacySDR) = sdr.n_channels

# A device that declares what it can do and can be given per-band replica gains
# and per-signal replica code amplitudes.
struct CapableSDR <: AbstractHardwareCorrelatorSDR
    raw::GNSSReceiver.SignalChannel{ComplexF64,1}
    n_channels::Int
    caps::HardwareCorrelatorCapabilities
    band_gains::Dict{Symbol,Float64}
    code_amplitudes::Dict{Symbol,Float64}
end

CapableSDR(
    n_channels,
    caps;
    band_gains = Dict{Symbol,Float64}(),
    code_amplitudes = Dict{Symbol,Float64}(),
) = CapableSDR(
    GNSSReceiver.SignalChannel{ComplexF64,1}(4000, 4),
    n_channels,
    caps,
    band_gains,
    code_amplitudes,
)

GNSSReceiver.raw_sample_channel(sdr::CapableSDR) = sdr.raw
GNSSReceiver.num_hardware_channels(sdr::CapableSDR) = sdr.n_channels
GNSSReceiver.hardware_capabilities(sdr::CapableSDR) = sdr.caps
GNSSReceiver.correlator_gain(sdr::CapableSDR, band_id::Symbol) =
    get(sdr.band_gains, band_id, 1.0)
GNSSReceiver.replica_code_amplitude(sdr::CapableSDR, signal::AbstractGNSSSignal) =
    get(sdr.code_amplitudes, get_signal_id(signal), get_code_amplitude(signal))

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
    sdr = LegacySDR(4)
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
    sdr = LegacySDR(4)
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
    @test occursin("LegacySDR", err.msg)
end

@testset "Configurations are validated before any channel is armed" begin
    sdr = LegacySDR(4)
    # The regression baseline passes.
    @test isnothing(validate_hardware_configuration(sdr, (GPSL1CA(),), 4e6Hz))

    err = try
        validate_hardware_configuration(sdr, (GalileoE1B(),), 4e6Hz)
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("GalileoE1B", err.msg)

    # A CombinedSignal is validated component by component: the pilot is fine
    # for a three-tap L5 device, the data component too, but neither is for an
    # L1 C/A-only one.
    @test_throws ArgumentError validate_hardware_configuration(
        sdr,
        (CombinedSignal(GPSL5Q(), GPSL5I()),),
        4e6Hz,
    )
end

@testset "A capable device is validated against what it declares" begin
    sdr = CapableSDR(4, L1_WIDE_CAPABILITIES)
    @test isnothing(validate_hardware_configuration(sdr, (GPSL1CA(), GalileoE1B()), 4e6Hz))
    # Something outside the declaration is still refused, by name.
    err = try
        validate_hardware_configuration(sdr, (GPSL5I(),), 4e6Hz)
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("GPSL5I", err.msg)
    @test occursin("CapableSDR", err.msg)
end

@testset "A record too narrow for the tracked correlator is caught up front" begin
    # A device that serves the five-tap family over a record format carrying
    # only three accumulator slots: the record cannot transport the correlator.
    @test wire_tap_slots(typeof(get_default_correlator(GPSL1CA()))) == 3
    @test wire_tap_slots(typeof(get_default_correlator(GalileoE1B()))) == 5
    msg = hardware_support_error(
        L1_WIDE_CAPABILITIES,
        GalileoE1B(),
        get_default_correlator(GalileoE1B()),
        4e6Hz;
        dump_tap_slots = 3,
    )
    @test !isnothing(msg)
    @test occursin("GalileoE1B", msg)
    @test occursin("3", msg)
    @test occursin("slot", msg)
    # The same record is fine for the three-tap signal it can carry.
    @test isnothing(
        hardware_support_error(
            L1_WIDE_CAPABILITIES,
            GPSL1CA(),
            get_default_correlator(GPSL1CA()),
            4e6Hz;
            dump_tap_slots = 3,
        ),
    )
end

@testset "Replica amplitudes are the device's to declare, per band and per signal" begin
    sdr = CapableSDR(
        4,
        L1_WIDE_CAPABILITIES;
        band_gains = Dict(:L1 => 4.0),
        # This device replicates E1B with a plain ±1 BOC(1,1) replica, so its
        # code amplitude is 1 where GNSSSignals' CBOC table's is ~19.9.
        code_amplitudes = Dict(:GalileoE1B => 1.0),
    )
    @test GNSSReceiver.correlator_gain(sdr, :L1) == 4.0
    @test GNSSReceiver.correlator_gain(sdr, :L5) == 1.0
    @test replica_code_amplitude(sdr, GalileoE1B()) == 1.0
    @test replica_code_amplitude(sdr, GPSL1CA()) == get_code_amplitude(GPSL1CA())
    # The defaults: unit carrier replica, the modelled code amplitude.
    legacy = LegacySDR(4)
    @test GNSSReceiver.correlator_gain(legacy) == 1
    @test GNSSReceiver.correlator_gain(legacy, :L1) == 1
    @test replica_code_amplitude(legacy, GalileoE1B()) == get_code_amplitude(GalileoE1B())
end
