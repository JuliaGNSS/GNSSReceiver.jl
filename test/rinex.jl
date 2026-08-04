# Note: this file is included from runtests.jl which provides all `using` statements.

# Fractional part of a carrier phase as `Tracking` reports it: wrapped into one cycle
# around zero.
_wrap(phase) = mod(phase + 0.5, 1) - 0.5

# GPS week 2124 (the week 10-bit `trans_week = 76` resolves to around mid-2020) starts on
# Sunday 2020-09-20; the reference times below are four days into it.
const _GPS_WEEK = 2124
const _GPS_TOW = 4 * 86400
const _GPS_TOC = DateTime(2020, 9, 24)

# A GPS LNAV data set carrying every field `is_decoding_completed_for_positioning` requires,
# so `rinex_ephemeris` can be checked against hand-computed record values. The orbital
# elements are a real broadcast set (GPS PRN 5) in the units the decoder stores — angles
# already scaled from semicircles to radians — so the record values are physically
# meaningful and a unit slip shows up as an impossible orbit. The subframe 4 page 18
# ionosphere / UTC parameters are passed in only where a test needs them.
_gps_lnav_data(; kwargs...) = GNSSDecoder.GPSL1CAData(;
    last_subframe_id = 3,
    integrity_status_flag = false,
    TOW = 82596,
    alert_flag = false,
    anti_spoof_flag = true,
    # Subframe 1 — clock correction and satellite status
    trans_week = 76,
    codeonl2 = 1,
    ura = 2.0,
    sv_health = "000000",
    IODC = "0000111111",   # 63
    l2pcode = false,
    T_GD = -1.071020960808e-8,
    t_0c = _GPS_TOW,
    a_f0 = -2.989266067743e-5,
    a_f1 = 1.364242052659e-12,
    a_f2 = 0.0,
    # Subframe 2 — ephemeris, part 1
    IODE_Sub_2 = "00111111",   # 63
    C_rs = -4.49375e1,
    Δn = 5.19633e-9,           # rad/s
    M_0 = 2.79930,             # rad
    C_uc = -2.296641469002e-6,
    e = 5.172203877009e-3,
    C_us = 7.340684533119e-6,
    sqrt_A = 5.1535496521e3,
    t_0e = _GPS_TOW,
    fit_interval = false,
    AODO = 0,
    # Subframe 3 — ephemeris, part 2
    C_ic = -3.166496753693e-8,
    Ω_0 = 2.91237,             # rad
    C_is = -6.705522537231e-8,
    i_0 = 0.947436,            # rad — a GPS orbit is inclined ~54°
    C_rc = 2.330625e2,
    ω = 0.577340,              # rad
    Ω_dot = -8.29275e-9,       # rad/s
    IODE_Sub_3 = "00111111",
    i_dot = -2.23225e-10,      # rad/s
    kwargs...,
)

# The same for Galileo I/NAV, with a real Galileo orbit (semi-major axis ~29 600 km, orbits
# inclined ~56°). `WN` is the 12-bit week counted from GPS week 1024, so the record's
# GPS-aligned week is `WN + 1024`.
_galileo_inav_data(; kwargs...) = GNSSDecoder.GalileoE1BData(;
    WN = _GPS_WEEK - 1024,
    TOW = 82596,
    SVID = 24,
    t_0e = Float64(_GPS_TOW),
    M_0 = -1.42357,            # rad
    e = 2.1998010845e-4,
    sqrt_A = 5.4406533737e3,
    Ω_0 = 1.83094,             # rad
    i_0 = 0.976274,            # rad — a Galileo orbit is inclined ~56°
    ω = -2.51837,              # rad
    i_dot = -1.75001e-10,
    Ω_dot = -5.35001e-9,
    Δn = 2.79251e-9,
    C_uc = -6.5192580223e-7,
    C_us = 8.4526836872e-6,
    C_rc = 1.4059375e2,
    C_rs = -2.28125e1,
    C_ic = 1.4901161194e-8,
    C_is = 1.4901161194e-8,
    SISA_e1_e5b = 107,
    t_0c = Float64(_GPS_TOW),
    a_f0 = -5.2394117482e-4,
    a_f1 = -1.5265128291e-12,
    a_f2 = 0.0,
    IOD_nav1 = UInt(37),
    IOD_nav2 = UInt(37),
    IOD_nav3 = UInt(37),
    IOD_nav4 = UInt(37),
    signal_health_e1b = GNSSDecoder.signal_ok,
    signal_health_e5b = GNSSDecoder.signal_ok,
    data_validity_status_e1b = GNSSDecoder.navigation_data_valid,
    data_validity_status_e5b = GNSSDecoder.navigation_data_valid,
    broadcast_group_delay_e1_e5a = 2.7939677238e-9,
    broadcast_group_delay_e1_e5b = 3.4924596548e-9,
    kwargs...,
)

# A decoder state holding `data` as validated navigation data. A freshly constructed decoder
# reports no bit count until it has synchronised; zero puts the transmit time exactly on the
# decoded time of week, which is what makes the expected pseudoranges below exact.
_decoder_with(system, prn, data) = GNSSDecoder.GNSSDecoderState(
    GNSSDecoderState(system, prn);
    data,
    raw_data = data,
    num_bits_after_valid_syncro_sequence = 0,
)

@testset "RINEX observation-header layout" begin
    # One constellation, one signal: four observables in header order.
    obs_types, layouts = GNSSReceiver.rinex_layout(((GPSL1CA(),),), (0.0Hz,))
    @test obs_types == ['G' => ["C1C", "L1C", "D1C", "S1C"]]
    @test layouts[:GPSL1CA].system == 'G'
    @test layouts[:GPSL1CA].code == "1C"
    @test layouts[:GPSL1CA].interm_freq == 0.0
    @test layouts[:GPSL1CA].carrier_frequency == 1.57542e9
    # The wavelength the phase and range-rate conversions use.
    @test GNSSReceiver.wavelength(layouts[:GPSL1CA]) ≈ 0.1902936728 atol = 1e-9

    # Two constellations on one band: one header entry each, in order of appearance. The
    # `CombinedSignal` is keyed and coded by its ranging signal — the pilot E1C — because
    # that is what the receiver actually measures on.
    obs_types, layouts = GNSSReceiver.rinex_layout(
        ((GPSL1CA(), CombinedSignal(GalileoE1C(), GalileoE1B())),),
        (0.0Hz,),
    )
    @test obs_types ==
          ['G' => ["C1C", "L1C", "D1C", "S1C"], 'E' => ["C1C", "L1C", "D1C", "S1C"]]
    @test layouts[:GalileoE1C].system == 'E'
    @test layouts[:GalileoE1C].code == "1C"
    @test !haskey(layouts, :GalileoE1B)

    # Two bands of one constellation share a single header entry, the second band's
    # observables following the first's — and each band keeps its own intermediate
    # frequency, which the carrier-phase accumulation needs.
    obs_types, layouts = GNSSReceiver.rinex_layout(
        ((GPSL1CA(),), (CombinedSignal(GPSL5Q(), GPSL5I()),)),
        (0.0Hz, 1.0e6Hz),
    )
    @test obs_types == ['G' => ["C1C", "L1C", "D1C", "S1C", "C5Q", "L5Q", "D5Q", "S5Q"]]
    @test layouts[:GPSL1CA].code == "1C"
    @test layouts[:GPSL5Q].code == "5Q"
    @test layouts[:GPSL5Q].interm_freq == 1.0e6
    # Nothing here tracks column positions any more: `SatObs` derives each observable's
    # place from the header, so a second band's types only have to reach the header.
    @test length(last(only(obs_types))) == 8
end

@testset "Every trackable signal has a RINEX observation code" begin
    # `rinex_layout` refuses to write a file it cannot label correctly, so the mapping has
    # to cover every signal this receiver can range on — otherwise enabling RINEX output
    # would fail at run time for a perfectly trackable configuration.
    for signal in (
        GPSL1CA(),
        GPSL1C_D(),
        GPSL1C_P(),
        GPSL2CM(),
        GPSL2CL(),
        GPSL5I(),
        GPSL5Q(),
        GalileoE1B(),
        GalileoE1B_BOC11(),
        GalileoE1C(),
        GalileoE1C_BOC11(),
        GalileoE5aI(),
        GalileoE5aQ(),
    )
        signal_id = get_signal_id(signal)
        @test haskey(GNSSReceiver.RINEX_SIGNAL_CODES, signal_id)
        @test GNSSReceiver.rinex_system_char(signal) in ('G', 'E')
        # A RINEX 3.05 observation code is a band digit plus a signal-attribute letter.
        code = GNSSReceiver.RINEX_SIGNAL_CODES[signal_id]
        @test length(code) == 2
        @test isdigit(code[1]) && isuppercase(code[2])
    end
end

@testset "Continuous carrier phase" begin
    # A satellite whose Doppler ramps linearly, of which tracking only ever reports the
    # fractional part of the replica phase. `phase` is the integral of the replica
    # frequency (Doppler plus intermediate frequency), i.e. the truth to recover.
    interm_freq = 1.0e5
    doppler(t) = 2000.0 - 300.0 * t
    phase(t) = (2000.0 + interm_freq) * t - 150.0 * t^2
    Δt = 0.004

    accumulator =
        GNSSReceiver.CarrierPhaseAccumulator(_wrap(phase(0.0)), doppler(0.0), 0.0, 0.0, 1)
    @test accumulator.restarted
    @test isnan(accumulator.anchor)
    @test accumulator.phase == 0.0

    for chunk = 2:251
        t = (chunk - 1) * Δt
        GNSSReceiver.advance!(
            accumulator,
            _wrap(phase(t)),
            doppler(t),
            0.0,
            t,
            chunk,
            interm_freq,
        )
    end
    t_end = 250 * Δt
    @test accumulator.chunk == 251
    # Over 250 chunks the replica turns ~42000 times, of which the wrapped phase shows
    # none: the accumulator has to reproduce every whole cycle, and does so to
    # floating-point precision.
    @test accumulator.phase ≈ phase(t_end) - interm_freq * t_end atol = 1e-9

    # The same at a Doppler large enough that a chunk spans many whole cycles, which is
    # where a naive unwrap of the fractional phase alone would fail outright.
    fast_doppler(t) = 5000.0 + 100.0 * t
    fast_phase(t) = 5000.0 * t + 50.0 * t^2
    accumulator = GNSSReceiver.CarrierPhaseAccumulator(
        _wrap(fast_phase(0.0)),
        fast_doppler(0.0),
        0.0,
        0.0,
        1,
    )
    for chunk = 2:251
        t = (chunk - 1) * Δt
        GNSSReceiver.advance!(
            accumulator,
            _wrap(fast_phase(t)),
            fast_doppler(t),
            0.0,
            t,
            chunk,
            0.0,
        )
    end
    @test accumulator.phase ≈ fast_phase(t_end) atol = 1e-9

    # The receiver's own oscillator offset rides on every satellite's Doppler, and the epochs
    # and pseudoranges are already free of it, so the accumulated phase must come out on the
    # GNSS time scale too: the same replica phase, plus the clock term integrated back out.
    clock_rate = 1165.0
    drifted = GNSSReceiver.CarrierPhaseAccumulator(
        _wrap(phase(0.0)),
        doppler(0.0),
        clock_rate,
        0.0,
        1,
    )
    for chunk = 2:251
        t = (chunk - 1) * Δt
        GNSSReceiver.advance!(
            drifted,
            _wrap(phase(t)),
            doppler(t),
            clock_rate,
            t,
            chunk,
            interm_freq,
        )
    end
    @test drifted.phase ≈ phase(t_end) - interm_freq * t_end + clock_rate * t_end atol =
        1e-9
end

@testset "GPS LNAV ephemeris record" begin
    decoder = _decoder_with(GPSL1CA(), 13, _gps_lnav_data())
    @test is_decoding_completed_for_positioning(decoder)
    eph = GNSSReceiver.rinex_ephemeris(decoder; approximate_year = 2020)
    @test eph isa GPSEphemeris
    @test eph.prn == 13
    # The 10-bit broadcast week resolved into the continuous week RINEX wants, and the
    # calendar time that week plus the clock reference second lands on.
    @test eph.week == _GPS_WEEK
    @test eph.toc == _GPS_TOC
    # Clock polynomial and group delay pass through unscaled.
    @test eph.af0 == -2.989266067743e-5
    @test eph.af1 == 1.364242052659e-12
    @test eph.af2 == 0.0
    @test eph.tgd == -1.071020960808e-8
    # RINEX wants angles in radians, which is what the decoder already holds: it applies the
    # ICD's semicircle scaling as it parses the message, so scaling again here would double
    # it. A GPS orbit is the check that catches that — its inclination is ~55°, which is
    # inside the ±π an angle may occupy but ×π would push far outside it.
    @test eph.m0 == 2.79930
    @test eph.omega0 == 2.91237
    @test eph.i0 == 0.947436
    @test eph.omega == 0.577340
    @test eph.deltan == 5.19633e-9
    @test eph.omegadot == -8.29275e-9
    @test eph.idot == -2.23225e-10
    @test all(abs(getfield(eph, f)) <= π for f in (:m0, :omega0, :i0, :omega))
    # A GPS orbit plane is inclined ~55°; a semicircle/radian slip would put it near 170°.
    @test rad2deg(eph.i0) ≈ 54.3 atol = 1.0
    # And the orbit it describes is a GPS one: a ~26 560 km semi-major axis, nearly circular.
    @test eph.sqrt_a^2 ≈ 2.656e7 rtol = 1e-3
    @test eph.e < 0.02
    # Harmonic corrections and the remaining elements are already in RINEX units.
    @test eph.crs == -4.49375e1
    @test eph.cuc == -2.296641469002e-6
    @test eph.e == 5.172203877009e-3
    @test eph.sqrt_a == 5.1535496521e3
    @test eph.toe == _GPS_TOW
    # The issue-of-data and health fields arrive as binary strings.
    @test eph.iode == 63
    @test eph.iodc == 63
    @test eph.sv_health == 0
    @test eph.sv_accuracy == 2.0
    @test eph.codes_on_l2 == 1
    @test eph.l2p_data_flag == 0.0
    @test eph.transmission_time == 82596
    # The broadcast flag only says "nominal" or "extended"; nominal is four hours.
    @test eph.fit_interval == 4.0
    eph_extended = GNSSReceiver.rinex_ephemeris(
        _decoder_with(GPSL1CA(), 13, _gps_lnav_data(; fit_interval = true));
        approximate_year = 2020,
    )
    @test eph_extended.fit_interval == 6.0

    # An unhealthy satellite's 6-bit health word is written as the number it encodes.
    eph_unhealthy = GNSSReceiver.rinex_ephemeris(
        _decoder_with(GPSL1CA(), 13, _gps_lnav_data(; sv_health = "111111"));
        approximate_year = 2020,
    )
    @test eph_unhealthy.sv_health == 63
end

@testset "Galileo ephemeris records" begin
    decoder = _decoder_with(GalileoE1B(), 24, _galileo_inav_data())
    @test is_decoding_completed_for_positioning(decoder)
    eph = GNSSReceiver.rinex_ephemeris(decoder; approximate_year = 2020)
    @test eph isa GalileoEphemeris
    @test eph.prn == 24
    # Galileo's 12-bit week counts from GPS week 1024, so the record's week is GPS-aligned.
    @test eph.week == _GPS_WEEK
    @test eph.toc == _GPS_TOC
    @test eph.iodnav == 37
    @test eph.bgd_e5a_e1 == 2.7939677238e-9
    @test eph.bgd_e5b_e1 == 3.4924596548e-9
    # I/NAV from E1-B with E5b·E1 clock terms: bit 0 and bit 9.
    @test eph.data_sources == 1 + 512
    # SISA index 107 falls in the fourth segment of Table 92: 2 m + 0.16 m per step.
    @test eph.sisa ≈ 2.0 + 0.16 * 7
    # All four Galileo signals healthy and valid.
    @test eph.sv_health == 0
    # Radians already, as for GPS — the decoder has applied the semicircle scaling.
    @test eph.m0 == -1.42357
    @test all(abs(getfield(eph, f)) <= π for f in (:m0, :omega0, :i0, :omega))
    # A Galileo orbit: inclined ~56°, semi-major axis ~29 600 km, nearly circular.
    @test rad2deg(eph.i0) ≈ 55.9 atol = 1.0
    @test eph.sqrt_a^2 ≈ 2.960e7 rtol = 1e-3
    @test eph.e < 0.01
    @test eph.transmission_time == 82596

    # A degraded E1B: the data-validity bit is bit 0 and the two health bits sit above it,
    # while E5b — reported healthy in the same message — leaves bits 6-8 clear.
    degraded = _decoder_with(
        GalileoE1B(),
        24,
        _galileo_inav_data(;
            signal_health_e1b = GNSSDecoder.signal_out_of_service,
            data_validity_status_e1b = GNSSDecoder.working_without_guarantee,
        ),
    )
    @test GNSSReceiver.rinex_ephemeris(degraded; approximate_year = 2020).sv_health == 0b011

    # An E5b out of service occupies bits 6-8 of the same word.
    e5b_down = _decoder_with(
        GalileoE1B(),
        24,
        _galileo_inav_data(; signal_health_e5b = GNSSDecoder.signal_out_of_service),
    )
    @test GNSSReceiver.rinex_ephemeris(e5b_down; approximate_year = 2020).sv_health ==
          0b010_000_000

    # Signal-in-space accuracy across every segment of the table, plus the two ranges that
    # mean "no prediction available".
    @test GNSSReceiver.sisa_metres(0) == 0.0
    @test GNSSReceiver.sisa_metres(49) ≈ 0.49
    @test GNSSReceiver.sisa_metres(50) ≈ 0.5
    @test GNSSReceiver.sisa_metres(74) ≈ 0.98
    @test GNSSReceiver.sisa_metres(75) ≈ 1.0
    @test GNSSReceiver.sisa_metres(99) ≈ 1.96
    @test GNSSReceiver.sisa_metres(100) ≈ 2.0
    @test GNSSReceiver.sisa_metres(125) ≈ 6.0
    @test GNSSReceiver.sisa_metres(200) == -1.0
    @test GNSSReceiver.sisa_metres(255) == -1.0
    @test GNSSReceiver.sisa_metres(nothing) == -1.0
end

@testset "RINEX 3.05 has no record for the GPS CNAV messages" begin
    # GPS L5 and L2C broadcast a quasi-Keplerian ephemeris RINEX 3.05 cannot express, so
    # those decoders yield no record — which must be reported as such rather than written
    # as a wrong one.
    for system in (GPSL5I(), GPSL2CM())
        decoder = GNSSDecoderState(system, 3)
        @test isnothing(GNSSReceiver.rinex_ephemeris(decoder; approximate_year = 2020))
    end
end

@testset "Navigation-header records from the broadcast message" begin
    # GPS subframe 4 page 18 carries the Klobuchar ionosphere model and the UTC offset.
    gps = _decoder_with(
        GPSL1CA(),
        13,
        _gps_lnav_data(;
            α_0 = 1.0244548321e-8,
            α_1 = 2.2351741791e-8,
            α_2 = -5.9604644775e-8,
            α_3 = -1.1920928955e-7,
            β_0 = 8.8064e4,
            β_1 = 4.9152e4,
            β_2 = -6.5536e4,
            β_3 = -1.9661e5,
            A_0 = -9.3132257462e-10,
            A_1 = -7.1054273576e-15,
            t_ot = 5.8982e5,
            WN_t = 76,
            Δt_LS = 18,
            Δt_LSF = 18,
            WN_LSF = 137,
            DN = 7,
        ),
    )
    iono = GNSSReceiver.nav_ionospheric_corrections!(IonosphericCorrection[], gps.data)
    @test length(iono) == 2
    @test iono[1].type == "GPSA"
    @test iono[1].parameters ==
          (1.0244548321e-8, 2.2351741791e-8, -5.9604644775e-8, -1.1920928955e-7)
    @test iono[2].type == "GPSB"
    utc = GNSSReceiver.nav_time_system_corrections!(TimeSystemCorrection[], gps.data)
    @test length(utc) == 1
    @test utc[1].type == "GPUT"
    @test utc[1].a0 == -9.3132257462e-10
    @test utc[1].reference_week == 76
    # The four-field leap-second record, which some parsers require.
    @test GNSSReceiver.nav_leap_seconds(gps.data) == (18, 18, 137, 7)
    # A satellite that has not sent the record yet contributes nothing.
    @test isempty(
        GNSSReceiver.nav_ionospheric_corrections!(
            IonosphericCorrection[],
            _gps_lnav_data(),
        ),
    )
    @test isnothing(GNSSReceiver.nav_leap_seconds(_gps_lnav_data()))

    # Galileo's NeQuick coefficients fill a Klobuchar-shaped record, and word type 10
    # additionally gives the Galileo-to-GPS time offset.
    galileo = _decoder_with(
        GalileoE1B(),
        24,
        _galileo_inav_data(;
            a_i0 = 4.5e1,
            a_i1 = 1.5625e-2,
            a_i2 = -3.0517578125e-3,
            A_0_utc = -9.3132257462e-10,
            A_1_utc = -7.1054273576e-15,
            t_0t = 5.184e5,
            WN_0t = 76,
            A_0G = 1.8626451492e-9,
            A_1G = 8.8817841970e-16,
            t_0G = 5.184e5,
            WN_0G = 12,
        ),
    )
    iono = GNSSReceiver.nav_ionospheric_corrections!(IonosphericCorrection[], galileo.data)
    @test length(iono) == 1
    @test iono[1].type == "GAL"
    @test iono[1].parameters == (4.5e1, 1.5625e-2, -3.0517578125e-3, 0.0)
    corrections =
        GNSSReceiver.nav_time_system_corrections!(TimeSystemCorrection[], galileo.data)
    @test [c.type for c in corrections] == ["GAUT", "GPGA"]
end

@testset "RINEX epoch and time helpers" begin
    # Times of week are differenced across the week boundary: a satellite that transmitted
    # just before the roll-over still yields a sane travel time.
    @test GNSSReceiver.week_difference(0.5, 604799.93) ≈ 0.57
    @test GNSSReceiver.week_difference(604799.93, 0.5) ≈ -0.57
    @test GNSSReceiver.week_difference(82621.7, 82621.63) ≈ 0.07

    # A GPS week and time of week to the calendar time plus sub-millisecond remainder a
    # RINEX epoch record carries.
    time, fraction = GNSSReceiver.epoch_datetime(_GPS_WEEK, _GPS_TOW)
    @test time == _GPS_TOC
    @test fraction == 0.0
    time, fraction = GNSSReceiver.epoch_datetime(_GPS_WEEK, _GPS_TOW + 0.1002)
    @test time == _GPS_TOC + Millisecond(100)
    # Only the resolution of a time of week itself is lost, ~0.06 ns.
    @test fraction ≈ 0.0002 atol = 1e-9

    # The PVT solution reports its epoch on the atomic scale; the round trip back to GPS
    # system-time seconds is what recovers the week the epoch falls in.
    total = _GPS_WEEK * 604800 + _GPS_TOW + 0.25
    origin = GNSSReceiver.gps_time_origin()
    epoch = TAIEpoch(floor(Int, total) + origin.second, total - floor(total))
    @test GNSSReceiver.gps_seconds(epoch) ≈ total
    @test round(Int, (GNSSReceiver.gps_seconds(epoch) - (_GPS_TOW + 0.25)) / 604800) ==
          _GPS_WEEK

    # Signal-strength indicator: the spec's 1-9 scale in 6 dB-Hz steps, and 0 for a CN0
    # estimate that has not converged.
    @test GNSSReceiver.signal_strength_indicator(45.0) == 7
    @test GNSSReceiver.signal_strength_indicator(54.0) == 9
    @test GNSSReceiver.signal_strength_indicator(90.0) == 9
    @test GNSSReceiver.signal_strength_indicator(0.0) == 1
    @test GNSSReceiver.signal_strength_indicator(NaN) == 0
    # A non-finite observable is written as a blank field by RINEXParser, so an unconverged
    # CN0 reaches it as an `ObsValue` and comes out absent rather than as an unparsable one.
    @test ObsValue(NaN; ssi = 0).value === NaN
end

# Raw transmit time of the synthetic satellite below: the decoded time of week, with the
# tracking state's code and carrier phase both left at zero.
const _TRANSMIT_TOW = 82596.0

# One receiver state with a single tracked GPS satellite, a decoded navigation message and a
# synthetic PVT solution, so the whole logger path can be driven without a real signal. The
# satellite transmitted `travel_time` before the receive instant, and `receive_offset` moves
# that instant relative to the epoch grid.
function _rinex_receiver_state(;
    prn = 13,
    travel_time = 0.07,
    receive_offset = 0.0,
    doppler = 0.0Hz,
    clock_drift = 0.0,
)
    system = GPSL1CA()
    key = get_signal_id(system)
    band_key = get_band_id(GNSSReceiver.system_band(system))
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(1),
    )
    track_state = merge_sats(
        receiver_state.track_state,
        key,
        [
            GNSSReceiver.create_tracked_sat(
                GNSSReceiver.tracking_signals(system),
                prn,
                0.0,
                doppler,
                NumAnts(1),
                receiver_state.track_state.doppler_estimator,
            ),
        ],
    )
    sat_state = GNSSReceiver.ReceiverSatState(
        prn,
        _decoder_with(system, prn, _gps_lnav_data()),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        3.0u"s",
        0.0u"s",
        0,
        # Scalar tracking: the loops are the satellite's own, not the vector-tracking
        # filter's. RINEX reads the measurements either way.
        false,
    )
    # `calc_pvt` references its pseudoranges to the latest satellite transmit time, so with a
    # single satellite that reference is its own transmission and the whole travel time is
    # carried by the estimated clock bias — exactly the large, negative bias a real solution
    # reports. Giving the satellite no clock error makes its corrected transmit time equal
    # the raw one, so the expected pseudorange is simply `travel_time · c`.
    reference_time = _TRANSMIT_TOW + receive_offset
    receive_time = reference_time + travel_time
    receive_seconds = _GPS_WEEK * 604800 + receive_time
    pvt = PositionVelocityTime.PVTSolution(;
        position = Geodesy.ECEF(4.1e6, 6.0e5, 4.8e6),
        time_correction = -travel_time * 299792458.0 * u"m",
        time = TAIEpoch(
            floor(Int, receive_seconds) + GNSSReceiver.gps_time_origin().second,
            receive_seconds - floor(receive_seconds),
        ),
        sats = Dictionary(
            [(key, prn)],
            [
                PositionVelocityTime.SatInfo(
                    Geodesy.ECEF(1.5e7, 1.2e7, 1.8e7),
                    reference_time,
                    0.1u"m",
                    # Post-fit range-rate residual, which RINEX output never reads: the
                    # Doppler it writes comes from the tracking loops, not the fit.
                    0.0u"m/s",
                ),
            ],
        ),
        relative_clock_drift = clock_drift,
        reference_system = GNSSSignals.GPST(),
    )
    ReceiverState(
        track_state,
        (; key => Dictionary([prn], [sat_state])),
        NamedTuple{(band_key,)}((GNSSReceiver.SampleBuffer(ComplexF64, 20000),)),
        NamedTuple{(band_key,)}((-Inf * 1.0u"s",)),
        pvt,
        PositionVelocityTime.SatelliteState[],
        # Scalar tracking: no vector-tracking state. RINEX is written from the tracking
        # loops and the PVT solution, which both modes produce alike.
        nothing,
        1.0u"s",
        1.0u"s",
    )
end

# Split a RINEX observation line into its fixed 16-column observation fields: a 14-column
# value followed by the loss-of-lock and signal-strength indicators.
_obs_fields(line, count) = [line[(4+16*(i-1)):(3+16*i)] for i = 1:count]
_obs_value(field) = parse(Float64, field[1:14])
_obs_lli(field) = field[15]
_obs_ssi(field) = field[16]

@testset "RINEX observation epoch from a receiver state" begin
    directory = mktempdir()
    obs_file = joinpath(directory, "test.obs")
    nav_file = joinpath(directory, "test.nav")
    state = _rinex_receiver_state()
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file, marker_name = "TEST-1"),
        ((GPSL1CA(),),),
        (0.0Hz,);
        approximate_year = 2020,
    )
    GNSSReceiver.log_rinex!(logger, state)
    close(logger)

    lines = readlines(obs_file)
    header_end = findfirst(line -> occursin("END OF HEADER", line), lines)
    header = lines[1:header_end]
    body = lines[(header_end+1):end]

    # The header describes the file the epochs were actually written under, including the
    # marker and the approximate position, which only the first fix could supply.
    @test any(
        line -> occursin("OBSERVATION DATA", line) && occursin("G: GPS", line),
        header,
    )
    @test any(line -> startswith(line, "TEST-1") && occursin("MARKER NAME", line), header)
    @test any(line -> occursin("G    4 C1C L1C D1C S1C", line), header)
    position_line = header[findfirst(line -> occursin("APPROX POSITION XYZ", line), header)]
    @test all(isapprox.(parse.(Float64, split(position_line[1:42])), [4.1e6, 6.0e5, 4.8e6]))
    @test any(line -> occursin("TIME OF FIRST OBS", line), header)

    # One epoch with one satellite, stamped on the nominal (integer-second) epoch the
    # measurements were steered onto. GPS week 2124 second 82596 is 2020-09-20 + 22:56:36.
    @test length(body) == 2
    epoch_line, sat_line = body
    @test startswith(epoch_line, "> 2020 09 20 22 56 36.0000000")
    @test occursin("  0  1", epoch_line)
    # No receiver clock offset field: the epoch itself was steered onto GNSS system time.
    @test length(rstrip(epoch_line)) == 35
    @test startswith(sat_line, "G13")

    pseudorange, phase, doppler, _ = _obs_fields(sat_line, 4)
    # The satellite transmitted 70 ms before the receive instant. With no Doppler there is no
    # propagation correction either, so the pseudorange is exactly that travel time times the
    # speed of light — and, crucially, the *raw* travel time: the satellite clock correction
    # must not be pre-applied, since every RINEX consumer applies it from the nav file.
    @test _obs_value(pseudorange) ≈ 0.07 * 299792458.0 rtol = 1e-9
    # The phase ambiguity is anchored on that same range, in cycles of the L1 wavelength.
    wavelength = 299792458.0 / 1.57542e9
    @test _obs_value(phase) ≈ 0.07 * 299792458.0 / wavelength rtol = 1e-9
    # First epoch of the arc, so the phase carries the loss-of-lock indicator.
    @test _obs_lli(phase) == '1'
    @test _obs_value(doppler) == 0.0
end

@testset "RINEX observables are propagated onto the steered epoch" begin
    # The receive instant is 70 ms past an integer second, so the epoch is steered back onto
    # it and every observable is carried along by its own Doppler. Range and phase must move
    # together — the range rate is `-λ·f_d` and the phase rate `-f_d` — or code and phase
    # would disagree by metres.
    directory = mktempdir()
    obs_file = joinpath(directory, "steer.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,);
        approximate_year = 2020,
    )
    GNSSReceiver.log_rinex!(logger, _rinex_receiver_state(; doppler = 1000.0Hz))
    close(logger)

    lines = readlines(obs_file)
    sat_line = last(lines)
    pseudorange, phase, doppler, _ = _obs_fields(sat_line, 4)
    wavelength = 299792458.0 / 1.57542e9
    # `_TRANSMIT_TOW + 0.07` rounds down onto the integer second, so the propagation is
    # −70 ms and both observables are carried backwards over it.
    propagation = -0.07
    range_at_measurement = 0.07 * 299792458.0
    @test _obs_value(pseudorange) ≈ range_at_measurement - wavelength * 1000.0 * propagation rtol =
        1e-9
    @test _obs_value(phase) ≈ range_at_measurement / wavelength - 1000.0 * propagation rtol =
        1e-9
    # A propagated range and phase stay consistent with each other to a fraction of a cycle.
    @test _obs_value(pseudorange) / wavelength ≈ _obs_value(phase) rtol = 1e-9
    @test _obs_value(doppler) == 1000.0
end

@testset "RINEX observables carry no receiver clock drift" begin
    # The receiver's oscillator runs fast or slow, which shows up as a common-mode Doppler on
    # every satellite. The epochs are steered onto GNSS system time and the pseudoranges are
    # differences of transmit times, so both are already free of it — the reported Doppler has
    # to be too, or the phase accumulated from it drifts away from the code range at hundreds
    # of metres per second.
    directory = mktempdir()
    obs_file = joinpath(directory, "drift.obs")
    drift = 7.4e-7
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,);
        approximate_year = 2020,
    )
    GNSSReceiver.log_rinex!(
        logger,
        _rinex_receiver_state(; doppler = -3991.437Hz, clock_drift = drift),
    )
    close(logger)

    _, _, doppler, _ = _obs_fields(last(readlines(obs_file)), 4)
    # Tolerance is the RINEX field's own three decimals.
    @test _obs_value(doppler) ≈ -3991.437 + drift * 1.57542e9 atol = 5e-4
    # A drift of that size is ~1165 Hz at L1 — far too large to be mistaken for noise.
    @test _obs_value(doppler) - (-3991.437) > 1000
end

@testset "RINEX epochs are written once per nominal interval" begin
    directory = mktempdir()
    obs_file = joinpath(directory, "interval.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing, interval = 1.0u"s"),
        ((GPSL1CA(),),),
        (0.0Hz,);
        approximate_year = 2020,
    )
    # Chunks 250 ms apart across four seconds of receive time. At a one-second interval only
    # one epoch per grid point may be written — the first chunk that rounds onto it — so the
    # 16 chunks below (receive times 0.1 s to 3.85 s past the first transmission) must
    # collapse onto the five grid points 0 s … 4 s.
    for chunk = 0:15
        state = _rinex_receiver_state(; receive_offset = 0.25 * chunk + 0.1)
        state = @set state.runtime = (1.0 + 0.25 * chunk) * u"s"
        GNSSReceiver.log_rinex!(logger, state)
    end
    close(logger)

    epochs = filter(line -> startswith(line, ">"), readlines(obs_file))
    @test length(epochs) == 5
    seconds = [parse(Float64, split(line)[7]) for line in epochs]
    # Every epoch lands exactly on an integer second, never in between.
    @test all(iszero, seconds .% 1)
    @test allunique(seconds)
end

@testset "RINEX epoch rate is unaffected by receiver clock drift" begin
    # The epoch grid is in GNSS system time, so the written rate is the configured interval no
    # matter what the receiver's clock does — the drift only decides *which* candidate chunk
    # rounds onto each grid point. Simulated with an absurd 20% drift, which spaces the
    # candidates 0.12 s apart in system time instead of 0.1 s: a rate derived from the
    # receiver's own clock would come out at 1.2 s per epoch, the grid gives exactly 1 s.
    directory = mktempdir()
    obs_file = joinpath(directory, "drifting.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing, interval = 1.0u"s"),
        ((GPSL1CA(),),),
        (0.0Hz,);
        approximate_year = 2020,
    )
    candidate = 0.1 * (1 + 0.2)
    for chunk = 1:60
        state = _rinex_receiver_state(; receive_offset = candidate * chunk)
        state = @set state.runtime = (1.0 + 0.1 * chunk) * u"s"
        GNSSReceiver.log_rinex!(logger, state)
    end
    close(logger)

    epochs = filter(line -> startswith(line, ">"), readlines(obs_file))
    seconds = [parse(Float64, split(line)[7]) for line in epochs]
    # 60 candidates spanning 7.2 s of system time collapse onto the 8 grid points 0 s … 7 s.
    @test length(epochs) == 8
    # Every epoch on an exact integer second, and consecutive: no drifting stamps, no gaps,
    # no duplicates — which is what a post-processing tool expecting a uniform rate needs.
    @test all(iszero, seconds .% 1)
    @test all(≈(1.0), diff(seconds))

    # A grid coarser than the candidate spacing is what makes that work; warn when it is not,
    # since then the drift can slide a candidate off a grid point entirely.
    @test_logs (:warn, r"not comfortably longer") GNSSReceiver.check_epoch_cadence(
        100u"ms",
        100u"ms",
    )
    @test_logs GNSSReceiver.check_epoch_cadence(1.0u"s", 100u"ms")
    @test_logs GNSSReceiver.check_epoch_cadence(1.0u"s", nothing)
end

@testset "RINEX navigation file from a decoded message" begin
    directory = mktempdir()
    nav_file = joinpath(directory, "nav.nav")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = nothing, nav_file),
        ((GPSL1CA(),),),
        (0.0Hz,);
        approximate_year = 2020,
    )
    state = _rinex_receiver_state()
    # Repeated chunks offer the same ephemeris again and again; RINEXParser must write it
    # exactly once, which is what makes it safe to forward every decoded message.
    for chunk = 1:5
        GNSSReceiver.log_rinex!(logger, @set state.runtime = Float64(chunk) * u"s")
    end
    close(logger)

    lines = readlines(nav_file)
    header_end = findfirst(line -> occursin("END OF HEADER", line), lines)
    @test any(
        line -> occursin("N: GNSS NAV DATA", line) && occursin("G: GPS", line),
        lines[1:header_end],
    )
    records = filter(line -> startswith(line, "G"), lines[(header_end+1):end])
    @test length(records) == 1
    @test startswith(only(records), "G13 2020 09 24 00 00 00")
    # One epoch line plus the seven broadcast-orbit lines of a GPS record.
    @test length(lines) - header_end == 8
end

@testset "RINEX files are named by the long filename convention" begin
    # The state of the epoch tests: GPS week 2124 second 82596, 2020-09-20 22:56:36, which
    # is day 264 of 2020 — so a derived name starts `20202642256`.
    state = _rinex_receiver_state()
    _run(config) = begin
        logger = GNSSReceiver.RinexLogger(
            config,
            ((GPSL1CA(),),),
            (0.0Hz,);
            approximate_year = 2020,
        )
        GNSSReceiver.log_rinex!(logger, state)
        close(logger)
        logger
    end

    # Named by the convention by default: the site from the marker name, the data type from
    # the constellation being tracked, the data frequency from the epoch interval.
    directory = mktempdir()
    logger = _run(RinexConfig(; directory, marker_name = "TEST-1"))
    @test basename(logger.obs_path) == "TEST00XXX_R_20202642256_00U_01S_GO.rnx"
    @test basename(logger.nav_path) == "TEST00XXX_R_20202642256_00U_GN.rnx"
    # The name the file was opened under is gone, not left behind beside the final one.
    @test sort(readdir(directory)) ==
          sort([basename(logger.obs_path), basename(logger.nav_path)])
    # And the name is one RINEXParser reads back into the fields it was built from.
    name = parse(RinexFileName, logger.obs_path)
    @test name.station == "TEST"
    @test name.country == "XXX"
    @test name.start_time == DateTime(2020, 9, 20, 22, 56)
    @test name.interval == 1.0
    @test isnothing(name.period)
    @test (name.system, name.kind) == ('G', 'O')

    # A marker name that is a site identification fills every site field of the name, and
    # the collection period and country the receiver cannot know are configuration.
    logger = _run(
        RinexConfig(;
            directory = mktempdir(),
            marker_name = "ROOF12DEU",
            period = Day(1),
            interval = 2.0u"s",
        ),
    )
    @test basename(logger.obs_path) == "ROOF12DEU_R_20202642256_01D_02S_GO.rnx"
    logger = _run(
        RinexConfig(; directory = mktempdir(), marker_name = "TEST-1", country = "DEU"),
    )
    @test basename(logger.obs_path) == "TEST00DEU_R_20202642256_00U_01S_GO.rnx"

    # A period may be a unitful time, like the epoch interval beside it.
    @test GNSSReceiver.rinex_period(1u"hr") == Second(3600)
    @test GNSSReceiver.rinex_period(Day(1)) == Day(1)
    @test isnothing(GNSSReceiver.rinex_period(nothing))
    logger = _run(RinexConfig(; directory = mktempdir(), period = 1u"hr"))
    @test basename(logger.obs_path) == "UNKN00XXX_R_20202642256_01H_01S_GO.rnx"

    # A run with observations switched off stamps no epoch, so its navigation file is named
    # after the first fix — the only absolute time it has.
    logger = _run(
        RinexConfig(; directory = mktempdir(), obs_file = nothing, marker_name = "TEST-1"),
    )
    @test isnothing(logger.obs_path)
    @test basename(logger.nav_path) == "TEST00XXX_R_20202642256_00U_GN.rnx"

    # A run that never got a fix stamped no epoch, so it has no start time of its own to be
    # named after and keeps the name it was opened under — which is a name of the
    # convention too, carrying the wall clock of the run instead.
    directory = mktempdir()
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; directory, marker_name = "TEST-1"),
        ((GPSL1CA(),),),
        (0.0Hz,);
        approximate_year = 2020,
    )
    close(logger)
    @test isnothing(logger.session_start)
    @test isfile(logger.obs_path)
    opened = parse(RinexFileName, logger.obs_path)
    @test opened.station == "TEST"
    # The wall clock of the run, give or take the minute it may have rolled into.
    @test Millisecond(0) <= now(UTC) - opened.start_time < Minute(2)
    @test length(readdir(directory)) == 2

    # A configured name is still a name, taken relative to the directory.
    directory = mktempdir()
    logger = _run(RinexConfig(; directory, obs_file = "roof.obs", nav_file = "roof.nav"))
    @test logger.obs_path == joinpath(directory, "roof.obs")
    @test logger.nav_path == joinpath(directory, "roof.nav")
    @test sort(readdir(directory)) == ["roof.nav", "roof.obs"]
    # An absolute one ignores the directory.
    absolute = joinpath(mktempdir(), "abs.obs")
    logger = _run(RinexConfig(; directory, obs_file = absolute, nav_file = nothing))
    @test logger.obs_path == absolute
    @test isfile(absolute)
end

@testset "a RINEX target and directory that cannot be written" begin
    # A mistyped `:convention` is not a path, so it is rejected at the `receive` call.
    @test_throws ArgumentError GNSSReceiver.rinex_config(
        RinexConfig(; obs_file = :convension),
    )
    @test_throws ArgumentError GNSSReceiver.rinex_config(RinexConfig(; nav_file = :latest))
    @test GNSSReceiver.rinex_config(RinexConfig()) == RinexConfig()

    # A directory that is not there is not created behind the caller's back.
    missing_directory = joinpath(mktempdir(), "not-there")
    @test_throws ArgumentError GNSSReceiver.RinexLogger(
        RinexConfig(; directory = missing_directory),
        ((GPSL1CA(),),),
        (0.0Hz,),
    )
    # Unless nothing is to be written there at all.
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(;
            directory = missing_directory,
            obs_file = nothing,
            nav_file = nothing,
        ),
        ((GPSL1CA(),),),
        (0.0Hz,),
    )
    close(logger)
    @test !isdir(missing_directory)
end

@testset "receive writes valid RINEX when disabled and when enabled" begin
    sampling_freq = 5e6Hz
    system = GPSL1CA()
    num_samples = 20000

    function noise_channel()
        GNSSReceiver.spawn_signal_channel_thread(;
            T = ComplexF64,
            num_samples,
            num_antenna_channels = 1,
        ) do ch
            rng = Random.Xoshiro(4321)
            foreach(_ -> put!(ch, randn(rng, ComplexF64, num_samples, 1) * 512), 1:20)
        end
    end

    # Off by default: not even the default output paths appear, so RINEX output really is
    # opt-in rather than something a caller has to remember to switch off.
    directory = mktempdir()
    cd(directory) do
        data_channel =
            receive(noise_channel(), system, sampling_freq; num_ants = NumAnts(1))
        collect_data(data_channel)
        @test isempty(readdir())
    end

    # Enabled: pure noise never reaches a fix, so there are no epochs — but the files must
    # still be valid, header-only RINEX rather than empty or truncated.
    obs_file = joinpath(directory, "noise.obs")
    nav_file = joinpath(directory, "noise.nav")
    data_channel = receive(
        noise_channel(),
        system,
        sampling_freq;
        num_ants = NumAnts(1),
        write_rinex = RinexConfig(; obs_file, nav_file),
    )
    collect_data(data_channel)

    obs_lines = readlines(obs_file)
    @test occursin("END OF HEADER", last(obs_lines))
    @test any(line -> occursin("OBSERVATION DATA", line), obs_lines)
    nav_lines = readlines(nav_file)
    @test occursin("END OF HEADER", last(nav_lines))
    @test any(line -> occursin("N: GNSS NAV DATA", line), nav_lines)

    # `write_rinex = true` is the same thing with the default paths.
    @test GNSSReceiver.rinex_config(true) == RinexConfig()
    @test isnothing(GNSSReceiver.rinex_config(false))
end
