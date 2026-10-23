# Note: this file is included from runtests.jl which provides all `using` statements.

# Fractional part of a carrier phase as `Tracking` reports it: wrapped into one cycle
# around zero.
_wrap(phase) = mod(phase + 0.5, 1) - 0.5

# GPS week 2124 (the week 10-bit `WN = 76` resolves to around mid-2020) starts on
# Sunday 2020-09-20; the reference times below are four days into it.
const _GPS_WEEK = 2124
const _GPS_TOW = 4 * 86400
const _GPS_TOC = DateTime(2020, 9, 24)

# A GPS LNAV data set carrying every field `is_decoding_completed_for_positioning` requires,
# so the logger writes its ephemeris. The orbital elements are a real broadcast set (GPS
# PRN 5) in the units the decoder stores. The subframe 4 page 18 ionosphere / UTC parameters
# are passed in only where a test needs them.
_gps_lnav_data(; kwargs...) = GNSSDecoder.GPSL1CAData(;
    last_subframe_id = 3,
    integrity_status_flag = false,
    TOW = 82596,
    alert_flag = false,
    anti_spoof_flag = true,
    # Subframe 1 — clock correction and satellite status
    WN = 76,
    code_on_L2 = 1,
    # URA index 0, the 2.0 m the record below expects.
    URA_index = 0,
    sv_health = 0,
    IODC = 63,
    L2_P_data_flag = false,
    T_GD = -1.071020960808e-8,
    t_0c = _GPS_TOW,
    a_f0 = -2.989266067743e-5,
    a_f1 = 1.364242052659e-12,
    a_f2 = 0.0,
    # Subframe 2 — ephemeris, part 1
    IODE_Sub_2 = 63,
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
    IODE_Sub_3 = 63,
    i_dot = -2.23225e-10,      # rad/s
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
    obs_codes, layouts = GNSSReceiver.rinex_layout(((GPSL1CA(),),), (0.0Hz,), 2.048e6Hz)
    @test obs_codes == ['G' => ["C1C", "L1C", "D1C", "S1C"]]
    @test layouts[:GPSL1CA].system == 'G'
    @test layouts[:GPSL1CA].codes == (C = "C1C", L = "L1C", D = "D1C", S = "S1C")
    @test layouts[:GPSL1CA].interm_freq == 0.0
    @test layouts[:GPSL1CA].carrier_frequency == 1.57542e9
    # The wavelength the phase and range-rate conversions use.
    @test GNSSReceiver.wavelength(layouts[:GPSL1CA]) ≈ 0.1902936728 atol = 1e-9

    # Two constellations on one band: one header entry each, in order of appearance. The
    # `CombinedSignal` is keyed and coded by its ranging signal — the pilot E1C — because
    # that is what the receiver actually measures on.
    obs_codes, layouts = GNSSReceiver.rinex_layout(
        ((GPSL1CA(), CombinedSignal(GalileoE1C(), GalileoE1B())),),
        (0.0Hz,),
        2.048e6Hz,
    )
    @test obs_codes ==
          ['G' => ["C1C", "L1C", "D1C", "S1C"], 'E' => ["C1C", "L1C", "D1C", "S1C"]]
    @test layouts[:GalileoE1C].system == 'E'
    @test layouts[:GalileoE1C].codes.L == "L1C"
    @test !haskey(layouts, :GalileoE1B)

    # Two bands of one constellation share a single header entry, the second band's
    # observables following the first's — and each band keeps its own intermediate
    # frequency, which the carrier-phase accumulation needs.
    obs_codes, layouts = GNSSReceiver.rinex_layout(
        ((GPSL1CA(),), (CombinedSignal(GPSL5Q(), GPSL5I()),)),
        (0.0Hz, 1.0e6Hz),
        2.048e6Hz,
    )
    @test obs_codes == ['G' => ["C1C", "L1C", "D1C", "S1C", "C5Q", "L5Q", "D5Q", "S5Q"]]
    @test layouts[:GPSL1CA].codes.L == "L1C"
    @test layouts[:GPSL5Q].codes.L == "L5Q"
    @test layouts[:GPSL5Q].interm_freq == 1.0e6
    # Nothing here tracks column positions any more: `SatObs` derives each observable's
    # place from the header, so a second band's types only have to reach the header.
    @test length(last(only(obs_codes))) == 8
end

@testset "RINEX carrier phases are aligned with their bands' reference signals" begin
    # RINEX 3.05 Table A23: every phase code this receiver can write is either its band's
    # reference signal or carries the correction that aligns it with that reference — the
    # table's, from RINEXParser, or this receiver's where the table gives none.
    # Enumerated from GNSSSignals, so a signal added upstream is covered without a list here.
    leaves(T) = isabstracttype(T) ? vcat(map(leaves, subtypes(T))...) : [T]
    signals = filter(!isnothing, map(leaves(AbstractGNSSSignal)) do T
        try
            T()
        catch
            nothing
        end
    end)
    @test length(signals) >= 20
    for signal in signals
        signal_id = get_signal_id(signal)
        signal_id == :GalileoE5aQP && continue
        code = observation_code('L', signal)
        shift = GNSSReceiver.rinex_phase_shift(RINEXParser.satellite_system(signal), code)
        @test isnothing(shift) || shift isa Float64
    end
    shift = GNSSReceiver.rinex_phase_shift
    @test isnothing(shift('G', "L1C"))
    @test shift('G', "L5Q") == -0.25
    @test shift('G', "L2L") == -0.25
    @test shift('E', "L1C") == 0.5
    @test shift('E', "L5Q") == -0.25
    @test isnothing(shift('C', "L1D"))
    @test shift('C', "L1P") == 0.25
    # A code the table gives no correction for, and this receiver none either, cannot be
    # written aligned.
    @test_throws ArgumentError shift('G', "L5X")

    # Ranging on a pilot writes its correction in the header; a reference signal is written
    # blank, as RINEXParser does for a code no record names.
    _, layouts = GNSSReceiver.rinex_layout(
        (
            (GPSL1CA(), CombinedSignal(GalileoE1C(), GalileoE1B())),
            (
                CombinedSignal(GPSL5Q(), GPSL5I()),
                CombinedSignal(GalileoE5aQ(), GalileoE5aI()),
            ),
        ),
        (0.0Hz, 0.0Hz),
        2.048e6Hz,
    )
    @test isnothing(layouts[:GPSL1CA].phase_shift)
    @test [
        (shift.system, shift.code, shift.correction) for
        shift in GNSSReceiver.rinex_phase_shifts(layouts)
    ] == [('E', "L1C", 0.5), ('E', "L5Q", -0.25), ('G', "L5Q", -0.25)]
    directory = mktempdir()
    obs_file = joinpath(directory, "aligned.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing),
        ((GPSL1CA(), CombinedSignal(GalileoE1C(), GalileoE1B())),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    close(logger)
    records = filter(line -> occursin("SYS / PHASE SHIFT", line), readlines(obs_file))
    @test any(line -> startswith(line, "G L1C ") && isempty(strip(line[6:60])), records)
    @test any(line -> startswith(line, "E L1C  0.50000"), records)
end

@testset "A signal RINEX 3.05 cannot name is refused" begin
    # E5a-QP postdates RINEX 3.05, so asking for it says that, and what to do about it here,
    # rather than looking like an omission.
    error = try
        GNSSReceiver.rinex_layout(((GalileoE5aQP(),),), (0.0Hz,), 2.048e6Hz)
    catch exception
        exception
    end
    @test error isa ArgumentError
    message = sprint(showerror, error)
    @test occursin("OS SIS ICD Issue 2.2", message)
    @test occursin("Track it without RINEX output", message)
end

@testset "Two signals written under the same codes are refused" begin
    # The BOC(1,1) approximation of E1-B is written as E1-B itself, so a satellite tracked on
    # both would give each code twice in one record.
    error = try
        GNSSReceiver.rinex_layout(
            ((GalileoE1B(), GalileoE1B_BOC11()),),
            (0.0Hz,),
            2.048e6Hz,
        )
    catch exception
        exception
    end
    @test error isa ArgumentError
    @test occursin("Range on one of them", sprint(showerror, error))
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
        GNSSReceiver.CarrierPhaseAccumulator(_wrap(phase(0.0)), doppler(0.0), 0.0, 1)
    @test accumulator.restarted
    @test isnan(accumulator.anchor)
    @test accumulator.phase == 0.0

    for chunk = 2:251
        t = (chunk - 1) * Δt
        GNSSReceiver.advance!(
            accumulator,
            _wrap(phase(t)),
            doppler(t),
            t,
            chunk,
            interm_freq,
        )
    end
    t_end = 250 * Δt
    @test accumulator.last_chunk == 251
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
        1,
    )
    for chunk = 2:251
        t = (chunk - 1) * Δt
        GNSSReceiver.advance!(
            accumulator,
            _wrap(fast_phase(t)),
            fast_doppler(t),
            t,
            chunk,
            0.0,
        )
    end
    @test accumulator.phase ≈ fast_phase(t_end) atol = 1e-9
end

@testset "The carrier beat phase and its polarity" begin
    # The arc starts from the measured beat phase: the replica's fractional phase with the IF
    # ramp of the chunk's end taken out, so every satellite's is against the same oscillator.
    interm_freq = 1.0e5 + 0.1
    accumulator = GNSSReceiver.CarrierPhaseAccumulator(0.3, 0.0, 2.5, 1; interm_freq)
    @test accumulator.beat_phase ≈ 0.05 atol = 1e-6
    @test accumulator.polarity == 0
    @test GNSSReceiver.carrier_beat_phase(accumulator) == accumulator.beat_phase

    # And carries it on: the beat phase now is the replica's, IF ramp out, with every cycle.
    phase(t) = (2000.0 + interm_freq) * t - 150.0 * t^2
    accumulator =
        GNSSReceiver.CarrierPhaseAccumulator(_wrap(phase(0.0)), 2000.0, 0.0, 1; interm_freq)
    for chunk = 2:251
        t = (chunk - 1) * 0.004
        GNSSReceiver.advance!(
            accumulator,
            _wrap(phase(t)),
            2000.0 - 300.0 * t,
            t,
            chunk,
            interm_freq,
        )
    end
    @test GNSSReceiver.carrier_beat_phase(accumulator) ≈ phase(1.0) - interm_freq atol =
        1e-6

    # Half a cycle on where the Costas loop locked off the carrier.
    locked_off = GNSSReceiver.CarrierPhaseAccumulator(0.3, 0.0, 0.0, 1; polarity = -1)
    @test GNSSReceiver.carrier_beat_phase(locked_off) ≈ 0.8
    locked_on = GNSSReceiver.CarrierPhaseAccumulator(0.3, 0.0, 0.0, 1; polarity = 1)
    @test GNSSReceiver.carrier_beat_phase(locked_on) ≈ 0.3

    # A polarity learnt before the arc's first epoch is latched; an unknown one changes
    # nothing; one that changes within the arc, or arrives after epochs were written without
    # it, ends the arc.
    fresh() = GNSSReceiver.CarrierPhaseAccumulator(0.3, 0.0, 0.0, 1)
    latched = fresh()
    @test GNSSReceiver.continues_arc(latched, Int8(-1), false)
    GNSSReceiver.take_polarity!(latched, Int8(-1))
    @test latched.polarity == -1
    GNSSReceiver.take_polarity!(latched, Int8(0))
    @test latched.polarity == -1
    @test GNSSReceiver.continues_arc(latched, Int8(0), false)
    @test GNSSReceiver.continues_arc(latched, Int8(-1), false)
    @test !GNSSReceiver.continues_arc(latched, Int8(1), false)
    written = fresh()
    written.anchor = 1.0e8
    @test GNSSReceiver.continues_arc(written, Int8(0), false)
    @test !GNSSReceiver.continues_arc(written, Int8(1), false)

    # In the vector loop the residual PLL's correction, every cycle counted, brings the
    # replica onto the carrier.
    vector = GNSSReceiver.CarrierPhaseAccumulator(
        0.3,
        0.0,
        0.0,
        1;
        polarity = 1,
        correction = 17.25,
        in_vector_loop = true,
    )
    @test vector.in_vector_loop
    @test GNSSReceiver.carrier_beat_phase(vector) ≈ 0.3 + 17.25
    # Joining the vector loop keeps the arc; leaving it ends it, as the correction drops to
    # zero and the scalar loop pulls the replica in afresh.
    scalar = fresh()
    @test GNSSReceiver.continues_arc(scalar, Int8(0), true)
    @test GNSSReceiver.continues_arc(scalar, Int8(0), false)
    @test GNSSReceiver.continues_arc(vector, Int8(0), true)
    @test !GNSSReceiver.continues_arc(vector, Int8(0), false)
end

@testset "A pilot's polarity comes from its secondary code" begin
    # The sync of a pilot's secondary code reads the carrier's polarity off the prompts, so it
    # is known without the data component's decoder having synchronised.
    system = CombinedSignal(GalileoE1C(), GalileoE1B())
    key = get_signal_id(GalileoE1C())
    track_state =
        GNSSReceiver.ReceiverState(
            ComplexF64,
            system;
            num_samples_for_acquisition = 20000,
            num_ants = NumAnts(1),
        ).track_state
    sat = GNSSReceiver.create_tracked_sat(
        GNSSReceiver.tracking_signals(system),
        11,
        0.0,
        0.0Hz,
        NumAnts(1),
        track_state.doppler_estimator,
    )
    synced(found, polarity) = merge_sats(
        track_state,
        key,
        [
            @set sat.signals[1].loop_state.bit_buffer = setproperties(
                sat.signals[1].loop_state.bit_buffer;
                found,
                polarity = Int8(polarity),
            )
        ],
    )
    _, layouts = GNSSReceiver.rinex_layout(((system,),), (0.0Hz,), 2.048e6Hz)
    decoder = GNSSDecoder.GNSSDecoderState(GalileoE1B(), 11)
    @test isnothing(decoder.num_bits_after_valid_syncro_sequence)
    polarity(state) = GNSSReceiver.carrier_polarity(state, key, 11, layouts[key], decoder)
    @test polarity(synced(true, 1)) == 1
    @test polarity(synced(true, -1)) == -1
    @test polarity(synced(false, 0)) == 0
end

@testset "The receive time of a solution epoch" begin
    # The time of week comes off in whole seconds first, so a solution epoch keeps its
    # precision: at present-day weeks the absolute count in one Float64 would round it to a
    # fifth of a microsecond.
    origin = GNSSReceiver.gps_time_origin()
    time = TAITime(origin.second + _GPS_WEEK * 604800 + 82596, 0.123456789123)
    @test GNSSReceiver.receive_time_of_week(time, GNSSSignals.GPST()) ≈ 82596.123456789123 atol =
        1e-12
    # On BDT's count, 14 s behind GPS Time.
    @test GNSSReceiver.receive_time_of_week(time, GNSSSignals.BDT()) ≈ 82582.123456789123 atol =
        1e-12
    # The scalar PVT's solution is dated at the end of the chunk it solved on.
    @test GNSSReceiver.solution_runtime(ConventionalPLLAndDLL(), 34.504) == 34.504
end

@testset "BeiDou observations are differenced on one time scale" begin
    # BDT runs 14 s behind GPS Time, and PositionVelocityTime leaves every transmit time
    # on its own constellation's scale. Differencing them as they come would put 14 s —
    # 4.2 million metres — into a BeiDou pseudorange.
    @test GNSSReceiver.time_scale_offset(GNSSSignals.GPST(), GNSSSignals.BDT()) == 14.0
    @test GNSSReceiver.time_scale_offset(GNSSSignals.BDT(), GNSSSignals.GPST()) == -14.0
    # GPS and Galileo both count TAI - 19 s, which is why a receiver of those two alone
    # never had to correct anything.
    @test GNSSReceiver.time_scale_offset(GNSSSignals.GPST(), GNSSSignals.GST()) == 0.0
end

@testset "RINEX epoch and time helpers" begin
    # Times of week are differenced across the week boundary: a satellite that transmitted
    # just before the roll-over still yields a sane travel time.
    @test GNSSReceiver.week_difference(0.5, 604799.93) ≈ 0.57
    @test GNSSReceiver.week_difference(604799.93, 0.5) ≈ -0.57
    @test GNSSReceiver.week_difference(82621.7, 82621.63) ≈ 0.07

    # The week count of each time system starts this many GPS weeks in.
    @test GNSSReceiver.week_offset(GNSSSignals.GPST()) == 0
    @test GNSSReceiver.week_offset(GNSSSignals.GST()) == 1024
    @test GNSSReceiver.week_offset(GNSSSignals.BDT()) == 1356

    # The PVT solution reports its epoch on the atomic scale; the round trip back to GPS
    # system-time seconds is what recovers the week the epoch falls in.
    total = _GPS_WEEK * 604800 + _GPS_TOW + 0.25
    origin = GNSSReceiver.gps_time_origin()
    epoch = TAITime(floor(Int, total) + origin.second, total - floor(total))
    @test GNSSReceiver.gps_seconds(epoch) ≈ total
    @test round(Int, (GNSSReceiver.gps_seconds(epoch) - (_GPS_TOW + 0.25)) / 604800) ==
          _GPS_WEEK

    # Signal-strength indicator: the spec's 1-9 scale in 6 dB-Hz steps, and blank for a CN0
    # estimate that has not converged, whichever non-finite value it holds.
    @test signal_strength_indicator(45.0) == 7
    @test signal_strength_indicator(54.0) == 9
    @test signal_strength_indicator(90.0) == 9
    @test signal_strength_indicator(0.0) == 1
    @test signal_strength_indicator(NaN) === nothing
    @test signal_strength_indicator(-Inf) === nothing
    # A non-finite observable is written as a blank field by RINEXParser, so an unconverged
    # CN0 reaches it as an `ObsValue` and comes out absent rather than as an unparsable one.
    @test ObsValue(NaN; ssi = nothing).value === NaN
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
    phase_locked = true,
    data = _gps_lnav_data(),
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
    sat = GNSSReceiver.create_tracked_sat(
        GNSSReceiver.tracking_signals(system),
        prn,
        0.0,
        doppler,
        NumAnts(1),
        receiver_state.track_state.doppler_estimator,
    )
    # The carrier loop's phase-lock indicator, which a fresh loop has not filled yet: a
    # phase-locked loop reads near 1, a spinning phase near 0.
    sat = @set sat.doppler_estimator_state.staging.lock_detector.indicator =
        phase_locked ? 1.0 : 0.0
    track_state = merge_sats(receiver_state.track_state, key, [sat])
    sat_state = GNSSReceiver.ReceiverSatState(
        prn,
        _decoder_with(system, prn, data),
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
    # The receive instant in whole and fractional seconds, as a solver builds it: one
    # Float64 of the absolute count would round it to a fifth of a microsecond.
    pvt = PositionVelocityTime.PVTSolution(;
        position = Geodesy.ECEF(4.1e6, 6.0e5, 4.8e6),
        time_correction = -travel_time * 299792458.0 * u"m",
        time = TAITime(
            _GPS_WEEK * 604800 +
            floor(Int, receive_time) +
            GNSSReceiver.gps_time_origin().second,
            receive_time - floor(receive_time),
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
        # Scalar tracking: no navigation cycle copied from a vector loop. RINEX is written
        # from the tracking loops and the PVT solution, which both modes produce alike.
        0,
        1.0u"s",
        1.0u"s",
        nothing,
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
        RinexConfig(; obs_file, nav_file, obs_header = (; marker_name = "TEST-1")),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
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
    # The C/N₀ observations are in dB-Hz, and the steered epochs carry the receiver clock
    # offset applied.
    @test any(
        line -> startswith(line, "DBHZ") && occursin("SIGNAL STRENGTH UNIT", line),
        header,
    )
    @test any(
        line -> startswith(line, "     1") && occursin("RCV CLOCK OFFS APPL", line),
        header,
    )

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
    # The phase ambiguity is anchored on that same range, in cycles of the L1 wavelength, and
    # is whole: the replica starts at phase zero, so the written phase has no fraction.
    wavelength = 299792458.0 / 1.57542e9
    @test _obs_value(phase) ≈ 0.07 * 299792458.0 / wavelength rtol = 1e-9
    @test isinteger(_obs_value(phase))
    # First epoch of the arc, so the phase carries the loss-of-lock indicator; and as the
    # bit buffer of this hand-built state never synchronised, the carrier's polarity is not
    # known, so it carries the half-cycle bit as well.
    @test _obs_lli(phase) == '3'
    @test _obs_value(doppler) == 0.0
end

@testset "A mixed file is written in GPS time" begin
    # The PVT solution's reference system is whichever constellation has the most satellites
    # in view. A mixed file is labelled GPS time, so its epochs must be GPS time whatever the
    # solution's reference is: a BeiDou one would put them 14 s early.
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = joinpath(mktempdir(), "mixed.obs"), nav_file = nothing),
        ((GPSL1CA(),), (BeiDouB1I(),)),
        (0.0Hz, 0.0Hz),
        2.048e6Hz;
        approximate_year = 2020,
    )
    @test logger.time_system == GNSSSignals.GPST()
    state = _rinex_receiver_state()
    state = @set state.pvt.reference_system = GNSSSignals.BDT()
    timing = GNSSReceiver.obs_epoch_timing(logger, state)
    @test timing.time_of_week == _TRANSMIT_TOW
    @test timing.receive_time ≈ _TRANSMIT_TOW + 0.07
    close(logger)

    # A single-constellation file is in that constellation's time.
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = nothing, nav_file = nothing),
        ((BeiDouB1I(),),),
        (0.0Hz,),
        2.048e6Hz,
    )
    @test logger.time_system == GNSSSignals.BDT()
    close(logger)
end

@testset "Epochs are written in the file's own time system" begin
    # The epoch time of a file of one constellation, from a solution 70 ms past GPS second
    # 82596 of week 2124 (2020-09-20 22:56:36): the same instant in GPS and Galileo time,
    # 14 s earlier in BDT. The last case lies in the first 14 s of the GPS week, so its BDT
    # epoch is still in the week before.
    function written_epoch(bands, receive_offset)
        logger = GNSSReceiver.RinexLogger(
            RinexConfig(;
                obs_file = joinpath(mktempdir(), "epoch.obs"),
                nav_file = nothing,
            ),
            bands,
            map(_ -> 0.0Hz, bands),
            2.048e6Hz;
            approximate_year = 2020,
        )
        state = _rinex_receiver_state(; receive_offset)
        timing = GNSSReceiver.obs_epoch_timing(logger, state)
        epoch = epoch_time(
            logger.time_system;
            week = timing.week,
            time_of_week = timing.time_of_week,
        )
        close(logger)
        timing, epoch
    end
    for (bands, receive_offset, week, time_of_week, time) in (
        (((GPSL1CA(),),), 0.0, 2124, 82596.0, DateTime(2020, 9, 20, 22, 56, 36)),
        (((GalileoE1B(),),), 0.0, 2124 - 1024, 82596.0, DateTime(2020, 9, 20, 22, 56, 36)),
        (((BeiDouB1I(),),), 0.0, 2124 - 1356, 82582.0, DateTime(2020, 9, 20, 22, 56, 22)),
        (
            ((BeiDouB1I(),),),
            5.0 - 82596.0,
            2124 - 1356,
            -9.0,
            DateTime(2020, 9, 19, 23, 59, 51),
        ),
    )
        timing, epoch = written_epoch(bands, receive_offset)
        @test (timing.week, timing.time_of_week) == (week, time_of_week)
        @test epoch.time == time
        @test epoch.fractional_second == 0.0
    end
end

@testset "A navigation message RINEX 3.05 has no record for is reported at once" begin
    # GPS L5 broadcasts CNAV, which RINEX 3.05 cannot express: the warning comes on the first
    # chunk, long before the message is complete, and only once.
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = nothing, nav_file = joinpath(mktempdir(), "l5.nav")),
        ((GPSL5I(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    state =
        (; receiver_sat_states = (; GPSL5I = [(; decoder = GNSSDecoderState(GPSL5I(), 3))]))
    ephemerides, _ =
        @test_logs (:warn, r"no navigation record.*GPSL5I") GNSSReceiver.build_nav_records(
            logger,
            state,
        )
    @test isempty(ephemerides)
    @test_logs GNSSReceiver.build_nav_records(logger, state)
    close(logger)
end

@testset "A satellite without a phase arc keeps its other observables" begin
    # Out of phase lock the satellite has no arc, but it is in the fix: its pseudorange,
    # Doppler and C/N₀ are written, and its phase is left blank.
    obs_file = joinpath(mktempdir(), "no_arc.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    GNSSReceiver.log_rinex!(logger, _rinex_receiver_state(; phase_locked = false))
    @test isempty(logger.accumulators)
    close(logger)
    sat_line = last(readlines(obs_file))
    @test startswith(sat_line, "G13")
    pseudorange, phase, _, _ = _obs_fields(rpad(sat_line, 3 + 4 * 16), 4)
    @test _obs_value(pseudorange) ≈ 0.07 * 299792458.0 rtol = 1e-9
    @test isempty(strip(phase))
end

@testset "A value too wide for its field is written blank" begin
    # A pseudorange off a wrong time of week does not fit its field. It is written blank,
    # with the satellite's other observables and every later epoch kept.
    obs_file = joinpath(mktempdir(), "overflow.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    bad = _rinex_receiver_state(; travel_time = 1.0e3, doppler = 1000.0Hz)
    GNSSReceiver.log_rinex!(logger, @set bad.runtime = 1.0u"s")
    # Another satellite: the first one's phase arc is anchored on the bad range.
    good = _rinex_receiver_state(;
        prn = 14,
        receive_offset = 1001.0,
        data = _gps_lnav_data(; TOW = 82596 + 1001),
    )
    GNSSReceiver.log_rinex!(logger, @set good.runtime = 2.0u"s")
    close(logger)
    @test !istaskfailed(logger.task)
    lines = readlines(obs_file)
    epochs = findall(line -> startswith(line, ">"), lines)
    @test length(epochs) == 2
    pseudorange, phase, doppler, _ =
        _obs_fields(rpad(lines[first(epochs)+1], 3 + 4 * 16), 4)
    @test isempty(strip(pseudorange)) && isempty(strip(phase))
    @test _obs_value(doppler) == 1000.0
    pseudorange, _, _, _ = _obs_fields(last(lines), 4)
    @test _obs_value(pseudorange) ≈ 0.07 * 299792458.0 rtol = 1e-9
end

@testset "An epoch interval shorter than a chunk is reported" begin
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(;
            obs_file = joinpath(mktempdir(), "fine.obs"),
            nav_file = nothing,
            interval = 0.1u"s",
        ),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    state = _rinex_receiver_state()
    GNSSReceiver.log_rinex!(logger, @set state.runtime = 1.0u"s")
    @test_logs (:warn, r"shorter than a processed chunk") GNSSReceiver.log_rinex!(
        logger,
        @set state.runtime = 1.25u"s"
    )
    # Once.
    @test_logs GNSSReceiver.log_rinex!(logger, @set state.runtime = 1.5u"s")
    close(logger)
end

@testset "A phase arc needs the carrier loop in phase lock" begin
    # No arc starts out of phase lock.
    @test GNSSReceiver.phase_measurable(nothing, true, false, true, 1.0)
    @test !GNSSReceiver.phase_measurable(nothing, false, false, true, 1.0)
    # In the vector loop the arc ends on the first chunk out of phase lock, and on the first
    # its signal is gone — a concealed satellite is coasted on the filter's prediction, which
    # measures no phase — locked or not.
    vector = GNSSReceiver.CarrierPhaseAccumulator(
        0.3,
        0.0,
        1.0,
        1;
        polarity = 1,
        correction = 2.0,
        in_vector_loop = true,
    )
    @test GNSSReceiver.phase_measurable(vector, true, true, true, 1.004)
    @test !GNSSReceiver.phase_measurable(vector, true, true, false, 1.004)
    @test !GNSSReceiver.phase_measurable(vector, false, true, true, 1.004)
    @test !GNSSReceiver.phase_measurable(vector, false, true, false, 1.004)
    # A scalar loop's verdict dips now and then on a locked signal; only 100 ms out of lock in
    # a row end its arc. Its signal presence is no matter.
    scalar = GNSSReceiver.CarrierPhaseAccumulator(0.3, 0.0, 1.0, 1)
    @test GNSSReceiver.phase_measurable(scalar, false, false, false, 1.004)
    scalar.unlocked_time = 0.092
    @test GNSSReceiver.phase_measurable(scalar, false, false, true, 1.004)
    @test !GNSSReceiver.phase_measurable(scalar, false, false, true, 1.008)

    # Through the logger: a satellite whose carrier loop is out of phase lock gets no arc,
    # and so no observations.
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = joinpath(mktempdir(), "unlocked.obs"), nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    GNSSReceiver.log_rinex!(logger, _rinex_receiver_state(; phase_locked = false))
    @test isempty(logger.accumulators)
    # Locked, it starts one; out of lock on the next chunks, the scalar arc counts the time
    # until it reaches 100 ms.
    at(state, runtime) = @set state.runtime = runtime * u"s"
    GNSSReceiver.log_rinex!(logger, at(_rinex_receiver_state(), 2.0))
    @test only(values(logger.accumulators)).unlocked_time == 0.0
    GNSSReceiver.log_rinex!(logger, at(_rinex_receiver_state(; phase_locked = false), 2.04))
    @test only(values(logger.accumulators)).unlocked_time ≈ 0.04
    GNSSReceiver.log_rinex!(logger, at(_rinex_receiver_state(; phase_locked = false), 2.1))
    @test isempty(logger.accumulators)
    close(logger)
end

@testset "RINEX output carries the broadcast leap seconds" begin
    # Once the UTC page is decoded the leap seconds reach the observation header, in
    # RINEXParser's own record, and the writer takes them without failing.
    directory = mktempdir()
    obs_file = joinpath(directory, "leap.obs")
    nav_file = joinpath(directory, "leap.nav")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    data = _gps_lnav_data(; Δt_LS = 18, Δt_LSF = 18, WN_LSF = 137, DN = 7)
    GNSSReceiver.log_rinex!(logger, _rinex_receiver_state(; data))
    close(logger)
    @test istaskdone(logger.task) && !istaskfailed(logger.task)
    lines = readlines(obs_file)
    leap = lines[findfirst(line -> occursin("LEAP SECONDS", line), lines)]
    @test parse.(Int, split(leap[1:24])) == [18, 18, 2185, 7]
    @test count(line -> startswith(line, ">"), lines) == 1
    @test any(line -> occursin("LEAP SECONDS", line), readlines(nav_file))
end

@testset "A pilot's phase is written aligned with its band's reference signal" begin
    # The same satellite as above, but as if its ranging signal were a pilot a quarter cycle
    # off its band's reference: the phase carries the correction, and its fraction with it.
    directory = mktempdir()
    obs_file = joinpath(directory, "shifted.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    layout = logger.layouts[:GPSL1CA]
    logger.layouts[:GPSL1CA] = @set layout.phase_shift = -0.25
    GNSSReceiver.log_rinex!(logger, _rinex_receiver_state())
    close(logger)
    _, phase, _, _ = _obs_fields(last(readlines(obs_file)), 4)
    wavelength = 299792458.0 / 1.57542e9
    @test _obs_value(phase) ≈ round(0.07 * 299792458.0 / wavelength) - 0.25 atol = 1e-3
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
        (0.0Hz,),
        2.048e6Hz;
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
        (0.0Hz,),
        2.048e6Hz;
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

@testset "A data polarity counts for an arc only from a preamble within it" begin
    _, layouts = GNSSReceiver.rinex_layout(
        ((GPSL1CA(), CombinedSignal(GalileoE1C(), GalileoE1B())),),
        (0.0Hz,),
        2.048e6Hz,
    )
    # The decoder read its polarity off a preamble within the arc, or before it, where the
    # loop may have slipped half a cycle since.
    @test GNSSReceiver.arc_polarity(Int8(-1), layouts[:GPSL1CA], 5.0, 4.0) == -1
    @test GNSSReceiver.arc_polarity(Int8(-1), layouts[:GPSL1CA], 3.0, 4.0) == 0
    # A pilot's polarity is read off every prompt.
    @test GNSSReceiver.arc_polarity(Int8(-1), layouts[:GalileoE1C], -Inf, 4.0) == -1

    # A preamble restarts the decoder's bit count; a one-bit move of it, which the vector
    # loop makes to follow the code phase, does not count as one.
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = nothing, nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    decoder = _decoder_with(GPSL1CA(), 13, _gps_lnav_data())
    synced(bits, runtime) = GNSSReceiver.note_decoder_sync!(
        logger,
        (:GPSL1CA, 13),
        GNSSDecoder.GNSSDecoderState(decoder; num_bits_after_valid_syncro_sequence = bits),
        runtime,
    )
    @test synced(nothing, 1.0) == -Inf
    @test synced(10, 2.0) == 2.0
    @test synced(60, 3.0) == 2.0
    @test synced(59, 3.004) == 2.0
    @test synced(5, 4.0) == 4.0
    @test synced(nothing, 5.0) == -Inf
    close(logger)
end

@testset "One clock correction for every satellite's phase" begin
    # The receiver's oscillator drift is integrated from the first chunk for every satellite
    # alike, filled in with the first solution's drift up to it.
    unsolved = (; time = nothing)
    clock = GNSSReceiver.ReceiverClockPhase()
    GNSSReceiver.update_clock_phase!(clock, unsolved, 0.5)
    GNSSReceiver.update_clock_phase!(clock, unsolved, 0.7)
    @test clock.phase == 0.0
    GNSSReceiver.update_clock_phase!(
        clock,
        (; time = 1, relative_clock_drift = 2.0e-7),
        1.0,
    )
    @test clock.phase ≈ 2.0e-7 * 0.5
    GNSSReceiver.update_clock_phase!(
        clock,
        (; time = 1, relative_clock_drift = 4.0e-7),
        2.0,
    )
    @test clock.phase ≈ 2.0e-7 * 0.5 + 3.0e-7 * 1.0

    # So an arc that starts later is corrected by the drift from before it too: its phase is
    # a whole number of cycles off the receiver-wide correction, as an arc that started with
    # the first chunk would be. Integrated over the arc alone, it would be off by a fraction.
    obs_file = joinpath(mktempdir(), "late_arc.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    drift = 7.4e-7
    at(state, runtime) = @set state.runtime = runtime * u"s"
    GNSSReceiver.log_rinex!(logger, at(_rinex_receiver_state(; clock_drift = drift), 1.0))
    # Out of phase lock for half a second: the first arc ends.
    GNSSReceiver.log_rinex!(
        logger,
        at(
            _rinex_receiver_state(;
                receive_offset = 0.5,
                clock_drift = drift,
                phase_locked = false,
            ),
            1.5,
        ),
    )
    @test isempty(logger.accumulators)
    GNSSReceiver.log_rinex!(
        logger,
        at(_rinex_receiver_state(; receive_offset = 1.0, clock_drift = drift), 2.0),
    )
    close(logger)
    lines = readlines(obs_file)
    @test count(line -> startswith(line, ">"), lines) == 2
    _, phase, doppler, _ = _obs_fields(last(lines), 4)
    @test _obs_lli(phase) == '3'
    # The written phase is the anchor less the beat phase, carried back the 70 ms from the
    # measurement onto the epoch at the Doppler; the beat phase is the correction alone.
    cycles = 1.57542e9 * drift * 1.0
    propagation = -0.07
    @test abs(
        rem(
            _obs_value(phase) + cycles + _obs_value(doppler) * propagation,
            1.0,
            RoundNearest,
        ),
    ) < 2e-3
end

@testset "RINEX epochs are written once per nominal interval" begin
    directory = mktempdir()
    obs_file = joinpath(directory, "interval.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing, interval = 1.0u"s"),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    # Chunks 250 ms apart across four seconds of receive time. At a one-second interval only
    # one epoch per grid point may be written — on the chunk that crosses it — so the 16
    # chunks below (receive times 0.17 s to 3.92 s past the integer second) must collapse
    # onto the four grid points 0 s … 3 s, the first written on the first chunk.
    for chunk = 0:15
        state = _rinex_receiver_state(; receive_offset = 0.25 * chunk + 0.1)
        state = @set state.runtime = (1.0 + 0.25 * chunk) * u"s"
        GNSSReceiver.log_rinex!(logger, state)
    end
    close(logger)

    epochs = filter(line -> startswith(line, ">"), readlines(obs_file))
    @test length(epochs) == 4
    seconds = [parse(Float64, split(line)[7]) for line in epochs]
    # Every epoch lands exactly on an integer second, never in between.
    @test all(iszero, seconds .% 1)
    @test allunique(seconds)
end

@testset "RINEX epoch rate is unaffected by receiver clock drift" begin
    # The epoch grid is in GNSS system time, so the written rate is the configured interval no
    # matter what the receiver's clock does — the drift only decides *which* chunk crosses
    # each grid point. Simulated with an absurd 20% drift, which spaces the
    # candidates 0.12 s apart in system time instead of 0.1 s: a rate derived from the
    # receiver's own clock would come out at 1.2 s per epoch, the grid gives exactly 1 s.
    directory = mktempdir()
    obs_file = joinpath(directory, "drifting.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file, nav_file = nothing, interval = 1.0u"s"),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
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
end

@testset "RINEX epochs are measured within one chunk of the grid" begin
    # A clock solution every 100 ms, chunks every 4 ms: every chunk's receive time is the
    # solution's carried on, and the epoch is due on the chunk that crosses the grid point, so
    # the observables are propagated back over less than one chunk — never over the up to half
    # a second the solution itself can lie off the grid.
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = joinpath(mktempdir(), "chunk.obs"), nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    drift = 1.0e-6
    timings = []
    # The chunk before the first fix, which bounds the first fix's chunk.
    state = _rinex_receiver_state(; receive_offset = 0.5, clock_drift = drift)
    unsolved = @set state.pvt.time = nothing
    @test isnothing(GNSSReceiver.obs_epoch_timing(logger, @set unsolved.runtime = 0.0u"s"))
    for chunk = 1:1000
        runtime = 0.004 * chunk
        # A fresh solution on every 25th chunk, measured at that chunk's end, 0.5 s past the
        # integer second at the start.
        if chunk % 25 == 1
            state = _rinex_receiver_state(;
                receive_offset = 0.5 + runtime * (1 - drift),
                clock_drift = drift,
            )
        end
        state = @set state.runtime = runtime * u"s"
        timing = GNSSReceiver.obs_epoch_timing(logger, state)
        isnothing(timing) || push!(timings, (chunk, timing))
    end
    close(logger)
    # Four seconds, four grid points crossed.
    @test length(timings) == 4
    @test [timing.time_of_week for (_, timing) in timings] ≈ _TRANSMIT_TOW .+ (1:4)
    for (chunk, timing) in timings
        @test -0.004 < timing.propagation <= 0
        # The receive time carried on from the solution is the one it would have solved for.
        @test timing.receive_time ≈ _TRANSMIT_TOW + 0.07 + 0.5 + 0.004 * chunk * (1 - drift) atol =
            1e-9
    end

    # A solution that stops coming is carried on for two update intervals, not for ever.
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = joinpath(mktempdir(), "stale.obs"), nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    state = _rinex_receiver_state(; receive_offset = 0.5)
    timings = []
    for chunk = 0:500
        state = @set state.runtime = (1.0 + 0.004 * chunk) * u"s"
        timing = GNSSReceiver.obs_epoch_timing(logger, state)
        isnothing(timing) || push!(timings, timing)
    end
    close(logger)
    # The first chunk the logger ever sees has no chunk before it to bound its span, so it
    # writes the grid point before it. The next grid point is 0.43 s on, past the solution's
    # 200 ms, and so is every one after it.
    @test length(timings) == 1
    @test only(timings).time_of_week == _TRANSMIT_TOW
end

@testset "RINEX navigation file from a decoded message" begin
    directory = mktempdir()
    nav_file = joinpath(directory, "nav.nav")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; obs_file = nothing, nav_file),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
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
            (0.0Hz,),
            2.048e6Hz;
            approximate_year = 2020,
        )
        GNSSReceiver.log_rinex!(logger, state)
        close(logger)
        logger
    end

    # Named by the convention by default: the site from the marker name, the data type from
    # the constellation being tracked, the data frequency from the epoch interval.
    directory = mktempdir()
    logger = _run(RinexConfig(; directory, obs_header = (; marker_name = "TEST-1")))
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
    @test (name.system, name.file_type) == ('G', 'O')

    # A marker name that is a site identification fills every site field of the name, and
    # the collection period and country the receiver cannot know are configuration.
    logger = _run(
        RinexConfig(;
            directory = mktempdir(),
            obs_header = (; marker_name = "ROOF12DEU"),
            period = Day(1),
            interval = 2.0u"s",
        ),
    )
    @test basename(logger.obs_path) == "ROOF12DEU_R_20202642256_01D_02S_GO.rnx"
    logger = _run(
        RinexConfig(;
            directory = mktempdir(),
            obs_header = (; marker_name = "TEST-1"),
            country = "DEU",
        ),
    )
    @test basename(logger.obs_path) == "TEST00DEU_R_20202642256_00U_01S_GO.rnx"

    # A period may be a unitful time, like the epoch interval beside it.
    @test GNSSReceiver.rinex_period(1u"hr") == Second(3600)
    @test GNSSReceiver.rinex_period(Day(1)) == Day(1)
    @test isnothing(GNSSReceiver.rinex_period(nothing))
    logger = _run(RinexConfig(; directory = mktempdir(), period = 1u"hr"))
    @test basename(logger.obs_path) == "UNKN00XXX_R_20202642256_01H_01S_GO.rnx"

    # A run with observations switched off stamps no epoch, so its navigation file is named
    # after its first ephemeris: the toc, 2020-09-24 00:00, day 268.
    logger = _run(
        RinexConfig(;
            directory = mktempdir(),
            obs_file = nothing,
            obs_header = (; marker_name = "TEST-1"),
        ),
    )
    @test isnothing(logger.obs_path)
    @test basename(logger.nav_path) == "TEST00XXX_R_20202680000_00U_GN.rnx"

    # A rerun of the same recording replaces its files rather than leaving them beside it.
    directory = mktempdir()
    first_run = _run(RinexConfig(; directory, obs_header = (; marker_name = "TEST-1")))
    rerun = _run(RinexConfig(; directory, obs_header = (; marker_name = "TEST-1")))
    @test (rerun.obs_path, rerun.nav_path) == (first_run.obs_path, first_run.nav_path)
    @test length(readdir(directory)) == 2

    # A run that never got a fix has no data to name its files by, and leaves none.
    directory = mktempdir()
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; directory, obs_header = (; marker_name = "TEST-1")),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz;
        approximate_year = 2020,
    )
    close(logger)
    @test isnothing(logger.obs_path) && isnothing(logger.nav_path)
    @test isempty(readdir(directory))

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
    # Header metadata is written verbatim over the receiver's defaults, but the records the
    # receiver fills in itself, and fields the header does not have, are refused.
    config = RinexConfig(; obs_header = (; marker_name = "ROOF12DEU", observer = "Me"))
    header = GNSSReceiver.initial_obs_header(config, ['G' => ["C1C"]])
    @test (header.marker_name, header.observer) == ("ROOF12DEU", "Me")
    @test (header.receiver_type, header.marker_type) == ("GNSSReceiver.jl", "GEODETIC")
    @test_throws ArgumentError GNSSReceiver.rinex_config(
        RinexConfig(; obs_header = (; interval = 30.0)),
    )
    @test_throws ArgumentError GNSSReceiver.rinex_config(
        RinexConfig(; obs_header = (; marker = "ROOF")),
    )
    # An interval of zero would divide the epoch grid by zero inside the tracking loop.
    @test_throws ArgumentError GNSSReceiver.rinex_config(RinexConfig(; interval = 0.0u"s"))
    @test_throws ArgumentError GNSSReceiver.RinexLogger(
        RinexConfig(; interval = -1.0u"s"),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz,
    )

    # A directory that is not there is not created behind the caller's back.
    missing_directory = joinpath(mktempdir(), "not-there")
    @test_throws ArgumentError GNSSReceiver.RinexLogger(
        RinexConfig(; directory = missing_directory),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz,
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
        2.048e6Hz,
    )
    close(logger)
    @test !isdir(missing_directory)
    # Or the files go to absolute paths, which ignore it.
    obs_file = joinpath(mktempdir(), "absolute.obs")
    logger = GNSSReceiver.RinexLogger(
        RinexConfig(; directory = missing_directory, obs_file, nav_file = nothing),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz,
    )
    close(logger)
    @test isfile(obs_file)
    @test !isdir(missing_directory)
    # A navigation file that cannot be opened leaves no observation file behind: neither the
    # empty file at a fixed path nor the temporary file of one named by the convention.
    obs_file = joinpath(mktempdir(), "orphan.obs")
    @test_throws ArgumentError GNSSReceiver.RinexLogger(
        RinexConfig(; directory = missing_directory, obs_file),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz,
    )
    @test !ispath(obs_file)
    directory = mktempdir()
    @test_throws SystemError GNSSReceiver.RinexLogger(
        RinexConfig(; directory, nav_file = joinpath(missing_directory, "orphan.nav")),
        ((GPSL1CA(),),),
        (0.0Hz,),
        2.048e6Hz,
    )
    @test isempty(readdir(directory))
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
