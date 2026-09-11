# The differential payload group delay the receiver hands `Tracking` so its
# multi-signal discriminator combining may take the data component's *code*
# measurement too. See the "Differential group delay" section in
# `src/GNSSReceiver.jl` for the sign convention and its derivation.

@testset "differential group delay from the decoded message" begin
    # Both components of a Galileo pair leave the satellite in one payload chain,
    # so the differential is a hard zero — no decoded message needed.
    @test GNSSReceiver.differential_group_delay(
        GNSSReceiver.CombinedSignal(GalileoE1C(), GalileoE1B()),
        GNSSDecoderState(GalileoE1B(), 1),
    ) === 0.0u"s"
    @test GNSSReceiver.differential_group_delay(
        GNSSReceiver.CombinedSignal(GalileoE5aQ(), GalileoE5aI()),
        GNSSDecoderState(GalileoE5aI(), 1),
    ) === 0.0u"s"

    # L2 CM and CL share the single broadcast `ISC_L2C`, so likewise zero.
    @test GNSSReceiver.differential_group_delay(
        GNSSReceiver.CombinedSignal(GPSL2CL(), GPSL2CM()),
        GNSSDecoderState(GPSL2CM(), 1),
    ) === 0.0u"s"

    # A data-only system tracks one signal, which *is* the estimator driver:
    # there is nothing to state a differential against.
    @test GNSSReceiver.differential_group_delay(GPSL1CA(), GNSSDecoderState(GPSL1CA(), 1)) ===
          nothing

    # GPS L5: `ISC_data − ISC_pilot`, and a time rather than a bare number —
    # `set_differential_group_delay!` refuses the latter.
    l5 = GNSSReceiver.CombinedSignal(GPSL5Q(), GPSL5I())
    fresh = GNSSDecoderState(GPSL5I(), 1)
    # Neither ISC decoded yet: unknown, NOT zero.
    @test GNSSReceiver.differential_group_delay(l5, fresh) === nothing
    # One of the two decoded is still unknown.
    @test GNSSReceiver.differential_group_delay(l5, (@set fresh.data.ISC_L5I5 = 2.0e-9)) ===
          nothing
    decoded = @set fresh.data.ISC_L5I5 = 2.0e-9
    decoded = @set decoded.data.ISC_L5Q5 = 0.5e-9
    @test GNSSReceiver.differential_group_delay(l5, decoded) ≈ 1.5e-9u"s"

    # GPS L1C, from CNAV-2's own ISC pair.
    l1c = GNSSReceiver.CombinedSignal(GPSL1C_P(), GPSL1C_D())
    l1c_decoder = GNSSDecoderState(GPSL1C_D(), 1)
    @test GNSSReceiver.differential_group_delay(l1c, l1c_decoder) === nothing
    l1c_decoder = @set l1c_decoder.data.ISC_L1CD = 1.0e-9
    l1c_decoder = @set l1c_decoder.data.ISC_L1CP = -1.0e-9
    @test GNSSReceiver.differential_group_delay(l1c, l1c_decoder) ≈ 2.0e-9u"s"
end

@testset "update_differential_group_delays! writes only what changed" begin
    system = GNSSReceiver.CombinedSignal(GPSL5Q(), GPSL5I())
    key = GNSSReceiver.signal_group_key(system)
    data_idx = GNSSReceiver.data_signal_index(system)

    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
    )
    track_state = merge_sats(
        receiver_state.track_state,
        key,
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            1,
            0.0,
            20.0u"Hz",
            NumAnts(1),
            receiver_state.track_state.doppler_estimator,
        )],
    )

    sat_state(prn, decoder) = GNSSReceiver.ReceiverSatState(
        prn,
        decoder,
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        0.0u"s",
        0,
        false,
    )

    # Nothing decoded yet: the data component stays at `nothing`, so the code
    # loop keeps it out while its carrier contribution runs from the first
    # integration.
    fresh = GNSSDecoderState(GPSL5I(), 1)
    states = (; key => Dictionary([1], [sat_state(1, fresh)]))
    GNSSReceiver.update_differential_group_delays!(track_state, states, (system,))
    @test get_differential_group_delay(track_state, key, 1, data_idx) === nothing

    # The ISCs arrive: the differential is written through to the tracked signal.
    decoded = @set fresh.data.ISC_L5I5 = 2.0e-9
    decoded = @set decoded.data.ISC_L5Q5 = 0.5e-9
    states = (; key => Dictionary([1], [sat_state(1, decoded)]))
    GNSSReceiver.update_differential_group_delays!(track_state, states, (system,))
    @test get_differential_group_delay(track_state, key, 1, data_idx) ≈ 1.5e-9u"s"

    # Unchanged on the next chunk: the satellite is not rebuilt.
    before = get_sat_state(track_state, key, 1)
    GNSSReceiver.update_differential_group_delays!(track_state, states, (system,))
    @test get_sat_state(track_state, key, 1) === before

    # A `ReceiverSatState` whose satellite has already been dropped from the
    # tracking state (the reacquisition book-keeping outlives the tracked sat)
    # is skipped rather than throwing.
    states = (; key => Dictionary([1, 7], [sat_state(1, decoded), sat_state(7, decoded)]))
    @test GNSSReceiver.update_differential_group_delays!(track_state, states, (system,)) ===
          track_state
end

@testset "single-signal groups are never given a differential group delay" begin
    # The only signal of a data-only group is the estimator driver, whose
    # differential is 0 by definition — `set_differential_group_delay!` throws on
    # a non-zero value there, so the update must skip such a group outright.
    system = GPSL1CA()
    key = get_signal_id(system)
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
    )
    track_state = merge_sats(
        receiver_state.track_state,
        key,
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            1,
            0.0,
            20.0u"Hz",
            NumAnts(1),
            receiver_state.track_state.doppler_estimator,
        )],
    )
    states = (; key => Dictionary([1], [GNSSReceiver.ReceiverSatState(system, 1)]))
    before = get_sat_state(track_state, key, 1)
    GNSSReceiver.update_differential_group_delays!(track_state, states, (system,))
    @test get_sat_state(track_state, key, 1) === before
    @test get_differential_group_delay(track_state, key, 1) === nothing
end

@testset "scalar tracking combines a group's signals" begin
    # The precondition `Tracking` cannot check for itself holds by construction
    # here (pilot first, one shared primary code period, no raised integration
    # length), so the receiver switches combining on.
    @test GNSSReceiver.doppler_estimator_for(false).signal_combining
    # Vector tracking has no equivalent: the navigation filter reads the driver's
    # discriminators only.
    @test GNSSReceiver.doppler_estimator_for(true) isa VectorPLLAndDLL

    system = GNSSReceiver.CombinedSignal(GalileoE1C(), GalileoE1B())
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
    )
    @test receiver_state.track_state.doppler_estimator.signal_combining
    # And it reaches the per-satellite estimator state, which is what the fold
    # actually reads.
    track_state = merge_sats(
        receiver_state.track_state,
        GNSSReceiver.signal_group_key(system),
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            1,
            0.0,
            20.0u"Hz",
            NumAnts(1),
            receiver_state.track_state.doppler_estimator,
        )],
    )
    @test Tracking.get_doppler_estimator_state(
        get_sat_state(track_state, GNSSReceiver.signal_group_key(system), 1),
    ).signal_combining
end

@testset "a combined group tracks through the combining fold" begin
    # End-to-end: `process` over a `CombinedSignal` group whose data component
    # already carries a differential group delay, so `track!` runs `Tracking`'s
    # combining fold with the code contribution enabled rather than only the
    # carrier one.
    system = GNSSReceiver.CombinedSignal(GPSL5Q(), GPSL5I())
    systems = (system,)
    key = GNSSReceiver.signal_group_key(system)
    bk = get_band_id(GNSSReceiver.system_band(system))
    # L5's 10.23 MHz chip rate needs a sampling frequency above it.
    sampling_freq = 11e6Hz
    measurement = randn(ComplexF64, 30000)

    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 30000,
    )
    track_state = merge_sats(
        receiver_state.track_state,
        key,
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            1,
            0.0,
            20.0u"Hz",
            NumAnts(1),
            receiver_state.track_state.doppler_estimator,
        )],
    )

    decoder = GNSSDecoderState(GPSL5I(), 1)
    decoder = @set decoder.data.ISC_L5I5 = 2.0e-9
    decoder = @set decoder.data.ISC_L5Q5 = 0.5e-9
    receiver_sat_states = (;
        key => Dictionary(
            [1],
            [GNSSReceiver.ReceiverSatState(
                1,
                decoder,
                GNSSReceiver.CodeLockDetector(),
                GNSSReceiver.CarrierLockDetector(),
                0.0u"s",
                0.0u"s",
                0,
                false,
            )],
        )
    )
    receiver_state = ReceiverState(
        track_state,
        receiver_sat_states,
        NamedTuple{(bk,)}((GNSSReceiver.SampleBuffer(ComplexF64, 30000),)),
        NamedTuple{(bk,)}((-Inf * 1.0u"s",)),
        PVTSolution(),
        SatelliteState[],
        nothing,
        0.0u"s",
        -Inf * 1.0u"s",
    )

    # Two chunks: the first supplies the delay, the second tracks with it in
    # place (the fold also carries passenger discriminators across the boundary).
    for _ in 1:2
        receiver_state = GNSSReceiver.process(
            receiver_state,
            (; key => plan_acquire(GPSL5Q(), float(sampling_freq), collect(1:32))),
            (measurement,),
            (systems,),
            sampling_freq,
            (0.0u"Hz",);
            acq_pfa = 1e-12,
        )
    end

    @test length(get_sat_states(receiver_state.track_state, key)) == 1
    @test get_differential_group_delay(
        receiver_state.track_state,
        key,
        1,
        GNSSReceiver.data_signal_index(system),
    ) ≈ 1.5e-9u"s"
end

@testset "combining is cleared for a group whose driver is not the longest" begin
    # `Tracking` deliberately does not check combining's one precondition — the
    # estimator-driver signal must be the group's longest-integrating one — because the
    # ordering is a property of the tuple its caller assembled. This receiver is that
    # caller, so it checks.
    @test GNSSReceiver.combines_signals(GPSL1CA())                                 # single signal
    @test GNSSReceiver.combines_signals(GNSSReceiver.CombinedSignal(GPSL5Q(), GPSL5I()))
    @test GNSSReceiver.combines_signals(GNSSReceiver.CombinedSignal(GalileoE1C(), GalileoE1B()))
    @test GNSSReceiver.combines_signals(GNSSReceiver.CombinedSignal(GPSL1C_P(), GPSL1C_D()))
    # L2 CL's 1.5 s primary code against CM's 20 ms — the widest margin of any pair, and
    # the right way round.
    @test GNSSReceiver.combines_signals(GNSSReceiver.CombinedSignal(GPSL2CL(), GPSL2CM()))
    # A deliberately mis-ordered pair: a 4 ms pilot driving a 10 ms data component, which
    # would reach the loop in one update out of two or three.
    mismatched = GNSSReceiver.CombinedSignal(GalileoE1C(), GPSL1C_D())
    @test !GNSSReceiver.combines_signals(mismatched)

    # Such a group's satellites have the flag cleared at handoff, while a well-ordered
    # one keeps what the estimator seeded.
    function combining_after_handoff(system)
        key = GNSSReceiver.signal_group_key(system)
        # A mis-ordered pair warns once here, since the loss it causes is otherwise
        # invisible: nothing errors and no measurement is wrong.
        receiver_state = GNSSReceiver.ReceiverState(
            ComplexF64,
            system;
            num_samples_for_acquisition = 20000,
        )
        track_state = merge_sats(
            receiver_state.track_state,
            key,
            [GNSSReceiver.create_tracked_sat(
                GNSSReceiver.tracking_signals(system),
                1,
                0.0,
                20.0u"Hz",
                NumAnts(1),
                receiver_state.track_state.doppler_estimator,
            )],
        )
        GNSSReceiver.apply_signal_combining!(track_state, key, system, [1])
        Tracking.get_doppler_estimator_state(
            get_sat_state(track_state, key, 1),
        ).signal_combining
    end

    @test combining_after_handoff(GNSSReceiver.CombinedSignal(GPSL5Q(), GPSL5I()))
    @test (@test_logs (:warn,) match_mode = :any !combining_after_handoff(mismatched))
    # A single-signal satellite keeps the flag — `Tracking` is bit-identical either way,
    # so there is nothing to clear.
    @test combining_after_handoff(GPSL1CA())
end
