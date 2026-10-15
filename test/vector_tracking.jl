# Note: this file is included from runtests.jl which provides all `using` statements.
#
# Vector tracking itself — the navigation engine, its filter and its satellite lifecycle —
# is TrackingLoops' `VectorPLLAndDLL` and tested there. What is tested here is the
# receiver's side of it: which estimator a receiver state is built with and how it is
# configured, and how the receiver reads the estimator's reports into its own satellite
# states, its PVT solution and its payload. The end-to-end run is in
# `ion_rtlsdr_integration.jl`.

# A stand-in for TrackingLoops' `SatelliteReport`: the receiver reads only these fields.
fake_report(;
    tracked = true,
    in_vector_loop = true,
    release_reason = TrackingLoops.VT_NOT_RELEASED,
    decoder = nothing,
) = (; tracked, in_vector_loop, release_reason, decoder)

@testset "Mixed sampling/IF units normalise to Hz for the tracking pass" begin
    # `process` builds `Tracking.BandMeasurement(m, sampling_freq, interm_freq)`. When the two
    # carry different units — a MHz sampling frequency and an Hz IF, as in a real front end —
    # Unitful's `promote` collapses both to the SI base `s^-1`, which the `Hz`-typed loop
    # states reject. `process` guards this by normalising both to `Hz`; document the trap and
    # the fix here.
    m = zeros(Complex{Int16}, 8, 1)
    raw = Tracking.BandMeasurement(m, 2.048u"MHz", 0.0u"Hz")
    @test Unitful.unit(Tracking.get_sampling_frequency(raw)) == u"s^-1"        # the trap
    fixed = Tracking.BandMeasurement(m, uconvert(u"Hz", 2.048u"MHz"), uconvert(u"Hz", 0.0u"Hz"))
    @test Unitful.unit(Tracking.get_sampling_frequency(fixed)) == u"Hz"        # the normalisation
end

@testset "Doppler estimator selection and receiver-state wiring" begin
    # The tracking-loop estimator follows from the mode alone.
    @test GNSSReceiver.doppler_estimator_for(false, (GPSL1CA(),)) isa ConventionalPLLAndDLL
    @test GNSSReceiver.doppler_estimator_for(true, (GPSL1CA(),)) isa VectorPLLAndDLL
    @test GNSSReceiver.doppler_estimator_for(GNSSReceiver.VectorTracking(), (GPSL1CA(),)) isa
          VectorPLLAndDLL

    # The vector estimator hands a fresh satellite to the scalar loop it wraps, so the
    # acquisition pull-in range is that loop's.
    @test GNSSReceiver.carrier_doppler_pull_in_range(VectorPLLAndDLL(GPSL1CA()), GPSL1CA()) ==
          GNSSReceiver.carrier_doppler_pull_in_range(
        ConventionalAssistedPLLAndDLL(),
        GPSL1CA(),
    )

    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    estimator = receiver_state.track_state.doppler_estimator
    @test estimator isa VectorPLLAndDLL
    # Nothing has run yet: no navigation cycle, and the filter is not seeded.
    @test receiver_state.navigation_cycle == 0
    @test TrackingLoops.navigation_cycle(estimator) == 0
    @test !TrackingLoops.navigation_status(estimator).running

    scalar_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 20000,
        vector_tracking = false,
    )
    @test scalar_state.track_state.doppler_estimator isa ConventionalPLLAndDLL
    @test isnothing(TrackingLoops.navigation_cycle(scalar_state.track_state.doppler_estimator))

    # A `VectorTracking` in place of `true` both enables the filter and configures it, and
    # the navigation keywords configure the estimator's cycle: the vector engine runs the
    # navigation itself, so it takes them at construction.
    config = VectorTracking(;
        h0 = 1.3e-22,
        hm2 = 2e-22,
        acceleration_noise_std = 1.0u"m/s^2",
        insufficient_meas_timeout = 25.0u"s",
    )
    configured_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 20000,
        vector_tracking = config,
        pvt_update_interval = 200u"ms",
        pvt_approximate_year = 2017,
        enable_ionospheric_correction = false,
    )
    navigation = configured_state.track_state.doppler_estimator.navigation
    @test configured_state.track_state.doppler_estimator isa VectorPLLAndDLL
    @test navigation.config === config
    @test navigation.cycle_time == 0.2u"s"
    # The scalar loop each satellite runs until the filter takes it over is the receiver's
    # scalar loop.
    @test configured_state.track_state.doppler_estimator.inner ==
          GNSSReceiver.SCALAR_DOPPLER_ESTIMATOR

    # Multi-constellation vector tracking: one estimator over both ranging signals.
    multi_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        (GPSL1CA(), GalileoE1B());
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    multi_estimator = multi_state.track_state.doppler_estimator
    @test multi_estimator isa VectorPLLAndDLL
    @test isnothing(TrackingLoops.satellite_report(multi_estimator, GPSL1CA(), 1))
    @test isnothing(TrackingLoops.satellite_report(multi_estimator, GalileoE1B(), 1))
end

@testset "Vector tracking ranges on a combined signal's pilot and decodes its data" begin
    # A `CombinedSignal` is handed to the vector estimator as `pilot => data`: the
    # satellite ranges on the pilot that drives its loops, and the engine decodes the data
    # component.
    combined = CombinedSignal(GalileoE1C(), GalileoE1B())
    # GNSSSignals' signal instances define no `==`, so compare their types.
    @test typeof(GNSSReceiver.vector_tracking_signal(combined)) ==
          Pair{typeof(GalileoE1C()),typeof(GalileoE1B())}
    @test GNSSReceiver.vector_tracking_signal(combined).first === combined.pilot
    @test GNSSReceiver.vector_tracking_signal(combined).second === combined.data
    @test GNSSReceiver.vector_tracking_signal(combined.data) === combined.data

    system = combined
    key = GNSSReceiver.signal_group_key(system)
    sampling_freq = 24.552e6Hz
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 100,
        vector_tracking = true,
    )
    estimator = receiver_state.track_state.doppler_estimator
    @test estimator isa VectorPLLAndDLL

    # Seed one satellite and step it on noise. The estimator registers the satellite on
    # its first records and reports it under the pilot, with the *data* component's
    # decoder, which the receiver takes over into a satellite state typed for that
    # decoder; a decoder of the wrong type could not be stored back.
    prn = 11
    track_state = merge_sats(
        receiver_state.track_state,
        key,
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            prn,
            0.0,
            0.0u"Hz",
            NumAnts(1),
            estimator,
        )],
    )
    state = ReceiverState(
        track_state,
        (; key => Dictionary([prn], [GNSSReceiver.ReceiverSatState(system, prn)])),
        receiver_state.acquisition_buffers,
        # Acquisition just ran, so no scan is due during these chunks.
        map(_ -> 0.0u"s", receiver_state.last_time_acquisition_ran),
        receiver_state.pvt,
        receiver_state.navigation_cycle,
        receiver_state.runtime,
        receiver_state.last_time_pvt_ran,
    )
    acq_plans = (; key => plan_acquire(GalileoE1B(), float(sampling_freq), [prn]))
    rng = Random.Xoshiro(3)
    num_samples = Int(upreferred(sampling_freq * 4u"ms"))
    for _ = 1:3
        state = GNSSReceiver.process(
            state,
            acq_plans,
            (randn(rng, ComplexF64, num_samples, 1) .* 10,),
            ((system,),),
            sampling_freq;
            acq_pfa = 1e-12,
            acquire_every = 1000u"s",
        )
    end
    report = TrackingLoops.satellite_report(estimator, GalileoE1C(), prn)
    @test !isnothing(report)
    @test report.decoder isa GNSSReceiver.decoder_state_type(system)
    # Reports are keyed by the driver: there is none under the data component.
    @test isnothing(TrackingLoops.satellite_report(estimator, GalileoE1B(), prn))
    # Still within its lock detectors' warm-up, so in lock and updated every chunk.
    @test GNSSReceiver.is_in_lock(state.receiver_sat_states[key][prn])
    @test state.receiver_sat_states[key][prn].decoder === report.decoder
end

@testset "Reading the vector estimator's satellite reports" begin
    # A scalar estimator reports `nothing` for every satellite: never a member, never
    # released.
    @test !GNSSReceiver.is_vector_loop_member(nothing)
    @test !GNSSReceiver.is_released_for_cause(nothing)

    # Membership is the latest cycle's decision, for a satellite still stepped.
    @test GNSSReceiver.is_vector_loop_member(fake_report())
    @test !GNSSReceiver.is_vector_loop_member(fake_report(; in_vector_loop = false))
    @test !GNSSReceiver.is_vector_loop_member(fake_report(; tracked = false))

    # Released for cause — ineligible or below the horizon — is forced out of lock; a
    # fallback hands the satellite back to its scalar loop and keeps it.
    released(reason) = GNSSReceiver.is_released_for_cause(
        fake_report(; in_vector_loop = false, release_reason = reason),
    )
    @test released(TrackingLoops.VT_INELIGIBLE)
    @test released(TrackingLoops.VT_BELOW_HORIZON)
    @test !released(TrackingLoops.VT_FALLBACK)
    @test !released(TrackingLoops.VT_NOT_RELEASED)

    # The vector estimator decodes every satellite it steps, so the receiver takes its
    # decoder over; a satellite it has no record of yet keeps the receiver's.
    system = GPSL1CA()
    own_decoder = GNSSDecoderState(system, 3)
    engine_decoder = GNSSDecoderState(system, 3)
    estimator = VectorPLLAndDLL(system)
    @test GNSSReceiver.updated_decoder(
        estimator,
        fake_report(; decoder = engine_decoder),
        own_decoder,
        nothing,
        system,
        3,
    ) === engine_decoder
    @test GNSSReceiver.updated_decoder(estimator, nothing, own_decoder, nothing, system, 3) ===
          own_decoder
end

@testset "Re-acquiring a satellite leaves the vector estimator's decoders alone" begin
    # Under vector tracking the receiver's decoder of a satellite is the one the estimator
    # reported, and the estimator reuses a freed slot's containers for the next satellite
    # that registers on it (`_rebind_decoder`). Satellite 5 dropped out and its slot went
    # to satellite 7, so the decoder still held for 5 shares its soft-symbol buffer with
    # 7's live one: re-acquiring 5 must not reset that decoder in place.
    system = GPSL1CA()
    key = GNSSReceiver.signal_group_key(system)
    dropped_prn, new_prn = 5, 7
    dropped_decoder = GNSSDecoderState(system, dropped_prn)
    live_decoder = decode!(
        TrackingLoops._rebind_decoder(dropped_decoder, new_prn),
        ones(100),
        100,
    )
    @test live_decoder.cache === dropped_decoder.cache
    buffered = collect(live_decoder.cache.soft_buffer)
    @test !isempty(buffered)

    receiver_sat_states = Dictionary(
        [dropped_prn],
        [GNSSReceiver.ReceiverSatState(
            dropped_prn,
            dropped_decoder,
            GNSSReceiver.CodeLockDetector(),
            GNSSReceiver.CarrierLockDetector(),
            0.0u"s",
            1.0u"s",
            0,
            false,
        )],
    )
    acq = Acquisition.AcquisitionResults(
        system,
        dropped_prn,
        5e6u"Hz",
        100.0u"Hz",
        10.0,
        nothing,
        45.0,
        1.0,
        1000.0,
        1,
        nothing,
        (-5000.0:1000.0:5000.0)u"Hz",
        1,
        5000,
        1,
    )
    @test GNSSReceiver.is_detected(acq; pfa = 1e-4)

    vector_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    _, vector_sat_states, acquired = GNSSReceiver.update_states_from_acquisition_results(
        [acq],
        1e-4,
        nothing,
        vector_state.track_state,
        receiver_sat_states,
        system,
        NumAnts(1),
    )
    @test acquired == [dropped_prn]
    @test collect(live_decoder.cache.soft_buffer) == buffered
    reacquired_decoder = vector_sat_states[dropped_prn].decoder
    @test reacquired_decoder.prn == dropped_prn
    @test reacquired_decoder.cache !== live_decoder.cache

    # A scalar receiver owns its decoder, so it restarts that one's sync in place and
    # keeps its decoded data.
    scalar_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        vector_tracking = false,
    )
    _, scalar_sat_states, _ = GNSSReceiver.update_states_from_acquisition_results(
        [acq],
        1e-4,
        nothing,
        scalar_state.track_state,
        receiver_sat_states,
        system,
        NumAnts(1),
    )
    @test scalar_sat_states[dropped_prn].decoder.cache === dropped_decoder.cache
    @test isempty(dropped_decoder.cache.soft_buffer)
end

@testset "Vector-loop members stay tracked through an outage" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    prn = 9
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    track_state = merge_sats(
        receiver_state.track_state,
        key,
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            prn,
            0.0,
            20.0u"Hz",
            NumAnts(1),
            receiver_state.track_state.doppler_estimator,
        )],
    )

    # A member out of code lock is neither reacquired nor removed.
    out_of_lock_member = GNSSReceiver.ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        out_of_lock_code_detector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        1.0u"s",
        0,
        true,
    )
    @test !GNSSReceiver.is_in_lock(out_of_lock_member)
    @test !GNSSReceiver.should_reacquire(out_of_lock_member)
    member_states = (; key => Dictionary([prn], [out_of_lock_member]))
    @test length(get_sat_states(
        GNSSReceiver.remove_lost_satellites(member_states, track_state),
    )) == 1
    # The same satellite out of the loop is removed.
    non_member_states = (; key => Dictionary([prn], [@set out_of_lock_member.in_vt_loop = false]))
    @test isempty(get_sat_states(
        GNSSReceiver.remove_lost_satellites(non_member_states, track_state),
    ))

    # While in the vector loop, lock follows the code detector alone.
    code_locked_member = GNSSReceiver.ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.set_out_of_lock(GNSSReceiver.CarrierLockDetector()),
        0.0u"s",
        0.0u"s",
        0,
        true,
    )
    @test GNSSReceiver.is_in_lock(code_locked_member)
    @test !GNSSReceiver.is_in_lock(@set code_locked_member.in_vt_loop = false)

    # Forcing out of lock trips both detectors so the satellite is removed and
    # reacquired through the normal path.
    forced = GNSSReceiver.force_out_of_lock(code_locked_member)
    @test !GNSSReceiver.is_in_lock(forced.code_lock_detector)
    @test !GNSSReceiver.is_in_lock(forced.carrier_lock_detector)
end

@testset "The receiver copies the vector estimator's solution once per cycle" begin
    system = GPSL1CA()
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    estimator = receiver_state.track_state.doppler_estimator
    previous = PVTSolution()
    navigate(new_cycle) = GNSSReceiver.update_navigation(
        estimator,
        new_cycle,
        (system,),
        receiver_state.track_state,
        receiver_state.receiver_sat_states,
        previous,
        3.0u"s",
        2.9u"s",
    )

    # No cycle ran in the chunk: the previous solution and its time carry forward.
    pvt, last_time_pvt_ran = navigate(false)
    @test pvt === previous
    @test last_time_pvt_ran == 2.9u"s"

    # A cycle ran: the estimator's solution, as an independent copy — the estimator
    # reuses its containers in the next cycle, while the receiver emits this one.
    pvt, last_time_pvt_ran = navigate(true)
    solution = TrackingLoops.navigation_solution(estimator)
    @test last_time_pvt_ran == 3.0u"s"
    @test pvt !== solution
    @test pvt.position == solution.position
    @test pvt.sats == solution.sats
    @test pvt.sats !== solution.sats
    @test pvt.inter_system_biases !== solution.inter_system_biases
    @test pvt.inter_frequency_biases !== solution.inter_frequency_biases
end

@testset "Vector-tracking status in the payload" begin
    scalar_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 20000,
    )
    @test isnothing(GNSSReceiver.vt_status_of_interest(scalar_state))

    vt_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    status = GNSSReceiver.vt_status_of_interest(vt_state)
    @test status isa GNSSReceiver.VTStatus
    @test !status.running
    @test isempty(status.member_sats)
    # Not running: the filter has no uncertainty to report.
    @test isnan(status.position_std)
    @test isnan(status.clock_std)
    # The per-member report is a copy, not the estimator's own container.
    @test status.member_sats !== TrackingLoops.member_sats(vt_state.track_state.doppler_estimator)
end

@testset "set_out_of_lock lock detectors" begin
    code_detector = GNSSReceiver.CodeLockDetector()
    @test GNSSReceiver.is_in_lock(code_detector)
    @test !GNSSReceiver.is_in_lock(GNSSReceiver.set_out_of_lock(code_detector))
    carrier_detector = GNSSReceiver.CarrierLockDetector()
    @test GNSSReceiver.is_in_lock(carrier_detector)
    @test !GNSSReceiver.is_in_lock(GNSSReceiver.set_out_of_lock(carrier_detector))
end
