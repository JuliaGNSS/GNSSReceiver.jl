@testset "process with number of antennas $i" for i in [1, 4]
    measurement = randn(ComplexF64, 20000, i)
    system = GPSL1CA()
    systems = (system,)
    key = get_signal_id(system)
    bk = get_band_id(GNSSReceiver.system_band(system))
    sampling_freq = 5e6Hz

    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(i),
    )

    acq_plans = (; key => plan_acquire(system, float(sampling_freq), collect(1:32)))

    # A tiny false-alarm probability so pure noise never acquires.
    next_receiver_state = GNSSReceiver.process(
        receiver_state,
        acq_plans,
        (measurement,),
        (systems,),
        sampling_freq,
        (0.0u"Hz",);
        num_ants = NumAnts(i),
        acq_pfa = 1e-12,
    )

    @test length(get_sat_states(next_receiver_state.track_state)) == 0

    # Now seed one tracked satellite and confirm it survives a processing step.
    # The sat must be built by `create_tracked_sat` — the receiver's canonical
    # constructor that pinned the track state's satellite-slot type (for i > 1
    # it injects the `EigenBeamformer` post-corr filter); a default `TrackedSat`
    # would be rejected by `merge_sats` for its slot type.
    track_state = merge_sats(
        receiver_state.track_state,
        key,
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            1,
            0.0,
            20.0u"Hz",
            NumAnts(i),
            receiver_state.track_state.doppler_estimator,
        )],
    )
    receiver_sat_states =
        (; key => Dictionary([1], [GNSSReceiver.ReceiverSatState(system, 1)]))
    acquisition_buffers = NamedTuple{(bk,)}((GNSSReceiver.SampleBuffer(ComplexF64, 20000),))
    last_time_acquisition_ran = NamedTuple{(bk,)}((-Inf * 1.0u"s",))
    pvt = PositionVelocityTime.PVTSolution()

    receiver_state = ReceiverState(
        track_state,
        receiver_sat_states,
        acquisition_buffers,
        last_time_acquisition_ran,
        pvt,
        PositionVelocityTime.SatelliteState[],
        0.0u"s",
        -Inf * 1.0u"s",
    )

    next_receiver_state = GNSSReceiver.process(
        receiver_state,
        acq_plans,
        (measurement,),
        (systems,),
        sampling_freq,
        (0.0u"Hz",);
        num_ants = NumAnts(i),
        acq_pfa = 1e-12,
    )

    @test length(get_sat_states(next_receiver_state.track_state)) == 1
end

# Exercise `update_pvt`'s timing gate with no ready satellites, so a broken
# `pvt_update_interval` / `time_in_lock` gate can't ship untested.
@testset "update_pvt timing gate" begin
    system = GPSL1CA()
    all_systems = (system,)
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(1),
    )
    runtime = 5.0u"s"
    pvt = PositionVelocityTime.PVTSolution()
    pvt_update_interval = 100u"ms"
    time_in_lock_before_pvt = 2u"s"

    # Gate closed: too soon since the last solve → pvt and timestamp unchanged.
    pvt_out, last_time = GNSSReceiver.update_pvt(
        all_systems,
        receiver_state.receiver_sat_states,
        receiver_state.track_state,
        pvt,
        receiver_state.pvt_sat_state_buffer,
        runtime,
        time_in_lock_before_pvt,
        runtime,               # last_time_pvt_ran == runtime ⇒ 0 elapsed
        pvt_update_interval,
    )
    @test pvt_out === pvt
    @test last_time == runtime

    # Gate open (interval elapsed) but no in-lock satellites ⇒ no fix, yet the
    # solve timestamp still advances to the current runtime.
    pvt_out, last_time = GNSSReceiver.update_pvt(
        all_systems,
        receiver_state.receiver_sat_states,
        receiver_state.track_state,
        pvt,
        receiver_state.pvt_sat_state_buffer,
        runtime,
        time_in_lock_before_pvt,
        -Inf * 1.0u"s",        # last_time_pvt_ran ⇒ gate open
        pvt_update_interval,
    )
    @test isnothing(pvt_out.time)
    @test last_time == runtime
end

# Helpers for the reacquisition-path tests below. A satellite is "out of lock" once
# its code lock detector has accumulated out-of-lock time past its threshold; we build
# that state directly instead of driving many `update` calls through `process`.
# `time_out_of_lock` is the `ReceiverSatState`'s own out-of-lock timer, which gates
# the reacquisition back-off (`should_reacquire`) — pass a value past the first
# back-off step (200 ms) to make the sat eligible for reacquisition.
out_of_lock_code_detector() =
    GNSSReceiver.CodeLockDetector(30.0u"dBHz", 250u"ms", 200u"ms", 80u"ms", 80u"ms")

function out_of_lock_sat_state(system, prn; time_out_of_lock = 0.0u"s")
    GNSSReceiver.ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        out_of_lock_code_detector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        uconvert(u"s", float(time_out_of_lock)),
        0,
    )
end

# A single-group track state with one satellite seeded through the receiver's
# canonical constructor, so its slot type matches what `ReceiverState` pinned.
function single_sat_track_state(system, prn; num_ants = NumAnts(1))
    base = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants,
    )
    merge_sats(
        base.track_state,
        get_signal_id(system),
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            prn,
            0.0,
            20.0u"Hz",
            num_ants,
            base.track_state.doppler_estimator,
        )],
    )
end

@testset "ReceiverSatState lock-timer transitions" begin
    system = GPSL1CA()
    # A satellite that has been in lock for a while (non-zero time_in_lock).
    state = GNSSReceiver.ReceiverSatState(
        1,
        GNSSDecoderState(system, 1),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        5.0u"s",
        0.0u"s",
        0,
    )

    lost = GNSSReceiver.increase_time_out_of_lock(state, 4u"ms")
    @test lost.time_in_lock == 0.0u"s"
    @test lost.time_out_of_lock == 4u"ms"
    @test lost.num_unsuccessful_reacquisition == 0

    lost_again = GNSSReceiver.increase_time_out_of_lock(lost, 4u"ms")
    @test lost_again.time_out_of_lock == 8u"ms"

    reattempted = GNSSReceiver.increment_num_unsuccessful_reacquisition(lost)
    @test reattempted.num_unsuccessful_reacquisition == 1
    @test GNSSReceiver.increment_num_unsuccessful_reacquisition(
        reattempted,
    ).num_unsuccessful_reacquisition == 2
end

@testset "remove_lost_satellites drops out-of-lock tracked satellites" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    track_state = single_sat_track_state(system, 5)
    receiver_sat_states = (; key => Dictionary([5], [out_of_lock_sat_state(system, 5)]))

    pruned = GNSSReceiver.remove_lost_satellites(receiver_sat_states, track_state)
    @test length(get_sat_states(pruned)) == 0
end

@testset "update_all_receiver_sat_states advances out-of-lock timer" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    track_state = single_sat_track_state(system, 5)
    receiver_sat_states = (; key => Dictionary([5], [out_of_lock_sat_state(system, 5)]))

    updated = GNSSReceiver.update_all_receiver_sat_states(
        receiver_sat_states,
        track_state,
        (system,),
        4u"ms",
    )
    @test updated[key][5].time_out_of_lock == 4u"ms"
    @test !GNSSReceiver.is_in_lock(updated[key][5])
end

@testset "tracked_sat_from_acq builds a matching tracked-sat slot ($i antennas)" for i in [1, 4]
    system = GPSL1CA()
    num_ants = NumAnts(i)
    empty_track_state =
        GNSSReceiver.ReceiverState(
            ComplexF64,
            system;
            num_samples_for_acquisition = 20000,
            num_ants,
        ).track_state
    acq = Acquisition.AcquisitionResults(
        system,
        5,
        5e6u"Hz",
        100.0u"Hz",
        10.0,
        nothing,
        45.0,
        1.0,
        20.0,
        1,
        nothing,
        (-5000.0:1000.0:5000.0)u"Hz",
        1,
        5000,
        1,
    )
    tracked_sat = GNSSReceiver.tracked_sat_from_acq(
        acq,
        GNSSReceiver.tracking_signals(system),
        num_ants,
        empty_track_state.doppler_estimator,
    )
    # Built with the same slot type the track state pins (for multiple antennas that
    # includes the `EigenBeamformer` post-corr filter), so `merge_sats` accepts it.
    @test tracked_sat isa eltype(get_sat_states(empty_track_state))
    merged = merge_sats(empty_track_state, get_signal_id(system), [tracked_sat])
    @test length(get_sat_states(merged)) == 1
end

@testset "update_states_from_acquisition_results is a no-op without detections" begin
    system = GPSL1CA()
    num_ants = NumAnts(1)
    base = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants,
    )
    empty_track_state = base.track_state
    empty_receiver_sat_states = base.receiver_sat_states[get_signal_id(system)]

    # No acquisition results leaves both the track state and the receiver-sat-state
    # dictionary untouched. (The detection-handover path is covered end-to-end by the
    # reacquisition integration test, which feeds it real acquisition results.)
    ts, rss, acquired = GNSSReceiver.update_states_from_acquisition_results(
        Acquisition.AcquisitionResults[],
        1e-4,
        nothing,
        empty_track_state,
        empty_receiver_sat_states,
        system,
        num_ants,
    )
    @test ts === empty_track_state
    @test rss === empty_receiver_sat_states
    @test isempty(acquired)
end

@testset "try_to_reacquire_lost_satellites counts failed reacquisitions" begin
    system = GPSL1CA()
    num_ants = NumAnts(1)
    sampling_freq = 5e6Hz
    acq_plan = plan_acquire(system, float(sampling_freq), collect(1:32))

    # Fill the acquisition buffer with noise so the reacquisition attempt runs but the
    # (deterministic) acquisition finds nothing, driving the failed-reacquisition
    # counter path.
    rng = Random.Xoshiro(1)
    noise = randn(rng, ComplexF64, 20000) * 512
    acquisition_buffer = GNSSReceiver.SampleBuffers.buffer(
        GNSSReceiver.SampleBuffer(ComplexF64, 20000),
        noise,
    )
    @test GNSSReceiver.SampleBuffers.isfull(acquisition_buffer)

    track_state = single_sat_track_state(system, 5)
    receiver_sat_states = Dictionary(
        [5],
        [out_of_lock_sat_state(system, 5; time_out_of_lock = 0.25u"s")],
    )
    @test GNSSReceiver.should_reacquire(receiver_sat_states[5])

    _, updated_receiver_sat_states = GNSSReceiver.try_to_reacquire_lost_satellites(
        track_state,
        receiver_sat_states,
        system,
        acq_plan,
        acquisition_buffer,
        0.0u"Hz",
        1e-4,
        nothing,
        num_ants,
        20000,
        true,
    )
    @test updated_receiver_sat_states[5].num_unsuccessful_reacquisition == 1
end
