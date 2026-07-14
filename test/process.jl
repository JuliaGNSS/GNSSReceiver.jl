@testset "Process measurement with number of antennas $i" for i in [1, 4]
    measurement = randn(ComplexF64, 20000, i)
    system = GPSL1CA()
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(i),
    )
    sampling_freq = 5e6Hz

    acq_plan = plan_acquire(system, float(sampling_freq), collect(1:32))

    next_receiver_state = GNSSReceiver.process(
        receiver_state,
        acq_plan,
        measurement,
        system,
        sampling_freq;
        num_ants = NumAnts(i),
    )

    @test length(get_sat_states(next_receiver_state.track_state)) == 0

    receiver_sat_states = (Dictionary([1], [GNSSReceiver.ReceiverSatState(system, 1)]),)

    track_state =
        TrackState(system, [TrackedSat(system, 1, 0.0, 20u"Hz"; num_ants = NumAnts(i))])

    acquisition_buffer = GNSSReceiver.SampleBuffer(ComplexF64, 20000)

    pvt = PositionVelocityTime.PVTSolution()

    decoder = GNSSDecoderState(system, 1)
    pvt_sat_state_buffer = SatelliteState{Float64,typeof(decoder),typeof(system)}[]

    receiver_state = ReceiverState(
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        pvt,
        pvt_sat_state_buffer,
        0.0u"s",
        -Inf * 1.0u"s",
        -Inf * 1.0u"s",
        0,
    )

    next_receiver_state = GNSSReceiver.process(
        receiver_state,
        acq_plan,
        measurement,
        system,
        sampling_freq;
        num_ants = NumAnts(i),
    )

    @test length(get_sat_states(next_receiver_state.track_state)) == 1
end

# Helpers for the reacquisition-path tests below. A satellite is "out of lock" once
# its code lock detector has accumulated out-of-lock time past its threshold; we build
# that state directly instead of driving many `update` calls through `process`.
out_of_lock_code_detector() =
    GNSSReceiver.CodeLockDetector(30.0u"dBHz", 250u"ms", 200u"ms", 80u"ms", 80u"ms")

function out_of_lock_sat_state(system, prn)
    GNSSReceiver.ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        out_of_lock_code_detector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        0.0u"s",
        0,
        100.0u"Hz",
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
        0.0u"Hz",
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

@testset "filter_in_lock_sats drops out-of-lock tracked satellites" begin
    system = GPSL1CA()
    track_state =
        TrackState(system, [TrackedSat(system, 5, 0.0, 20u"Hz"; num_ants = NumAnts(1))])
    receiver_sat_states = Dictionary([5], [out_of_lock_sat_state(system, 5)])

    filtered = GNSSReceiver.filter_in_lock_sats(receiver_sat_states, track_state)
    @test length(get_sat_states(filtered)) == 0
end

@testset "update_receiver_sat_states advances out-of-lock timer" begin
    system = GPSL1CA()
    track_state =
        TrackState(system, [TrackedSat(system, 5, 0.0, 20u"Hz"; num_ants = NumAnts(1))])
    receiver_sat_states = Dictionary([5], [out_of_lock_sat_state(system, 5)])

    updated =
        GNSSReceiver.update_receiver_sat_states(receiver_sat_states, track_state, 4u"ms")
    @test updated[5].time_out_of_lock == 4u"ms"
    @test !GNSSReceiver.is_in_lock(updated[5])
end

@testset "create_sat_state_from_acq builds a matching tracked-sat slot" begin
    system = GPSL1CA()
    num_ants = NumAnts(1)
    empty_track_state =
        ReceiverState(
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
    tracked_sat = GNSSReceiver.create_sat_state_from_acq(acq, empty_track_state, num_ants)
    # Built with the same slot type the track state uses, so `merge_sats` accepts it.
    @test tracked_sat isa eltype(get_sat_states(empty_track_state))
end

@testset "update_states_from_acquisition_results is a no-op without detections" begin
    system = GPSL1CA()
    num_ants = NumAnts(1)
    base = ReceiverState(ComplexF64, system; num_samples_for_acquisition = 20000, num_ants)
    empty_track_state = base.track_state
    empty_receiver_sat_states = base.receiver_sat_states[1]

    # No acquisition results leaves both the track state and the receiver-sat-state
    # dictionary untouched. (The detection-handover path is covered end-to-end by the
    # reacquisition integration test, which feeds it real acquisition results.)
    @test GNSSReceiver.update_states_from_acquisition_results(
        Acquisition.AcquisitionResults[],
        1e-4,
        30.0u"dBHz",
        empty_track_state,
        empty_receiver_sat_states,
        num_ants,
    ) === (empty_track_state, empty_receiver_sat_states)
end

@testset "try_to_reacquire_lost_satellites counts failed reacquisitions" begin
    system = GPSL1CA()
    num_ants = NumAnts(1)
    sampling_freq = 5e6Hz
    acq_plan = plan_acquire(system, float(sampling_freq), collect(1:32))

    # Fill the acquisition buffer with noise so `should_reacquire` fires but the
    # (deterministic) acquisition finds nothing, driving the failed-reacquisition
    # counter path.
    rng = Random.Xoshiro(1)
    noise = randn(rng, ComplexF64, 20000) * 512
    acquisition_buffer = GNSSReceiver.SampleBuffers.buffer(
        GNSSReceiver.SampleBuffer(ComplexF64, 20000),
        noise,
    )
    @test GNSSReceiver.SampleBuffers.isfull(acquisition_buffer)

    track_state =
        TrackState(system, [TrackedSat(system, 5, 0.0, 20u"Hz"; num_ants = NumAnts(1))])
    receiver_sat_states = Dictionary([5], [out_of_lock_sat_state(system, 5)])
    @test GNSSReceiver.should_reacquire(receiver_sat_states[5])

    _, updated_receiver_sat_states = GNSSReceiver.try_to_reacquire_lost_satellites(
        acq_plan,
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        0.0u"Hz",
        1e-4,
        30.0u"dBHz",
        num_ants,
        20000,
    )
    @test updated_receiver_sat_states[5].num_unsuccessful_reacquisition == 1
end
