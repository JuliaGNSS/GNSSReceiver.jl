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
        nothing,
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

# `update_pvt` is now unconditional (the cadence gate lives in `process`): with
# fewer than four PVT-ready satellites it returns the previous solution unchanged.
@testset "update_pvt returns the previous solution without enough satellites" begin
    system = GPSL1CA()
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(1),
    )
    pvt = PositionVelocityTime.PVTSolution()
    # No satellites are in lock, so nothing is PVT-ready and the previous solution
    # is returned as-is (same object).
    pvt_out = GNSSReceiver.update_pvt(
        (system,),
        receiver_state.track_state,
        receiver_state.receiver_sat_states,
        pvt,
        receiver_state.pvt_sat_state_buffer,
    )
    @test pvt_out === pvt
end

# The PVT cadence gate lives in `process`: a navigation cycle runs (advancing
# `last_time_pvt_ran` to the current runtime) only once `pvt_update_interval` of
# signal time has elapsed since the last one.
@testset "process PVT cadence gate" begin
    system = GPSL1CA()
    systems = (system,)
    key = get_signal_id(system)
    sampling_freq = 5e6Hz
    measurement = randn(ComplexF64, 20000, 1)
    base = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(1),
    )
    acq_plans = (; key => plan_acquire(system, float(sampling_freq), collect(1:32)))

    with_pvt_timing(runtime, last_time_pvt_ran) = ReceiverState(
        base.track_state,
        base.receiver_sat_states,
        base.acquisition_buffers,
        base.last_time_acquisition_ran,
        base.pvt,
        base.pvt_sat_state_buffer,
        nothing,
        runtime,
        last_time_pvt_ran,
    )

    # Gate closed: <100 ms since the last solve ⇒ `last_time_pvt_ran` unchanged.
    closed = GNSSReceiver.process(
        with_pvt_timing(5.0u"s", 4.95u"s"),
        acq_plans,
        (measurement,),
        (systems,),
        sampling_freq,
        (0.0u"Hz",);
        acq_pfa = 1e-12,
    )
    @test closed.last_time_pvt_ran == 4.95u"s"

    # Gate open: ≥100 ms elapsed ⇒ a cycle runs and `last_time_pvt_ran` advances to runtime.
    opened = GNSSReceiver.process(
        with_pvt_timing(5.0u"s", 4.8u"s"),
        acq_plans,
        (measurement,),
        (systems,),
        sampling_freq,
        (0.0u"Hz",);
        acq_pfa = 1e-12,
    )
    @test opened.last_time_pvt_ran == 5.0u"s"
end

# Helpers for the reacquisition-path tests below. A satellite is "out of lock" once
# its code lock detector has accumulated out-of-lock time past its threshold; we build
# that state directly instead of driving many `update` calls through `process`.
# `time_out_of_lock` is the `ReceiverSatState`'s own out-of-lock timer, which gates
# the reacquisition back-off (`should_reacquire`) — pass a value past the first
# back-off step (200 ms) to make the sat eligible for reacquisition.
# Driven through `update` rather than written field by field, so it stays correct across a
# retuning of the stage timings (and across the struct layout, which now nests a `LockDwell`).
function out_of_lock_code_detector()
    detector = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30.0u"dBHz",
        # GPS L1 C/A's code period, which every timing is a multiple of.
        reference_integration_time = 1u"ms",
    )
    # Enough clean evidence to latch out of the pull-in stage, so the loop below only has to
    # spend the (tighter) steady-state dwell.
    for _ = 1:100
        detector = GNSSReceiver.update(detector, 45.0u"dBHz", 1u"ms", 4u"ms")
    end
    @assert GNSSReceiver.has_pulled_in(detector)
    while GNSSReceiver.is_in_lock(detector)
        detector = GNSSReceiver.update(detector, 0.0u"dBHz", 1u"ms", 4u"ms")
    end
    detector
end

function out_of_lock_sat_state(system, prn; time_out_of_lock = 0.0u"s")
    GNSSReceiver.ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        out_of_lock_code_detector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        uconvert(u"s", float(time_out_of_lock)),
        0,
        false,
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


# ---------------------------------------------------------------------------------------
# The ranging-readiness gate on the PVT solve. `is_in_lock` is deliberately generous
# through the acquisition → tracking handover so a converging satellite is kept rather than
# dropped and reacquired; `is_ranging_ready` is what stops that tolerance from leaking a
# code phase that is still walking the coarse acquisition estimate in — a pseudorange wrong
# by metres to tens of metres — into the solve. The gate lives in `collect_pvt_sat_states!`,
# so it is asserted there and not only on the detectors.
# ---------------------------------------------------------------------------------------

# GPS L1 C/A's code period, the reference integration time its detectors are configured in.
const PVT_GATE_REFERENCE_T = 1u"ms"

fresh_code_detector() = GNSSReceiver.CodeLockDetector(;
    cn0_threshold = 30.0u"dBHz",
    reference_integration_time = PVT_GATE_REFERENCE_T,
)

fresh_carrier_detector() =
    GNSSReceiver.CarrierLockDetector(; reference_integration_time = PVT_GATE_REFERENCE_T)

# Detectors driven to ranging readiness through `update` on healthy evidence, exactly as
# `out_of_lock_code_detector` above drives one to loss: the helper never touches a field, so
# it stays independent of the dwell's layout, of its stage timings and of how ranging
# readiness is decided. The loop asks the detector instead of counting a fixed number of
# updates (~680 code periods of clean evidence on GPS L1 C/A today), so retuning the
# thresholds cannot silently turn this into a different test; the cap only stops a runaway
# loop if readiness became unreachable.
function ranging_ready_code_detector(; max_updates = 10_000)
    detector = fresh_code_detector()
    updates = 0
    while !GNSSReceiver.is_ranging_ready(detector)
        detector =
            GNSSReceiver.update(detector, 45.0u"dBHz", PVT_GATE_REFERENCE_T, 4u"ms")
        updates += 1
        @assert updates <= max_updates "code detector never reported ranging readiness"
    end
    detector
end

function ranging_ready_carrier_detector(; max_updates = 10_000)
    detector = fresh_carrier_detector()
    # A strongly in-phase prompt: the phase-lock indicator converges to cos(0)·ρ/(ρ+1),
    # far above the 0.25 threshold. Four per chunk, as a 4 ms chunk over a 1 ms code
    # period really produces.
    prompts = ntuple(_ -> complex(10.0, 0.1), 4)
    updates = 0
    while !GNSSReceiver.is_ranging_ready(detector)
        detector = GNSSReceiver.update(detector, prompts, 4u"ms")
        updates += 1
        @assert updates <= max_updates "carrier detector never reported ranging readiness"
    end
    detector
end

# A satellite that has been in lock long past `time_in_lock_before_calculating_pvt`, so the
# only thing the PVT gate can still object to is the state of its lock detectors.
function pvt_sat_state(
    system,
    prn,
    code_lock_detector,
    carrier_lock_detector;
    time_in_lock = 5.0u"s",
    in_vt_loop = false,
)
    GNSSReceiver.ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        code_lock_detector,
        carrier_lock_detector,
        uconvert(u"s", float(time_in_lock)),
        0.0u"s",
        0,
        in_vt_loop,
    )
end

@testset "is_ranging_ready on a ReceiverSatState needs both detectors" begin
    system = GPSL1CA()
    ready_code = ranging_ready_code_detector()
    ready_carrier = ranging_ready_carrier_detector()
    fresh_code = fresh_code_detector()
    fresh_carrier = fresh_carrier_detector()

    # The per-detector premise of the test: driven detectors are ready, fresh ones are not.
    @test GNSSReceiver.is_ranging_ready(ready_code)
    @test GNSSReceiver.is_ranging_ready(ready_carrier)
    @test !GNSSReceiver.is_ranging_ready(fresh_code)
    @test !GNSSReceiver.is_ranging_ready(fresh_carrier)

    combinations = Dictionary(
        [(false, false), (true, false), (false, true), (true, true)],
        [
            pvt_sat_state(system, 1, fresh_code, fresh_carrier),
            pvt_sat_state(system, 1, ready_code, fresh_carrier),
            pvt_sat_state(system, 1, fresh_code, ready_carrier),
            pvt_sat_state(system, 1, ready_code, ready_carrier),
        ],
    )
    # An AND, not an OR and not just one of the two: only the state whose *both* detectors
    # have settled may be ranged on.
    for ((code_ready, carrier_ready), state) in pairs(combinations)
        @test GNSSReceiver.is_ranging_ready(state) == (code_ready && carrier_ready)
        # And ranging readiness is a strictly later gate than lock: every one of these
        # satellites is in lock, including the three that must not be ranged on.
        @test GNSSReceiver.is_in_lock(state)
    end

    # A vector-loop member is judged on its code detector alone, mirroring `is_in_lock`. The
    # navigation filter steers both its NCOs through an outage, so its code stays aligned
    # while its carrier phase-lock indicator is starved of prompts — ANDing the carrier latch
    # would permanently exclude a member that joined the loop before that latch fired.
    @test GNSSReceiver.is_ranging_ready(
        pvt_sat_state(system, 1, ready_code, fresh_carrier; in_vt_loop = true),
    )
    @test !GNSSReceiver.is_ranging_ready(
        pvt_sat_state(system, 1, ready_code, fresh_carrier),
    )
    # But a member whose *code* detector has not settled is still not rangeable.
    @test !GNSSReceiver.is_ranging_ready(
        pvt_sat_state(system, 1, fresh_code, ready_carrier; in_vt_loop = true),
    )
end

@testset "collect_pvt_sat_states! admits only ranging-ready satellites" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    track_state = single_sat_track_state(system, 5)
    time_in_lock_before_pvt = 2u"s"

    not_ready = pvt_sat_state(system, 5, fresh_code_detector(), fresh_carrier_detector())
    ready = pvt_sat_state(
        system,
        5,
        ranging_ready_code_detector(),
        ranging_ready_carrier_detector(),
    )
    # The two states differ in ranging readiness and in nothing else the gate looks at:
    # both are in lock and both are well past the time-in-lock threshold. So a satellite
    # kept by the handover tolerance is still refused a pseudorange.
    for state in (not_ready, ready)
        @test GNSSReceiver.is_in_lock(state)
        @test state.time_in_lock > time_in_lock_before_pvt
    end
    @test !GNSSReceiver.is_ranging_ready(not_ready)
    @test GNSSReceiver.is_ranging_ready(ready)

    states = PositionVelocityTime.SatelliteState[]
    GNSSReceiver.collect_pvt_sat_states!(
        states,
        (system,),
        (; key => Dictionary([5], [not_ready])),
        track_state,
        time_in_lock_before_pvt,
    )
    @test isempty(states)

    GNSSReceiver.collect_pvt_sat_states!(
        states,
        (system,),
        (; key => Dictionary([5], [ready])),
        track_state,
        time_in_lock_before_pvt,
    )
    @test length(states) == 1
    # PVT is handed the ranging signal, and the satellite's own decoder and code phase.
    @test states[1].system == GNSSReceiver.ranging_signal(system)
    @test states[1].code_phase == get_code_phase(get_sat_state(track_state, key, 5))
    @test states[1].decoder === ready.decoder

    # The gate is not the time gate in disguise: readiness alone does not admit a satellite
    # that has not yet been in lock long enough to have decoded usable data.
    empty!(states)
    GNSSReceiver.collect_pvt_sat_states!(
        states,
        (system,),
        (;
            key => Dictionary(
                [5],
                [
                    pvt_sat_state(
                        system,
                        5,
                        ranging_ready_code_detector(),
                        ranging_ready_carrier_detector();
                        time_in_lock = 1.0u"s",
                    ),
                ],
            )
        ),
        track_state,
        time_in_lock_before_pvt,
    )
    @test isempty(states)
end

@testset "PVT waits for the satellites' loops to settle" begin
    # `time_in_lock_before_calculating_pvt` keeps a freshly locked satellite out of the
    # solve until its loops have settled: the pseudorange is built from the tracked code
    # phase, and during pull-in that phase is still a transient. Its detectors are driven to
    # ranging readiness first, so the *time* gate is the only thing this testset varies —
    # `is_ranging_ready`, the other half of the same conjunction, is covered above.
    system = GPSL1CA()
    key = get_signal_id(system)
    track_state = single_sat_track_state(system, 1)
    ready_code = ranging_ready_code_detector()
    ready_carrier = ranging_ready_carrier_detector()
    settled(time_in_lock) = (;
        key => Dictionary(
            [1],
            [
                GNSSReceiver.ReceiverSatState(
                    1,
                    GNSSDecoderState(system, 1),
                    ready_code,
                    ready_carrier,
                    time_in_lock,
                    0.0u"s",
                    0,
                    false,
                ),
            ],
        )
    )
    collect_ready(time_in_lock, threshold) = GNSSReceiver.collect_pvt_sat_states!(
        PositionVelocityTime.SatelliteState[],
        (system,),
        settled(time_in_lock),
        track_state,
        threshold,
    )

    @test isempty(collect_ready(0.0u"s", 2u"s"))   # just locked
    @test isempty(collect_ready(2.0u"s", 2u"s"))   # exactly at the gate — strictly greater
    @test length(collect_ready(2.5u"s", 2u"s")) == 1
    # A zero threshold admits a satellite the moment it locks, which is what a caller
    # asking for no settling time gets.
    @test length(collect_ready(0.004u"s", 0u"s")) == 1

    # The timer counts *continuous* lock: losing lock resets it, so a satellite that
    # flickers has to earn its settling time again.
    relocked = GNSSReceiver.increase_time_out_of_lock(
        only(settled(5.0u"s")[key]),
        4u"ms",
    )
    @test relocked.time_in_lock == 0.0u"s"
end

@testset "ReceiverSatState lock-timer transitions" begin
    system = GPSL1CA()
    state = GNSSReceiver.ReceiverSatState(
        1,
        GNSSDecoderState(system, 1),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        0.0u"s",
        0,
        false,
    )

    lost = GNSSReceiver.increase_time_out_of_lock(state, 4u"ms")
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
