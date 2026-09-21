# Issue #142: a stretch of all-zero samples — a front-end / DMA dropout, as in the
# M2SDR recording the issue was found on — must not kill the receiver. Tracking turns
# an integration window that is zero from end to end into an all-zero correlator
# record, and its discriminators turn that into `0/0`: NaN loop-filter states, NaN
# Dopplers, and an `InexactError: Int64(NaN)` from the next correlate
# (JuliaGNSS/Tracking.jl#234). The receiver's dead-input guard detects the dropout
# before tracking and drops the affected band's satellites through the normal
# lost-lock path instead, so they are reacquired once the signal returns.
@testset "dead-input guard (issue #142)" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    band_key = get_band_id(GNSSReceiver.system_band(system))
    sampling_freq = 5e6Hz
    num_samples = 20000 # 4 ms: four code periods per chunk
    window = 5000       # one GPS L1 C/A code period in samples

    # A receiver tracking PRN 1 from `code_phase`, with no acquisition scan due (the
    # zero samples must reach `track!`, not `acquire!`).
    function tracked_receiver(code_phase)
        base = GNSSReceiver.ReceiverState(
            ComplexF64,
            system;
            num_samples_for_acquisition = num_samples,
        )
        track_state = merge_sats(
            base.track_state,
            key,
            [
                GNSSReceiver.create_tracked_sat(
                    GNSSReceiver.tracking_signals(system),
                    1,
                    code_phase,
                    20.0Hz,
                    NumAnts(1),
                    base.track_state.doppler_estimator,
                ),
            ],
        )
        ReceiverState(
            track_state,
            (; key => Dictionary([1], [GNSSReceiver.ReceiverSatState(system, 1)])),
            base.acquisition_buffers,
            NamedTuple{(band_key,)}((0.0u"s",)),
            PVTSolution(),
            SatelliteState[],
            nothing,
            0.0u"s",
            -Inf * 1.0u"s",
        )
    end
    acq_plans = (; key => plan_acquire(system, float(sampling_freq), collect(1:32)))
    step(receiver_state, measurement) = GNSSReceiver.process(
        receiver_state,
        acq_plans,
        (measurement,),
        ((system,),),
        sampling_freq,
        (0.0Hz,);
        acq_pfa = 1e-12,
    )
    rng = Random.Xoshiro(142)
    noise() = randn(rng, ComplexF64, num_samples, 1)
    # Noise with the samples in `dead` zeroed — a dropout inside the chunk.
    function noise_with_dropout(dead)
        m = noise()
        m[dead, :] .= 0
        m
    end
    tracked_prns(receiver_state) =
        collect(keys(get_sat_states(receiver_state.track_state, key)))
    sat_state(receiver_state) = receiver_state.receiver_sat_states[key][1]
    dropped_warning = (:warn, r"Dead input")

    @testset "an all-zero chunk drops the tracked satellite instead of crashing" begin
        receiver_state = tracked_receiver(0.0)
        # Without the guard this is the crash of the issue: `InexactError: Int64(NaN)`
        # out of `track!`.
        next_state = @test_logs dropped_warning match_mode = :any step(
            receiver_state,
            zeros(ComplexF64, num_samples, 1),
        )
        @test isempty(tracked_prns(next_state))
        @test !GNSSReceiver.is_in_lock(sat_state(next_state))
        @test next_state.trailing_dead_samples[band_key] == num_samples
        # The receiver keeps running on the dead stream and on the signal's return; the
        # dropped satellite waits in the reacquisition back-off.
        next_state = step(next_state, zeros(ComplexF64, num_samples, 1))
        @test next_state.trailing_dead_samples[band_key] == 2 * num_samples
        next_state = step(next_state, noise())
        @test next_state.trailing_dead_samples[band_key] == 0
        @test isempty(tracked_prns(next_state))
        @test sat_state(next_state).time_out_of_lock > 0.0u"s"
        @test next_state.runtime == 3 * num_samples / sampling_freq
    end

    @testset "a dropout inside a chunk that swallows a whole code period" begin
        # Code phase 0: integration windows end at samples 5000, 10000, 15000, 20000.
        # Zeros over 4001:10500 cover the window 5001:10000 entirely.
        receiver_state = tracked_receiver(0.0)
        next_state = @test_logs dropped_warning match_mode = :any step(
            receiver_state,
            noise_with_dropout(4001:10500),
        )
        @test isempty(tracked_prns(next_state))
        @test next_state.trailing_dead_samples[band_key] == 0
    end

    @testset "a dropout straddling two chunks" begin
        # 409.2 chips in: the windows end at 3000, 8000, 13000, 18000 and — in the next
        # chunk — 3000. Zeros over the last 2000 samples of the first chunk and the
        # first 3000 of the second cover the window 18001:23000 entirely, while
        # neither chunk alone holds a run as long as one code period.
        receiver_state = tracked_receiver(409.2)
        first_state = step(receiver_state, noise_with_dropout(18001:20000))
        @test tracked_prns(first_state) == [1]
        @test first_state.trailing_dead_samples[band_key] == 2000
        second_state = @test_logs dropped_warning match_mode = :any step(
            first_state,
            noise_with_dropout(1:3000),
        )
        @test isempty(tracked_prns(second_state))
        @test second_state.trailing_dead_samples[band_key] == 0
    end

    @testset "a dropout shorter than one code period is left to the tracking loops" begin
        # No window can be all-zero, so no record is: the loops ride it out.
        receiver_state = tracked_receiver(0.0)
        next_state = step(receiver_state, noise_with_dropout(1001:5900))
        @test tracked_prns(next_state) == [1]
        @test GNSSReceiver.is_in_lock(sat_state(next_state))
        @test next_state.trailing_dead_samples[band_key] == 0
        # ... including one at the end of the chunk, which is only remembered.
        next_state = step(next_state, noise_with_dropout(16001:20000))
        @test tracked_prns(next_state) == [1]
        @test next_state.trailing_dead_samples[band_key] == 4000
        next_state = step(next_state, noise())
        @test tracked_prns(next_state) == [1]
        @test next_state.trailing_dead_samples[band_key] == 0
    end

    @testset "dead_run" begin
        dead_run = GNSSReceiver.dead_run
        m = randn(rng, ComplexF64, 100)
        @test dead_run(m, 0, 10) == (false, 0)
        @test dead_run(zeros(ComplexF64, 100), 0, 10) == (true, 100)
        # A chunk shorter than the window is dead only together with what preceded it.
        @test dead_run(zeros(ComplexF64, 4), 0, 10) == (false, 4)
        @test dead_run(zeros(ComplexF64, 4), 6, 10) == (true, 10)
        # Leading zeros complete the previous chunk's trailing run.
        m = randn(rng, ComplexF64, 100)
        m[1:5] .= 0
        @test dead_run(m, 4, 10) == (false, 0)
        @test dead_run(m, 5, 10) == (true, 0)
        # Trailing zeros are reported for the next chunk; a whole window of them is dead.
        m = randn(rng, ComplexF64, 100)
        m[94:100] .= 0
        @test dead_run(m, 0, 10) == (false, 7)
        m[91:100] .= 0
        @test dead_run(m, 0, 10) == (true, 10)
        # An internal run is dead from one window's length on, wherever it lies.
        for start = 2:60
            m = randn(rng, ComplexF64, 100)
            m[start:(start+8)] .= 0
            @test dead_run(m, 0, 10) == (false, 0)
            m[start:(start+9)] .= 0
            @test dead_run(m, 0, 10) == (true, 0)
        end
        # Multi-antenna: a sample is dead only when every antenna reads zero.
        m = randn(rng, ComplexF64, 100, 2)
        m[:, 1] .= 0
        @test dead_run(m, 0, 10) == (false, 0)
        m[:, 2] .= 0
        @test dead_run(m, 0, 10) == (true, 100)
        # Integer samples, as the Int16 backend delivers.
        m = Complex{Int16}.(round.(randn(rng, ComplexF32, 100) * 512))
        m[41:60] .= 0
        @test dead_run(m, 0, 10) == (true, 0)
    end
end
