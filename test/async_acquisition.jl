# Tests for acquisition running off the processing task (issue #107).
#
# Two levels: the scheduler's contract is driven directly with a worker whose
# scan is executed by the test (so dispatch, back-pressure and the merge's
# code-phase advance are pinned deterministically, with no timing assumptions),
# and then the whole receiver is run against a real synthetic signal with
# `acquire_async = true` — which can only end in lock if the delayed handover
# lands on the right code phase.

using GNSSReceiver:
    InlineAcquisition,
    AsyncAcquisition,
    BandAcquisitionWorker,
    AcquisitionRequest,
    AcquisitionResponse,
    close_acquisition!

# A GPS L1 C/A signal at a known code phase and Doppler, plus unit-variance noise.
function l1ca_signal(num_samples, prn, sampling_freq; code_phase, doppler, amplitude, rng)
    system = GPSL1CA()
    fs = ustrip(Hz, sampling_freq)
    code_freq =
        ustrip(Hz, get_code_frequency(system)) +
        doppler * get_code_center_frequency_ratio(system)
    samples = Matrix{ComplexF64}(undef, num_samples, 1)
    for n = 0:(num_samples-1)
        t = n / fs
        samples[n+1, 1] =
            amplitude * get_code(system, code_phase + code_freq * t, prn) *
            cis(2π * doppler * t) + randn(rng, ComplexF64) / sqrt(2)
    end
    samples
end

# A worker with no task behind it: the test plays the worker, so every step of
# the scheduler's per-chunk contract is observed exactly when it happens.
function idle_worker(acq_plan, group_key, ::Type{T}, interm_freq, acq_pfa) where {T}
    prn_type = NamedTuple{(group_key,),Tuple{Vector{Int}}}
    result_vector_type =
        GNSSReceiver._scan_result_type(acq_plan, T, interm_freq, acq_pfa)
    result_type = NamedTuple{(group_key,),Tuple{result_vector_type}}
    BandAcquisitionWorker(
        Channel{AcquisitionRequest{T,prn_type}}(1),
        Channel{AcquisitionResponse{T,result_type}}(1),
        prn_type((collect(Int, acq_plan.avail_prns),)),
        result_type((result_vector_type(),)),
        T[],
        false,
        nothing,
        0,
        0,
        0,
        NaN,
    )
end

@testset "A scan is dispatched once, and merged whenever it comes back" begin
    system = GPSL1CA()
    prn = 7
    key = get_signal_id(system)
    band_key = get_band_id(GNSSReceiver.system_band(system))
    sampling_freq = 4e6Hz
    chunk = 4000                      # 1 ms
    window = 4 * chunk                # the acquisition window, 4 ms
    true_code_phase = 231.7
    true_doppler = 1000.0
    rng = Random.Xoshiro(0xACC0)

    acq_plan = plan_acquire(
        system,
        float(sampling_freq),
        [prn];
        num_coherently_integrated_code_periods = 4,
    )
    worker = idle_worker(acq_plan, key, ComplexF64, 0.0Hz, GNSSReceiver.DEFAULT_ACQ_PFA)
    scheduler = AsyncAcquisition(NamedTuple{(band_key,)}((worker,)))

    state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = window,
    )
    signal = l1ca_signal(
        40 * chunk,
        prn,
        sampling_freq;
        code_phase = true_code_phase,
        doppler = true_doppler,
        amplitude = 1.0,
        rng,
    )
    frame(i) = signal[((i-1)*chunk+1):(i*chunk), :]

    step!(i) = GNSSReceiver.process(
        state,
        (; key => acq_plan),
        (frame(i),),
        ((system,),),
        sampling_freq,
        (0.0Hz,);
        acquisition = scheduler,
    )

    # The window is 4 chunks long, so nothing can be dispatched before it fills.
    for i = 1:3
        state = step!(i)
        @test !worker.in_flight
        @test !isready(worker.requests)
    end
    state = step!(4)
    @test worker.in_flight
    @test isready(worker.requests)
    request = take!(worker.requests)
    @test length(request.samples) == window
    @test request.prns[key] == [prn]
    # The window ends with the 4th frame and starts `window` samples earlier —
    # i.e. at the very beginning of the run.
    @test request.window_start_runtime ≈ 0.0u"s" atol = 1e-12u"s"

    # While that scan is outstanding no second one is dispatched, however many
    # chunks go by — and every one of those chunks is processed as usual.
    for i = 5:12
        state = step!(i)
        @test !isready(worker.requests)
        @test isempty(get_sat_states(state.track_state, key))
    end
    @test state.runtime ≈ 12 * chunk / sampling_freq

    # Now play the worker: run the scan the receiver asked for, on the window it
    # handed over, and answer with it.
    results = GNSSReceiver._run_scan(
        (acq_plan,),
        request,
        0.0Hz,
        GNSSReceiver.DEFAULT_ACQ_PFA,
        true,
        typeof(worker.empty_results),
    )
    acquired = only(results[key])
    @test Acquisition.is_detected(acquired; pfa = GNSSReceiver.DEFAULT_ACQ_PFA)
    put!(
        worker.responses,
        AcquisitionResponse(results, request.window_start_runtime, request.samples, 0.42),
    )

    # The next chunk merges it. The handover code phase must be the acquired one
    # advanced over everything that has been processed since the window started —
    # 12 chunks — not the raw acquisition estimate.
    merge_runtime = state.runtime
    state = step!(13)
    @test !worker.in_flight
    sat_states = get_sat_states(state.track_state, key)
    @test haskey(sat_states, prn)
    expected = GNSSReceiver.advance_code_phase(
        acquired,
        round(Int, ustrip(upreferred((merge_runtime - request.window_start_runtime) * sampling_freq))),
    )
    # `process` also tracks the merging chunk, which advances the code phase by
    # one more frame; compare against the seed the handover used, modulo the code
    # length, with a tolerance of a fraction of a chip.
    seeded = mod(get_code_phase(sat_states[prn]) - chunk * ustrip(Hz, get_code_frequency(system)) / ustrip(Hz, sampling_freq),
                 get_code_length(system))
    @test min(
        abs(seeded - expected.code_phase),
        get_code_length(system) - abs(seeded - expected.code_phase),
    ) < 1.0
    # The dispatched window is released back to the worker for the next scan.
    @test length(worker.spare) == window
    # Diagnostics: one scan out, one back, one satellite merged, and the scan's
    # measured duration carried with it.
    @test worker.dispatched_scans == 1
    @test worker.completed_scans == 1
    @test worker.detected_prns == 1
    @test worker.last_scan_seconds == 0.42

    # And the periodic timer was restarted at the dispatching chunk (whose
    # runtime is the start of the 4th frame), not at the merge.
    @test state.last_time_acquisition_ran[band_key] ≈ 3 * chunk / sampling_freq
end

@testset "Asynchronous acquisition locks a satellite through `receive`" begin
    # End to end: only a merge that puts the handover on the right code phase
    # (and a pipeline that kept tracking while the scan ran) ends in lock.
    system = GPSL1CA()
    prn = 19
    key = get_signal_id(system)
    sampling_freq = 4e6Hz
    chunk = 4000
    num_chunks = 400
    rng = Random.Xoshiro(0xBEE7)
    signal = l1ca_signal(
        num_chunks * chunk,
        prn,
        sampling_freq;
        code_phase = 512.3,
        doppler = -800.0,
        amplitude = 0.35,       # ≈ 48 dBHz against unit-variance noise
        rng,
    )

    measurement_channel = GNSSReceiver.spawn_signal_channel_thread(;
        T = ComplexF64,
        num_samples = chunk,
        num_antenna_channels = 1,
    ) do ch
        for c = 1:num_chunks
            put!(ch, signal[((c-1)*chunk+1):(c*chunk), :])
        end
    end

    data_channel = receive(
        measurement_channel,
        system,
        sampling_freq;
        prns = [prn],
        acquire_async = true,
        # A synthetic signal carries no navigation data, so PVT can never solve.
        time_in_lock_before_calculating_pvt = 1000u"s",
    )
    results = collect_data(data_channel)

    @test !isempty(results)
    final = last(results)
    @test haskey(final.sat_data, (key, prn))
    @test final.sat_data[(key, prn)].cn0 > 40dBHz
end

@testset "The inline scheduler is unchanged by the new seam" begin
    # Same signal, same settings, inline: the default path must still acquire and
    # lock (and is what every other test in the suite exercises).
    system = GPSL1CA()
    prn = 19
    key = get_signal_id(system)
    sampling_freq = 4e6Hz
    chunk = 4000
    num_chunks = 200
    rng = Random.Xoshiro(0xBEE7)
    signal = l1ca_signal(
        num_chunks * chunk,
        prn,
        sampling_freq;
        code_phase = 512.3,
        doppler = -800.0,
        amplitude = 0.35,
        rng,
    )
    measurement_channel = GNSSReceiver.spawn_signal_channel_thread(;
        T = ComplexF64,
        num_samples = chunk,
        num_antenna_channels = 1,
    ) do ch
        for c = 1:num_chunks
            put!(ch, signal[((c-1)*chunk+1):(c*chunk), :])
        end
    end
    results = collect_data(
        receive(
            measurement_channel,
            system,
            sampling_freq;
            prns = [prn],
            time_in_lock_before_calculating_pvt = 1000u"s",
        ),
    )
    @test last(results).sat_data[(key, prn)].cn0 > 40dBHz
end

@testset "Closing the scheduler ends its workers" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    band_key = get_band_id(GNSSReceiver.system_band(system))
    acq_plan = plan_acquire(GPSL1CA(), 4e6Hz, [1])
    scheduler = AsyncAcquisition(
        (band_key,),
        ((system,),),
        (; key => acq_plan),
        (ComplexF64,),
        (0.0Hz,),
        GNSSReceiver.DEFAULT_ACQ_PFA,
        true,
    )
    worker = scheduler.workers[band_key]
    @test !worker.in_flight
    close_acquisition!(scheduler)
    # The worker task takes from a closed channel and returns; the channels stay
    # closed so a late dispatch cannot silently queue work for a finished run.
    @test !isopen(worker.requests)
    @test !isopen(worker.responses)
    # And the run does not end while the worker is still computing: shutdown
    # waits for it, so nothing is left running into the process teardown.
    @test istaskdone(worker.task)
    # Idempotent, and a no-op for the inline scheduler.
    @test isnothing(close_acquisition!(scheduler))
    @test isnothing(close_acquisition!(InlineAcquisition()))
end
