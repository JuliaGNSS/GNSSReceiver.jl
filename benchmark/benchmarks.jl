using BenchmarkTools
using GNSSReceiver
using GNSSReceiver: ReceiverState, ReceiverSatState, process, NumAnts, SampleBuffer
using GNSSSignals
using Unitful: Hz, s, ms
using Unitful
using Tracking
using Tracking: SatState, SystemSatsState, TrackState, EarlyPromptLateCorrelator, BitBuffer
using Acquisition: AcquisitionPlan
using GNSSDecoder
using PositionVelocityTime
using Dictionaries
using StaticArrays

const SUITE = BenchmarkGroup()

# ── Helper: build a ReceiverState with N satellites already tracked ────────

function make_receiver_state(;
    system = GPSL1(),
    num_samples = 20000,
    sampling_freq = 5e6Hz,
    num_ants = 1,
    num_sats = 0,
    fill_acq_buffer = false,
    runtime = 0.0u"s",
    last_time_acquisition_ran = -Inf * 1.0u"s",
)
    prns = 1:num_sats
    correlator = Tracking.get_default_correlator(system, NumAnts(num_ants))
    post_corr_filter = GNSSReceiver.create_post_corr_filter(NumAnts(num_ants))

    correlator_type = typeof(correlator)
    filter_type = typeof(post_corr_filter)
    sat_states = SatState{correlator_type,filter_type}[
        SatState(
            system,
            prn,
            10.5 + prn * 0.1,
            (1000.0 + prn * 10) * Hz;
            num_ants = NumAnts(num_ants),
            post_corr_filter,
        ) for prn in prns
    ]

    track_state = TrackState(system, sat_states)

    decoder_type = typeof(GNSSDecoder.GNSSDecoderState(system, 1))
    receiver_sat_states_dict = Dictionary(
        collect(prns),
        ReceiverSatState{decoder_type}[ReceiverSatState(system, prn) for prn in prns],
    )

    acquisition_buffer = SampleBuffer(ComplexF64, num_samples)
    if fill_acq_buffer
        # Fill the buffer so acquisition can fire
        acquisition_buffer =
            GNSSReceiver.SampleBuffers.buffer(acquisition_buffer, randn(ComplexF64, num_samples))
    end

    pvt = PositionVelocityTime.PVTSolution()

    ReceiverState(
        track_state,
        (receiver_sat_states_dict,),
        acquisition_buffer,
        pvt,
        runtime,
        last_time_acquisition_ran,
        -Inf * 1.0u"s",
        0,
    )
end

function make_acq_plans(; system = GPSL1(), num_samples = 20000, sampling_freq = 5e6Hz)
    acq_plan = AcquisitionPlan(system, num_samples, float(sampling_freq))
    coarse_step = 2 * sampling_freq / num_samples
    fine_step = 1 / 4 / (num_samples / sampling_freq)
    fine_doppler_range = -coarse_step:fine_step:coarse_step
    fast_re_acq_plan =
        AcquisitionPlan(system, num_samples, sampling_freq; dopplers = fine_doppler_range)
    acq_plan, fast_re_acq_plan
end

# Check if process supports downconvert_and_correlator keyword
const _process_supports_dc = any(methods(GNSSReceiver.process)) do m
    :downconvert_and_correlator in Base.kwarg_decl(m)
end

function _process_kwargs(; num_ants, sampling_freq)
    kwargs = Dict{Symbol,Any}(:num_ants => NumAnts(num_ants))
    if _process_supports_dc
        kwargs[:downconvert_and_correlator] =
            Tracking.CPUThreadedDownconvertAndCorrelator(Val(sampling_freq))
    end
    return pairs(kwargs)
end

# ── Benchmark: initialization with acquisition ────────────────────────────

function bench_process_with_acquisition(; num_ants = 1)
    system = GPSL1()
    num_samples = 20000
    sampling_freq = 5e6Hz

    receiver_state = make_receiver_state(;
        system,
        num_samples,
        sampling_freq,
        num_ants,
        num_sats = 0,
        fill_acq_buffer = true,
        runtime = 10.0u"s",
        last_time_acquisition_ran = 0.0u"s",
    )
    acq_plan, fast_re_acq_plan = make_acq_plans(; system, num_samples, sampling_freq)
    measurement =
        num_ants == 1 ? randn(ComplexF64, num_samples) :
        randn(ComplexF64, num_samples, num_ants)
    kwargs = _process_kwargs(; num_ants, sampling_freq)

    @benchmarkable GNSSReceiver.process(
        $receiver_state,
        $acq_plan,
        $fast_re_acq_plan,
        $measurement,
        $system,
        $sampling_freq;
        $kwargs...,
    )
end

# ── Benchmark: steady-state tracking (no acquisition) ─────────────────────

function bench_process_steady_state(; num_ants = 1, num_sats = 8)
    system = GPSL1()
    num_samples = 20000
    sampling_freq = 5e6Hz

    # Runtime is 5s, last acquisition at 0s, acquire_every defaults to 10s
    # → no acquisition fires. Buffer not full → reacquisition also skipped.
    receiver_state = make_receiver_state(;
        system,
        num_samples,
        sampling_freq,
        num_ants,
        num_sats,
        fill_acq_buffer = false,
        runtime = 5.0u"s",
        last_time_acquisition_ran = 0.0u"s",
    )
    acq_plan, fast_re_acq_plan = make_acq_plans(; system, num_samples, sampling_freq)
    measurement =
        num_ants == 1 ? randn(ComplexF64, num_samples) :
        randn(ComplexF64, num_samples, num_ants)
    kwargs = _process_kwargs(; num_ants, sampling_freq)

    @benchmarkable GNSSReceiver.process(
        $receiver_state,
        $acq_plan,
        $fast_re_acq_plan,
        $measurement,
        $system,
        $sampling_freq;
        $kwargs...,
    )
end

# ── Register benchmarks ──────────────────────────────────────────────────

SUITE["process with acquisition"]["1-ant"] = bench_process_with_acquisition(; num_ants = 1)
SUITE["process with acquisition"]["4-ant"] = bench_process_with_acquisition(; num_ants = 4)

SUITE["process steady-state 8sat"]["1-ant"] = bench_process_steady_state(; num_ants = 1)
SUITE["process steady-state 8sat"]["4-ant"] = bench_process_steady_state(; num_ants = 4)
