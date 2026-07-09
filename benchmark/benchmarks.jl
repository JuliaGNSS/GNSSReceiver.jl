using BenchmarkTools
using GNSSReceiver
using GNSSReceiver: ReceiverState, NumAnts, process
using GNSSSignals
using Unitful
using Unitful: Hz, ms, s
using Tracking
import Acquisition
using GNSSDecoder
using PositionVelocityTime
using Dictionaries
using StaticArrays

const SUITE = BenchmarkGroup()

# ── Real signal ───────────────────────────────────────────────────────────
# The 60 s ION RTL-SDR GPS L1 recording used by the integration test (2.048 MS/s,
# 8-bit unsigned offset-binary I/Q). Each benchmark processes RUN_SECONDS of it in
# 4 ms chunks; the label reports that duration so the measured time can be judged
# against real time (real-time capable iff time < RUN_SECONDS).
const SIGNAL_URL = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
const SAMPLING_FREQ = 2.048e6u"Hz"
# GNSSSignals v3 renamed GPSL1 -> GPSL1CA. Feature-detect so this one script runs
# against both the v3 head and a pre-v3 base (benchpkg --bench-on=head).
const SYSTEM =
    isdefined(GNSSSignals, :GPSL1CA) ? GNSSSignals.GPSL1CA() : GNSSSignals.GPSL1()
const CHUNK = Int(upreferred(SAMPLING_FREQ * 4u"ms"))        # 8192 samples per process() call
const ACQ_CODE_CYCLES = 10        # 10 ms coherent integration locks the full healthy set
const RUN_SECONDS = 45            # benchmarked duration (enough to get a position fix, ~35 s)
const LOCK_SECONDS = 3            # steady-state setup: lock the sats before the timed run
const NEVER = 1_000_000u"s"       # acquire_every large enough that (re)acquisition never fires

const N_RUN = floor(Int, upreferred(SAMPLING_FREQ * (RUN_SECONDS * u"s")) / CHUNK)
const N_LOCK = floor(Int, upreferred(SAMPLING_FREQ * (LOCK_SECONDS * u"s")) / CHUNK)
const RUN_LABEL = "$(uconvert(u"s", N_RUN * CHUNK / SAMPLING_FREQ)) signal"

# Tracking v3 dropped the `Val(sampling_freq)` constructor argument.
const DC =
    hasmethod(Tracking.CPUThreadedDownconvertAndCorrelator, Tuple{}) ?
    Tracking.CPUThreadedDownconvertAndCorrelator() :
    Tracking.CPUThreadedDownconvertAndCorrelator(Val(SAMPLING_FREQ))

# Fast integer backend for the `Complex{Int16}` benchmark variants, passed
# explicitly (unlike `receive`, `process` doesn't auto-select). `max_meas = 2^7`
# matches the 8-bit ION source recentred on 128. Available on both revisions
# (both use Tracking v3), so the Int16 rows compare like-for-like across base/head
# and the float-vs-Int16 gap within a column is the integer-backend speedup.
const DC_INT16 = Tracking.Int16ThreadedDownconvertAndCorrelator(2^7)

# Feature-detect the acquisition API so this one script runs against both the
# Acquisition-v2 head (plan_acquire, single plan) and a pre-v2 base (AcquisitionPlan
# + a fine-Doppler reacquisition plan). Lets `benchpkg --bench-on=head` compute a
# real base-vs-head ratio across this API-breaking bump.
const _HAS_V2_ACQ = isdefined(Acquisition, :plan_acquire)

# `receive` renamed its acquisition-length keyword across the v3 bump
# (`num_code_cycles_for_acquisition` -> `acquisition_num_coherent_code_periods`).
# Pick whichever the loaded `receive` method declares so the receive benchmark
# below runs against both the pre-v3 base and the v3 head.
const _ACQ_CYCLES_KW =
    :acquisition_num_coherent_code_periods in Base.kwarg_decl(first(methods(receive))) ?
    (; acquisition_num_coherent_code_periods = ACQ_CODE_CYCLES) :
    (; num_code_cycles_for_acquisition = ACQ_CODE_CYCLES)

# The `receive` benchmark feeds `Complex{Int16}` chunks and lets `receive`
# auto-select the downconvert-and-correlator from the element type. On the head
# revision that resolves to Tracking's fast integer backend, which needs
# `max_meas` (the front-end full-scale). The ION recording is 8-bit offset-binary
# recentred on 128, so |real|/|imag| ≤ 128 → `max_meas = 2^7`. The pre-`max_meas`
# base revision has no such keyword, so feature-detect it: base then falls back to
# its CPU-on-Int16 default and the base/head ratio shows the integer-backend win.
const _MAX_MEAS_KW =
    :max_meas in Base.kwarg_decl(first(methods(receive))) ? (; max_meas = 2^7) : (;)

# `plan_args` splat into `process` so the same call handles both signatures:
# v2 process(rs, acq_plan, meas, …); v1 process(rs, acq_plan, fast_re_acq_plan, meas, …).
_process(rs, plan_args, meas, dc; kwargs...) = process(
    rs,
    plan_args...,
    meas,
    SYSTEM,
    SAMPLING_FREQ;
    downconvert_and_correlator = dc,
    num_ants = NumAnts(1),
    approximate_year = 2017,
    kwargs...,
)

function make_receiver_and_plan(::Type{T}) where {T}
    nacq = round(
        Int,
        get_code_length(SYSTEM) * upreferred(SAMPLING_FREQ / get_code_frequency(SYSTEM)) *
        ACQ_CODE_CYCLES,
    )
    rs = ReceiverState(T, SYSTEM; num_ants = NumAnts(1), num_samples_for_acquisition = nacq)
    plan_args = if _HAS_V2_ACQ
        (Acquisition.plan_acquire(
            SYSTEM,
            float(SAMPLING_FREQ),
            collect(1:32);
            num_coherently_integrated_code_periods = ACQ_CODE_CYCLES,
        ),)
    else
        acq_plan = Acquisition.AcquisitionPlan(SYSTEM, nacq, float(SAMPLING_FREQ); prns = 1:32)
        coarse_step = 2 * SAMPLING_FREQ / nacq
        fine_step = 1 / 4 / (nacq / SAMPLING_FREQ)
        fast = Acquisition.AcquisitionPlan(
            SYSTEM,
            nacq,
            SAMPLING_FREQ;
            dopplers = -coarse_step:fine_step:coarse_step,
            prns = 1:32,
        )
        (acq_plan, fast)
    end
    return rs, plan_args
end

# Load `nchunks` consecutive 4 ms chunks of the recording as ComplexF32 vectors.
# Fetches only the needed byte range (not the whole 246 MB file).
function load_chunks(nchunks)
    cache = joinpath(tempdir(), "gnssreceiver_bench_RTLSDR_Bands-L1.uint8")
    nbytes = 2 * nchunks * CHUNK
    if !isfile(cache) || filesize(cache) < nbytes
        run(`curl -sfL -r 0-$(nbytes - 1) -o $cache $SIGNAL_URL`)
    end
    chunks = Vector{Vector{ComplexF32}}(undef, nchunks)
    raw = Vector{UInt8}(undef, 2 * CHUNK)
    open(cache) do io
        for k = 1:nchunks
            read!(io, raw)
            chunks[k] = ComplexF32[
                ComplexF32(Float32(raw[2i-1]) - 127.5f0, Float32(raw[2i]) - 127.5f0) for i = 1:CHUNK
            ]
        end
    end
    return chunks
end

# All chunks needed: LOCK_SECONDS of setup followed by RUN_SECONDS of timed run.
const CHUNKS = load_chunks(N_LOCK + N_RUN)

# `Complex{Int16}` copies of the same chunks for the integer-backend variants. The
# float `CHUNKS` are the 8-bit source recentred on 127.5; round to `Int16` (values
# within ±128, matching `DC_INT16`'s `max_meas = 2^7`).
const CHUNKS_INT16 =
    [complex.(round.(Int16, real.(c)), round.(Int16, imag.(c))) for c in CHUNKS]

# `receive` consumes matrix chunks (num_samples × num_ants) off a channel, whereas
# the direct-`process` benchmarks fold over plain vectors. Materialise the first
# N_RUN Int16 chunks as (CHUNK, 1) matrices once, so the timed feed only enqueues
# them — and so `receive` auto-selects the integer backend from the element type.
const MATRIX_CHUNKS = [reshape(c, CHUNK, 1) for c in @view CHUNKS_INT16[1:N_RUN]]

# Drive `process` with a plain reassignment `for` loop rather than `foldl`. This
# mirrors how `receive` actually calls `process`. It matters for allocation: the
# `process`/`foldl` combination is fully type-stable (the fold operator infers a
# concrete return equal to its input), but reaching `process` through `foldl`'s
# higher-order reduction defeats the optimizer's escape analysis for the immutable
# temporaries that `Tracking.track!` builds each chunk, so they get heap-allocated
# (~4x the real allocation). A `for` loop keeps `process` in one frame where those
# temporaries are elided, matching real-world `receive` behaviour.
function run_process(rs, plan_args, chunks, dc, acquire_every)
    for c in chunks
        rs = _process(rs, plan_args, c, dc; acquire_every)
    end
    return rs
end

# Drive the full public `receive` pipeline: a producer task feeds `Complex{Int16}`
# chunks onto a MatrixSizedChannel, `receive` spawns its own acquisition+tracking
# task, and the resulting data channel is drained here. No `downconvert_and_correlator`
# is passed, so `receive` auto-selects it from the element type — the integer
# backend on head (via `_MAX_MEAS_KW`), the CPU fallback on the pre-`max_meas` base.
# The base/head delta is the integer-backend speedup diluted by the channel
# producer/consumer plumbing and per-chunk ReceiverState churn that the
# `process`-only benchmarks bypass.
function run_receive(chunks, acquire_every)
    measurement_channel = GNSSReceiver.MatrixSizedChannel{Complex{Int16}}(CHUNK, 1) do ch
        for c in chunks
            put!(ch, c)
        end
    end
    data_channel = receive(
        measurement_channel,
        SYSTEM,
        SAMPLING_FREQ;
        num_ants = NumAnts(1),
        acquire_every,
        _ACQ_CYCLES_KW...,
        _MAX_MEAS_KW...,
    )
    GNSSReceiver.consume_channel(_ -> nothing, data_channel)
    return nothing
end

# ── Benchmark: process without acquisition over RUN_SECONDS ───────────────
# Acquire and lock over the first LOCK_SECONDS (setup, not timed), then benchmark
# processing the following RUN_SECONDS with acquire_every huge — no acquisition in
# the timed run. A fix is obtained partway through, so PVT runs too. Parameterized
# on sample element type + correlator so the float and Int16 variants time the same
# work through their respective backends (fair like-for-like within each column).
function bench_process_without_acquisition(::Type{T}, dc, chunks) where {T}
    rs, plan_args = make_receiver_and_plan(T)
    locked = run_process(rs, plan_args, @view(chunks[1:N_LOCK]), dc, NEVER)
    timed_chunks = chunks[N_LOCK+1:N_LOCK+N_RUN]
    # `process` mutates the receiver state in place — the in-place `track!`
    # (c1c8b26) and the in-place `map!` in `update_receiver_sat_states`. If every
    # BenchmarkTools evaluation re-ran the *same* `locked` object, each one would
    # start from the previous run's already-advanced state: the satellites fall
    # out of lock and later evaluations track almost nothing, collapsing the
    # reported minimum time and memory to a meaningless (tiny) value. Give each
    # sample a fresh `deepcopy` of the locked state and pin `evals=1`, so every
    # measured sample is one honest 45 s forward pass over the full sat set.
    @benchmarkable(
        run_process(state, $plan_args, $timed_chunks, $dc, NEVER),
        setup = (state = deepcopy($locked)),
        evals = 1,
    )
end

# ── Benchmark: process with acquisition every 10 sec over RUN_SECONDS ─────
# Process RUN_SECONDS from a fresh receiver, re-acquiring every 10 s as in normal
# operation — acquire, lock, decode, and reach a position fix. Parameterized like
# `bench_process_without_acquisition`.
function bench_process_with_acquisition(::Type{T}, dc, chunks) where {T}
    rs, plan_args = make_receiver_and_plan(T)
    timed_chunks = chunks[1:N_RUN]
    # Same in-place-mutation caveat as above: hand each sample a fresh deepcopy
    # of the (empty) starting receiver and pin evals=1 so every sample is one
    # honest 45 s run that acquires and locks from scratch. The acquisition plan
    # holds only reusable FFT scratch (overwritten each `acquire!`), so it is
    # shared across samples rather than copied.
    @benchmarkable(
        run_process(state, $plan_args, $timed_chunks, $dc, 10u"s"),
        setup = (state = deepcopy($rs)),
        evals = 1,
    )
end

# ── Benchmark: full receive() pipeline with acquisition every 10 sec ──────
# End-to-end analogue of `bench_process_with_acquisition`, but through the public
# `receive` entry point (channel producer/consumer + spawned tracking task)
# instead of a direct `process` fold — so it captures the plumbing overhead the
# process benchmarks isolate away. Fresh receiver, re-acquiring every 10 s.
function bench_receive_with_acquisition()
    @benchmarkable run_receive($MATRIX_CHUNKS, 10u"s")
end

# ── Register benchmarks ───────────────────────────────────────────────────
# Float and Int16 variants of each process benchmark so float-vs-Int16 is compared
# like-for-like within a single build (the integer-backend speedup is a Tracking
# feature present on both revisions, not a base/head difference).
SUITE["process without acquisition ($RUN_LABEL)"]["1-ant float"] =
    bench_process_without_acquisition(ComplexF32, DC, CHUNKS)
SUITE["process without acquisition ($RUN_LABEL)"]["1-ant Int16"] =
    bench_process_without_acquisition(Complex{Int16}, DC_INT16, CHUNKS_INT16)
SUITE["process with acquisition every 10 sec ($RUN_LABEL)"]["1-ant float"] =
    bench_process_with_acquisition(ComplexF32, DC, CHUNKS)
SUITE["process with acquisition every 10 sec ($RUN_LABEL)"]["1-ant Int16"] =
    bench_process_with_acquisition(Complex{Int16}, DC_INT16, CHUNKS_INT16)
SUITE["receive with acquisition every 10 sec ($RUN_LABEL)"]["1-ant"] = bench_receive_with_acquisition()
