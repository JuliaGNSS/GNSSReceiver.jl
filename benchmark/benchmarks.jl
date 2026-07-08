using BenchmarkTools
using GNSSReceiver
using GNSSReceiver: ReceiverState, NumAnts, process
using GNSSSignals
using Unitful
using Unitful: Hz, ms, s
using Tracking
using Acquisition: AcquisitionPlan
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
const SYSTEM = GPSL1()
const CHUNK = Int(upreferred(SAMPLING_FREQ * 4u"ms"))        # 8192 samples per process() call
const ACQ_CODE_CYCLES = 10        # 10 ms coherent integration locks the full healthy set
const RUN_SECONDS = 45            # benchmarked duration (enough to get a position fix, ~35 s)
const LOCK_SECONDS = 3            # steady-state setup: lock the sats before the timed run
const NEVER = 1_000_000u"s"       # acquire_every large enough that (re)acquisition never fires

const N_RUN = floor(Int, upreferred(SAMPLING_FREQ * (RUN_SECONDS * u"s")) / CHUNK)
const N_LOCK = floor(Int, upreferred(SAMPLING_FREQ * (LOCK_SECONDS * u"s")) / CHUNK)
const RUN_LABEL = "$(uconvert(u"s", N_RUN * CHUNK / SAMPLING_FREQ)) signal"

const DC = Tracking.CPUThreadedDownconvertAndCorrelator(Val(SAMPLING_FREQ))

_process(rs, acq_plan, fast, meas; kwargs...) = process(
    rs,
    acq_plan,
    fast,
    meas,
    SYSTEM,
    SAMPLING_FREQ;
    downconvert_and_correlator = DC,
    num_ants = NumAnts(1),
    approximate_year = 2017,
    kwargs...,
)

function make_receiver_and_plans()
    nacq = round(
        Int,
        get_code_length(SYSTEM) * upreferred(SAMPLING_FREQ / get_code_frequency(SYSTEM)) *
        ACQ_CODE_CYCLES,
    )
    rs = ReceiverState(ComplexF32, SYSTEM; num_ants = NumAnts(1), num_samples_for_acquisition = nacq)
    acq_plan = AcquisitionPlan(SYSTEM, nacq, float(SAMPLING_FREQ); prns = 1:32)
    coarse_step = 2 * SAMPLING_FREQ / nacq
    fine_step = 1 / 4 / (nacq / SAMPLING_FREQ)
    fast = AcquisitionPlan(SYSTEM, nacq, SAMPLING_FREQ; dopplers = -coarse_step:fine_step:coarse_step, prns = 1:32)
    return rs, acq_plan, fast
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

run_process(rs, acq_plan, fast, chunks, acquire_every) =
    foldl((s, c) -> _process(s, acq_plan, fast, c; acquire_every), chunks; init = rs)

# ── Benchmark: process without acquisition over RUN_SECONDS ───────────────
# Acquire and lock over the first LOCK_SECONDS (setup, not timed), then benchmark
# processing the following RUN_SECONDS with acquire_every huge — no acquisition in
# the timed run. A fix is obtained partway through, so PVT runs too.
function bench_process_without_acquisition()
    rs, acq_plan, fast = make_receiver_and_plans()
    locked = run_process(rs, acq_plan, fast, @view(CHUNKS[1:N_LOCK]), NEVER)
    timed_chunks = CHUNKS[N_LOCK+1:N_LOCK+N_RUN]
    @benchmarkable run_process($locked, $acq_plan, $fast, $timed_chunks, NEVER)
end

# ── Benchmark: process with acquisition every 10 sec over RUN_SECONDS ─────
# Process RUN_SECONDS from a fresh receiver, re-acquiring every 10 s as in normal
# operation — acquire, lock, decode, and reach a position fix.
function bench_process_with_acquisition()
    rs, acq_plan, fast = make_receiver_and_plans()
    timed_chunks = CHUNKS[1:N_RUN]
    @benchmarkable run_process($rs, $acq_plan, $fast, $timed_chunks, 10u"s")
end

# ── Register benchmarks ───────────────────────────────────────────────────
SUITE["process without acquisition ($RUN_LABEL)"]["1-ant"] = bench_process_without_acquisition()
SUITE["process with acquisition every 10 sec ($RUN_LABEL)"]["1-ant"] = bench_process_with_acquisition()
