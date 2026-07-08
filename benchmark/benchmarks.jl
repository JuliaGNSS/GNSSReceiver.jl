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
# 8-bit unsigned offset-binary I/Q). We warm up over the first ~45 s — long enough
# to acquire, lock, decode the ephemeris and get a real position fix — then
# benchmark a single 4 ms `process` step on that realistic navigating state.
const SIGNAL_URL = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
const SAMPLING_FREQ = 2.048e6u"Hz"
const SYSTEM = GPSL1()
const CHUNK = Int(upreferred(SAMPLING_FREQ * 4u"ms"))        # 8192 samples per process() call
# Label surfaces the chunk's real-time duration so time-per-call can be judged
# against real time (real-time capable iff time-per-call < this).
const CHUNK_LABEL = "$(uconvert(u"ms", CHUNK / SAMPLING_FREQ)) signal"
const ACQ_CODE_CYCLES = 10        # 10 ms coherent integration locks the full healthy set
const SETUP_SECONDS = 45          # long enough to decode ephemeris and get a fix (~35 s)
const NEVER = 1_000_000u"s"       # acquire_every large enough that (re)acquisition never fires

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

# Ensure the first `nsamples` complex samples of the recording are cached (byte-range
# download so we don't fetch the whole 246 MB file).
function ensure_signal(nsamples)
    cache = joinpath(tempdir(), "gnssreceiver_bench_RTLSDR_Bands-L1.uint8")
    if !isfile(cache) || filesize(cache) < 2 * nsamples
        run(`curl -sfL -r 0-$(2 * nsamples - 1) -o $cache $SIGNAL_URL`)
    end
    return cache
end

# Warm a receiver over SETUP_SECONDS of real signal: acquire once (acquire_every = NEVER),
# keeping the acquisition buffer fresh (always_buffer) so the acquisition benchmark can
# re-fire. By SETUP_SECONDS the ephemeris is decoded and a PVT fix obtained, so this is a
# true steady state (locked + navigating), not merely "locked".
# Returns (receiver_state, acq_plan, fast_re_acq_plan, last_chunk).
function warm_up()
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

    nchunks = ceil(Int, upreferred(SAMPLING_FREQ * (SETUP_SECONDS * u"s")) / CHUNK)
    cache = ensure_signal((nchunks + 1) * CHUNK)
    raw = Vector{UInt8}(undef, 2 * CHUNK)
    last_chunk = ComplexF32[]
    open(cache) do io
        for _ = 1:nchunks
            readbytes!(io, raw, 2 * CHUNK) == 2 * CHUNK || break
            chunk = ComplexF32[
                ComplexF32(Float32(raw[2i-1]) - 127.5f0, Float32(raw[2i]) - 127.5f0) for i = 1:CHUNK
            ]
            rs = _process(rs, acq_plan, fast, chunk; acquire_every = NEVER, always_buffer = true)
            last_chunk = chunk
        end
    end
    return rs, acq_plan, fast, last_chunk
end

const WARM_RS, WARM_ACQ_PLAN, WARM_FAST, WARM_CHUNK = warm_up()

# ── Register benchmarks (same warmed, navigating receiver for both) ────────
# Steady-state: track the locked+navigating receiver, no (re)acquisition.
SUITE["process steady-state ($CHUNK_LABEL)"]["1-ant"] =
    @benchmarkable _process($WARM_RS, $WARM_ACQ_PLAN, $WARM_FAST, $WARM_CHUNK; acquire_every = NEVER)
# With acquisition: identical state, but acquisition fires on this step (re-acquisition
# over all PRNs while the locked sats keep tracking).
SUITE["process with acquisition ($CHUNK_LABEL)"]["1-ant"] =
    @benchmarkable _process($WARM_RS, $WARM_ACQ_PLAN, $WARM_FAST, $WARM_CHUNK; acquire_every = 0u"s")
