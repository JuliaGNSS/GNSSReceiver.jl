using BenchmarkTools
using GNSSReceiver
using GNSSReceiver: ReceiverState, NumAnts, SampleBuffer, process
using GNSSReceiver.SampleBuffers: buffer
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
# A 60 s ION RTL-SDR GPS L1 recording (2.048 MS/s, 8-bit unsigned offset-binary
# I/Q). Same recording the integration test uses. We only need the first few
# seconds, so we fetch a byte range rather than the whole 246 MB file.
const SIGNAL_URL = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
const SAMPLING_FREQ = 2.048e6u"Hz"
const SYSTEM = GPSL1()
# One process() call handles a 4 ms chunk; the label surfaces that so the
# measured time can be compared against real time (real-time capable iff
# time-per-call < chunk duration).
const CHUNK = Int(upreferred(SAMPLING_FREQ * 4u"ms"))            # 8192 samples
const CHUNK_LABEL = "$(uconvert(u"ms", CHUNK / SAMPLING_FREQ)) signal"
# Coherent-integration length for acquisition (10 ms locks the full healthy set).
const ACQ_CODE_CYCLES = 10
# Seconds of signal to load: enough to fill the acq buffer, let acquisition fire,
# and let the sats reach lock for the steady-state benchmark.
const SETUP_SECONDS = 3.0

# Read the first `n` complex samples of the recording as centered ComplexF32.
function load_ion_signal(n)
    cache = joinpath(tempdir(), "gnssreceiver_bench_RTLSDR_Bands-L1.uint8")
    if !isfile(cache) || filesize(cache) < 2n
        # Fetch just the bytes we need (curl is available on CI runners).
        run(`curl -sfL -r 0-$(2n - 1) -o $cache $SIGNAL_URL`)
    end
    raw = Vector{UInt8}(undef, 2n)
    open(cache) do io
        read!(io, raw)
    end
    return ComplexF32[
        ComplexF32(Float32(raw[2i-1]) - 127.5f0, Float32(raw[2i]) - 127.5f0) for i = 1:n
    ]
end

# Split a flat signal vector into consecutive 4 ms chunks (column vectors, 1 antenna).
chunks_of(sig) = [sig[(k-1)*CHUNK+1:k*CHUNK] for k = 1:(length(sig)÷CHUNK)]

# Build a receiver state + acquisition plans mirroring `receive`.
function make_receiver_and_plans()
    nacq = round(
        Int,
        get_code_length(SYSTEM) * upreferred(SAMPLING_FREQ / get_code_frequency(SYSTEM)) *
        ACQ_CODE_CYCLES,
    )
    receiver_state = ReceiverState(
        ComplexF32,
        SYSTEM;
        num_ants = NumAnts(1),
        num_samples_for_acquisition = nacq,
    )
    acq_plan = AcquisitionPlan(SYSTEM, nacq, float(SAMPLING_FREQ); prns = 1:32)
    coarse_step = 2 * SAMPLING_FREQ / nacq
    fine_step = 1 / 4 / (nacq / SAMPLING_FREQ)
    fast_re_acq_plan =
        AcquisitionPlan(SYSTEM, nacq, SAMPLING_FREQ; dopplers = -coarse_step:fine_step:coarse_step, prns = 1:32)
    return receiver_state, acq_plan, fast_re_acq_plan, nacq
end

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

# ── Benchmark: process WITH acquisition (real signal) ─────────────────────
# The acquisition buffer is pre-filled with real signal, and the state is set so
# acquisition fires on the benchmarked call.
function bench_process_with_acquisition()
    _, acq_plan, fast, nacq = make_receiver_and_plans()
    sig = load_ion_signal(nacq + CHUNK)
    receiver_state = ReceiverState(
        ComplexF32,
        SYSTEM;
        num_ants = NumAnts(1),
        num_samples_for_acquisition = nacq,
        acquisition_buffer = buffer(SampleBuffer(ComplexF32, nacq), sig[1:nacq]),
    )
    measurement = sig[nacq+1:nacq+CHUNK]
    # acquire_every default (10 s) with last_time_acquisition_ran = -Inf → fires now.
    @benchmarkable _process($receiver_state, $acq_plan, $fast, $measurement)
end

# ── Benchmark: steady-state tracking on real signal (no re-acquisition) ───
# Acquire once and let tracking lock, then benchmark a single tracking step with
# acquire_every set so large that acquisition never fires again.
function bench_process_steady_state()
    receiver_state, acq_plan, fast, nacq = make_receiver_and_plans()
    n = ceil(Int, upreferred(SAMPLING_FREQ * (SETUP_SECONDS * u"s")))
    chunks = chunks_of(load_ion_signal(n + CHUNK))
    # Setup (not benchmarked): run through the real signal so sats acquire and lock.
    for chunk in chunks
        receiver_state = _process(receiver_state, acq_plan, fast, chunk)
    end
    locked = receiver_state
    measurement = chunks[end]
    # acquire_every huge → acquisition (and reacquisition buffering) never fires;
    # this measures pure tracking + the periodic PVT solve on the locked sats.
    @benchmarkable _process($locked, $acq_plan, $fast, $measurement; acquire_every = 1_000_000u"s")
end

# ── Register benchmarks ───────────────────────────────────────────────────
SUITE["process with acquisition ($CHUNK_LABEL)"]["1-ant"] = bench_process_with_acquisition()
SUITE["process steady-state ($CHUNK_LABEL)"]["1-ant"] = bench_process_steady_state()
