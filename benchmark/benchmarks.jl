using BenchmarkTools
using GNSSReceiver
using GNSSReceiver: ReceiverState, NumAnts, process, is_in_lock
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
# 8-bit unsigned offset-binary I/Q).
#
# A receiver run passes through three cost regimes, and each is benchmarked
# separately over just **1 second** of signal (`N_1S` chunks):
#
#   1. acquisition        — buffer fills, `acquire!` searches the PRNs, first locks.
#   2. tracking pre-decode — sats are locked and their nav bits are being decoded,
#                            but not enough ephemeris yet for a fix (no PVT).
#   3. tracking + PVT      — steady state: tracking, decoding, and `calc_pvt` every
#                            `pvt_update_interval`.
#
# Why 1 s per stage: BenchmarkTools takes samples until its time budget (default
# 5 s) is spent, but always at least one. A single 45 s pass costs more than the
# whole budget, so exactly ONE sample was collected — and the CI's "report the
# minimum" strategy then has nothing to pick a minimum from, so it just echoes one
# noisy run. A ~1 s timed sample fits many times into the budget, so the reported
# minimum is a real least-disturbed sample. The state that starts each stage is
# built once in an untimed setup pre-roll (`capture_stage_snapshots`), not timed.
#
# Each stage is run in a float and an `Int16` variant (like the earlier process
# benchmarks): the float-vs-Int16 gap within a build is the integer-backend
# speedup, present on both revisions, so it is not a base/head difference.
const SIGNAL_URL = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
const SAMPLING_FREQ = 2.048e6u"Hz"
# GNSSSignals v3 renamed GPSL1 -> GPSL1CA. Feature-detect so this one script runs
# against both the v3 head and a pre-v3 base (benchpkg --bench-on=head).
const SYSTEM =
    isdefined(GNSSSignals, :GPSL1CA) ? GNSSSignals.GPSL1CA() : GNSSSignals.GPSL1()
const CHUNK = Int(upreferred(SAMPLING_FREQ * 4u"ms"))        # 8192 samples per process() call
const ACQ_CODE_CYCLES = 10        # 10 ms coherent integration locks the full healthy set

# Each stage is timed over N_1S chunks (~1 s of signal).
const N_1S = floor(Int, upreferred(SAMPLING_FREQ * 1u"s") / CHUNK)   # 250 chunks
# Signal to load for the untimed setup pre-roll: enough to reach a PVT fix (~35 s
# on this recording) plus the 1 s stage-3 window, with margin.
const MAX_SECONDS = 45
const N_CHUNKS = floor(Int, upreferred(SAMPLING_FREQ * (MAX_SECONDS * u"s")) / CHUNK)

const NEVER = 1_000_000u"s"       # acquire_every large enough that (re)acquisition never fires
# Realistic periodic-acquisition cadence, used both for the setup pre-roll and for
# the acquisition-stage timed run (so the acquisition-stage second contains a real
# `acquire!` event, as at cold start).
const ACQUIRE_EVERY = 10u"s"

const STAGE_LABEL = "$(uconvert(u"s", N_1S * CHUNK / SAMPLING_FREQ)) signal"

# Coherent integration time (ACQ_CODE_CYCLES code periods) — the main driver of
# `acquire!` cost, so it goes in the acquisition benchmark's label. Derived from
# the system's code length / chip rate rather than hard-coded.
const COHERENT_INTEGRATION =
    uconvert(u"ms", ACQ_CODE_CYCLES * get_code_length(SYSTEM) / get_code_frequency(SYSTEM))

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

const CHUNKS = load_chunks(N_CHUNKS)

# `Complex{Int16}` copies of the same chunks for the integer-backend variants. The
# float `CHUNKS` are the 8-bit source recentred on 127.5; round to `Int16` (values
# within ±128, matching `DC_INT16`'s `max_meas = 2^7`).
const CHUNKS_INT16 =
    [complex.(round.(Int16, real.(c)), round.(Int16, imag.(c))) for c in CHUNKS]

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

# ── Stage snapshots (untimed setup) ─────────────────────────────────────────
# Drive one fresh receiver forward through the recording and deepcopy-snapshot the
# receiver state at the start of each pipeline stage, together with the N_1S chunks
# that follow that snapshot. This whole pre-roll is setup — it is not part of any
# timed sample. Run once per sample element type (float / Int16) since each uses a
# different receiver-state and correlator type.
has_fix(rs) = !isnothing(rs.pvt.time)
num_in_lock(rs) = count(is_in_lock, rs.receiver_sat_states[1])

function capture_stage_snapshots(::Type{T}, dc, chunks) where {T}
    rs, plan_args = make_receiver_and_plan(T)
    acq_snapshot = deepcopy(rs)          # fresh receiver = the start of acquisition
    track_snapshot = nothing
    track_start = 0
    pvt_snapshot = nothing
    pvt_start = 0
    for (i, c) in enumerate(chunks)
        rs = _process(rs, plan_args, c, dc; acquire_every = ACQUIRE_EVERY)
        # Stage 2 starts once a PVT-capable number of sats (4) are locked and being
        # decoded — but no fix yet (the full ephemeris takes ~30 s to decode).
        if isnothing(track_snapshot) && num_in_lock(rs) >= 4 && !has_fix(rs)
            track_snapshot = deepcopy(rs)
            track_start = i
        end
        # Stage 3 starts at the first PVT fix.
        if isnothing(pvt_snapshot) && has_fix(rs)
            pvt_snapshot = deepcopy(rs)
            pvt_start = i
            break
        end
    end
    isnothing(track_snapshot) &&
        error("setup ($T): never reached the tracking stage (>= 4 sats in lock) in $MAX_SECONDS s")
    isnothing(pvt_snapshot) &&
        error("setup ($T): never reached a PVT fix in $MAX_SECONDS s of signal")
    pvt_start + N_1S <= length(chunks) ||
        error("need $(pvt_start + N_1S) chunks for the stage-3 window; only $(length(chunks)) loaded")
    return (;
        plan_args,
        acq_snapshot,
        acq_chunks = chunks[1:N_1S],
        track_snapshot,
        track_chunks = chunks[track_start+1:track_start+N_1S],
        pvt_snapshot,
        pvt_chunks = chunks[pvt_start+1:pvt_start+N_1S],
    )
end

const STAGES_FLOAT = capture_stage_snapshots(ComplexF32, DC, CHUNKS)
const STAGES_INT16 = capture_stage_snapshots(Complex{Int16}, DC_INT16, CHUNKS_INT16)

# Every stage benchmark hands each BenchmarkTools sample a fresh `deepcopy` of its
# snapshot and pins `evals = 1`. `process` mutates the receiver state in place (the
# in-place `track!` and the `map!` in `update_receiver_sat_states`), so re-running
# the same object would start each sample from the previous one's already-advanced
# state — sats fall out of lock and later samples measure almost nothing. A fresh
# deepcopy per sample makes every measured sample one honest 1 s forward pass.

# ── Stage 1: acquisition ────────────────────────────────────────────────────
# Fresh receiver, `acquire_every = ACQUIRE_EVERY`: the buffer fills within ~12 ms,
# `acquire!` searches all 32 PRNs, and the first sats lock — the cold-start second.
function bench_acquisition_stage(stages, dc)
    (; plan_args, acq_snapshot, acq_chunks) = stages
    @benchmarkable(
        run_process(state, $plan_args, $acq_chunks, $dc, $ACQUIRE_EVERY),
        setup = (state = deepcopy($acq_snapshot)),
        evals = 1,
    )
end

# ── Stage 2: tracking, pre-decode ───────────────────────────────────────────
# Sats already locked; `acquire_every = NEVER` so no acquisition fires — pure
# tracking + nav-bit decoding, no PVT yet.
function bench_tracking_stage(stages, dc)
    (; plan_args, track_snapshot, track_chunks) = stages
    @benchmarkable(
        run_process(state, $plan_args, $track_chunks, $dc, $NEVER),
        setup = (state = deepcopy($track_snapshot)),
        evals = 1,
    )
end

# ── Stage 3: tracking + PVT ─────────────────────────────────────────────────
# Post-fix steady state; `acquire_every = NEVER`. Tracking + decoding + `calc_pvt`
# every `pvt_update_interval` (100 ms) → ~10 PVT solves in the 1 s window.
function bench_pvt_stage(stages, dc)
    (; plan_args, pvt_snapshot, pvt_chunks) = stages
    @benchmarkable(
        run_process(state, $plan_args, $pvt_chunks, $dc, $NEVER),
        setup = (state = deepcopy($pvt_snapshot)),
        evals = 1,
    )
end

# ── Stage 3 through the public `receive` pipeline ───────────────────────────
# `receive` consumes (CHUNK × 1) matrix chunks off a `MatrixSizedChannel`, spawns
# its own tracking task, and builds the per-chunk `sat_data` / `ReceiverDataOfInterest`
# — plumbing the direct-`process` benchmarks don't exercise. Feed it the post-fix
# `Int16` snapshot + the same 1 s of chunks (element type `Complex{Int16}`, so
# `receive` auto-selects the integer backend) to measure steady-state end-to-end
# cost (channel + sat_data build + PVT) rather than a 45 s cold start.
const RECEIVE_STEADY_CHUNKS = [reshape(c, CHUNK, 1) for c in STAGES_INT16.pvt_chunks]

function run_receive_steady(chunks, receiver_state)
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
        acquire_every = NEVER,
        receiver_state,
        _ACQ_CYCLES_KW...,
        _MAX_MEAS_KW...,
    )
    GNSSReceiver.consume_channel(_ -> nothing, data_channel)
    return nothing
end

function bench_receive_steady()
    pvt_snapshot = STAGES_INT16.pvt_snapshot
    @benchmarkable(
        run_receive_steady($RECEIVE_STEADY_CHUNKS, state),
        setup = (state = deepcopy($pvt_snapshot)),
        evals = 1,
    )
end

# ── Register benchmarks ───────────────────────────────────────────────────
# Float and Int16 variants of each process-stage benchmark so float-vs-Int16 is
# compared like-for-like within a single build.
SUITE["acquisition ($STAGE_LABEL, $COHERENT_INTEGRATION coherent integration)"]["1-ant float"] =
    bench_acquisition_stage(STAGES_FLOAT, DC)
SUITE["acquisition ($STAGE_LABEL, $COHERENT_INTEGRATION coherent integration)"]["1-ant Int16"] =
    bench_acquisition_stage(STAGES_INT16, DC_INT16)
SUITE["tracking pre-decode ($STAGE_LABEL)"]["1-ant float"] = bench_tracking_stage(STAGES_FLOAT, DC)
SUITE["tracking pre-decode ($STAGE_LABEL)"]["1-ant Int16"] = bench_tracking_stage(STAGES_INT16, DC_INT16)
SUITE["tracking + PVT ($STAGE_LABEL)"]["1-ant float"] = bench_pvt_stage(STAGES_FLOAT, DC)
SUITE["tracking + PVT ($STAGE_LABEL)"]["1-ant Int16"] = bench_pvt_stage(STAGES_INT16, DC_INT16)
SUITE["receive steady-state ($STAGE_LABEL)"]["1-ant Int16"] = bench_receive_steady()
