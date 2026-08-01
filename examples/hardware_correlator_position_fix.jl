# Live GPS L1 C/A position fix through the LiteX-M2SDR hardware correlator.
#
# The full GNSSReceiver pipeline — acquisition, decoding, PVT — runs on the CPU
# off the raw DMA0 stream; correlation and the tracking NCOs run on the FPGA,
# linked through GNSSReceiver's hardware-correlator interface (issue #107).
#
# Structure:
#   1. JIT warm-up of the receive() path on synthetic samples.
#   2. Raw stream: m2sdr_record → FIFO → SignalChannel (antenna 0 of the 2R2T
#      words), with a producer-side sample count and a tap ring for the
#      calibration acquisitions.
#   3. Origin calibration: the bank's free-running sample counter is latched
#      before streaming starts (it only counts samples DMA0 accepts, so that
#      latch is exact up to dropped buffers); a code-phase sweep against one
#      acquired satellite then verifies/corrects it modulo the code period.
#   4. receive(sdr, …) with DMA-streamed dumps (dump_source = :dma,
#      epoch_period = 50 → ~2.2 ms feedback transport) until the PVT solve
#      returns a fix; it then rides RUN_AFTER_FIX seconds and stops.
#
# Usage:
#   julia -t 6,2 --project=. hardware_correlator_position_fix.jl [MAX_SECONDS] [RUN_AFTER_FIX]
# (defaults 600 and 30; HWFIX_ACQ_EVERY / HWFIX_ASYNC set the rescan cadence and
#  whether the scan runs off the processing task. One default-pool thread is for
#  the acquisition worker, so give it at least three.)
#
# Bit-stream instrument (issue #107). Set HWFIX_BITLOG=<name> to log, next to
# this script, every soft bit and every 1 ms filtered prompt on the way into
# `decode`, plus the E/P/L accumulators, both Dopplers and the per-channel
# record-continuity counters:
#
#   HWFIX_BITLOG=bitstream.log julia -t 6,2 --project=. \
#       hardware_correlator_position_fix.jl 220 20
#   julia analysis/subframes_softbits.jl bitstream.log 7      # real subframes?
#   julia analysis/word_errors.jl        bitstream.log 7 993  # where the errors are
#   julia analysis/walkoff.jl            bitstream.log 10     # E/L imbalance over time
#
# Unset (the default) it costs nothing: the wrapper's per-chunk hook returns
# immediately. Logging goes through the `correlator_source` seam rather than
# `receive`'s payload channel because that channel emits once per
# `pvt_update_interval` while the bit buffer is reset every chunk — sampling it
# sees ~1/50 of the bits. `advance_tracking!` runs after the fold and before
# `decode` consumes the buffer, so what it sees IS what the decoder gets.
#
# FPGA-vs-CPU correlator probe (issue #107). Set HWFIX_XCORR=<name> to correlate
# the *same raw samples* on the CPU, with the *same NCO parameters*, once per
# hardware record, and log both prompts side by side:
#
#   HWFIX_XCORR=xcorr.log HWFIX_XCORR_PRNS=7,20 julia -t 6,2 --project=. \
#       hardware_correlator_position_fix.jl 120 10
#   julia analysis/fpga_vs_cpu.jl xcorr.log
#
# This is the direct measurement of what the hardware correlator loses: the
# 200 s bit-error study (issue #107) put a ~3 % BER on the FPGA bit stream while
# the pure software receiver validated full ephemerides off the same DMA0 stream
# in the same minute, which leaves only the correlator itself. Both streams see
# one input, so every shared error source — antenna, RF, DMA0, acquisition —
# cancels, and the residual is the device's.
#
# Companion branches (this exact stack produced the first live fix 2026-08-01,
# lat 50.768612° lon 6.072698° alt 271.2 m):
#   GNSSM2SDR.jl feat/code-phase-anchor — immediate-CSR NCO updates, record
#     dedup, O_NONBLOCK DMA reader; Tracking.jl hwfix/hardware-correlator-run —
#     1 ms integrations, degenerate-record skips.
#
# Board bring-up (Jetson + LiteX M2SDR; redo after EVERY reboot):
#   sudo insmod ~/litex_m2sdr/litex_m2sdr/software/kernel/m2sdr.ko
#     (that build has DMA_BUFFER_COUNT=256 — the gateware descriptor table is
#      256 deep; larger rings wrap it into stale-buffer corruption. Never
#      modprobe: the installed module is an older single-device build.)
#   sudo sysctl -w fs.pipe-max-size=67108864   # not persistent
#   ~/litex_m2sdr/litex_m2sdr/software/user/m2sdr_rf \
#     --sample-rate 4000000 --bandwidth 2500000 --rx-freq 1575420000 \
#     --rx-gain 60 --tx-att 89
#     (long flags only; a bare m2sdr_rf resets to 30.72M/2.4GHz defaults)
#   Verify: m2sdr_record streams ~32 MB/s with ~0 adjacent-duplicate samples.
#   MAXN power mode + jetson_clocks: the scan itself now overlaps tracking
#   (asynchronous acquisition), but it still competes for cores with the fold
#   and the CSR/DMA reader, so the clocks matter.

using Printf
using Statistics: mean, median, std
using StaticArrays: SVector
using Unitful
using Unitful: Hz, ms, ustrip
using Geodesy: LLAfromECEF, wgs84

using Acquisition: acquire
using FFTW
using GNSSSignals: GPSL1CA, get_code, get_code_frequency
import Tracking
using Tracking: ConventionalAssistedPLLAndDLL
using SignalChannels: SignalChannel
using GNSSReceiver
using GNSSM2SDR
using GNSSM2SDR: schedule!, load_code!, spacing_word, carrier_word, code_word,
    sample_count, apply_status, GPS_L1_HZ, GPS_CA_CHIP_RATE, CA_CODE_LENGTH

const FS_HZ = 4e6
const FS = 4e6Hz
# 2 ms chunks: per-chunk pipeline bookkeeping (ReceiverState rebuild, extract,
# decode consume, PVT) is a fixed cost per chunk; at 4000-sample chunks the
# receiver runs ~0.55x real time with 5 sats on this box, and rt<1x means
# either sample loss (small pipe, old runs) or unbounded lag (big pipe). The
# fold grid stays at 1 ms epochs inside advance_tracking!; only feedback
# batching coarsens (~+2 ms tau, fine at 8 Hz PLL).
const CHUNK = 8000                       # 2 ms
const CSR_CSV = expanduser("~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch20_ant1/csr.csv")
const FIFO = "/tmp/hwfix_raw.fifo"
const FRAC = 24
const SWEEP_MARGIN = 24_000              # samples ahead for sweep commits (6 ms)
const ACQ_COVERAGE = 25_000.0Hz          # one-sided; LO offset ~14 kHz + Doppler
const CN0_FLOOR_DBHZ = 38.0              # calibration satellite quality gate
const MAX_SECONDS = length(ARGS) >= 1 ? parse(Float64, ARGS[1]) : 600.0
const RUN_AFTER_FIX = length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 30.0
# Periodic rescan cadence. Acquisition now runs off the processing task
# (`acquire_async`), so a scan no longer stalls the chunk pipeline and a real
# rescan cadence is affordable — the `t=` vs `rt=` gap in the progress line is
# the direct measurement of that. `HWFIX_ASYNC=0` puts the search back inline
# for comparison, which is what forced the earlier runs to disable the rescan
# altogether (`acquire_every = 300 s`).
const ACQ_EVERY = parse(Float64, get(ENV, "HWFIX_ACQ_EVERY", "30")) * u"s"
const ACQ_ASYNC = get(ENV, "HWFIX_ASYNC", "1") == "1"

const gpsl1 = GPSL1CA()
# Bit-stream instrument (issue #107): unset means no logging, no cost.
const BIT_LOG_NAME = get(ENV, "HWFIX_BITLOG", "")
# FPGA-vs-CPU correlator probe (issue #107): likewise off unless named.
const XCORR_LOG_NAME = get(ENV, "HWFIX_XCORR", "")
# Restrict the probe to these PRNs (empty ⇒ every assigned satellite). One CPU
# correlation per hardware record costs ~50 µs, so five satellites at 1 kHz add
# ~0.25 s of CPU per second of signal to the *processing task* — watch the
# `t=` vs `rt=` gap and narrow this (or raise HWFIX_XCORR_EVERY) if it grows.
const XCORR_PRNS =
    Set(parse.(Int, filter(!isempty, split(get(ENV, "HWFIX_XCORR_PRNS", ""), ','))))
const XCORR_EVERY = parse(Int, get(ENV, "HWFIX_XCORR_EVERY", "1"))
const RAW_CHUNKS = Threads.Atomic{Int}(0)

# Cap FFTW's own pthread pool: acquisition storms (initial + per-satellite
# reacquisition backoff) otherwise oversubscribe the CPU and the OS preempts
# the interactive-pool CSR poller — every preemption is a burst of missed
# dumps, and a missed dump inside a subframe breaks its parity. Keep the total
# OS-thread demand at least two cores short of the machine.
FFTW.set_num_threads(4)

# ── 1. JIT warm-up ────────────────────────────────────────────────────────────
# The warm-up must compile the SAME specializations the live call uses: the
# HARDWARE advance_tracking!/fold path (via an AbstractHardwareCorrelatorSDR
# source with real dumps), the ConventionalAssistedPLLAndDLL track state, the
# PRN-restricted acquisition and the extract payload. A software-source warm-up
# with different kwargs leaves 25-110 s of JIT to happen live, during which the
# chunk backlog grows and every tracking loop is born open.
using StaticArrays: SVector
using PipeChannels: PipeChannel
using Tracking: CorrelatorOutput, EarlyPromptLateCorrelator
using GNSSReceiver: CorrelatorDump, NCOUpdate, AbstractHardwareCorrelatorSDR, epoch_strobe

# ── FPGA ↔ CPU correlator probe ───────────────────────────────────────────────
# Correlate the same raw samples the FPGA correlated, over the same span, with
# the same NCO parameters, and log both prompts per record.
#
# The alignment is the whole trick. A hardware record covers device samples
# `[sample_index - integrated_samples, sample_index)` and reports the code NCO's
# own phase at its end; the raw stream is the same samples, offset by the
# constant `sdr.device_origin` (the run's origin calibration is what pins it).
# So the probe rings the raw chunks by device index and, for each record, slices
# exactly that span back out. Nothing is resampled or re-timed: if the two
# prompts disagree, the disagreement is in the correlator.
#
# Three things are deliberately NOT taken from the host's model of the device:
#
#   * The code phase comes off the wire (`CorrelatorDump.code_phase`), so the
#     CPU replica sits where the device's replica actually sat, not where the
#     host thinks it steered it.
#   * A slow CPU-side DLL (`delta`, chips) rides on top of it. An exact device
#     axis leaves it at zero; a residual is the host↔device sample-mapping error
#     in chips, measured rather than assumed — and it keeps the CPU reference on
#     the peak while that is measured, so the comparison stays fair.
#   * The mixing sign is voted on at the first record rather than inherited from
#     Tracking's convention: getting it wrong costs the CPU side all of its
#     correlation gain, and nothing else in the output would say so.
#
# The carrier *frequency* is shared on purpose — both sides mix with the Doppler
# the loop commanded. A carrier NCO that does not honour that word then shows up
# offline as a phase ramp on the FPGA prompt against a stationary CPU one.
#
# The kernels live in `analysis/cpu_correlator.jl` so they can be pinned against
# an analytically known injected signal without a board; see
# `analysis/selftest_cpu_correlator.jl`.
include(joinpath(@__DIR__, "analysis", "cpu_correlator.jl"))
using .CpuCorrelator: SampleRing, Workspace, push_chunk!, covers, correlate_epl,
    search_offset

# The coarse search slides the code replica ±this many whole samples (±16 chips
# at 4 MHz), which covers any plausible residual of the origin calibration.
const XCORR_SEARCH_SAMPLES = 64

# A record waiting for its samples, with everything the comparison needs frozen
# at the moment it was drained.
#
# Records reach the host *ahead* of the raw samples they were computed from: a
# dump crosses DMA1 in a couple of milliseconds while the same samples are still
# queued in the recorder pipe and the receiver's own sample channel. Correlating
# only what the ring already holds would therefore throw away nearly every
# record — so a record is parked here and picked up once the raw stream has
# caught up with it, typically one or two chunks later.
#
# Freezing the Dopplers at drain time (rather than reading them when the samples
# arrive) also keeps the CPU replica on the loop state that was actually driving
# the device around that record.
struct PendingRecord
    channel::Int32
    prn::Int32
    sample_index::Int64
    n::Int32
    code_phase::Float64
    late::ComplexF64
    prompt::ComplexF64
    early::ComplexF64
    carrier_hz::Float64
    code_doppler_hz::Float64
    code_freq_hz::Float64
    shift::Int32
end

mutable struct CpuReference
    const io::IO
    # Device sample index of host raw-sample 0, i.e. `sdr.device_origin` at the
    # moment `receive` took the stream over. Constant for the run.
    const origin::Int64
    const fs::Float64
    const ring::SampleRing
    const ws::Workspace
    const codes::Dict{Int,Vector{Float32}}
    const prns::Set{Int}
    const every::Int
    # Records drained but not yet reached by the raw stream.
    const pending::Vector{PendingRecord}
    const max_pending::Int
    # Per hardware channel.
    const last_index::Vector{Int64}      # newest record seen, for dedup
    const delta::Vector{Float64}         # CPU code-phase offset, chips
    const anchor::Vector{Float64}        # where the coarse search put `delta`
    const occupant::Vector{Int32}        # PRN the probe last coarse-aligned to
    # The CPU's own carrier NCO, mirroring the device's: phase in cycles, the
    # device sample it refers to, and the frequency running since then.
    const carrier_cycles::Vector{Float64}
    const carrier_ref::Vector{Int64}
    const last_carrier_hz::Vector{Float64}
    const seen::Vector{Int}
    sign::Int                            # carrier mixing sign, 0 until voted
    records::Int
    skipped_old::Int                     # the ring rolled over the samples first
    dropped_pending::Int                 # the queue overflowed
    unaligned::Int                       # coarse search found no peak
    max_wait::Int64                      # deepest record→sample lag seen, samples
    cpu_seconds::Float64
end

function CpuReference(io::IO, origin::Integer, fs::Real, n_channels::Integer;
                      ring_seconds = 0.5, prns = Set{Int}(), every::Integer = 1,
                      max_pending::Integer = 8192)
    p = CpuReference(
        io, Int64(origin), Float64(fs),
        SampleRing(round(Int, ring_seconds * fs)),
        Workspace(2 * CHUNK, XCORR_SEARCH_SAMPLES),
        Dict{Int,Vector{Float32}}(), Set{Int}(prns), Int(every),
        PendingRecord[], Int(max_pending),
        fill(typemin(Int64), n_channels), zeros(n_channels), zeros(n_channels),
        zeros(Int32, n_channels), zeros(n_channels),
        fill(typemin(Int64), n_channels), zeros(n_channels), zeros(Int, n_channels),
        0, 0, 0, 0, 0, Int64(0), 0.0,
    )
    # Compile the kernels here rather than during the first live chunk: the
    # probe runs inside the processing task, and a JIT pause there is a chunk
    # backlog and a burst of stale NCO commits. They are separate methods, so
    # they have to be *run*, not merely referenced — over the still-zeroed ring,
    # which correlates to nothing and costs a millisecond. Nothing durable is
    # touched: the sign vote is `search_offset`'s return value, not its state.
    let n = 400, code = xcorr_code_table!(p, 1)
        correlate_epl(p.ws, p.ring, Int64(0), n; code, cp_start = 0.0,
                      cps = 0.2558, freq = 1000.0, fs = p.fs, sign = 1, shift = 1)
        search_offset(p.ws, p.ring, Int64(0), n; code, cp_start = 0.0,
                      cps = 0.2558, freq = 1000.0, fs = p.fs,
                      margin = XCORR_SEARCH_SAMPLES)
    end
    println(io, "# X prn ch sample_index n code_phase delta_chips ",
                "re_Pf im_Pf abs_Lf abs_Ef re_Pc im_Pc abs_Lc abs_Ec ",
                "carrier_doppler code_doppler")
    println(io, "# A prn ch sample_index offset_samples offset_chips peak_over_median sign")
    p
end

xcorr_code_table!(p::CpuReference, prn::Integer) = get!(p.codes, Int(prn)) do
    Float32[get_code(gpsl1, i, prn) for i = 0:(CA_CODE_LENGTH-1)]
end

# Until the first usable vote, mix as Tracking does; every `A` line carries the
# vote's evidence, so a run that never voted says so.
_xcorr_sign(p::CpuReference) = p.sign == 0 ? 1 : p.sign

xcorr_append!(p::CpuReference, samples, host_start::Integer) =
    push_chunk!(p.ring, samples, p.origin + Int64(host_start))

# Capture one drained record: freeze what the comparison will need and park it
# until the raw stream reaches it.
function xcorr_capture!(p::CpuReference, assignments, track_state, dump)
    ch = Int(dump.channel)
    (1 <= ch <= length(p.last_index)) || return p          # epoch strobe
    dump.output.sample_index <= p.last_index[ch] && return p
    p.last_index[ch] = dump.output.sample_index
    assignment = assignments[ch]
    isnothing(assignment) && return p
    assignment.prn == Int(dump.prn) || return p
    (isempty(p.prns) || assignment.prn in p.prns) || return p
    isnan(dump.code_phase) && return p
    n = Int(dump.output.integrated_samples)
    (n <= 0 || n > length(p.ws.mixed)) && return p
    p.seen[ch] += 1
    (p.every > 1 && p.seen[ch] % p.every != 0) && return p

    sat_states = Tracking.get_sat_states(track_state, assignment.group_key)
    haskey(sat_states, assignment.prn) || return p
    sat = sat_states[assignment.prn]
    tracked = Tracking.get_signals(sat)[assignment.signal_index]
    signal = Tracking.get_signal(tracked)
    code_doppler = ustrip(Hz, uconvert(Hz, Tracking.get_code_doppler(sat)))
    code_freq = ustrip(Hz, get_code_frequency(signal)) + code_doppler
    # Whole-sample Early↔Late spacing, quantised exactly the way `_assign!`
    # quantised it for the gateware, so the CPU taps sit where the device's do.
    shift = max(1, round(Int, Tracking.get_early_late_sample_spacing(
        Tracking.get_correlator(tracked), p.fs * Hz, code_freq * Hz) / 2))
    acc = Tracking.get_accumulators(dump.output.correlator)   # [late, prompt, early]

    if length(p.pending) >= p.max_pending
        # The raw stream is not catching up (or has stalled). Drop the oldest —
        # a diagnostic must never become the reason the receiver falls behind.
        popfirst!(p.pending)
        p.dropped_pending += 1
    end
    push!(p.pending, PendingRecord(
        Int32(ch), Int32(assignment.prn), Int64(dump.output.sample_index), Int32(n),
        dump.code_phase,
        _xcorr_scalar(acc[1]), _xcorr_scalar(acc[2]), _xcorr_scalar(acc[3]),
        ustrip(Hz, uconvert(Hz, Tracking.get_carrier_doppler(sat))),
        code_doppler, code_freq, Int32(shift)))
    p
end

# Correlate every parked record the raw stream has now reached, in order, and
# log both sides. Records still ahead of the stream stay parked.
function xcorr_drain_pending!(p::CpuReference)
    keep = 0
    for rec in p.pending
        s0 = rec.sample_index - rec.n
        status = covers(p.ring, s0, Int(rec.n))
        if status === :future
            keep += 1
            p.pending[keep] = rec
            p.max_wait = max(p.max_wait, rec.sample_index - p.ring.ending)
            continue
        elseif status === :old
            p.skipped_old += 1
            continue
        end
        xcorr_compare!(p, rec, s0)
    end
    resize!(p.pending, keep)
    p
end

# One record: correlate the raw span the FPGA integrated over, at the code phase
# the FPGA reported, with the Doppler the loop commanded — then log both sides.
function xcorr_compare!(p::CpuReference, rec::PendingRecord, s0::Int64)
    ch = Int(rec.channel)
    n = Int(rec.n)
    shift = Int(rec.shift)
    cps = rec.code_freq_hz / p.fs
    code = xcorr_code_table!(p, rec.prn)
    # The dump's phase is the replica's at the end of the integration.
    cp_start = rec.code_phase - n * cps + p.delta[ch]

    # Mirror the device's carrier NCO instead of restarting at zero. Its phase
    # runs continuously across records, so a reference that restarts each time
    # differs from it by `carrier_doppler x record length` — tens of cycles per
    # record at GPS Dopplers, aliasing to an effectively random offset. That
    # leaves every magnitude intact and destroys every phase comparison, which
    # is exactly the comparison this probe exists to make.
    if p.carrier_ref[ch] == typemin(Int64) || p.occupant[ch] != rec.prn
        p.carrier_cycles[ch] = 0.0
    else
        p.carrier_cycles[ch] = mod(
            p.carrier_cycles[ch] +
            p.last_carrier_hz[ch] * (s0 - p.carrier_ref[ch]) / p.fs, 1.0)
    end
    p.carrier_ref[ch] = s0
    p.last_carrier_hz[ch] = rec.carrier_hz
    phase0 = p.carrier_cycles[ch]

    # A fresh occupant is coarse-aligned again. `delta` is not reset with it:
    # most of it is the host↔device axis error, which every channel shares, so
    # the previous occupant's value is the best starting point.
    if p.occupant[ch] != rec.prn
        p.occupant[ch] = rec.prn
        offset, ratio, sign = search_offset(
            p.ws, p.ring, s0, n; code, cp_start, cps, freq = rec.carrier_hz,
            fs = p.fs, signs = p.sign == 0 ? (1, -1) : (p.sign,),
            margin = XCORR_SEARCH_SAMPLES, phase0)
        if ratio >= 4
            p.sign = sign
            p.delta[ch] += offset * cps
            cp_start += offset * cps
        else
            p.unaligned += 1
        end
        p.anchor[ch] = p.delta[ch]
        @printf(p.io, "A %d %d %d %d %.4f %.2f %+d\n",
                rec.prn, ch, rec.sample_index, offset, offset * cps, ratio, sign)
    end

    late, prompt, early = correlate_epl(
        p.ws, p.ring, s0, n; code, cp_start, cps, freq = rec.carrier_hz,
        fs = p.fs, sign = _xcorr_sign(p), shift, phase0)

    # Slow normalised early-late envelope DLL on the CPU side. Deliberately
    # sluggish, and bounded to the coarse search's anchor: it exists to hold the
    # reference on the peak and to *measure* the residual axis error, not to
    # track the satellite — the FPGA is doing that.
    denom = abs(early) + abs(late)
    if denom > 0
        disc = (abs(early) - abs(late)) / denom
        p.delta[ch] = clamp(p.delta[ch] + 0.05 * disc * shift * cps,
                            p.anchor[ch] - 2.0, p.anchor[ch] + 2.0)
    end

    @printf(p.io, "X %d %d %d %d %.4f %.4f %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.3f %.5f\n",
            rec.prn, ch, rec.sample_index, n, rec.code_phase, p.delta[ch],
            real(rec.prompt), imag(rec.prompt), abs(rec.late), abs(rec.early),
            real(prompt), imag(prompt), abs(late), abs(early),
            rec.carrier_hz, rec.code_doppler_hz)
    p.records += 1
    p
end

_xcorr_scalar(x::Complex) = x
_xcorr_scalar(x::AbstractVector) = x[1]      # multi-antenna: antenna 0 only

# Every record the link drained this chunk. `drain_buffer` is the link's own
# batch `take!` scratch, so reading it here tees the dump stream without a
# second consumer on the SPSC ring; records the drain did not refresh are
# filtered out by the per-channel `sample_index` dedup.
#
# The capture step gets the assignment table rather than the link so that its
# specialisation depends only on the record and tracking-state types — the
# warm-up's stub SDR then compiles the same method the live run calls — and the
# correlation step, which is the expensive one, is free of both.
function xcorr_process!(p::CpuReference, link, track_state)
    t0 = time()
    for dump in link.drain_buffer
        xcorr_capture!(p, link.assignments, track_state, dump)
    end
    xcorr_drain_pending!(p)
    p.cpu_seconds += time() - t0
    p
end

# ── Bit-stream instrument ─────────────────────────────────────────────────────
# `receive`'s payload channel emits once per `pvt_update_interval` (100 ms) but
# the bit buffer is reset every chunk (2 ms), so the run-script bit log sees
# ~1/50 of the soft bits — useless for checking preamble spacing or parity.
# This wraps the correlator source instead: `advance_tracking!` is the one call
# per chunk that runs after the fold and before `decode` consumes the buffer,
# so what it sees IS what the decoder gets, bit for bit.
mutable struct BitLogSource{L}
    const link::L
    const io::Union{Nothing,IO}
    # Per-PRN sink for the raw 1 ms filtered prompts, as interleaved Float32
    # (re, im). The soft bits alone cannot separate "the 20 ms accumulation is
    # aligned on the wrong edge" from "the prompts have no phase coherence to
    # accumulate": with the prompts, both bit phase and coherence can be
    # measured offline, and the bits re-derived at all 20 edge hypotheses.
    const prompts::Dict{Int,IO}
    const prompt_dir::String
    # The FPGA-vs-CPU probe, or `nothing`. It hangs off the same wrapper because
    # it needs both halves of one chunk: the raw samples on the way in, and the
    # records the fold drained on the way out.
    const xcorr::Union{Nothing,CpuReference}
    chunks::Int
end

function GNSSReceiver.advance_tracking!(
    source::BitLogSource,
    band_measurements,
    track_state,
    band_systems,
)
    probe = source.xcorr
    # Ring the chunk BEFORE the fold: `advance_tracking!` is what advances
    # `samples_consumed`, so this is the one moment its value still names the
    # first sample of this chunk.
    isnothing(probe) || xcorr_append!(
        probe,
        Tracking.get_samples(first(values(band_measurements))),
        source.link.samples_consumed,
    )
    track_state = GNSSReceiver.advance_tracking!(
        source.link,
        band_measurements,
        track_state,
        band_systems,
    )
    isnothing(probe) || xcorr_process!(probe, source.link, track_state)
    io = source.io
    isnothing(io) && return track_state
    source.chunks += 1
    link = source.link
    boundary = link.next_epoch_boundary
    for (prn, sat) in pairs(Tracking.get_sat_states(track_state))
        signal = Tracking.get_signals(sat)[1]
        soft_bits = signal.bit_buffer.soft_bits
        if !isempty(soft_bits)
            print(io, "B ", prn, ' ', boundary)
            for b in soft_bits
                print(io, ' ', b)
            end
            println(io)
        end
        prompts = Tracking.get_filtered_prompts(signal)
        if !isempty(prompts)
            sink = get!(source.prompts, prn) do
                open(joinpath(source.prompt_dir, "prompts_prn$(prn).f32"), "w")
            end
            for p in prompts
                write(sink, Float32(real(p)), Float32(imag(p)))
            end
            println(io, "P ", prn, ' ', boundary, ' ', length(prompts),
                    ' ', signal.bit_buffer.found,
                    ' ', signal.bit_buffer.prompt_accumulator_integrated_code_blocks)
            # The prompt amplitude decaying over tens of seconds is the thing
            # that stops the decoder, so log what would explain it: the early /
            # late imbalance (the DLL's own view of how far off the peak the
            # replica sits) and the two Dopplers plus the absolute code phase
            # the host is steering with.
            acc = Tracking.get_accumulators(
                Tracking.get_last_fully_integrated_correlator(signal),
            )
            println(io, "D ", prn, ' ', boundary,
                    ' ', abs(acc[1]), ' ', abs(acc[2]), ' ', abs(acc[3]),
                    ' ', Tracking.get_carrier_doppler(sat),
                    ' ', Tracking.get_code_doppler(sat),
                    ' ', Tracking.get_code_phase(sat))
        end
    end
    # ~1 s of chunks: the per-channel record-continuity counters. `lost` is in
    # device samples of records the host never saw — at 4000 samples per code
    # block that is directly the number of code blocks this satellite's
    # navigation bit clock has slipped behind the sample axis.
    if source.chunks % 500 == 0
        for ch in eachindex(link.assignments)
            assignment = link.assignments[ch]
            isnothing(assignment) && continue
            println(io, "C ", ch, ' ', assignment.prn, ' ', boundary,
                    ' ', link.lost_record_samples[ch],
                    ' ', link.overlapping_record_samples[ch],
                    ' ', link.last_record_end[ch])
        end
        println(io, "L ", boundary, ' ', link.lost_record_gaps,
                ' ', link.stale_dumps, ' ', link.dropped_dumps,
                ' ', link.skipped_epochs)
        flush(io)
    end
    track_state
end

# `receive(::AbstractHardwareCorrelatorSDR, …)` builds the link internally, so
# to wrap it we build it here and go in through the sample-channel method.
function hw_receive(
    sdr,
    io,
    prompt_dir = mktempdir();
    doppler_update_interval = nothing,
    feedback_delay_epochs::Integer = 2,
    acquire_async::Bool = true,
    xcorr = nothing,
    kwargs...,
)
    link = GNSSReceiver.HardwareCorrelatorLink(
        sdr;
        sampling_freq = FS,
        reference_signal = GNSSReceiver.ranging_signal(gpsl1),
        doppler_update_interval,
        feedback_delay_epochs,
    )
    source = BitLogSource(link, io, Dict{Int,IO}(), prompt_dir, xcorr, 0)
    BIT_LOG_SOURCE[] = source
    GNSSReceiver.receive(
        GNSSReceiver.raw_sample_channel(sdr),
        gpsl1,
        FS;
        correlator_source = source,
        acquire_async,
        kwargs...,
    )
end

const BIT_LOG_SOURCE = Ref{Any}(nothing)
const BIT_LOG_WARM = Ref{Union{Nothing,IO}}(devnull)

my_extract(state) = (
    data = GNSSReceiver.default_data_of_interest(state),
    decode = [
        begin
            sats = Tracking.get_sat_states(state.track_state, gk)
            bb = haskey(sats, rss.prn) ?
                 Tracking.get_signals(sats[rss.prn])[1].bit_buffer : nothing
            (prn = rss.prn,
             nbits = something(rss.decoder.num_bits_after_valid_syncro_sequence, -1),
             tow = Int64(something(rss.decoder.data.TOW, -1)),
             found = isnothing(bb) ? false : bb.found,
             nsoft = isnothing(bb) ? -1 : length(bb.soft_bits),
             soft = isnothing(bb) ? Float32[] : copy(bb.soft_bits))
        end
        for (gk, dict) in pairs(state.receiver_sat_states) for rss in dict
    ],
)

mutable struct WarmSDR{C} <: AbstractHardwareCorrelatorSDR
    const raw::SignalChannel{Complex{Int16},1}
    const dumps::PipeChannel{CorrelatorDump{C}}
    const ncos::PipeChannel{NCOUpdate}
end
warm_epl(l, p, e) = EarlyPromptLateCorrelator(SVector{3,ComplexF64}(l, p, e), 1)
const WARM_EPL = typeof(warm_epl(0, 0, 0))
GNSSReceiver.raw_sample_channel(s::WarmSDR) = s.raw
GNSSReceiver.correlator_dump_channel(s::WarmSDR) = s.dumps
GNSSReceiver.nco_update_channel(s::WarmSDR) = s.ncos
GNSSReceiver.num_hardware_channels(s::WarmSDR) = 5
GNSSReceiver.assign_channel!(s::WarmSDR, args...; kwargs...) = nothing
GNSSReceiver.release_channel!(s::WarmSDR, ch) = nothing

function warm_up()
    @info "warming up receive() (hardware-path JIT)…"
    t0 = time()
    sdr = WarmSDR{WARM_EPL}(
        SignalChannel{Complex{Int16},1}(CHUNK, 16),
        PipeChannel{CorrelatorDump{WARM_EPL}}(1 << 16),
        PipeChannel{NCOUpdate}(1 << 12),
    )
    task = Threads.@spawn begin
        bufs = [Complex{Int16}.(rand(-100:100, CHUNK, 1), rand(-100:100, CHUNK, 1)) for _ = 1:3]
        nepoch = CHUNK ÷ 4000
        for i = 1:200
            batch = CorrelatorDump{WARM_EPL}[]
            for e = 1:nepoch
                base = ((i - 1) * nepoch + e) * 4000
                for ch = 1:2
                    push!(batch, CorrelatorDump(ch, ch,
                        CorrelatorOutput(warm_epl(400.0 + 0im, 1000.0 + 10im, 400.0 + 0im),
                                         4000, base - 50 + ch),
                        mod(5.0 + 0.001base, 1023.0)))
                end
                push!(batch, epoch_strobe(warm_epl(0, 0, 0), base))
            end
            put!(sdr.dumps, batch)
            put!(sdr.raw, bufs[mod1(i, 3)])
            while Base.n_avail(sdr.ncos) > 0
                take!(sdr.ncos)
            end
        end
        close(sdr.raw)
        close(sdr.dumps)
    end
    data = hw_receive(
        sdr,
        BIT_LOG_WARM[];
        # The probe's own kernels are compiled by its constructor; this is for
        # the wrapper around them, which only exists when the probe does.
        xcorr = isempty(XCORR_LOG_NAME) ? nothing :
                CpuReference(devnull, 0, FS_HZ, 5; ring_seconds = 0.05),
        prns = [1, 2, 3, 4, 5, 7],
        acq_min_doppler_coverage = ACQ_COVERAGE,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 3,
        max_meas = 2^11,
        acquire_every = 50ms,
        # Same scheduler as the live call, so its merge path is compiled here
        # and not during the decode window.
        acquire_async = ACQ_ASYNC,
        feedback_delay_epochs = 1,
        doppler_estimator = ConventionalAssistedPLLAndDLL(),
        extract = my_extract,
    )
    for _ in data
    end
    wait(task)
    @info @sprintf("warm-up done in %.1f s", time() - t0)
end

# ── 2. Raw stream plumbing ────────────────────────────────────────────────────
# Tap ring: circular, no per-chunk memmove. `pos` is the producer count at the
# ring's end; the newest sample sits just behind index mod(pos-1, len)+1.
mutable struct TapRing
    const data::Vector{ComplexF32}
    pos::Int
end
TapRing(len) = TapRing(zeros(ComplexF32, len), 0)

function tap_append!(ring::TapRing, chunk::AbstractMatrix{Complex{Int16}})
    n = length(ring.data)
    base = mod(ring.pos, n)
    @inbounds for k = 1:size(chunk, 1)
        ring.data[mod(base + k - 1, n)+1] = ComplexF32(chunk[k, 1])
    end
    ring.pos += size(chunk, 1)
    ring
end

# Freshest `m` samples in time order plus the producer count at their end.
function tap_snapshot(ring::TapRing, lock_, m)
    lock(lock_) do
        n = length(ring.data)
        m <= ring.pos || error("tap ring not yet filled ($(ring.pos) < $m)")
        m <= n || error("tap ring too small ($n < $m)")
        out = Vector{ComplexF32}(undef, m)
        start = ring.pos - m           # 0-based producer index of out[1]
        @inbounds for k = 1:m
            out[k] = ring.data[mod(start + k - 1, n)+1]
        end
        (out, ring.pos)
    end
end

mutable struct RawStream
    channel::SignalChannel{Complex{Int16},1}
    tap::TapRing
    tap_lock::ReentrantLock
    recorder::Base.Process
    reader::Task
end

function start_raw_stream(; tap_seconds = 0.6, capacity_chunks = 4000)
    run(ignorestatus(`pkill -x m2sdr_record`))
    sleep(0.3)

    channel = SignalChannel{Complex{Int16},1}(CHUNK, capacity_chunks)
    tap = TapRing(round(Int, tap_seconds * FS_HZ))
    tap_lock = ReentrantLock()

    # Read the recorder's stdout through a libuv pipe: reads block the *task*,
    # not the thread. A FIFO + blocking IOStream instead parks the reader's OS
    # thread in a raw read ccall, and any Polyester sticky worker pinned to
    # that thread (acquisition!) starves forever.
    #
    # Warm the exact read path on a throwaway process first: the recorder
    # starts streaming the moment it launches, and every millisecond the
    # reader spends compiling is DMA ring the startup may overrun. (The sweep
    # calibration absorbs a lossy start into the measured origin either way.)
    let warm = open(`head -c 65536 /dev/zero`, "r")
        read!(warm, Vector{UInt8}(undef, 65536))
        close(warm)
    end
    recorder = open(`m2sdr_record -q - 0`, "r")

    # 64 KiB of pipe = ~2 ms of slack at 32 MB/s; any longer reader stall backs
    # into the kernel ring (65 ms) and the driver drops whole buffers, slipping
    # the host axis ~262 chips (mod 1023) per 1024-sample buffer — every
    # handover placed after that lands off-peak. ~1 s of slack removes it.
    let want = 32 * 1024 * 1024
        got = -1
        while want >= 1024 * 1024
            got = ccall(:fcntl, Cint, (Cint, Cint, Cint),
                        Base._fd(recorder.out), 1031 #= F_SETPIPE_SZ =#, want)
            got > 0 && break
            want >>= 1
        end
        got > 0 ? (@info "recorder pipe capacity: $(got >> 20) MiB") :
                  (@warn "could not grow the recorder pipe (losses will slip the host axis)")
    end

    reader = Threads.@spawn :interactive begin
        io = recorder
        nbuf = capacity_chunks + 2
        pool = [Matrix{Complex{Int16}}(undef, CHUNK, 1) for _ = 1:nbuf]
        raw = Vector{UInt8}(undef, CHUNK * 8)
        idx = 1
        try
            while isopen(channel)
                read!(io, raw)
                words = reinterpret(Int16, raw)     # 4 × Int16 per sample (2R2T)
                buf = pool[idx]
                @inbounds for k = 1:CHUNK
                    buf[k, 1] = Complex(words[4k-3], words[4k-2])   # antenna 0
                end
                lock(tap_lock) do
                    tap_append!(tap, buf)
                end
                put!(channel, buf)
                Threads.atomic_add!(RAW_CHUNKS, 1)
                idx = mod1(idx + 1, nbuf)
            end
        catch e
            e isa EOFError || e isa InvalidStateException || rethrow()
        finally
            close(channel)
            close(io)
        end
    end
    Base.errormonitor(reader)
    RawStream(channel, tap, tap_lock, recorder, reader)
end

acq_snapshot(stream::RawStream, noncoherent) =
    tap_snapshot(stream.tap, stream.tap_lock, 10 * 4000 * noncoherent)

function acquire_prns(stream::RawStream, prns; noncoherent = 4)
    signal, host_end = acq_snapshot(stream, noncoherent)
    results = acquire(gpsl1, signal, FS, prns;
                      min_doppler_coverage = ACQ_COVERAGE,
                      num_coherently_integrated_code_periods = 10,
                      num_noncoherent_accumulations = noncoherent)
    # Acquired code phases are referenced to the start of the signal buffer.
    (results, host_end - length(signal))
end

# ── 3. Origin calibration ─────────────────────────────────────────────────────
prompt_power(d) = abs2(d.correlator.accumulators[2])

function collect_powers(sdr, ch, n; skip = 2)
    powers = Float64[]
    seen = 0
    last_count = -1
    deadline = time() + 0.002 * (n + skip) + 1.0
    while length(powers) < n && time() < deadline
        yield()
        d = GNSSM2SDR._read_dump_csrs(sdr, ch, Val(1))
        d === nothing && continue
        d.count == last_count && continue
        last_count = d.count
        seen += 1
        seen > skip && push!(powers, prompt_power(d))
    end
    powers
end

function sweep_measure(sdr, ch, S0, r, doppler0, phases, dumps_each)
    out = Tuple{Float64,Float64}[]
    for phi_base in phases
        target = sample_count(sdr.bank) + SWEEP_MARGIN
        phi = mod(phi_base + (target - S0) * r, CA_CODE_LENGTH)
        schedule!(ch, target; carrier_hz = doppler0,
                  code_doppler_hz = doppler0 * GPS_CA_CHIP_RATE / GPS_L1_HZ,
                  code_phase_chips = phi)
        t0 = time()
        while apply_status(ch).armed && time() - t0 < 0.5
            yield()
        end
        powers = collect_powers(sdr, ch, dumps_each)
        isempty(powers) || push!(out, (Float64(phi_base), mean(powers)))
    end
    out
end

"""
Sweep-calibrate against `acq_res`. Returns `delta` samples to add to
`sdr.device_origin`, or `nothing` when no usable correlation peak was found.
"""
function calibrate_origin(sdr, stream, acq_res)
    ch = sdr.bank.channels[1]
    doppler0 = Float64(ustrip(Hz, acq_res.carrier_doppler))
    code_step = code_word(ch, doppler0)
    r = code_step / (1 << FRAC)               # chips per sample
    sample_shift = max(1, round(Int, 0.5 * (1 << FRAC) / code_step))
    load_code!(ch, acq_res.prn,
               [get_code(gpsl1, i, acq_res.prn) > 0 ? 1 : 0 for i = 0:1022])
    write(sdr.csr, ch.prefix * "spacing", spacing_word(ch, sample_shift, doppler0))
    write(sdr.csr, ch.prefix * "carrier_freq", carrier_word(ch, doppler0))
    write(sdr.csr, ch.prefix * "code_freq", code_step)

    S0 = sample_count(sdr.bank)
    coarse = sweep_measure(sdr, ch, S0, r, doppler0, 0:1022, 6)
    isempty(coarse) && return nothing
    best_coarse = coarse[argmax(last.(coarse))][1]
    fine = sweep_measure(sdr, ch, S0, r, doppler0,
                         [mod(best_coarse - 3 + 0.25i, 1023) for i = 0:24], 30)
    isempty(fine) && return nothing
    phi_fine = fine[argmax(last.(fine))][1]
    floor_power = median(last.(fine))
    recheck = sweep_measure(sdr, ch, S0, r, doppler0,
                            [mod(phi_fine - 2 + 0.25i, 1023) for i = 0:16], 20)
    isempty(recheck) && return nothing
    phi_peak, peak_power = recheck[argmax(last.(recheck))]
    ratio = peak_power / floor_power
    @info @sprintf("PRN %d sweep peak %.2fx floor at %.2f chips",
                   acq_res.prn, ratio, phi_peak)
    ratio < 2.5 && return nothing

    # Fresh acquisition right after the sweep keeps Doppler staleness tiny.
    results, host_at_acq = acquire_prns(stream, [Int(acq_res.prn)])
    fresh = results[argmax(map(x -> x.CN0, results))]
    phi_acq = Float64(fresh.code_phase)
    doppler = Float64(ustrip(Hz, fresh.carrier_doppler))
    code_freq = GPS_CA_CHIP_RATE * (1.0 + doppler / GPS_L1_HZ)

    predicted = mod(
        phi_acq + code_freq * (S0 - (sdr.device_origin + host_at_acq)) / FS_HZ,
        CA_CODE_LENGTH,
    )
    err_chips = rem(predicted - phi_peak, CA_CODE_LENGTH, RoundNearest)
    delta = round(Int, err_chips * FS_HZ / code_freq)
    @info @sprintf("origin correction %.3f chips = %+d samples (predicted %.2f, swept %.2f)",
                   err_chips, delta, predicted, phi_peak)
    delta
end

# One assign-style handover trial at a probe offset (chips) from the
# acquisition-predicted phase; returns the mean prompt power.
function handover_trial(sdr, acq, host_at_acq, offset_chips; dumps = 8)
    GNSSReceiver.assign_channel!(
        sdr, 1, acq.prn, acq.carrier_doppler,
        acq.carrier_doppler * (GPS_CA_CHIP_RATE / GPS_L1_HZ),
        mod(Float64(acq.code_phase) + offset_chips, 1023.0), host_at_acq;
        el_sample_spacing = 2, signal = gpsl1)
    powers = collect_powers(sdr, sdr.bank.channels[1], dumps)
    isempty(powers) ? 0.0 : mean(powers)
end

"""
Refine the origin with a *fresh-frame* mini-scan: the coarse sweep's peak is
biased by Doppler staleness over its ~20 s (the trial phases propagate from a
sweep-start anchor with the initial acquisition's Doppler, and the LO drifts),
so it is only good to a couple of chips. Here every trial is a fresh
assign-style handover — staleness per trial is milliseconds — so the peak
offset *is* the residual origin error.
"""
function refine_origin!(sdr, stream, prn; halfspan = 3.0)
    results, host_at_acq = acquire_prns(stream, [prn])
    acq = results[argmax(map(x -> x.CN0, results))]
    code_freq = GPS_CA_CHIP_RATE *
                (1.0 + Float64(ustrip(Hz, acq.carrier_doppler)) / GPS_L1_HZ)
    scan = [(off, handover_trial(sdr, acq, host_at_acq, off))
            for off = -halfspan:0.25:halfspan]
    powers = last.(scan)
    best = argmax(powers)
    ratio = powers[best] / median(powers)
    offset = scan[best][1]
    # The peak sitting at probe offset +o means the un-probed handover phase is
    # too LOW by o; raising the phase means LOWERING the origin (the phase is
    # propagated over `target - origin - host`).
    delta = -round(Int, offset * FS_HZ / code_freq)
    @info @sprintf("refine origin: peak %.1fx scan-floor at %+.2f chips → %+d samples",
                   ratio, offset, delta)
    (isnan(ratio) || ratio < 3) && return nothing
    sdr.device_origin += delta
    delta
end

# Program a handover exactly the way `assign_channel!` will during the run,
# then compare prompt power on the predicted phase against a deliberately
# wrong phase — the go/no-go for the corrected origin.
function verify_origin(sdr, stream, prn)
    results, host_at_acq = acquire_prns(stream, [prn])
    acq = results[argmax(map(x -> x.CN0, results))]
    doppler_hz = Float64(ustrip(Hz, acq.carrier_doppler))
    GNSSReceiver.assign_channel!(
        sdr, 1, acq.prn, acq.carrier_doppler,
        acq.carrier_doppler * (GPS_CA_CHIP_RATE / GPS_L1_HZ),
        Float64(acq.code_phase), host_at_acq;
        el_sample_spacing = 2, signal = gpsl1)
    sleep(0.05)
    powers = collect_powers(sdr, sdr.bank.channels[1], 40)
    isempty(powers) && return 0.0
    target = sample_count(sdr.bank) + SWEEP_MARGIN
    schedule!(sdr.bank.channels[1], target;
              carrier_hz = doppler_hz,
              code_doppler_hz = doppler_hz * GPS_CA_CHIP_RATE / GPS_L1_HZ,
              code_phase_chips = mod(Float64(acq.code_phase) + 500.0, 1023.0))
    sleep(0.05)
    floor_powers = collect_powers(sdr, sdr.bank.channels[1], 40)
    isempty(floor_powers) && return 0.0
    mean(powers) / mean(floor_powers)
end

# ── Main ──────────────────────────────────────────────────────────────────────
function main()
    # Redirected stderr is fully buffered; flush it so the log is observable
    # live. A Timer callback that throws kills the timer silently — never let
    # that happen to the only thing making the log visible.
    flusher = Timer(1; interval = 2) do _
        try
            flush(stderr)
            flush(stdout)
        catch
        end
    end

    warm_up()

    # Latch the origin BEFORE streaming: the bank counter only advances on
    # samples DMA0 accepts, so its pre-stream value is exactly the device index
    # of the stream's first sample.
    csr_probe = GNSSM2SDR.LiteXCSR(CSR_CSV)
    bank_probe = GNSSM2SDR.GNSSBank(csr_probe; fs = FS)
    origin_prelatch = sample_count(bank_probe)
    close(csr_probe)
    @info "pre-stream device counter: $origin_prelatch"

    stream = start_raw_stream()
    @info "raw stream started"
    let last = Ref(0)
        global stream_watchdog = Timer(5; interval = 5) do _
            c = RAW_CHUNKS[]
            rate = (c - last[]) / 5
            alive = process_running(stream.recorder)
            (rate < 250 || !alive) &&
                @warn "raw stream unhealthy" chunks_per_s = rate recorder_alive = alive
            last[] = c
        end
    end

    # Only drive 8 of the 20 hardware channels: every channel dumps at ~1 kHz
    # whether assigned or not (there is no per-channel enable), so the CSR
    # poller's ioctl load scales with the bank size — and 8 is plenty for a fix.
    sdr = M2SDRCorrelator(CSR_CSV, stream.channel; fs = FS, n_channels = 5)
    @info "gateware exposes $(GNSSReceiver.num_hardware_channels(sdr)) channels"

    # Poll dumps over CSR (the doc'd bring-up path: DMA1's IRQ cadence is far
    # too slow for a 1 kHz loop until the driver gains per-channel buffers).
    # DMA1 dump source: with the rebuilt driver (DMA_BUFFER_PER_IRQ = 1) a
    # buffer completes and interrupts every 64 records — at a 16 kHz strobe
    # that is ~2-3 ms of dump latency, inside the 8 Hz PLL's delay budget, and
    # the ring makes the dump stream LOSSLESS. CSR polling could never be:
    # its depth-1 dump registers turned every scheduler hiccup into missed
    # dumps, and one missed dump per subframe breaks that subframe's parity.
    start!(sdr; dump_source = :dma, epoch_period = 50)
    sdr.device_origin = origin_prelatch

    # Host-axis slip watch: the floor of (device counter − origin) − delivered
    # host samples rises only when DMA0 buffers were dropped.
    let base = Ref{Int64}(typemax(Int64)), win = Ref{Int64}(typemax(Int64)),
        tick = Ref(0), t0 = time()

        global slip_watch = Timer(10; interval = 10) do _
            try
                infl = (sample_count(sdr.bank) - sdr.device_origin) -
                       Int64(RAW_CHUNKS[]) * CHUNK
                win[] = min(win[], infl)
                tick[] += 1
                if tick[] % 6 == 0
                    if time() - t0 < 130
                        base[] = min(base[], win[])
                    elseif base[] != typemax(Int64)
                        slip = win[] - base[]
                        slip > 1200 && @warn @sprintf(
                            "host axis slipped %+d samples (%.0f chips mod 1023) since start — handovers now land off-peak",
                            slip, mod(slip * 0.2558, 1023.0))
                    end
                    win[] = typemax(Int64)
                end
            catch
            end
        end
    end

    # Keep the channel drained during calibration and count what we eat.
    drain_running = Ref(true)
    drained = Threads.Atomic{Int}(0)
    # Batch-drain: PipeChannel's single-item blocking take! parks in a
    # sleep-poll (~10 ms granularity), capping a one-at-a-time drainer at
    # ~100 chunks/s — an order of magnitude below the stream rate. The backlog
    # that builds then makes `valid_at` minutes stale by receiver start, and
    # the handover propagation degrades with it.
    drainer = Threads.@spawn :interactive begin
        while drain_running[]
            n = Base.n_avail(stream.channel.channel)
            if n == 0
                sleep(0.002)
                continue
            end
            take!(stream.channel, n)
            Threads.atomic_add!(drained, n * CHUNK)
        end
    end
    Base.errormonitor(drainer)

    # Wait for the tap ring to actually fill (a fixed nap raced a slow reader
    # start and threw "tap ring not yet filled").
    let deadline = time() + 30
        while stream.tap.pos < length(stream.tap.data) && time() < deadline
            sleep(0.1)
        end
        stream.tap.pos >= length(stream.tap.data) ||
            error("raw stream did not fill the tap ring within 30 s — is m2sdr_record streaming?")
    end
    results, _ = acquire_prns(stream, collect(1:32); noncoherent = 10)
    strong = sort(filter(r -> r.CN0 > CN0_FLOOR_DBHZ, results); by = r -> -r.CN0)
    @info "acquired $(length(strong)) satellites above $(CN0_FLOOR_DBHZ) dBHz:"
    for r in strong[1:min(end, 8)]
        @info @sprintf("  PRN %2d  CN0 %.1f dBHz  doppler %+7.0f Hz",
                       r.prn, r.CN0, ustrip(Hz, r.carrier_doppler))
    end
    isempty(strong) && error("no satellites acquired — check antenna/RF")

    calibrated = false
    # Fast path: every calibration so far measured the pre-stream latch to be
    # exact within ±9 samples (±2.3 chips) — inside a widened fresh-frame
    # mini-scan. Try that first; it takes seconds and tolerates weaker
    # satellites than a 6-minute sweep whose peak also smears with LO drift.
    # The refine scan IS the verification: a ≥3× peak at the predicted phase
    # demonstrates an assign-style handover landing on the satellite. A separate
    # re-check trial minutes later just samples the fade statistics again.
    for cand in strong[1:min(end, 3)]
        refinement = refine_origin!(sdr, stream, Int(cand.prn); halfspan = 6.0)
        if !isnothing(refinement)
            calibrated = true
            break
        end
    end
    # Fallback: the full code-phase sweep (origin unknown beyond the code
    # period, e.g. after a lossy stream start).
    if !calibrated
        for cand in strong[1:min(end, 3)]
            delta = calibrate_origin(sdr, stream, cand)
            isnothing(delta) && continue
            sdr.device_origin += delta
            refinement = refine_origin!(sdr, stream, Int(cand.prn))
            if isnothing(refinement)
                sdr.device_origin -= delta
                continue
            end
            ratio = verify_origin(sdr, stream, Int(cand.prn))
            @info @sprintf("handover verification: %.1fx floor", ratio)
            if ratio > 3
                calibrated = true
                break
            else
                sdr.device_origin -= delta + refinement
            end
        end
    end
    GNSSReceiver.release_channel!(sdr, 1)   # also clears the poller's active mask
    calibrated || error("origin calibration failed — no candidate verified")

    # Hand the stream to the receiver: stop draining, account for what we ate,
    # flush stale records.
    drain_running[] = false
    # Unblock the drainer if it is parked in take! by feeding it nothing — it
    # exits after its current take!, which the reader keeps satisfying.
    wait_t0 = time()
    while !istaskdone(drainer) && time() - wait_t0 < 3
        sleep(0.01)
    end
    consumed_before_receive = drained[]
    sdr.device_origin += consumed_before_receive
    while Base.n_avail(sdr.dumps) > 0
        take!(sdr.dumps)
    end
    @info "starting receiver ($(consumed_before_receive) samples consumed pre-receive)"

    # Decode progress alongside the default payload: bits since preamble sync
    # and the decoded TOW per satellite — the visibility that separates "loop
    # works, bits scrambled" from "waiting on subframes".
    # Both decoder fields are Union{Nothing,Int} — an unguarded conversion
    # throws inside the processing task and silently closes the data channel.
    # `found`/`nsoft` expose Tracking's bit-buffer state: whether bit sync ever
    # happened and how many soft bits this chunk carried to the decoder.

    strong_prns = [Int(r.prn) for r in strong[1:min(end, 6)]]
    @info "receiver PRN set: $strong_prns"
    bit_log = isempty(BIT_LOG_NAME) ? nothing :
              open(joinpath(dirname(@__FILE__), BIT_LOG_NAME), "w")
    isnothing(bit_log) ||
        @info "logging every soft bit and 1 ms prompt to $(BIT_LOG_NAME)"
    # The probe's device axis is `sdr.device_origin` as it stands now — the
    # calibrated origin plus everything the pre-receiver drainer ate. It is
    # fixed for the rest of the run, which is exactly what makes a raw sample
    # addressable by the device index a record reports.
    xcorr_log = isempty(XCORR_LOG_NAME) ? nothing :
                open(joinpath(dirname(@__FILE__), XCORR_LOG_NAME), "w")
    probe = isnothing(xcorr_log) ? nothing :
            CpuReference(xcorr_log, sdr.device_origin, FS_HZ,
                         GNSSReceiver.num_hardware_channels(sdr);
                         prns = XCORR_PRNS, every = XCORR_EVERY)
    isnothing(probe) || @info string(
        "FPGA-vs-CPU probe → ", XCORR_LOG_NAME, " (origin ", sdr.device_origin,
        ", PRNs ", isempty(XCORR_PRNS) ? "all" : join(sort!(collect(XCORR_PRNS)), ","),
        ", every ", XCORR_EVERY, " records)")
    data = hw_receive(
        sdr,
        bit_log,
        dirname(@__FILE__);
        xcorr = probe,
        prns = strong_prns,
        acq_min_doppler_coverage = ACQ_COVERAGE,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 3,
        max_meas = 2^11,
        # The first acquisition fires as soon as the buffer fills (the timer
        # starts at -Inf). With asynchronous acquisition the periodic rescan
        # costs the pipeline nothing, so it runs at a cadence that can actually
        # recover a lost satellite instead of being switched off.
        acquire_every = ACQ_EVERY,
        acquire_async = ACQ_ASYNC,
        # Loop-delay management: the correction from epoch k reaches the NCO
        # ~τ = (fold latency + feedback delay) later. The default 18 Hz PLL at
        # τ ≈ 4-5 ms sits in the delay-instability zone (BL·τ ≈ 0.08): power
        # and frequency hold, phase never locks, no bits decode. Cut both
        # knobs: 1-epoch apply (a late commit just applies immediately) and an
        # 8 Hz PLL (BL·τ ≈ 0.02).
        feedback_delay_epochs = 1,
        doppler_estimator = ConventionalAssistedPLLAndDLL(;
            carrier_loop_filter_bandwidth = 12.0Hz,
        ),
        extract = my_extract,
    )

    # GC pauses stop the CSR poller with the rest of the world; each pause is a
    # burst of missed dumps = bit slips that scramble the decoders. With 55 GB
    # free and this run capped at minutes, trading memory for a pause-free
    # decode window is the right deal — with a valve, re-enabled if RAM runs low.
    GC.gc(true)
    GC.enable(false)
    gc_off = true


    t0 = time()
    last_print = 0.0
    first_fix = nothing
    fix_time = 0.0
    n_solutions = 0
    bits_log = Dict{Int,Vector{Float32}}()
    for payload in data
        d = payload.data
        for e in payload.decode
            isempty(e.soft) || append!(get!(bits_log, e.prn, Float32[]), e.soft)
        end
        t = time() - t0
        has_fix = d.pvt.time !== nothing
        if has_fix
            n_solutions += 1
            if isnothing(first_fix)
                first_fix = d.pvt
                fix_time = t
                lla = LLAfromECEF(wgs84)(d.pvt.position)
                @info @sprintf(
                    "*** FIRST FIX after %.1f s: lat %.6f° lon %.6f° alt %.1f m (%d sats) ***",
                    t, lla.lat, lla.lon, lla.alt, length(d.pvt.sats))
            end
        end
        if t - last_print >= 5.0
            last_print = t
            sats = join(
                [@sprintf("%d:%.0f", k[2], 10log10(ustrip(Hz, Unitful.linear(v.cn0))))
                 for (k, v) in pairs(d.sat_data)],
                " ")
            dec = join(
                [@sprintf("%d:%db%s%s%d", e.prn, e.nbits, e.tow >= 0 ? "*" : "",
                          e.found ? "F" : "-", e.nsoft)
                 for e in payload.decode],
                " ")
            pos = if has_fix
                lla = LLAfromECEF(wgs84)(d.pvt.position)
                @sprintf("%.6f°, %.6f°, %.0fm", lla.lat, lla.lon, lla.alt)
            else
                "no fix"
            end
            # lag = wall time minus receiver runtime: how far the processing
            # is behind real time. The NCO corrections apply `lag` late, so a
            # growing lag silently opens the loop delay — the PLL stops
            # phase-locking long before anything else looks wrong.
            @info @sprintf("t=%5.1fs rt=%5.1fs cn0[%s] dec[%s] | %s | missed=%d free=%dG",
                           t, ustrip(u"s", d.runtime), sats, dec, pos,
                           sdr.missed_csr_dumps, Sys.free_memory() ÷ 2^30)
            # Memory valve: a paused decode beats an OOM kill.
            if gc_off && Sys.free_memory() < 8 * 2^30
                @warn "low memory — re-enabling GC"
                GC.enable(true)
                gc_off = false
            end
        end
        (!isnothing(first_fix) && t - fix_time > RUN_AFTER_FIX) && break
        t > MAX_SECONDS && break
    end

    @info "stopping"
    let link = BIT_LOG_SOURCE[].link
        @info "record continuity: " * join(
            [string(a.prn, "→ch", ch, " lost=", link.lost_record_samples[ch],
                    "smp overlap=", link.overlapping_record_samples[ch], "smp")
             for (ch, a) in enumerate(link.assignments) if !isnothing(a)], "  ")
        @info string("link: gaps=", link.lost_record_gaps, " stale=", link.stale_dumps,
                     " dropped=", link.dropped_dumps, " skipped_epochs=", link.skipped_epochs)
    end
    if !isnothing(probe)
        @info string(
            "FPGA-vs-CPU probe: ", probe.records, " records, mixing sign ", probe.sign,
            ", CPU-side delta ", join([@sprintf("%.3f", d) for d in probe.delta], "/"),
            " chips; skipped ", probe.skipped_future, " too-new / ", probe.skipped_old,
            " rolled-out, ", probe.unaligned, " unaligned, ",
            @sprintf("%.1f", probe.cpu_seconds), " s of CPU")
        close(xcorr_log)
    end
    isnothing(bit_log) || close(bit_log)
    foreach(close, values(BIT_LOG_SOURCE[].prompts))
    for (prn, v) in bits_log
        open(joinpath(homedir(), "hwfix/run", "bits_c93_prn$(prn).f32"), "w") do io
            write(io, v)
        end
    end
    @info "soft-bit totals: " * join(["$k:$(length(v))" for (k, v) in bits_log], " ")
    gc_off && GC.enable(true)
    stop!(sdr)
    close(stream.channel)
    process_running(stream.recorder) && kill(stream.recorder)

    if isnothing(first_fix)
        @error "no position fix obtained"
        # Let any errormonitor'd task failure reach the log before exiting.
        sleep(3)
        flush(stderr)
        exit(1)
    end
    lla = LLAfromECEF(wgs84)(first_fix.position)
    @info @sprintf("SUCCESS: first fix at t=%.1fs → %.6f° %.6f° %.1f m; %d solutions total",
                   fix_time, lla.lat, lla.lon, lla.alt, n_solutions)
end

main()
