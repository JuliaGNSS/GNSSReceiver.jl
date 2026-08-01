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
#   4. receive(sdr, …) with CSR-polled dumps until the PVT solve returns a fix.
#
# Usage: julia -t 8 --project=. position_fix.jl [MAX_SECONDS] [RUN_AFTER_FIX]

using Printf
using Statistics: mean, median, std
using StaticArrays: SVector
using Unitful
using Unitful: Hz, ms, ustrip
using Geodesy: LLAfromECEF, wgs84

using Acquisition: acquire
using FFTW
using GNSSSignals: GPSL1CA, get_code
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

const gpsl1 = GPSL1CA()
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
    data = GNSSReceiver.receive(
        sdr,
        gpsl1,
        FS;
        prns = [1, 2, 3, 4, 5, 7],
        acq_min_doppler_coverage = ACQ_COVERAGE,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 3,
        max_meas = 2^11,
        acquire_every = 50ms,
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
    data = GNSSReceiver.receive(
        sdr,
        gpsl1,
        FS;
        prns = strong_prns,
        acq_min_doppler_coverage = ACQ_COVERAGE,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 3,
        max_meas = 2^11,
        # The first acquisition fires as soon as the buffer fills (the timer
        # starts at -Inf); a long period after that keeps multi-second
        # acquisition stalls out of the decode window.
        acquire_every = 300u"s",
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
