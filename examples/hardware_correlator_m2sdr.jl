# A hardware-correlator receiver on a LiteX-M2SDR running the gnss-m2sdr
# gateware (issue #107). The FPGA downconverts and correlates; this host runs
# acquisition, the tracking loop filters, decoding and PVT off the correlator
# dumps, and pushes NCO corrections back. The raw sample stream keeps flowing
# alongside — acquisition, decoding and the receiver's clock run on it, and the
# correlators only see samples while it is being drained.
#
# Needs, on the host the board is plugged into:
#   - the `m2sdr` litepcie driver (`/dev/m2sdr0` for CSRs and the raw DMA0
#     stream, `/dev/m2sdr1` for the DMA1 record stream) and `m2sdr_record` on
#     the PATH, with the RF front end already set up (4 MS/s, L1 centre
#     frequency);
#   - the gateware's CSR map (`CSR_CSV` below);
#   - GNSSM2SDR.jl, the vendor half of the `AbstractHardwareCorrelatorSDR`
#     interface for this board (https://github.com/JuliaGNSS/GNSSM2SDR.jl), in
#     the active project;
#   - Julia started with interactive threads, e.g. `julia -t 6,3`: the chunk
#     pipeline runs on the interactive pool so an acquisition scan on the default
#     pool cannot park it while the device holds a stale NCO word.
#
# Usage: julia -t 6,3 --project=. hardware_correlator_m2sdr.jl [MAX_SECONDS] [SECONDS_AFTER_FIX]

using Printf
using Unitful
using Unitful: Hz, ms, s, dBHz, ustrip
using Geodesy: LLAfromECEF, wgs84
using GNSSSignals: GPSL1CA
using SignalChannels: SignalChannel
using Tracking: ConventionalAssistedPLLAndDLL
using GNSSReceiver
using GNSSM2SDR

const FS = 4e6Hz
const CHUNK = 8000                       # 2 ms of samples per processing chunk
const N_HW_CHANNELS = 8                  # of the 20 the gateware has; plenty for a fix
const CSR_CSV = expanduser("~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch20_ant1/csr.csv")
const MAX_SECONDS = length(ARGS) >= 1 ? parse(Float64, ARGS[1]) : 300.0
const SECONDS_AFTER_FIX = length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 60.0
const gpsl1 = GPSL1CA()

# ── Raw sample stream ─────────────────────────────────────────────────────────
# `m2sdr_record` drains DMA0 into a pipe; a task turns antenna 0 of the 2R2T
# sc16 stream into chunks on a `SignalChannel`. The pipe is grown to 32 MiB
# (~1 s at 32 MB/s) so a late reader backs into the pipe rather than into the
# driver's ring, where a dropped buffer would silently shift the host's sample
# count against the device's.
function start_raw_stream(; capacity_chunks = 4000)
    run(ignorestatus(`pkill -x m2sdr_record`))
    sleep(0.3)
    recorder = open(`m2sdr_record -q - 0`, "r")
    let want = 32 * 1024 * 1024, got = -1
        while want >= 1024 * 1024
            got = ccall(:fcntl, Cint, (Cint, Cint, Cint), Base._fd(recorder.out), 1031, want)
            got > 0 && break
            want >>= 1
        end
        got > 0 || @warn "could not grow the recorder pipe; a reader stall may drop raw samples"
    end
    channel = SignalChannel{Complex{Int16},1}(CHUNK, capacity_chunks)
    reader = Threads.@spawn :interactive begin
        nbuf = capacity_chunks + 2
        pool = [Matrix{Complex{Int16}}(undef, CHUNK, 1) for _ = 1:nbuf]
        raw = Vector{UInt8}(undef, CHUNK * 8)
        idx = 1
        try
            while isopen(channel)
                read!(recorder, raw)
                words = reinterpret(Int16, raw)          # 4 × Int16 per sample (2R2T)
                buf = pool[idx]
                @inbounds for k = 1:CHUNK
                    buf[k, 1] = Complex(words[4k-3], words[4k-2])   # antenna 0
                end
                put!(channel, buf)
                idx = mod1(idx + 1, nbuf)
            end
        catch e
            e isa EOFError || e isa InvalidStateException || rethrow()
        finally
            close(channel)
            close(recorder)
        end
    end
    Base.errormonitor(reader)
    (; channel, recorder, reader)
end

# ── Reporting ─────────────────────────────────────────────────────────────────
cn0_db(cn0) = 10 * log10(Unitful.linear(cn0) / Hz)
sat_summary(sat_data) = join(
    (@sprintf("%d:%.0f", prn, cn0_db(sat.cn0)) for ((_, prn), sat) in pairs(sat_data) if sat.is_in_lock),
    " ",
)

function position_summary(pvt)
    lla = LLAfromECEF(wgs84)(pvt.position)
    @sprintf("%.6f° %.6f° %.0f m (%d sats)", lla.lat, lla.lon, lla.alt, length(pvt.sats))
end

function main()
    # The bank's sample counter only advances on samples DMA0 accepts, so its
    # value just before the stream starts is the device index of the stream's
    # first sample — the constant that maps the host's sample count onto the
    # device's, which every acquisition handover is timed with.
    origin = let csr = LiteXCSR(CSR_CSV)
        count = sample_count(GNSSBank(csr; fs = ustrip(Hz, FS)))
        close(csr)
        count
    end

    stream = start_raw_stream()
    sdr = M2SDRCorrelator(CSR_CSV, stream.channel; fs = FS, n_channels = N_HW_CHANNELS)
    # 80 kHz epoch strobes (`epoch_period` in samples): the litepcie driver only
    # completes whole 8 KiB DMA buffers, so the strobe rate is what bounds the
    # dump latency the loop sees (~0.75 ms here). It is also what fills the
    # driver's 256-buffer ring in ~190 ms — a host that stops draining the ring
    # for longer than half of that loses records.
    start!(sdr; dump_source = :dma, epoch_period = 50)
    sdr.device_origin = origin
    @info "gateware exposes $(num_hardware_channels(sdr)) channels; driving $N_HW_CHANNELS"

    # Built here rather than through `receive(sdr, …)` only so its counters can
    # be read at the end.
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = FS,
        reference_signal = gpsl1,
        # One epoch ahead: the vendor package applies rate updates immediately
        # anyway, and a shorter loop delay is what keeps a 12 Hz PLL stable.
        feedback_delay_epochs = 1,
    )
    data = receive(
        raw_sample_channel(sdr),
        gpsl1,
        FS;
        correlator_source = link,
        acquire_async = true,
        processing_threadpool = :interactive,
        prns = 1:32,
        max_meas = 2^11,
        # The front end's LO offset puts the constellation ~14 kHz off before
        # the physical ±5 kHz starts; 10 ms coherent × 5 rounds finds the
        # 25–32 dBHz satellites that decide whether there is a fourth.
        acq_min_doppler_coverage = 25_000.0Hz,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 5,
        acquire_every = 60s,
        # Feedback reaches the NCO a few milliseconds after the measurement; the
        # default 18 Hz PLL sits in the delay-instability zone at that delay.
        doppler_estimator = ConventionalAssistedPLLAndDLL(; carrier_loop_filter_bandwidth = 12.0Hz),
        # Hold a satellite through a fade: the 31–40 dBHz satellites breathe
        # several dB either side of the default 30 dBHz, and every drop costs
        # symbol timing and TOW.
        code_lock_cn0_threshold = 24.0dBHz,
    )

    t0 = time()
    first_fix_at = nothing
    last_fix_time = nothing
    fresh_fixes = 0
    last_print = -Inf
    for d in data
        t = time() - t0
        if !isnothing(d.pvt.time) && !isequal(d.pvt.time, last_fix_time)
            last_fix_time = d.pvt.time
            fresh_fixes += 1
            if isnothing(first_fix_at)
                first_fix_at = t
                @info @sprintf("first fix after %.1f s: %s", t, position_summary(d.pvt))
            end
        end
        if t - last_print >= 5
            last_print = t
            @info @sprintf("t=%6.1f s  runtime=%6.1f s  cn0[%s]  %s", t, ustrip(s, d.runtime),
                           sat_summary(d.sat_data),
                           isnothing(d.pvt.time) ? "no fix" : position_summary(d.pvt))
        end
        (!isnothing(first_fix_at) && t - first_fix_at > SECONDS_AFTER_FIX) && break
        t > MAX_SECONDS && break
    end

    # Closing the output stops `receive`; its processing task closes the sample
    # channel, which ends the raw reader. Only then take the device down.
    close(data)
    wait(stream.reader)
    process_running(stream.recorder) && kill(stream.recorder)
    stop!(sdr)
    isnothing(sdr.reader) || wait(sdr.reader)
    isnothing(sdr.writer) || wait(sdr.writer)

    @info "dump stream: lost-record gaps $(link.lost_record_gaps), re-arm gaps $(link.rearm_gaps), " *
          "device-reported drops $(link.dropped_dumps), skipped epochs $(link.skipped_epochs), " *
          "implausible indices $(link.implausible_dumps)"
    if isnothing(first_fix_at)
        @error "no position fix"
        exit(1)
    end
    @info @sprintf("first fix after %.1f s, %d fresh solutions afterwards", first_fix_at, fresh_fixes)
end

main()
