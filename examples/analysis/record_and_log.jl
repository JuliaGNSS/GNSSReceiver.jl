# A live hardware-correlator run on the LiteX-M2SDR that records the raw DMA0
# stream to a file and logs every satellite's tracking state once per chunk, so
# a run can be replayed offline (replay_software.jl, replay_simfpga.jl) and
# compared with what the board did.
#
# The raw stream is teed to RAWFILE by the recorder process itself (2R2T sc16:
# four Int16 per sample, antenna 1 = the first two words, 32 MB/s at 4 MS/s).
#
# Usage: julia -t 6,4 --project=<env> record_and_log.jl RAWFILE CSVFILE [MAX_S] [PLL_HZ] [MODE] [DELAY_EPOCHS]
#   MODE          std | nco | nconp (see tracking_log.jl); default nco, the receiver's default
#   PLL_HZ        carrier loop bandwidth, default 18 (the GPS L1 C/A reference)
#   DELAY_EPOCHS  the link's feedback_delay_epochs, default 2
#
# Two things about instrumented runs:
#
#   - Julia block-buffers stderr when it is a file, so `julia … 2> log` shows
#     nothing until exit. Pipe through `cat` or `tee` for a live log.
#   - A non-default `extract` (the `snapshot` here) makes the receive pipeline
#     recompile for its payload type inside the first live chunk — measured as
#     1000–1500 skipped epochs and 10–14 lost-record gaps at t ≈ 5 s in every
#     instrumented run — because the package precompiles the pipeline for the
#     default payload only. The script therefore takes the first payload off
#     the output *before* enabling the device, so the compile happens while the
#     dump ring is still empty.

using GNSSSignals: GPSL1CA
using GNSSM2SDR
include(joinpath(@__DIR__, "tracking_log.jl"))
using Unitful: ms, dBHz

const FS = 4e6Hz
const CHUNK = 8000                         # 2 ms of samples per processing chunk
const N_HW_CHANNELS = 20                   # all of the gateware's channels
const CSR_CSV = expanduser("~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch20_ant1/csr.csv")
const RAWFILE = ARGS[1]
const CSVFILE = ARGS[2]
const MAX_SECONDS = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 90.0
const PLL_HZ = length(ARGS) >= 4 ? parse(Float64, ARGS[4]) : 18.0
const MODE = length(ARGS) >= 5 ? ARGS[5] : "nco"
const DELAY_EPOCHS = length(ARGS) >= 6 ? parse(Int, ARGS[6]) : 2
const gpsl1 = GPSL1CA()

function main()
    origin = let csr = LiteXCSR(CSR_CSV)
        count = sample_count(GNSSBank(csr; fs = ustrip(Hz, FS)))
        close(csr)
        count
    end
    run(ignorestatus(`pkill -x m2sdr_record`))
    sleep(0.3)
    # Tee the raw stream to RAWFILE while the receiver consumes it.
    stream = start_raw_stream(;
        chunk = CHUNK,
        command = `sh -c "m2sdr_record -c 0 -q - 0 | tee $RAWFILE"`,
    )
    sdr = M2SDRCorrelator(CSR_CSV, stream.channel; fs = FS, n_channels = N_HW_CHANNELS)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = FS,
        reference_signal = gpsl1,
        feedback_delay_epochs = DELAY_EPOCHS,
    )
    @info "recording run" RAWFILE MODE PLL_HZ DELAY_EPOCHS
    data = receive(
        sdr,
        gpsl1,
        FS;
        link,
        doppler_estimator = estimator_for(MODE, PLL_HZ),
        prns = 1:32,
        max_meas = 2^11,
        acq_min_doppler_coverage = 25_000.0Hz,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 5,
        acquire_every = 60s,
        code_lock_cn0_threshold = 24.0dBHz,
        pvt_update_interval = 2ms,        # one snapshot per chunk
        extract = snapshot,
    )
    io = open(CSVFILE, "w")
    println(io, CSV_HEADER)
    # Let the pipeline compile for the snapshot payload on the first raw chunk,
    # with the correlators still off — see the header comment.
    first_rows = take!(data)
    write_rows(io, first_rows, link)
    start!(sdr; dump_source = :dma, epoch_period = 50)
    sdr.device_origin = origin

    t0 = time()
    last_print = -Inf
    for rows in data
        t = time() - t0
        write_rows(io, rows, link)
        if t - last_print >= 5
            last_print = t
            @info @sprintf("t=%6.1f s  runtime=%6.1f s  locked[%s]  dropped_nco=%d lost_gaps=%d",
                t, isempty(rows) ? NaN : rows[1].runtime, locked_summary(rows),
                link.dropped_nco_updates, link.lost_record_gaps)
            flush(io)
            flush(stderr)
        end
        t > MAX_SECONDS && break
    end
    close(data)
    close(stream)
    stop!(sdr)
    isnothing(sdr.reader) || wait(sdr.reader)
    close(io)
    run(ignorestatus(`pkill -x m2sdr_record`))
    @info "dump stream: lost-record gaps $(link.lost_record_gaps), re-arm gaps $(link.rearm_gaps), " *
          "device drops $(link.dropped_dumps), skipped epochs $(link.skipped_epochs), " *
          "stale $(link.stale_dumps), implausible $(link.implausible_dumps), " *
          "dropped NCO updates $(link.dropped_nco_updates)"
end

main()
