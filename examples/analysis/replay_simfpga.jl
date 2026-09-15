# Replay a recorded DMA0 stream through the simulated hardware correlator the
# tests use (test/simulated_fpga.jl) — a device that honours `apply_at_sample`
# exactly — with a chosen feedback delay and estimator, to reproduce the
# hardware loop's behaviour offline and deterministically.
#
# Usage: julia -t 4,2 --project=<env> replay_simfpga.jl RAWFILE CSVFILE DELAY_EPOCHS PLL_HZ SECONDS [MODE]
#   MODE  std | nco | nconp (see tracking_log.jl); default nco
#
# The environment needs GNSSReceiver (`dev`ed, for the test utility) and
# GNSSSignals; nothing device-specific.

using GNSSSignals: GPSL1CA
include(joinpath(@__DIR__, "tracking_log.jl"))
include(joinpath(pkgdir(GNSSReceiver), "test", "simulated_fpga.jl"))
using Unitful: ms, dBHz

const FS = 4e6Hz
const CHUNK = 8000
const RAWFILE, CSVFILE = ARGS[1], ARGS[2]
const DELAY = parse(Int, ARGS[3])
const PLL_HZ = parse(Float64, ARGS[4])
const SECONDS = parse(Float64, ARGS[5])
const MODE = length(ARGS) >= 6 ? ARGS[6] : "nco"
const gpsl1 = GPSL1CA()

function main()
    sdr = SimulatedFPGA(gpsl1; sampling_freq = FS, chunk = CHUNK, sample_type = Complex{Int16})
    producer = replay_raw_file!(sdr, RAWFILE; chunk = CHUNK, seconds = SECONDS)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = FS,
        reference_signal = gpsl1,
        feedback_delay_epochs = DELAY,
    )
    @info "simulated-FPGA replay" RAWFILE DELAY PLL_HZ SECONDS MODE
    data = receive(
        sdr,
        gpsl1,
        FS;
        link,
        acquire_async = false,
        prns = 1:32,
        max_meas = 2^11,
        acq_min_doppler_coverage = 25_000.0Hz,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 5,
        acquire_every = 60s,
        code_lock_cn0_threshold = 24.0dBHz,
        doppler_estimator = estimator_for(MODE, PLL_HZ),
        pvt_update_interval = 2ms,
        extract = snapshot,
    )
    io = open(CSVFILE, "w")
    println(io, CSV_HEADER)
    t0 = time()
    last_print = -Inf
    for rows in data
        write_rows(io, rows, link)
        if time() - last_print >= 10
            last_print = time()
            @info @sprintf("wall=%6.1f s  runtime=%6.2f s  locked[%s]", time() - t0,
                isempty(rows) ? NaN : rows[1].runtime, locked_summary(rows))
            flush(io)
            flush(stderr)
        end
    end
    wait(producer)
    close(io)
    late = count(((u, at),) -> at > u.apply_at_sample, zip(sdr.applied, sdr.applied_at))
    @info "applied NCO updates: $(length(sdr.applied)) ($late applied later than scheduled); " *
          "dropped NCO updates: $(link.dropped_nco_updates), lost-record gaps: $(link.lost_record_gaps)"
end

main()
