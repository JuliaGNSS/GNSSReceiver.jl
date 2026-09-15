# Replay a recorded M2SDR DMA0 stream (2R2T sc16: I₁ Q₁ I₂ Q₂ per sample,
# antenna 1 = the first two words) through the SOFTWARE receiver with the same
# acquisition settings as the hardware runs, logging the per-chunk tracking
# state in the same CSV layout as record_and_log.jl. This is the reference the
# hardware loop is measured against: same samples, no feedback delay.
#
# Usage: julia -t 6,4 --project=<env> replay_software.jl RAWFILE CSVFILE [PLL_HZ] [MAX_SECONDS] [MODE]

using GNSSSignals: GPSL1CA
include(joinpath(@__DIR__, "tracking_log.jl"))
using Unitful: ms, dBHz

const FS = 4e6Hz
const CHUNK = 8000
const RAWFILE = ARGS[1]
const CSVFILE = ARGS[2]
const PLL_HZ = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 18.0
const MAX_SECONDS = length(ARGS) >= 4 ? parse(Float64, ARGS[4]) : Inf
const MODE = length(ARGS) >= 5 ? ARGS[5] : "std"
const gpsl1 = GPSL1CA()

function main()
    channel = GNSSReceiver.spawn_signal_channel_thread(;
        T = Complex{Int16}, num_samples = CHUNK, num_antenna_channels = 1) do ch
        open(RAWFILE) do io
            raw8 = Vector{UInt8}(undef, 8 * CHUNK)
            n = 0
            while n * CHUNK / ustrip(Hz, FS) < MAX_SECONDS
                try
                    read!(io, raw8)          # exactly one chunk, or EOFError at the end
                catch e
                    e isa EOFError ? break : rethrow()
                end
                raw = reinterpret(Int16, raw8)
                buf = Matrix{Complex{Int16}}(undef, CHUNK, 1)
                @inbounds for k = 1:CHUNK
                    buf[k, 1] = Complex(raw[4k-3], raw[4k-2])   # antenna 1
                end
                put!(ch, buf)
                n += 1
            end
        end
    end
    @info "software replay" RAWFILE PLL_HZ MODE
    data = receive(
        channel,
        gpsl1,
        FS;
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
        write_rows(io, rows, nothing)
        if time() - last_print >= 5
            last_print = time()
            @info @sprintf("wall=%6.1f s  runtime=%6.1f s  locked[%s]", time() - t0,
                isempty(rows) ? NaN : rows[1].runtime, locked_summary(rows))
            flush(io)
            flush(stderr)
        end
    end
    close(io)
end

main()
