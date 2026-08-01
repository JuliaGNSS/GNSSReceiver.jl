# Control experiment for the hardware-correlator work (issue #107): the pure
# SOFTWARE receiver (CPU correlation) on the same M2SDR raw stream, same
# antenna, same sky. It answers the one question the hardware run cannot ask of
# itself — is a bit stream that never validates an ephemeris the correlator's
# fault or the front end's?
#
# If this decodes and holds TOW, the RF/LO chain, acquisition, the decoder and
# PVT are all fine and the fault is in the FPGA correlation path. If it also
# never bit-syncs, the problem is upstream of the correlators.
#
# Run it right after (or before) `hardware_correlator_position_fix.jl`, on the
# same board, so the sky and the antenna are the same. It streams DMA0 itself,
# so claim the board first — two `m2sdr_record` instances split the buffers and
# both receivers then see garbage.
#
# Result on 2026-08-01, against a hardware-correlator run in the same hour that
# validated nothing on 4-5 satellites over 200 s:
#
#   t= 91.3s  dec[20:218b*]                    <- PRN 20 ephemeris validated, TOW held
#   t=207.6s  dec[11:33b*  20:333b*  30:33b*]  <- PRN 11 and 30 too
#
# (`nbits` leaves -1 only once GPS subframes 1-3 have all decoded with every
# word's parity closing and IODC/IODE agree, so a non-negative value here means
# a full validated ephemeris, and `*` means TOW is available.)
#
# Usage: julia -t 8 --project=. hardware_correlator_software_control.jl [MAX_SECONDS]

using Printf
using Statistics: mean
using Unitful
using Unitful: Hz, ms, ustrip
using Geodesy: LLAfromECEF, wgs84
using GNSSSignals: GPSL1CA
using SignalChannels: SignalChannel
using GNSSReceiver

const FS_HZ = 4e6
const FS = 4e6Hz
const CHUNK = 4000
const MAX_SECONDS = length(ARGS) >= 1 ? parse(Float64, ARGS[1]) : 300.0
const gpsl1 = GPSL1CA()

flusher = Timer(_ -> (flush(stderr); flush(stdout)), 1; interval = 2)

run(ignorestatus(`pkill -x m2sdr_record`)); sleep(0.3)
let warm = open(`head -c 65536 /dev/zero`, "r")
    read!(warm, Vector{UInt8}(undef, 65536))
    close(warm)
end
recorder = open(`m2sdr_record -q - 0`, "r")
channel = SignalChannel{Complex{Int16},1}(CHUNK, 4000)
reader = Threads.@spawn :interactive begin
    pool = [Matrix{Complex{Int16}}(undef, CHUNK, 1) for _ = 1:4002]
    raw = Vector{UInt8}(undef, CHUNK * 8)
    idx = 1
    try
        while isopen(channel)
            read!(recorder, raw)
            words = reinterpret(Int16, raw)
            buf = pool[idx]
            @inbounds for k = 1:CHUNK
                buf[k, 1] = Complex(words[4k-3], words[4k-2])
            end
            put!(channel, buf)
            idx = mod1(idx + 1, 4002)
        end
    catch e
        e isa EOFError || e isa InvalidStateException || rethrow()
    finally
        close(channel)
        close(recorder)
    end
end
Base.errormonitor(reader)

my_extract(state) = (
    data = GNSSReceiver.default_data_of_interest(state),
    decode = [
        (prn = rss.prn,
         nbits = something(rss.decoder.num_bits_after_valid_syncro_sequence, -1),
         tow = Int64(something(rss.decoder.data.TOW, -1)))
        for dict in values(state.receiver_sat_states) for rss in dict
    ],
)

data = GNSSReceiver.receive(
    channel,
    gpsl1,
    FS;
    acq_min_doppler_coverage = 25_000.0Hz,
    acq_coherent_integration_time = 10ms,
    acquire_every = 60u"s",
    extract = my_extract,
)

t0 = time()
last_print = 0.0
first_fix = nothing
for payload in data
    global last_print, first_fix
    d = payload.data
    t = time() - t0
    if d.pvt.time !== nothing && isnothing(first_fix)
        first_fix = d.pvt
        lla = LLAfromECEF(wgs84)(d.pvt.position)
        @info @sprintf("*** SOFTWARE FIX after %.1f s: %.6f° %.6f° %.1f m (%d sats) ***",
                       t, lla.lat, lla.lon, lla.alt, length(d.pvt.sats))
    end
    if t - last_print >= 5.0
        last_print = t
        sats = join(
            [@sprintf("%d:%.0f", k[2], 10log10(ustrip(Hz, Unitful.linear(v.cn0))))
             for (k, v) in pairs(d.sat_data)], " ")
        dec = join(
            [@sprintf("%d:%db%s", e.prn, e.nbits, e.tow >= 0 ? "*" : "")
             for e in payload.decode], " ")
        @info @sprintf("t=%5.1fs cn0[%s] dec[%s] | %s", t, sats, dec,
                       isnothing(first_fix) ? "no fix" : "FIXED")
    end
    (!isnothing(first_fix) && t > MAX_SECONDS) && break
    t > MAX_SECONDS && break
end
close(channel)
kill(recorder)
@info isnothing(first_fix) ? "software receiver: NO fix/decode" : "software receiver: SUCCESS"
