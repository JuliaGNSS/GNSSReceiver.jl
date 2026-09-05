# What does the first call to each pipeline stage cost on this machine?
#
# A live receiver pays every one of these after the samples have started
# flowing — the first scan, the first tracked satellite, the first decoded
# subframe, the first fix — and each stall holds every tracking loop open
# (issue #107). Run it in a fresh session; the numbers are compilation, not
# work, and the second call of each is microseconds.
#
# Measured on a Jetson AGX Orin before and after the packages gained their
# PrecompileTools workloads: 55.6 s of compilation on the critical path
# became 1.3 s.
#
#   julia -t 6,3 --project=. first_call_cost.jl

# First-call cost of each stage on this machine, in one fresh session.
t_load = @elapsed using GNSSReceiver, GNSSSignals, Tracking, Acquisition, GNSSDecoder,
    PositionVelocityTime, Unitful, Random, FFTW
using Unitful: Hz
using GNSSSignals: gen_code, get_code_center_frequency_ratio, get_code_frequency
FFTW.set_num_threads(1)
println("versions: Acquisition ", pkgversion(Acquisition), "  Tracking ", pkgversion(Tracking),
        "  GNSSDecoder ", pkgversion(GNSSDecoder), "  PVT ", pkgversion(PositionVelocityTime),
        "  GNSSReceiver ", pkgversion(GNSSReceiver), "  GNSSSignals ", pkgversion(GNSSSignals))
println("load                    ", round(t_load; digits = 2), " s")

gpsl1 = GPSL1CA(); fs = 4e6Hz
rng = Xoshiro(1)
sig16 = Complex{Int16}.(round.(randn(rng, ComplexF32, 10 * 5 * 4000) .* 512))
t = @elapsed acquire(gpsl1, sig16, fs, [1, 2]; min_doppler_coverage = 25_000.0Hz,
                     num_coherently_integrated_code_periods = 10, num_noncoherent_accumulations = 5)
println("first acquire           ", round(t; digits = 2), " s")

code_frequency = 200.0Hz * get_code_center_frequency_ratio(gpsl1) + get_code_frequency(gpsl1)
sig = ComplexF32.(cis.(2π .* 200.0 .* (0:3999) ./ 4e6) .* gen_code(4000, gpsl1, 1, fs, code_frequency, 0.0))
ts = TrackState(gpsl1, [TrackedSat(gpsl1, 1, 0.0, 180.0Hz)])
t = @elapsed track!(sig, ts, fs)
println("first track!            ", round(t; digits = 2), " s")

sym = Float32[isodd(k >> 3) ? 1.0f0 : -1.0f0 for k = 1:8000]
t = @elapsed decode(GNSSDecoderState(gpsl1, 25), sym, length(sym))
println("first decode            ", round(t; digits = 2), " s")

# The decoder's *sync* path, which the line above never reaches: a stream with
# the TLM preamble on the 300-bit subframe grid runs `try_sync`,
# `read_tlm_and_how_words` and its `can_decode_word` closures. A live receiver
# pays this at the first bit sync, ~30 s in, inside the fold.
synced = let preamble = Bool[1, 0, 0, 0, 1, 0, 1, 1], bits = Bool[]
    for _ = 1:3
        append!(bits, preamble)
        append!(bits, [isodd(k >> 2) ⊻ isodd(k >> 5) for k = 1:(300-length(preamble))])
    end
    Float32[b ? 1.0f0 : -1.0f0 for b in bits]
end
t = @elapsed decode(GNSSDecoderState(gpsl1, 7), synced, length(synced))
println("first decode (bit sync) ", round(t; digits = 2), " s")

ch = GNSSReceiver.spawn_signal_channel_thread(; T = Complex{Int16}, num_samples = 4000, num_antenna_channels = 1) do c
    foreach(1:12) do _
        put!(c, Complex{Int16}.(round.(randn(ComplexF32, 4000, 1) .* 512)))
    end
end
t = @elapsed begin
    data = receive(ch, gpsl1, fs; max_meas = 2^12, acquire_every = 4u"ms", pvt_update_interval = 4u"ms")
    collect_data(data)
end
println("first receive           ", round(t; digits = 2), " s")

# The hardware-correlator receiver, driven by a device defined *here* — outside
# GNSSReceiver, exactly as a vendor package's is. That is the whole point of the
# measurement: the pipeline is specialised on its correlator source, and if
# anything about it depended on the device's type (or bound the device
# interface's method table), the package's precompiled copy would be unusable
# and the 1.7 s of compilation would land inside the first live chunk, with the
# device's dump ring unattended throughout (issue #107).
using StaticArrays: SVector
using PipeChannels: PipeChannel
using SignalChannels: SignalChannel
using Tracking: CorrelatorOutput, EarlyPromptLateCorrelator
using GNSSReceiver: CorrelatorDump, NCOUpdate, AbstractHardwareCorrelatorSDR, epoch_strobe

epl(late, prompt, early) =
    EarlyPromptLateCorrelator(SVector{3,ComplexF64}(late, prompt, early), 1)
const EPL = typeof(epl(0, 0, 0))

struct VendorSDR <: AbstractHardwareCorrelatorSDR
    raw::SignalChannel{Complex{Int16},1}
    dumps::PipeChannel{CorrelatorDump{EPL}}
    ncos::PipeChannel{NCOUpdate}
end
GNSSReceiver.raw_sample_channel(sdr::VendorSDR) = sdr.raw
GNSSReceiver.correlator_dump_channel(sdr::VendorSDR) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::VendorSDR) = sdr.ncos
GNSSReceiver.num_hardware_channels(::VendorSDR) = 5
GNSSReceiver.assign_channel!(::VendorSDR, args...; kwargs...) = nothing
GNSSReceiver.release_channel!(::VendorSDR, hw_channel) = nothing

hw = VendorSDR(
    SignalChannel{Complex{Int16},1}(4000, 16),
    PipeChannel{CorrelatorDump{EPL}}(1 << 14),
    PipeChannel{NCOUpdate}(1 << 10),
)
feeder = Threads.@spawn begin
    chunk = Complex{Int16}.(round.(randn(Xoshiro(2), ComplexF32, 4000, 1) .* 512))
    try
        for i = 1:12
            base = i * 4000
            batch = [CorrelatorDump(ch, ch,
                                    CorrelatorOutput(epl(400 + 0im, 1000 + 10im, 400 + 0im),
                                                     4000, base + ch),
                                    mod(0.001 * base, 1023.0)) for ch = 1:2]
            push!(batch, epoch_strobe(epl(0, 0, 0), base))
            put!(hw.dumps, batch)
            put!(hw.raw, chunk)
            while Base.n_avail(hw.ncos) > 0
                take!(hw.ncos)
            end
        end
    finally
        close(hw.raw)
        close(hw.dumps)
    end
end
t = @elapsed begin
    collect_data(receive(hw, gpsl1, fs; max_meas = 2^11, acquire_every = 4u"ms",
                         pvt_update_interval = 4u"ms"))
    wait(feeder)
end
println("first hardware receive  ", round(t; digits = 2), " s")
