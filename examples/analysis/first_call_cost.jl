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
