# Self-test: synthetic GPS L1 C/A at 4 MHz, Complex{Int16}, acquire then track with the same code path as direct_track.jl
using Acquisition, Tracking, GNSSSignals, Unitful, Printf, Random
using Unitful: Hz; using FFTW; FFTW.set_num_threads(1)
FS = 4e6Hz; fs = 4e6; prn = 14; dop = 3390.0; cp0 = 288.5; if_hz = -5090.0
N = round(Int, 3 * fs)   # 3 s
rng = Xoshiro(1)
code_freq = 1.023e6 * (1 + dop / 1575.42e6)
t = (0:N-1) ./ fs
code = get_code.(GPSL1CA(), cp0 .+ code_freq .* t, prn)
carrier = cis.(2π .* (dop + if_hz) .* t)
amp = 30.0  # amplitude vs noise std 86 → CN0 ≈ 10log10(amp^2/(2σ²)*fs)... 
noise = 86 .* (randn(rng, N) .+ im .* randn(rng, N))
sig = Complex{Int16}.(round.(Int16, real.(amp .* code .* carrier .+ noise)), round.(Int16, imag.(amp .* code .* carrier .+ noise)))
@printf("synthetic: expected CN0 ≈ %.1f dBHz\n", 10log10(amp^2 / (2 * 86^2) * fs))
res = acquire(GPSL1CA(), @view(sig[1:200_000]), FS, prn; min_doppler_coverage = 25_000.0Hz,
              num_coherently_integrated_code_periods = 10, num_noncoherent_accumulations = 5)
@printf("acq: CN0 %.1f dop %.0f cp %.2f\n", ustrip(res.CN0), ustrip(Hz, res.carrier_doppler), res.code_phase)
ts = TrackState(; signal = GPSL1CA())
ts = add_satellite!(ts; prn, code_phase = res.code_phase, carrier_doppler = res.carrier_doppler - if_hz * Hz)
dc = Int16ThreadedDownconvertAndCorrelator(2^11)
chunk = 8000
for k in 1:(N ÷ chunk)
    track!(@view(sig[(k-1)*chunk+1:k*chunk]), ts, FS; downconvert_and_correlator = dc, intermediate_frequency = if_hz * Hz)
    if k % 125 == 0
        s = get_sat_state(ts, :default, prn)
        @printf("t=%4.2fs cn0 %5.1f dop %7.1f cp %8.3f (true cp %8.3f)\n", k*chunk/fs, ustrip(Tracking.estimate_cn0(s, 1)),
                ustrip(Hz, get_carrier_doppler(s)), get_code_phase(s), mod(cp0 + code_freq * k*chunk/fs, 1023))
    end
end
