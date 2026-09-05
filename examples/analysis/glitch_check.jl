using Acquisition, GNSSSignals, Unitful, Printf, Statistics
using Unitful: Hz; using FFTW; FFTW.set_num_threads(1)
ant0 = ARGS[1]; prn = parse(Int, ARGS[2]); step_ms = parse(Int, ARGS[3]); dur_s = parse(Float64, ARGS[4])
FS = 4e6Hz
sig = Vector{Complex{Int16}}(undef, filesize(ant0) ÷ 4); read!(ant0, sig)
blk = 10 * 4000
res0 = acquire(GPSL1CA(), @view(sig[1:blk*5]), FS, prn; min_doppler_coverage = 25_000.0Hz,
               num_coherently_integrated_code_periods = 10, num_noncoherent_accumulations = 5)
dop0 = ustrip(Hz, res0.carrier_doppler); cp0 = res0.code_phase
drift = dop0 / 1540.0   # chips/s
@printf("ref: dop %.0f cp %.2f drift %.3f chips/s\n", dop0, cp0, drift)
starts = 0:(step_ms*4000):(round(Int, dur_s*4e6) - blk)
prev = NaN; bad = 0
for s in starts
    r = acquire(GPSL1CA(), @view(sig[s+1:s+blk]), FS, prn; min_doppler_coverage = 2_000.0Hz,
                interm_freq = dop0 * Hz, num_coherently_integrated_code_periods = 10, num_noncoherent_accumulations = 1)
    t = s / 4e6
    pred = mod(cp0 + drift * t, 1023.0)
    resid = mod(r.code_phase - pred + 511.5, 1023.0) - 511.5
    pn = r.peak_to_noise_ratio
    flag = abs(resid) > 0.5 || pn < 4 ? "  <-- " : ""
    global bad += abs(resid) > 0.5
    (s % (step_ms*4000*10) == 0 || flag != "") && @printf("t=%6.3f cp %8.2f resid %7.2f dop %6.0f CN0 %5.1f p/n %5.1f%s\n", t, r.code_phase, resid, ustrip(Hz, r.carrier_doppler) + dop0, ustrip(r.CN0), pn, flag)
end
println("segments with |resid|>0.5 chips: $bad / $(length(starts))")
