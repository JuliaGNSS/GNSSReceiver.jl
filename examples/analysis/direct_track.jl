using Acquisition, Tracking, GNSSSignals, Unitful, Printf, Statistics
using Unitful: Hz, ms
using FFTW; FFTW.set_num_threads(1)
ant0 = ARGS[1]; prns = parse.(Int, split(ARGS[2], ",")); if_hz = parse(Float64, ARGS[3])
FS = 4e6Hz
n = filesize(ant0) ÷ 4
sig = Vector{Complex{Int16}}(undef, n); read!(ant0, sig)
acq_len = 10 * 4000 * 5
res = acquire(GPSL1CA(), @view(sig[1:acq_len]), FS, prns; min_doppler_coverage = 25_000.0Hz,
              num_coherently_integrated_code_periods = 10, num_noncoherent_accumulations = 5)
ts = TrackState(; signal = GPSL1CA())
for r in res
    @printf("PRN %2d acq: CN0 %.1f dop %.0f Hz cp %.2f\n", r.prn, ustrip(r.CN0), ustrip(Hz, r.carrier_doppler), r.code_phase)
    global ts = add_satellite!(ts; prn = r.prn, code_phase = r.code_phase,
                               carrier_doppler = r.carrier_doppler - if_hz * Hz)
end
dc = Int16ThreadedDownconvertAndCorrelator(2^11)
chunk = 8000
group = first(keys(Tracking.get_sat_states(ts)))  # group key
nchunks = n ÷ chunk
for k in 1:nchunks
    buf = @view sig[(k-1)*chunk+1:k*chunk]
    track!(buf, ts, FS; downconvert_and_correlator = dc, intermediate_frequency = if_hz * Hz)
    if k % 250 == 0
        parts = String[]
        for r in res
            s = get_sat_state(ts, :default, r.prn)
            cn0 = ustrip(Tracking.estimate_cn0(s, 1))
            push!(parts, @sprintf("%2d:%4.1f dop%6.0f cp%7.2f", r.prn, cn0, ustrip(Hz, get_carrier_doppler(s)), get_code_phase(s)))
        end
        @printf("t=%4.1fs | %s\n", k * chunk / 4e6, join(parts, " | "))
    end
end
