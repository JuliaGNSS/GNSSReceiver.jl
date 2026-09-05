# Offline acquisition on a raw M2SDR capture (sc16 2R2T words; antenna 0).
using Acquisition, GNSSSignals, Unitful, Printf, Statistics
using Unitful: Hz, ms
using FFTW
FFTW.set_num_threads(1)
const FS = 4e6Hz
file = ARGS[1]
offsets_s = length(ARGS) >= 2 ? parse.(Float64, split(ARGS[2], ",")) : [0.0, 2.0, 4.0, 6.0, 8.0]
systems = length(ARGS) >= 3 ? split(ARGS[3], ",") : ["gps"]
nc = 10; rounds = 5
nsamp = nc * 4000 * rounds
function read_ant0(file, offset_s, n)
    open(file) do io
        seek(io, round(Int, offset_s * 4e6) * 8)
        raw = read(io, n * 8)
        w = reinterpret(Int16, raw)
        [Complex{Int16}(w[4k-3], w[4k-2]) for k in 1:n]
    end
end
first = true
for sysname in systems
    sys, prns, cov = sysname == "gps" ? (GPSL1CA(), 1:32, 25_000.0Hz) :
                     sysname == "gal" ? (GalileoE1B(), 1:36, 25_000.0Hz) :
                     error("unknown system $sysname")
    println("=== $sysname  coherent $(nc) code periods, $(rounds) noncoherent rounds, ±$(cov)")
    tbl = Dict{Int,Vector{Any}}()
    for off in offsets_s
        sig = read_ant0(file, off, nsamp)
        t = @elapsed res = acquire(sys, sig, FS, prns; min_doppler_coverage = cov,
                                   num_coherently_integrated_code_periods = nc,
                                   num_noncoherent_accumulations = rounds)
        if first; println("fields: ", fieldnames(typeof(res[1]))); global first = false; end
        @printf("-- offset %.1f s (%.2f s)\n", off, t)
        for r in res
            push!(get!(tbl, Int(r.prn), []), (off, r))
        end
    end
    # Summary per PRN: median CN0, doppler spread, code-phase drift consistency
    println(@sprintf("%4s %7s %7s %9s %9s %11s %8s", "PRN", "CN0med", "CN0max", "dop_med", "dop_spread", "cp_drift_ch/s", "cp_resid"))
    rows = []
    for prn in sort(collect(keys(tbl)))
        v = tbl[prn]
        cn0 = [ustrip(r.CN0) for (_, r) in v]
        dop = [ustrip(Hz, r.carrier_doppler) for (_, r) in v]
        offs = [o for (o, _) in v]
        cps = [r.code_phase for (_, r) in v]
        # expected code phase drift ≈ doppler/1540 chips/s (mod code length); fit linear drift mod L
        L = sysname == "gps" ? 1023.0 : 4092.0
        exp_drift = median(dop) / (1575.42e6 / 1.023e6) * (L / 1023.0)
        # unwrap code phases against the expected drift
        pred = [cps[1] + exp_drift * (o - offs[1]) for o in offs]
        resid = [mod(c - p + L/2, L) - L/2 for (c, p) in zip(cps, pred)]
        push!(rows, (median(cn0), prn, maximum(cn0), median(dop), maximum(dop) - minimum(dop), exp_drift, maximum(abs, resid)))
    end
    sort!(rows; rev = true)
    for (cm, prn, cx, dm, ds, ed, rr) in rows
        @printf("%4d %7.1f %7.1f %9.0f %9.0f %11.3f %8.2f\n", prn, cm, cx, dm, ds, ed, rr)
    end
end
