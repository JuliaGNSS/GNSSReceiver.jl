# What does the hardware correlator's record stream look like, satellite by
# satellite, and where between the FPGA and `decode` do the navigation bits go
# wrong?
#
# Reads the bit log written by `../hardware_correlator_position_fix.jl` under
# `HWFIX_BITLOG` (`B`/`P`/`D`/`A`/`C`/`L` lines) and its `.records` companion
# (`R` lines: every 1 ms hardware record as it came off the wire) and answers,
# per PRN:
#
#   1. Is the record stream intact? Record lengths, forward gaps (with the
#      channel-table event nearest each), overlaps.
#   2. Is there a signal, and is the carrier phase-locked? `|P|` against the
#      Early/Late taps (noise gives a ratio of 1), and the phase step between
#      consecutive 1 ms prompts: a locked carrier flips sign only at the ~50 %
#      of bit edges where the data changes (≈2.5 % of pairs), a Costas alias
#      500 Hz off flips *every* pair, and an unlocked carrier spreads the steps
#      around the circle.
#   3. Re-fold the raw 1 ms prompts into 20 ms bits at the best edge, and walk
#      the GPS word parity on (a) those bits and (b) the soft bits Tracking
#      handed `decode`, re-searching the alignment when parity stops closing so
#      an inserted or dropped bit shows up as a signed slip rather than as a
#      ~50 % word error rate.
#
# usage: julia records.jl <bitstream.log> [prn …]

using Printf
using Statistics: mean, median

const FS = 4e6
const BLOCKS_PER_BIT = 20
const PREAMBLE = [1, 0, 0, 0, 1, 0, 1, 1]

struct Rec
    ch::Int
    prn::Int
    sidx::Int64
    n::Int
    cp::Float64
    L::ComplexF64
    P::ComplexF64
    E::ComplexF64
end

function read_records(path)
    recs = Rec[]
    for line in eachline(path)
        f = split(line)
        (length(f) < 12 || f[1] != "R") && continue
        push!(recs, Rec(
            parse(Int, f[2]), parse(Int, f[3]), parse(Int64, f[4]), parse(Int, f[5]),
            parse(Float64, f[6]),
            complex(parse(Float64, f[7]), parse(Float64, f[8])),
            complex(parse(Float64, f[9]), parse(Float64, f[10])),
            complex(parse(Float64, f[11]), parse(Float64, f[12])),
        ))
    end
    recs
end

function read_bitlog(path)
    softbits = Dict{Int,Vector{Tuple{Int64,Float32}}}()   # prn → (boundary, bit)
    dlines = Dict{Int,Vector{Vector{String}}}()
    plines = Dict{Int,Vector{Vector{String}}}()
    alines = Vector{Vector{String}}()
    clines = Vector{Vector{String}}()
    llines = Vector{Vector{String}}()
    for line in eachline(path)
        f = split(line)
        isempty(f) && continue
        if f[1] == "B"
            prn = parse(Int, f[2]); b = parse(Int64, f[3])
            v = get!(softbits, prn, Tuple{Int64,Float32}[])
            for s in f[4:end]
                push!(v, (b, parse(Float32, s)))
            end
        elseif f[1] == "D"
            push!(get!(dlines, parse(Int, f[2]), Vector{String}[]), f)
        elseif f[1] == "P"
            push!(get!(plines, parse(Int, f[2]), Vector{String}[]), f)
        elseif f[1] == "A"
            push!(alines, f)
        elseif f[1] == "C"
            push!(clines, f)
        elseif f[1] == "L"
            push!(llines, f)
        end
    end
    (; softbits, dlines, plines, alines, clines, llines)
end

# ── GPS LNAV parity (IS-GPS-200 20.3.5.2) ─────────────────────────────────────
function parity_ok(w, d29, d30)
    d = zeros(Int, 24)
    for i = 1:24
        d[i] = w[i] ⊻ d30
    end
    D25 = d29 ⊻ d[1] ⊻ d[2] ⊻ d[3] ⊻ d[5] ⊻ d[6] ⊻ d[10] ⊻ d[11] ⊻ d[12] ⊻ d[13] ⊻ d[14] ⊻ d[17] ⊻ d[18] ⊻ d[20] ⊻ d[23]
    D26 = d30 ⊻ d[2] ⊻ d[3] ⊻ d[4] ⊻ d[6] ⊻ d[7] ⊻ d[11] ⊻ d[12] ⊻ d[13] ⊻ d[14] ⊻ d[15] ⊻ d[18] ⊻ d[19] ⊻ d[21] ⊻ d[24]
    D27 = d29 ⊻ d[1] ⊻ d[3] ⊻ d[4] ⊻ d[5] ⊻ d[7] ⊻ d[8] ⊻ d[12] ⊻ d[13] ⊻ d[14] ⊻ d[15] ⊻ d[16] ⊻ d[19] ⊻ d[20] ⊻ d[22]
    D28 = d30 ⊻ d[2] ⊻ d[4] ⊻ d[5] ⊻ d[6] ⊻ d[8] ⊻ d[9] ⊻ d[13] ⊻ d[14] ⊻ d[15] ⊻ d[16] ⊻ d[17] ⊻ d[20] ⊻ d[21] ⊻ d[23]
    D29 = d30 ⊻ d[1] ⊻ d[3] ⊻ d[5] ⊻ d[6] ⊻ d[7] ⊻ d[9] ⊻ d[10] ⊻ d[14] ⊻ d[15] ⊻ d[16] ⊻ d[17] ⊻ d[18] ⊻ d[21] ⊻ d[22] ⊻ d[24]
    D30 = d29 ⊻ d[3] ⊻ d[5] ⊻ d[6] ⊻ d[8] ⊻ d[9] ⊻ d[10] ⊻ d[11] ⊻ d[13] ⊻ d[15] ⊻ d[19] ⊻ d[22] ⊻ d[23] ⊻ d[24]
    ([D25, D26, D27, D28, D29, D30] == w[25:30]), D29, D30
end

# Consecutive words closing parity from 0-based bit `start`, seeding D29*/D30*
# from whichever pair makes the first word close.
function run_length(bits, start)
    start + 30 > length(bits) && return 0
    d29, d30, seeded = 0, 0, false
    for a = 0:1, b = 0:1
        ok, _, _ = parity_ok(view(bits, start+1:start+30), a, b)
        ok && ((d29, d30, seeded) = (a, b, true); break)
    end
    seeded || return 0
    n, k = 0, start
    while k + 30 <= length(bits)
        ok, d29, d30 = parity_ok(view(bits, k+1:k+30), d29, d30)
        ok || break
        n += 1
        k += 30
    end
    n
end

# Walk the word grid; when parity breaks, look ±max_slip bits for where ≥3 clean
# words resume. A resume a whole number of words later is not a slip — the grid
# is intact and those words simply had bit errors — so shifts ≡ 0 (mod 30) are
# reported as `bad_words`, and only the residual shift as a slip. Returns
# (clean_words, total_words, slips, bad_words, error_bits): `error_bits` are the
# 0-based bit positions where each failing run began.
function slip_walk(bits; max_slip = 45)
    n = length(bits)
    n < 120 && return 0, n ÷ 30, Int[], 0, Int[]
    pos = 0
    while pos < min(n - 120, 600)
        run_length(bits, pos) >= 3 && break
        pos += 1
    end
    pos >= min(n - 120, 600) && return 0, n ÷ 30, Int[], 0, Int[]
    clean, slips, bad, err_at = 0, Int[], 0, Int[]
    while true
        r = run_length(bits, pos)
        clean += r
        endpos = pos + 30r
        best = nothing
        for shift in sort(collect(-max_slip:max_slip); by = abs)
            cand = endpos + shift
            (cand < 0 || cand + 120 > n) && continue
            if run_length(bits, cand) >= 3
                best = (shift, cand)
                break
            end
        end
        isnothing(best) && break
        shift = best[1]
        push!(err_at, endpos)
        words, residual = divrem(shift, 30, RoundNearest)
        bad += max(0, words)
        residual == 0 || push!(slips, residual)
        pos = best[2]
        pos + 120 > n && break
    end
    clean, n ÷ 30, slips, bad, err_at
end

# TLM preambles (either polarity) whose TLM and HOW words both close parity;
# returns (bit index 0-based, TOW count, subframe id).
function subframes(bits)
    out = Tuple{Int,Int,Int}[]
    for i = 0:(length(bits)-61)
        w = view(bits, i+1:i+60)
        pol = w[1:8] == PREAMBLE ? 0 : (w[1:8] == 1 .- PREAMBLE ? 1 : -1)
        pol < 0 && continue
        b = pol == 1 ? 1 .- w : w
        ok1, d29, d30 = parity_ok(b[1:30], 0, 0)
        ok1 || ((ok1, d29, d30) = parity_ok(b[1:30], 0, 1))
        ok1 || continue
        ok2, _, _ = parity_ok(b[31:60], d29, d30)
        ok2 || continue
        how = b[31:60] .⊻ d30
        tow = foldl((acc, x) -> 2acc + x, how[1:17]; init = 0)
        sfid = foldl((acc, x) -> 2acc + x, how[20:22]; init = 0)
        push!(out, (i, tow, sfid))
    end
    out
end

# ── record-stream analysis ────────────────────────────────────────────────────
# Split one channel's records into stretches with no forward gap and no
# assignment change, in time order.
function contiguous_stretches(recs)
    stretches = Vector{Vector{Rec}}()
    cur = Rec[]
    for r in recs
        if !isempty(cur) && (r.sidx - r.n != cur[end].sidx || r.prn != cur[end].prn)
            push!(stretches, cur)
            cur = Rec[]
        end
        push!(cur, r)
    end
    isempty(cur) || push!(stretches, cur)
    stretches
end

# Bit-edge search on 1 ms prompts: energy of the coherent 20-block sums per
# edge hypothesis. Returns (best phase, coherence at best, energy ratio to the
# runner-up).
function bit_edge(P)
    N = length(P)
    N < 3BLOCKS_PER_BIT && return 0, NaN, NaN
    energy = zeros(BLOCKS_PER_BIT)
    coh = zeros(BLOCKS_PER_BIT)
    for ph = 0:BLOCKS_PER_BIT-1
        nb = (N - ph) ÷ BLOCKS_PER_BIT
        e, c = 0.0, 0.0
        for j = 1:nb
            seg = view(P, ph+(j-1)*BLOCKS_PER_BIT+1:ph+j*BLOCKS_PER_BIT)
            s = sum(seg)
            e += abs2(s)
            c += abs(s) / sum(abs, seg)
        end
        energy[ph+1] = e / nb
        coh[ph+1] = c / nb
    end
    order = sortperm(energy; rev = true)
    order[1] - 1, coh[order[1]], energy[order[1]] / energy[order[2]]
end

fold_bits(P, ph) = [
    real(sum(@view P[ph+(k-1)*BLOCKS_PER_BIT+1:ph+k*BLOCKS_PER_BIT])) for
    k = 1:((length(P)-ph)÷BLOCKS_PER_BIT)
]

# Phase-step statistics between consecutive 1 ms prompts of one stretch.
function phase_steps(P)
    z = [P[k] * conj(P[k-1]) for k = 2:length(P)]
    isempty(z) && return (flip = NaN, rot_hz = NaN, hist = zeros(Int, 8))
    flip = mean(real.(z) .< 0)
    # Squared step removes the data sign; its mean angle is 2·Δφ, so the
    # residual carrier offset follows modulo 500 Hz (a 500 Hz alias reads 0
    # here — the flip fraction is what exposes it).
    rot_hz = angle(sum(z .^ 2)) / 2 / (2π * 1e-3)
    hist = zeros(Int, 8)
    for x in z
        hist[clamp(floor(Int, (angle(x) + π) / (2π) * 8) + 1, 1, 8)] += 1
    end
    (; flip, rot_hz, hist)
end

t_of(sidx, origin) = (sidx - origin) / FS

function analyse_prn(prn, recs_by_ch, origin, log)
    println("\n", "="^78)
    println("PRN $prn")
    println("="^78)
    # Every record on any channel while that channel was assigned this PRN —
    # except the noise channel, whose decoy PRN rotates through the family and
    # would otherwise contribute open-loop records to whichever satellite it
    # happens to be replicating.
    noise_channels = Set(parse(Int, a[2]) for a in log.alines if parse(Int, a[4]) < 0)
    mine = Rec[]
    for (ch, recs) in recs_by_ch
        ch in noise_channels && continue
        for r in recs
            r.prn == prn && push!(mine, r)
        end
    end
    sort!(mine; by = r -> r.sidx)
    if isempty(mine)
        println("  no hardware records")
    else
        chans = unique(r.ch for r in mine)
        @printf("  %d records on channel(s) %s, t = %.1f .. %.1f s\n", length(mine),
                join(chans, ","), t_of(mine[1].sidx, origin), t_of(mine[end].sidx, origin))
        lens = [r.n for r in mine]
        hist = Dict{Int,Int}()
        for n in lens
            hist[n] = get(hist, n, 0) + 1
        end
        println("  record lengths (samples: count): ",
                join(["$k:$v" for (k, v) in sort(collect(hist); by = x -> -x[2])][1:min(end, 8)], "  "))
        # Gaps / overlaps, per channel.
        for ch in chans
            recs = filter(r -> r.ch == ch, mine)
            for k = 2:length(recs)
                gap = recs[k].sidx - recs[k].n - recs[k-1].sidx
                gap == 0 && continue
                # Nearest channel-table event.
                near = ""
                for a in log.alines
                    b = parse(Int64, a[5])
                    if abs(b - recs[k].sidx) < 2FS
                        near = @sprintf(" [A ch%s %s→%s at t=%.1f]", a[2], a[3], a[4], t_of(b, origin))
                    end
                end
                @printf("  %s ch%d t=%.2f: %s of %d samples (%.2f blocks); record after it is %d samples long, code_phase %.3f → %.3f%s\n",
                        gap > 0 ? "GAP    " : "OVERLAP", ch, t_of(recs[k].sidx, origin),
                        gap > 0 ? "hole" : "overlap", abs(gap), gap / 4000, recs[k].n,
                        recs[k-1].cp, recs[k].cp, near)
            end
        end
        # Signal presence and phase behaviour over time, in 10 s bins.
        println("\n  time-binned prompt statistics (10 s):")
        println("     t[s]   recs   |P|/|EL|   |P|      flip%   rot[Hz]   edge  coh   E/Ebest2   phase-step histogram (−π..π, 8 bins, %)")
        t0 = t_of(mine[1].sidx, origin)
        tend = t_of(mine[end].sidx, origin)
        tb = floor(t0 / 10) * 10
        while tb < tend
            sel = filter(r -> tb <= t_of(r.sidx, origin) < tb + 10, mine)
            if length(sel) >= 40
                P = [r.P for r in sel]
                el = mean(vcat(abs.([r.E for r in sel]), abs.([r.L for r in sel])))
                # Phase steps only inside contiguous stretches.
                steps = ComplexF64[]
                for st in contiguous_stretches(sel)
                    for k = 2:length(st)
                        push!(steps, st[k].P * conj(st[k-1].P))
                    end
                end
                flip = isempty(steps) ? NaN : mean(real.(steps) .< 0)
                rot = isempty(steps) ? NaN : angle(sum(steps .^ 2)) / 2 / (2π * 1e-3)
                hist = zeros(Int, 8)
                for x in steps
                    hist[clamp(floor(Int, (angle(x) + π) / (2π) * 8) + 1, 1, 8)] += 1
                end
                # Bit edge on the longest contiguous stretch in the bin.
                longest = argmax(length, contiguous_stretches(sel))
                ph, coh, ratio = bit_edge([r.P for r in longest])
                @printf("  %7.0f  %5d   %6.2f   %8.0f   %5.1f   %7.1f   %3d   %.2f   %5.2f     %s\n",
                        tb, length(sel), mean(abs, P) / el, mean(abs, P), 100flip, rot,
                        ph, coh, ratio,
                        join([@sprintf("%2.0f", 100h / max(1, length(steps))) for h in hist], " "))
            end
            tb += 10
        end
        # Offline bits from the raw prompts: longest contiguous stretch.
        stretches = sort(contiguous_stretches(mine); by = length, rev = true)
        st = stretches[1]
        P = [r.P for r in st]
        ph, coh, ratio = bit_edge(P)
        bits = fold_bits(P, ph)
        hard = [b > 0 ? 1 : 0 for b in bits]
        @printf("\n  offline re-fold of the longest gap-free stretch (%d records, t=%.1f..%.1f s): edge %d, coherence %.2f, %d bits\n",
                length(st), t_of(st[1].sidx, origin), t_of(st[end].sidx, origin), ph, coh, length(hard))
        sf = subframes(hard)
        if isempty(sf)
            println("    no TLM+HOW pair closes parity")
        else
            for (i, tow, id) in sf[1:min(end, 8)]
                @printf("    preamble at bit %5d  TOW-count %6d  subframe %d\n", i, tow, id)
            end
            length(sf) > 8 && println("    … $(length(sf) - 8) more")
        end
        clean, total, slips, bad, err_at = slip_walk(hard)
        @printf("    parity walk: %d of %d words clean, %d words with bit errors, %d slips%s\n",
                clean, total, bad, length(slips), isempty(slips) ? "" : " (" * join(slips, ",") * ")")
        bit_t0 = t_of(st[1].sidx, origin) + (ph + BLOCKS_PER_BIT) / 1000
        isempty(err_at) || println("    error runs begin at t = ",
                join([@sprintf("%.1f", bit_t0 + e / 50) for e in err_at], " "), " s")
        # Phase of the 20 ms bit sums: for a phase-locked carrier the data sits
        # on the real axis, so |Im| > |Re| means that bit was integrated more
        # than 45° off — the direct signature of a phase-unlocked loop, and the
        # thing that turns a strong bit into a wrong one.
        sums = [sum(@view P[ph+(k-1)*BLOCKS_PER_BIT+1:ph+k*BLOCKS_PER_BIT]) for k = 1:length(bits)]
        println("    bit-sum phase per 10 s: t, bits, % past 45°, % past 90° (inverted), median |phase| deg")
        k0 = 1
        while k0 <= length(sums)
            k1 = min(length(sums), k0 + 499)
            seg = @view sums[k0:k1]
            ang = [abs(rem(angle(z), π, RoundNearest)) for z in seg]
            @printf("      %7.1f  %4d   %5.1f   %5.1f   %5.1f\n", bit_t0 + (k0 - 1) / 50, length(seg),
                    100mean(ang .> π / 4), 100mean(abs.(angle.(seg)) .> π / 2), rad2deg(median(ang)))
            k0 = k1 + 1
        end
    end
    # Tracking's own soft bits.
    sb = get(log.softbits, prn, Tuple{Int64,Float32}[])
    if isempty(sb)
        println("\n  Tracking handed decode no soft bits for this PRN")
    else
        hard = [b > 0 ? 1 : 0 for (_, b) in sb]
        @printf("\n  Tracking's soft bits: %d, t=%.1f..%.1f s (%.1f bits/s over that span)\n",
                length(hard), t_of(sb[1][1], origin), t_of(sb[end][1], origin),
                length(hard) / max(1e-9, t_of(sb[end][1], origin) - t_of(sb[1][1], origin)))
        # Bits per chunk: more than one per 20 ms boundary step is a burst.
        sf = subframes(hard)
        if isempty(sf)
            println("    no TLM+HOW pair closes parity")
        else
            for (i, tow, id) in sf[1:min(end, 8)]
                @printf("    preamble at bit %5d  TOW-count %6d  subframe %d\n", i, tow, id)
            end
            length(sf) > 8 && println("    … $(length(sf) - 8) more")
        end
        clean, total, slips, bad, err_at = slip_walk(hard)
        @printf("    parity walk: %d of %d words clean, %d words with bit errors, %d slips%s\n",
                clean, total, bad, length(slips), isempty(slips) ? "" : " (" * join(slips, ",") * ")")
        isempty(err_at) || println("    error runs begin at t = ",
                join([@sprintf("%.1f", t_of(sb[min(end, e + 1)][1], origin)) for e in err_at], " "), " s")
        mags = [abs(b) for (_, b) in sb]
        @printf("    soft-bit magnitude: median %.1f, 10%% quantile %.1f, weakest 1%% %.1f\n",
                median(mags), sort(mags)[max(1, length(mags) ÷ 10)], sort(mags)[max(1, length(mags) ÷ 100)])
    end
    # Loop state from the D lines, every ~10 s.
    dl = get(log.dlines, prn, Vector{String}[])
    if !isempty(dl)
        println("\n  loop state (D lines, ~10 s apart): t, |P|, carrier Doppler, code Doppler, C/N0, polarity, presync window")
        last_t = -Inf
        for f in dl
            t = t_of(parse(Int64, f[3]), origin)
            t - last_t < 10 && continue
            last_t = t
            extra = length(f) >= 14 ? @sprintf("%6s dBHz  pol %2s  win %s", f[12], f[13], f[14]) : ""
            @printf("    %7.1f  |P| %9.0f  %10s Hz  %8s Hz  %s\n", t, parse(Float64, f[5]),
                    f[7], f[9], extra)
        end
    end
end

function main(args)
    isempty(args) && error("usage: julia records.jl <bitstream.log> [prn …]")
    path = args[1]
    log = read_bitlog(path)
    recs = read_records(path * ".records")
    isempty(recs) && error("no R lines in $(path).records")
    origin = minimum(r.sidx for r in recs)
    recs_by_ch = Dict{Int,Vector{Rec}}()
    for r in recs
        push!(get!(recs_by_ch, r.ch, Rec[]), r)
    end
    @printf("%d records over %.1f s on %d channels\n", length(recs),
            t_of(maximum(r.sidx for r in recs), origin), length(recs_by_ch))
    println("\nchannel-table events (A lines: ch, previous PRN, new PRN; negative = noise-channel decoy):")
    for a in log.alines
        @printf("  t=%7.2f  ch%s  %s → %s\n", t_of(parse(Int64, a[5]), origin), a[2], a[3], a[4])
    end
    isempty(log.llines) || println("\nlink counters at the end: gaps=", log.llines[end][3],
                                   " stale=", log.llines[end][4], " dropped=", log.llines[end][5],
                                   " skipped_epochs=", log.llines[end][6])
    prns = length(args) > 1 ? parse.(Int, args[2:end]) :
           sort(unique(r.prn for r in recs if r.prn > 0 && any(a -> parse(Int, a[4]) == r.prn, log.alines)))
    isempty(prns) && (prns = sort(unique(r.prn for r in recs if r.prn > 0)))
    for prn in prns
        analyse_prn(prn, recs_by_ch, origin, log)
    end
end

main(ARGS)
