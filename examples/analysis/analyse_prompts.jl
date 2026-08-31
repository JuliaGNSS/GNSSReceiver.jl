# Offline post-mortem on the 1 ms filtered prompts logged by position_fix_bits.jl.
#
# Answers, per PRN:
#   1. is the prompt stream phase-coherent over 20 ms at all (can a data bit be
#      accumulated), and how much residual carrier frequency is left,
#   2. does ANY of the 20 bit-edge hypotheses produce a decodable GPS subframe —
#      with and without an offline carrier-phase derotation.
#
# usage: julia analyse_prompts.jl <dir>

const PREAMBLE = [1, 0, 0, 0, 1, 0, 1, 1]

read_prompts(path) = reinterpret(ComplexF32, read(path))

# GPS L1 C/A word parity (IS-GPS-200 20.3.5.2). `w` is 30 received bits.
function parity_ok(w, d29, d30)
    d = zeros(Int, 25)
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

# How many (TLM, HOW) pairs pass parity behind a preamble, over both polarities.
function subframe_hits(bits)
    hits = 0
    good = 0
    for i = 1:(length(bits)-60)
        for inv in (0, 1)
            w = view(bits, i:i+59)
            head = [b ⊻ inv for b in view(bits, i:i+7)]
            head == PREAMBLE || continue
            hits += 1
            for d29 = 0:1, d30 = 0:1
                ok1, a, b = parity_ok([x ⊻ inv for x in w[1:30]], d29, d30)
                ok1 || continue
                ok2, _, _ = parity_ok([x ⊻ inv for x in w[31:60]], a, b)
                ok2 && (good += 1; break)
            end
        end
    end
    hits, good
end

hard(v) = [real(x) > 0 ? 1 : 0 for x in v]

# Accumulate 20 prompts per bit starting at `offset` (0-based).
function fold_bits(p, offset)
    n = (length(p) - offset) ÷ 20
    [sum(@view p[offset+(k-1)*20+1:offset+k*20]) for k = 1:n]
end

# Estimate and remove the residual carrier phase. Squaring kills the ±1 data
# modulation; the smoothed squared phase, halved, is the carrier phase.
function derotate(p; window = 50)
    sq = ComplexF64.(p) .^ 2
    out = similar(ComplexF64.(p))
    n = length(p)
    acc = sum(@view sq[1:min(window, n)])
    for i = 1:n
        lo = max(1, i - window ÷ 2)
        hi = min(n, i + window ÷ 2)
        s = sum(@view sq[lo:hi])
        φ = angle(s) / 2
        out[i] = p[i] * cis(-φ)
    end
    out
end

function report(prn, p)
    n = length(p)
    mags = abs.(p)
    # Coherence: |Σp| / Σ|p| over non-overlapping 20 ms windows. 1.0 = perfectly
    # coherent; 1/√20 ≈ 0.224 = random phase.
    coh = Float64[]
    for k = 1:(n÷20)
        w = @view p[(k-1)*20+1:k*20]
        push!(coh, abs(sum(w)) / sum(abs, w))
    end
    sort!(coh)
    med_coh = coh[max(1, length(coh) ÷ 2)]
    # Residual carrier: slope of the unwrapped squared phase, per second.
    sq = ComplexF64.(p) .^ 2
    step = 20
    dφ = Float64[]
    for i = 1:step:(n-step)
        a = sum(@view sq[i:i+step-1])
        b = sum(@view sq[min(n - step + 1, i + step):min(n, i + 2step - 1)])
        (abs(a) == 0 || abs(b) == 0) && continue
        push!(dφ, angle(b * conj(a)) / 2 / (step / 1000))
    end
    sort!(dφ)
    med_f = isempty(dφ) ? NaN : dφ[max(1, length(dφ) ÷ 2)] / 2π

    println("\nPRN $prn: $n prompts ($(round(n/1000, digits=1)) s), " *
            "median |prompt| $(round(sum(mags)/n, digits=1))")
    println("  20 ms coherence  : median $(round(med_coh, digits=3))  " *
            "(1.0 = coherent, 0.224 = random phase)")
    println("  residual carrier : median $(round(med_f, digits=2)) Hz " *
            "(from the squared prompt)")

    for (label, q) in (("raw", ComplexF64.(p)), ("derotated", derotate(p)))
        best = (0, 0, 0)
        line = String[]
        for off = 0:19
            b = hard(fold_bits(q, off))
            h, g = subframe_hits(b)
            push!(line, "$off:$h/$g")
            g > best[3] && (best = (off, h, g))
        end
        println("  $label bit-edge sweep (offset:preambles/parity-ok)")
        println("    " * join(line, " "))
        println("    best offset $(best[1]) with $(best[3]) validated subframes")
    end
end

function main(dir)
    for f in sort(readdir(dir))
        startswith(f, "prompts_prn") || continue
        prn = parse(Int, match(r"prn(\d+)", f).captures[1])
        p = read_prompts(joinpath(dir, f))
        length(p) < 2000 && continue
        report(prn, p)
    end
end

main(length(ARGS) >= 1 ? ARGS[1] : ".")
