# Did the 20 ms accumulation lose the bits, or did the prompts never carry them?
#
# The bit-stream log (`HWFIX_BITLOG`) records both halves of the same fold: the
# `B` lines are the soft bits Tracking handed `decode`, and `prompts_prn<N>.f32`
# is every 1 ms filtered prompt that went into them, in fold order. This script
# re-derives the bits from those prompts at all 20 bit-edge hypotheses and scores
# each stream the only way that needs no external truth — the GPS word parity,
# which closes on 6 of every 30 bits and cannot close by luck.
#
#   * prompts decode and the receiver's own bits do not ⇒ the fold / bit-clock
#     path, findable in code with no hardware;
#   * neither decodes ⇒ the 20 ms coherent accumulation itself (loop phase
#     dynamics), and the per-phase energy column says by how much;
#   * both decode ⇒ the bits were never the blocker.
#
# usage: julia softbits_vs_prompts.jl <bitstream.log> <prompts_prn<N>.f32> <prn>

using Printf
using Statistics: mean

const BLOCKS_PER_BIT = 20

# IS-GPS-200 Table 20-XIV, verbatim from `word_errors.jl` so the two scripts
# cannot disagree about what a valid word is. `w` is 30 raw bits, `d29`/`d30`
# the previous word's last two; returns (ok, D29, D30) for chaining.
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

read_prompts(path) = collect(reinterpret(ComplexF32, read(path)))

function read_softbits(path, prn)
    v = Float32[]
    for line in eachline(path)
        f = split(line)
        (isempty(f) || f[1] != "B" || parse(Int, f[2]) != prn) && continue
        for s in f[4:end]
            push!(v, parse(Float32, s))
        end
    end
    v
end

# Best word-parity failure rate over the 30 possible word alignments (the
# D30* mechanism makes this polarity-invariant, so only the alignment and the
# two seed bits are searched). Returns (rate, offset, words, bad).
function best_parity_rate(bits::Vector{Int})
    best = (1.0, 0, 0, 0)
    n = length(bits)
    for offset = 0:29
        n_words = (n - offset) ÷ 30
        n_words >= 8 || continue
        # Seed from whichever (D29*, D30*) makes the first word close; if none
        # does, the alignment is almost certainly wrong anyway, so start at 0/0
        # and let the failure count say so.
        d29, d30 = 0, 0
        for a = 0:1, b = 0:1
            ok, x, y = parity_ok(view(bits, offset+1:offset+30), a, b)
            if ok
                d29, d30 = a, b
                break
            end
        end
        bad = 0
        for w = 1:n_words
            k = offset + (w - 1) * 30
            ok, d29, d30 = parity_ok(view(bits, k+1:k+30), d29, d30)
            ok || (bad += 1)
        end
        rate = bad / n_words
        rate < best[1] && (best = (rate, offset, n_words, bad))
    end
    best
end

hardbits(soft) = [s > 0 ? 1 : 0 for s in soft]

fold(prompts, phase) = [
    real(sum(@view prompts[phase+(k-1)*BLOCKS_PER_BIT+1:phase+k*BLOCKS_PER_BIT])) for
    k = 1:((length(prompts)-phase)÷BLOCKS_PER_BIT)
]

function main(logpath, promptpath, prn)
    prompts = read_prompts(promptpath)
    rx = read_softbits(logpath, prn)
    @printf(
        "PRN %d — %d prompts (%.1f s at 1 kHz), %d soft bits from the log (%.2f bit/s)\n\n",
        prn,
        length(prompts),
        length(prompts) / 1000,
        length(rx),
        length(rx) / (length(prompts) / 1000)
    )

    rate, off, words, bad = best_parity_rate(hardbits(rx))
    @printf("receiver's own soft bits : |bit| mean %9.0f   words %5d   parity failures %5d  (%.2f %%)\n\n",
            mean(abs.(rx)), words, bad, 100 * rate)

    println("bits re-derived from the same prompts, per bit-edge hypothesis:")
    best = (1.0, -1, 0, 0, 0.0)
    for phase = 0:(BLOCKS_PER_BIT-1)
        soft = fold(prompts, phase)
        r, _, w, b = best_parity_rate(hardbits(soft))
        @printf("  phase %2d : |bit| mean %9.0f   words %5d   parity failures %5d  (%.2f %%)\n",
                phase, mean(abs.(soft)), w, b, 100 * r)
        r < best[1] && (best = (r, phase, w, b, mean(abs.(soft))))
    end
    @printf("\nbest edge %d: %.2f %% of words fail parity (%d/%d)\n",
            best[2], 100 * best[1], best[4], best[3])
    @printf("receiver  : %.2f %% of words fail parity (%d/%d)\n", 100 * rate, bad, words)
end

main(ARGS[1], ARGS[2], parse(Int, ARGS[3]))
