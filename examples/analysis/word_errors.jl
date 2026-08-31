# Where do the bit errors sit?
#
# Once one real subframe is located, every subframe boundary in the stream is
# known (300 bits apart). Walk the whole stream on that grid, check each of the
# 10 words' parity, and report the failure map together with the soft-bit
# magnitudes — uniform sprinkling means thermal noise, bursts mean discrete
# events in the ingest path.
#
# usage: julia word_errors.jl <bitstream.log> <prn> <anchor-bit>

using Printf

const PREAMBLE = [1, 0, 0, 0, 1, 0, 1, 1]

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

function main(path, prn, anchor)
    soft = read_softbits(path, prn)
    bits = [s > 0 ? 1 : 0 for s in soft]
    n = length(bits)
    # Walk backwards and forwards from the anchor on the 300-bit grid.
    starts = [s for s = (anchor%300==0 ? 300 : anchor%300):300:(n-300)]
    println("PRN $prn: $n bits, anchor $anchor ⇒ $(length(starts)) full subframes on the grid\n")
    println("subframe   preamble   words failing parity (of 10)   min |soft| in frame / median")
    total_words = 0
    bad_words = 0
    for s in starts
        frame = view(bits, s:s+299)
        # Polarity: the preamble tells us, in either orientation.
        head = frame[1:8]
        inv = head == PREAMBLE ? 0 : (head == [1 - b for b in PREAMBLE] ? 1 : -1)
        pre = inv == -1 ? "✗" : (inv == 1 ? "inv" : "ok")
        fails = Int[]
        if inv != -1
            f = [b ⊻ inv for b in frame]
            d29, d30 = 0, 0
            # Seed from whichever of the four seeds makes word 1 close.
            seeded = false
            for a = 0:1, b = 0:1
                ok, x, y = parity_ok(f[1:30], a, b)
                if ok
                    d29, d30 = x, y
                    seeded = true
                    break
                end
            end
            seeded || push!(fails, 1)
            for w = 2:10
                ok, d29, d30 = parity_ok(f[(w-1)*30+1:w*30], d29, d30)
                ok || push!(fails, w)
            end
            total_words += 10
            bad_words += length(fails) + (seeded ? 0 : 0)
        end
        mags = abs.(view(soft, s:s+299))
        med = sort(collect(mags))[150]
        @printf("%8d   %-8s   %-28s   %.2f\n", s, pre,
                isempty(fails) ? "none — CLEAN" : string(fails), minimum(mags) / med)
    end
    println("\nword parity failure rate: $bad_words / $total_words")
end

main(ARGS[1], parse(Int, ARGS[2]), parse(Int, ARGS[3]))
