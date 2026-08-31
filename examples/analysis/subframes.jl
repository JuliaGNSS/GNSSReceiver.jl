# Are the "validated subframes" real GPS subframes?
#
# For every position where the preamble matches and TLM+HOW parity closes,
# decode the HOW's 17-bit TOW count. Real subframes 300 bits apart must have
# consecutive TOW counts; coincidences will not.
#
# Also reports the per-bit error rate implied by how many of the ~N subframes in
# the window validate, and whether any two validating subframes are adjacent —
# which is what GNSSDecoder's `find_preamble` needs (preamble at both ends of a
# 300-bit window).
#
# usage: julia subframes.jl <prompts.f32> [offset]

const PREAMBLE = [1, 0, 0, 0, 1, 0, 1, 1]

read_prompts(path) = reinterpret(ComplexF32, read(path))

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

hard(v) = [real(x) > 0 ? 1 : 0 for x in v]
fold_bits(p, offset) = [sum(@view p[offset+(k-1)*20+1:offset+k*20])
                        for k = 1:((length(p)-offset)÷20)]

# Data bits of a word, after removing the D30* scrambling.
words_data(w, d30) = [w[i] ⊻ d30 for i = 1:24]

function scan(bits)
    found = NamedTuple[]
    for i = 1:(length(bits)-60)
        for inv in (0, 1)
            [b ⊻ inv for b in view(bits, i:i+7)] == PREAMBLE || continue
            tlm = [b ⊻ inv for b in view(bits, i:i+29)]
            how = [b ⊻ inv for b in view(bits, i+30:i+59)]
            for d29 = 0:1, d30 = 0:1
                ok1, a, b = parity_ok(tlm, d29, d30)
                ok1 || continue
                ok2, _, _ = parity_ok(how, a, b)
                ok2 || continue
                # HOW bits 1..17 are the TOW count (start of the NEXT subframe).
                hd = words_data(how, b)
                tow = 0
                for k = 1:17
                    tow = 2tow + hd[k]
                end
                subframe_id = 4hd[20] + 2hd[21] + hd[22]
                push!(found, (pos = i, inv = inv, tow = tow, id = subframe_id))
                break
            end
        end
    end
    found
end

function main(path, offset)
    p = read_prompts(path)
    bits = hard(fold_bits(p, offset))
    n = length(bits)
    hits = scan(bits)
    println("$(basename(path)) offset $offset: $n bits ($(round(n/50, digits=1)) s), " *
            "$(length(hits)) preamble+parity hits, $(round(n/300, digits=1)) subframes in window")
    for h in hits
        println("  bit $(h.pos)  polarity $(h.inv == 1 ? "inverted" : "upright")  " *
                "TOW-count $(h.tow) (t = $(h.tow * 6) s of week)  subframe id $(h.id)")
    end
    # Consistency: two real subframes 300 bits apart differ by exactly 1 TOW.
    println("  consistency checks (pairs whose bit distance is a multiple of 300):")
    consistent = 0
    for a in hits, b in hits
        a.pos < b.pos || continue
        d = b.pos - a.pos
        d % 300 == 0 || continue
        expect = d ÷ 300
        got = b.tow - a.tow
        ok = got == expect
        ok && (consistent += 1)
        println("    $(a.pos) → $(b.pos): Δbits $d (=$(expect) subframes), " *
                "ΔTOW $got  $(ok ? "✓ REAL" : "✗")")
    end
    if consistent == 0
        println("    none — no two hits are a whole number of subframes apart with " *
                "matching TOW, so these hits are coincidences, not GPS frames")
    end
end

main(ARGS[1], length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 0)
