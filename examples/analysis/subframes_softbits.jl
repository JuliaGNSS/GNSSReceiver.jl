# Are the subframes in the SOFT BITS real GPS subframes?
#
# Same check as subframes.jl, but on the soft bits Tracking actually handed to
# `decode` (the `B` lines of the bit-stream log) rather than on a re-fold of the
# prompts. Any difference between the two is Tracking's accumulation or the
# receiver's per-chunk feed, not the signal.
#
# For every position where the preamble matches and TLM+HOW parity closes, the
# HOW's 17-bit TOW count is decoded. Real subframes 300 bits apart must have
# consecutive TOW counts; coincidences will not.
#
# usage: julia subframes_softbits.jl <bitstream.log> <prn>

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

function main(path, prn)
    soft = read_softbits(path, prn)
    bits = [s > 0 ? 1 : 0 for s in soft]
    n = length(bits)
    hits = scan(bits)
    println("PRN $prn soft bits from the log: $n bits ($(round(n/50, digits=1)) s), " *
            "$(length(hits)) preamble+parity hits, $(round(n/300, digits=1)) subframes in window")
    for h in hits
        println("  bit $(h.pos)  polarity $(h.inv == 1 ? "inverted" : "upright")  " *
                "TOW-count $(h.tow)  subframe id $(h.id)")
    end
    consistent = 0
    for a in hits, b in hits
        a.pos < b.pos || continue
        d = b.pos - a.pos
        d % 300 == 0 || continue
        ok = (b.tow - a.tow) == d ÷ 300
        ok && (consistent += 1)
        println("    $(a.pos) → $(b.pos): Δbits $d, ΔTOW $(b.tow - a.tow)  $(ok ? "✓ REAL" : "✗")")
    end
    consistent == 0 && println("    no consistent pair")
end

main(ARGS[1], parse(Int, ARGS[2]))
