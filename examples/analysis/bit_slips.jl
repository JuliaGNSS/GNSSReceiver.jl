# Are the bad words bit ERRORS or bit SLIPS?
#
# The two look identical in a word-parity rate over a long stream and are
# completely different defects. A 20 ms bit carrying ~16 dB of SNR cannot
# produce a 3 % bit error rate thermally — but a single *inserted* or *dropped*
# bit shifts every following word off the 30-bit grid, so one slip fails every
# word after it and reads as a catastrophic error rate.
#
# This walks the stream on the word grid, and whenever parity stops closing it
# re-searches the alignment over ±`MAX_SLIP` bits. If parity resumes at a
# shifted alignment, that shift IS the slip, signed: `+k` means the stream
# gained k bits, `-k` means it lost k. Long clean runs between slips prove the
# bits themselves are fine.
#
# usage: julia bit_slips.jl <bitstream.log|prompts.f32> <prn>          (log)
#        julia bit_slips.jl <prompts_prn<N>.f32> --prompts [edge]      (re-fold)

using Printf

const MAX_SLIP = 40
const BLOCKS_PER_BIT = 20

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

fold(p, phase) = [
    real(sum(@view p[phase+(k-1)*BLOCKS_PER_BIT+1:phase+k*BLOCKS_PER_BIT])) for
    k = 1:((length(p)-phase)÷BLOCKS_PER_BIT)
]

# How many consecutive words close parity starting at 0-based bit `start`,
# chaining D29*/D30* and seeding from whichever pair makes the first word close.
function run_length(bits, start)
    start + 30 > length(bits) && return 0, 0, 0
    d29, d30, seeded = 0, 0, false
    for a = 0:1, b = 0:1
        ok, _, _ = parity_ok(view(bits, start+1:start+30), a, b)
        if ok
            d29, d30, seeded = a, b, true
            break
        end
    end
    seeded || return 0, 0, 0
    n = 0
    k = start
    while k + 30 <= length(bits)
        ok, d29, d30 = parity_ok(view(bits, k+1:k+30), d29, d30)
        ok || break
        n += 1
        k += 30
    end
    n, d29, d30
end

function main(bits)
    n = length(bits)
    @printf("%d bits (%.1f s at 50 bit/s)\n\n", n, n / 50)

    # Walk the stream: from the current alignment run parity until it breaks,
    # then search ±MAX_SLIP bits for where it resumes.
    pos = 0
    # Find the first alignment that gives at least 3 clean words.
    while pos < min(n - 300, 600)
        r, _, _ = run_length(bits, pos)
        r >= 3 && break
        pos += 1
    end
    if pos >= min(n - 300, 600)
        println("no clean word alignment found in the first 600 bits — this is not a slip pattern")
        return
    end
    @printf("first clean alignment at bit %d\n\n", pos)
    println("   from bit    clean words    then: slip (bits)   resumed at")
    slips = Int[]
    runs = Int[]
    total_clean = 0
    while true
        r, _, _ = run_length(bits, pos)
        r == 0 && (r = 0)
        total_clean += r
        push!(runs, r)
        endpos = pos + 30 * r
        # Where does parity resume? Search the smallest shift that yields >= 3
        # consecutive clean words.
        best = nothing
        for s in sortperm([abs(x) for x = -MAX_SLIP:MAX_SLIP])
            shift = (-MAX_SLIP:MAX_SLIP)[s]
            cand = endpos + shift
            cand < 0 && continue
            cand + 120 > n && continue
            rr, _, _ = run_length(bits, cand)
            if rr >= 3
                best = (shift, cand, rr)
                break
            end
        end
        if isnothing(best)
            @printf("%10d   %11d    %-17s   %s\n", pos, r, "—", "end / unrecoverable")
            break
        end
        @printf("%10d   %11d    %+17d   %10d\n", pos, r, best[1], best[2])
        push!(slips, best[1])
        pos = best[2]
        pos + 300 > n && break
    end

    words_total = (n - runs[1] * 0) ÷ 30
    @printf("\nclean words %d of ~%d on the grid; %d slips over %.1f s ⇒ one every %.1f s\n",
            total_clean, n ÷ 30, length(slips), n / 50,
            length(slips) == 0 ? Inf : (n / 50) / length(slips))
    if !isempty(slips)
        @printf("slip sizes: %s\n", join(slips, ", "))
        @printf("median clean run: %d words (%.1f s)\n",
                sort(runs)[(length(runs)+1)÷2], 30 * sort(runs)[(length(runs)+1)÷2] / 50)
    end
end

if length(ARGS) >= 2 && ARGS[2] == "--prompts"
    p = read_prompts(ARGS[1])
    phase = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : begin
        best, bp = -1.0, 0
        for ph = 0:19
            e = sum(abs2, fold(p, ph))
            e > best && ((best, bp) = (e, ph))
        end
        bp
    end
    println("re-folded from prompts at edge phase $phase")
    main([s > 0 ? 1 : 0 for s in fold(p, phase)])
else
    main([s > 0 ? 1 : 0 for s in read_softbits(ARGS[1], parse(Int, ARGS[2]))])
end
