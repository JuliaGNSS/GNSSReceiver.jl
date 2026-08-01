# Why does the prompt amplitude decay over tens of seconds?
#
# Reads the `D` lines (|late| |prompt| |early|, carrier Doppler, code Doppler,
# absolute code phase, per chunk) and prints them binned in time. The early/late
# imbalance is the DLL's own measurement of how far the replica sits off the
# correlation peak: if the prompt decays while the imbalance stays ~0, the loss
# is not a code walk-off; if the imbalance grows with the decay, it is.
#
# usage: julia walkoff.jl <bitstream.log> [bin_seconds]

const FS = 4e6

function main(path, bin)
    rows = Dict{Int,Vector{NTuple{7,Float64}}}()
    for line in eachline(path)
        f = split(line)
        (isempty(f) || f[1] != "D") && continue
        prn = parse(Int, f[2])
        # The Dopplers print with their unit ("123.4 Hz"), so pull the numbers
        # out rather than trusting the field count.
        nums = [parse(Float64, m.match) for m in
                eachmatch(r"-?\d+\.?\d*(?:[eE][-+]?\d+)?", line)]
        # nums[1] is the PRN; the seven that follow are the payload.
        length(nums) >= 8 || continue
        push!(get!(rows, prn, NTuple{7,Float64}[]), Tuple(nums[2:8]))
    end
    for prn in sorted_keys(rows)
        v = rows[prn]
        t0 = v[1][1]
        println("\nPRN $prn — $(length(v)) chunks")
        println("  t[s]     |E|      |P|      |L|    (E-L)/(E+L)   carrier_dop[Hz]  code_dop[Hz]")
        binned = Dict{Int,Vector{NTuple{7,Float64}}}()
        for r in v
            push!(get!(binned, Int((r[1] - t0) ÷ (FS * bin)), NTuple{7,Float64}[]), r)
        end
        for k in sort(collect(keys(binned)))
            b = binned[k]
            med(i) = sort([x[i] for x in b])[max(1, length(b) ÷ 2)]
            # D-line order is (late, prompt, early) — Tracking's accumulator order.
            l, p, e = med(2), med(3), med(4)
            disc = (e + l) > 0 ? (e - l) / (e + l) : 0.0
            println("  " * rpad(k * bin, 8) * rpad(round(e, digits=1), 9) *
                    rpad(round(p, digits=1), 9) * rpad(round(l, digits=1), 9) *
                    rpad(round(disc, digits=4), 15) *
                    rpad(round(med(5), digits=2), 17) * string(round(med(6), digits=4)))
        end
    end
end

sorted_keys(d) = sort(collect(keys(d)))

main(ARGS[1], length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 10.0)
