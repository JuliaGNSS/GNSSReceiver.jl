function main(path)
    p = ComplexF64.(reinterpret(ComplexF32, read(path)))
    n = length(p)
    best, boff = -Inf, 0
    for off = 0:19
        s = sum(abs(sum(@view p[off+(k-1)*20+1:off+k*20])) for k = 1:((n-off)÷20))
        if s > best
            best, boff = s, off
        end
    end
    println("prompts: $n; best bit-edge offset $boff")
    bits = [sum(@view p[boff+(k-1)*20+1:boff+k*20]) for k = 1:((n-boff)÷20)]
    mag = abs.(bits)
    med = sort(mag)[length(mag)÷2]
    weak = findall(m -> m < 0.3med, mag)
    println("bits: $(length(bits)); median |bit| = $(round(med, digits=0)); " *
            "weak (<0.3·median): $(length(weak)) = " *
            "$(round(100length(weak)/length(bits), digits=1))%")
    pm = sort(abs.(p))[n÷2]
    println("\nthe 20 one-millisecond prompts inside a few weak bits")
    println("(real parts / median 1 ms |prompt|; a real data transition inside the")
    println(" window shows as one clean sign change part-way through)")
    for k in weak[1:min(10, end)]
        seg = @view p[boff+(k-1)*20+1:boff+k*20]
        println("  bit $k (t=$(round((boff+(k-1)*20)/1000, digits=2))s) " *
                "|Σ|/med=$(round(mag[k]/med, digits=2)):  " *
                join([string(round(real(x) / pm, digits=1)) for x in seg], " "))
    end
    strong = findall(m -> m > 0.9med, mag)
    println("\nand inside a few strong bits, for contrast")
    for k in strong[1:min(3, end)]
        seg = @view p[boff+(k-1)*20+1:boff+k*20]
        println("  bit $k:  " *
                join([string(round(real(x) / pm, digits=1)) for x in seg], " "))
    end
end
main(ARGS[1])
