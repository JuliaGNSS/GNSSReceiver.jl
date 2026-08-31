# How much of the 1 ms prompt energy survives coherent accumulation, as a
# function of the accumulation length?
#
# A navigation bit is 20 coherently summed 1 ms prompts. If the prompts hold
# their phase, the coherent sum grows as N·A while the noise grows as √N·σ, so
# the *coherent gain* `|Σ_N p| / (N · A₁)` stays at 1 for every N up to the bit
# length. If the replica's phase is walking — a loop tracking its own noise, a
# feedback delay, an NCO that is not doing what the host thinks — the gain
# falls off with N, and the N at which it falls says how fast the phase moves.
#
# `A₁` is the per-record signal amplitude, estimated from the sum itself rather
# than assumed: over a window of `N` records the mean squared prompt is
# `A² + σ²` and the coherent sum is `N·A`, which determines both. Data-bit sign
# flips are removed by taking the sum's own sign per window (a window is at most
# one bit long and windows are aligned to the best bit edge).
#
# usage: julia prompt_coherence.jl <prompts_prn<N>.f32> [bit-edge phase]

using Printf
using Statistics: mean, median, std

read_prompts(path) = collect(reinterpret(ComplexF32, read(path)))

# Coherent sum magnitude over consecutive, non-overlapping windows of `n`
# records starting at `phase`.
function window_sums(p, n, phase)
    m = (length(p) - phase) ÷ n
    out = Vector{ComplexF64}(undef, m)
    @inbounds for k = 1:m
        acc = zero(ComplexF64)
        base = phase + (k - 1) * n
        for j = 1:n
            acc += p[base+j]
        end
        out[k] = acc
    end
    out
end

# The bit edge that maximises 20 ms energy — the same criterion Tracking's CFAR
# detector maximises, so this is the edge the receiver should have found.
function best_edge(p)
    best, bestphase = -1.0, 0
    for phase = 0:19
        e = mean(abs2, window_sums(p, 20, phase))
        e > best && ((best, bestphase) = (e, phase))
    end
    bestphase
end

function main(path, phase_arg)
    p = read_prompts(path)
    phase = isnothing(phase_arg) ? best_edge(p) : phase_arg
    @printf("%d prompts (%.1f s at 1 kHz), bit edge phase %d\n\n", length(p),
            length(p) / 1000, phase)

    # Per-record statistics, and the amplitude implied by the 20 ms sum.
    mean_sq = mean(abs2, p)
    sums20 = window_sums(p, 20, phase)
    a1 = mean(abs, sums20) / 20                     # per-record signal amplitude
    noise_var = max(mean_sq - a1^2, 0.0)
    @printf("per-record  E|p|² = %.4g   ⇒  A₁ = %.4g,  σ = %.4g,  1 ms SNR = %.2f dB\n",
            mean_sq, a1, sqrt(noise_var), 10 * log10(a1^2 / max(noise_var, eps())))
    @printf("            C/N₀ from that = %.1f dBHz (1 ms records)\n\n",
            10 * log10(a1^2 / max(noise_var, eps()) / 1e-3))

    println("      N   |Σ_N| / (N·A₁)    predicted (thermal only)   Σ|p| ratio")
    for n in (1, 2, 4, 5, 10, 20, 40, 100)
        n > length(p) ÷ 4 && continue
        s = window_sums(p, n, phase)
        gain = mean(abs, s) / (n * a1)
        # With a fixed phase and thermal noise only, |Σ_N| is Rician with
        # signal N·A₁ and noise variance N·σ²; its mean exceeds N·A₁ slightly.
        snr_n = n * a1^2 / max(noise_var, eps())
        predicted = sqrt(1 + 1 / snr_n)
        incoh = mean(abs, s) / (n * mean(abs, p))
        @printf("  %5d   %13.4f    %23.4f   %10.4f\n", n, gain, predicted, incoh)
    end

    # Per-record phase innovation, modulo the data-bit sign: arg(pₖ·conj(pₖ₋₁))
    # folded into (-π/2, π/2]. Thermal noise alone gives
    # σ ≈ √2 / √(2·SNR₁) rad.
    d = Float64[]
    for k = 2:length(p)
        z = p[k] * conj(p[k-1])
        a = angle(z)
        a > pi / 2 && (a -= pi)
        a < -pi / 2 && (a += pi)
        push!(d, a)
    end
    snr1 = a1^2 / max(noise_var, eps())
    @printf("\nper-record phase step: σ = %.3f rad (median |step| %.3f); thermal alone ≈ %.3f rad\n",
            std(d), median(abs.(d)), sqrt(2 / (2 * snr1)))

    # Where does the 20 ms energy sit relative to the worst edge? A clean bit
    # edge gives a 2:1 peak-to-trough triangle over the 20 hypotheses.
    energies = [mean(abs, window_sums(p, 20, ph)) for ph = 0:19]
    @printf("20 ms |sum| over the 20 edges: peak %.4g at phase %d, trough %.4g ⇒ ratio %.2f (clean = 2.0)\n",
            maximum(energies), argmax(energies) - 1, minimum(energies),
            maximum(energies) / minimum(energies))
end

main(ARGS[1], length(ARGS) >= 2 ? parse(Int, ARGS[2]) : nothing)
