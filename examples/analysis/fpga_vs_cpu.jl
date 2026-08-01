#!/usr/bin/env julia
#
# What does the FPGA correlator lose against a CPU correlation of the same
# samples? Reads the `HWFIX_XCORR` log of
# `../hardware_correlator_position_fix.jl`, which carries, per hardware record,
# the device's Early/Prompt/Late beside a CPU correlation of exactly the samples
# that record integrated, with exactly the NCO parameters the device ran.
#
#   julia fpga_vs_cpu.jl xcorr.log [PRN…]
#
# Both prompts see one input, so the antenna, the RF chain, DMA0, acquisition
# and the loop filters are common-mode and cancel. What the report shows is the
# difference between the two correlators — and, because the 200 s bit-error
# study (issue #107) had already put the software receiver's decode at zero
# errors and the hardware path's at ~3 %, the difference is the whole gap.
#
# Nothing here needs the board.

using Printf
using Statistics: mean, median, std

const T_INT = 1e-3          # nominal record length, for the C/N0 scaling
const FS = 4e6

struct Series
    prn::Int
    channel::Int
    sample::Vector{Int64}
    n::Vector{Int}
    code_phase::Vector{Float64}
    delta::Vector{Float64}
    pf::Vector{ComplexF64}
    lf::Vector{Float64}
    ef::Vector{Float64}
    pc::Vector{ComplexF64}
    lc::Vector{Float64}
    ec::Vector{Float64}
    carrier::Vector{Float64}
end

Series(prn, ch) = Series(prn, ch, Int64[], Int[], Float64[], Float64[],
                         ComplexF64[], Float64[], Float64[],
                         ComplexF64[], Float64[], Float64[], Float64[])

function read_log(path)
    series = Dict{Int,Series}()
    aligns = Tuple{Int,Int,Int,Float64,Float64,Int}[]
    for line in eachline(path)
        (isempty(line) || line[1] == '#') && continue
        f = split(line)
        if f[1] == "X" && length(f) >= 17
            prn = parse(Int, f[2])
            ch = parse(Int, f[3])
            s = get!(() -> Series(prn, ch), series, prn)
            push!(s.sample, parse(Int64, f[4]))
            push!(s.n, parse(Int, f[5]))
            push!(s.code_phase, parse(Float64, f[6]))
            push!(s.delta, parse(Float64, f[7]))
            push!(s.pf, complex(parse(Float64, f[8]), parse(Float64, f[9])))
            push!(s.lf, parse(Float64, f[10]))
            push!(s.ef, parse(Float64, f[11]))
            push!(s.pc, complex(parse(Float64, f[12]), parse(Float64, f[13])))
            push!(s.lc, parse(Float64, f[14]))
            push!(s.ec, parse(Float64, f[15]))
            push!(s.carrier, parse(Float64, f[16]))
        elseif f[1] == "A" && length(f) >= 7
            push!(aligns, (parse(Int, f[2]), parse(Int, f[3]),
                           parse(Int, f[5]), parse(Float64, f[6]),
                           parse(Float64, f[7]), parse(Int, f[8])))
        end
    end
    (series, aligns)
end

# The beat between the two prompt streams, in Hz: the frequency at which
# `Pf·conj(Pc)` rotates. Nav-bit flips are common to both and cancel in that
# product, so this needs no bit sync — and because the CPU side mixed with the
# Doppler the loop commanded, a non-zero beat means the device's carrier NCO is
# not running at the word it was given.
#
# Found as a periodogram peak rather than from consecutive phase increments: at
# low coherence the increment estimator is dominated by noise and returns a
# plausible-looking wrong answer, which then destroys the coherence figure it
# feeds. `BEAT_SEARCH_HZ` bounds it; a device off by more than that has already
# lost all its correlation gain, which the C/N0 pair reports on its own.
const BEAT_SEARCH_HZ = 5.0
const BEAT_BLOCK = 20            # decimate to 50 Hz before searching

function beat_frequency(s::Series)
    length(s.sample) < 10BEAT_BLOCK && return 0.0
    # Decimate: the beats of interest are sub-Hz to a few Hz, far inside the
    # 25 Hz Nyquist a 20 ms block leaves.
    nb = length(s.pf) ÷ BEAT_BLOCK
    r = Vector{ComplexF64}(undef, nb)
    t = Vector{Float64}(undef, nb)
    for b = 1:nb
        idx = ((b-1)*BEAT_BLOCK+1):(b*BEAT_BLOCK)
        r[b] = sum(k -> s.pf[k] * conj(s.pc[k]), idx)
        t[b] = mean(view(s.sample, idx)) / FS
    end
    span = t[end] - t[1]
    span <= 0 && return 0.0
    # A quarter of the Rayleigh resolution: fine enough that scalloping cannot
    # hide the peak, coarse enough to stay a fraction of a second of work.
    step = 0.25 / span
    dt = span / (nb - 1)
    coarse = _peak_uniform(r, dt, -BEAT_SEARCH_HZ, BEAT_SEARCH_HZ, step)
    # Refine on the true (possibly gapped) timestamps.
    _peak_exact(r, t, coarse - 2step, coarse + 2step, step / 20)
end

# Incremental rotation on an assumed-uniform grid: one complex multiply per
# point per trial frequency. A hole in the record stream smears this peak
# slightly; the exact-time refinement below puts it back.
function _peak_uniform(r, dt, lo, hi, step)
    best_f, best_p = lo, -1.0
    f = lo
    while f <= hi
        w = cis(-2π * f * dt)
        c = one(ComplexF64)
        acc = zero(ComplexF64)
        @inbounds for k in eachindex(r)
            acc += r[k] * c
            c *= w
        end
        p = abs(acc)
        p > best_p && ((best_f, best_p) = (f, p))
        f += step
    end
    best_f
end

function _peak_exact(r, t, lo, hi, step)
    best_f, best_p = lo, -1.0
    f = lo
    while f <= hi
        acc = zero(ComplexF64)
        @inbounds for k in eachindex(r)
            acc += r[k] * cis(-2π * f * t[k])
        end
        p = abs(acc)
        p > best_p && ((best_f, best_p) = (f, p))
        f += step
    end
    best_f
end

# How much of the two prompts is the *same* signal. `ρ = 1` means the FPGA
# prompt is the CPU prompt up to a gain and a phase; the shortfall is noise one
# of them has and the other does not.
function coherence(s::Series, beat)
    t = (s.sample .- s.sample[1]) ./ FS
    num = sum(s.pf .* conj.(s.pc) .* cis.(-2π * beat .* t))
    den = sqrt(sum(abs2, s.pf) * sum(abs2, s.pc))
    den == 0 ? 0.0 : abs(num) / den
end

# Narrowband/wideband power ratio C/N0 (Beaulieu's NWPR), on blocks of `m`
# records aligned to the bit edge that maximises coherent gain. This is the
# estimator that needs no noise reference, which is what makes it usable on two
# streams whose absolute scales differ by a factor of a hundred.
function nwpr_cn0(p::Vector{ComplexF64}, edge::Int; m::Int = 20)
    nbp_over_wbp = Float64[]
    k = edge + 1
    while k + m - 1 <= length(p)
        block = view(p, k:(k+m-1))
        wbp = sum(abs2, block)
        wbp > 0 && push!(nbp_over_wbp, abs2(sum(block)) / wbp)
        k += m
    end
    isempty(nbp_over_wbp) && return (NaN, 0)
    mu = mean(nbp_over_wbp)
    snr = (mu - 1) / (m - mu)
    (snr <= 0 ? NaN : 10log10(snr / T_INT), length(nbp_over_wbp))
end

# The 20 ms bit edge, chosen the way a receiver would: the alignment whose
# coherent sums are largest.
function best_edge(p::Vector{ComplexF64}; m::Int = 20)
    best, best_power = 0, -Inf
    for edge = 0:(m-1)
        power = 0.0
        k = edge + 1
        while k + m - 1 <= length(p)
            power += abs2(sum(view(p, k:(k+m-1))))
            k += m
        end
        power > best_power && ((best, best_power) = (edge, power))
    end
    best
end

# Hard bit decisions from each stream on the same 20 ms grid, compared. The
# phase reference is each stream's own block sum, so a constant phase offset
# between the two correlators cannot register as a bit error; only a block whose
# sign genuinely differs does.
function bit_disagreement(s::Series, edge::Int; m::Int = 20)
    disagree = 0
    total = 0
    k = edge + 1
    ref_f = one(ComplexF64)
    ref_c = one(ComplexF64)
    while k + m - 1 <= length(s.pf)
        sf = sum(view(s.pf, k:(k+m-1)))
        sc = sum(view(s.pc, k:(k+m-1)))
        if abs(sf) > 0 && abs(sc) > 0
            bf = real(sf * conj(ref_f)) >= 0
            bc = real(sc * conj(ref_c)) >= 0
            bf == bc || (disagree += 1)
            total += 1
            # Carry each stream's phase forward, flipped by its own decision, so
            # the reference rides the carrier instead of drifting out of it.
            ref_f = bf ? sf : -sf
            ref_c = bc ? sc : -sc
        end
        k += m
    end
    (disagree, total)
end

imbalance(e, l) = mean((e .- l) ./ max.(e .+ l, eps()))

function report(s::Series, aligns)
    n = length(s.pf)
    n < 100 && (@printf("PRN %2d: only %d records — skipped\n", s.prn, n); return)
    span = (s.sample[end] - s.sample[1]) / FS
    beat = beat_frequency(s)
    # Regression coefficient, not a ratio of magnitudes: |Pf|/|Pc| averages the
    # FPGA's *own* noise into the gain, which inflates it exactly when the
    # device is worst. This one is unbiased under independent noise.
    t = (s.sample .- s.sample[1]) ./ FS
    scale = abs(sum(s.pf .* conj.(s.pc) .* cis.(-2π * beat .* t))) / sum(abs2, s.pc)
    ratio = abs.(s.pf) ./ max.(scale .* abs.(s.pc), eps())
    rho = coherence(s, beat)
    edge = best_edge(s.pc)
    cn0_f, blocks = nwpr_cn0(s.pf, edge)
    cn0_c, _ = nwpr_cn0(s.pc, edge)
    disagree, total = bit_disagreement(s, edge)
    excess = rho > 0 && rho < 1 ? 10log10(1 / rho^2 - 1) : NaN

    @printf("\nPRN %2d  (hardware channel %d)\n", s.prn, s.channel)
    @printf("  %d records over %.1f s, %.0f Hz mean carrier Doppler\n",
            n, span, mean(s.carrier))
    for a in aligns
        a[1] == s.prn || continue
        @printf("  coarse alignment: %+d samples (%+.3f chips), peak %.1f× floor, mixing sign %+d\n",
                a[3], a[4], a[5], a[6])
    end
    @printf("  CPU-side code offset: %+.3f … %+.3f chips (the host↔device axis residual)\n",
            minimum(s.delta), maximum(s.delta))
    @printf("  amplitude scale Pf/Pc: %.4g (median of |Pf|/|Pc|: %.4g), per-record spread %.1f %%\n",
            scale, median(abs.(s.pf) ./ max.(abs.(s.pc), eps())), 100 * std(ratio))
    @printf("  prompt coherence ρ = %.4f  ⇒  FPGA-only noise %+.1f dB relative to the common prompt\n",
            rho, excess)
    @printf("  FPGA↔CPU beat frequency: %+.3f Hz  (0 ⇒ the carrier NCO runs at the commanded word)\n",
            beat)
    @printf("  C/N0 over %d bit-aligned 20 ms blocks: FPGA %.1f dBHz | CPU %.1f dBHz | Δ %+.1f dB\n",
            blocks, cn0_f, cn0_c, cn0_f - cn0_c)
    @printf("  20 ms bit decisions disagreeing: %d of %d (%.2f %%)\n",
            disagree, total, 100 * disagree / max(total, 1))
    @printf("  early−late imbalance: FPGA %+.4f | CPU %+.4f\n",
            imbalance(s.ef, s.lf), imbalance(s.ec, s.lc))
end

function main(args)
    isempty(args) && (println("usage: fpga_vs_cpu.jl <xcorr.log> [PRN…]"); return)
    series, aligns = read_log(args[1])
    wanted = length(args) > 1 ? parse.(Int, args[2:end]) : sort!(collect(keys(series)))
    isempty(series) && (println("no X records in $(args[1])"); return)
    println("$(args[1]): $(length(series)) satellites, ",
            sum(length(s.pf) for s in values(series)), " compared records")
    println("""
    ρ is the normalised complex correlation between the two prompt streams: 1.0 is
    "the FPGA prompt IS the CPU prompt up to gain and phase". The C/N0 pair is the
    same NWPR estimator run on each stream, so the Δ is the correlator's own loss.""")
    for prn in wanted
        haskey(series, prn) || continue
        report(series[prn], aligns)
    end
end

main(ARGS)
