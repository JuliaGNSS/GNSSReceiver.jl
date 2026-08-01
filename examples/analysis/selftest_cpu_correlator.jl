#!/usr/bin/env julia
#
# Pin the CPU reference correlator against an analytically known signal.
#
# The FPGA-vs-CPU probe's whole claim is "these two correlated the same samples,
# so a difference is the device's". That claim is worth nothing if the reference
# is itself mis-aligned, mis-scaled or mixing with the wrong sign — every one of
# which shows up as "the FPGA is worse". So the reference is checked here the
# way the gateware's datapath was: inject a signal whose PRN, code phase and
# Doppler are known exactly, and require the correlator to return the answer
# arithmetic says it must.
#
#   julia --project=examples examples/analysis/selftest_cpu_correlator.jl
#
# No board, no receiver, no logs.

using Test
using Random: MersenneTwister
using GNSSSignals: GPSL1CA, get_code

include(joinpath(@__DIR__, "cpu_correlator.jl"))
using .CpuCorrelator: SampleRing, Workspace, push_chunk!, covers, fetch!,
    correlate_epl, search_offset

const FS = 4e6
const CODE_LENGTH = 1023
const CHIP_RATE = 1.023e6
const gpsl1 = GPSL1CA()

code_table(prn) = Float32[get_code(gpsl1, i, prn) for i = 0:(CODE_LENGTH-1)]

# One code period's worth of GPS L1 C/A at a known phase and Doppler. The
# convention here is the *signal's*: a satellite Doppler of `f` puts the carrier
# at `exp(+i2πft)`, so a correlator that mixes with `exp(+i2πft)` (sign +1) must
# come out with nothing and one that mixes with the conjugate (sign −1) with
# everything. That asymmetry is what the probe's sign vote resolves at run time.
function inject(prn, n, cp0, doppler, amplitude; noise = 0.0, seed = 42)
    rng = MersenneTwister(seed)
    code = code_table(prn)
    cps = (CHIP_RATE * (1 + doppler / 1.57542e9)) / FS
    out = Vector{ComplexF32}(undef, n)
    for k = 0:(n-1)
        chip = code[Int(mod(floor(Int, cp0 + k * cps), CODE_LENGTH))+1]
        carrier = cis(2π * doppler * k / FS)
        out[k+1] = ComplexF32(amplitude * chip * carrier)
        noise > 0 && (out[k+1] += ComplexF32(noise * (randn(rng) + im * randn(rng))))
    end
    (out, cps)
end

# Rings `samples` at an arbitrary device origin, the way the live probe does.
function ringed(samples, origin)
    ring = SampleRing(1 << 16)
    push_chunk!(ring, reshape(samples, :, 1), origin)
    ring
end

const ORIGIN = Int64(987_654_321)
const N = 4000                 # one 1 ms record at 4 MHz
const PRN = 7
const CP0 = 123.456
const DOPPLER = -2537.0
const AMP = 30.0

@testset "CPU reference correlator" begin
    code = code_table(PRN)
    ws = Workspace(8192, 64)

    @testset "ring addresses samples by device index" begin
        samples, _ = inject(PRN, 8192, CP0, DOPPLER, AMP)
        ring = ringed(samples, ORIGIN)
        @test covers(ring, ORIGIN, 8192) === :ok
        @test covers(ring, ORIGIN + 8192, 1) === :future
        @test covers(ring, ORIGIN - 1, 2) === :old
        dst = Vector{ComplexF32}(undef, 100)
        fetch!(dst, ring, ORIGIN + 500, 100)
        @test dst == samples[501:600]
    end

    @testset "the prompt is the analytic value" begin
        samples, cps = inject(PRN, N, CP0, DOPPLER, AMP)
        ring = ringed(samples, ORIGIN)
        late, prompt, early = correlate_epl(
            ws, ring, ORIGIN, N; code, cp_start = CP0, cps, freq = DOPPLER,
            fs = FS, sign = -1, shift = 1)
        # Carrier and code both wiped: every sample contributes `amplitude`.
        @test abs(prompt) ≈ AMP * N rtol = 1e-3
        # …and it is real, i.e. the mixing left no residual phase rotation.
        @test abs(imag(prompt)) < 1e-3 * abs(prompt)
        # One sample of Early/Late spacing costs exactly the samples whose chip
        # changes across that shift — 24 % of them here, so E and L keep ~76 %
        # of the prompt and sit level with each other on an aligned replica.
        # (The idealised `1 - 0.2558 chips` is 1 % off: how many of a Gold
        # code's 1023 chip boundaries are actual sign transitions is a property
        # of the PRN, so the expectation is computed from the code itself.)
        chips = [code[Int(mod(floor(Int, CP0 + k * cps), CODE_LENGTH))+1] for k = 0:N]
        transitions = count(k -> chips[k] != chips[k+1], 1:N)
        @test abs(early) / abs(prompt) ≈ 1 - 2 * transitions / N rtol = 1e-3
        @test abs(early) ≈ abs(late) rtol = 0.02
    end

    @testset "the wrong mixing sign loses everything" begin
        samples, cps = inject(PRN, N, CP0, DOPPLER, AMP)
        ring = ringed(samples, ORIGIN)
        _, wrong, _ = correlate_epl(
            ws, ring, ORIGIN, N; code, cp_start = CP0, cps, freq = DOPPLER,
            fs = FS, sign = +1, shift = 1)
        # 2×Doppler is ~5 kHz over a 1 ms integration: five full turns.
        @test abs(wrong) < 0.05 * AMP * N
    end

    @testset "the wrong PRN loses everything" begin
        samples, cps = inject(PRN, N, CP0, DOPPLER, AMP)
        ring = ringed(samples, ORIGIN)
        _, cross, _ = correlate_epl(
            ws, ring, ORIGIN, N; code = code_table(PRN + 1), cp_start = CP0, cps,
            freq = DOPPLER, fs = FS, sign = -1, shift = 1)
        @test abs(cross) < 0.1 * AMP * N
    end

    @testset "the search finds the injected alignment and votes the sign" begin
        samples, cps = inject(PRN, N, CP0, DOPPLER, AMP; noise = 8.0)
        ring = ringed(samples, ORIGIN)
        offset, ratio, sign = search_offset(
            ws, ring, ORIGIN, N; code, cp_start = CP0, cps, freq = DOPPLER,
            fs = FS, margin = 64)
        @test offset == 0
        @test sign == -1
        @test ratio > 20
    end

    @testset "the search measures a sample-mapping error" begin
        # Hand the correlator a code phase 7 samples too *low*: the peak must
        # come back at +7 samples, which is exactly how the live probe turns an
        # origin residual into chips.
        for err in (-31, -7, 1, 12)
            samples, cps = inject(PRN, N, CP0, DOPPLER, AMP; noise = 8.0)
            ring = ringed(samples, ORIGIN)
            offset, ratio, _ = search_offset(
                ws, ring, ORIGIN, N; code, cp_start = CP0 + err * cps, cps,
                freq = DOPPLER, fs = FS, signs = (-1,), margin = 64)
            @test offset == -err
            @test ratio > 20
        end
    end

    @testset "an early-late imbalance points the right way" begin
        # Replica a quarter chip late ⇒ the Early tap is closer to the peak ⇒ a
        # positive discriminator, which is the sign the probe's CPU-side DLL
        # steers on. Get this backwards and the reference walks off the peak.
        samples, cps = inject(PRN, N, CP0, DOPPLER, AMP)
        ring = ringed(samples, ORIGIN)
        late, _, early = correlate_epl(
            ws, ring, ORIGIN, N; code, cp_start = CP0 - 0.25, cps,
            freq = DOPPLER, fs = FS, sign = -1, shift = 1)
        @test (abs(early) - abs(late)) / (abs(early) + abs(late)) > 0.1
        late, _, early = correlate_epl(
            ws, ring, ORIGIN, N; code, cp_start = CP0 + 0.25, cps,
            freq = DOPPLER, fs = FS, sign = -1, shift = 1)
        @test (abs(early) - abs(late)) / (abs(early) + abs(late)) < -0.1
    end

    @testset "a long integration does not drift the carrier replica" begin
        # 20 code periods: an unrenormalised ComplexF32 rotation would lose
        # amplitude here, and that loss would be charged to the CPU reference.
        n = 20N
        samples, cps = inject(PRN, n, CP0, DOPPLER, AMP)
        ring = SampleRing(1 << 18)
        push_chunk!(ring, reshape(samples, :, 1), ORIGIN)
        ws_long = Workspace(n, 4)
        _, prompt, _ = correlate_epl(
            ws_long, ring, ORIGIN, n; code, cp_start = CP0, cps, freq = DOPPLER,
            fs = FS, sign = -1, shift = 1)
        @test abs(prompt) ≈ AMP * n rtol = 2e-3
    end
end
