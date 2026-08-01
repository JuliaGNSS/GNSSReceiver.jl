# A plain CPU correlator over a ring of raw samples, addressed by the device's
# own free-running sample index.
#
# This is the reference half of the FPGA-vs-CPU probe (issue #107): given the
# span of raw samples a hardware record integrated over, and the NCO parameters
# the device was running, it produces the Early/Prompt/Late the device *should*
# have produced. Both sides then see one input, so every shared error source —
# antenna, RF, DMA0, acquisition, the loop filters — cancels, and what is left
# over is the correlator's.
#
# It deliberately depends on nothing: no GNSSReceiver, no Tracking, no vendor
# package, not even GNSSSignals (the ±1 code comes in as a plain vector). That
# is what lets `selftest_cpu_correlator.jl` pin it against an analytically known
# injected signal with no board and no receiver in the loop — the same way the
# gateware's own datapath was pinned before it was believed.
module CpuCorrelator

export SampleRing, Workspace, push_chunk!, covers, fetch!, correlate_epl,
    search_offset, accumulate_offsets!, peak_offset

"""
    SampleRing(len)

Raw samples addressed by *device* sample index: the slot for index `d` is
`mod(d, len) + 1`, so no bookkeeping is needed beyond how much of the ring is
currently valid (`[ending - filled, ending)`).
"""
mutable struct SampleRing
    const data::Vector{ComplexF32}
    ending::Int64
    filled::Int64
end

SampleRing(len::Integer) = SampleRing(zeros(ComplexF32, len), typemin(Int64), 0)

"""
    push_chunk!(ring, samples, start_index) -> ring

Append one chunk at its device index. A discontinuity — the first chunk, or a
host axis that jumped — restarts the ring rather than leaving it silently
correlating against samples from the wrong time.

`samples` is indexed `samples[k, 1]`, i.e. antenna 0 of a matrix chunk, or a
vector.
"""
function push_chunk!(ring::SampleRing, samples, start_index::Integer)
    n = size(samples, 1)
    len = length(ring.data)
    start = Int64(start_index)
    if ring.ending != start
        ring.ending = start
        ring.filled = 0
    end
    @inbounds for k = 1:n
        ring.data[mod(start + k - 1, len)+1] = ComplexF32(samples[k, 1])
    end
    ring.ending += n
    ring.filled = min(ring.filled + n, Int64(len))
    ring
end

"""
    covers(ring, s0, n) -> Symbol

`:ok`, `:future` (the span reaches past the newest sample delivered) or `:old`
(the ring has already rolled over its start).
"""
function covers(ring::SampleRing, s0::Integer, n::Integer)
    s0 + n > ring.ending && return :future
    s0 < ring.ending - ring.filled && return :old
    :ok
end

"""
    fetch!(dst, ring, s0, n)

Copy `[s0, s0+n)` out of the ring, unwrapping at most one seam so the kernels
below see a contiguous span.
"""
function fetch!(dst, ring::SampleRing, s0::Integer, n::Integer)
    len = length(ring.data)
    i0 = Int(mod(Int64(s0), len))
    head = min(Int(n), len - i0)
    copyto!(dst, 1, ring.data, i0 + 1, head)
    head < n && copyto!(dst, head + 1, ring.data, 1, Int(n) - head)
    dst
end

"""
    Workspace(max_samples, max_margin)

Scratch for one record: the carrier-wiped samples, the code replica (which is
`2·margin` samples longer than the record, because taps are taken by sliding the
replica rather than the samples), and the search's power spectrum.
"""
struct Workspace
    mixed::Vector{ComplexF32}
    replica::Vector{Float32}
    powers::Vector{Float64}
end

Workspace(max_samples::Integer, max_margin::Integer) = Workspace(
    Vector{ComplexF32}(undef, max_samples),
    Vector{Float32}(undef, max_samples + 2 * max_margin),
    Vector{Float64}(undef, 2 * max_margin + 1),
)

# Wipe the carrier off in place:
# `mixed[k] *= exp(i·sign·2π·(phase0 + freq·(k-1)/fs))`.
#
# `phase0` (in cycles) is not cosmetic. A hardware correlator's carrier NCO runs
# *continuously* across records; a reference that restarts at zero every record
# differs from it by a phase that steps by `freq × record length` each time —
# tens of thousands of cycles at GPS Dopplers, aliasing to an arbitrary offset
# per record. Magnitudes survive that, so a code-phase search still finds its
# peak, but every phase comparison between the two is destroyed. Carry the
# accumulated phase in and the two replicas are the same oscillator.
#
# The incremental rotation is renormalised periodically — a few thousand chained
# ComplexF32 multiplies otherwise shrink the replica by ~1e-4, which would read
# as an amplitude loss on the CPU side of the comparison.
function mix!(mixed, n::Integer, freq::Real, fs::Real, sign::Integer,
              phase0::Real = 0.0)
    w = ComplexF32(cis(sign * 2π * freq / fs))
    c = ComplexF32(cis(sign * 2π * phase0))
    @inbounds for k = 1:n
        mixed[k] *= c
        c *= w
        (k & 511) == 0 && (c /= abs(c))
    end
    mixed
end

# ±1 code replica of `total` samples starting at chip phase `cp0`, stepping
# `cps` chips per sample. Fixed point for the same reason the gateware uses it:
# a Float64 phase accumulated over a long run drifts, an integer one cannot.
function code_replica!(rep, code::Vector{Float32}, cp0::Real, cps::Real, total::Integer)
    fb = 32
    scale = Float64(Int64(1) << fb)
    len = length(code)
    step = round(Int64, cps * scale)
    limit = Int64(len) << fb
    ph = round(Int64, mod(Float64(cp0), Float64(len)) * scale)
    ph >= limit && (ph -= limit)
    @inbounds for k = 1:total
        rep[k] = code[(ph>>fb)+1]
        ph += step
        ph >= limit && (ph -= limit)
    end
    rep
end

# One correlator tap: the replica slid `j` samples later in code phase.
function tap(mixed, rep, n::Integer, j::Integer)
    re = 0.0
    im = 0.0
    @inbounds @simd for k = 1:n
        x = mixed[k]
        c = Float64(rep[k+j])
        re = muladd(Float64(real(x)), c, re)
        im = muladd(Float64(imag(x)), c, im)
    end
    complex(re, im)
end

"""
    correlate_epl(ws, ring, s0, n; code, cp_start, cps, freq, fs, sign, shift)
        -> (late, prompt, early)

Correlate `[s0, s0+n)` against `code` starting at chip phase `cp_start`, having
mixed out `freq`. `shift` is the prompt→Early offset **in whole samples**, which
is how both `Tracking` and the gateware quantise the Early/Late spacing, so the
CPU taps land exactly where the device's do. `phase0` is the carrier replica's
phase in cycles at `s0`: pass the running phase of an NCO mirroring the device's
(see `mix!`), or leave it at zero when only magnitudes are wanted.
"""
function correlate_epl(ws::Workspace, ring::SampleRing, s0::Integer, n::Integer;
                       code::Vector{Float32}, cp_start::Real, cps::Real,
                       freq::Real, fs::Real, sign::Integer, shift::Integer,
                       phase0::Real = 0.0)
    fetch!(ws.mixed, ring, s0, n)
    mix!(ws.mixed, n, freq, fs, sign, phase0)
    code_replica!(ws.replica, code, cp_start - shift * cps, cps, n + 2shift)
    (tap(ws.mixed, ws.replica, n, 0),
     tap(ws.mixed, ws.replica, n, shift),
     tap(ws.mixed, ws.replica, n, 2shift))
end

"""
    accumulate_offsets!(acc, ws, ring, s0, n; code, cp_start, cps, freq, fs, sign,
                        margin, phase0) -> acc

Add one record's power over replica offsets `-margin … +margin` samples into
`acc` (length `2·margin+1`), the offsets measured from `cp_start`.

Accumulated across records because a single 1 ms integration cannot localise a
peak: at 40 dBHz its peak stands only ~11× above the noise floor, and the
largest of 129 noise bins is regularly a good fraction of that, so the argmax of
one record is frequently just noise — an alignment search built on it walks the
reference off the satellite while reporting a plausible-looking ratio. Summing
`k` records leaves the ratio where it is and shrinks the floor's spread by
`√k`, which is what makes the argmax trustworthy. Every record brings its own
`cp_start`, so the offsets stay comparable while the code phase advances.
"""
function accumulate_offsets!(acc, ws::Workspace, ring::SampleRing, s0::Integer,
                             n::Integer; code::Vector{Float32}, cp_start::Real,
                             cps::Real, freq::Real, fs::Real, sign::Integer,
                             margin::Integer, phase0::Real = 0.0)
    code_replica!(ws.replica, code, cp_start - margin * cps, cps, n + 2margin)
    fetch!(ws.mixed, ring, s0, n)
    mix!(ws.mixed, n, freq, fs, sign, phase0)
    @inbounds for j = 0:2margin
        acc[j+1] += abs2(tap(ws.mixed, ws.replica, n, j))
    end
    acc
end

"""
    peak_offset(acc, margin) -> (offset_samples, peak_over_median, peak_over_centre)

Where an accumulated scan peaks, how far above its own noise floor, and how far
above the offset the scan was centred on. The last one is what says whether
moving the reference is an improvement or a coin toss.
"""
function peak_offset(acc, margin::Integer)
    k = argmax(acc)
    floor_power = _median(acc)
    centre = acc[margin+1]
    (k - 1 - margin,
     floor_power > 0 ? acc[k] / floor_power : 0.0,
     centre > 0 ? acc[k] / centre : Inf)
end

"""
    search_offset(ws, ring, s0, n; code, cp_start, cps, freq, fs, signs, margin)
        -> (offset_samples, peak_over_median, sign)

Slide the replica ±`margin` whole samples around `cp_start` and return the
offset of the peak, how far it stands above the median of the scan, and which
of `signs` produced it.

Two jobs in one pass. The offset is the residual host↔device sample-mapping
error — the thing a comparison against a reference must not silently absorb,
because a reference that is not correlating at all makes the device look good.
The sign vote settles the mixing convention empirically instead of inheriting
it: get it wrong and the CPU side loses all of its correlation gain, and nothing
else in the output would say so.
"""
function search_offset(ws::Workspace, ring::SampleRing, s0::Integer, n::Integer;
                       code::Vector{Float32}, cp_start::Real, cps::Real,
                       freq::Real, fs::Real, signs = (1, -1), margin::Integer = 64,
                       phase0::Real = 0.0)
    code_replica!(ws.replica, code, cp_start - margin * cps, cps, n + 2margin)
    best_offset, best_ratio, best_sign = 0, 0.0, first(signs)
    powers = ws.powers
    length(powers) >= 2margin + 1 || resize!(powers, 2margin + 1)
    for sign in signs
        fetch!(ws.mixed, ring, s0, n)
        mix!(ws.mixed, n, freq, fs, sign, phase0)
        @inbounds for j = 0:2margin
            powers[j+1] = abs2(tap(ws.mixed, ws.replica, n, j))
        end
        scan = view(powers, 1:(2margin+1))
        k = argmax(scan)
        floor_power = _median(scan)
        ratio = floor_power > 0 ? scan[k] / floor_power : 0.0
        if ratio > best_ratio
            best_offset, best_ratio, best_sign = k - 1 - margin, ratio, sign
        end
    end
    (best_offset, best_ratio, best_sign)
end

# Local median so the module stays dependency-free (Statistics is not in the
# examples environment's minimal path for the offline scripts).
function _median(v::AbstractVector)
    s = sort(collect(v))
    n = length(s)
    n == 0 && return 0.0
    isodd(n) ? Float64(s[(n+1)÷2]) : (Float64(s[n÷2]) + Float64(s[n÷2+1])) / 2
end

end # module
