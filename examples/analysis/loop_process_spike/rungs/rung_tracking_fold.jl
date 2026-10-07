# Milestone 0, the estimator rung: Tracking's per-record fold — discriminators,
# the FLL-assisted loop filter, the C/N0 estimator, the bit buffer with its
# sync detector — stepped over synthetic records, exactly the code
# `TrackingLoops.jl` extracts. Built with the whole Tracking dependency graph;
# the verifier's answer here is the dependency question of Q3.

using Tracking
using GNSSSignals
using StaticArrays
using Unitful
using Unitful: Hz, dBHz

function fold(steps::Int)
    signal = GPSL1CA()
    fs = 4e6Hz
    sat = TrackedSat(signal, 7, 0.0, 100.0Hz)
    # One measured noise density, ready: the pair the fold threads per signal.
    noise = ((1.0e-6 / Hz, true),)
    for k = 1:steps
        # A carrier pulling in from 30 Hz off with a data bit flip every 20
        # records, so the bit buffer has an edge to find.
        phase = 0.8 * exp(-k / 300)
        sign = isodd(div(k - 1, 20)) ? -1.0 : 1.0
        p = 2000.0 * sign * cis(phase)
        output = CorrelatorOutput(
            EarlyPromptLateCorrelator(SVector{3,ComplexF64}(0.5p, p, 0.5p), 0.5),
            4000,
            4000k,
        )
        Tracking.append_correlator_output!(sat, output)
        sat = Tracking._update_tracked_sat_doppler(sat, fs, noise)
    end
    sat
end

function (@main)(args::Vector{String})::Cint
    sat = fold(2000)
    Core.println(Core.stdout, round(Int, 1000 * ustrip(Hz, get_carrier_doppler(sat))))
    Core.println(Core.stdout, round(Int, 1000 * ustrip(Hz, get_code_doppler(sat))))
    cn0 = estimate_cn0(sat)
    Core.println(Core.stdout, round(Int, 10 * log10(ustrip(Hz, Unitful.linear(cn0)))))
    Core.println(Core.stdout, has_bit_or_secondary_code_been_found(sat) ? 1 : 0)
    return Cint(0)
end

