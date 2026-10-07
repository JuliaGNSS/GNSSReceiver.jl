# ─────────────────────────────────────────────────────────────────────────────
# The delay-aware tracking loop's receiver-side pieces (GNSSReceiver.jl #107).
#
# The estimator itself — `NCOReferencedPLLAndDLL`, its per-satellite state and
# the per-record `step_loop` that attributes every record to the NCO word it
# really ran under and sizes the correction for the sample it will land at — is
# TrackingLoops', re-exported by Tracking, and it is stepped by the loop
# process (HardwareLoopCore.jl). Through the software receiver (`track!`)
# Tracking runs it with the chunk's own replica Doppler and no delay, i.e. as
# the conventional loop. What stays here is the acquisition pull-in range the
# receiver sizes its Doppler bins from, and a per-satellite shim the tests use
# to drive one loop by hand. See
# docs/plans/2026-09-14-delay-aware-hardware-loop.md for the measurements.
# ─────────────────────────────────────────────────────────────────────────────

# Pull-in is the FLL discriminator's, exactly as for the conventional assisted
# loop (see `carrier_doppler_pull_in_range` in receive.jl).
function carrier_doppler_pull_in_range(::NCOReferencedPLLAndDLL, signal::AbstractGNSSSignal)
    T = handover_coherent_integration_time(signal)
    uconvert(Hz, 1 / (4 * T))
end

# Per-satellite update with a channel's words and landing sample: Tracking's
# per-sat fold with the two hardware arguments filled in.
_nco_update_tracked_sat(
    sat::TrackedSat,
    estimator::NCOReferencedPLLAndDLL,
    sampling_frequency,
    noise::Tuple,
    words,
    landing_sample::Int64,
) = Tracking._update_tracked_sat_doppler(
    sat,
    estimator,
    sampling_frequency,
    noise,
    words,
    landing_sample,
)
