# Continuous carrier phase
#
# `Tracking` reports the replica's carrier phase wrapped into one cycle; RINEX wants a
# continuous cycle count. Between two chunks the replica advances by
# `(carrier_doppler + interm_freq) · Δt` cycles, which the loops' Doppler predicts to far
# better than half a cycle, so rounding the prediction onto the measured fractional change
# recovers the whole cycles.

"""
    CarrierPhaseAccumulator(fractional, doppler, runtime, chunk; interm_freq = 0.0,
                            polarity = 0, correction = 0.0, in_vector_loop = false)

Continuous carrier phase of one tracking arc of one satellite, in cycles, against the
receiver's oscillator (`update_clock_phase!` takes its drift out). Advanced once per processed
chunk by [`advance!`](@ref). When an arc ends and a new one starts is set out in
`phase_arcs.jl`; each new arc's first epoch carries the loss-of-lock indicator.
"""
mutable struct CarrierPhaseAccumulator
    # Last fractional replica phase [cycles] and carrier Doppler [Hz] — the state the next
    # advance is measured against.
    fractional::Float64
    doppler::Float64
    runtime::Float64
    # Index of the chunk this arc was last advanced on, so an arc that ended (the satellite
    # was not advanced on the current chunk) is recognised without comparing times.
    last_chunk::Int
    # Continuous Doppler-only phase advance since this arc started [cycles]: the replica
    # advance with the intermediate-frequency ramp taken back out.
    phase::Float64
    # The carrier beat phase [cycles] at the arc's start: the replica's fractional phase with
    # the IF ramp taken out, so it is measured against the receiver's oscillator as every
    # satellite's is. With `phase` it is the beat phase now.
    beat_phase::Float64
    # In the vector loop the navigation filter sets the carrier NCO, so the replica drifts off
    # the carrier: the residual PLL's estimate of that [cycles, every cycle counted] at the
    # latest chunk's end, `0` out of the vector loop. And whether the satellite was in it.
    correction::Float64
    in_vector_loop::Bool
    # The carrier's polarity against the replica, latched for the arc (`0` until known):
    # `-1` where the Costas loop locked half a cycle off it (see `carrier_polarity`).
    polarity::Int8
    # The RINEX phase ambiguity: a whole number of cycles (`NaN` until set), chosen on the
    # arc's first pseudorange to keep the written cycle counts in the range a receiver would
    # report. Whole, so the fractional part stays the measured one.
    anchor::Float64
    # True while the arc's first epoch has not been written yet, so that epoch carries the
    # loss-of-lock indicator.
    restarted::Bool
    # How long [s] the scalar carrier loop has reported itself out of phase lock in a row,
    # which ends the arc at `ARC_UNLOCKED_TIME` (see `phase_measurable`).
    unlocked_time::Float64
    # Receiver time [s] the arc started at: a polarity the decoder resolved before it does
    # not speak for it (see `arc_polarity`).
    start_runtime::Float64
end

CarrierPhaseAccumulator(
    fractional,
    doppler,
    runtime,
    chunk;
    interm_freq = 0.0,
    polarity = 0,
    correction = 0.0,
    in_vector_loop = false,
) = CarrierPhaseAccumulator(
    fractional,
    doppler,
    runtime,
    chunk,
    0.0,
    rem(fractional - interm_freq * runtime, 1.0, RoundNearest),
    correction,
    in_vector_loop,
    Int8(polarity),
    NaN,
    true,
    0.0,
    runtime,
)

# The arc's beat phase now [cycles]: the replica's, corrected onto the carrier in the vector
# loop, and half a cycle on where the loop locked off the carrier.
carrier_beat_phase(acc::CarrierPhaseAccumulator) =
    acc.beat_phase + acc.phase + acc.correction + (acc.polarity < 0 ? 0.5 : 0.0)

"""
    advance!(acc::CarrierPhaseAccumulator, fractional, doppler, runtime, chunk, interm_freq)

Advance `acc` to `chunk`, which ended at `runtime` with fractional replica phase `fractional`
(cycles) and carrier Doppler `doppler` (Hz). Returns `acc`.
"""
function advance!(
    acc::CarrierPhaseAccumulator,
    fractional,
    doppler,
    runtime,
    chunk,
    interm_freq,
)
    Δt = runtime - acc.runtime
    # Predicted replica advance over the chunk, trapezoidal in the Doppler so that a linearly
    # changing Doppler is integrated exactly.
    predicted = (0.5 * (acc.doppler + doppler) + interm_freq) * Δt
    # Measured fractional change; `round` supplies the whole cycles a wrapped phase cannot
    # carry (and absorbs the wrap itself, so no separate unwrapping step is needed).
    fractional_change = fractional - acc.fractional
    advance = fractional_change + round(predicted - fractional_change)
    # Only the Doppler tracks the changing range, so the IF ramp comes back out. The
    # oscillator's drift stays in, alike for every satellite (`update_clock_phase!`).
    acc.phase += advance - interm_freq * Δt
    acc.fractional = fractional
    acc.doppler = doppler
    acc.runtime = runtime
    acc.last_chunk = chunk
    acc
end
