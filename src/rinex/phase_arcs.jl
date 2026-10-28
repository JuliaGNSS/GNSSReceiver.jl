# Phase arcs
#
# Per chunk, each tracked satellite either continues its arc, starts a new one, or has none:
#
#   1. `phase_measurable`: whether the carrier loop measures a phase on this chunk at all.
#      Where it does not, the satellite gets no arc.
#   2. `continues_arc`: whether the existing arc can take this chunk. It cannot after the
#      satellite left the vector loop, or where the carrier's polarity contradicts the arc's.
#   3. Otherwise a new arc starts, as long as the carrier loop is in phase lock.
#
# The polarity an arc goes by is `carrier_polarity`, counted for the arc only once the
# decoder synchronised within it (`arc_polarity`).

"""
    carrier_polarity(track_state, group_key, prn, layout::RinexSignalLayout, decoder) -> Int8

The polarity of the incoming carrier against the replica: `1` where the replica is on it,
`-1` where the Costas loop locked half a cycle off it, `0` where it is not known.

A pilot with a secondary code gives it directly: the sign its sync read off the prompts for
the known overlay. Otherwise it is the decoder's bit polarity times the sign the bit buffer's
sync locked at, as the bits the decoder is fed already carry that sign; both have to be
known. In the vector loop the prompts are decoded on the replica plus the residual PLL's
correction, so the same rule holds for the beat phase that includes it. The convention is
GNSSDecoder's, a positive soft bit for a 0 bit.
"""
function carrier_polarity(track_state, group_key, prn, layout::RinexSignalLayout, decoder)
    if is_dataless_with_secondary_code(layout.signal)
        pilot = get_bit_buffer(track_state, group_key, prn, RANGING_SIGNAL_INDEX)
        return pilot.found ? Int8(pilot.polarity) : Int8(0)
    end
    isnothing(decoder.num_bits_since_sync) && return Int8(0)
    isnothing(get_broadcast_time_of_week(decoder)) && return Int8(0)
    bit_buffer = get_bit_buffer(track_state, group_key, prn, layout.data_index)
    bit_buffer.found || return Int8(0)
    Int8(bit_buffer.polarity * (decoder.is_shifted_by_180_degrees ? -1 : 1))
end

# The `carrier_polarity` an arc started at `arc_start` can take, given that the decoder
# last synchronised at `synced_at` (both receiver time [s]). The decoder reads its polarity
# off each preamble and keeps it until the next one, so after the loop has been out of phase
# lock, which may have left it half a cycle further on, the polarity it still holds is the
# old one for up to a frame. The bit sync, too, holds the sign it locked at. So a data
# component's polarity counts for an arc only once a preamble was read within it; a pilot's
# is read off every prompt and is current.
arc_polarity(polarity, layout, synced_at, arc_start) =
    is_dataless_with_secondary_code(layout.signal) || synced_at >= arc_start ? polarity :
    Int8(0)

# The receiver time [s] the decoder of `key` last found a preamble at, as far as the chunks
# seen show it: its count of bits since the frame its time of week stamps restarts there.
# A decoder first seen synchronised counts as synchronising now; one not synchronised has
# none.
function note_decoder_sync!(logger, key, decoder, runtime)
    bits = decoder.num_bits_since_sync
    if isnothing(bits)
        delete!(logger.decoder_syncs, key)
        return -Inf
    end
    previous = get(logger.decoder_syncs, key, nothing)
    # Under vector tracking the count is moved by a bit now and then to follow the code
    # phase, which is no sync.
    synced_at = isnothing(previous) || bits < first(previous) - 1 ? runtime : last(previous)
    logger.decoder_syncs[key] = (bits, synced_at)
    synced_at
end

# How long a scalar carrier loop may report itself out of phase lock before its arc ends. Its
# verdict is that of an indicator averaged over 0.1 s, which a phase-locked GPS L1 C/A signal
# fails on about 1% of chunks at 30 dB-Hz, so a single failing chunk is no evidence of a slip;
# 100 ms of them in a row is.
const ARC_UNLOCKED_TIME = 0.1

# Whether the carrier loop measures a phase this chunk that the arc `accumulator` (or a new
# one, where it is `nothing`) can take. In the vector loop the residual PLL measures nothing
# once out of phase lock or once the signal is gone (`signal_present` false, when it is held
# on the filter's prediction though its own lock test may pass). A scalar loop keeps steering
# its replica onto the carrier, so only `ARC_UNLOCKED_TIME` out of phase lock ends its arc.
function phase_measurable(
    accumulator,
    phase_locked,
    in_vector_loop,
    signal_present,
    runtime,
)
    in_vector_loop && !signal_present && return false
    phase_locked && return true
    (isnothing(accumulator) || in_vector_loop) && return false
    accumulator.unlocked_time + (runtime - accumulator.runtime) < ARC_UNLOCKED_TIME
end

# Whether the arc `accumulator` continues with the carrier `polarity` measured now and the
# satellite `in_vector_loop`. Leaving the vector loop ends it, as the correction drops to zero
# and the phase steps by the residual; joining it does not, as the residual PLL starts at
# zero. A known polarity the arc does not have ends it — it would move the phase by half a
# cycle — unless no epoch was written yet, when `take_polarity!` latches it.
function continues_arc(accumulator, polarity, in_vector_loop)
    accumulator.in_vector_loop && !in_vector_loop && return false
    iszero(polarity) ||
        polarity == accumulator.polarity ||
        (iszero(accumulator.polarity) && isnan(accumulator.anchor))
end

# Latch a known polarity into an arc that `continues_arc` under it.
take_polarity!(accumulator, polarity) =
    iszero(polarity) || (accumulator.polarity = polarity)

# The C/N₀ [dB-Hz] below which the vector loop takes a satellite's signal for absent and holds
# its residual PLL on the filter's prediction; no satellite is in a vector loop otherwise.
vector_lock_cn0_threshold(estimator::VectorPLLAndDLL) =
    estimator.navigation.lock_cn0_threshold_dbhz
vector_lock_cn0_threshold(::AbstractDopplerEstimator) = -Inf

# Keep one accumulator per tracked, in-lock satellite up to date, and forget the arcs that
# ended so their next appearance starts a fresh, loss-of-lock-flagged one.
function update_carrier_phases!(logger::RinexLogger, receiver_state, runtime)
    track_state = receiver_state.track_state
    for (group_key, sat_states) in pairs(receiver_state.receiver_sat_states)
        layout = get(logger.layouts, group_key, nothing)
        isnothing(layout) && continue
        for sat_state in get_sat_states(track_state, group_key)
            prn = get_prn(sat_state)
            (haskey(sat_states, prn) && is_in_lock(sat_states[prn])) || continue
            key = (group_key, prn)
            decoder = sat_states[prn].decoder
            # Read on every chunk, so a preamble is seen whether or not an arc is running.
            synced_at = note_decoder_sync!(logger, key, decoder, runtime)
            estimator_state = get_doppler_estimator_state(sat_state)
            in_vector_loop = carrier_loop_stage(estimator_state) == VECTOR_LOOP
            phase_locked = TrackingLoops.phase_locked(estimator_state)
            signal_present =
                ustrip(estimate_cn0(sat_state, RANGING_SIGNAL_INDEX)) >=
                vector_lock_cn0_threshold(track_state.doppler_estimator)
            previous = get(logger.accumulators, key, nothing)
            # Left unadvanced, an arc ends below.
            measurable = phase_measurable(
                previous,
                phase_locked,
                in_vector_loop,
                signal_present,
                runtime,
            )
            measurable || continue

            # `get_carrier_phase` reports the replica phase in cycles, wrapped into one cycle,
            # as the accumulator and RINEX count it. In the vector loop the replica's phase is
            # the NCO's, which the navigation filter sets; the correction, in cycles too, brings
            # it onto the carrier.
            fractional = get_carrier_phase(sat_state)
            doppler = ustrip(Hz, get_carrier_doppler(sat_state))
            correction =
                get_carrier_phase_correction(
                    track_state,
                    group_key,
                    prn,
                    layout.sampling_freq,
                )
            polarity = carrier_polarity(track_state, group_key, prn, layout, decoder)

            arc_start = isnothing(previous) ? runtime : previous.start_runtime
            known_polarity = arc_polarity(polarity, layout, synced_at, arc_start)
            continues =
                !isnothing(previous) &&
                continues_arc(previous, known_polarity, in_vector_loop)
            if continues
                take_polarity!(previous, known_polarity)
                previous.unlocked_time =
                    phase_locked ? 0.0 :
                    previous.unlocked_time + (runtime - previous.runtime)
                advance!(
                    previous,
                    fractional,
                    doppler,
                    runtime,
                    logger.chunk_count,
                    layout.interm_freq,
                )
                previous.correction = correction
                previous.in_vector_loop = in_vector_loop
            elseif phase_locked
                logger.accumulators[key] = CarrierPhaseAccumulator(
                    fractional,
                    doppler,
                    runtime,
                    logger.chunk_count;
                    interm_freq = layout.interm_freq,
                    polarity = arc_polarity(polarity, layout, synced_at, runtime),
                    correction,
                    in_vector_loop,
                )
            end
        end
    end
    # Anything not advanced on this chunk lost lock, so its arc ended.
    filter!(pair -> last(pair).last_chunk == logger.chunk_count, logger.accumulators)
    nothing
end
