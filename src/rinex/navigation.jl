# The navigation file: the ephemerides of the satellites being tracked and the header
# records broadcast alongside them. Turning a decoded message into a RINEX record — units,
# issue-of-data and health words, the week counts — is RINEXParser's, through its GNSSDecoder
# extension (`rinex_ephemeris`, `add_header_records!`); this side only picks the decoders and
# supplies the observation's year.

# Every tracked satellite's current ephemeris, plus the navigation header assembled from
# whatever ionosphere, time-system and leap-second records have been decoded so far.
# RINEXParser deduplicates the ephemerides, so offering all of them every time is the
# documented way to write each one exactly once.
function build_nav_records(logger::RinexLogger, receiver_state)
    ephemerides = Vector{Union{GPSEphemeris,GalileoEphemeris,BeiDouEphemeris}}()
    header = RinexNavHeader(; satellite_system = nav_satellite_system(logger.systems))
    isnothing(logger.config.nav_file) && return ephemerides, header
    for (group_key, sat_states) in pairs(receiver_state.receiver_sat_states)
        haskey(logger.layouts, group_key) || continue
        for sat_state in sat_states
            decoder = sat_state.decoder
            # A message RINEX 3.05 has no record for is reported as soon as it is seen; a
            # partially decoded one will have its record once it is complete.
            if !has_rinex_ephemeris(decoder)
                warn_unsupported_ephemeris(logger, group_key)
                continue
            end
            is_decoding_completed_for_positioning(decoder) || continue
            # The record's week is the decoder's (GNSSDecoder's `get_week`), the GPS L1 C/A
            # rollover resolved against the observation's year as for the PVT solution, so
            # a written ephemeris and the fix never disagree about the cycle a recording is
            # in. A message that has not decoded its week yet has no record.
            (; approximate_year) = logger
            ephemeris = rinex_ephemeris(decoder; approximate_year)
            isnothing(ephemeris) || push!(ephemerides, ephemeris)
            add_header_records!(header, decoder; approximate_year)
        end
    end
    ephemerides, header
end

function warn_unsupported_ephemeris(logger::RinexLogger, group_key::Symbol)
    group_key in logger.warned_signals && return nothing
    push!(logger.warned_signals, group_key)
    @warn "RINEX 3.05 has no navigation record for the message decoded on $group_key, so " *
          "its ephemerides are not written (see `RINEXParser.rinex_ephemeris` for the " *
          "messages it has one for). Observations of $group_key are unaffected."
    nothing
end
