# The GNSS time the RINEX output works in: the raw transmit times and the receiver clock's
# drift. The time systems' weeks, times of week and leap seconds are GNSSSignals'.

"""
    uncorrected_transmit_time(decoder, signal, sat_state, time_system) -> Float64

Time of week the satellite's clock read when it transmitted the signal now arriving, from
the decoded time of week and the loops' code and carrier phase, on the count of
`time_system`.

This is the *raw* transmit time a RINEX pseudorange is measured against, not the corrected
one the PVT solution reports: a reader applies the broadcast satellite clock correction
itself, and would otherwise apply it twice. `PositionVelocityTime` keeps every transmit time
on its own constellation's count, and RINEX differences them against one receive time, so it
is brought onto the file's time system (`get_leap_second_offset`): zero between GPS and
Galileo time, but a BeiDou second of week reads 14 s lower, 4.2 million metres of range.
"""
uncorrected_transmit_time(decoder, signal, sat_state, time_system) =
    PositionVelocityTime.calc_uncorrected_transmit_time(
        PositionVelocityTime.SatelliteState(decoder, signal, sat_state),
    ) + ustrip(s, get_leap_second_offset(get_time_system(signal), time_system))

# The receiver clock's drift as a rate of time (s/s), which steers the epochs, Dopplers and
# phases onto system time; `PVTSolution` reports it in range units (m/s), like the bias.
relative_clock_drift(pvt) = ustrip(m / s, pvt.clock_drift) / SPEED_OF_LIGHT
