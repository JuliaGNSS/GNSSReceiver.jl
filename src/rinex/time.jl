# The GNSS time the RINEX output works in: system origins, times of week and the offsets
# between the constellations' time scales.

# The GPS time origin on the atomic (TAI) scale. Computed on each call, as the leap-second
# table cannot be baked in at precompile time.
gps_time_origin() = TAITime(from_utc(get_system_start_time(GPSL1CA()); scale = TAI))

# Seconds of GNSS system time since the GPS origin of a solution's TAI epoch. Only good for
# the week: at 1.2e9 s a `Float64` resolves 0.2 µs, 70 m of range, so everything precise
# works in times of week.
gps_seconds(time::TAITime) = time - gps_time_origin()

# Weeks from the start of GPS time to the start of `time_system`'s own week count: 1024 for
# Galileo, 1356 for BeiDou. Its origin is a UTC calendar time, a few leap seconds off the
# week boundary, which the rounding takes out.
week_offset(time_system) = round(
    Int,
    (get_system_start_time(time_system) - get_system_start_time(GNSSSignals.GPST())) /
    Millisecond(Week(1)),
)

# Difference of two times of week, resolved across the week boundary. A receive and a
# transmit instant lie a fraction of a second apart, so a difference approaching a whole
# week can only mean the week rolled over between them.
function week_difference(later, earlier)
    difference = later - earlier
    difference > SECONDS_PER_WEEK / 2 ? difference - SECONDS_PER_WEEK :
    difference < -SECONDS_PER_WEEK / 2 ? difference + SECONDS_PER_WEEK : difference
end

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
