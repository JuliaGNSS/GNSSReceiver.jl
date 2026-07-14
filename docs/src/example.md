# Worked Example (Real Data)

This page runs the **complete receiver pipeline on a real GNSS recording** and computes an
actual position fix. Every code block below is executed when the documentation is built,
so the numbers and plots you see were produced by GNSSReceiver.jl on live data — not
hand-written.

The recording is the public [ION SDR sample
data](https://sdr.ion.org/api-sample-data.html): 8-bit unsigned offset-binary I/Q at
2.048 MS/s, GPS L1, captured on 2017-09-10 in Oegstgeest, NL. It is the same signal used
by the package's integration test.

## Download the recording

We only need the first ~47 seconds to acquire the satellites and decode enough of the
navigation message for a fix, so we fetch a prefix of the file with an HTTP range request
(≈ 193 MB) rather than the full 60-second, ~246 MB recording.

```@example iondata
using Downloads

url = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
file = joinpath(tempdir(), "RTLSDR_Bands-L1_prefix.uint8")
nbytes = 193_000_000

if !isfile(file) || filesize(file) < nbytes
    Downloads.download(url, file; headers = ["Range" => "bytes=0-$(nbytes - 1)"])
end
filesize(file)
```

## Run the receiver

Reading the raw offset-binary file, running the pipeline and collecting the results is
three calls — [`read_uint8_iq_file`](@ref), [`receive`](@ref) and [`collect_data`](@ref):

```@example iondata
using GNSSReceiver, GNSSSignals, Unitful

sampling_freq = 2.048e6u"Hz"
system = GPSL1CA()
num_samples = Int(upreferred(sampling_freq * 4u"ms"))   # 4 ms chunks

measurement_channel = read_uint8_iq_file(file, num_samples)

data_channel = receive(
    measurement_channel,
    system,
    sampling_freq;
    acquisition_num_coherent_code_periods = 10,   # 10 ms coherent integration
    approximate_year = 2017,                       # resolves the GPS week rollover
    max_meas = 2^7,                                # bytes recentred on 128 ⇒ |sample| ≤ 128
)

results = collect_data(data_channel)
length(results)   # number of processed chunks
```

!!! note "Where `max_meas = 2^7` comes from"
    The raw file stores each I and Q value as an 8-bit **unsigned** byte (`0…255`) in
    *offset-binary* form: the zero-signal level sits at the middle of the range, `128`,
    rather than at `0`. So a byte of `128` means ≈ zero amplitude, `255` means maximum
    positive and `0` means maximum negative.

    [`read_uint8_iq_file`](@ref) turns that into normal signed baseband samples by
    **recentring on 128** — subtracting `128` from every byte — which maps the range
    `0…255` to `−128…+127`. The largest magnitude any component can then reach is `128`
    (from byte `0` → `−128`), i.e. `|real|, |imag| ≤ 128`.

    That maximum is exactly the front-end full-scale [`receive`](@ref) needs for
    `Complex{Int16}` samples, so we pass `max_meas = 2^7 = 128`. (Subtracting the integer
    `128` leaves a harmless ~0.5-LSB DC offset that the carrier loop removes; for exact
    midscale recentring in floating point use `read_uint8_iq_file(...; center = 127.5,
    type = ComplexF32)`, which uses the float backend and needs no `max_meas`.)

`results` is a `Vector` of [`ReceiverDataOfInterest`](@ref
GNSSReceiver.ReceiverDataOfInterest) snapshots. The last one is the final state of the
receiver:

```@example iondata
final = last(results)
final.runtime
```

## Carrier-to-noise ratio per satellite

The estimated CN0 of every tracked satellite — this is the same information the live GUI
shows as a bar chart:

```@example iondata
using UnicodePlots

prns = collect(keys(final.sat_data))
cn0s = [ustrip(sd.cn0) for sd in values(final.sat_data)]

barplot(
    string.(prns),
    round.(cn0s; digits = 1);
    xlabel = "CN0 [dBHz]",
    ylabel = "PRN",
    title = "Carrier-to-Noise-Density Ratio",
)
```

## The position fix

Ephemeris decoding takes about half a minute, so the first PVT solution appears roughly
35 seconds into the recording. Here is the final fix:

```@example iondata
using PositionVelocityTime: get_LLA

lla = get_LLA(final.pvt)

println("Time (TAI):        ", final.pvt.time)
println("Latitude:          ", lla.lat)
println("Longitude:         ", lla.lon)
println("Altitude:          ", round(lla.alt; digits = 1), " m")
println("Satellites in fix: ", length(final.pvt.sats))
println("Position DOP:      ", round(final.pvt.dop.PDOP; digits = 2))
```

The recording was made at roughly **52.177° N, 4.490° E** in Oegstgeest, Netherlands —
which is exactly what the receiver reports. You can drop those coordinates straight into a
map:

```@example iondata
println("https://www.google.com/maps/search/$(ustrip(lla.lat)),$(ustrip(lla.lon))")
```

## Getting data beyond the summary

[`receive`](@ref) reports a deliberately compact summary. Each snapshot is a
[`ReceiverDataOfInterest`](@ref GNSSReceiver.ReceiverDataOfInterest) with three fields —
`sat_data`, `pvt` and `runtime` — and each entry of `sat_data` is a
[`SatelliteDataOfInterest`](@ref GNSSReceiver.SatelliteDataOfInterest) carrying only the
CN0, the prompt correlator value and a health flag. The PVT solution, however, is the full
`PositionVelocityTime.PVTSolution`, so a lot is already available without any extra work:

```@example iondata
pvt = final.pvt
println("Velocity (ECEF):     ", round.(pvt.velocity; digits = 2), " m/s")
println("Clock drift:         ", pvt.relative_clock_drift)
println("GDOP / HDOP / VDOP:  ", round(pvt.dop.GDOP; digits = 2), " / ",
        round(pvt.dop.HDOP; digits = 2), " / ", round(pvt.dop.VDOP; digits = 2))
println("Prompt (PRN 5):      ", final.sat_data[5].prompt)
```

If you need a quantity that the summary does **not** carry — the raw carrier Doppler, code
phase or carrier phase of the tracking loops, or the decoded navigation data — pass your
own `extract` function to [`receive`](@ref). Each processed chunk, `receive` calls
`extract(receiver_state)` on the full [`ReceiverState`](@ref) and puts the result on the
channel; the default is [`default_data_of_interest`](@ref
GNSSReceiver.default_data_of_interest), which builds the `ReceiverDataOfInterest` you have
been using. Your function can return anything (the channel's element type is inferred from
it) — read the extra fields with the accessors from
[Tracking](https://github.com/JuliaGNSS/Tracking.jl) and
[GNSSDecoder](https://github.com/JuliaGNSS/GNSSDecoder.jl):

```@example iondata
using Tracking

# Emit per-satellite carrier Doppler and code phase — neither is in the default summary.
function doppler_and_code_phase(receiver_state)
    track_state = receiver_state.track_state
    sats = Dict(
        prn => (
            doppler = get_carrier_doppler(track_state, prn),
            code_phase = get_code_phase(track_state, prn),
        ) for prn in keys(get_sat_states(track_state))
    )
    return (; runtime = receiver_state.runtime, sats)
end

data_channel = receive(
    read_uint8_iq_file(file, num_samples),
    system,
    sampling_freq;
    acquisition_num_coherent_code_periods = 10,
    approximate_year = 2017,
    max_meas = 2^7,
    extract = doppler_and_code_phase,
)

custom = collect_data(data_channel)   # a Vector of our named tuples

last_snapshot = last(custom)
for prn in sort(collect(keys(last_snapshot.sats)))
    s = last_snapshot.sats[prn]
    println(
        "PRN ", lpad(prn, 2),
        "   carrier Doppler ", round(ustrip(u"Hz", s.doppler); digits = 1), " Hz",
        "   code phase ", round(s.code_phase; digits = 1),
    )
end
```

`extract` must be read-only and return an immutable value: it runs inside the tracking
loop on a `ReceiverState` that the next chunk mutates in place. The full state also holds
the per-satellite decoder states in `receiver_state.receiver_sat_states[1]`, from which
you can read the decoded ephemeris and health
(`GNSSReceiver.is_sat_healthy(receiver_state.receiver_sat_states[1][prn].decoder)`), so an
`extract` can surface those too.

Because your channel carries whatever `extract` returns, you can feed it straight to
[`collect_data`](@ref) (as above) or [`save_data`](@ref), just like the default one.

To watch this unfold live instead of post-processing it, hand the data channel to the
[Graphical User Interface](@ref) rather than to [`collect_data`](@ref).
