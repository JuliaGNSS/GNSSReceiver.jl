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

To watch this unfold live instead of post-processing it, hand the data channel to the
[Graphical User Interface](@ref) rather than to [`collect_data`](@ref).
