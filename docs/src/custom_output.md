# Custom Receiver Output

By default [`receive`](@ref) emits a compact [`ReceiverDataOfInterest`](@ref
GNSSReceiver.ReceiverDataOfInterest) for every processed chunk: a `sat_data` dictionary
(each satellite's CN0, prompt correlator value and health), the current `pvt` solution and
the `runtime`. That covers most needs, and since `pvt` is the full
`PositionVelocityTime.PVTSolution` a lot more is already reachable from it (velocity, DOP,
clock drift — see [Worked Example (Real Data)](@ref)).

When you need something the summary does **not** carry — the raw carrier Doppler, code
phase or carrier phase of the tracking loops, or the decoded navigation data — pass your
own `extract` function to [`receive`](@ref).

## How it works

`receive` runs the acquire → track → decode → PVT pipeline and, each processed chunk,
calls `extract(receiver_state)` on the full [`ReceiverState`](@ref) and puts the result on
the returned channel. The default is [`default_data_of_interest`](@ref
GNSSReceiver.default_data_of_interest), which builds the `ReceiverDataOfInterest`. Because
your function decides the payload, the channel's element type is **inferred from what it
returns** (so the channel stays type-stable), and consumers like [`collect_data`](@ref)
and [`save_data`](@ref) work unchanged.

!!! warning "`extract` must be read-only and return an immutable value"
    It runs inside the tracking loop on a `ReceiverState` that the **next chunk mutates in
    place** (this in-place reuse is what makes the receiver allocation-free in steady
    state). Copy what you need into a fresh, immutable value — a named tuple, a `Dict`, or
    your own `struct` — and never return a reference into the state, or a later chunk will
    change it underneath your consumer.

## What's in the `ReceiverState`

The argument handed to `extract` is a [`ReceiverState`](@ref). The fields you will usually
read are:

| Field | What it holds | Useful accessors |
|---|---|---|
| `track_state` | Tracking's per-satellite loop state | `get_sat_states`, `get_carrier_doppler`, `get_code_phase`, `get_carrier_phase`, `estimate_cn0`, `get_prompt` (from [Tracking](https://github.com/JuliaGNSS/Tracking.jl)) |
| `receiver_sat_states[1]` | Per-PRN decoder state (dictionary keyed by PRN) | `GNSSReceiver.is_sat_healthy(…​.decoder)`, plus the decoded ephemeris in `…​.decoder` (from [GNSSDecoder](https://github.com/JuliaGNSS/GNSSDecoder.jl)) |
| `pvt` | Current PVT solution | fields of `PositionVelocityTime.PVTSolution` |
| `runtime` | Elapsed signal time | — |

`get_sat_states(track_state)` returns a `Dictionaries.Dictionary` keyed by PRN, so
iterating its `keys` gives you the currently tracked satellites.

## Example

The example below emits, per satellite, the raw carrier Doppler and code phase (from the
tracking loops) and the decoded health flag (from the decoder) — none of which is in the
default summary. It runs on the same public recording as the [Worked Example (Real
Data)](@ref); the download is skipped if that page already fetched it.

```@example custom
using Downloads, GNSSReceiver, GNSSSignals, Unitful, Tracking

# Same recording as the worked example (downloaded once, then cached on disk).
url = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
file = joinpath(tempdir(), "RTLSDR_Bands-L1_prefix.uint8")
nbytes = 193_000_000
if !isfile(file) || filesize(file) < nbytes
    Downloads.download(url, file; headers = ["Range" => "bytes=0-$(nbytes - 1)"])
end

sampling_freq = 2.048e6u"Hz"
system = GPSL1CA()
num_samples = Int(upreferred(sampling_freq * 4u"ms"))
nothing # hide
```

Define an `extract` that copies the quantities of interest into a plain named tuple:

```@example custom
function tracking_details(receiver_state)
    track_state = receiver_state.track_state
    decoders = receiver_state.receiver_sat_states[1]
    sats = Dict(
        prn => (
            doppler = get_carrier_doppler(track_state, prn),
            code_phase = get_code_phase(track_state, prn),
            healthy = GNSSReceiver.is_sat_healthy(decoders[prn].decoder),
        ) for prn in keys(get_sat_states(track_state))
    )
    return (; runtime = receiver_state.runtime, sats)
end
```

Pass it to [`receive`](@ref) and collect the run. Note the channel's element type is now
our named tuple, not a `ReceiverDataOfInterest`:

```@example custom
data_channel = receive(
    read_uint8_iq_file(file, num_samples),
    system,
    sampling_freq;
    acquisition_num_coherent_code_periods = 10,
    approximate_year = 2017,
    max_meas = 2^7,
    extract = tracking_details,
)

eltype(data_channel)
```

```@example custom
snapshots = collect_data(data_channel)   # a Vector of our named tuples
final = last(snapshots)

for prn in sort(collect(keys(final.sats)))
    s = final.sats[prn]
    println(
        "PRN ", lpad(prn, 2),
        "   carrier Doppler ", round(ustrip(u"Hz", s.doppler); digits = 1), " Hz",
        "   code phase ", round(s.code_phase; digits = 1),
        "   healthy ", s.healthy,
    )
end
```

## Persisting and plotting

Because the channel simply carries whatever `extract` returns, the usual consumers work
without change:

- [`collect_data`](@ref) gathers the run into a `Vector` (as above) for analysis.
- [`save_data`](@ref) writes it to a JLD2 file.
- [`get_gui_data_channel`](@ref) + [`gui`](@ref GNSSReceiver.gui) expect the default
  `ReceiverDataOfInterest`, so keep the default `extract` when you want the live GUI.

## Defining a typed payload

For a long-lived pipeline you may prefer a concrete `struct` over a named tuple — it
documents the payload and keeps the channel element type explicit:

```@example custom
struct DopplerSnapshot
    runtime::typeof(1.0u"s")
    dopplers::Dict{Int,typeof(1.0u"Hz")}
end

function doppler_only(receiver_state)
    track_state = receiver_state.track_state
    dopplers = Dict(
        prn => get_carrier_doppler(track_state, prn)
        for prn in keys(get_sat_states(track_state))
    )
    return DopplerSnapshot(receiver_state.runtime, dopplers)
end
nothing # hide
```

Passing `extract = doppler_only` then gives you a `channel` — and a
`collect_data` result — of `DopplerSnapshot`.
