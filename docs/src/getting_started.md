# Getting Started

This page introduces the building blocks of GNSSReceiver.jl and shows the three ways to
feed it with samples: a **live SDR**, a **file already stored in the sample element
type**, and a **raw 8-bit offset-binary I/Q recording**.

## The pipeline in one picture

At the centre of the package is [`receive`](@ref). It consumes a *measurement channel* —
a lock-free channel of sample matrices (one column per antenna) — and returns a *data
channel* of [`ReceiverDataOfInterest`](@ref GNSSReceiver.ReceiverDataOfInterest)
snapshots, one per processed chunk:

```
measurement channel ──▶ receive(...) ──▶ data channel ──┬─▶ get_gui_data_channel ─▶ gui
   (raw samples)                    (PVT + per-sat CN0)  ├─▶ save_data   (to a JLD2 file)
                                                         └─▶ collect_data (to a Vector)
```

Everything runs on spawned tasks, so `receive` returns immediately and the pipeline
processes samples as they arrive. A data-channel snapshot carries:

- `sat_data` — a dictionary mapping each tracked PRN to its
  [`SatelliteDataOfInterest`](@ref GNSSReceiver.SatelliteDataOfInterest) (carrier-to-noise
  ratio, prompt correlator value, health flag);
- `pvt` — the current PVT solution (position, velocity, time, DOP, …);
- `runtime` — the elapsed *signal* time.

## Consuming the results

You typically hand the data channel to exactly one of three consumers:

| Consumer | What it does |
|---|---|
| [`get_gui_data_channel`](@ref) → [`gui`](@ref GNSSReceiver.gui) | Live terminal display (see [Graphical User Interface](@ref)). |
| [`save_data`](@ref) | Consume on a background task and write a JLD2 file when the stream ends. |
| [`collect_data`](@ref) | Block until the stream ends and return a `Vector` for offline analysis/plotting. |

Want more than one? Split the channel with `PipeChannels.tee` and attach a consumer to
each branch.

## 1. Receiving live from an SDR

The one-call convenience [`gnss_receiver_gui`](@ref) opens a SoapySDR device, configures
it, runs [`receive`](@ref) and shows the GUI:

```julia
using SoapyRTLSDR_jll          # or SoapyLMS7_jll, SoapyBladeRF_jll, SoapyUHD_jll, …
using GNSSReceiver, GNSSSignals, Tracking, SoapySDR, Unitful

gnss_receiver_gui(;
    system = GPSL1CA(),
    sampling_freq = 2e6u"Hz",
    acquisition_time = 4u"ms",     # longer ⇒ more acquisition sensitivity, more compute
    run_time = 40u"s",
    num_ants = Tracking.NumAnts(1),
    dev_args = first(Devices()),   # pick the first attached device
)
```

To record raw samples now and process them later, use [`gnss_write_to_file`](@ref).

## 2. Replaying a file stored in the sample element type

If your recording already holds samples in the element type you want (e.g. an
`Complex{Int16}` dump from [`gnss_write_to_file`](@ref)), read it with
[`read_files`](@ref). Pass one path per antenna channel:

```julia
using GNSSReceiver, GNSSSignals, Unitful
using Unitful: Hz, ms

system = GPSL1CA()
sampling_freq = 5e6u"Hz"
files = map(i -> "antenna$i.dat", 1:4)     # one file per antenna channel

# The chunk length must be an integer number of milliseconds. It sets how much signal
# each acquisition attempt sees: longer ⇒ higher acquisition sensitivity, more compute.
num_samples = Int(upreferred(sampling_freq * 4ms))

measurement_channel = read_files(files, num_samples; type = Complex{Int16})

data_channel = receive(
    measurement_channel,
    system,
    sampling_freq;
    num_ants = NumAnts(4),
    max_meas = 2^11,               # required for Complex{Int16}; see below
)

gui_channel = get_gui_data_channel(data_channel)
GNSSReceiver.gui(gui_channel)      # watch the GUI update live
```

!!! note "`max_meas` for integer samples"
    `Complex{Int16}` inputs are routed to Tracking's fast **integer**
    downconvert-and-correlator, which needs `max_meas` — the front end's full-scale, i.e.
    the largest `|real|`/`|imag|` any sample takes (e.g. `2^11` for a 12-bit ADC). It has
    no default because under-declaring it silently corrupts the correlation. Float element
    types (e.g. `ComplexF32`) use the general backend and ignore `max_meas`.

## 3. Reading a raw 8-bit offset-binary recording

Many SDRs and public sample sets store samples as interleaved **8-bit unsigned
offset-binary** I/Q bytes (128 ≈ zero). [`read_uint8_iq_file`](@ref) reads that format
directly, recentring each byte on 128 and emitting `Complex{Int16}` baseband samples — so
you don't have to write the byte-unpacking loop yourself:

```julia
using GNSSReceiver, GNSSSignals, Unitful

sampling_freq = 2.048e6u"Hz"
num_samples = Int(upreferred(sampling_freq * 4u"ms"))

measurement_channel = read_uint8_iq_file("RTLSDR_Bands-L1.uint8", num_samples)

data_channel = receive(
    measurement_channel,
    GPSL1CA(),
    sampling_freq;
    max_meas = 2^7,                # bytes recentred on 128 ⇒ |sample| ≤ 128
)

results = collect_data(data_channel)   # gather the whole run for analysis
```

The [Worked Example (Real Data)](@ref) uses exactly this path on a public recording and
plots a real fix.
