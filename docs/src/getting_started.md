# Getting Started

This page introduces the building blocks of GNSSReceiver.jl and shows the three ways to
feed it with samples: a **live SDR**, a **file already stored in the sample element
type**, and a **raw 8-bit offset-binary I/Q recording**.

Throughout, `receive`'s second argument is a *system*: a single GNSS signal such as
`GPSL1CA()`, a [`CombinedSignal`](@ref) pilot+data pair, or a **tuple** of these that
share one RF band (e.g. `(GPSL1CA(), GalileoE1B())`) — every constellation in the tuple is
acquired, tracked and decoded and all are fused into a single multi-GNSS PVT solution. The
examples below use a single system; see [Acquisition & Tracking Parameters](@ref) for the
multi-constellation and multi-band forms. Note that the full-CBOC `GalileoE1B()` needs a
sampling frequency above ~12.3 MHz — at lower rates use its BOC(1,1) approximation
`GalileoE1B_BOC11()` (details on the parameters page).

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

- `sat_data` — a `Dictionaries.Dictionary` mapping each tracked satellite, keyed by
  `(signal_id, prn)` (e.g. `(:GPSL1CA, 5)`), to its
  [`SatelliteDataOfInterest`](@ref GNSSReceiver.SatelliteDataOfInterest) (carrier-to-noise
  ratio, prompt correlator value, health flag). Keying by `(signal_id, prn)` lets the same
  PRN appear on several constellations or bands without colliding, and matches `pvt.sats`;
- `pvt` — the current PVT solution (position, velocity, time, DOP, …);
- `runtime` — the elapsed *signal* time.

## Consuming the results

You typically hand the data channel to exactly one of three consumers:

| Consumer | What it does |
|---|---|
| [`get_gui_data_channel`](@ref) → [`gui`](@ref GNSSReceiver.gui) | Live terminal display (see [Graphical User Interface](@ref)). |
| [`save_data`](@ref) | Consume on a background task and write a JLD2 file when the stream ends. |
| [`collect_data`](@ref) | Block until the stream ends and return a `Vector` for offline analysis/plotting. |

Want more than one? Split the channel with `SignalChannels.tee` and attach a consumer to
each branch (see [Graphical User Interface](@ref) for an example).

## 1. Receiving live from an SDR

The one-call convenience [`gnss_receiver_gui`](@ref) opens a SoapySDR device, configures
it, runs [`receive`](@ref) and shows the GUI:

```julia
using SoapyRTLSDR_jll          # or SoapyLMS7_jll, SoapyBladeRF_jll, SoapyUHD_jll, …
using GNSSReceiver, GNSSSignals, Tracking, SoapySDR, Unitful

gnss_receiver_gui(;
    system = GPSL1CA(),
    sampling_freq = 2e6u"Hz",
    chunk_time = 4u"ms",           # processing chunk length (tracking granularity)
    run_time = 40u"s",
    num_ants = Tracking.NumAnts(1),
    dev_args = first(Devices()),   # pick the first attached device
)
```

`system` here can also be a tuple of systems sharing one RF band, e.g.
`system = (GPSL1CA(), GalileoE1B())`, to receive several constellations at once.

To record raw samples now and process them later, use [`gnss_write_to_file`](@ref).

## 2. Replaying a file stored in the sample element type

If your recording already holds samples you can read with [`read_files`](@ref), pass one
path per antenna channel. Read them as `ComplexF32` so the default
`CPUThreadedDownconvertAndCorrelator()` handles them with no extra configuration:

```julia
using GNSSReceiver, GNSSSignals, Tracking, Unitful
using Unitful: Hz, ms

system = GPSL1CA()
sampling_freq = 5e6u"Hz"
files = map(i -> "antenna$i.dat", 1:4)     # one file per antenna channel

# The chunk length sets the processing/tracking granularity. A few milliseconds is a
# sensible default.
num_samples = Int(upreferred(sampling_freq * 4ms))

measurement_channel = read_files(files, num_samples; type = ComplexF32)

data_channel = receive(
    measurement_channel,
    system,
    sampling_freq;
    num_ants = NumAnts(4),
)

gui_channel = get_gui_data_channel(data_channel)
GNSSReceiver.gui(gui_channel)      # watch the GUI update live
```

!!! note "Sample element type and the correlator backend"
    `receive` picks its downconvert-and-correlator from the default
    `CPUThreadedDownconvertAndCorrelator()`, which works with float samples (e.g.
    `ComplexF32`). If a recording is stored in an integer type such as `Complex{Int16}`,
    read it as `ComplexF32` (`read_files(...; type = ComplexF32)` above) — no per-front-end
    full-scale value is needed.

## 3. Reading a raw 8-bit offset-binary recording

Many SDRs and public sample sets store samples as interleaved **8-bit unsigned
offset-binary** I/Q bytes (128 ≈ zero). [`read_uint8_iq_file`](@ref) reads that format
directly, recentring each byte and emitting baseband samples — so you don't have to write
the byte-unpacking loop yourself. Read as `ComplexF32` (with `center = 127.5` for exact
midscale recentring) so the default correlator backend applies:

```julia
using GNSSReceiver, GNSSSignals, Unitful

sampling_freq = 2.048e6u"Hz"
num_samples = Int(upreferred(sampling_freq * 4u"ms"))

measurement_channel =
    read_uint8_iq_file("RTLSDR_Bands-L1.uint8", num_samples; center = 127.5, type = ComplexF32)

data_channel = receive(measurement_channel, GPSL1CA(), sampling_freq)

results = collect_data(data_channel)   # gather the whole run for analysis
```

The [Worked Example (Real Data)](@ref) uses exactly this path on a public recording and
plots a real fix.
