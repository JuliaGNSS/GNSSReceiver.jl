# GNSSReceiver.jl

A software-defined GNSS (Global Navigation Satellite System) receiver written in pure
Julia. GNSSReceiver.jl takes raw radio-frequency samples — streamed live from an
[SoapySDR](https://github.com/JuliaTelecom/SoapySDR.jl) device or replayed from a file —
and runs the full receiver pipeline:

```
samples ─▶ acquisition ─▶ tracking ─▶ navigation-bit decoding ─▶ PVT solution
```

The result is a **P**osition/**V**elocity/**T**ime fix that you can watch update live in
a terminal GUI, persist to disk, or post-process in Julia.

## Features

- End-to-end **acquire → track → decode → PVT** pipeline over a lock-free sample channel.
- Live reception from any **SoapySDR** device (RTL-SDR, LimeSDR, BladeRF, USRP, …).
- Offline **replay from files**, including raw 8-bit offset-binary I/Q recordings.
- **Multi-antenna** processing with eigen-beamforming.
- Automatic backend selection: `Complex{Int16}` samples use Tracking's fast integer
  downconvert-and-correlator; float samples use the general CPU backend.
- A live **terminal GUI** showing carrier-to-noise ratios, a satellite sky plot, and the
  computed position.
- Fully configurable acquisition, tracking, lock-detection and PVT parameters — see
  [Acquisition & Tracking Parameters](@ref).

## Installation

```julia
using Pkg
Pkg.add("GNSSReceiver")
```

Or from the Julia REPL:

```julia-repl
julia> ]
pkg> add GNSSReceiver
```

To receive from a real SDR you also need a SoapySDR driver for your device, e.g.
`SoapyRTLSDR_jll` for an RTL-SDR dongle.

## Quick start

Receive live from an SDR and show the GUI (see [Getting Started](@ref) for the full
walk-through):

```julia
using SoapyRTLSDR_jll          # your device's SoapySDR driver
using GNSSReceiver, GNSSSignals, Tracking, SoapySDR, Unitful

gnss_receiver_gui(;
    system = GPSL1CA(),
    sampling_freq = 2e6u"Hz",
    acquisition_time = 4u"ms",
    run_time = 40u"s",
    num_ants = Tracking.NumAnts(1),
    dev_args = first(Devices()),
)
```

Don't have an SDR handy? The [Worked Example (Real Data)](@ref) page downloads a public
GNSS recording and computes a real position fix — and it runs automatically every time
these docs are built.

## Where to go next

- [Getting Started](@ref) — the building blocks ([`receive`](@ref),
  [`get_gui_data_channel`](@ref), [`save_data`](@ref)) and how to feed the receiver from
  a device or a file.
- [Acquisition & Tracking Parameters](@ref) — every knob you can turn (coherent
  integration length, false-alarm probability, lock thresholds, correlator, …) and how
  to set it.
- [Worked Example (Real Data)](@ref) — a complete, runnable end-to-end fix on real data.
- [Custom Receiver Output](@ref) — emit your own per-chunk payload (raw carrier Doppler,
  code phase, decoded data, …) instead of the default summary.
- [Graphical User Interface](@ref) — the live terminal GUI.
- [API Reference](@ref) — every exported function.
