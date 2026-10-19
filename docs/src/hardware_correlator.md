# Hardware Correlator

```@meta
CurrentModule = GNSSReceiver
```

With a hardware correlator, the downconversion and correlation of every tracked
satellite run on an FPGA, and the receiver splits into two processes:

- the **loop process** sits next to the device, reads every correlator record,
  closes the carrier and code loops and writes the NCO words back within a
  millisecond — allocation-free and GC-free, so nothing can make it miss that
  deadline. It is built on
  [HardwareLoopCore.jl](https://github.com/JuliaGNSS/HardwareLoopCore.jl) and is
  typically a trimmed executable (`juliac --trim=safe`);
- **this receiver** keeps everything that may allocate: acquisition on the raw
  samples, the lock detectors, decoding and the PVT solution.

The two share one
[HardwareLoopProtocol.jl](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl)
segment, a file under `/dev/shm`: the receiver sends arm and release commands,
the loop process publishes per hardware channel the records, the navigation
bits and the epoch states (Dopplers, code phase, carrier phase and sync state at
a fold boundary common to all channels).

```julia
using GNSSReceiver, GNSSSignals, Unitful

loop = HardwareLoop("/dev/shm/gnss-loop-m2sdr0"; spawn = `gnss_loop --csr=… --antenna=2`)
data = receive(loop, raw_channel, GPSL1CA(), 4e6u"Hz";
               device_sample_origin = () -> origin_of(raw_channel))
for d in data
    isnothing(d.pvt.time) || println(d.pvt.position)
end
```

`raw_channel` is the device's raw sample stream (a `SignalChannel` of one
antenna). The receiver acquires on it and arms each detected satellite with the
code phase it had at the *device* sample the acquisition window started at,
which is why it needs `device_sample_origin`: the device's sample index of the
stream's first sample. A device driver that tags its raw stream (the
LiteX-M2SDR's does) knows it exactly; the stream must then keep host sample `k`
at device sample `origin + k`, e.g. by filling a lost buffer with zeros.

The output is the same [`ReceiverDataOfInterest`](@ref) as the software
receiver's, so [`get_gui_data_channel`](@ref), [`save_data`](@ref) and the
dashboard work unchanged.

## What happens per chunk

1. **Acquisition** runs on the CPU in a task of its own, every `acquire_every`,
   for the PRNs not tracked yet, so the events keep draining while it searches.
   Its Doppler span (`acquisition_doppler_coverage`, 20 kHz) is wider than the
   software receiver's because a hardware front end's oscillator offset adds to
   the satellites' Doppler.
   The search fills the default threadpool for seconds, so start Julia with an
   interactive thread for the chunk loop (`julia -t auto,1`), which then keeps
   up with the raw stream while it searches — and one more if the raw stream's
   reader holds one of its own (GNSSM2SDR's does: `julia -t auto,2`).
   `close(loop)` waits for a search still in flight.
2. **Events**: each satellite's records feed its lock detectors — on the device's
   record time, not on the arrival of raw chunks, which comes in bursts whenever
   this process has been held up; its bits are queued with the device sample
   they completed at; its epoch states are kept.
3. **Lock**: a satellite whose detectors lose it, or whose records stop for a
   second, is released and reacquired later. One hardware channel per band is
   the loop's noise reference, which every C/N₀ is measured against.
4. **Navigation**: every `pvt_update_interval` the decoders are brought up to a
   fold boundary B that every channel has passed — every bit that completed by B
   has then been published — and the PVT is solved from the epoch states at B,
   so all code phases refer to one reception instant.

## Limitations

- One band, one antenna, plain (non-combined) signals of one type for now, and
  scalar tracking (vector tracking in the loop process is HardwareLoopCore#15).
- The loop's tap layout is the default correlator's for each signal.

## API

```@docs
HardwareLoop
receive(::HardwareLoop, ::SignalChannel, ::Any, ::Any)
```
