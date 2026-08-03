# API Reference

```@meta
CurrentModule = GNSSReceiver
```

## Running the receiver

```@docs
receive
gnss_receiver_gui
ReceiverState
```

## Signals

```@docs
CombinedSignal
```

## Reading and recording samples

```@docs
read_files
read_uint8_iq_file
gnss_write_to_file
```

The lower-level `write_to_file` (write raw samples straight to disk) is re-exported from
[SignalChannels](https://github.com/JuliaGNSS/SignalChannels.jl); see that package for its
documentation.

## Consuming the results

```@docs
get_gui_data_channel
default_data_of_interest
save_data
collect_data
gui
```

## RINEX output

[`RinexConfig`](@ref) and the internals behind it are documented on the [RINEX Output](@ref)
page, together with how the epochs are timed and what ends up in the files.

## Data types

```@docs
ReceiverDataOfInterest
SatelliteDataOfInterest
```

## Lock detection

```@docs
AbstractLockDetector
CodeLockDetector
CarrierLockDetector
is_in_lock
```

## Beamforming

```@docs
EigenBeamformer
```

## Internals

```@docs
process
GNSSReceiver.VectorTracking
GNSSReceiver.VectorTrackingState
GNSSReceiver.VTStatus
```
