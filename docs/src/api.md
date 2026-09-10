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

## Hardware correlators

```@docs
AbstractHardwareCorrelatorSDR
HardwareCorrelatorLink
CorrelatorDump
NCOUpdate
EPOCH_STROBE_CHANNEL
raw_sample_channel
correlator_dump_channel
nco_update_channel
num_hardware_channels
assign_channel!
release_channel!
assignment_start_sample
dropped_dump_count!
correlator_gain
epoch_strobe
is_epoch_strobe
is_observation_gap
restart_lost_bit_clocks!
take_bit_clock_restart!
coherent_integration_blocks
advance_tracking!
flush_partial_records!
```

## Consuming the results

```@docs
get_gui_data_channel
default_data_of_interest
save_data
collect_data
gui
```

## Data types

```@docs
ReceiverDataOfInterest
SatelliteDataOfInterest
```

## Lock detection

```@docs
AbstractLockDetector
LockDwell
CodeLockDetector
CarrierLockDetector
is_in_lock
has_pulled_in
is_ranging_ready
phase_lock_indicator
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
