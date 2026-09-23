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

See the [hardware-correlator contract](@ref "Hardware-correlator contract") for
what a device package and its loop process have to satisfy.

```@docs
AbstractHardwareCorrelatorSDR
RemoteHardwareLoop
receive(::RemoteHardwareLoop, ::Any, ::Any)
device_sample_origin
raw_sample_channel
num_hardware_channels
correlator_gain
is_observation_gap
take_bit_clock_restart!
advance_tracking!
```

### Capabilities

```@docs
HardwareCorrelatorCapabilities
GNSSReceiver.LEGACY_GPS_L1CA_CAPABILITIES
hardware_capabilities
validate_hardware_configuration
check_hardware_support
GNSSReceiver.hardware_support_error
GNSSReceiver.wire_tap_slots
GNSSReceiver.DEFAULT_MAX_INTEGRATION_TIME
replica_code_amplitude
supports_secondary_code_wipeoff
```

### RF bands, inputs and the receiver timebase

```@docs
GNSSReceiver.HardwareBandRoute
GNSSReceiver.HardwareBandPlan
GNSSReceiver.hardware_band_plan
GNSSReceiver.band_rf_input
GNSSReceiver.band_device_index
GNSSReceiver.band_hardware_channels
GNSSReceiver.clock_synchronization
GNSSReceiver.reference_band
GNSSReceiver.reference_sampling_frequency
GNSSReceiver.band_ids
GNSSReceiver.band_route
GNSSReceiver.band_sampling_frequency
GNSSReceiver.receiver_timebase_scale
GNSSReceiver.to_receiver_samples
GNSSReceiver.to_band_samples
GNSSReceiver.band_plan_error
GNSSReceiver.band_bank_error
```

### The delay-aware tracking loop

The estimator a hardware correlator's loops run — `NCOReferencedPLLAndDLL`, its
per-satellite state, the `NCOTimeline` of what a device NCO ran, and the
per-record `step_loop` that attributes a record to the word it really ran under
— lives in [TrackingLoops.jl](https://github.com/JuliaGNSS/TrackingLoops.jl) and
is stepped by the loop process. Tracking.jl re-exports it, and the software
receiver runs it with the chunk's own replica word and no landing sample, which
makes it the conventional loop to the bit. Its documentation is TrackingLoops'.

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
