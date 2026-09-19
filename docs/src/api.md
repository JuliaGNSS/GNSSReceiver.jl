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
what an adapter package and its gateware have to satisfy.

```@docs
AbstractHardwareCorrelatorSDR
HardwareCorrelatorLink
CorrelatorDump
num_correlator_taps
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
advance_code_phases!
anchor_bit_phases!
restart_lost_bit_clocks!
take_bit_clock_restart!
anchor_secondary_phases!
GNSSReceiver.is_secondary_code_removed
coherent_integration_blocks
coherent_integration_periods
allows_partial_primary_records
primary_code_wraps
primary_code_block_phase
record_integration_periods
GNSSReceiver.RecordBlockSpan
advance_tracking!
flush_partial_records!
push_nco_updates!
estimate_dopplers!
```

### Capabilities and channel configuration

```@docs
HardwareCorrelatorCapabilities
GNSSReceiver.LEGACY_GPS_L1CA_CAPABILITIES
hardware_capabilities
HardwareChannelConfig
validate_hardware_configuration
check_hardware_support
GNSSReceiver.hardware_support_error
GNSSReceiver.wire_tap_slots
GNSSReceiver.DEFAULT_MAX_INTEGRATION_TIME
replica_code_amplitude
supports_secondary_code_wipeoff
GNSSReceiver.requested_secondary_code_mode
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
GNSSReceiver.receiver_sampling_frequency
GNSSReceiver.channel_band_id
GNSSReceiver.channel_sampling_frequency
```

### The delay-aware tracking loop

```@docs
NCOReferencedPLLAndDLL
GNSSReceiver.SatNCOReferencedPLLAndDLL
NCOTimeline
mean_nco_word
FixedNCOWord
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
