# Hardware-correlator contract

```@meta
CurrentModule = GNSSReceiver
```

This page is the contract between GNSSReceiver and a *hardware correlator* — an
SDR whose FPGA downconverts and correlates on-device and streams correlator
dumps to the host, which runs only the loop filters and pushes NCO words back.
It is what an adapter package (the Julia side) and the gateware it drives have
to satisfy. The types themselves are documented in the
[API reference](@ref "Hardware correlators"); this page says what they *oblige*
you to do.

The split itself is described under [`AbstractHardwareCorrelatorSDR`](@ref): the
device's raw sample stream keeps driving acquisition, decoding, PVT and the
runtime clock exactly as in the software receiver, and only the per-chunk
correlator outputs change origin.

## 1. Declare what the device can do

```julia
GNSSReceiver.hardware_capabilities(sdr::MyDevice) = HardwareCorrelatorCapabilities(;
    signals = [:GPSL1CA, :GalileoE1B, :GalileoE1C],
    modulations = [:LOC, :CBOC],
    max_primary_code_length = 4092,
    code_frequency_limits = (1.023e6, 1.023e6),
    tap_layouts = [3, 5],
    max_tap_offset_chips = 1.0,
    num_antennas = 1,
    bands = [:L1],
    num_rf_inputs = 1,
    max_secondary_code_length = 1,
    reports_code_phase = true,
)
```

A device that implements nothing is taken to be
[`LEGACY_GPS_L1CA_CAPABILITIES`](@ref) — the GPS L1 C/A correlator this
interface was originally written against. That is the compatibility path: an
adapter written before this interface existed keeps working unchanged, and keeps
being described truthfully.

Declare every field honestly. An **under**-declared capability is one the
receiver will refuse to use; an **over**-declared one is a channel that arms and
never locks, which is far harder to attribute.

## 2. Nothing is armed before it is validated

[`receive`](@ref)`(::AbstractHardwareCorrelatorSDR, systems, sampling_freq; …)`
calls [`validate_hardware_configuration`](@ref) before it builds a link, before
it starts a task and before a single CSR is written. Every component of every
configured system is checked against the declaration above, and every problem is
reported in one `ArgumentError`:

```
MyDevice cannot track GalileoE1B on this hardware correlator:
  - the device has no code generator for GalileoE1B (it declares GPSL1CA)
  - the device cannot synthesise CBOC modulation (it declares LOC)
  - the primary code is 4092 chips, past the device's 1023-chip code memory
  - GalileoE1B is tracked with a 5-tap VeryEarlyPromptLateCorrelator, and the
    device's correlator bank produces 3-tap layouts
```

Use [`check_hardware_support`](@ref) for a single signal, and
[`GNSSReceiver.hardware_support_error`](@ref) when you want the reasons as a
string rather than an exception.

The same check runs again, per assignment, immediately before the device write
in `GNSSReceiver._assign!`. There it refuses the channel and counts it in the
link's `unsupported_signals` instead of throwing: that code runs on the chunk
path, where taking the receiver down would cost every other satellite its lock.

## 3. Arming a channel

The link calls

```julia
GNSSReceiver.assign_channel!(sdr, hw_channel, config::HardwareChannelConfig)
```

Implement this method for anything beyond GPS L1 C/A. A device that implements
only the older positional [`assign_channel!`](@ref) still works — GNSSReceiver
unpacks the configuration into it — but everything the configuration adds is
then dropped, which is correct only for the L1 C/A shape it was written for.

!!! warning "Give `assign_channel!` a concrete signature"
    An `assign_channel!(::MyDevice, args...; kwargs...)` catch-all is neither
    more nor less specific than GNSSReceiver's configuration shim, so the first
    handover raises an ambiguity `MethodError`.

[`HardwareChannelConfig`](@ref) carries:

| Field                                                              | The device's obligation                                                                                                          |
|:------------------------------------------------------------------ |:-------------------------------------------------------------------------------------------------------------------------------- |
| `signal`, `signal_index`, `group_key`, `prn`                       | Replicate *this* component of this satellite. A pilot/data pair occupies two channels differing only in `signal`/`signal_index`. |
| `carrier_doppler`, `code_doppler`, `code_phase`, `valid_at_sample` | The satellite at `valid_at_sample` on the host's raw-sample count; propagate to whatever sample you actually start on.           |
| `tap_sample_shifts`                                                | Program **exactly** these replica offsets, in whole input samples, latest first, prompt at zero.                                 |
| `el_sample_spacing`                                                | The Early-to-Late distance implied by the shifts; kept because it is the one number the legacy call carried.                     |
| `replica_amplitude`, `code_amplitude`                              | Declare, do not change: what the host divides out (see below).                                                                   |
| `secondary_code_mode`                                              | `:primary_only` — replicate the primary code only; the host removes the overlay (see below). `:wipeoff` is reserved and never requested. |
| `carrier_phase_offset`                                             | Informational. Do **not** add it (see below).                                                                                    |
| `band_id`, `sampling_freq`                                         | Which RF band, and the rate the shifts and sample counts are expressed in.                                                       |

### Why all the tap offsets, and not just the spacing

`Tracking`'s discriminators do not use the spacing the host programmed: they
recover it from the correlator handed back to them, through that correlator's
preferred code shifts. So the device has to reproduce the *host's* quantisation,
not its own. For a three-tap bank getting this wrong is a DLL loop-gain error
(~2.3 % at 4 MHz and a 0.5-chip preferred shift). For a five-tap bank it is
worse: the VE/VL distance enters the discriminator separately, and there is no
single number to re-derive it from. `tap_sample_shifts` removes the guesswork —
program the array.

## 4. Streaming dumps

Each completed integration becomes a [`CorrelatorDump`](@ref) on the device's
[`correlator_dump_channel`](@ref). The record is `isbits` so the ring stays
allocation-free; use integer accumulators and let `integrated_samples` do the
float normalisation on the host.

**Accumulator order is latest first.** For three taps that is
`[late, prompt, early]`; for five,
`[very late, late, prompt, early, very early]`. Building it in E/P/L order
inverts the DLL discriminator and the loop never converges.

**One link can carry both layouts.** Fix your stream's correlator type at the
*widest* layout the device produces — `VeryEarlyPromptLateCorrelator` if it does
five taps at all — and set `num_taps` on each record to how many leading slots
that channel actually filled ([`num_correlator_taps`](@ref)). The trailing slots
are never read and need not be zeroed. The host takes the leading `num_taps`
values and retags them with the tracked satellite's own correlator type and
spacing.

Nothing is invented in either direction. A record whose `num_taps` is not the
tap count of the correlator its satellite is tracked with is **dropped** and
counted in the link's `tap_layout_mismatches`: padding a three-tap record out to
five would hand `dll_disc` two accumulators that never saw a replica.
[`GNSSReceiver.wire_tap_slots`](@ref) is what the pre-arm validation compares
your stream's width against.

Also stream [epoch strobes](@ref epoch_strobe) at a fixed period, regardless of
what the channels are doing — without them the host's epoch clock stalls
whenever every channel falls silent.

Report `CorrelatorDump.code_phase` if the hardware can latch it. It is the
absolute pseudorange anchor; without it the host dead-reckons from the
acquisition seed, which tracks fine and ranges worse — and for a device that
dumps inside a code period (section 4a) it is not optional at all.

## 4a. Dump cadence, and codes longer than an integration

**The default contract is one record per primary code period**, and for almost
every signal that is also the right one: GPS L1 C/A's code period is 1 ms, GPS
L5's 1 ms, Galileo E1's 4 ms, GPS L2CM's 20 ms. The host's whole record
accounting — the navigation bit clock, the overlay counter, the coherent
pre-accumulation — is built on code blocks, and a device that dumps on the code
wrap hands it exactly those.

GPS L2CL breaks it. Its primary code is 767 250 chips at 511.5 kcps: **one code
period is 1.5 seconds**. A device that only dumps on the wrap would hand the
tracking loops one record, and take one NCO correction, every one and a half
seconds — which is not a tracking loop, it is an open loop with a heartbeat. So
for a code period longer than the receiver's `max_integration_time`
([`GNSSReceiver.DEFAULT_MAX_INTEGRATION_TIME`](@ref), 20 ms) the device has to be
able to dump *inside* a code period, and
[`HardwareCorrelatorCapabilities`](@ref)`.supports_partial_code_dumps` is where
it says so. The pre-arm validation refuses the combination otherwise, naming the
code period and the integration length, rather than arming a channel that
"tracks" at 0.67 Hz of update rate.

What a device producing such **partial-primary dumps** owes the host:

  - **Align the dump grid to the code-block boundary.** The dump interval must
    divide the primary code period, so that a record always ends on the code
    wrap when it reaches one. The host cuts its coherent accumulation on that
    boundary — it is what keeps a record a whole number of code blocks for every
    signal that has a navigation bit or an overlay grid to stay on — and a
    device whose dumps straddle the wrap leaves nothing to cut on. The host
    detects it, folds the record anyway and counts it in the link's
    `misaligned_dump_boundaries`.
  - **Report `CorrelatorDump.code_phase` on every record.** A record shorter than
    a code period cannot be placed on the code-block grid from its sample count
    alone, because the phase it starts at is not the boundary. The host
    re-anchors its running block phase
    ([`primary_code_block_phase`](@ref)) to the reported replica phase on every
    record that carries one, which is also what keeps a 1.5 s code from being
    dead-reckoned across a whole run.
  - **Keep the records tiling the sample axis.** Unchanged from the
    one-per-period contract, and it matters more, not less: the host tells a
    re-arm hole from a lost record by comparing the hole to *the channel's own
    record length*, so a stream of quarter-period records has quarter-period
    holes and each of them is a lost record.

The host side of this is [`coherent_integration_periods`](@ref), which sizes a
record in fractional code periods, and
[`allows_partial_primary_records`](@ref), which decides per signal whether a
record shorter than a code period may reach the loops at all. A short record is
never counted as a completed code period: [`primary_code_wraps`](@ref) counts
what the code did, not what the device streamed.

The converse extreme is Galileo E5a-QP, whose 330-chip primary code is 64.5 µs —
one record per code period would be fifteen thousand records a second. Nothing
is asked of the device there; the host folds to `Tracking`'s own stated ceiling
for the signal (one 2 ms, 31-block code cycle).

Neither the gateware nor the adapter side of this has landed:
[gnss-m2sdr#29](https://github.com/JuliaGNSS/gnss-m2sdr/issues/29) is the
configurable dump interval and the absolute code-phase metadata, and
[GNSSM2SDR.jl#8](https://github.com/JuliaGNSS/GNSSM2SDR.jl/issues/8) is the
adapter that configures it. Until they do, every device declares
`supports_partial_code_dumps = false` and GPS L2CL is refused before arming —
which is the honest outcome, and the one this section exists to make legible.

## 5. Amplitude normalisation, per band and per signal

The host brings every accumulator onto one scale — the scale its own software
correlator would have produced from the same samples — by dividing by

```
replica_amplitude × code_amplitude / GNSSSignals.get_code_amplitude(signal)
```

Both factors come from the device:

  - [`correlator_gain`](@ref)`(sdr, band_id)` — the amplitude of the *carrier*
    replica the gateware wipes off with, relative to a unit-amplitude replica.
    A device mixing with a ±127 sine/cosine table returns `127`. Declare it per
    band: a multi-band front end rarely has one gain chain, and the error is a
    flat `20·log10(g)` dB offset on that band's C/N₀ — exactly the kind of bias
    a lock-detector threshold silently absorbs.
  - [`replica_code_amplitude`](@ref)`(sdr, signal)` — the RMS amplitude of the
    *code* replica, on the scale `GNSSSignals.get_code_amplitude` reports. The
    default says the device reproduces the modelled code exactly, which is true
    for every ±1 code. Override it where the gateware approximates: a device
    replicating Galileo E1B with a plain ±1 BOC(1,1) replica has an amplitude of
    `1` where the modelled CBOC table has ≈ 19.92, and without the override the
    same satellite reads ~26 dB apart depending on which correlator produced it.

Both are read once per assignment and frozen into the channel's scale, so they
may be computed rather than stored.

The discriminators are ratios and cannot see any of this; the noise-referenced
C/N₀ estimator can, because it divides the prompt power by a floor measured
elsewhere. That is why it has to be declared rather than left at the default.

## 6. Carrier-phase conventions for components

A satellite's components share one carrier. The receiver locks the *driver*
component (the first of `tracking_signals`, i.e. the pilot for a pilot/data
pair) onto the real axis and de-rotates every other component by its own ICD
phase offset — `GNSSSignals.get_carrier_phase_offset`, e.g. `−π/2` for GPS L5-Q
against L5-I — so a quadrature component does not decode off the collapsed real
part.

That only works if the phase relationship survives into the accumulators. So:

  - Mix every channel of one band against **one common in-phase carrier
    reference**, at the band's carrier, with no per-component rotation.
  - `HardwareChannelConfig.carrier_phase_offset` states the component's offset
    against that reference. It is there to be *preserved*, not applied. A device
    that unavoidably rotates per component must remove exactly this value again
    before dumping.
  - Do not pre-combine components in hardware, and do not pre-combine antennas:
    beamforming is post-correlation on the host and adapts from the per-antenna
    prompt covariance, so an N-antenna device streams `SVector{N,Complex}`
    accumulators.

## 7. Secondary codes

**The host removes the overlay; the device replicates the primary code.** That
is the ownership decision, and
[`HardwareChannelConfig`](@ref)`.secondary_code_mode` is where it is stated per
channel: it is `:primary_only` for every device and every signal, and
[`GNSSReceiver.requested_secondary_code_mode`](@ref) is the one function that
decides it. A device is never asked to wipe an overlay, so the overlay is removed
exactly once — on the host, at the ingest, before the coherent pre-accumulation
and therefore before the discriminators, the C/N₀ estimator and the navigation
bit accumulation alike.

Why the host: an overlay's *phase* is not known when a channel is armed. It is
recovered by the bit/secondary-code sync detector, some way into tracking, from
the prompts themselves, and a device asked to wipe at the wrong phase cancels the
signal instead of accumulating it. Handing the job to the gateware therefore
needs more than a flag in the handover — it needs a *scheduled* command ("from
device sample `n`, overlay chip `k`") on the same sample-exact footing as an
[`NCOUpdate`](@ref), plus a way to read the device's overlay counter back so that
a lost record cannot leave the two ends disagreeing in silence. That contract is
not defined yet. The host's per-dump sign, meanwhile, is exact rather than
approximate: the device's replicas run continuously, so multiplying one
primary-period dump by its overlay chip *is* the correlation the device would
have produced with the overlay baked into its replica.

What the device owes this is therefore only what section 4 already asks for:
**one record per primary code period**. A record covering several code periods
has summed those periods' overlay chips inside the accumulator, and no single
sign takes them off again — the host detects that, stops removing rather than
guessing, and counts it. The same holds for a record that does not start where
the previous one ended: the overlay counter rides the record stream, so a lost
record, a duplicate or a counter step back drops the phase until the next
synchronisation re-seeds it. Nothing is ever wiped at a guessed phase, because a
wrong sign is invisible to everything downstream.

`max_secondary_code_length` therefore stays a *declaration* — the longest overlay
the gateware could wipe if it were asked — and `1`, the default, is not an error
and costs a device nothing. It is what [`supports_secondary_code_wipeoff`](@ref)
reads, and nothing requests it. Where it would become load-bearing is the
scheduled-wipeoff contract sketched above, which would additionally let a device
pre-accumulate across code periods; until then
[`coherent_integration_blocks`](@ref) does that on the host, across records the
host has wiped.

Removing secondary codes from hardware dumps after synchronisation is
[issue #132](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/132). The
host-side half of it has landed; the scheduled hardware-wipeoff contract has not,
and no live-hardware demonstration has been run.

## 8. Feedback

One [`NCOUpdate`](@ref) per assigned channel is pushed per folded epoch, each
scheduled at a named future sample. Read the warning in [`NCOUpdate`](@ref)
before implementing the apply path: a device that treats arming as a
single-register commit and lets a new update cancel a pending one silently opens
every loop, which cost a season of "tracks, then walks off and never decodes" in
issue #107.

Implement [`dropped_dump_count!`](@ref) if the gateware has a sticky overflow
status, and [`assignment_start_sample`](@ref) if arming is asynchronous.

## 9. Adapter checklist

  - [ ] [`raw_sample_channel`](@ref), [`correlator_dump_channel`](@ref),
    [`nco_update_channel`](@ref), [`num_hardware_channels`](@ref)
  - [ ] [`assign_channel!`](@ref) taking a [`HardwareChannelConfig`](@ref), with
    a concrete signature
  - [ ] [`release_channel!`](@ref)
  - [ ] [`hardware_capabilities`](@ref), declared field by field
  - [ ] [`correlator_gain`](@ref) per band, and
    [`replica_code_amplitude`](@ref) wherever the gateware approximates a
    code
  - [ ] Dump records: latest-first accumulators, correct `num_taps`, a wire type
    wide enough for every layout, epoch strobes, `code_phase` if available
  - [ ] One record per primary code period, tiling the sample axis with neither
    gaps nor overlap — what the host's overlay removal rides on (section 7)
  - [ ] `supports_partial_code_dumps`, and — if it is `true` — a dump interval
    that divides the code period plus `code_phase` on every record (section 4a);
    without it a code period longer than the receiver's integration length is
    refused before arming
  - [ ] [`dropped_dump_count!`](@ref) and [`assignment_start_sample`](@ref)
    where the hardware can support them

## 10. What this contract does not yet cover

This page is the interface as of
[issue #131](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/131). Extending
the hardware path from GPS L1 C/A to every GNSSSignals signal is tracked by
[issue #130](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/130); the parts
still to land are a *scheduled* hardware secondary-code wipeoff (#132 removes the
overlay on the host; the device-side contract is sketched in section 7), the
gateware and adapter halves of dumps shorter than a primary period (#133 has
landed on the host and is specified for a device in section 4a; the gateware is
gnss-m2sdr#29 and the adapter GNSSM2SDR.jl#8), primary codes longer than a
device's code memory, routing channels
and noise estimates across RF bands (#134), and the per-signal validation matrix
(#135). Until those land, declare conservatively: a capability you cannot serve
is a channel that never locks.
