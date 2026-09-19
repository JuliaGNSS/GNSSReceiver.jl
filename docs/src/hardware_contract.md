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

## 5a. RF bands, inputs and the receiver timebase

Supporting every signal is not the same as receiving every RF band at once. The
first is a property of the correlator; the second is a property of the front end,
and this section is where it is declared, checked and — where it is exceeded —
refused.

### Declare where each band arrives

```julia
GNSSReceiver.band_rf_input(sdr::MyDevice, band_id) = band_id === :L1 ? 1 : 2
GNSSReceiver.band_device_index(sdr::MyDevice, band_id) = 1
GNSSReceiver.clock_synchronization(sdr::MyDevice) = :single_device
GNSSReceiver.band_hardware_channels(sdr::MyDevice, band_id) =
    band_id === :L1 ? (1:10) : (11:20)
GNSSReceiver.raw_sample_channel(sdr::MyDevice, band_id) = sdr.raw[band_id]
```

Everything here has a single-band default, so a one-band adapter implements none
of it. A device that receives several bands at once implements all of it, and
the receiver builds a [`HardwareBandPlan`](@ref) from it
([`hardware_band_plan`](@ref)) — one [`HardwareBandRoute`](@ref) per band,
carrying the band's RF input, its device and its sample rate.

| Concept             | What it counts                               | Where it is declared                                                              |
|:------------------- |:-------------------------------------------- |:--------------------------------------------------------------------------------- |
| **RF input**        | Independently *tuned* bands received at once | [`band_rf_input`](@ref), capped by `HardwareCorrelatorCapabilities.num_rf_inputs` |
| **Antenna**         | Coherent chains of **one** band              | `HardwareCorrelatorCapabilities.num_antennas`                                     |
| **Correlator bank** | The channels that can see a given band       | [`band_hardware_channels`](@ref)                                                  |

These are three different things and none substitutes for another. A four-antenna
L1 front end is *one* route with `num_antennas = 4`: the antennas share one LO
and one sample clock, and the bank despreads all of them into one
`SVector{4,Complex}` per tap so the host can beamform from the prompt covariance.
Two *bands* share nothing but the board.

### Per-band sample rates and the one receiver timebase

Each band's raw samples, each dump's `sample_index` and each
[`NCOUpdate`](@ref)'s `apply_at_sample` are counted **on that band's own
counter**, at that band's own rate. The receiver folds, ranges and schedules on
one axis instead: the **reference band** — the first route of the plan — whose
counter *is* the receiver timebase. [`receive`](@ref) takes that band's rate as
its `sampling_freq` and the others alongside it:

```julia
receive(sdr, ((GPSL1CA(),), (BeiDouB1I(),)), (4e6u"Hz", 5e6u"Hz"))
```

The mapping between the axes is the exact ratio of the two rates — no offset, no
estimate, no drift — so one millisecond is 4000 counts on a 4 MS/s band and 5000
on a 5 MS/s one, and both close the *same* epoch. Concretely:

  - a record's `sample_index` is mapped onto the timebase before the epoch clock
    sees it, so a band sampled faster cannot drive the clock fast and strand
    every slower band's records in the past;
  - `HardwareChannelConfig.valid_at_sample` is written on the channel's own band
    counter, so a handover is propagated in the units the device counts in;
  - every `NCOUpdate` of one fold names the same *instant*, written on each
    channel's own counter;
  - every satellite's code phase is extrapolated to the same instant on its own
    band's axis, which is the common reception time the multi-band pseudoranges
    are differences of.

**Epoch strobes are the exception, and deliberately so**: a strobe is the
timebase marker, so it is stated *in* the timebase. Emit them on the reference
band's counter only.

For a single-band device every scale is exactly `1.0`, both mappings are the
identity, and nothing above changes anything.

### Multi-device clock synchronisation

[`clock_synchronization`](@ref) says what makes counters on different devices
comparable:

| Value            | Meaning                                                                                          | Supported                           |
|:---------------- |:------------------------------------------------------------------------------------------------ |:----------------------------------- |
| `:single_device` | One device, one sample clock                                                                     | yes (the default)                   |
| `:shared_clock`  | Several devices on one reference **and** one distributed sample clock, counters aligned at start | yes                                 |
| `:independent`   | Free-running clocks                                                                              | **refused** for a multi-device plan |

`:independent` is refused rather than approximated. Two free-running clocks drift
by parts per million against each other — metres of pseudorange per second — and
the ratio mapping above has no offset term to absorb it, so there is no common
reception epoch and a fix built from both bands would be wrong with nothing
looking wrong. A receiver that needs it has to estimate and steer the
inter-device offset first, which this package does not do.

### What happens when the request exceeds the front end

[`validate_hardware_configuration`](@ref) checks the plan as a whole, before
anything is armed, and reports every problem at once
([`GNSSReceiver.band_plan_error`](@ref)):

  - a band the front end does not declare it can tune;
  - more bands on one device than it has RF inputs;
  - two bands routed to one RF input;
  - a multi-device plan whose clocks are `:independent`;
  - a band whose correlator bank is empty, or two bands claiming one channel
    ([`GNSSReceiver.band_bank_error`](@ref)) — a band the receiver can tune and
    then never track anything on is the same silent partial configuration.

There is **no automatic fallback to sequential retuning** — receiving band A for
a while, then retuning to band B. It would turn a simultaneous request into a
time-multiplexed one behind the caller's back, changing every band's C/N₀, its
measurement epochs and its fix rate, and leaving no band continuously tracked. A
receiver that wants it runs one [`receive`](@ref) per band configuration and
retunes between them, which is explicit; the refusal message says so.

### Per-band noise references and replica gain

A noise density belongs to one front end's gain chain, antenna, filter and
interference environment, and to the modulation that despreads it. Two bands
share none of those, so the receiver keeps **one reference per band** and never
pools them:

  - `noise_source = :channel` (the default) spends one hardware channel *of each
    band's own bank* on an open-loop despread, armed on a signal of that band at
    that band's rate, and its observation reaches only that band's signals;
  - `noise_source = :samples` meters `Σ|x|²` on each band's own raw frame;
  - [`correlator_gain`](@ref)`(sdr, band_id)` is read per band, so a band whose
    replica table is scaled differently still lands on the same C/N₀ scale.

Pooling two floors would describe neither band, and the error lands on every
satellite of both as a C/N₀ bias — the quantity the code lock detector thresholds
on, and the one number in the receiver that nothing downstream can contradict.

### Measured: what the LiteX-M2SDR front end can actually do

The reference device for this contract is a **one-band** front end, and that is
worth stating precisely rather than leaving to be discovered. Read off the board
(`orin2`, gateware `LiteX-M2SDR SoC / m2 variant / built on 2026-07-29`,
20-channel GNSS build) on 2026-09-16:

  - the RF configuration registers are **singular**: one
    `ad9361_active_rx_frequency_khz`, one `ad9361_active_sample_rate`, one
    `ad9361_active_bandwidth`. There is no per-input frequency or rate register
    anywhere in the CSR map.
  - the *only* per-RX-chain registers are the AGC saturation counters
    (`ad9361_agc_count_rx1_*`, `ad9361_agc_count_rx2_*`), and the RF utility
    exposes `--rx-gain1` / `--rx-gain2` against a single `--rx-freq` and a single
    `--sample-rate`. RX1 and RX2 are two coherent **antenna** chains behind one
    AD9361 RX LO and one sample clock — `num_antennas = 2`, `num_rf_inputs = 1`.
  - none of the 20 correlator channels (`gnss_ch0…gnss_ch19`) carries a band, RF
    input or antenna selector: one bank, fed from one datapath.
  - retuning is device-wide. Writing L5 into that one register moved the whole
    front end (`0x001809fc` = 1 575 420 kHz → `0x0011f382` = 1 176 450 kHz) and
    writing L1 back moved it all the way back; nothing stayed on L1 in between.

So **a supported multi-band configuration cannot be demonstrated on this
hardware**, and this page does not claim one. An M2SDR adapter should declare
`num_rf_inputs = 1` and whatever `num_antennas` its build has, and a two-band
request will then be refused before arming with the message in
[`GNSSReceiver.band_plan_error`](@ref) — which is the correct outcome for this
front end, not a limitation of the routing. The multi-band path above is
validated in simulation (`test/multi_band_routing.jl`) and awaits a front end
with two independently tuned inputs.

### Supported configurations, today

| Configuration                                                                 | Status                                                            |
|:----------------------------------------------------------------------------- |:----------------------------------------------------------------- |
| One band, one or more coherent antennas                                       | Supported; the path every existing adapter is on                  |
| Several bands on one device, one sample rate                                  | Supported                                                         |
| Several bands on one device, different sample rates per band                  | Supported (validated in simulation, `test/multi_band_routing.jl`) |
| Several bands over several devices on a shared reference **and** sample clock | Accepted by the validation; not demonstrated on hardware          |
| Two bands on the LiteX-M2SDR                                                  | Impossible: one RX LO, one sample clock (measured above)          |
| Several bands over devices on independent clocks                              | Refused before arming                                             |
| More bands than RF inputs, by sequential retuning                             | Not performed; refused before arming                              |

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
  - [ ] For a multi-band device: [`band_rf_input`](@ref),
    [`band_hardware_channels`](@ref), [`raw_sample_channel`](@ref)`(sdr, band_id)`,
    and — across devices — [`band_device_index`](@ref) and
    [`clock_synchronization`](@ref) (section 5a). Dumps and NCO updates on each
    band's own counter; epoch strobes on the reference band's
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
device's code memory, and the per-signal validation matrix (#135). Routing
channels and noise estimates across RF bands (#134) is specified in section 5a
and validated in simulation; what is *not* yet demonstrated there is a live
multi-band capture on real gateware, and multi-device operation on a shared
sample clock is accepted by the validation without having been run. Until the
rest lands, declare conservatively: a capability you cannot serve is a channel
that never locks.
