# Hardware-correlator contract

```@meta
CurrentModule = GNSSReceiver
```

This page is the contract between GNSSReceiver and a *hardware correlator* — an
SDR whose FPGA downconverts and correlates on-device. Its tracking loops are
closed by a **separate, allocation-free loop process** built on
[HardwareLoopCore.jl](https://github.com/JuliaGNSS/HardwareLoopCore.jl) over a
vendor driver; the receiver talks to that process through a
[HardwareLoopProtocol.jl](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl)
shared-memory segment and never sees a correlator dump or writes an NCO word
itself. What a vendor package still describes to *this* package is the
**device**: its raw sample streams, its declared capabilities and RF band plan,
its replica amplitudes and the origin of its sample counter. That is what this
page specifies. The types are documented in the
[API reference](@ref "Hardware correlators"); this page says what they *oblige*
you to do.

The split is described under [`AbstractHardwareCorrelatorSDR`](@ref): the
device's raw sample stream keeps driving acquisition, decoding, PVT and the
runtime clock exactly as in the software receiver, and only the correlation and
the loop closure move — to the device and its loop process.

## 0. The three halves of a vendor package

| Half | Lives in | Talks to |
|:---- |:-------- |:-------- |
| **Driver** — `HardwareLoopCore.AbstractLoopDriver`: read records off the device, write NCO words, arm and release channels, report the device sample count and what the gateware can do | the loop process (e.g. GNSSM2SDR's `M2SDRLoop`, built with `juliac --trim=safe`) | the FPGA |
| **Device** — `AbstractHardwareCorrelatorSDR`: this page | the receiver process | GNSSReceiver |
| **Loop handle** — [`RemoteHardwareLoop`](@ref)`(sdr; segment, spawn)` | the receiver process | the loop process over the segment |

The receiver's entry point is

```julia
loop = RemoteHardwareLoop(sdr; segment = "/dev/shm/gnss-loop-m2sdr0", spawn = `gnss_loop …`)
data = receive(loop, GPSL1CA(), 4e6u"Hz")
```

A vendor package wraps this — GNSSM2SDR's `M2SDRRemote` and `remote_loop` do —
so a user names a CSR map, a raw stream and a sample rate and gets the loop
handle back.

The driver's obligations (record format, latest-first accumulators, one record
per primary code period, sample-exact scheduled arms and words, strobes) are
HardwareLoopCore's contract and documented there. Everything below is the
device's.

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
never locks, which is far harder to attribute. The loop process's driver declares
the same facts on its side (`driver_capabilities`) and refuses an arm it cannot
serve; the two declarations should be read off the same gateware registers.

## 2. Nothing is armed before it is validated

`receive(::RemoteHardwareLoop, systems, sampling_freq; …)` calls
[`validate_hardware_configuration`](@ref) before it starts a task and before a
single arm command is published. Every component of every configured system is
checked against the declaration above, and every problem is reported in one
`ArgumentError`:

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

`receive` also checks the loop process itself: the segment's band table has to
carry every requested band at the requested sample rate, and vector tracking —
which closes the loops on the host — is refused.

The same per-signal check runs again for every arm, in the loop process
(`REJECT_UNSUPPORTED`): there it refuses the channel and the receiver counts it
in its `unsupported_signals` instead of throwing, because that code runs on the
chunk path, where taking the receiver down would cost every other satellite its
lock.

## 3. The raw stream and the device counter

[`raw_sample_channel`](@ref)`(sdr)` — per band, `(sdr, band_id)` — is the raw
I/Q the receiver acquires on and clocks itself by. It must keep running for the
whole session: on the LiteX-M2SDR the tracking bank observes the RX datapath
non-intrusively and only sees samples while the raw DMA drains.

[`device_sample_origin`](@ref)`(sdr, band_id)` is the device counter reading
that corresponds to host raw sample 0 of that band. An acquisition yields a code
phase at a host sample; the receiver adds this origin before it asks the loop
process to arm a channel at that device sample. The default is 0, right for a
device whose counter starts with the stream (and for the simulated device); a
real device latches its counter when the raw stream starts — GNSSM2SDR does it
in `latch_origin!`, after the first raw chunk has arrived. Get this wrong by
*n* samples and every handover is *n* samples off in code phase: a handover
error past about half a chip never pulls in.

## 4. What the loop process gets per arm

The receiver builds one arm command per tracked signal component from the
tracking state, and it is the same information the in-process link used to hand
a device: the signal and PRN, the acquisition's Dopplers and code phase with the
device sample they are valid at, every quantised tap offset in whole input
samples (latest first, prompt at zero — see below), the replica normalisation
(section 5), `:primary_only` as the secondary-code mode (section 7) and the
band. The receiver arms one **noise reference** per band as well: an open-loop
despread on an unused PRN of the band's reference signal, which is what the
C/N₀ estimator divides by.

### Why all the tap offsets, and not just the spacing

`Tracking`'s discriminators do not use the spacing the host programmed: they
recover it from the correlator handed back to them, through that correlator's
preferred code shifts. So the device has to reproduce the *host's* quantisation,
not its own. For a three-tap bank getting this wrong is a DLL loop-gain error
(~2.3 % at 4 MHz and a 0.5-chip preferred shift). For a five-tap bank it is
worse: the VE/VL distance enters the discriminator separately, and there is no
single number to re-derive it from. The arm command carries the array — program
it.

## 5. Amplitude normalisation, per band and per signal

The loop brings every accumulator onto one scale — the scale the software
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

Both are read once per assignment and frozen into the arm command, so they may
be computed rather than stored.

The discriminators are ratios and cannot see any of this; the noise-referenced
C/N₀ estimator can, because it divides the prompt power by a floor measured
elsewhere — and the receiver's code-lock threshold (30 dBHz) reads that number.
That is why it has to be declared rather than left at the default.

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
GNSSReceiver.device_sample_origin(sdr::MyDevice, band_id) = sdr.origin[band_id]
```

Everything here has a single-band default, so a one-band adapter implements none
of it. A device that receives several bands at once implements all of it, and
the receiver builds a [`HardwareBandPlan`](@ref) from it
([`hardware_band_plan`](@ref)) — one [`HardwareBandRoute`](@ref) per band,
carrying the band's RF input, its device and its sample rate. The loop process's
segment carries the same band table (band ids and sample rates), and `receive`
refuses a request the two disagree on.

| Concept             | What it counts                               | Where it is declared                                                              |
|:------------------- |:-------------------------------------------- |:--------------------------------------------------------------------------------- |
| **RF input**        | Independently *tuned* bands received at once | [`band_rf_input`](@ref), capped by `HardwareCorrelatorCapabilities.num_rf_inputs` |
| **Antenna**         | Coherent chains of **one** band              | `HardwareCorrelatorCapabilities.num_antennas`                                     |
| **Correlator bank** | The channels that can see a given band       | [`band_hardware_channels`](@ref)                                                  |

These are three different things and none substitutes for another. A four-antenna
L1 front end is *one* route with `num_antennas = 4`: the antennas share one LO
and one sample clock, and the bank despreads all of them into one accumulator
vector per tap so the host can beamform from the prompt covariance. Two *bands*
share nothing but the board.

### Per-band sample rates and the one receiver timebase

Each band's raw samples, each record and each NCO word are counted **on that
band's own counter**, at that band's own rate. The receiver folds, ranges and
schedules on one axis instead: the **reference band** — the first route of the
plan — whose counter *is* the receiver timebase. [`receive`](@ref) takes that
band's rate as its `sampling_freq` and the others alongside it:

```julia
receive(loop, ((GPSL1CA(),), (BeiDouB1I(),)), (4e6u"Hz", 5e6u"Hz"))
```

The mapping between the axes is the exact ratio of the two rates — no offset, no
estimate, no drift — so one millisecond is 4000 counts on a 4 MS/s band and 5000
on a 5 MS/s one, and both close the *same* epoch. A handover's `valid_at_sample`
is written on the channel's own band counter (plus that band's
[`device_sample_origin`](@ref)), and every satellite's code phase is extrapolated
to the same instant on its own band's axis, which is the common reception time
the multi-band pseudoranges are differences of.

For a single-band device every scale is exactly `1.0`, both mappings are the
identity, and nothing above changes anything.

### Multi-device clock synchronisation

[`clock_synchronization`](@ref) says what makes counters on different devices
comparable:

| Value            | Meaning                                                                                          | Supported                           |
|:---------------- |:------------------------------------------------------------------------------------------------ |:----------------------------------- |
| `:single_device` | One device, one sample clock                                                                     | yes (the default)                   |
| `:shared_clock`  | Several devices on one reference **and** one distributed sample clock, counters aligned at start | accepted by the validation          |
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
share none of those, so the receiver arms **one noise reference per band** in
the loop process and never pools them, and [`correlator_gain`](@ref)`(sdr,
band_id)` is read per band, so a band whose replica table is scaled differently
still lands on the same C/N₀ scale. Pooling two floors would describe neither
band, and the error lands on every satellite of both as a C/N₀ bias — the
quantity the code lock detector thresholds on, and the one number in the
receiver that nothing downstream can contradict.

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
front end, not a limitation of the routing. The multi-band routing was validated
in simulation on the in-process link this loop process replaced; the loop
process has so far run one band only, and a multi-band run through it awaits a
front end with two independently tuned inputs.

### Supported configurations, today

| Configuration                                                                 | Status                                                            |
|:----------------------------------------------------------------------------- |:----------------------------------------------------------------- |
| One band, one or more coherent antennas                                       | Supported; the path every existing adapter is on                  |
| Several bands on one device                                                   | Accepted by the validation; not yet run through the loop process  |
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
  - The component's offset against that reference is there to be *preserved*,
    not applied. A device that unavoidably rotates per component must remove
    exactly this value again before dumping.
  - Do not pre-combine components in hardware, and do not pre-combine antennas:
    beamforming is post-correlation on the host and adapts from the per-antenna
    prompt covariance, so an N-antenna device dumps one accumulator per antenna
    per tap.

## 7. Secondary codes

**The loop process removes the overlay; the device replicates the primary
code.** That is the ownership decision, stated per arm as the secondary-code
mode: it is `:primary_only` for every device and every signal, which is also
the protocol's own default. A device is never asked to wipe an overlay, so the overlay is removed
exactly once — in the loop core's ingest, before the coherent pre-accumulation
and therefore before the discriminators, the C/N₀ estimator and the navigation
bit accumulation alike.

Why not the device: an overlay's *phase* is not known when a channel is armed.
It is recovered by the bit/secondary-code sync detector, some way into tracking,
from the prompts themselves, and a device asked to wipe at the wrong phase
cancels the signal instead of accumulating it. Handing the job to the gateware
therefore needs a *scheduled* command ("from device sample `n`, overlay chip
`k`") on the same sample-exact footing as an NCO word, plus a way to read the
device's overlay counter back so that a lost record cannot leave the two ends
disagreeing in silence. That contract is not defined yet.

`max_secondary_code_length` therefore stays a *declaration* — the longest overlay
the gateware could wipe if it were asked — and `1`, the default, is not an error
and costs a device nothing. It is what [`supports_secondary_code_wipeoff`](@ref)
reads, and nothing requests it.

Removing secondary codes after synchronisation is
[issue #132](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/132). It was
demonstrated end to end for GPS L5I on the in-process link this loop process
replaced; the loop core carries the same ingest path, and the demonstration
through the loop process is the follow-up recorded in the
[signal support matrix](@ref "Signal Support & Evidence").

## 8. Liveness, gaps and restarts

The loop process heartbeats into the segment; [`RemoteHardwareLoop`](@ref)
checks it every chunk. A loop whose heartbeat stops for `heartbeat_timeout`
seconds is restarted from its `spawn` command and every tracked satellite
re-armed from the receiver's own state; a receiver attaching to a loop that is
already running releases every channel first and starts from a clean bank.

A satellite whose records were delayed on their way from the device is not a
satellite that lost the signal: [`is_observation_gap`](@ref) freezes its lock
detectors through a gap of up to `max_dump_gap` seconds instead of letting them
decay, and `has_current_observations` keeps a frozen bit count out of
navigation meanwhile. Past the budget the path is not late but broken, and the
detectors are allowed to decay so the satellite is eventually released rather
than held in a lock nothing confirms. When the loop process has to restart a
satellite's bit clock (records were lost on the device), the receiver learns it
through [`take_bit_clock_restart!`](@ref) and restarts the decoder.

## 9. Adapter checklist

  - [ ] [`raw_sample_channel`](@ref) and [`num_hardware_channels`](@ref)
  - [ ] [`hardware_capabilities`](@ref), declared field by field and agreeing
    with the driver's `driver_capabilities`
  - [ ] [`device_sample_origin`](@ref), latched when the raw stream starts
  - [ ] [`correlator_gain`](@ref) per band, and
    [`replica_code_amplitude`](@ref) wherever the gateware approximates a
    code
  - [ ] For a multi-band device: [`band_rf_input`](@ref),
    [`band_hardware_channels`](@ref), [`raw_sample_channel`](@ref)`(sdr, band_id)`,
    [`device_sample_origin`](@ref)`(sdr, band_id)` and — across devices —
    [`band_device_index`](@ref) and [`clock_synchronization`](@ref) (section 5a)
  - [ ] A driver for the loop process (HardwareLoopCore's `AbstractLoopDriver`)
    and an executable that runs `LoopCore` over it, plus the `spawn` command a
    [`RemoteHardwareLoop`](@ref) starts it with

## 10. What this contract does not yet cover

Extending the hardware path from GPS L1 C/A to every GNSSSignals signal is
tracked by [issue #130](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/130);
the parts still to land are a *scheduled* hardware secondary-code wipeoff (#132
removes the overlay in the loop process; the device-side contract is sketched in
section 7), the gateware and adapter halves of records shorter than a primary
period (the loop core accounts for them; the gateware is gnss-m2sdr#29 and the
adapter GNSSM2SDR.jl#8), primary codes longer than a device's code memory, and
the per-signal validation matrix (#135). Routing channels and noise estimates
across RF bands (#134) is specified in section 5a and checked by the
validation; what is *not* yet demonstrated is a multi-band run through the loop
process, and multi-device operation on a shared sample clock is accepted by the
validation without having been run. Adopting a running loop's satellites on
attach, rather than releasing them, is not implemented either. Until the rest
lands, declare conservatively: a capability you cannot serve is a channel that
never locks.
