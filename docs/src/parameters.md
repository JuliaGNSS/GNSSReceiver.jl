# Acquisition & Tracking Parameters

Almost everything about how the receiver behaves is controlled through keyword arguments
to [`receive`](@ref) (and, for the live path, [`gnss_receiver_gui`](@ref)). This page
groups those keywords by pipeline stage and shows how to set them. The full, authoritative
list lives in the [`receive` docstring](@ref receive) at the bottom of the page.

## Systems: multi-constellation and multi-band

`receive`'s second argument selects which signals to receive:

- A **single system** — one GNSS signal (`GPSL1CA()`), a [`CombinedSignal`](@ref)
  pilot+data pair, or a tuple of these — that shares one RF band:

  ```julia
  receive(measurement_channel, GPSL1CA(), sampling_freq)
  ```

- A **tuple of systems** sharing one RF band, fused into a single multi-GNSS PVT
  solution. GPS L1 C/A and Galileo E1 both live on L1, so they can share one stream:

  ```julia
  receive(measurement_channel, (GPSL1CA(), GalileoE1B()), sampling_freq)
  ```

  (All systems in the tuple must share one RF band; mixing bands — e.g. L1 and L5 —
  throws.)

  !!! note "Galileo E1B needs a high sampling frequency — or its BOC(1,1) approximation"
      `GalileoE1B()` is the full CBOC(6,1,1/11) signal, whose BOC(6,1) component
      requires a sampling frequency above ~12.3 MHz (Nyquist for the 6.138 MHz
      subcarrier). At typical low SDR rates — like the 2.048 MHz used in the examples
      on these pages — use `GalileoE1B_BOC11()` instead, the BOC(1,1) approximation
      that ignores the BOC(6,1) component and correlates fine at low rates (at a small
      C/N₀ penalty).

- For **several RF bands**, use the multi-band method: a tuple of measurement channels
  (one per band), a tuple of per-band system groups and a tuple of `interm_freqs`, all
  aligned band-by-band. Every band is fused into one solution with per-constellation clock
  biases and per-band inter-frequency biases:

  ```julia
  receive(
      (l1_channel, l5_channel),
      ((GPSL1CA(),), (CombinedSignal(GPSL5Q(), GPSL5I()),)),
      sampling_freq;
      interm_freqs = (0.0u"Hz", 0.0u"Hz"),
  )
  ```

A [`CombinedSignal`](@ref)`(pilot, data)` tracks the dataless pilot (which the loops range
on) and the data component (whose navigation message is decoded) together in one group.

## Acquisition

Acquisition is the search that finds which satellites are visible and gives a first
estimate of their code phase and Doppler. It re-runs at most every `acquire_every` of
signal time. Its **coherent integration length and Doppler resolution are no longer user
knobs** — they are derived internally, per system, from the tracking loop's carrier-Doppler
*pull-in range*, so that the worst-case post-acquisition residual always lands inside the
loop's capture range. A detection is accepted purely on a CFAR test (constant false-alarm
rate) at a fixed internal false-alarm probability — there is no acquisition CN0 threshold.

| Keyword | Default | Meaning |
|---|---|---|
| `acquire_every` | `10u"s"` | How often (in signal time) acquisition re-runs to look for new satellites. |
| `prns` | `nothing` | Which PRNs to search for. `nothing` ⇒ each constellation's default range; a per-GNSS `NamedTuple`/`Dict` keyed by constellation; or a plain collection applied to every system. |

### Restricting the PRN search

`prns` narrows (or widens) the satellites acquisition searches for. For a multi-GNSS run
you can give a per-constellation list keyed by constellation name:

```julia
data_channel = receive(
    measurement_channel, (GPSL1CA(), GalileoE1B()), sampling_freq;
    prns = (GPS = [1, 8, 30], Galileo = [3, 9, 24]),
)
```

A plain collection is applied to every system, and `nothing` uses each constellation's
default range. Each system's search is further restricted to the PRNs that actually
broadcast its signal.

## Front end & correlator

| Keyword | Default | Meaning |
|---|---|---|
| `num_ants` | `NumAnts(1)` | Number of antenna channels. Must match the columns of the measurement channel(s). |
| `interm_freq` / `interm_freqs` | `0.0u"Hz"` | Intermediate frequency of the incoming samples (single-band `interm_freq`; a tuple `interm_freqs` for the multi-band method). |
| `downconvert_and_correlator` | auto (by element type) | The correlator backend. `nothing` auto-selects: Tracking's fast integer backend for `Complex{Int16}` samples, the float `CPUThreadedDownconvertAndCorrelator()` otherwise. Pass one explicitly to override. |
| `max_meas` | `nothing` | Front-end full-scale (largest `\|real\|`/`\|imag\|` of any sample, e.g. `2^11` for a 12-bit ADC). **Required** for `Complex{Int16}` samples (the integer backend); ignored for float samples or when `downconvert_and_correlator` is given. |

The correlator backend is auto-selected from the sample element type: `Complex{Int16}`
recordings use Tracking's fast integer downconvert-and-correlator (which needs `max_meas`),
and every other element type uses the float `CPUThreadedDownconvertAndCorrelator()`. You can
therefore either pass `Complex{Int16}` samples with `max_meas`, or read integer recordings
as `ComplexF32` (`read_uint8_iq_file(...; center = 127.5, type = ComplexF32)`) to use the
float backend with no full-scale value. See [Getting Started](@ref).

## Lock detection

A satellite contributes to the PVT solution only while it is *in lock*. Lock is declared
per satellite by a [`CodeLockDetector`](@ref GNSSReceiver.CodeLockDetector) — a threshold on
the post-integration SNR — **and** a
[`CarrierLockDetector`](@ref GNSSReceiver.CarrierLockDetector) — the narrowband
difference/power phase-lock indicator `Σ(I²−Q²)/Σ(I²+Q²)`, fed every prompt the tracking
loops produce. Both track elapsed signal time, so their behaviour is independent of how the
signal is chunked, and both take their timings as counts of the ranging signal's primary
code period rather than in seconds, so they mean the same thing on every signal (see
[Timings scale with the code period](@ref)).

The detector thresholds and timings (the code-lock CN0 threshold, the carrier phase-lock
threshold and its smoothing window, and the out-of-lock and warm-up windows) are set at
detector construction from per-signal defaults; see their docstrings in the
[API Reference](@ref).

The CN0 threshold is referred to a record spanning **one primary code period**, because what
decides detectability is the post-integration SNR `CN0 · T` rather than CN0 on its own.
When tracking integrates over `N` code blocks per record, a satellite therefore holds lock
at `10·log10(N)` dB less CN0 — 3 dB at two blocks — since the longer coherent integration
recovers exactly that much. Anchoring to the code period rather than to the symbol period
keeps the number meaning the same thing for the default one-block cadence and stays
defined for pilot signals, which have no data rate.

The CN0 itself is estimated by Tracking, whose default estimator is the measured noise
reference (`NoiseRefCN0Estimator`) as of Tracking 7: it divides each record's prompt power
by a noise density measured by despreading the signal's own code at a randomised phase,
through the same correlator the satellites go through, rather than inferring a floor from
the prompt stream. The receiver provisions one such reference per tracked signal
automatically, so nothing has to be configured for it.

The threshold keeps its units and its meaning, but the statistic it is compared against is
a better behaved one. It has no noise-only floor to clear — on pure noise the estimate is
`-Inf dBHz` or single digits, against the moment ratio's ~27.6 dB-Hz — so a dead or falsely
acquired channel drops out of lock promptly rather than clearing a 30 dB-Hz threshold on
~19 % of records. And because the floor is measured rather than inferred, the number is the
true CN0: on the ION reference recording it reads a median 1.0 dB above what the
narrowband/wideband ratio (Tracking 6's default) reported, and up to 2.4 dB on the least
steadily tracked satellite, whose loop phase noise that coherent estimator charged to the
signal. The threshold is not a `receive` keyword: it comes from the per-signal default
(`GNSSReceiver.get_default_code_lock_cn0_threshold`, 30 dB-Hz) that each satellite's
`CodeLockDetector` is constructed with, which is where sensitivity would have to be traded.
The estimate is also what `sat_data[…].cn0` reports and what the GUI plots.

### Two stages: handover, then steady state

The acquisition → tracking handover is deliberately coarse — the acquisition Doppler bin is
half the tracking loop's pull-in range, and the code phase is only resolved to the sample
grid, halved by `subsample_interpolation`. Until the loops have pulled those errors in, the
prompt correlator sits off the correlation peak and reports *less* power than the satellite
really has, and the carrier phase is still spinning at the residual Doppler, so a perfectly
healthy satellite looks weak to both detectors.

Both detectors therefore run a **pull-in stage** with a generous out-of-lock allowance,
latching permanently into a tighter **steady state** once the satellite has either strung
together enough good evidence to count as converged or exhausted its pull-in window. This buys
handover tolerance without making a genuine signal loss slower to notice. Measured on
synthetic handovers at this receiver's own worst-case residual, the peak accumulated
out-of-lock time a satellite the receiver should keep demands is 292 code periods — more than
the 200-code-period steady-state dwell tolerates, which is why a single un-staged dwell drops
it.

Ranging is gated separately from tracking, and on a *later* criterion. A satellite in its
pull-in stage is still *kept* — that is the point — but it does not contribute to the PVT
solution until its loops have settled, because a code phase that is still converging produces
a pseudorange biased by tens of metres: measured at a 0.25-chip handover, the residual is 0.15
to 0.22 chip (44 to 64 m) after 200 code periods of clean evidence and 0.018 to 0.15 chip (5
to 44 m) after 600. `time_in_lock_before_calculating_pvt` alone would not achieve this: it
counts from the handover, so it can elapse while the loops are still settling — and on a code
longer than 1 ms the settling takes proportionally longer while that gate stays fixed in
seconds.

Ranging readiness needs a **timer** as well as an evidence path, for the same reason the
pull-in stage does: the evidence clock is reset by any out-of-lock verdict, so an uninterrupted
run is a bar a satellite can fail to clear indefinitely — and it would then be tracked,
converged, and silently absent from the PVT solve forever. Since PVT needs four satellites,
that trades a degraded fix for no fix. Under Tracking's default noise-reference estimator the
code detector rarely blocks the run (measured steady-state per-chunk out-of-lock rates are 0.58
at the 30 dB-Hz threshold itself, 0.10 one dB above it and 0.00 from two dB up), but the
carrier detector does: its phase-lock indicator takes 460 to 590 code periods to come up
through the handover. And against any disturbance recurring on a period shorter than the run
the evidence path never fires at all, at any C/N0. The backstop admits such a satellite once
enough signal has *elapsed* — two code-loop time constants — with its out-of-lock clock still
inside what steady state tolerates. What it concedes is a few metres of extra code-phase
thermal jitter (1σ of 0.039 chip, 11 m, at 30 dB-Hz against 0.004 chip, 1.2 m, at 45), not a
converging handover bias.

One residual case is left open deliberately: a satellite whose accumulated out-of-lock time
parks *between* the steady-state and pull-in allowances — 200 to 600 code periods — never
leaves the pull-in stage, and so never becomes ranging-ready either. Reaching it takes a burst
of failures followed by a sustained near-50% duty cycle; under a stochastic signal the clock
random-walks out of that band, either down to zero (whereupon it latches) or up past the
pull-in allowance (whereupon lock is lost and the satellite is reacquired). Closing it would
mean letting the pull-in stage latch while its clock is still above the steady-state threshold,
which is exactly the discontinuity the latch guard exists to prevent.

### Timings scale with the code period

Every detector timing is configured as a multiple of the ranging signal's **primary code
period** rather than in seconds, because that is what the tracking loops' time constants
scale with: Tracking sizes its default bandwidths at `B_L·T ≈ 0.018`, so a 1 ms code gets an
18 Hz carrier / 1 Hz code loop while a 10 ms code gets 1.8 Hz / 0.1 Hz — every `1/B_L` is a
fixed number of code periods.

| Signal | Code period | Warm-up (80 `T`) | Code dwell (200 `T`) | Pull-in allowance (600 `T`) | Ranging ready (≥ 680 `T`) | Ranging backstop (2000 `T`) |
|---|---|---|---|---|---|---|
| GPS L1 C/A | 1 ms | 80 ms | 200 ms | 600 ms | ≥ 680 ms | 2 s |
| Galileo E1B | 4 ms | 320 ms | 800 ms | 2.4 s | ≥ 2.7 s | 8 s |
| GPS L1C-D | 10 ms | 800 ms | 2 s | 6 s | ≥ 6.8 s | 20 s |

"Ranging ready" is the evidence path — the warm-up plus an uninterrupted 600-code-period run,
so 680 `T` is a floor rather than a typical value: measured on real handovers it lands at 760
to 1500 code periods on GPS L1 C/A, because the carrier loop has its own Doppler pull-in to do
before the phase-lock indicator stops resetting the run. "Ranging backstop" is the timer that
bounds the wait for a satellite whose evidence never comes cleanly. The carrier
detector's own dwell is 4000 `T` (4 s on GPS L1 C/A) and, being floored at that, is unstaged:
it clears outright on any favourable chunk, so declaring loss already needs a consecutive run
of failures far longer than any handover transient.

A dwell fixed in seconds means something different on every signal: the 200 ms this receiver
used to allow is 200 code periods on GPS L1 C/A — the signal it was tuned against, and where
it is correct — but only 50 on Galileo E1B and 20 on GPS L1C-D, against loops that are four
and ten times slower. That is why lock detection failed on the slower signals while GPS
L1 C/A looked fine. The GPS L1 C/A warm-up and dwell are unchanged.

One consequence worth stating outright: on codes longer than 1 ms these timings, not
`time_in_lock_before_calculating_pvt`, are what actually decide when a satellite joins the
solve. A healthy GPS L1 C/A satellite is ranging-ready between 0.76 s and 1.5 s depending on
its C/N0, so the 2 s default still binds there. On Galileo E1B (2.7 s at best) and GPS L1C-D
(6.8 s) it no longer does, and lowering it below those figures has no effect at all. The extra
seconds are in practice dwarfed by the tens of seconds of subframe decoding that set time to
first fix.

The counts themselves are set at detector construction; see the
[`CodeLockDetector`](@ref GNSSReceiver.CodeLockDetector) and
[`CarrierLockDetector`](@ref GNSSReceiver.CarrierLockDetector) docstrings in the
[API Reference](@ref) for the defaults.

## PVT

| Keyword | Default | Meaning |
|---|---|---|
| `time_in_lock_before_calculating_pvt` | `2u"s"` | A satellite must be locked this long before it is used for PVT. |
| `pvt_update_interval` | `100u"ms"` | How often the PVT solution is recomputed (also the rate at which the data channel emits). |
| `enable_ionospheric_correction` | `true` | Apply the broadcast ionospheric correction. |
| `enable_tropospheric_correction` | `true` | Apply the tropospheric correction. |
| `pvt_approximate_year` | current UTC year | Resolves the GPS week-number rollover for old recordings. |

`pvt_approximate_year` matters for archived data: an old recording processed with the wrong
year lands ~19.6 years off. The [Worked Example (Real Data)](@ref) sets
`pvt_approximate_year = 2017` for its 2017 recording.

```julia
data_channel = receive(
    measurement_channel, GPSL1CA(), 2.048e6u"Hz";
    pvt_update_interval = 200u"ms",
    enable_tropospheric_correction = false,
    pvt_approximate_year = 2017,
)
```

## Vector tracking

By default every satellite closes its own code/carrier loops (scalar tracking).
`vector_tracking = true` switches the receiver to vector tracking: once a first scalar
fix is available, a navigation Kalman filter fuses all satellites' accumulated
discriminator outputs into one position/velocity/clock solution and closes every
tracking loop centrally from it (a vector delay/frequency lock loop, VDFLL). Weak or
briefly obscured satellites are carried through outages by the collective solution, and
the emitted PVT solutions come from the navigation filter (at `pvt_update_interval`).
They carry the same per-satellite diagnostics as a scalar fix, including the post-fit
pseudorange and range-rate residuals (`SatInfo.residual` and `SatInfo.rate_residual`) of
every satellite in the vector loop — reported even for a satellite whose signal the loop
is currently carrying through an outage.

```julia
data_channel = receive(
    measurement_channel, (GPSL1CA(), GalileoE1B()), 2.048e6u"Hz";
    vector_tracking = true,
)
```

Multi-constellation and multi-band configurations are supported with the same bias model
as the scalar solve: the filter estimates one receiver clock bias per GNSS time system
(all driven by the one oscillator's clock drift) and one receiver inter-frequency bias
per band beyond a reference band. The pseudorange measurements are corrected for the
broadcast ionospheric model and the Saastamoinen tropospheric delay, exactly like
`calc_pvt` (toggled by the same `enable_ionospheric_correction` /
`enable_tropospheric_correction` keywords).

`vector_tracking = true` runs the filter with its default configuration. To configure it,
pass a [`VectorTracking`](@ref GNSSReceiver.VectorTracking) instead — it both enables vector
tracking and describes the platform and the front end, which is worth doing whenever either
is known: the defaults assume vehicular dynamics and a TCXO-grade oscillator, and both the
dynamics (`acceleration_noise_std`) and the oscillator stability (`h0`, `hm2`) materially
change how the filter weighs its prediction against the measurements. The same struct also
selects the measurement set (`use_pseudorange_rates = false` for a pseudorange-only VDLL
instead of the default VDFLL), the motion and clock model orders, the inter-frequency-bias
process noise and how long the filter may coast before falling back to scalar tracking:

```julia
data_channel = receive(
    measurement_channel, (GPSL1CA(), GalileoE1B()), 2.048e6u"Hz";
    vector_tracking = VectorTracking(;
        acceleration_noise_std = 1.0u"m/s^2",   # pedestrian dynamics
        insufficient_meas_timeout = 30.0u"s",
    ),
)
```

Each emitted payload then reports what the filter is doing through its
[`VTStatus`](@ref GNSSReceiver.VTStatus): whether it is running, its own position and clock
uncertainty, every loop member's post-fit residuals (coasted members included) and how long
it has been coasting on epochs it could not solve.

## Custom per-chunk output

`extract` replaces the per-chunk payload builder; the default
[`default_data_of_interest`](@ref GNSSReceiver.default_data_of_interest) emits a
[`ReceiverDataOfInterest`](@ref GNSSReceiver.ReceiverDataOfInterest). Pass your own
`extract(receiver_state)` to emit anything else — see [Custom Receiver Output](@ref).

## Multi-antenna processing

For multiple antennas (`NumAnts(N)` with `N > 1`) the post-correlation filter is an
[`EigenBeamformer`](@ref GNSSReceiver.EigenBeamformer). The number of antenna channels in
each measurement channel must equal `N` in `num_ants`.

The reported C/N₀ is referenced to the beamformer's own output: Tracking measures the
array's `N×N` spatial noise covariance `R̂` once per signal and each satellite reduces it
through its current weights, `N₀ = wᴴR̂w`. So the array gain a well-steered beam buys shows
up in C/N₀, and only that — a beamformer's weight scaling cancels out of the ratio.

## Full reference

The complete, authoritative list of keyword arguments — with their exact defaults — is in
the docstrings of [`receive`](@ref) and [`ReceiverState`](@ref) in the
[API Reference](@ref).
