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
per satellite by a [`CodeLockDetector`](@ref GNSSReceiver.CodeLockDetector) **and** a
[`CarrierLockDetector`](@ref GNSSReceiver.CarrierLockDetector). Both track elapsed signal
time, so their behaviour is independent of how the signal is chunked.

The lock knob surfaced directly on [`receive`](@ref) is the code-lock CN0 threshold:

| Keyword | Default | Meaning |
|---|---|---|
| `code_lock_cn0_threshold` | `nothing` (⇒ `30u"dBHz"` per signal) | A satellite is declared code-locked while its estimated CN0 stays above this. |

```julia
data_channel = receive(
    measurement_channel, GPSL1CA(), 2.048e6u"Hz";
    code_lock_cn0_threshold = 32u"dBHz",   # stricter lock
)
```

The remaining detector timings (out-of-lock, warm-up and carrier integration windows) are
set at detector construction; see their docstrings in the [API Reference](@ref) for the
defaults.

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

## Custom per-chunk output

`extract` replaces the per-chunk payload builder; the default
[`default_data_of_interest`](@ref GNSSReceiver.default_data_of_interest) emits a
[`ReceiverDataOfInterest`](@ref GNSSReceiver.ReceiverDataOfInterest). Pass your own
`extract(receiver_state)` to emit anything else — see [Custom Receiver Output](@ref).

## Multi-antenna processing

For multiple antennas (`NumAnts(N)` with `N > 1`) the post-correlation filter is an
[`EigenBeamformer`](@ref GNSSReceiver.EigenBeamformer). The number of antenna channels in
each measurement channel must equal `N` in `num_ants`.

## Full reference

The complete, authoritative list of keyword arguments — with their exact defaults — is in
the docstrings of [`receive`](@ref) and [`ReceiverState`](@ref) in the
[API Reference](@ref).
