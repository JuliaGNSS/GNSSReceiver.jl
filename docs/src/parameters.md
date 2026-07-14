# Acquisition & Tracking Parameters

Almost everything about how the receiver behaves is controlled through keyword arguments
to [`receive`](@ref) (and, for the live path, [`gnss_receiver_gui`](@ref)). This page
groups those knobs by pipeline stage and shows how to set them. The full, authoritative
list lives in the [`receive` docstring](@ref receive) at the bottom of the page.

## Acquisition

Acquisition is the search that finds which satellites are visible and gives a first
estimate of their code phase and Doppler. It re-runs at most every `acquire_every` of
signal time.

| Keyword | Default | Meaning |
|---|---|---|
| `acquisition_num_coherent_code_periods` | `4` | **Coherent integration length**, in code periods. Longer ⇒ more sensitivity, more compute. |
| `acquisition_num_noncoherent_accumulations` | `1` | Number of coherently-integrated blocks summed **non-coherently**. Extends integration past the navigation-bit edge. |
| `bit_edge_search_steps` | `1` | Number of bit-edge hypotheses tried during acquisition. |
| `acquire_every` | `10u"s"` | How often (in signal time) acquisition re-runs to look for new satellites. |
| `acquisition_false_alarm_probability` | `1e-4` | CFAR detector false-alarm probability. Lower ⇒ fewer false detections, less sensitivity. |
| `prns` | `1:32` | Which PRNs to search for. |

### Coherent integration length

The most important sensitivity knob is the **coherent integration length**. One code
period of GPS L1 C/A is 1 ms, so `acquisition_num_coherent_code_periods = 10` integrates
coherently over 10 ms:

```julia
data_channel = receive(
    measurement_channel, GPSL1CA(), 2.048e6u"Hz";
    acquisition_num_coherent_code_periods = 10,   # 10 ms coherent integration
    max_meas = 2^7,
)
```

Coherent integration can only extend up to the 20 ms navigation-bit period before a bit
flip cancels the accumulation; to integrate longer, add **non-coherent** accumulations:

```julia
data_channel = receive(
    measurement_channel, GPSL1CA(), 2.048e6u"Hz";
    acquisition_num_coherent_code_periods = 10,       # 10 ms coherent …
    acquisition_num_noncoherent_accumulations = 3,    # … × 3 blocks, non-coherently
    max_meas = 2^7,
)
```

By default the acquisition sample buffer is sized automatically from these two numbers,
so you don't have to compute the sample count yourself.

## Front end & correlator

| Keyword | Default | Meaning |
|---|---|---|
| `num_ants` | `NumAnts(1)` | Number of antenna channels. Must match the columns of the measurement channel. |
| `interm_freq` | `0.0u"Hz"` | Intermediate frequency of the incoming samples. |
| `max_meas` | `nothing` | Front-end full-scale; **required** for `Complex{Int16}` samples (integer backend). |
| `downconvert_and_correlator` | auto from element type | Override the correlator backend explicitly. |

The backend is chosen from the sample element type: `Complex{Int16}` inputs use Tracking's
fast integer downconvert-and-correlator (needs `max_meas`), and every other element type
uses the general CPU backend. See [Getting Started](@ref) for the `max_meas` note.

## Tracking loop configuration (via `ReceiverState`)

The correlator layout, the post-correlation filter (beamformer) and the Doppler estimator
are pinned when the [`ReceiverState`](@ref) is built. To customise them, construct the
state yourself and pass it to [`receive`](@ref) as `receiver_state`. Constructing the
state needs no data, so this snippet runs as-is:

```@example params
using GNSSReceiver, GNSSSignals, Tracking, Unitful

system = GPSL1CA()
sampling_freq = 2.048e6u"Hz"
num_ants = NumAnts(1)

# Size the acquisition buffer for 10 ms coherent integration.
num_coherent = 10
num_samples_for_acquisition = round(
    Int,
    get_code_length(system) *
    upreferred(sampling_freq / get_code_frequency(system)) *
    num_coherent,
)

state = ReceiverState(
    Complex{Int16},
    system;
    num_ants,
    num_samples_for_acquisition,
    correlator = Tracking.get_default_correlator(system, num_ants),   # or a custom one
    doppler_estimator = ConventionalPLLAndDLL(),
)
```

You would then thread it into `receive` (this part needs a real measurement channel, so
it is shown but not executed):

```julia
data_channel = receive(
    measurement_channel, system, sampling_freq;
    num_ants,
    receiver_state = state,
    acquisition_num_coherent_code_periods = num_coherent,   # keep this in sync with the buffer size
    max_meas = 2^7,
)
```

For multiple antennas (`NumAnts(N)` with `N > 1`) the post-correlation filter defaults to
an [`EigenBeamformer`](@ref GNSSReceiver.EigenBeamformer); pass `post_corr_filter` to
`ReceiverState` to override it.

## Lock detection

A satellite contributes to the PVT solution only while it is *in lock*. Lock is declared
per satellite by a [`CodeLockDetector`](@ref GNSSReceiver.CodeLockDetector) **and** a
[`CarrierLockDetector`](@ref GNSSReceiver.CarrierLockDetector). Both track elapsed signal
time, so their behaviour is independent of how the signal is chunked.

The one lock knob surfaced directly on [`receive`](@ref) is the code-lock CN0 threshold:

| Keyword | Default | Meaning |
|---|---|---|
| `code_lock_cn0_threshold` | `30u"dBHz"` | A satellite is declared code-locked while its estimated CN0 stays above this. |

```julia
data_channel = receive(
    measurement_channel, GPSL1CA(), 2.048e6u"Hz";
    code_lock_cn0_threshold = 32u"dBHz",   # stricter lock
    max_meas = 2^7,
)
```

The remaining detector timings (out-of-lock, warm-up and carrier integration windows) are
set at detector construction; see their docstrings in the [API Reference](@ref) for the
defaults.

## PVT

| Keyword | Default | Meaning |
|---|---|---|
| `time_in_lock_before_calculating_pvt` | `2u"s"` | A satellite must be locked this long before it is used for PVT. |
| `pvt_update_interval` | `100u"ms"` | How often the PVT solution is recomputed. |
| `approximate_year` | current UTC year | Resolves the GPS week-number rollover for old recordings. |
| `always_buffer` | `false` | Keep the acquisition buffer filled every frame for faster re-acquisition. |

`approximate_year` matters for archived data: an old recording processed with the wrong
year lands ~19.6 years off. The [Worked Example (Real Data)](@ref) sets
`approximate_year = 2017` for its 2017 recording.

## Full reference

The complete, authoritative list of keyword arguments — with their exact defaults — is in
the docstrings of [`receive`](@ref) and [`ReceiverState`](@ref) in the
[API Reference](@ref).
