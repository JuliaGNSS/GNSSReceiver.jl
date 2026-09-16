# Signal Support & Evidence

GNSSReceiver's constructors accept every signal `GNSSSignals` defines. That is not the
same as every signal being *validated*, and this page is the difference: a per-signal,
per-role record of what has actually been demonstrated, how, and what the rest is
waiting on.

It exists because "supported" is not a property a receiver can claim of a signal as a
whole. A pilot never decodes a navigation message; a data component is rarely what the
loops range on; a signal whose replica is provably correct may still have no tracking
baseline behind it. So support is recorded per **(signal, role)** pair, and a cell that
has no evidence says so rather than inheriting a claim from its neighbours.

The table below is generated from `test/signal_support.jl` and checked against it on
every test run, so it cannot drift from what the suite actually asserts. The checks that
fill it in live in `test/signal_validation.jl` and run against the deterministic
reference harness described at the end of this page.

!!! note "Most cells are `untested`, on purpose"

    This matrix is being filled in by the all-signal roadmap
    ([issue #130](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/130)), whose
    hardware-correlator steps have not landed yet. An `untested` cell means no evidence
    exists, not that the capability is known to be missing — and recording that
    honestly is the entire value of the artifact. A cell only turns into `software`,
    `simulated_fpga`, `hardware_replay` or `live_rf` when a check produces that
    evidence.

## Evidence levels

| Level             | What it means                                                                       |
|:----------------- |:----------------------------------------------------------------------------------- |
| `software`        | The software receive path, over reference-harness samples or a recording.           |
| `simulated_fpga`  | The simulated hardware correlator (`test/simulated_fpga.jl`), fed the same samples. |
| `hardware_replay` | Recorded samples replayed through real gateware.                                    |
| `live_rf`         | A live antenna through the hardware-correlator path.                                |

Only `software` evidence exists today. The other three become reachable as the
gateware steps of the roadmap land, and the harness is deliberately arranged so they
compare against the *same* reference rather than against each other.

## The matrix

<!-- BEGIN GENERATED MATRIX -->
| Family | Signal | replica | acquisition handover | tracking | secondary sync | data decode | pvt |
|---|---|---|---|---|---|---|---|
| GPS L1 C/A | `GPSL1CA` | software (harness_replica) | software (harness_acquisition) | software (harness_receive) | n/a (no_secondary_code) | software (ion_recording) | software (ion_recording) |
| GPS L1C | `GPSL1C_D` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | n/a (no_secondary_code) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `GPSL1C_P` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
| GPS L2C | `GPSL2CM` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | n/a (no_secondary_code) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `GPSL2CL` | software (harness_replica) | n/a (acquisition_window_too_long) | untested (l2cl_tracking_sweep_pending) | n/a (no_secondary_code) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
| GPS L5 | `GPSL5I` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | simulated_fpga (harness_hardware_overlay) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `GPSL5Q` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
| Galileo E1 | `GalileoE1B` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | n/a (no_secondary_code) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `GalileoE1C` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
|  | `GalileoE1B_BOC11` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | n/a (no_secondary_code) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `GalileoE1C_BOC11` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
| Galileo E5 | `GalileoE5aI` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `GalileoE5aQ` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
|  | `GalileoE5aQP` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | n/a (no_secondary_code) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
|  | `GalileoE5bI` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `GalileoE5bQ` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
| Galileo E6 | `GalileoE6B` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | n/a (no_secondary_code) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `GalileoE6C` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
| BeiDou legacy/B2 | `BeiDouB1I` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `BeiDouB3I` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `BeiDouB2bI` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | n/a (no_secondary_code) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `BeiDouB2aI` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `BeiDouB2aQ` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |
| BeiDou B1C | `BeiDouB1C_D` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | n/a (no_secondary_code) | untested (decode_sweep_pending) | untested (pvt_sweep_pending) |
|  | `BeiDouB1C_P` | software (harness_replica) | software (harness_acquisition) | untested (tracking_sweep_pending) | untested (secondary_sync_sweep_pending) | n/a (pilot_no_data) | untested (pvt_sweep_pending) |

### Roles

- **replica** — The code replica is generated correctly: unit-power, balanced, correlating to a single dominant peak at the stated code phase, isolated from other PRNs, and (where there is one) carrying the right secondary code.
- **acquisition handover** — A cold acquisition detects the satellite and hands over a code phase and Doppler accurate enough for the tracking loops to pull in.
- **tracking** — The code and carrier loops hold lock and keep the replica aligned.
- **secondary sync** — The receiver synchronises to the secondary (overlay) code, so integration can extend past one primary code period.
- **data decode** — The navigation message is demodulated and decoded into a usable ephemeris.
- **pvt** — The satellite contributes valid measurements to a position/velocity/time fix.

### Evidence sources

- **`harness_acquisition`** (`test/signal_validation.jl`) — The reference harness's per-signal acquisition check: a cold `acquire!` over harness samples has to detect the satellite and hand over a code phase and Doppler within the stated tolerances.
- **`harness_hardware_overlay`** (`test/secondary_code_removal.jl`) — GPS L5I through the simulated hardware correlator of `test/simulated_fpga.jl` over reference-harness samples: the device replicates the primary code only, `Tracking`'s own detector finds the NH10 overlay, the link then removes it from every dump, and the decoded symbols are the ones the harness transmitted — at close to the full ten blocks of energy per symbol rather than the overlay's own sum of two.
- **`harness_receive`** (`test/signal_validation.jl`) — A full `receive` run over harness samples: the satellite has to be acquired, tracked and held in lock with its C/N₀ within tolerance of the case's.
- **`harness_replica`** (`test/signal_validation.jl`) — The reference harness's per-signal replica check: unit code power, code balance, peak alignment against a fractional code phase and a Doppler, main-peak dominance, cross-PRN isolation and secondary-code agreement, all against `ReferenceHarness`' noise-free reference.
- **`ion_recording`** (`test/ion_rtlsdr_integration.jl`) — The 60 s ION RTL-SDR live-sky GPS L1 recording through the software receive path, asserted against a captured baseline: eleven healthy satellites, decoded ephemerides, and a PVT fix repeatable to one metre per ECEF component.

### Why a cell is not supported

- **`acquisition_window_too_long`** — One coherent acquisition window is a whole primary code period, and this signal's is 1.5 s — 3 million samples at four samples per chip. So L2CL is never the signal a receiver acquires: `acquisition_signal` falls back to the pairing's L2CM data component above ~0.67 Hz Doppler resolution, and L2CL is handed over from L2CM's code phase rather than searched for. A property of the signal, not a gap in the evidence.
- **`decode_sweep_pending`** — A decoder exists for this signal (`GNSSDecoderState` has a method), but nothing has demonstrated a decode. Synthetic samples cannot: the harness modulates a reproducible bit stream, not a navigation message with a preamble, parity and an ephemeris. Evidence has to come from a recording or live sky.
- **`l2cl_tracking_sweep_pending`** — The hardware path's *timing and accounting* for this signal are validated and the code loop does pull in: test/partial_primary_records.jl runs GPS L2CL through the simulated correlator of test/simulated_fpga.jl dumping inside its 1.5 s primary code period, the summed dumps reproduce the harness's `reference_correlation` over the same span, the loops are handed a record every `max_integration_time` instead of once per 1.5 s code wrap, and no short record is counted as a completed code period. That is not a tracking sweep. Nothing has yet shown the *carrier* loop holding lock on L2CL — in the same simulated run its Doppler estimate barely moves against a deliberate offset, while GPS L1 C/A through the identical harness converges — and whether that is a property of the signal, of the 20 ms coherent window a 1.5 s code forces, or of `Tracking`'s per-signal support is the subject of the software-support audit (JuliaGNSS/Tracking.jl#236) and the per-signal tracking sweep of step 9, not of the record accounting.
- **`no_secondary_code`** — The signal has no secondary code (`get_secondary_code_length` is 1), so there is nothing to synchronise to.
- **`pilot_no_data`** — A pilot component carries no navigation data (`get_data_frequency` is 0 Hz), so there is no message to decode. It contributes to a fix through its `CombinedSignal` pairing with the data component, not on its own.
- **`pvt_sweep_pending`** — No fix has been computed with this signal contributing. Follows the decode sweep for a data component, and the tracking sweep for a pilot, which contributes pseudoranges through its `CombinedSignal` pairing.
- **`secondary_sync_sweep_pending`** — The receiver's own secondary-code synchronisation is not swept per signal yet. The harness does verify that the secondary code is recoverable from the reference (it is part of the replica check), which is the prerequisite, not the capability.
- **`tracking_sweep_pending`** — No per-signal tracking sweep exists yet. The harness can generate the case; what is missing is the run and its baseline — part of step 9 itself, and only meaningful once the loops it exercises are the ones the roadmap settles on.

<!-- END GENERATED MATRIX -->

## The reference harness

Every software cell above is produced by one generator and compared against one
reference model, both in `test/reference_harness.jl`. The point is that a later step
cannot invent its own: "the software path and the gateware agree" only means something
when both were handed the same samples and both were measured against the same
reference with the same stated tolerance.

A case is described entirely by its parameters — signal variant, PRN, sampling
frequency, intermediate frequency, Doppler, fractional code phase, carrier phase,
C/N₀ or amplitude, noise power, antenna count and steering vector, optional data
modulation, and a seed:

```julia
case = ReferenceCase(
    GalileoE1B();
    prn = 11,
    sampling_freq = 24.552e6,
    carrier_doppler = 1234.0,
    code_phase = 1513.4271,   # fractional chips
    cn0_dbhz = 45.0,
    num_ants = 2,
    steering = [1.0, 0.5im],
)
samples = generate_samples(case, 40_000)
```

Two properties make it usable as a shared reference:

  - **Determinism.** The same case produces bit-identical samples on any machine.
  - **Chunk independence.** The noise is drawn one sample at a time from a single
    seeded generator and the signal is a closed-form function of the absolute sample
    index, so reading the stream in 4000-sample chunks and reading it in one block give
    the same samples. A device replay and a direct correlation therefore see one signal.

`reference_correlation(case, layout, num_samples)` is the reference itself: what an
ideal correlator locked on the truth would accumulate, for a three-tap
(`epl_taps()`) or five-tap (`vepl_taps()`) layout, per antenna. Every path under test
is compared against *it*, never against another path, so a bug shared by two
implementations cannot pass as agreement.

### Tolerances

Comparisons report named deviations against an explicit budget rather than a bare
verdict, so a failure says which quantity moved:

| Quantity                     | Default budget | Why                                                                                                                  |
|:---------------------------- |:-------------- |:-------------------------------------------------------------------------------------------------------------------- |
| `code_phase_chips`           | 0.01 chips     | ~3 m on GPS L1 C/A, far below any loop's own jitter.                                                                 |
| `carrier_phase_cycles`       | 0.02 cycles    | ≈ 7°.                                                                                                                |
| `doppler_hz`                 | 1 Hz           | Below one acquisition bin at any coherent length the receiver uses.                                                  |
| `amplitude_relative`         | 5 %            | The replica-amplitude budget: a differently scaled replica lands here rather than silently in the code-phase number. |
| `cn0_db`                     | 1 dB           |                                                                                                                      |
| `correlation_relative`       | 5 %            | Per-tap complex agreement, normalised by the reference prompt.                                                       |
| `cross_correlation_relative` | 15 %           | The ceiling a *wrong* PRN must stay under; the worst Gold-code cross-correlation is only ~24 dB down.                |

Two derived budgets keep those numbers honest rather than aspirational:

  - `quantization_tolerances(num_bits)` widens the amplitude-related entries by a
    `num_bits` quantiser's own loss, so a comparison against a quantised path states its
    quantisation instead of assuming it away.
  - `statistical_tolerances(case, num_samples)` widens them by five standard deviations
    of the thermal noise *the case itself specifies*. A 45 dBHz signal integrated over
    one GPS L1 C/A code period carries 18 % accumulator noise, so no comparison at that
    C/N₀ and that window can hold 5 % — lengthening the window is what buys a tighter
    budget, and the budget says so.

The code-phase budget additionally carries `code_phase_resolution(case)`, half a
sample in chips. A sampled code is piecewise constant, so the correlation peak is a
plateau one sample wide rather than a point; no code-phase estimate can be read more
sharply than that, however clean the signal.

## Known limitations of the software evidence

  - **The generator models an infinite-bandwidth front end.** It evaluates the ideal
    code at the sampling instants; a real front end band-limits first, which rounds the
    chip transitions off and costs correlation power. This does not affect comparisons
    between the software path, a simulated gateware and a replay of these same samples —
    all three see the same stream — but absolute amplitudes against live RF will differ.
  - **BOC signals need a fast sampler.** A BOC(m, n) sub-carrier flips `2m/n` times per
    chip, and `GNSSSignals`' code generator refuses to produce a replica below one
    sample per half-cycle. The BOC(6, 1) component of Galileo E1B/E1C and GPS L1C-P puts
    that floor at 12.276 MHz — which is why those rows are acquired at 24.55 MHz while
    GPS L1 C/A manages at 4.092 MHz.
  - **Synthetic samples carry no navigation message.** The harness can modulate a
    reproducible ±1 bit stream, which is enough to make a tracking loop face bit
    transitions, but it has no preamble, parity or ephemeris. Every `data_decode` and
    `pvt` claim therefore has to come from a recording or live sky, which is why GPS L1
    C/A is the only row with one.
  - **Simultaneous all-band reception is a separate constraint.** A single sample stream
    carries one RF band, and the matrix says nothing about how many bands a given front
    end can receive at once.

## Reproducing it

The whole matrix is produced by the test suite:

```
julia --project=. -e 'using Pkg; Pkg.test()'
```

To run only the signal-validation part while iterating:

```
julia --project=. -e 'using Test, GNSSReceiver, GNSSSignals, Acquisition, Unitful;
                      using Unitful: Hz, ms; include("test/signal_validation.jl")'
```

After changing a cell in `test/signal_support.jl`, regenerate the block between the
generated-matrix markers on this page with `SignalSupport.render_markdown()`; the
suite fails while the two disagree.
