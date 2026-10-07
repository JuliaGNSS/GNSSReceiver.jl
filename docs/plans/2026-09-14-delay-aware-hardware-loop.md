# A delay-aware tracking loop for the hardware-correlator path

**Date:** 2026-09-14
**Status:** Implemented in GNSSReceiver on 2026-09-14 (`NCOReferencedPLLAndDLL`,
`src/nco_referenced_loop.jl`; see §8 for what was done, what was measured and
what remains). Root cause established on hardware and reproduced offline;
estimator prototyped against the simulated device and tried on the M2SDR.
**Context:** GNSSReceiver.jl PR #129 (`hardware-correlator-12`, head `03e7486`),
GNSSM2SDR.jl `master` at `58790aa`, issue #107.
**Prototype code and data:** `docs/plans/2026-09-14-delay-aware-hardware-loop/`
(this repo) and `~/review-pr129-fable` on `orin@orin2` (recordings in `rec/`).
The tooling now lives in `examples/analysis/` (§8); the prototype files are kept
as the evidence trail only.

## 1. The problem in one paragraph

`receive(::AbstractHardwareCorrelatorSDR, …)` with its defaults (18 Hz
FLL-assisted third-order PLL, `feedback_delay_epochs = 2`) cannot hold a lock on
the LiteX-M2SDR: every scan acquires 5–7 satellites and loses all of them within
5–15 s. The PR's example only works because it bypasses that method and lowers
the PLL to 12 Hz. The cause is **not** the signal and not the bandwidth as such:
the same recorded samples track cleanly at 18 Hz through the software receiver.
It is the **closed-loop feedback delay**: a correction computed from a record is
applied to the device NCO 3–4 ms after that record ended (DMA buffer completion
≈ 0.75 ms, up to one 2 ms raw chunk before the fold runs, up to 0.75 ms until the
service task commits the CSR), while the loop filter assumes it acts before the
next measurement. The proportional phase term, sized to remove ~1/3 of the phase
error per millisecond, then acts for 5–6 ms per measurement, overshoots, and the
carrier limit-cycles (±80 Hz Doppler swing, ~25 ms period, prompt phase never
settles). C/N₀ and code lock stay perfect — a 70 Hz error barely dents a 1 ms
correlation — so nothing looks wrong until the carrier-lock detector releases
the satellite after ~4.5 s. Bit sync never lands, so the chunk-coherent
pre-accumulation that would lengthen the update interval never engages.

## 2. Evidence (all reproducible from the files listed in §7)

### 2.1 Live runs on orin2 (PR head, GNSSM2SDR 58790aa, `-t 6,4`)

| Run | Setup | Result |
|---|---|---|
| A | unmodified example (12 Hz PLL, delay 1) | fix 38.7 s, 5 sats, 527 solutions/60 s, 0 lost records |
| B | `receive(sdr, …)` defaults | no fix in 300 s; 7 sats per scan, all lost ≤15 s |
| B1 | B + `feedback_delay_epochs = 1` | same failure |
| B2 | B + 12 Hz PLL | fix 43.4 s, 268 solutions/30 s |
| C | example + 0.7 s SIGSTOP after the fix | 0 lost records, 0 skipped epochs, fixes resume |
| D1 | defaults, instrumented, DMA0 recorded (`rec/D1_raw.sc16`, 92 s) | PRN 24 @ 48 dBHz: Doppler std 50 Hz, ±80 Hz swing, 25 ms period, no bit sync, dropped at 4.7 s |
| D2 | same at 12 Hz | holds 87 s; Doppler std 22–30 Hz (4–5× the software value) |

### 2.2 Same recording (D1), offline

| Path | PLL | Delay | PRN 24 Doppler std | Bit sync |
|---|---|---|---|---|
| software receiver (`replay_software.jl`) | 18 Hz | none | 5 Hz | 0.19 s |
| simulated device (`replay_simfpga.jl std`) | 18 Hz | 1 epoch | 3.6 Hz | 0.5 s |
| simulated device | 18 Hz | 2 epochs | 3.6 Hz (weak PRN 25 hunts) | 1.8 s |
| simulated device | 18 Hz | 3 epochs | 48 Hz, dropped 4.5 s | never |
| simulated device | 18 Hz | 4 epochs | 47 Hz, dropped 4.3 s | never |
| simulated device | 12 Hz | 4 epochs | 3.1 Hz (weak PRN 25: 19.5 Hz) | 1.1 s |

The simulated device at 3–4 epochs of delay reproduces the hardware run almost
line for line, including the time to drop. Dump-vs-raw timing jitter measured on
hardware: the newest record's lead over the raw chunk end spans 2.2 ms (p5–p95),
i.e. the fold sees the dumps anywhere within one chunk.

### 2.3 What is *not* the cause

- Acquisition latency (seconds). It is a one-off handover delay outside the loop;
  the estimates are propagated to the commit sample and the loop pulls in once.
- Two records per 2 ms chunk folded with one NCO update. The software receiver
  has the same structure (4 ms chunks in the ION test) and is stable.
- Integrator windup during the dead time. The third-order integrator paths add
  ~1 Hz over two extra records; negligible. The proportional term is the problem.

## 3. Design: loop internals referenced to the device NCO

Principle (the reviewer's wording, confirmed by the prototype): **update the loop
only from what actually happened at the NCO — and what is already committed to
happen there.** No correction of a correction.

Per record integrated over `[a, b)` with applied carrier/code word `w(t)` known
from a per-channel timeline:

1. **Phase error `φ`** from the prompt is the true accumulated error under the
   applied words. It drives the frequency and rate integrators exactly as in
   Tracking's `ThirdOrderAssistedBilinearLF` (`1.1·Δt·ω₀²·φ`, `Δt·ω₀³·φ`).
2. **Frequency measurement is absolute:** `f_meas = w̄ + fll_disc`, where `w̄` is
   the mean applied word over the record. The integrators are driven by
   `f_meas − f̂`, not by a discriminator relative to a word the filter merely
   assumed was applied.
3. **The proportional term acts on the phase error predicted at the landing
   sample of the new command:** `φ_land = φ + 2π ∫_b^{land} (f̂ − w_sched(τ)) dτ`,
   integrating over the words already scheduled at the NCO. The command is
   `f̂ + 2.4·ω₀·φ_land`. If a command already in flight will remove most of the
   error before this one lands, this one is sized for the remainder.
4. **DLL** is Tracking's, normalised with the applied code word.

The negative control matters: re-basing on the applied word **without** step 3
fails exactly like the standard loop (§4). The scheduled words are part of "what
will happen at the NCO"; leaving them out is what makes each new command restate
the whole correction.

Requirements this creates:

- The **apply time of every word must be known exactly** (device honours
  `apply_at_sample`, or reports the commit sample). On the M2SDR today words apply
  on arrival, ≈1 ms before the scheduled sample; that mismatch is the identified
  cause of the prototype's residual noise on hardware (§4).
- The estimator needs, per record, the applied carrier and code word. The link
  is the only party that can provide it.

## 4. Prototype results

`nco_aware_pll.jl` implements §3 as a custom `AbstractDopplerEstimator`
(`NCOAwarePLL`, per-sat `SatNCOAwarePLL`, `predict::Bool` switches step 3). The
link tells it what it scheduled via a `note_scheduled_nco!` hook added to
`push_nco_updates!` (`note_scheduled_nco_hook.patch`, applies to PR head).

Simulated device, recording D1, 18 Hz, PRN 24 at 49 dBHz (weak PRN 25 at 38 dBHz
in brackets), Doppler std after 2 s and time to bit sync:

| Delay | Standard loop | NCO-aware, `predict = false` | NCO-aware, `predict = true` |
|---|---|---|---|
| 1 epoch | 3.6 Hz, 0.5 s | | 11 Hz, 0.4 s |
| 3 epochs | 48 Hz, dropped 4.5 s | 47 Hz, dropped 4.3 s | 10 Hz, 0.5 s (18 Hz, 4 s) |
| 4 epochs | 47 Hz, dropped 4.3 s | 43 Hz, dropped 4.3 s | 9 Hz, 0.5 s (20 Hz, 5.5 s) |
| 6 epochs | | | 8 Hz, 0.4 s (19 Hz, 5.5 s) |

Live on the M2SDR at 18 Hz with the default 2-epoch delay (`debug_nco.jl … nco`,
run D3, `rec/D3_*`): the 63 s scan's six satellites held for the rest of the run,
bit sync in 1.9–4.6 s on the 40 dBHz ones, Doppler std 18–19 Hz; the standard
18 Hz loop loses everything within 15 s. The first scan (mostly 23–27 dBHz that
day) was still lost, and a 42 dBHz satellite hunted at 52 Hz std. Two known
shortfalls, both addressed by the plan:

- **Apply-time model mismatch on the M2SDR** (applies on arrival, not at
  `apply_at_sample`), so records near a word switch are attributed to the wrong
  word and the landing prediction is ~1 ms off.
- **Noise at zero delay:** 11 Hz std vs 4 Hz for the standard loop. The prototype
  reuses Tracking's gain coefficients in an observer structure they were not
  designed for; the gains need a proper (Kalman-style) design.

## 5. Plan

### Phase 0 — Tooling into the repo (small, independent, do first)

- [ ] `examples/analysis/record_and_log.jl`: the `debug_nco.jl` run — tee DMA0 to a
      file via `start_raw_stream(command = sh -c "m2sdr_record -c 0 -q - 0 | tee FILE")`,
      one CSV row per chunk per satellite via `extract` (Doppler, code phase,
      prompt, records folded, C/N₀, bit sync, lock, link counters). Document that
      Julia block-buffers stderr to a file (pipe through `cat` for live logs).
- [ ] `examples/analysis/replay_software.jl` and `replay_simfpga.jl` (recording →
      software receiver; recording → simulated device with `feedback_delay_epochs`
      and estimator mode). Raw layout: 2R2T sc16, four `Int16` per sample,
      antenna 1 = words 1–2.
- [ ] Promote `SimulatedFPGA` from `test/hardware_correlator.jl` to a shared test
      utility so tests and the replay share one device model. Give it 20 channels
      by default (8 let first-scan false alarms starve the real satellites).
- [ ] Precompile or warm up a generic output path: a non-default `extract`
      recompiles the pipeline inside the first live chunk (~1–1.5 s, measured as
      1045–1544 skipped epochs and 10–14 lost-record gaps at t ≈ 5 s in every
      instrumented run). At minimum document "start the device after the first
      chunk" for instrumented runs.

### Phase 1 — Make the applied word knowable (GNSSM2SDR + link)

- [ ] **GNSSM2SDR:** apply NCO words at `apply_at_sample` from a real queue in the
      service task (`_drain_ncos!` writes a word when `sample_count(bank) ≥
      apply_at`), keeping the 20 ms staleness discard. Alternative if the queue is
      not wanted: publish the actual commit sample per channel (an atomic, like
      `assignment_start`), so the receiver can build the timeline from facts.
- [ ] **Link:** per hardware channel, a timeline of `(effective_sample, carrier,
      code)` — applied word plus scheduled words — fed from `push_nco_updates!`
      (and from the vendor's confirmation when available).
- [ ] **Link:** cut records at word boundaries the same way they are cut at bit
      edges, and attach the applied carrier/code word to each record handed to the
      estimator. Preferred carrier: a replica-Doppler field on Tracking's
      `CorrelatorOutput` (backward compatible, default `NaN`); the
      `note_scheduled_nco!` hook is the stopgap the prototype uses.
- [ ] **Link:** log `commit_sample − record_end` as a latency diagnostic
      (`nco_latency` counters), so the delay is measured, not inferred.

### Phase 2 — Delay-aware estimator (Tracking)

- [ ] New estimator (name TBD, e.g. `NCOReferencedPLLAndDLL`) implementing §3 with
      properly designed gains: observer form so that zero-delay performance
      matches `ConventionalAssistedPLLAndDLL` (target ≤ 5 Hz Doppler std at
      49 dBHz on recording D1), delay taken from the link (scheduled words, or a
      configured `delay` when the device applies on arrival).
- [ ] Handle multi-record chunks and records cut at bit edges/word boundaries;
      carrier-bandwidth `1/N` scaling as Tracking does; DLL normalised with the
      applied code word.
- [ ] `carrier_doppler_pull_in_range` method in GNSSReceiver (FLL bound, as for the
      assisted filter).
- [ ] Unit tests: synthetic phase step and frequency step with feedback delay
      d = 0…6 records; response must be the same for every d. A `predict = false`
      variant kept as a documented negative control.

### Phase 3 — Receiver integration and defaults

- [ ] `receive(::AbstractHardwareCorrelatorSDR, …)` defaults to the delay-aware
      estimator at the signal's reference bandwidth (18 Hz for GPS L1 C/A);
      `doppler_estimator` stays overridable. The example goes through the public
      method so the two cannot drift apart again.
- [ ] Shrink the delay independently: fold when dumps arrive (or when either
      stream arrives), not only on raw chunks; offer 1 ms chunks; keep the 80 kHz
      strobes.
- [ ] Handover under channel scarcity: assign the strongest detections first;
      guard the first scan against a window straddling the startup stall (seven
      false alarms with C/N₀ = −∞ blocked every real satellite for a minute in run
      D1).
- [ ] Revisit the two PR review findings that interact with this: dropped
      `NCOUpdate`s are not counted (`push_nco_updates!` returns 0 silently), and
      `should_reacquire`'s new `max_reacquire_attempts = 0` default reaches the
      software receiver.

### Phase 4 — Verification

- [ ] Closed-loop tests on the simulated device: 18 Hz at 1, 3 and 4 epochs of
      delay must lock with bounded Doppler std; the standard loop at 3 epochs is the
      documented failing reference.
- [ ] Hardware acceptance (recorded and logged, GNSSM2SDR with Phase 1): 18 Hz,
      bit sync < 2 s and Doppler std < 10 Hz for satellites ≥ 40 dBHz, a fix, zero
      lost-record gaps outside startup; archive the recording for replay.
- [ ] Re-run the PR's own results table with the new defaults.

## 6. Open questions

- Queue at `apply_at_sample` vs. apply-on-arrival with a reported commit sample:
  the queue gives exact timing but adds latency equal to the margin; reporting
  keeps latency minimal but needs a second host→device→host round trip of
  information. Prototype both on the simulated device before deciding.
- Where the delay-aware estimator lives: Tracking (natural home for a
  `AbstractDopplerEstimator`) or GNSSReceiver (the only package that knows about
  device NCOs). Tracking needs at least the replica-Doppler-per-record carrier.
- Whether `feedback_delay_epochs` should remain a user knob or be replaced by the
  measured latency plus a margin.

## 7. Files and environments

In this directory (`docs/plans/2026-09-14-delay-aware-hardware-loop/`). Only
`gnssm2sdr_scheduled_nco_apply.patch` is committed; the prototype scripts below
are superseded by `examples/analysis/` and `src/nco_referenced_loop.jl` and
were left out of the repository (copies remain on orin2, §7 bottom):

| File | What |
|---|---|
| `nco_aware_pll.jl` | prototype estimator (§3), included by the replay/run scripts |
| `note_scheduled_nco_hook.patch` | 16-line link hook; `git apply` on PR head `03e7486` |
| `replay_simfpga.jl` | recording → simulated device; args `RAW CSV DELAY_EPOCHS PLL_HZ SECONDS [std|nco|nconp]` |
| `replay_software.jl` | recording → software receiver; args `RAW CSV [PLL_HZ] [SECONDS]` |
| `debug_nco.jl` | live M2SDR run with DMA0 recording and per-chunk CSV; args `RAW CSV [MAX_S] [PLL_HZ] [std|nco|nconp]` |
| `simsum.py`, `analyze.py` | CSV summaries used for the tables above |

The scripts expect an environment with GNSSReceiver, GNSSM2SDR, GNSSSignals,
Tracking, Unitful, Geodesy, Printf `dev`ed/added, and the CSR map at
`~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch20_ant1/csr.csv` on the Orin. They call
Tracking internals (`_apply_correlator_output`, `_process_passenger_signals`,
`_signal_noise_densities`, `_foreach_group!`, `_band_sampling_frequency`); the
real estimator must replace those calls with public API or live in Tracking.

On `orin@orin2` (`export PATH=$HOME/.juliaup/bin:$PATH`, Julia 1.12.6):
`~/review-pr129-fable/{GNSSReceiver,GNSSM2SDR,env}` — GNSSReceiver has the hook
patch applied uncommitted; `rec/D1_raw.sc16` (18 Hz failure, 92 s), `rec/D2_*`
(12 Hz), `rec/D3_*` (prototype live run), `rec/S_*.csv` / `rec/N_*.csv`
(simulation sweeps), `run*.log`. One receiver process at a time on the device;
`pkill -x m2sdr_record` before a run if a previous one died.

## 8. Implementation record (2026-09-14)

### What was built

**Estimator — `NCOReferencedPLLAndDLL` (GNSSReceiver, `src/nco_referenced_loop.jl`).**
It lives in GNSSReceiver rather than Tracking (open question 2 resolved for now:
it is the only package that knows about device NCOs, and Tracking is a registry
dependency here). It implements §3 with one deliberate change to step 2, made
because the prototype's gains were the problem it named in §4:

- The filter and gains are Tracking's `ThirdOrderAssistedBilinearLF` verbatim.
  The estimator is a **Smith predictor** around the conventional loop: each
  record is attributed to the word that really ran under it (timeline), and the
  filter is stepped with the discriminators *predicted at the landing sample*
  of the command this fold produces — the phase error advanced by
  `2π ∫ (f̂ − w(τ)) dτ` over the scheduled words, and the absolute frequency
  measurement `applied word + FLL` taken relative to the word that will be
  running under the record at landing (§3 step 2 said "relative to `f̂`"; that
  observer form is what made the prototype 11 Hz vs 4 Hz at zero delay).
- With no delay both steps are the identity and the estimator is the
  conventional loop **bit for bit** (tested). The zero-delay noise target
  (≤ 5 Hz at 49 dBHz) is therefore met by construction.
- Multi-record folds map every record by the same shift, `landing − end of the
  fold's last record` (a per-record shift is wrong for the second record of a
  2 ms chunk).
- `predict_landing = false` is the documented negative control.
- Through the software receiver (`track!`) it runs with the chunk's replica
  Doppler and no delay, i.e. as the conventional loop.

**Link (`src/hardware_correlator.jl`).**

- Per hardware channel an `NCOTimeline`: the applied word plus the scheduled
  words, reset to the handover words on `assign_channel!`, extended by
  `push_nco_updates!` with every update the device *accepted*, folded forward
  once the records that ran on a word are folded. `mean_nco_word(timeline, a, b)`
  is the time-weighted replica word over a span.
- Records are cut where a scheduled word lands between two dumps, as they are
  cut on bit edges; a switch inside a dump is weighted by `mean_nco_word`.
- The fold decides the landing sample (`scheduled_apply_at_sample`) before the
  estimator runs; `estimate_dopplers!(link, …)` dispatches the NCO-referenced
  estimator with the timelines and that sample, and Tracking's estimators
  unchanged.
- Refused `NCOUpdate`s are counted (`dropped_nco_updates`), warned about and
  kept out of the timelines — the first of the two review findings in Phase 3.

**Defaults and example.** `receive(::AbstractHardwareCorrelatorSDR, …)` defaults
to `NCOReferencedPLLAndDLL()` at the signal's reference bandwidth (18 Hz for
GPS L1 C/A) unless `doppler_estimator` or `vector_tracking` is given, and takes
a pre-built `link` so callers can read its counters. The M2SDR example goes
through the public method with the defaults (2-epoch delay, 18 Hz) and drives
all 20 gateware channels (the Phase 3 scarcity item, in its cheap form).

**Tooling (Phase 0).** `examples/analysis/{record_and_log,replay_software,
replay_simfpga,tracking_log}.jl` and a README; `SimulatedFPGA` promoted to
`test/simulated_fpga.jl` (20 channels by default, records the sample each word
was actually applied at, `replay_raw_file!` for recordings) and shared by the
tests and the replay. The instrumented live script takes the first payload
before enabling the device, so the `extract` recompilation happens with the
dump ring empty.

**Tests (Phases 2 and 4).** `test/nco_referenced_loop.jl`: timeline arithmetic;
a noise-free model of the delayed loop on which the estimator is bit-identical
to the conventional loop at d = 0, holds lock at d = 1…6 records and pulls in
±100–120 Hz offsets, while the conventional loop and the negative control
limit-cycle at d = 4 (±80 Hz swing, the board's signature); link-level tests of
the timeline, the word-boundary cut and the landing sample reaching the
estimator. Closed loop on the simulated device (`test/hardware_correlator.jl`):
18 Hz at 1, 3 and 4 epochs of delay on 2 ms chunks locks with the true Doppler
within 15 Hz; the conventional 18 Hz loop at 3 epochs does not converge and is
kept as the failing reference.

### Hardware runs (orin2, GNSSM2SDR `58790aa`, 20 channels, 18 Hz, 2-epoch delay, `-t 6,4`)

| Run | Vendor NCO apply | Result |
|---|---|---|
| E1 | on arrival (stock) | 4 satellites (34–39 dBHz) held for the whole 120 s run, bit sync on all four, 0 dropped NCO updates, 7 lost-record gaps (all in the ~1.2 s start-up stall). Phase std at 39 dBHz 0.46–0.51 rad (measurement floor 0.25 rad), Doppler std 18 Hz; bit sync only after 40–70 s of lock. Three 28–35 dBHz first-scan satellites lost within 30 s. |
| D3 (prototype, for comparison) | on arrival | 40 dBHz: phase std 0.35–0.38 rad, bit sync ≈ 60 s after lock |
| D2 (12 Hz conventional) | on arrival | 38 dBHz: phase std 0.24 rad, bit sync 12 s after lock; 46 dBHz: 0.13 rad, immediate |
| E2 | held until `apply_at_sample` (patch to `_drain_ncos!`, see below) | 6 satellites held at once (32–41 dBHz), the two 32 dBHz ones lost after 95 s; 0 dropped NCO updates. Words landed **1.0 ms late on average** (max 13 ms; the service loop only wakes per DMA buffer, ~2 ms): the mismatch moved rather than vanished. Bit sync on the 36–41 dBHz satellites 4–5 s after lock (E1: 40–70 s). Phase std at 40.7 dBHz 0.51 rad, Doppler std 17–20 Hz. |
| R1 (software receiver on recording D1, 18 Hz; the floor) | — | phase std 0.41 rad at 37 dBHz, 0.19 rad at 43 dBHz, 0.09 rad at 49.5 dBHz; bit sync 0.2–2 s |
| D1 (conventional 18 Hz on hardware) | on arrival | phase std 0.9 rad on every satellite — the uniform distribution of a carrier that is not phase-locked |
| E3 | held, and the service loop's poll bounded by the next due word | Still **1.0 ms late on average** (max 12.9 ms): the lag is the CSR write path, not the wake-up. Phase std 0.40–0.45 rad at 37–38 dBHz — the software receiver's own 0.41 rad at 37 dBHz; Doppler std 14–22 Hz (software at 37 dBHz: 21 Hz jitter). Bit sync 3–11 s after (re)lock on the ≥ 35 dBHz satellites of the 60 s scan; the first scan's 30–37 dBHz satellites were all lost by 38 s (signal conditions differ run to run; E2 held six). |

Reading: the loop is stable at 18 Hz through the real delay (the conventional
18 Hz loop loses everything within 15 s on this device), but on hardware it runs
at about 1.5× the software receiver's phase noise at equal C/N₀, where the
simulation with exact timing shows no such excess. The known apply-time
mismatch (§4) is the prime suspect: the vendor applies each word on arrival, up
to ~1 ms before `apply_at_sample` and jittering with host timing, so records are
attributed to the wrong word for part of their span and the landing prediction
is off by that much. E2 tests exactly that with a vendor-side queue.

**GNSSM2SDR patch (uncommitted, `~/review-pr129-fable/GNSSM2SDR/src/sdr.jl` on
orin2).** `_drain_ncos!` now holds each update in a per-channel slot (newest
wins) and writes the immediate `carrier_freq`/`code_freq` CSRs at the first
service pass at or after `apply_at_sample`, keeping the 20 ms staleness discard;
`stop!` logs the number of commits, the mean and maximum lag past the scheduled
sample, and the stale drops. This is Phase 1's first item in its queue form; the
landing error becomes one-sided and bounded by the service pass (~0.75 ms).

### Not done, and why

- **Reacquisition knobs** (second Phase 3 review finding). `should_reacquire`'s
  `(10 s, 0 attempts)` default still reaches the software receiver. Exposing
  the knobs means threading a policy through the positional
  `invariant_acq_args` tuple across a dozen signatures in `process.jl` and
  `async_acquisition.jl`; that is a refactor orthogonal to the loop and is left
  as a follow-up with this note. The behaviour is unchanged from PR head.
- **Fold when dumps arrive / 1 ms chunks** (Phase 3). Not attempted; the delay
  is now compensated rather than minimised. Whether the Orin keeps up with 1 ms
  chunks has not been measured.
- **Strongest-first handover** (Phase 3). Fresh satellites carry no C/N₀ and the
  acquisition strength is not retained in the tracked state, so there is
  nothing to order by without a new field; driving all 20 channels removes the
  scarcity instead.
- **Reported commit sample** (open question 1, alternative form). Not built; the
  queue was tried first. If E2's residual noise warrants it, the vendor can
  publish `(scheduled, actual)` per channel and the link can re-time the
  matching timeline entry before promotion.
- **Tracking internals.** The estimator calls `Tracking._apply_correlator_output`,
  `_process_passenger_signals`, `_signal_noise_densities`,
  `_warn_noise_density_missing`, `_band_sampling_frequency`,
  `_copy_groups_slot_vectors` and the sync-snap helpers, exactly as the
  conventional estimator does internally. Upstreaming (a public per-record
  fold in Tracking, or the estimator itself) is the follow-up §7 asked for.
- **Hardware acceptance** (Phase 4: bit sync < 2 s, Doppler std < 10 Hz at
  ≥ 40 dBHz, a fix). Not demonstrated: no run had a ≥ 42 dBHz satellite for
  long, and at 37–39 dBHz the software receiver itself sits at 0.4 rad / 21 Hz,
  so the thresholds are only testable on a stronger pass. What the three runs
  do show: the loop is stable at 18 Hz through the real 2-epoch delay in every
  run (the conventional loop is not), zero dropped feedback, and with the
  vendor holding words to their scheduled sample (E2/E3) the phase noise is
  within 1.0–1.3× of the software receiver at equal C/N₀ and bit sync arrives
  in seconds rather than a minute. The PR's results table has not been
  re-run; the GNSSM2SDR patch (`gnssm2sdr_scheduled_nco_apply.patch` in the
  plan directory) is uncommitted on orin2 and should go to GNSSM2SDR.jl.
- **Weak satellites.** In all runs the 28–35 dBHz satellites of a scan are lost
  within a minute while ≥ 36 dBHz ones hold. The software receiver's phase
  noise at 37 dBHz is already 0.41 rad; below that the 18 Hz / 1 ms loop is
  marginal in either receiver. Lengthening the coherent integration once bit
  sync lands (the chunk-limited pre-accumulation) is where the margin is.
- **Stale dumps.** ~4 k dumps/s arrive for channels the link has no occupant
  for (`stale_dumps` ≈ 500 k per 120 s with 20 channels driven): the gateware
  dumps on every channel whether assigned or not. Harmless to the loop, but it
  is DMA bandwidth and drain time a vendor could save by gating idle channels.
