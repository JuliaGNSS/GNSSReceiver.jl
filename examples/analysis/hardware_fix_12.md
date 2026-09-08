# Issue 107: position timing and runtime latency

For a portable pinned environment, exact vendor source, archived evidence,
and a launcher that preserves each run separately, see
[repro_hwfix12/README.md](repro_hwfix12/README.md). The companion decoder fix is
GNSSDecoder commit `28725906711265c341b59542bb4a5dabc99a9e87`.

Test host: `orin@orin2`, 2026-09-08. The baseline uses
`hardware-correlator-11` at `6de9940`; the new work is based on `eb956d5`.
The antenna and RF configuration were unchanged.

## Defect

Before bit synchronization, the hardware path wraps code phase at one primary
code period. Synchronization can occur partway through a chunk containing
several records. The bit buffer counts the remaining records, but the phase
still lacks their integer code-period count. Tracking's secondary-code phase
snap does not correct GPS L1 C/A, which has no secondary code.

In `hc11n_bits.log` (run 16), the first synchronized snapshots have:

| PRN | Complete periods in open bit | Last-record to fold delay (ms) | Reported phase (chips) | Corrected phase (chips) |
|---|---:|---:|---:|---:|
| 19 | 1 | 0.92250 | 943.467 | 1966.467 |
| 20 | 0 | 0.66325 | 678.323 | 678.323 |
| 15 | 1 | 0.27075 | 276.777 | 1299.777 |
| 24 | 1 | 0.40275 | 411.993 | 1434.993 |

The delays were checked against the corresponding `.records` file, using the
latest record strictly before the actual fold boundary (the logged `P` boundary
is the *next* boundary). The discrepancy persists throughout each assignment.
One primary period is 1 ms, or approximately 300 km of range. A relative error
between satellites cannot be absorbed by the common receiver clock. Even a
shared one-period offset becomes a relative whole-bit error when one satellite
wraps its 20 ms phase window before another; this explains why a run can alternate
between correct solutions and extreme excursions.

`anchor_bit_phases!` joins the two clocks once, after the estimator first finds
bit synchronization: complete periods from the bit buffer, signed phase residual
at the latest record, and elapsed phase from that record to the common reception
boundary. It preserves fractional phase and handles either side of code wrap.
Channel release/reassignment resets the anchoring flag. Secondary-code and
multi-signal states retain their existing behavior.

## Recorded-data validation

`replay_pvt.jl` re-decodes the actual logged soft bits and calls the PVT solver
with the recorded phase and with a constant integer-period correction inferred
at the first synchronized snapshot of each assignment. It does not re-run
tracking. Both solves start without a position hint; the reference is used only
to score their output. `calc_pvt` can retain its previous solution on failure,
so these statistics describe returned positions, not an integrity assessment.

Reference: approximately 50.7686° N, 6.0727° E, assumed ellipsoidal height 270 m.
Errors below are three-dimensional distances from that reference.

| Recording | Returned comparisons | Original median error | Corrected median error | Corrected error range |
|---|---:|---:|---:|---:|
| Run 6 (`hc11f`) | 93 | 34 m, with extreme excursions | 133 m | 10–244 m |
| Run 11 (`hc11k`) | 51 | 4452 km | 55 m | 13–153 m |
| Run 16 (`hc11n`) | 267 | 4013 km | 131 m | 15–484 m |

Run 16's corrected median PDOP is **25.7**. Removing the timing error therefore
does not establish consistently accurate positioning with this sky geometry.
Run 6's original median includes held previous solutions and hides its extreme
excursions; median error alone is insufficient to assess reliability.

Reproduce on the board with its original dependency environment:

```bash
cd ~/hwfix12/run
~/.juliaup/bin/julia --project=. replay_pvt.jl \
    ~/hwfix11/run/hc11n_bits.log 50.7686 6.0727 270
```

## Fresh baseline

The 240-second baseline acquired PRNs 24 and 25. Both eventually decoded
navigation data, but only one held lock through most of the run. There was no
position solution. Every channel had zero lost record samples, and the link
reported zero lost gaps and zero dropped dumps. Hardware-path warm-up was 3.8 s,
before streaming. Logs: `~/hwfix12/run/baseline.log` and
`baseline_bits.log{,.records}`.

The example now records cumulative compile/GC counters in each `T` line, writes
its exported bits beside the run script, and labels returned PVT solutions
without asserting that their accuracy has been validated.

## First-solve latency

In a fresh Julia process with both `GNSSReceiver` and `GNSSM2SDR` loaded, using
the receiver's concrete GPS PVT precompile fixtures:

| Call | Wall time | Compilation | Recompilation |
|---|---:|---:|---:|
| First real `calc_pvt` | 15.49 ms | 14.22 ms | 0 |
| Second | 0.34 ms | 0 | 0 |

This checks a successful solve even when the live sky supplies fewer than four
satellites. It does not measure the complete live processing chunk. The test
suite was running concurrently, so these wall times were not measured on an idle
host. Output: `~/hwfix12/run/pvt_first_cost.log`.

## Remaining decoder compilation exposed by the live run

The receiver-only patch was tested for another 240 seconds. Three satellites
(PRNs 25, 24, 32) decoded navigation data. Across 314,397 synchronized diagnostic
snapshots their integer code-period count agreed with their bit buffer; the
previous persistent one-period discrepancy was gone.

However, this run exposed a separate late compile stall: **302.8 ms** near
192 seconds, including **298.3 ms** of compilation. It coincided with three
record gaps; PRN 24's retained counter recorded 1,116,008 lost samples (~279 ms).
Total compilation after the first logged chunk was 0.760 s, and GC time was zero.
The first post-handover gap was 128 ms, including 121 ms compilation.
Logs: `~/hwfix12/run/patched.log` and `patched_bits.log{,.records}`.

Replaying only the recorded `B` lines through GNSSDecoder reproduced **291.9 ms**
of compilation at exactly the same signal epoch. The trace identified
`can_decode_word` specialized on the HOW closure containing
`{Int64, Nothing, Nothing}`: the symbol counter was running, but a previous
missing/rejected HOW had cleared both time-of-week history fields.

The companion GNSSDecoder patch captures the concrete decoder state rather
than those three optional scalars. This preserves the plausibility checks and
recovery behavior while giving cold start, normal decoding, and HOW recovery
the same closure type. Both patches are needed; the receiver phase correction
alone does not remove this decoder stall.

GNSSDecoder checkout: `/workspace/GNSSDecoder-hardware-correlator-12`, based on
v4.2.0 (`5534ff5`), branch `fix/lnav-recovery-compilation`. The board's copy is
`~/hwfix12/GNSSDecoder`; its run environment develops that local package.

## Satellite discovery

The example previously restricted every acquisition plan to the PRNs detected
above the opening scan's reporting floor. A start with two or three satellites
could never discover a newly visible fourth without restarting. Periodic scans
now search all 32 GPS PRNs, trying the initial detections first. CFAR continues
to gate handovers and the hardware link enforces its channel capacity. An empty
opening scan also continues into the receiver instead of aborting immediately.

With the companion patch, the same 34,153 decoder calls have **14.62 ms total
compilation**, with a maximum of **8.15 ms** (8.28 ms wall time) at the formerly
292 ms recovery event. The residual specialization is the small
`is_plausible_TOW(::UInt64, ::Nothing, ::Nothing, ::Int64)` helper. The first PVT
solve with the combined patches remained 15.00 ms wall / 13.90 ms compilation.

```bash
cd ~/hwfix12/run
~/.juliaup/bin/julia -t 6,3 --project=. \
    --trace-compile=decoder_fixed_trace.jl --trace-compile-timing \
    decoder_latency.jl patched_bits.log
```

Results: `decoder_fixed_cost.log`, `decoder_fixed_trace.jl`, and
`pvt_first_cost_final.log` in the board's run directory.

## Combined-patch live run

A 240-second run with both patches and all 32 PRNs searched every 60 seconds
completed with **zero lost record gaps, zero dropped dumps, and zero lost
samples on retained channels**. PRNs 24, 32, 29, and 25 decoded at different
points, but never supplied four usable satellites together. There was no live
position fix. PRN 25 was discovered on a later scan despite being absent from
the opening visibility report.

This run still incurred a 136 ms processing gap at the first handover,
including 130 ms compilation. **Record continuity does not establish timely
feedback:** the FPGA advances its replicas with previously programmed NCO
frequencies while the host stalls, but fresh loop corrections stop. Buffering
protects observations, not feedback latency. This remaining handover path needs
separate tracing and warm-up before calling the runtime latency resolved.

The test harness disables GC during these short runs. They do not validate
hours of operation with GC enabled, nor does a returned/stale PVT solution
establish current positioning integrity. The full-constellation searches also
promoted several candidates that quickly failed tracking; acquisition selection
under this obstructed sky remains a separate robustness concern.

Logs: `~/hwfix12/run/final.log`, `final_bits.log{,.records}`, and
`final_analysis.log`.

## Feedback latency follow-up

A fresh-process `--trace-compile=stderr --trace-compile-timing` run identified
cold diagnostic I/O methods behind the 130 ms handover pause. The previous
warm-up used `devnull` and no tracked satellites, so it missed the live
`IOStream` writes for prompts, bits, and phase diagnostics. It also used a
differently typed acquisition interval, leaving the processing closure to
compile again before the first logged chunk. The vendor assignment method
compiled separately on first use.

The example now warms a populated tracking/logging state with temporary
`IOStream`s, exercises soft-bit export and standalone reporting calls, uses
the live acquisition/loop configuration, and precompiles vendor assignment,
release, and dump-count methods before tracking. The warm-up also exercises
the first satellite merge, a successful PVT solve, and the HOW recovery helper.
The slip-watch timer first fires while its baseline is unset and tracking has
not started, warming its callback without changing the origin.

The first 65-second verification (`warm_trace.log`) removed the 130 ms I/O
pause and the large processing-closure/vendor-assignment specializations from
live tracking. It exposed another 26 ms satellite merge and an 11 ms timer
wrapper; those were then included in the final warm-up. Four satellites held
tracking during this short run, but not all decoded ephemerides before it ended.


## Live position and final latency measurements

`warm_final.log` records a fresh four-satellite fix at **91.0 s**, initially
50.769324° N, 6.072955° E, 190.0 m. Replaying its logged observations produced
170 fresh timestamp updates at half-second sampling, from signal time 91.55 s
to 176.17 s. The original and corrected replay paths agree, with every inferred
integer-period offset zero. Against the approximate reference, the fresh
positions have **78 m median 3D error**, **43–182 m range**, and **4.67 median
PDOP**. This validates removal of the large timing error; it is not surveyed
accuracy validation.

After PRN 25 faded, only three satellites remained usable. The solver held its
previous position through the end of the 240-second capture. The example now
counts only changed PVT timestamps and displays the age since the last fresh
solution; held output no longer counts as a continuing stream of fixes.

The run had zero lost record gaps, zero dropped dumps, and zero lost samples
on retained channels. Total compilation after the first logged chunk was
63.73 ms across 240.09 s. The largest processing gap was 33.72 ms **before the
first satellite assignment**; after assignment the largest gap was 12.41 ms,
with zero compilation in that gap. The two remaining traced live cold calls
were integer addition (6.8 ms) and the first-fix satellite-count query (8.1 ms).
Those have also been added to warm-up. These are host chunk-gap measurements,
not direct measurements of when each NCO command took effect on the FPGA.

A fresh process running the updated pre-stream warm-up and then replaying all
34,720 decode calls from this capture reported **0 ms compilation** and a
maximum decode wall time of 0.245 ms. Fresh/held PVT detection is asserted in
the same warm-up. Output: `ready_verification.log`.

The original successful capture segfaulted during Julia process exit, after
all records and final results were written. The harness previously closed log
sinks and re-enabled GC while its processing/device tasks could still run.
Shutdown now closes the output channel, waits for receiver/acquisition/raw-reader
termination, and stops and joins the FPGA service tasks before closing sinks
and re-enabling GC. This requires a separate shutdown verification; it does
not change the collected live positioning results.

## Automated validation and deployment

- Full GNSSReceiver test suite passed on the Orin, including the new 64 phase
  anchoring assertions and 188 integration assertions, recorded-data PVT tests,
  sample-slip recovery, and vector-tracking integration. Log:
  `~/hwfix12/testenv/tests.log`.
- The hardware-correlator test file passed again with both local packages loaded:
  `~/hwfix12/run/combined_hardware_tests.log`.
- GNSSDecoder validation passed 17,447 assertions in two stages. The initial
  suite passed 15,024 assertions, including all GPS L1 C/A tests and the new HOW
  recovery tests, then stopped because the board copy lacked the repository's
  BeiDou generator scripts. After deploying those resources, the remaining
  2,423 assertions passed. Logs: `~/hwfix12/decoder_testenv/tests.log` and
  `remaining_all.log`. The opt-in Flexiband capture test was not enabled.
- Julia parsing, shell syntax, and `git diff --check` passed.

The board environment `~/hwfix12/run` develops both `~/hwfix12/GNSSReceiver`
and `~/hwfix12/GNSSDecoder`. It retains the prior Tracking checkout
`~/hwfix11/Tracking`, GNSSM2SDR checkout `~/hwfix9/GNSSM2SDR`, Acquisition 2.8.0,
GNSSSignals 4.1.0, and PositionVelocityTime 5.4.0. The prepared example is
`~/hwfix12/run/position_fix_ready.jl`:

```bash
cd ~/hwfix12/run
HWFIX_PROC_POOL=interactive HWFIX_ACQ_EVERY=60 HWFIX_BITLOG=next_bits.log \
    ~/.juliaup/bin/julia -t 6,3 --project=. position_fix_ready.jl 600 500
```

The large cold stalls found in these GPS runs have been addressed. Continuous
reliability still requires enough satellites to hold carrier lock and decode
simultaneously, handling unsuccessful initial carrier pull-in and weak/spurious
acquisition candidates, and validating long operation with GC enabled. The
reference is approximate and the observed error should not be presented as
surveyed accuracy. No GNSS-only software change can guarantee a fresh 3D fix
when fewer than four independent usable GPS observations remain.

## Final shutdown retest

The prepared example completed a 100-second hardware run and shut down cleanly
(expected exit status 1 because it obtained no fix; no segfault or lingering
recorder). Its warm-up/fresh-versus-held assertions also passed. Logs:
`~/hwfix12/run/ready.log`, `ready_bits.log{,.records}`, and `ready_analysis.log`.

There were **no traced compilation calls during active tracking**. The 40.03 ms
process-wide compilation counted after the first `T` occurred before the first
satellite assignment or when closing the output at shutdown. Record loss and
dropped-dump counters remained zero. The largest post-assignment processing gap
was **25.68 ms with zero compilation**. Thus non-JIT scheduling/I/O jitter is
still observable; eliminating the identified cold methods does not guarantee
a hard feedback deadline. This retest exercised orderly shutdown without a
position fix; the successful-fix return path was not independently repeated.
