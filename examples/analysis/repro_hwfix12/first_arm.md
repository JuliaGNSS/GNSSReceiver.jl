# Confirmed first-arm boundary

A channel's PRN metadata changes before its scheduled replica phase load takes
effect. Dumps produced between those operations already match the newly
assigned PRN, but their integrations do not belong to that acquisition.
Increasing the DMA buffer cannot correct their phase provenance.

The adapter now revokes the previous assignment boundary before loading code
or scheduling an arm. It publishes the effective device sample only after the
commit verifier confirms an on-time arm. The receiver discards dumps while the
assignment is unconfirmed and discards any integration starting before that
sample, including integrations crossing the boundary and records already in
its input queue. The cutoff uses the same device sample counter as the dumps.

The first accepted integration establishes continuity for the fresh satellite.
Time before that observation is not credited to its bit clock. Reacquisition,
including the same PRN on the same channel, starts with a fresh satellite and
a newly revoked boundary. Failed arms remain unconfirmed. Per-channel locks
prevent a verifier for an old assignment from publishing its cutoff over a
new assignment. NCO updates are also rejected while the arm is unconfirmed.

This fixes the first-arm observation lifecycle. It does not change DMA buffer
capacity or remove the separate handling of actual gaps in an established
record stream.

## Reproduction

Use the adjacent `setup.sh` and `run.sh` with Julia 1.12.6 on `orin2`; both pinned
dependencies are unchanged. The vendored adapter in this checkout is required.
Run the adapter tests with:

```bash
julia --project=examples/analysis/repro_hwfix12 -e \
  'using Pkg; Pkg.test("GNSSM2SDR")'
```

The receiver suite includes first-arm and same-PRN reacquisition cases with
pending, queued old, boundary-straddling, and valid short integrations. The
adapter suite covers pending, confirmed, failed, released, and changed-PRN
verification states.

## Validation on 2026-09-09

- The full receiver suite passed locally on Julia 1.12.1, including all 22 new
  boundary assertions. The adapter suite passed, including 15 arm-verification
  assertions.
- An initial 100-second board run obtained a four-satellite fix after 39.9 s
  and returned 596 fresh timestamps. Its trace exposed missing warm-up of two
  reporting property reads and the new assignment wrapper. The final wrapper
  has an explicit signature, and reporting now warms those standalone reads.
- The final fresh-process run on Julia 1.12.6 used `run.sh 100 200` with
  `--trace-compile=stderr --trace-compile-timing`. It obtained a four-satellite
  fix after 90.5 s and returned 92 fresh timestamps. Acquisition at 60 s brought
  in additional satellites after early signals faded. Position accuracy was
  not independently validated.
- The final run reported `lost_gaps=0`, `rearm_gaps=0`, `dropped=0`. Retained
  channels had zero lost, re-arm, and overlapping samples. `stale=339175`
  includes both unassigned-channel records and rejected assignment records;
  it is not a count solely of premature first-arm dumps.
- Across 50,379 timing records there was no compilation before the 100-second
  stop. The only cumulative compilation increase was 8.41 ms at 100.063 s,
  consistent with the trace's shutdown `close` call. The largest observed
  chunk interval was 29.80 ms, so this is not a hard real-time guarantee.

The final status log and timing summary are in `verification/first-arm`.
Full traces, bit/record logs, tests, and the tested checkout are retained at
`orin2:/home/orin/first-arm-WYp5Hn`. The final RF run is
`examples/analysis/repro_hwfix12/runs/run-OR2FNL` there. The setup fetched the
same pinned Tracking and Decoder commits listed above; no dependency pins or
hardware buffers changed. For a traced reproduction, point `JULIA` at a shell
wrapper executing Julia with the two trace flags followed by `"$@"`.
