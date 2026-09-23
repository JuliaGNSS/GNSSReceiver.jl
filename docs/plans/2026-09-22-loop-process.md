# A separate, allocation-free loop process for the hardware correlator

**Date:** 2026-09-22
**Status:** Design agreed (interview of 2026-09-22); nothing implemented yet.
Milestone 0 (the trim spike) is the first thing to build.
**Context:** GNSSReceiver.jl PR #129 (`hardware-correlator-12`), the review of
2026-09-21 and its root-cause analysis of the hardware path's decode latency;
issue #142 / PR #143 (dead-input NaN in the software receiver, found on the way).
**Where the work lands:** this branch stays open as the integration branch; the
in-process link it carries is replaced piece by piece by what is described here.

## 1. Why

The hardware receiver decodes navigation data far later than its C/N₀ would
allow (PRN 5 at 43 dBHz: 64 s from bit sync to a healthy ephemeris; satellites
at 33–38 dBHz never), and every such delay traces back to the same event. The
per-chunk tracking log of a recorded live run shows half-cycle carrier slips
clustered exactly where the host received no correlator records for
100–250 ms — GC pauses of ~170 ms, the 1.2 s of live compilation at start-up,
acquisition merges, and the catch-up cascades that follow them. Outside those
events a 44 dBHz satellite shows no slip at all for tens of seconds.

The mechanism: while the host stalls the device holds its last NCO word. Tracking's
FLL-assisted 18 Hz loop on 1 ms records has ~9 Hz rms Doppler jitter at 43 dBHz —
the *same* on the software path and on the hardware path, measured on one
recording — so a 200 ms open loop integrates to one to two carrier cycles. When
records resume, the loop re-acquires phase through slips; each slip inverts the
bit stream, an LNAV subframe fails parity, and the ephemeris waits for the next
30 s frame. Recovery makes it worse: the backlog is folded with landing
predictions that wrap, and the resulting words are scheduled at device samples
already in the past.

The software receiver never opens its loop: a stall only delays processing, and
the loop still closes on every chunk in sample time. To make the hardware path at
least that dependable, loop closure has to be decoupled from the receiver
process's stalls. Julia's collector is stop-the-world for a whole process, so a
thread pool is not isolation; a second **process** is.

## 2. The decision tree

Each node below was put to the user and settled. Rationale is kept where the
alternative was close.

### 2.1 What moves into the loop process (Q1, Q6, Q25, Q26, Q31)

- Everything `ConventionalAssistedPLLAndDLL` / `NCOReferencedPLLAndDLL` did, and
  everything counted in records: record ingest and the epoch grid, coherent
  pre-accumulation, discriminators and loop filters, NCO word scheduling and the
  CSR writes, the navigation-bit accumulator and bit/secondary sync, the absolute
  code-phase bookkeeping (anchors, bit-phase anchor) that pseudoranges rest on,
  the C/N₀ noise reference, and the *execution* of channel arm / release.
- The receiver keeps acquisition, decoding, PVT, the raw-sample clock, and the
  **channel allocation policy**: it says "arm channel k with this configuration",
  the loop process executes. (Alternative rejected: loop-side allocation; policy
  stays where the satellite bookkeeping is.)
- **Multi-band inside one loop process from the start**: band plan, per-band
  banks, timebase scaling and one noise reference per band, as the current link
  has. Every message carries band id and device index.
- **Noise reference is `:channel` only.** The loop process never sees raw
  samples, so `:samples` would need a receiver-published density — exactly the
  stall coupling this removes. One hardware channel per band is the open-loop
  reference.
- **Vector tracking is refused** on this path (`vector_tracking = true` is an
  `ArgumentError` for the remote loop). A later "steer" command can add
  receiver-side aiding without giving the receiver the loop back.

### 2.2 Shared code and packages (Q2, Q3, Q16, Q17, Q24, Q27)

- **Two new packages.**
  - `TrackingLoops.jl` (tentative): discriminators, loop filters, the Doppler
    estimators, the bit buffer, the C/N₀ estimator, the post-correlation filter,
    NCO timelines. Tracking.jl depends on it for the software correlator; the
    loop process depends on it without Tracking.
  - `HardwareLoopCore.jl` (added 2026-09-23, superseding the first draft's
    placement of the core inside `TrackingLoops`): the device-independent loop
    core — epoch fold, coherent accumulation, word scheduling, code-phase and
    bit-phase anchoring, secondary code removal, the command handling, the
    driver API below and the simulated FPGA. It depends on `TrackingLoops` and
    `HardwareLoopProtocol`; nothing Tracking.jl imports knows a device or a
    segment, and the protocol stays Base-only.
  - `HardwareLoopProtocol.jl` (tentative): the shared-memory layout, rings,
    event and command types. No dependencies beyond Base. Both GNSSReceiver
    and the vendor package depend on it.
- **One estimator API for software and hardware:**
  `step(estimator, state, record, words, landing_sample) -> (state, carrier_doppler, code_doppler)`
  on a plain per-satellite state struct. `words` answers `mean_nco_word` over a
  span (a `FixedNCOWord` for the software correlator, the channel's timeline for
  hardware); `landing_sample` is `NO_LANDING_SAMPLE` for software. The same
  function steps the bit buffer, the C/N₀ estimator and the post-correlation
  filter, so the record history is identical on both paths. Tracking's `track!`
  calls it per record; the loop core calls it per folded record.
- **Dependency policy:** keep Unitful and StaticArrays and prove them with the
  trim verifier on the spike; strip a dependency only when the verifier rejects
  it. (Alternative: a plain-Float64 core with units at the rim — fall-back if
  the spike fails.)
- **Driver API** (in `HardwareLoopCore`), concrete and statically dispatched (the
  driver is a type parameter of the core state):
  `read_records!(driver, buffer) -> n`, `write_word!(driver, channel, carrier, code)`,
  `arm!(driver, channel, config)`, `release!(driver, channel)`,
  `sample_count(driver, band)`, `capabilities(driver)`, per-band routing.
  GNSSM2SDR's driver half and the simulated FPGA implement it. The
  `PipeChannel`-based `AbstractHardwareCorrelatorSDR` contract, its `@docs` and
  the "Hardware-correlator contract" page are **retired** once the new path is in.
- **The existing link is rewritten, not trimmed in place.** The core is written
  fresh for zero allocation (fixed per-channel state, no `Dict`, no `sort!`, no
  logging, no `invokelatest`). The in-process `HardwareCorrelatorLink` shrinks to
  a transport that feeds the same core from Julia channels, so the closed-loop
  simulated-device tests, the replay scripts and a software-only CI path run the
  code the binary runs. The existing tests are ported to the core.
- **Stall-tolerance rules are designed in from the start** (Q19), not fixed on the
  current link first: no record cut at an NCO word change (`mean_nco_word`
  already weights a change inside a record; post-sync records then reach the
  symbol boundary and the loop runs on 20 ms integrations); never schedule a
  word at a device sample in the past (the core knows the device counter); clamp
  the landing shift to the nominal delay and treat a backlog older than a few
  epochs as observation-only (C/N₀, bits), stepping the filter with the newest
  records alone; reject non-finite words before they reach the device.

### 2.3 Toolchain (Q4, Q22, Q23, Q29)

- **Julia 1.13 + JuliaC.jl for the loop process.** 1.13 ships JuliaC as an app,
  roots closures in spawned tasks, trims finalizers and `mapreduce`; on 1.12 the
  in-tree script drops task bodies silently and rejects `stderr` logging and
  wide string interpolation. The receiver may stay on 1.12: the two processes
  share memory, not a runtime.
- **Milestone 0 is a trim spike**: a throwaway `--trim=safe` binary on the Orin
  that opens the CSR handle, reads and parses DMA1 records, steps one estimator
  on synthetic records and writes a word, built with the intended dependency set.
  It answers the dependency question and the direct-read question in days.
- **Prebuilt binaries** from GitHub Actions on `ubuntu-24.04-arm` (aarch64, same
  glibc generation as the Orin's Ubuntu 24.04) and x86_64 runners, published as
  platform-tagged tarballs wired through an `Artifacts.toml`; **on-demand
  JuliaC build** into a Scratch.jl directory when no artifact matches the host.
- **CI proofs:** AllocCheck (`@check_allocs`, `ignore_throw = true`) on the
  core's per-record, per-epoch and per-command paths in `TrackingLoops`' test
  suite (fast, every PR, 1.12 and 1.13); the `juliac --trim` build as the vendor
  package's CI job on both architectures, failing on verifier errors — the same
  job that publishes the artifact.

### 2.4 Device and process (Q5, Q11, Q13, Q14, Q15, Q21)

- **Ownership:** the loop process holds its own CSR handle and reads
  `/dev/m2sdr1` directly (`poll` + `read` of 8 KiB buffers, advisory writer lock);
  the receiver keeps its DMA0 recorder pipe untouched. Facts: CSR access is one
  atomic `ioctl` per register with no exclusive open, so two handles are fine;
  the only hazards are logical (one writer per channel). The direct read gives
  up the recorder pipe's 3 s of slack; a single-purpose allocation-free process
  stays inside the driver ring (~190 ms at 80 kHz strobes).
- **Concurrency:** one thread, one service loop:
  `poll(DMA1, timeout = min(time to next due word, 1 ms)) → read → fold closed epochs → commit due words → drain commands → publish events`.
  A code load (one CSR write per chip) is sliced between polls so no step
  exceeds the deadline.
- **Scheduling:** at start-up the process pins itself to one core
  (`sched_setaffinity`) and takes `SCHED_FIFO` at moderate priority
  (`sched_setscheduler`), both via `ccall`; needs `CAP_SYS_NICE` (setcap on the
  binary or a service unit). Recorder and receiver stay on the other cores.
  `isolcpus`/`nohz_full` for that core is an optional second step.
- **Launch and discovery:** `receive(sdr, …)` spawns the binary (path from the
  vendor package's artifact or scratch build) with the CSR map, device index and
  a per-device segment name (`/dev/shm/gnss-loop-<device>`); it attaches, or
  re-attaches if a segment with a live heartbeat already exists.
- **Timebase:** everything crosses the boundary on the **device sample counter**.
  The receiver reads the counter once when it starts its raw stream (its own
  read-only CSR handle, two ioctls) and converts its raw-sample count with that
  constant; the loop process never learns about raw samples.

### 2.5 Protocol (Q7–Q10, Q12, Q18, Q20, Q28)

- **Shared memory** (`mmap` of a file under `/dev/shm`), zero-copy, no
  serialisation. Header: magic, protocol version, layout hash, channel count,
  band table, heartbeat counters for both sides. A mismatching header is refused.
- **Per hardware channel:** one single-producer single-consumer **event ring** of
  fixed-size tagged events in loop order with a monotone sequence number
  (record / bit / epoch-state / status), plus a **seqlock snapshot slot** with the
  newest epoch state for readers that only want "now". Ordering between prompts,
  bits and state changes is preserved by construction.
- **What must be complete history** (checked against Tracking's
  `_apply_correlator_output`): per record — filtered prompt, integration time,
  C/N₀ after the record, block credit (the carrier lock detector averages every
  prompt of a chunk; the code lock detector needs the integration time); per bit
  — the soft bit (the decoder consumes the whole ordered stream). **Newest wins**
  for the epoch state: carrier and code Doppler, absolute code phase at the
  common fold boundary, carrier phase, sync flags and block count (what PVT and
  the dashboard read).
- **Record event payload:** prompt and scalars by default (~48 B); full correlator
  taps for all antennas behind a per-channel flag set by a configure command
  (dashboard, replay recording, a future vector-tracking path).
- **Commands** (receiver → loop) in one SPSC command ring with sequence numbers:
  arm channel k with a `HardwareChannelConfig`, release, configure (epoch length,
  feedback delay, loop bandwidths, event flags), shutdown. Each is **acknowledged
  by a status event** in the channel's ring ("armed at device sample S" /
  "rejected, reason code"); the receiver treats a channel as armed only on the
  ack, keeping today's confirmed-arm rule.
- **Overflow:** the producer never blocks. A full ring **drops the oldest** and
  raises "history lost since sequence N" for that channel; the receiver treats it
  like a lost record today (restart bit sync and decoder, keep tracking). Rings
  are sized for 8 s of events per channel so this needs a receiver stall far
  longer than any GC pause.
- **Synchronisation:** polling only — the receiver reads the rings when it
  processes a chunk, the loop process writes whenever it has something. Head and
  tail are 64-bit counters written with release and read with acquire ordering
  through Julia's atomic pointer intrinsics on the mapped memory; the snapshot
  slot is a seqlock. No futex, no eventfd, zero syscalls on the hot path.
- **Failure semantics:** if the loop process dies or stops heartbeating
  (timeout 500 ms), the receiver marks every hardware satellite lost, respawns
  the process and re-arms from acquisition. If the receiver dies, the loop
  process keeps the device tracking; a new receiver attaches, reads the channel
  status events (PRN, signal, Dopplers, code phase, sync state) and **adopts**
  the tracked satellites instead of re-acquiring.
- **Receiver-side consumer:** a new `correlator_source` (`RemoteHardwareLoop`)
  drains each channel's events once per chunk and **mirrors them into the
  existing Tracking `TrackState`** (filtered prompts, bit buffer soft bits, C/N₀
  state, Dopplers, phases), so lock detectors, decoder, PVT and dashboard are
  unchanged. Needs Tracking to expose setters or a constructor for that state.
  The mirror is receiver-only code and may allocate.

## 3. Milestones

0. **Trim spike (Orin, Julia 1.13).** Throwaway binary: CSR open, direct DMA1
   read and record parse, one `ConventionalAssistedPLLAndDLL` step on synthetic
   records, one word write; built `--trim=safe` with Unitful, StaticArrays and
   GNSSM2SDR's driver files. Deliverable: the list of dependencies that verify,
   the read-path latency, and the fall-back decision for Q3.
1. **`TrackingLoops.jl`.** Extract estimators, bit buffer, C/N₀ estimator and
   filters from Tracking.jl behind the per-record `step` API; retarget Tracking
   and GNSSReceiver. Acceptance: the software receiver on the ION recording is
   bit-identical to before.
2. **`HardwareLoopProtocol.jl`.** Segment layout, rings, seqlock slot, event and
   command types, header versioning; tests with two threads standing in for two
   processes, then two processes.
3. **Loop core in `HardwareLoopCore`** (built inside `TrackingLoops` first, moved
   out on 2026-09-23). Epoch fold, accumulation, timelines and the
   stall-tolerance rules, the driver API, the simulated FPGA as first driver;
   port the closed-loop and record-accounting tests; AllocCheck on the hot paths
   in the test suite.
4. **GNSSM2SDR split and executable.** Trimmable driver half implementing the
   driver API (CSR, bank, record parsing, direct DMA1 reader), the loop
   executable (`@main`, service loop, pinning and `SCHED_FIFO`), the CI job that
   trims on `ubuntu-24.04-arm` and x86_64 and publishes artifacts, the
   `Artifacts.toml` and the scratch-space fallback build. Retire the in-process
   adapter.
5. **Receiver integration.** `RemoteHardwareLoop` consumer and `TrackState`
   mirror, spawn/attach/adopt, restart on lost heartbeat, `receive(sdr, …)` on
   the new path, refusal of vector tracking; replay tools on the new core.
6. **Board validation.** 300 s live runs with GC logging in the loop process,
   record-to-word latency histogram, the slip analysis, time to fix.

## 4. Acceptance criteria

1. AllocCheck proves the loop core's per-record, per-epoch and per-command paths
   allocation-free; the trim build is green on both architectures.
2. A 300 s live run on the Orin shows zero GC events in the loop process and a
   record-to-word latency under 3 ms at the 99th percentile.
3. Satellites above 40 dBHz show no half-cycle slips outside signal fades, and a
   GPS fix arrives within one LNAV frame of the fourth ephemeris.
4. The closed-loop simulated-device tests and the replay tools pass against the
   new core.
5. The software receiver through Tracking on the ION recording is bit-identical
   to before the estimator extraction.

## 5. Defaults written in (veto by editing this file)

- Segment header: magic, protocol version, layout hash; mismatch refused.
- Ring capacity: 8 s of events per channel at the channel's record rate.
- Heartbeat every epoch from the loop process, every chunk from the receiver;
  500 ms timeout either way.
- Package names `TrackingLoops.jl` and `HardwareLoopProtocol.jl`; segment name
  `/dev/shm/gnss-loop-<device>`.

## 6. Risks

- Trim verification of Unitful/StaticArrays and of GNSSM2SDR's driver half is
  unproven (no public evidence either way); Milestone 0 exists for this.
- The estimator extraction touches Tracking.jl's public surface; the
  bit-identical ION test is the guard.
- Direct DMA1 reads give up the recorder pipe's slack; a long code load must be
  sliced, and the ring's ~190 ms is the whole budget.
- `CAP_SYS_NICE` and `/dev/shm` permissions are deployment concerns on every new
  board.
- Adoption on reattach needs the loop process's status events to carry a full
  seed; an incomplete seed silently degrades to re-acquisition.
