# Loop process — Milestones 1 to 5: what was built and what was measured

Status notes for `docs/plans/2026-09-22-loop-process.md`, written 2026-09-22 as
the work landed. Milestone 0 has its own note (`milestone-0-trim-spike.md`);
Milestone 6 (the 300 s board run) is in `milestone-6-board-validation.md`.

## Where the code lives

| Package | Location (development checkouts) | What |
|---|---|---|
| `TrackingLoops.jl` | `../TrackingLoops.jl` | the per-record loop arithmetic extracted from Tracking.jl (what Tracking.jl imports) |
| `HardwareLoopCore.jl` | `../HardwareLoopCore.jl` | the loop process's engine: driver API, `LoopCore`, ingest/fold/commands/service, `SimulatedDevice` (depends on TrackingLoops and HardwareLoopProtocol) |
| `HardwareLoopProtocol.jl` | `../HardwareLoopProtocol.jl` | the shared-memory segment: header, rings, seqlock slot, event and command types |
| `Tracking.jl` (branch `trackingloops`) | `../Tracking.jl` | imports and re-exports TrackingLoops; `track!` unchanged |
| `GNSSM2SDR.jl/M2SDRLoop/` | `../GNSSM2SDR.jl/M2SDRLoop` | the trimmable device half: CSR/DMA/record/bank layer, `M2SDRDriver`, `gnss_loop` entry point, `build.sh` |
| `GNSSM2SDR.jl` | `../GNSSM2SDR.jl` | imports the device layer from `M2SDRLoop`; adds `M2SDRRemote` and `remote_loop` (receiver side) |
| GNSSReceiver.jl (this branch) | `src/remote_hardware_loop.jl` | `RemoteHardwareLoop`, the correlator source over the segment; `receive(loop, …)` |
| `GNSSSignals.jl` (branch `trim-safe-code-tables`) | `../GNSSSignals.jl` | `read_in_codes` without the closure `open` and specialised on the element type, so a trimmed binary can construct signals |

Every `[sources]` entry points at these local paths; publishing the two new
packages and the branches is the step after this one.

## Milestone 1 — the extraction

Moved from Tracking.jl into TrackingLoops.jl, verbatim where possible:
correlators, discriminators, loop filters and bandwidth defaults, the bit
buffer with its CFAR sync detectors, the C/N₀ estimators, the noise-estimator
window, the sample-count parameters, and the estimators
(`ConventionalPLLAndDLL`, `ConventionalAssistedPLLAndDLL`,
`NCOReferencedPLLAndDLL` with `NCOTimeline`, moved here from GNSSReceiver).
The per-record API is `apply_record` (bit buffer, C/N₀, prompt filter) and
`step_loop` (the estimator step; renamed from the plan's `step`, which clashes
with `Base.step`). Two behavioural details changed on purpose: the bit-boundary
overshoot no longer logs inside the fold (it is returned as a flag and Tracking
logs it), and `NCOTimeline` is a fixed-capacity ring rather than a growing
vector, so scheduling a word never allocates.

**Acceptance (bit-identity).** Tracking-only A/B runs of `track!` — the
registry Tracking 8.3.1 against the TrackingLoops-backed one — are
bit-identical on a 6 s single-satellite ComplexF64 stream at 4 MS/s (hash
`34a3e21df3938b31` both) and on a 4 s three-satellite ComplexF32 stream at
2.048 MS/s in the ION recording's regime (hash `6565129fc269b80c` both). The
receiver-level fingerprint on the ION RTL-SDR recording is **not** a usable
oracle: two runs of the *unchanged* baseline at `-t 1` differ from each other
(`b121c8ee0d240e15`, `89862dd22145834e`), and the changed stack produced hashes
from the same set, so the receiver is nondeterministic at that level
(acquisition timing) independently of this work. Tracking.jl's own suite passes
on the retargeted branch (380 test sets), and so does GNSSReceiver's.

## Milestone 2 — the protocol

`HardwareLoopProtocol.jl`: a 4 KiB header (magic, version, FNV-1a layout hash
over every record's field offsets, band table, PIDs, heartbeats, loop state),
one command ring (receiver → loop) and one event ring plus one seqlock snapshot
slot per channel (loop → receiver). Rings are single-producer single-consumer
with 64-bit acquire/release counters and per-slot sequence words, so a torn
slot is detected rather than misread; the producer overwrites the oldest entry
(`publish!`) or refuses (`try_publish!`). Slots are 192 B (events) and 256 B
(commands). Names are 24-byte fixed strings. Everything goes through the atomic
pointer intrinsics on mmapped memory; a heap-backed segment serves the
in-process tests. Tests: single thread, two threads (no torn slot ever
observed), two processes over a file-backed segment, attach-or-replace of a
dead loop.

## Milestone 3 — the loop core

`HardwareLoopCore.jl` (written as `TrackingLoops/src/core/` first, moved into
its own package on 2026-09-23 so that Tracking.jl's dependency does not know
devices or segments and the protocol stays Base-only): the driver API (`driver.jl`), the state
(`state.jl`: per-signal `ChannelBank`s dispatched over a tuple so no record is
boxed, a flat `ChannelTable`, per-band noise references), ingest (`ingest.jl`:
the link's block-grid accounting, record continuity, overlay wipe and coherent
accumulation), the fold (`fold.jl`: emit, code-phase anchoring, epoch state,
word scheduling), commands (`commands.jl`) and the service pass
(`service.jl`). The simulated FPGA moved here as `SimulatedDevice`, with a
record-delay knob.

One design change against the plan's defaults: words are **committed in the
pass that computes them**, not deferred `feedback_delay_epochs` ahead. The loop
process has no host-side chunk latency to hide, the deferral would have let
every word be superseded before it fell due (as it did on the first closed-loop
run), and the timeline records where each word really landed
(`reschedule_word!`) so the delay-aware estimator attributes records correctly
regardless. `LoopConfig.commit_lead_samples` (default 0) replaces
`feedback_delay_epochs`; the protocol's `ConfigureCommand` field was renamed
accordingly.

A second change of default, from the board runs (Milestone 6): once bit sync
is found the core steps the loops once per primary code period
(`coherent_code_blocks = 1`) rather than once per whole symbol. Stepping an
18 Hz PLL every 20 ms puts its bandwidth–time product at 0.36 and the loop
drifts off within seconds — reproduced with the simulated device; the bit
buffer still accumulates the symbol for the decoder.

**Tests** (`HardwareLoopCore/test/core.jl`): arming over the command ring answers
`STATUS_ARMED` at the device sample the channel started; unsupported signals
are rejected; the loop closes through the simulated device at 0, 2 and 4
epochs of record delay (pull-in from 20 Hz and a quarter chip, C/N₀ within
44–50 dBHz of the 48 dBHz truth, bits on the 80 000-sample grid alternating);
release/query/configure/shutdown acknowledged; the noise reference hops decoys.
**Allocation:** a warm `service_pass!` allocates 0 bytes (`@allocated` over a
second of chunks); AllocCheck's static report on `service_pass!` lists only the
inherent `push!` growth paths of pre-sized vectors, `time_ns`, and nothing
else. The one-off allocations found on the way (the sync detector's accumulator
seeding, the first soft bit, `FixedName()`'s dynamic `ntuple`) were removed.

## Milestone 4 — the M2SDR driver and the executable

The device layer of GNSSM2SDR (`csr.jl`, `dma.jl`, `record.jl`,
`subcarrier.jl`, `bank.jl`) moved into the sub-package `M2SDRLoop`, which
GNSSM2SDR imports; `M2SDRDriver` implements the driver API over it (direct
DMA1 reads woken by `poll(2)`, immediate `carrier_freq`/`code_freq` writes
with unchanged words skipped, code load plus a sample-exact scheduled handover
verified on the following passes and re-scheduled up to three times, a failed
handover reported as `REJECT_DEVICE_ERROR`). `loop_main` parses the command
line, opens the board, creates the segment, pins and (optionally) sets
`SCHED_FIFO`, and runs the service loop with a periodic status report.

`LiteXCSR(csv; device = nothing)` is a device-less shadow register file, so the
driver is tested against the board's recorded `csr.csv` without a board:
records come off the wire latest tap first (`[L, P, E]`, `[VL, L, P, E, VE]`),
duplicates are dropped, an arm writes the expected PRN, code length, tap
offsets, code phase and apply-at words, confirmation follows the target plus
half the margin, a never-committing handover fails after three attempts, and
the loop core arms and confirms through the driver with no allocation on a warm
pass.

**Trim build.** `build.sh` builds `gnss_loop` with `juliac --trim=safe` in
7 s on x86_64 (4.7 MB) and 22 s on the Orin (4.5 MB), zero verifier errors.
The blockers found and fixed on the way: runtime `Type` arguments
(`Array{type}(undef, …)`), splatting (`_fnv1a(x...)`, `Int[v...]`), a
constructor whose return type depended on a runtime integer (`NumAnts(n)`), a
union of tuple types flowing into the core's constructor, an unspecialised
`Function` argument (`apply(post_corr_filter, …)`), and error messages that
print a vector.

**First live run** (Orin, 10 s, DMA0 draining, no channel armed): 1 000
epochs/s folded, ~6 700 records/s read (the six idle channels dump at 1 kHz
each plus the strobes), 0 duplicates, 0 bytes allocated, 0 GC pauses, longest
service pass 1.6 ms.

Not done from the plan's list for this milestone: the CI job on
`ubuntu-24.04-arm`, `Artifacts.toml` and the scratch-space fallback build (the
binary is built on the target with `build.sh` and found at
`M2SDRLoop/build/gnss_loop`). The in-process adapter (`HardwareCorrelatorLink`,
`M2SDRCorrelator`) was retired on 2026-09-23, after the board validation of
Milestone 6 had used it as the A/B reference (see below).

## Milestone 5 — the receiver

`RemoteHardwareLoop` (`src/remote_hardware_loop.jl`) is a correlator source:
`process` dispatches `advance_tracking!` on it as on the link. Per chunk it
heartbeats, checks the loop's heartbeat, releases the channels of dropped
satellites, arms one noise reference per band and a channel per newly tracked
signal component (`ArmCommand`s built from the tracked signal's correlator taps
and the acquisition's Dopplers and code phase, with `valid_at_sample` moved
onto the device counter through `device_sample_origin`), then drains every
channel's event ring and mirrors it into the `TrackState`: record events set
the ranging prompt, the record length and the C/N₀ (written into the signal's
`NoiseRefCN0Estimator` ring), bit events push soft bits, epoch states set the
satellite's Dopplers, carrier phase and sync flags and extrapolate its code
phase to the chunk end on the common reception instant. `is_observation_gap`,
`has_current_observations` and `take_bit_clock_restart!` answer from the
per-chunk bookkeeping. A stale loop heartbeat restarts the process from its
spawn command and re-arms everything from the receiver's state; attaching to a
loop that is already running releases every channel first. Vector tracking is
refused. `receive(loop, systems, fs; …)` is the entry point; GNSSM2SDR's
`M2SDRRemote`/`remote_loop` provide the device side (raw stream, capabilities,
counter origin, the `gnss_loop` command).

**Tests** (`test/remote_hardware_loop.jl`): the whole receiver over a loop core
that runs in-process on a heap-backed segment, fed the same synthetic signal as
the device — acquisition, arm, lock, C/N₀ 40–52 dBHz, the device pulled onto the
true Doppler and off the quarter-chip handover error, at 0 and 2 epochs of
record delay; refusal of vector tracking, of an unknown band and of a mismatched
sample rate; a stub loop process spawned from a command, attached to by a
second receiver (which releases the bank), killed and restarted on the lost
heartbeat.

Not done: adoption of a running loop's satellites on attach (the seed events
exist — `STATUS_CHANNEL_STATE` — but the receiver releases the bank instead of
seeding tracked satellites and decoder states from them), and the replay tools
on the new core.

**Retired with this work (2026-09-23):** the in-process `HardwareCorrelatorLink`
and everything only it used — `CorrelatorDump`, `NCOUpdate`, the epoch strobes,
`HardwareChannelConfig`, the `PipeChannel` dump/NCO contract
(`correlator_dump_channel`, `nco_update_channel`, `assign_channel!`,
`release_channel!`, `assignment_start_sample`, `dropped_dump_count!`),
`receive(::AbstractHardwareCorrelatorSDR, …)`, its precompile workload, the
simulated FPGA of the test suite and the tests built on it (link ingest,
multi-band routing, partial primary records, secondary-code removal), and the
in-process examples. What a vendor package describes to the receiver is now the
*device* only (`src/hardware_device.jl`: raw streams, capabilities, band plan,
replica amplitudes, counter origin); the loop closure is the loop process's.
Two pieces of evidence went with the deleted tests and are follow-ups on the
loop core: the end-to-end GPS L5I overlay removal (issue #132; the GPSL5I
`secondary_sync` cell of the support matrix is `untested` again, with the
reason recorded) and the GPS L2CL partial-primary record accounting.
