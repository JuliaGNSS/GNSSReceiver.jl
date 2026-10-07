# Loop process — Milestone 6: the first board runs (2026-09-22)

The receiver over `gnss_loop` on orin2 (LiteX-M2SDR, the six-channel five-tap
gateware `gnss_m2sdr_m2_x1_ch6_ant1_code4092_tap5_sub12_synthPerfSpread`,
Julia 1.13, the trimmed loop executable from `M2SDRLoop/build.sh`), driven by
`GNSSM2SDR.jl/examples/loop_process.jl` at 4 MS/s, GPS L1 C/A, with the
acquisition settings of `closed_loop_multi.jl`. Raw logs are on the board under
`~/loop/`: `run300_a.*` (300 s, whole-symbol accumulation), `runB.*` and
`runC.*` (traced diagnosis runs), `link_ab*.out` (the in-process link on the
same sky), `runF.*` (60 s) and `runG.*` (300 s) with the one-block default.
`*.epochs.csv` holds every epoch state and record the receiver mirrored.

## What the loop process did (acceptance criteria 1 and 2)

300 s run (`run300_a.loop.log`, one status block every 10 s):

| quantity | value |
|---|---|
| epochs folded | 300 881 (1 kHz, the whole run) |
| records read off DMA1 | 13.8 million (6 channels × 1 kHz + 40 kHz strobes), 0 duplicates, 0 dropped |
| words written | 61 782, 0 late |
| GC pauses in the loop process | **0** |
| bytes allocated in the loop process | 50 KB per 10 s while satellites were being armed, from the arm path (code tables, replica shapes); 0 between arms |
| longest service pass | 3.7 ms |
| record age at the fold (= record-to-word latency, since the word is written in the same pass) | 111 066 passes in [1, 2) ms, 104 664 in [2, 3) ms, 4 in [3, 4) ms, none above; maximum 2.6 ms |

So the loop process is allocation-free and GC-free in steady state, and the
record-to-word latency is under 3 ms at the 99.99th percentile — with the
epoch-strobe period at 100 samples (40 kHz). At the plan's 1 kHz strobe the
litepcie driver completed an 8 KiB buffer only every ~9 ms and the latency sat
at 8–12 ms; `--strobe` therefore defaults to `fs / 40000` and is independent
of the fold epoch.

## What the receiver saw with whole-symbol accumulation (`run300_a`, `runB`, `runC`)

The receiver acquired, armed and mirrored satellites throughout: 21 arms in
300 s, none rejected, no events lost, no commands refused, no restarts. PRN 19
was tracked continuously for the whole run at 42–48 dBHz. The other satellites
(15, 20, 23, 24; 40–48 dBHz at handover) were held for 10–60 s, then their C/N₀
decayed and the receiver dropped and re-acquired them. No PVT fix.

The traced runs (`runB.epochs.csv`, `runC.epochs.csv`: every epoch state the
receiver mirrored) show the mechanism: the carrier Doppler the loop commands
ramps linearly (−12 to +28 Hz/s) from the moment **bit sync is found**, with
the C/N₀ decaying alongside. It is not bound to a channel (run C forced the
first satellite onto hardware channel 4 and PRN 19 decayed on channels 3 and 5
but held on channel 6; every channel's registers read back correctly), and the
in-process link (`closed_loop_multi.jl`, same board, minutes later) showed the
same shape on PRN 20 and no fix either.

The simulated device reproduces it. With the plan's default — one whole
navigation bit accumulated coherently once sync is found — a 48 dBHz satellite
in the closed-loop simulation runs 30 Hz off in six seconds after bit sync, at
0, 2 or 4 epochs of record delay, with or without landing prediction. Stepping
the loops once per 20 ms symbol puts an 18 Hz PLL's bandwidth–time product at
0.36, where the loop drifts or limit-cycles; the loops are tuned for one step
per primary code period. `LoopConfig.coherent_code_blocks` therefore defaults
to 1 (the bit buffer still accumulates the symbol), and the link — which
accumulated the whole symbol too — very likely lost its satellites the same
way. With one-block records the simulated loop holds C/N₀ and code lock for
the full eight seconds; its Doppler estimate then shows the ±10 Hz post-sync
wander that Tracking's own software `track!` shows on the same synthetic
signal (data-bit flips through the FLL discriminator), which is Tracking's
pre-existing behaviour, not the core's.

## The 300 s run with the one-block default (`runG.*`)

| quantity | value |
|---|---|
| first position fix | **43.1 s** after the raw stream started; 10 381 of 11 936 receiver outputs afterwards carried a fix |
| satellites | PRN 20 held the **whole 300 s** at 45 dBHz (295 809 records, one every epoch, no gap); PRN 23 held 296 s at 42 dBHz; PRN 12 116 s then 143 s; PRN 13 140 s; PRN 15 110 s then 34 s; PRN 10 101 s at 33 dBHz |
| arms / rejections / restarts | 19 arms, 0 rejected, 0 restarts, 0 events lost, 0 commands refused |
| loop process | 301 700 epochs, 13.9 M records, 846 729 words (0 late), **0 GC pauses**, 100 KB allocated per 10 s only while arming, longest pass 4.0 ms |
| record-to-word latency | 110 908 passes in [1, 2) ms, 105 377 in [2, 3) ms, 2 in [3, 4) ms, none above (max 2.85 ms) |

The fix is a real one (an ECEF position 6 371 km from the Earth's centre in
the board's neighbourhood); on some outputs the solution jumped to an
implausible point while a marginal satellite was in the mix, which is PVT's
satellite selection and not the loop. The one-block default holds the strong
satellites for the whole run where the whole-symbol accumulation lost them
within a minute (`runA`: PRN 20 held 10–60 s at a time; `runG`: 300 s).

Run `runD` (the first with the one-block default) is an unexplained outlier:
every satellite's C/N₀ read 10–25 dBHz from its first record and none held. A
control immediately afterwards with the whole-symbol build (`runE`) and the
one-block build again (`runF`) both read normal C/N₀s, so the build was not the
cause; the raw stream or front end of that run is the suspect, and it did not
recur.

**Acceptance criterion 3.** A fix arrived 43 s after start; the strong
satellites showed no lock loss for 300 s. A half-cycle-slip count above
40 dBHz has not been extracted from the traces yet (the per-record prompts are
in `runG.epochs.csv`).

## Fixed on the way

- The noise reference's pooled power was not brought onto the satellites'
  amplitude scale (the gateware's ±127 carrier ROM), so every C/N₀ read −∞ and
  every satellite was dropped at once. `_pool_noise_record!` now divides by the
  channel's scale squared; the simulated device carries the declared replica
  gain so the core test covers it.
- The strobe period is decoupled from the fold epoch (see above).
- Closing the receiver's handle while a chunk was in flight unmapped the segment
  under the processing task; `advance_tracking!` now checks the handle.

## Why only the strongest satellite held for the whole run (`runG`, `runI`)

Asked after `runG`: PRN 20 held 300 s, PRN 23 296 s, the others were dropped
and re-acquired every 50–150 s. The traces answer it:

- The receiver drops a satellite when its C/N₀ falls below the code-lock
  detector's 30 dBHz threshold. In every drop the last outputs read
  28–31 dBHz with `in_lock` still true; the C/N₀ had decayed there from
  38–42 dBHz over the preceding 50–100 s.
- The noise density the loop measures is constant (5.0–5.3·10⁻³ in the
  receiver's units, every 10 s bucket, every satellite): the decays are in the
  prompt power itself, and they are **not** synchronous across satellites —
  PRN 20 rose from 45 to 49 dBHz while PRN 12 fell from 40 to 31 in the same
  minute, and PRN 20 itself swung 49 → 38 → 45 → 39 dBHz over the run.
- The code loop is not walking off the peak: its correction beyond the
  carrier aiding stays within ±0.01 Hz, and with the taps published (`runI`,
  200 s, `T,` rows in `runI.epochs.csv`) the early/late balance
  `(E−L)/(E+L)` stays within ±0.004 through every fade, while the
  prompt-to-early ratio is the correlation triangle's (1.88 for the 48 dBHz
  satellite, i.e. taps at ±0.5 chip; the lower ratios of weak satellites are
  the noise floor in E and L, and their signal parts still give ~1.9).
- The same fades appear on the in-process link on the same sky
  (`link_ab.out`: PRN 20 46 → 31 dBHz over 40 s).

So the loop tracks through the fades — the replica stays centred and every
epoch has its record — and the receiver's threshold does the dropping. The
fades are in the received power (multipath on a poorly sited antenna is the
obvious candidate: 10 dB swings with 100–200 s periods, uncorrelated across
satellites), and the "strongest satellite holds" pattern is just the one
satellite whose fades never reach 30 dBHz. Whether the antenna site or the
threshold should change is a receiver question, not a loop-process one.

Two runs (`runD`, `runH`) had an empty raw stream from the start (samples at
the noise floor of the ADC, acquisition locking onto noise peaks, every tap at
~9 000 raw units where a signal reads 20 000–75 000); `loop_process.jl` now
probes DMA0 before it starts and aborts if the samples carry no power. Its
trigger — back-to-back `m2sdr_record` restarts — has not been pinned down.

## The same samples through the software receiver (2026-09-23, `runJ`)

To rule the loop process out entirely, `loop_process.jl` can now tee the raw
2R2T stream to a file (`LOOP_RECORD=path`, 32 MB/s) while it tracks, and
`examples/replay_software.jl` replays that file through the software receiver
(every correlation on the CPU, `ConventionalAssistedPLLAndDLL` or
`NCOReferencedPLLAndDLL`, one loop step per code period). A 150 s run was
recorded to `/dev/shm/raw_2r2t.sc16` (5.6 GB, tmpfs) and replayed with both
estimators; the recording has one 121 ms all-zero stretch at 1.54 s (a recorder
dropout at start-up) that the replay dithers, because a zero-energy chunk turns
the software loop's normalised discriminators into NaN and kills the run.

Per-satellite C/N₀ in 10 s buckets, loop process live vs. software replay
(`compare_runs.py`): within 0.5 dB in every bucket. PRN 7 fades from 42.8 to
31 dBHz over the first 50 s and is dropped by both receivers (the loop at
~48 s, the software at ~55 s), both re-acquire it at ~85 s and hold it at
39–42 dBHz to the end; PRN 6 swings 39 → 43 → 40 dBHz identically in both;
PRN 11 is dropped by both at ~55 s at 31–32 dBHz; PRN 21 sits at 32–36 dBHz in
both. The two software estimators are identical to each other, as the tests
say they are at zero delay. The software receiver ran at 0.3× real time on the
Orin (5 threads), which is the other half of the case for the loop process.

So the fades are in the received signal, the loop process reproduces the
software receiver's C/N₀ to within its estimator noise, and the satellites are
lost to the receiver's 30 dBHz threshold in both.

## Record-to-word latency against the strobe rate (2026-09-23)

The latency is set by the litepcie DMA buffer: the driver completes whole
8 KiB buffers of 64 records, so a dump waits for the buffer it fell into to
fill, and the epoch strobes are what fill it. The loop process alone, 20 s each,
raw stream draining, record age at the fold (record end → pass start):

| strobe period | strobes | record age | max | loop CPU (one core = 100 %) |
|---|---|---|---|---|
| 100 samples | 40 kHz | 51 % in [1, 2) ms, 49 % in [2, 3) ms | 2.8 ms | 7.0 % |
| 50 samples | 80 kHz | 25 % in [0.5, 1) ms, 75 % in [1, 2) ms | 1.8 ms | (9 %) |
| 25 samples | 160 kHz | 66 % in [0.25, 0.5) ms, 34 % in [0.5, 1) ms | 1.0 ms | 10.7 % |

The default strobe period is now `fs / 160000` (25 samples at 4 MS/s, 21 MB/s
of strobe records on DMA1). Shrinking the DMA buffer itself would need a
rebuilt litepcie module (the size is a compile-time constant and also chunks
the raw DMA0 stream); the strobe rate reaches the same point without it.

## Scheduler jitter: pinning versus `SCHED_FIFO` (2026-09-23)

The loop process alone at 160 kHz strobes, 20 s each, with twelve `yes`
busy-loops saturating every core of the Orin. Record age = record end → start
of the pass that folds it; "max pass" includes the 1 ms `poll` wait.

| configuration | record age | max age | max pass |
|---|---|---|---|
| idle board, no flags (reference) | 66 % < 0.5 ms, 34 % < 1 ms | 1.0 ms | 1.2 ms |
| loaded, no flags | 53 % < 0.5 ms, 47 % < 1 ms, **7 passes 1–7 ms** | **6.9 ms** | **10.7 ms** |
| loaded, `--core=11` only | 50 % < 0.5 ms, 50 % < 1 ms | 0.9 ms | **11.2 ms** (preempted mid-pass by the busy loop sharing its core) |
| loaded, `--fifo=50` only | 50 % < 0.5 ms, 50 % < 1 ms | 0.8 ms | 1.05 ms |
| loaded, both | 50 % < 0.5 ms, 50 % < 1 ms | 0.75 ms | 1.07 ms |

`SCHED_FIFO` alone removes the tail entirely — under full load the loop behaves
as on an idle board. Pinning alone does not: a pinned CFS task still shares its
core's time slices with whatever else runs there, and a pass can be preempted
for a whole slice. Pinning adds nothing measurable on top of FIFO here; it
would only matter for cache or interrupt affinity.

`SCHED_FIFO` needs `CAP_SYS_NICE`. The clean way is a file capability on the
binary — `sudo setcap cap_sys_nice+ep build/gnss_loop` — after which the
receiver spawns it unprivileged with `fifo = 50`; verified working. Under
`sudo` instead, `HOME`/`JULIA_DEPOT_PATH` must point at the user's depot: the
trimmed binary still loads `libopenspecfun` (SpecialFunctions, via
TrackingLoops' `erfinv`) from the depot's artifacts at start-up, so it is not
yet self-contained (JuliaC's bundling, or dropping that one dependency, would
fix it). The loop always sleeps in `poll` between passes, so a FIFO priority
cannot starve the core.

**Kernel-side latency.** The board runs `6.8.12-tegra` with `CONFIG_PREEMPT=y`
(the fully preemptible non-RT model). With `SCHED_FIFO` and twelve busy loops,
plus a `dd oflag=direct` write storm on the eMMC, a `find /` walk and a
ping flood against the board's own interface (block, VFS and network
interrupts and softirqs), 25 s: 12 483 passes under 0.5 ms, 23 in [0.5, 1) ms,
max age 0.81 ms, longest pass 1.16 ms — the same as the idle board. Whatever
non-preemptible kernel latency remains is below the DMA buffer granularity;
a PREEMPT_RT kernel has nothing measurable left to bound on this workload.
