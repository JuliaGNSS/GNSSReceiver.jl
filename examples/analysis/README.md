# Hardware-correlator analysis scripts

Tooling around the LiteX-M2SDR hardware-correlator path (issue #107): replay a
raw recording through the software receiver and compare its tracking loops,
record by record, with what the loop process did on the same samples. This is
how the received-power fades behind the satellite drops of the loop-process
validation (`docs/plans/2026-09-22-loop-process/milestone-6-board-validation.md`)
were told apart from loop behaviour.

| Script | What |
|---|---|
| `replay_software.jl` | recording → software receiver (no feedback delay): the reference |
| `tracking_log.jl` | the shared `extract` snapshot, CSV layout and `MODE` → estimator mapping |
| `hardware_live_m2sdr.md` | the record of the 2026-09-18 live-sky run on the in-process adapter (evidence `live_m2sdr_l1_20260918`) |

The live run itself is GNSSM2SDR's `examples/loop_process.jl`: it starts the
`gnss_loop` process, runs `receive` over `RemoteHardwareLoop`, and can tee the raw
2R2T stream to a file (`LOOP_RECORD`) for `replay_software.jl`. The in-process
adapter's live and simulated-device replays (`record_and_log.jl`,
`replay_simfpga.jl`, `hardware_live_m2sdr.jl`) were retired with it and are in
git history before the loop process landed.

`MODE` is `std` (Tracking's conventional FLL-assisted loop), `nco`
(`NCOReferencedPLLAndDLL`, the loop process's default) or `nconp` (its negative
control: re-based on the applied word, no landing prediction). Through the
software receiver all three run with the chunk's own replica Doppler and no
delay, so `nco` and `std` differ only in bookkeeping there.

The raw layout is the M2SDR's 2R2T sc16 stream: four `Int16` per sample
(`I₁ Q₁ I₂ Q₂`), antenna 1 in the first two words, 32 MB/s at 4 MS/s.

Environment: GNSSReceiver, GNSSSignals, Tracking, Unitful, Printf.

Julia block-buffers stderr to a file, so pipe through `cat` or `tee` for a live
log.
