# Hardware-correlator analysis scripts

Tooling for the LiteX-M2SDR hardware-correlator path (issue #107): record a live
run, replay the recording through the software receiver and through the
simulated device, and compare the tracking loops record by record. This is how
the feedback-delay instability behind
`docs/plans/2026-09-14-delay-aware-hardware-loop.md` was found and how the
delay-aware loop was verified.

| Script | What |
|---|---|
| `record_and_log.jl` | live M2SDR run; tees DMA0 to a raw file, one CSV row per chunk per satellite |
| `replay_software.jl` | recording → software receiver (no feedback delay): the reference |
| `replay_simfpga.jl` | recording → simulated device (`test/simulated_fpga.jl`) with a chosen `feedback_delay_epochs` and estimator |
| `tracking_log.jl` | the shared `extract` snapshot, CSV layout and `MODE` → estimator mapping |

`MODE` is `std` (Tracking's conventional FLL-assisted loop), `nco`
(`NCOReferencedPLLAndDLL`, the hardware receiver's default) or `nconp` (its
negative control: re-based on the applied word, no landing prediction).

The raw layout is the M2SDR's 2R2T sc16 stream: four `Int16` per sample
(`I₁ Q₁ I₂ Q₂`), antenna 1 in the first two words, 32 MB/s at 4 MS/s.

Environment: GNSSReceiver, GNSSSignals, Tracking, Unitful, Printf; the live
script additionally GNSSM2SDR and the gateware's CSR map at
`~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch20_ant1/csr.csv`.

Two practicalities for instrumented runs (details in `record_and_log.jl`):
Julia block-buffers stderr to a file, so pipe through `cat` or `tee` for a live
log; and a non-default `extract` recompiles the pipeline inside the first live
chunk, so the live script takes the first payload before enabling the device.

Only one receiver process at a time can own the device; `pkill -x m2sdr_record`
if a previous run died.
