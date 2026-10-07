# Milestone 0 — the trim spike on the Orin (Julia 1.13)

**Date:** 2026-09-22
**Board:** `orin@orin2`, aarch64 Cortex-A78AE, kernel 6.8.12-tegra, LiteX-M2SDR with
the six-channel five-tap gateware (`gnss_m2sdr_m2_x1_ch6_ant1_code4092_tap5_sub12_synthPerfSpread`,
CSR layout 3, record format 2).
**Toolchain:** Julia 1.13.0 (juliaup channel `1.13`), JuliaC app 0.3.10 for 1.13.
**Sources:** `examples/analysis/loop_process_spike/` — `./run.sh all` builds every rung
`--trim=safe`; `CSR=… ./run.sh device 5` runs the device rung against the board.

## What was built

| rung | what it exercises | `--trim=safe` | verifier errors |
|---|---|---|---|
| `rung_device` | GNSSM2SDR's `csr.jl`, `dma.jl`, `record.jl` verbatim: open the CSR handle, read `gnss_version`/`gnss_capabilities`, enable the bank and the 80 kHz strobe, take the DMA1 writer lock, `poll` + `read` 8 KiB buffers directly from `/dev/m2sdr1`, parse records, write carrier/code words to every channel | **built** (2.0 MB) | **0** |
| `rung_fileread` | reading a file three ways: `open(path, "r")` + `read!`, `read(path)`, raw `ccall(:open/:read)` | **built** (1.9 MB) | **0** |
| `rung_tracking_fold` | Tracking 8.3.1's per-record fold — `TrackedSat`, `_update_tracked_sat_doppler` (discriminators, FLL-assisted third-order loop filter, `NoiseRefCN0Estimator`, bit buffer with the CFAR bit-edge detector) over 2000 synthetic records, with the whole Tracking dependency graph (Unitful, StaticArrays, TrackingLoopFilters, SpecialFunctions, BitIntegers, GNSSSignals, Polyester, SIMD, SinCosLUT, FastSinCos, Dictionaries, Accessors) | failed | **4, of two kinds** |

The two blockers in the estimator rung are both outside the arithmetic:

1. `string(::String, ::String, ::Vararg{String})` in
   `Tracking.validate_preferred_num_code_blocks_to_integrate` — an error message built
   by concatenating interpolated strings in the `TrackedSignal` constructor. Under
   `--trim=safe` the varargs splat is unresolved. Rule for `TrackingLoops`: no dynamic
   string building on any path the loop process reaches; error messages are literals or
   `LazyString`s.
2. `open(f::Function, filename)` in `GNSSSignals.read_in_codes` — the code tables are
   read with the closure form of `open`, whose keyword-argument plumbing is unresolved.
   `rung_fileread` shows that `open(path, "r")` + `read!` and `read(path)` both trim, so
   the fix is a one-line change in GNSSSignals (or the loop executable reads the tables
   itself and constructs the signal from the matrix).

Everything else Tracking's fold touches verifies: **Unitful and StaticArrays stay** (the
Q3 fall-back is not needed), and so do TrackingLoopFilters, SpecialFunctions' `erfinv`,
BitIntegers' `UInt1800`, the `SVector`-based correlators and the noise window.
Multi-argument `Core.println` is *not* resolvable under trim (it goes through
`Core._apply_iterate`); the binary prints one value per call.

## What the device rung measured (5 s, 80 kHz strobes, raw stream draining DMA0)

```
records                  370560      (69 k strobes/s + the channels' records)
reads                      1930      max 24576 bytes per read (three DMA buffers)
duplicates_or_reorders      276      (the ring re-delivers a buffer occasionally, as GNSSM2SDR notes)
mean record age at read  1984 µs     measured against gnss_sample_count read right after the read
  age <   1 ms           25266
  age <   2 ms          162454
  age <   4 ms          182840
  age >=  4 ms               0
max loop iteration       9840 µs     (one iteration; unpinned, no SCHED_FIFO yet)
allocated bytes in loop       0
GC pauses in loop             0
NCO word writes             120      (carrier + code CSR per channel) mean 6 µs, max 67 µs
```

Reading: a record's age at the host is dominated by the driver's 8 KiB buffer
granularity (64 records, ~0.75 ms at this rate) plus the two-ioctl `sample_count`
read the measurement itself pays, and never reached 4 ms without any real-time
scheduling. The 9.8 ms worst iteration is the case pinning and `SCHED_FIFO` exist for.
The loop body allocated nothing and triggered no collection over 370 k records.

## Decisions taken

- **Q3:** keep Unitful and StaticArrays in `TrackingLoops`; no plain-Float64 core.
- **Direct DMA1 read** stays the design: the read path is well inside the ring's ~190 ms.
- `TrackingLoops` avoids dynamic strings on the loop paths; `GNSSSignals.read_in_codes`
  needs the non-closure `open` (a follow-up for GNSSSignals).
