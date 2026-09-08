# Reproduce the issue 107 hardware state

Use this directory from the exact GNSSReceiver commit linked in issue 107,
with **Julia 1.12.6**. `Manifest.toml` preserves the measured registry package
versions and tree hashes. Its four local package paths are relative:

| Package | Source |
|---|---|
| GNSSReceiver | This checkout, three directories above |
| GNSSDecoder | `28725906711265c341b59542bb4a5dabc99a9e87` |
| Tracking | `a2ff103b1c7657e6db8e177c59f4fe7f8d84c5ac` |
| GNSSM2SDR | Exact board source copied into `vendor/GNSSM2SDR`, including its MIT license |

The vendor package on the board had no Git metadata. Its source snapshot is
intentional: replacing it with another version would not reproduce these tests.
Setup fetches the two pinned Git revisions and instantiates the manifest; it
does not update dependencies or reconfigure the FPGA/RF hardware.

```bash
export JULIA="$HOME/.juliaup/bin/julia"  # must resolve to 1.12.6
bash examples/analysis/repro_hwfix12/setup.sh
bash examples/analysis/repro_hwfix12/run.sh 240 200
```

Each invocation creates a separate ignored `runs/run-*` directory, copies the
example there, and writes its logs beside it. The defaults match the campaign:
6 default / 3 interactive Julia threads, processing on the interactive pool,
full-constellation acquisition every 60 seconds, 4 MSPS, 2 ms chunks, 12 Hz
carrier-loop bandwidth, eight hardware channels (one noise channel), and one
epoch feedback delay. The example warms the actual processing/logging paths
before tracking. It disables GC for the short measurement; long-running GC
behavior remains unvalidated.

## Board prerequisites

The existing `orin2` setup uses kernel `6.8.12-1021-tegra`, the `m2sdr` module,
`/dev/m2sdr0` and `/dev/m2sdr1`, and `/usr/bin/m2sdr_record`. The example expects
`~/gnss-m2sdr/build/gnss_m2sdr_m2_x1_ch20_ant1/csr.csv`. RF settings used were
4 MHz sample rate, 2.5 MHz bandwidth, 1575.42 MHz receive frequency, RX gain 60,
and antenna 0 of the captured 2R2T stream. The physical antenna was unchanged.
The hardware bring-up commands are documented in the example; do not assume a
fresh board has this driver, gateware, RF setup, or pipe-size limit.

Hardware artifacts and original run/test logs are preserved in the board
archive described in `archive.txt`. `hardware.sha256` identifies the copied
CSR map, gateware build artifact, kernel module, recorder, and host/library
metadata. The gateware hash identifies the build file, not an FPGA readback.
The binary module is specific to the recorded kernel. The archive is retained
on `orin2`, not hosted in Git; access requires the same SSH access as the tests.

## Verification and replay

```bash
julia --project=examples/analysis/repro_hwfix12 -e \
  'using Pkg; Pkg.test("GNSSReceiver")'
julia --project=examples/analysis/repro_hwfix12 -e \
  'using Pkg; Pkg.test("GNSSDecoder")'
julia --project=examples/analysis/repro_hwfix12 \
  examples/analysis/replay_pvt.jl /path/to/warm_final_bits.log 50.7686 6.0727 270
julia --project=examples/analysis/repro_hwfix12 \
  examples/analysis/decoder_latency.jl /path/to/patched_bits.log
```

The decoder-only replay measures package cold calls; reproducing the zero-JIT
result after the example's complete warm-up uses `run/verify_ready.jl` and
`run/position_fix_ready.jl` from the board archive. The final source and those
files differ only in explanatory comments. See [the measurement report](../hardware_fix_12.md)
for the tested revisions, reference-position limitations, test results,
fresh-versus-held positions, and remaining non-JIT feedback jitter.
