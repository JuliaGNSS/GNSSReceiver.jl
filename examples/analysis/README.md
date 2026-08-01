# Offline analysis of the hardware correlator

These scripts read what `../hardware_correlator_position_fix.jl` writes when it
is run with `HWFIX_BITLOG` or `HWFIX_XCORR` set. They exist because the question
"the receiver tracks but never fixes — is that the bits, the bit clock, the
tracking, the correlator or the decoder?" cannot be answered from the receiver's
own indicators. In particular
`GNSSDecoderState.num_bits_after_valid_syncro_sequence` only leaves `nothing`
once GPS subframes 1, 2 **and** 3 have decoded with every word's parity closing
and `IODC[3:10] == IODE_Sub_2 == IODE_Sub_3`, so it reads `-1` throughout a run
in which preamble sync is firing on every subframe. See
[GNSSReceiver.jl#107](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/107).

Nothing here needs the board, only the logs.

## Bit-stream log format

One text log (`$HWFIX_BITLOG`) plus one binary file per PRN
(`prompts_prn<N>.f32`), written next to the run script.

| line | fields | meaning |
|---|---|---|
| `B` | `prn boundary softbit…` | every soft bit this chunk produced, in order. `boundary` is the device sample index the fold reached, so it timestamps the chunk. |
| `P` | `prn boundary n found blocks` | `n` 1 ms filtered prompts were appended to `prompts_prn<prn>.f32` this chunk; `found` is the bit-sync state and `blocks` the open bit's block count. |
| `D` | `prn boundary abs(late) abs(prompt) abs(early) carrier_doppler code_doppler code_phase` | the last fully integrated correlator and what the loops are steering with. |
| `C` | `ch prn boundary lost overlap last_record_end` | per-channel record continuity, once a second. `lost` is in device samples of records the host never saw. |
| `L` | `boundary gaps stale dropped skipped_epochs` | the link's own counters, once a second. |

`prompts_prn<N>.f32` is interleaved `Float32` real/imaginary parts of the 1 ms
filtered prompts, contiguous in time (the receiver's `filtered_prompts` vector
is emptied every chunk, so the log captures all of them).

## FPGA-vs-CPU log format

One text log (`$HWFIX_XCORR`), written next to the run script. Every line is one
hardware record with a CPU correlation of the very same raw samples beside it.

| line | fields | meaning |
|---|---|---|
| `X` | `prn ch sample_index n code_phase delta_chips re_Pf im_Pf abs_Lf abs_Ef re_Pc im_Pc abs_Lc abs_Ec carrier_doppler code_doppler` | `…f` is the FPGA's accumulator, `…c` the CPU's over device samples `[sample_index-n, sample_index)`. `code_phase` is the device replica's own phase at the end of the record; `delta_chips` is the CPU-side offset on top of it, i.e. the measured host↔device axis residual. |
| `A` | `prn ch sample_index offset_samples offset_chips peak_over_median sign` | one per channel occupant: the coarse ±64-sample alignment search, and the mixing-sign vote. `peak_over_median` below ~4 means the CPU reference never found the satellite, and every `X` line for that occupant is meaningless. |

The two sides differ in absolute scale (the gateware's accumulators are integer
counts of a 127-amplitude replica), so `fpga_vs_cpu.jl` fits the gain rather
than assuming it. Nothing else is normalised.

## The scripts

| script | question it answers |
|---|---|
| `fpga_vs_cpu.jl <log> [prn…]` | **What does the hardware correlator lose?** Per PRN: the gain between the two prompts, their complex coherence ρ (and so how much noise the FPGA has that the CPU does not), the beat frequency between them (non-zero ⇒ the carrier NCO is not running the word it was given), a C/N0 for each stream from the same NWPR estimator, and how often their 20 ms bit decisions disagree. |
| `selftest_cpu_correlator.jl` | Is the CPU reference itself trustworthy? Injects a signal of known PRN, code phase and Doppler and requires the analytic answer — prompt amplitude, Early/Late split, the alignment search recovering a deliberate sample error, the discriminator's sign, and no amplitude drift over a 20 ms integration. Run it before believing a comparison. |
| `subframes_softbits.jl <log> <prn>` | Do the soft bits **the decoder was handed** contain real GPS subframes? Reports every preamble whose TLM+HOW parity closes, its decoded TOW count and subframe id, and checks that pairs 300 bits apart differ by exactly one TOW. That last check is what separates a real frame from a coincidence. |
| `subframes.jl <prompts.f32> [offset]` | The same, but on a re-fold of the raw 1 ms prompts at a chosen bit-edge offset. Differences against the previous script are Tracking's accumulation or the per-chunk feed, not the signal. |
| `word_errors.jl <log> <prn> <anchor-bit>` | Given one known subframe position, walk the whole stream on the 300-bit grid and report which of each subframe's ten words fail parity. Uniform sprinkling is thermal noise; bursts are discrete events in the ingest path. |
| `walkoff.jl <log> [bin_seconds]` | Is the replica drifting off the correlation peak? Prints `|E|`, `|P|`, `|L|`, the early/late imbalance and both Dopplers, binned in time. A decaying prompt with the imbalance pinned at zero is *not* a code walk-off. |
| `analyse_prompts.jl <dir>` | Per PRN: 20 ms phase coherence, residual carrier from the squared prompt, and a sweep of all 20 bit-edge hypotheses (raw and carrier-derotated) for a validating subframe. |
| `weakbits.jl <prompts.f32>` | Finds the bits whose 20 ms accumulation nearly cancelled and prints the 20 individual 1 ms prompts inside them, so a mid-bit sign flip or amplitude collapse is visible directly. |

The GPS parity implementation is IS-GPS-200 20.3.5.2 and is self-tested against
200 synthetically encoded words by `bit_shape.py`'s `selftest` in the issue
thread; `subframes*.jl` share the same equations.

## Worked example

```bash
cd examples
HWFIX_BITLOG=bitstream.log julia -t 6,2 --project=. \
    hardware_correlator_position_fix.jl 220 20

julia analysis/subframes_softbits.jl bitstream.log 7
#   bit  993  TOW-count 94647  subframe id 2
#   bit 1293  TOW-count 94648  subframe id 3      993 → 1293: Δbits 300, ΔTOW 1 ✓ REAL
julia analysis/word_errors.jl bitstream.log 7 993
#   word parity failure rate: 139 / 230           ⇒ BER ≈ 3%, only 2 of 31 subframes clean
julia analysis/walkoff.jl bitstream.log 10
#   (E-L)/(E+L) inside ±0.02 for 200 s            ⇒ not a code walk-off
```

…and the correlator comparison, which is a separate run (both instruments at
once cost more CPU than the processing task has to spare):

```bash
julia analysis/selftest_cpu_correlator.jl        # trust the reference first
HWFIX_XCORR=xcorr.log julia -t 6,2 --project=. \
    hardware_correlator_position_fix.jl 240 60
julia analysis/fpga_vs_cpu.jl xcorr.log
```
