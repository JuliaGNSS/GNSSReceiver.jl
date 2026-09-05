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
| `T` | `wall_ns boundary new_dumps latest_sample_index samples_consumed` | one per chunk: wall clock, the epoch boundary the fold reached, how many new dumps the chunk drained (~2 when the pipeline is steady) and the device's newest sample seen. Tens of dumps in one chunk followed by chunks draining none is a host stall, and the wall clock says whether the raw stream or the processing task stalled. |
| `A` | `ch previous_prn new_prn boundary samples_consumed` | a hardware channel changed occupant: a handover, a release (`0`), or the noise channel re-arming onto its next decoy (negative PRN). |

The `D` line carries three fields appended after the original eight: the C/N₀
the lock detector sees (dBHz), the bit buffer's polarity and its pre-sync
window length.

A second file, `$HWFIX_BITLOG.records`, holds every hardware record the fold
drained, as it came off the wire — i.e. on the true 1 ms grid, *before* the link
coalesces a chunk's dumps into one 2-8 ms record for the loops:

| line | fields | meaning |
|---|---|---|
| `R` | `ch prn sample_index integrated_samples code_phase re_L im_L re_P im_P re_E im_E` | one dump; the accumulators are the gateware's raw integer counts (127× a unit-replica prompt). |

Five satellites at 1 kHz write ~35 MB of `R` lines per minute.

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
| `records.jl <log> [prn…]` | **Signal, false lock or noise — and where do the bits go wrong?** Per PRN, from the raw 1 ms records: record lengths, every stream gap beside the channel-table event nearest it, `\|P\|` against the Early/Late taps (noise gives 1.0), and the phase step between consecutive 1 ms prompts — a locked carrier flips sign at ~2.5 % of pairs (the data transitions), a Costas alias 500 Hz off flips ~96 %, an unlocked one spreads the steps around the circle. Then re-folds the prompts into bits at the best edge and walks the word parity with slip re-search on both those bits and the soft bits Tracking handed `decode`. |
| `weakbits.jl <prompts.f32>` | Finds the bits whose 20 ms accumulation nearly cancelled and prints the 20 individual 1 ms prompts inside them, so a mid-bit sign flip or amplitude collapse is visible directly. |

The GPS parity implementation is IS-GPS-200 20.3.5.2 and is self-tested against
200 synthetically encoded words by `bit_shape.py`'s `selftest` in the issue
thread; `subframes*.jl` share the same equations.

### Without a board

These need only a raw capture (`m2sdr_record -q cap.bin <bytes>`, sc16 2R2T) or
nothing at all, and they exist to answer the question that comes before every
other one: *is the receiver's problem in the signal, or in the receiver?*

| script | question it answers |
|---|---|
| `offline_acq.jl <cap.bin> [offsets_s] [gps,gal]` | **Are the acquisitions real satellites?** Acquires every PRN on several independent segments of the capture and tabulates each one's C/N₀, Doppler and code phase across them. A real satellite holds its Doppler to a few Hz and walks its code phase at `doppler/1540` chips/s; noise picks a fresh random Doppler on every segment. This is what separated four real satellites from 28 noise peaks at a 26.6-27.7 dBHz floor (issue #107). |
| `glitch_check.jl <ant0.bin> <prn> <step_ms> <dur_s>` | **Did the capture lose samples?** Acquires one PRN in a sliding window and checks the code phase against a straight line. Any drop in the recorder shows up as a step; 500 consecutive acquisitions staying inside 0.25 chips says the stream is intact and the fault is downstream. |
| `direct_track.jl <ant0.bin> <prns> <if_hz>` | **Does *software* tracking hold this sky?** Acquisition → `Tracking` on the same samples, reporting C/N₀ and Doppler per second. The lesson it taught: pass the front end's LO offset as Doppler and `IF = 0`, because the offset comes off the same reference as the ADC clock and so is already in the code rate — hand it over as an intermediate frequency instead and the carrier aiding is wrong by 3.3 chips/s and nothing locks, while acquisition on the same file stays rock solid. |
| `sw_receive.jl <cap.bin> [prns] [if_hz]` | The same capture through the whole software `receive` pipeline, for a like-for-like comparison against a hardware-correlator run. |
| `synth_track.jl` | The control: synthetic GPS L1 C/A at 4 MHz through the same acquire→track path, with the answer known. Run it before believing either of the two above. |
| `vis.py <tle-file> <lat> <lon> [utc]` | **Was that PRN even above the horizon?** Two-body + J2 propagation from celestrak TLEs (`curl -O https://celestrak.org/NORAD/elements/gps-ops.txt`), giving azimuth, elevation and predicted Doppler per satellite. Observed-minus-predicted Doppler agreeing on one common offset across every acquired PRN is what proves a scan found satellites rather than noise. No dependencies. |
| `analyze_run.sh <bits.log>` | Summary of one instrumented run: the channel-assignment timeline, every fold gap above 80 ms with the compile and GC time inside it, and the handover's `\|P\|/\|E,L\|` per assignment. |
| `stall_ctx.sh <bits.log> <min_gap_s>` | The chunk sequence either side of each stall — how many dumps each chunk folded and how far the device's sample index moved — which is what distinguishes a starved dump reader from a blocked fold. |

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
