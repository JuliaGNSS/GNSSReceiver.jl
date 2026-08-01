# Offline analysis of the hardware-correlator bit stream

These scripts read what `../hardware_correlator_position_fix.jl` writes when it
is run with `HWFIX_BITLOG` set. They exist because the question "the receiver
tracks but never fixes — is that the bits, the bit clock, the tracking or the
decoder?" cannot be answered from the receiver's own indicators. In particular
`GNSSDecoderState.num_bits_after_valid_syncro_sequence` only leaves `nothing`
once GPS subframes 1, 2 **and** 3 have decoded with every word's parity closing
and `IODC[3:10] == IODE_Sub_2 == IODE_Sub_3`, so it reads `-1` throughout a run
in which preamble sync is firing on every subframe. See
[GNSSReceiver.jl#107](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/107).

Nothing here needs the board, only the logs.

## Log format

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

## The scripts

| script | question it answers |
|---|---|
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
