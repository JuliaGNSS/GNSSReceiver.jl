# Field record: live sky through the LiteX-M2SDR hardware correlator, 2026-09-18

This is the record behind the `live_m2sdr_l1_20260918` evidence source of the
[signal-support matrix](../../docs/src/signal_support.md). It cannot be
reproduced in CI: it needs the board, the antenna and the sky of that day. What
it can be is exact about what ran, so the claims in the matrix can be checked
against it.

## Setup

| | |
|---|---|
| Host | orin2 (NVIDIA Jetson Orin, Ubuntu 24.04, Julia 1.12.6) |
| Board | LiteX-M2SDR, AD9361, one antenna seeing about a quarter of the hemisphere |
| Gateware | gnss-m2sdr `gnss_m2sdr_m2_x1_ch4_ant1_code4092_tap5_sub12_placeSpread` — 4 channels, 5 taps, 4092-chip code memory, 12 sub-chips, CSR layout v3, record format v2; built from gnss-m2sdr `fix/carrier-rom-init` (the carrier-ROM fix on top of `8795de1`) with LiteX `37b75bd4`, timing met at WNS +0.003 ns; md5 `107f9ab635d4bd7210a7dc43b42c6c85`; flashed 10:00 UTC |
| RF | `m2sdr_rf --sample-rate 4000000 --rx-freq 1575420000 --rx-gain 60 --bandwidth 4000000` |
| Host software | GNSSReceiver `hardware-correlator-12` at `d8886b2`; GNSSM2SDR `master` `67ca254` plus the NCO-queue fix (`fix/nco-queue`, `d5ba2f1`) for the runs marked *queued*; Tracking, Acquisition, GNSSSignals as resolved in the board's environment on that day |
| Script | [`hardware_live_m2sdr.jl`](hardware_live_m2sdr.jl), `julia -t 6,4`, `noise_source = :samples`, `feedback_delay_epochs = 2`, `epoch_period = 25` (160 kHz strobes) |

Acceptance of the gateware itself before any of this: gnss-m2sdr's
`scripts/hw_accept_v3.py` passed all five checks (CSR layout 3 / record 2,
capabilities read back as built, sample counter at 4.0025 MS/s, GPS L1 C/A
acquired on the FPGA sweep, DMA1 records framed with both a three-tap and a
five-tap channel on the wire).

## Runs

### GPS L1 C/A alone (`l1ca`, 300 s, adapter before the queue fix)

Regression baseline for the four-channel build. PRN 14 held at 47–52 dBHz for
the whole run, PRN 21 at 34–45 dBHz, PRN 20 at 27–43 dBHz; PRN 5 for a minute
at 34–38 dBHz.

```
NCO commits: 359439 at their scheduled sample, landing 0.04 ms late on average (max 19.143 ms); 72 dropped as stale
dump stream: lost-record gaps 3, re-arm gaps 0, device-reported drops 0, skipped epochs 909, implausible indices 0, dropped NCO updates 0, tap-layout mismatches 0, unsupported signals 0
```

No fix: four channels hold at most four satellites, the acquisition hands
false alarms onto free channels along with real satellites, and the three
satellites that stayed locked are one short. The three lost-record gaps were
one 71 ms event across all channels at the first acquisition merge.

### Galileo E1 pilot/data pair alone (`e1only`, 240 s, adapter before the queue fix)

The pair takes two channels per satellite. E1C PRN 16 locked at up to
48.7 dBHz, PRN 3 at 41.0, PRN 5 at 35.6, PRN 34 at 30.9 — the hardware
correlates the BOC(1,1) replica on five taps — but every lock decayed within
ten seconds of its acquisition:

```
NCO commits: 46 at their scheduled sample, landing 5.343 ms late on average (max 16.708 ms); 64 dropped as stale
```

Forty-six commits in four minutes against ~1200 per second for GPS. The E1B
data component alone (`e1bonly`) behaved the same (24 commits, 60 stale), so
the pairing was not the cause. The link's epoch grid was healthy (traced:
boundary 0–4 ms past the newest record). The cause was in the adapter: it held
one pending NCO word per channel and let the next word supersede a word not yet
due. GPS words fall due at the rate they arrive (one per 2 ms chunk, scheduled
2 ms ahead); E1 words arrive every 4 ms fold scheduled 8 ms ahead, so each was
replaced before its sample came and the device ran on its handover word.

### Galileo E1B alone (`e1bonly`, 150 s, *queued*)

With the adapter queuing words per channel (GNSSM2SDR `fix/nco-queue`):

```
first fix after 39.8 s: 50.769195° 6.072997° 267 m (4 sats)
NCO commits: 68338 at their scheduled sample, landing 0.01 ms late on average (max 10.29 ms); 48 dropped as stale
dump stream: lost-record gaps 0, re-arm gaps 0, device-reported drops 0, skipped epochs 1902, implausible indices 3, dropped NCO updates 0, tap-layout mismatches 0, unsupported signals 0
best C/N0 while in lock: E1B PRN 16 46.8 dBHz, PRN 34 41.7, PRN 32 37.2, PRN 3 36.0
```

Four Galileo satellites on five-tap channels with the 4092-chip primary code,
their I/NAV pages decoded to ephemerides within 40 s, and a Galileo-only
position fix at the site. This is the evidence for the `GalileoE1B_BOC11`
acquisition-handover, tracking, data-decode and PVT cells.

What it is not: the CBOC replica (`GalileoE1B`), which needs at least
12.276 MS/s for the software acquisition's replica and was not run; the pilot
component's secondary-code synchronisation on hardware (`GalileoE1C_BOC11`
`secondary_sync` stays untested); and a position accuracy statement — no
surveyed reference.

### GPS L1 C/A alone (`l1ca`, 300 s, *queued*)

The regression baseline with the queued adapter. Four satellites in lock for
stretches (PRN 5, 14, 21 and 18, then 5, 21, 15 and 30, 32–45 dBHz); no fix,
because no four of them had decoded ephemerides at the same time within the
run. More commits than before, since a word is no longer lost when the next
one arrives early:

```
NCO commits: 477862 at their scheduled sample, landing 0.04 ms late on average (max 18.678 ms); 77 dropped as stale
dump stream: lost-record gaps 3, re-arm gaps 0, device-reported drops 0, skipped epochs 549, implausible indices 0, dropped NCO updates 0, tap-layout mismatches 0, unsupported signals 0
```

### GPS L1 C/A next to Galileo E1B in one bank (`mixb`, 200 s, *queued*, `HW_GPS_PRNS=14,5`)

The mixed-layout case: three-tap GPS channels and five-tap Galileo channels in
the same bank and the same record stream, with the GPS search limited to two
satellites so the four-channel bank had room for both constellations. From
t = 90 s to the end GPS PRN 14 (42–46 dBHz) and PRN 15/5 (35–45 dBHz) ran next
to Galileo E1B PRN 34 (40–45 dBHz), PRN 16 (26–39 dBHz) and PRN 15 (41 dBHz
peak):

```
NCO commits: 335485 at their scheduled sample, landing 0.062 ms late on average (max 18.155 ms); 186 dropped as stale
dump stream: lost-record gaps 4, re-arm gaps 0, device-reported drops 0, skipped epochs 6586, implausible indices 2, dropped NCO updates 0, tap-layout mismatches 0, unsupported signals 0
best C/N0 while in lock: GPS 14 46.5 dBHz, GPS 5 45.6, E1B 34 45.1, E1B 16 42.7, E1B 15 41.0
```

No tap-layout mismatch and no unsupported-signal count in any run; the
C/N₀s of the same satellites agree with the single-constellation runs, so the
per-layout scaling holds. No fix: a two-constellation solution needs a fifth
satellite for the second clock bias, and four channels cannot hold one.

## The six-channel build (`gnss_m2sdr_m2_x1_ch6_ant1_code4092_tap5_sub12_synthPerfSpread`, *queued*)

Flashed 13:05 UTC (md5 `2ebd04238d9b6fa24dafbbcb270651ea`, WNS +0.007 ns),
power-cycled, `hw_accept_v3.py` 5 of 5 (n_channels 6). Same RF settings.

### GPS L1 C/A alone, 300 s and 600 s

Six satellites in lock at once (PRN 18, 24, 20, 22, 5, 23 at 30–45 dBHz at
t = 190 s of the first run); in the 600 s run with scans every 120 s, four to
six satellites at 35–45 dBHz from t = 359 s to 537 s with **zero** lost-record
gaps:

```
NCO commits: 1074708 at their scheduled sample, landing 0.04 ms late on average (max 16.577 ms); 80 dropped as stale
dump stream: lost-record gaps 0, re-arm gaps 0, device-reported drops 0, skipped epochs 450, implausible indices 0, dropped NCO updates 0, tap-layout mismatches 0, unsupported signals 0
```

**No fix in either run**, and the per-satellite flags showed why: every GPS
satellite became ranging-ready (bit clock found) within seconds, but in 240 s
only one of five reached a decoded, healthy ephemeris while satellites at 41
to 46 dBHz stayed undecoded.

### Why GPS did not decode, and the fix

Logging every folded prompt for the strong satellites first ruled out the loop:
PRN 15 at 41 dBHz had a carrier-phase error of 17° standard deviation after
removing the bit sign and an imaginary part 0.2 of the real part, so the PLL
was phase-locked, not merely frequency-locked, and the feedback delay was not
the problem.

Logging the decoder's soft-bit buffer and `Tracking`'s bit accumulator found
it. After bit sync the decoder received bits at 50 per second for under a
second, then never again: the bit buffer's `prompt_accumulator_integrated_code_blocks`
had gone 18 → 21 and kept climbing (33 000 blocks two minutes later) instead of
wrapping at 20. `Tracking` emits a bit only when that count *equals* the blocks
per bit, so a single record that crosses the boundary silences a satellite for
good. Galileo E1B is one block per symbol and cannot cross anything, which is
why it decoded.

A block-level trace of the link showed the record that crosses. Two causes,
both in `coherent_integration_blocks` / `_accumulate_dump!`:

1. The link sized records from its own `pending_blocks` tally, zeroed after every
   estimator pass on the assumption that the pass consumed every queued record.
   A record ending past the fold boundary stays queued for the next pass, so the
   tally said 0 where the queue held one block. Now the sizing reads the blocks
   still queued in the signal's own `correlator_outputs`, with `Tracking`'s own
   block arithmetic.
2. A record cut on an NCO-word change hands over blocks the part-accumulated
   record after it was sized without: at 18 blocks the target was 2, a 1-block
   cut went out, and the partial already holding 1 wrap met a target that had
   shrunk to 1 with 2 wraps — `>=` let it through as a 2-block record, 18 + 1 + 2
   = 21. Now the target is read after the cut, and a partial that already
   reaches it is handed over before the next dump joins it, so a record never
   grows past the boundary.

With both, a traced 160 s run showed no satellite's accumulator above 19,
every decoder buffer filled to its 308 bits, and PRN 13, 15, 20 and 23 (40 to
42 dBHz) synchronised their LNAV frames; PRN 10 and 12 at 33 to 35 dBHz had
full buffers and no sync yet within the run.

### GPS L1 C/A alone, 600 s, with both fixes (`HW_ACQ_EVERY=150`)

```
first fix after 205.1 s: 50.770094° 6.073362° 218 m (4 sats)
NCO commits: 688136 at their scheduled sample, landing 0.041 ms late on average (max 15.343 ms); 112 dropped as stale
dump stream: lost-record gaps 4, re-arm gaps 0, device-reported drops 0, skipped epochs 2072, implausible indices 0, dropped NCO updates 0, tap-layout mismatches 0, unsupported signals 0
first fix after 205.1 s, 1095 fresh solutions afterwards
```

PRN 23 decoded first (healthy at 39 s), then 20, 12 and 15; the fix came when
the fourth ephemeris landed, and held for the remaining 100 s of the run, at the
same site the Galileo-only fix had placed the antenna. **GPS L1 C/A through the
six-channel hardware correlator: acquisition, tracking, LNAV decode and a
position fix, on sky.** The time to fix is set by the LNAV frame cadence and by
bit sync arriving late for some satellites (below), not by the correlator.

Two things seen on the way, not fixed here: `Tracking`'s bit buffer should not
be silenced forever by one overshooting record (an `>=` with a resync, or a
reported error, would be kinder than the `==`); and bit sync on the hardware
path often arrives only after a record-loss restart rather than within seconds
of the handover (PRN 13 at 41 dBHz: tracked from t = 8 s, bit sync at
t = 122 s, right after the second acquisition scan's stall restarted it), which
points at the first, fractional record after arming and deserves its own look.

## What the four-channel build cannot show

A GPS fix (four satellites with ephemerides at once did not coincide in
300 s), a mixed-constellation fix (five satellites), and anything that needs
the second antenna. The six-channel build removes the first two limits in
principle; the GPS decode finding above is what stands in the way now.
