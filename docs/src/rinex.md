# RINEX Output

[`receive`](@ref) can write the measurements it makes, and the ephemerides it decodes,
straight to **RINEX 3.05** files while it runs — the interchange format every GNSS
post-processing tool reads (RTKLIB, gLAB, GAMIT, the IGS archives). That turns a run of this
receiver into an observation file you can post-process for a precise position, compare
against a reference station, or archive.

RINEX output is **off by default**. Switch it on with the `write_rinex` keyword:

```julia
receive(measurement_channel, GPSL1CA(), sampling_freq; write_rinex = true)
```

which writes a pair of files named by the RINEX long filename convention into the working
directory — `UNKN00XXX_R_20200010000_00U_01S_GO.rnx` and its `…_GN.rnx` navigation
counterpart. Pass a [`RinexConfig`](@ref) instead of `true` to choose where they go, what
the name says about the site, the epoch interval and the header metadata:

```julia
receive(
    measurement_channel,
    GPSL1CA(),
    sampling_freq;
    write_rinex = RinexConfig(;
        directory = "recordings",
        interval = 1.0u"s",
        period = 1u"hr",
        # Header metadata the receiver cannot know, as RinexObsHeader keywords.
        obs_header = (;
            marker_name = "ROOF12DEU",
            antenna_type = "TRM59800.00     SCIS",
            observer = "A. Observer",
            agency = "Example University",
        ),
    ),
)
```

That run writes `recordings/ROOF12DEU_R_20200010000_01H_01S_GO.rnx` and
`recordings/ROOF12DEU_R_20200010000_01H_GN.rnx`. Set `obs_file` or `nav_file` to a path of
your own to name a file yourself, or to `nothing` to skip it.

## File names

The names come from [RINEXParser's](https://github.com/JuliaGNSS/RINEXParser.jl)
`rinex_filename`, which builds them from the header of the file being written, so they are
the convention's own fields rather than anything this package invents:

| Field                | Where it comes from                                                                                                                                                                                                              |
|:-------------------- |:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `XXXXMRCCC` site     | the `marker_name` of `obs_header`, which RINEX 3 expects to *be* the nine-character site identification — one is taken apart into station, monument, receiver and country, any other marker name yields the station alone (`"ROOF-1"` → `ROOF00XXX`) |
| `R` data source      | always "from receiver data"                                                                                                                                                                                                      |
| `YYYYDDDHHMM` start  | the first epoch the run stamped, in GNSS system time; a navigation file without observations takes the `toc` of its first ephemeris                                                                                             |
| `PPU` file period    | `period`, or `00U` where the length of the run was not planned                                                                                                                                                                   |
| `FFU` data frequency | `interval`, so a 1 Hz run says `01S`; navigation names carry no such field                                                                                                                                                       |
| `DT` data type       | the constellations being tracked and the kind of file: `GO`, `EO`, `MO`, `MN`, …                                                                                                                                                 |

A long filename carries the start time of the file, and this receiver does not know that
until its first fix — long after the files have to be open, since an unwritable path should
fail at the `receive` call. So the files are written to temporary `*.rnx.part` files in
`directory` and named when they are closed, by RINEXParser's `RinexDirectory`. Rerunning the
same recording therefore lands on the same names and replaces the files. A file with no data
to name it by — a run that never got a fix, a navigation file that never got an ephemeris —
is not kept; give `obs_file` and `nav_file` paths to keep header-only files of such a run.

Names are placed in `directory`, which also prefixes a relative `obs_file` or `nav_file`; an
absolute one ignores it. The directory has to exist — a run does not create one, so a typo
puts nothing in an unexpected place.

The same keyword works on [`gnss_receiver_gui`](@ref), so a live SDR session can record
RINEX while you watch the terminal display.

The formatting itself lives in [RINEXParser.jl](https://github.com/JuliaGNSS/RINEXParser.jl),
a receiver-agnostic streaming writer. So do the RINEX conventions of the signals and
messages: its GNSSSignals extension names each signal's observation codes, and its
GNSSDecoder extension turns a decoded navigation message into an ephemeris record and
header records. This package converts the receiver's own state — tracking loops, PVT
solution, decoders — into those records. Files are written from a background task, so the tracking loop never waits on
the disk, and they are complete and closed by the time the data channel closes.

## What ends up in the files

The **observation file** carries, per satellite and signal, the four observables the
receiver measures:

| RINEX | Observable                   | Source                                                        |
|:----- |:---------------------------- |:------------------------------------------------------------- |
| `C…`  | Pseudorange (m)              | satellite transmit time of the PVT solution against the epoch |
| `L…`  | Carrier phase (whole cycles) | continuously accumulated tracking-loop carrier phase          |
| `D…`  | Doppler (Hz)                 | carrier Doppler of the tracking loops                         |
| `S…`  | Signal strength (dB-Hz)      | `estimate_cn0`                                                |

The observation codes come from the constellation code tables of RINEX 3.05 — Table 14
(GPS), 16 (Galileo) and 19 (BDS), as RINEXParser's `observation_code` gives them — and name
the signal the receiver actually ranges on — for a [`CombinedSignal`](@ref) that is the pilot, so
`CombinedSignal(GalileoE1C(), GalileoE1B())` is written as `C1C`/`L1C`/`D1C`/`S1C` under
system `E`. A satellite tracked on several signals or bands appears once per epoch, with
each signal in its own column, and several constellations produce one mixed (`M`) file.

The **navigation file** carries the decoded broadcast ephemerides, deduplicated so each is
written exactly once, plus the ionosphere (Klobuchar / NeQuick), time-system (UTC, GGTO) and
leap-second header records as satellites broadcast them, the latest set of each kind
(BeiDou's per transmitting satellite and hour). The leap seconds of a mixed file are the
GPS-counted record where a satellite has sent one. Week numbers the message
broadcasts truncated — the 8-bit UTC and leap-second weeks, Galileo's 6-bit GGTO week — are
written as the continuous week RINEX asks for, and an ephemeris broadcast across a week
boundary is filed under the week of its `toe`.

!!! note "Which navigation messages can be written"

    RINEX 3.05 defines broadcast-ephemeris records for **GPS LNAV** (L1 C/A), for **Galileo
    I/NAV** (E1B, E5b) and **F/NAV** (E5a), and for **BeiDou D1/D2** (B1I, B3I). The GPS
    CNAV and CNAV-2 messages (L5, L2C, L1C), the BeiDou B-CNAV messages (B1C, B2a, B2b) and
    Galileo's E6 C/NAV broadcast a quasi-Keplerian ephemeris that RINEX only gained a record
    for in version 4, so those satellites are logged with a warning and contribute
    observations but no ephemerides. Their *observations* are unaffected, and an external
    navigation file can supply the orbits.

!!! note "The one signal that cannot be written at all"

    Galileo **E5a-QP** is new in OS SIS ICD Issue 2.2, published after RINEX 3.05, whose
    Galileo codes cover only E5a I, Q and I+Q. There is no observation code to write it
    under, so enabling RINEX output for a run that ranges on it is an error rather than a
    guess at a code that means a different signal.

## BeiDou and the BDT time scale

BeiDou keeps its own clock: **BDT runs 14 s behind GPS Time**, because it started at zero on
2006-01-01 aligned with UTC, by which point GPS had accrued 14 leap seconds. Two consequences
are handled here rather than left to the reader of the file:

  - `PositionVelocityTime` reports every satellite's transmit time on *its own*
    constellation's scale — it has to, since the satellite position is propagated from that
    same time against an ephemeris on that same scale. Pseudoranges are differences of
    transmit times, so this receiver applies the identical correction the PVT solve does
    before differencing them. Left out, a BeiDou pseudorange would be wrong by 14 s of
    light travel: 4.2 million metres.
  - A BeiDou navigation record is stamped in BDT throughout — its week is the broadcast BDT
    week, not aligned onto the GPS week as Galileo's is, and its `toc` is the calendar time
    that week and second make on BeiDou's own origin.
  - An observation file's epochs are in the time system its header states: BDT for a
    BeiDou-only file, and **GPS time for a mixed one**, whatever constellation the PVT
    solution happens to reference (it picks the one with the most satellites in view).

The same care runs through the header records. BeiDou's broadcast leap seconds are BDT −
UTC, 14 fewer than the GPS − UTC a blank time-system field is read as, and its event week
and day count from the BDT epoch — so the `LEAP SECONDS` record names `BDT`, without which
its numbers would be read as GPS ones. And a BDS ionosphere record carries the satellite and
the hour of the day it was broadcast on, which RINEX makes mandatory for BeiDou alone: it
transmits several parameter sets a day, and nothing else tells them apart.

## Epoch times, pseudoranges and the receiver clock

A hardware receiver stamps its epochs with its own clock and reports how far that clock has
drifted from GNSS system time. A software receiver has no such clock — the only time it
knows is the one the PVT solution derives from the satellites. So instead of steering a
clock, this receiver steers the *epochs*:

  - Observations begin at the **first PVT fix**, since before it there is no absolute time to
    stamp an epoch with.
  - Every chunk after the fix has a receive time: the latest solution's own epoch, carried
    on to the end of the chunk the observables are read at. Under vector tracking that epoch
    lies on the navigation filter's cycle grid, a little before the chunk ends; a solution
    is carried on for at most two `pvt_update_interval`s.
  - Each epoch is placed on an integer multiple of `RinexConfig.interval` in GNSS system time.
    It is written on the chunk whose receive time crosses that grid point, and the
    observables are propagated back onto it with their own Doppler — over less than one
    chunk. That matters for the carrier phase: the loops' instantaneous Doppler is noisy by a
    hertz or two, which over the half second a solution can lie off the grid would add a
    cycle of noise to every phase.
  - Consequently **no receiver clock offset is reported**: this receiver's clock *is* GNSS
    system time by construction. Post-processing tools that expect epochs on round seconds
    with a small clock error (the IGS convention) get exactly that. The header says so with
    `RCV CLOCK OFFS APPL` set to 1: epochs, code and phase have the clock offset applied.
  - The rate is therefore **exactly** `interval`, and immune to receiver clock drift. Because
    the grid is in system time rather than receiver time, drift only decides *which* chunk
    crosses each grid point, never how far apart the grid points are. Post-processing
    tools that want a true 1 Hz or 2 Hz stream — on exact integer seconds, no jitter, matching
    the `INTERVAL` header record — get that. (A hardware receiver without clock steering
    instead puts its epochs on round *receiver-clock* seconds, which walk away from system
    time, and leans on the reported clock offset to explain the difference.)

Gaps in the epoch sequence mean what they say: the receiver had no fix for those seconds
(fewer than four usable satellites). RINEX allows them and post-processing handles them, but
they are real missing data, not a rate artefact.

Pseudoranges are formed from the satellite transmit times of the PVT solution, so they are
exactly the ranges the reported fix was computed from. This also means **only satellites
that contributed to the fix carry observations**: a satellite still decoding its navigation
message has no transmit time, and hence no pseudorange to report.

One consequence is worth spelling out. The tracking loops measure against the receiver's own
oscillator, so its frequency error appears as a **common-mode Doppler** on every satellite —
for the recording used below, 7.4·10⁻⁷, or 1165 Hz at L1. The epochs and pseudoranges are
already free of it, so it is removed from the reported Doppler, using the clock drift of
the PVT solution. The carrier phase, measured against that oscillator, is corrected by the
solution's clock drift, integrated from the first chunk for every satellite alike. Every
observable in the file therefore lives on the GNSS time scale, and two satellites' phases
stay a whole number of cycles apart whenever each of their arcs began. Leaving the clock in
would let code and phase diverge at 220 m/s — kilometres over a minute — which breaks
cycle-slip detection and any carrier-phase processing.

The carrier phase is accumulated across every processed chunk rather than differentiated from
the Doppler, which is what keeps it at the millimetre precision that makes a phase observable
worth having. `Tracking` reports the phase wrapped into a single cycle; the whole cycles are
recovered by rounding the loops' own Doppler prediction onto the measured fractional change,
so none is lost or invented. Each arc starts from the measured carrier beat phase (the
replica's phase with the IF ramp taken out), and its ambiguity is a whole number of cycles
chosen near the arc's first pseudorange, so the fractional part of `L` is the measured one and
the ambiguities of two satellites differ by whole cycles.

Under vector tracking the navigation filter sets each satellite's carrier NCO, so the
replica drifts off the carrier; the residual PLL's estimate of that drift
(`get_carrier_phase_correction`) is added back, whole cycles included. A satellite released
from the vector loop starts a new arc, as its phase steps by the residual there.

The Costas loop locks either on the carrier or half a cycle off it. Where the receiver ranges
on a pilot with a secondary code, the pilot's sync resolves which within a second: the
overlay's sign it read off the prompts is the carrier's against the replica. Otherwise the
decoder resolves it, from the polarity of the navigation bits combined with the sign the bit
sync locked at, read off a preamble within the arc: after the loop has been out of phase lock
it may have slipped half a cycle, which the decoder only notices at its next preamble. The
phase is moved by half a cycle where the loop locked off the carrier. Under vector tracking
the estimator decodes the prompts rotated by its residual PLL, onto the replica's phase plus
the correction, so the same rule resolves the phase written there. Until both are known the
phase carries the **half-cycle bit** of the loss-of-lock indicator.

A phase is only written while the carrier loop is in phase lock, as the loop itself decides
it. In the vector loop that is the residual PLL's verdict, and the arc ends on the first chunk
it fails: riding out a short unlocked spell keeps few arcs and lets slips into them. It also
ends once a satellite's signal is gone — concealed, its C/N₀ below the vector loop's lock
threshold — where the filter coasts it on its prediction, which measures no phase. A scalar
loop's verdict — an indicator averaged over 0.1 s — dips now and then on a locked signal,
while the loop keeps steering its replica onto the carrier, so its arc ends after 100 ms out
of phase lock in a row. A satellite in the fix without a phase arc is still written, with
its pseudorange, Doppler and C/N₀ and a blank phase. An observable too wide for its field
— a pseudorange off a wrongly decoded time of week — is written blank, with a warning,
rather than dropping the epoch.

RINEX 3.05 requires every phase aligned with its band's reference signal (Table A23). The
phase this receiver measures is that of the signal it ranges on, which for a pilot is not the
reference: Galileo `L1C` is half a cycle off `L1B`, GPS and Galileo `L5Q` a quarter off
`L5I`. The correction Table A23 gives is added to it and stated in the header's
`SYS / PHASE SHIFT` records; a reference signal's record is blank. The BDS-3 pilots, which the
table only says must be aligned, take the quarter cycle RTKLIB uses.

When a satellite drops out of lock or out of phase lock, or its polarity changes within an
arc, the arc ends, and the first epoch of the next one carries the **loss-of-lock indicator**
so post-processing knows not to connect the two.

## A complete example

The run below is the same public recording as the [Worked Example (Real Data)](@ref), with
RINEX output switched on.

```@example rinex
using Downloads, GNSSReceiver, GNSSSignals, Unitful

# Same recording as the worked example (downloaded once, then cached on disk).
url = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
file = joinpath(tempdir(), "RTLSDR_Bands-L1_prefix.uint8")
nbytes = 193_000_000
if !isfile(file) || filesize(file) < nbytes
    Downloads.download(url, file; headers = ["Range" => "bytes=0-$(nbytes - 1)"])
end

sampling_freq = 2.048e6u"Hz"
num_samples = Int(upreferred(sampling_freq * 4u"ms"))
directory = mktempdir()

data_channel = receive(
    read_uint8_iq_file(file, num_samples; center = 127.5, type = ComplexF32),
    GPSL1CA(),
    sampling_freq;
    approximate_year = 2017,   # resolves the GPS week-number rollover for old data
    write_rinex = RinexConfig(;
        directory,
        interval = 1.0u"s",
        country = "NLD",   # the recording is from the Netherlands
        # Not a site identification, so the marker name names the station.
        obs_header = (; marker_name = "ION-RTLSDR", observer = "GNSSReceiver.jl"),
    ),
)

# Draining the channel runs the receiver; the files are complete once it closes.
results = collect_data(data_channel)
length(results)
```

The names the files ended up with. The recording is from 2017, and the names say so: they
report the start time of the data, not of the run that read it.

```@example rinex
readdir(directory)
```

The observation file — header, then one epoch record per second followed by one line per
satellite:

```@example rinex
obs_file = only(filter(endswith("GO.rnx"), readdir(directory; join = true)))
print(join(first(readlines(obs_file), 24), "\n"))
```

And the first broadcast ephemeris in the navigation file:

```@example rinex
nav_file = only(filter(endswith("GN.rnx"), readdir(directory; join = true)))
nav = readlines(nav_file)
print(join(nav[1:(findfirst(contains("END OF HEADER"), nav)+8)], "\n"))
```

## API

```@docs
RinexConfig
GNSSReceiver.RinexLogger
GNSSReceiver.log_rinex!
GNSSReceiver.CarrierPhaseAccumulator
GNSSReceiver.advance!
GNSSReceiver.ObsEpochTiming
```
