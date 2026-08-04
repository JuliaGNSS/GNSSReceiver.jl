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
        marker_name = "ROOF12DEU",
        antenna_type = "TRM59800.00     SCIS",
        observer = "A. Observer",
        agency = "Example University",
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
| `XXXXMRCCC` site     | `marker_name`, which RINEX 3 expects to *be* the nine-character site identification — one is taken apart into station, monument, receiver and country, any other marker name yields the station alone (`"ROOF-1"` → `ROOF00XXX`) |
| `R` data source      | always "from receiver data"                                                                                                                                                                                                      |
| `YYYYDDDHHMM` start  | the first epoch the run stamped, in GNSS system time                                                                                                                                                                             |
| `PPU` file period    | `period`, or `00U` where the length of the run was not planned                                                                                                                                                                   |
| `FFU` data frequency | `interval`, so a 1 Hz run says `01S`; navigation names carry no such field                                                                                                                                                       |
| `DT` data type       | the constellations being tracked and the kind of file: `GO`, `EO`, `MO`, `MN`, …                                                                                                                                                 |

A long filename carries the start time of the file, and this receiver does not know that
until its first fix — long after the files have to be open, since an unwritable path should
fail at the `receive` call and a run that never gets a fix should still leave valid
header-only files. So the files are written under the name the run's own wall clock gives
them and renamed onto the name their data gives them when they are closed. Rerunning the
same recording therefore lands on the same name and replaces it, and a run that never got a
fix keeps the name it was opened under.

Names are placed in `directory`, which also prefixes a relative `obs_file` or `nav_file`; an
absolute one ignores it. The directory has to exist — a run does not create one, so a typo
puts nothing in an unexpected place.

The same keyword works on [`gnss_receiver_gui`](@ref), so a live SDR session can record
RINEX while you watch the terminal display.

The formatting itself lives in [RINEXParser.jl](https://github.com/JuliaGNSS/RINEXParser.jl),
a receiver-agnostic streaming writer; this package only converts receiver state into its
record types. Files are written from a background task, so the tracking loop never waits on
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

The observation codes follow RINEX 3.05 Table A2 and name the signal the receiver actually
ranges on — for a [`CombinedSignal`](@ref) that is the pilot, so
`CombinedSignal(GalileoE1C(), GalileoE1B())` is written as `C1C`/`L1C`/`D1C`/`S1C` under
system `E`. A satellite tracked on several signals or bands appears once per epoch, with
each signal in its own column, and several constellations produce one `MIXED` file.

The **navigation file** carries the decoded broadcast ephemerides, deduplicated so each is
written exactly once, plus the ionosphere (Klobuchar / NeQuick), time-system (UTC, GGTO) and
leap-second header records as satellites broadcast them.

!!! note "Which navigation messages can be written"

    RINEX 3.05 defines broadcast-ephemeris records for **GPS LNAV** (L1 C/A) and for
    **Galileo I/NAV** (E1B) and **F/NAV** (E5a). The GPS CNAV and CNAV-2 messages (L5, L2C,
    L1C) broadcast a quasi-Keplerian ephemeris that RINEX only gained a record for in
    version 4, so those satellites are logged with a warning and contribute observations but
    no ephemerides. Their *observations* are unaffected, and an external navigation file can
    supply the orbits.

## Epoch times, pseudoranges and the receiver clock

A hardware receiver stamps its epochs with its own clock and reports how far that clock has
drifted from GNSS system time. A software receiver has no such clock — the only time it
knows is the one the PVT solution derives from the satellites. So instead of steering a
clock, this receiver steers the *epochs*:

  - Observations begin at the **first PVT fix**, since before it there is no absolute time to
    stamp an epoch with.
  - Each epoch is placed on an integer multiple of `RinexConfig.interval` in GNSS system time,
    and the observables are propagated from the measurement instant onto that nominal time
    with their own Doppler. The propagation is at most half an emission interval, over which
    the range rate is constant to well under a millimetre.
  - Consequently **no receiver clock offset is reported**: this receiver's clock *is* GNSS
    system time by construction. Post-processing tools that expect epochs on round seconds
    with a small clock error (the IGS convention) get exactly that.
  - The rate is therefore **exactly** `interval`, and immune to receiver clock drift. Because
    the grid is in system time rather than receiver time, drift only decides *which* chunk
    lands closest to each grid point, never how far apart the grid points are. Post-processing
    tools that want a true 1 Hz or 2 Hz stream — on exact integer seconds, no jitter, matching
    the `INTERVAL` header record — get that. (A hardware receiver without clock steering
    instead puts its epochs on round *receiver-clock* seconds, which walk away from system
    time, and leans on the reported clock offset to explain the difference.)
  - The one requirement is that the interval be comfortably longer than `receive`'s
    `pvt_update_interval` (100 ms by default), since that is how often an epoch time becomes
    available. At 1 s against 100 ms there is 10× of margin. If the two are made equal, drift
    slowly slides the candidate instants against the grid until a grid point gets two of them
    or none — one repeat or gap every `1/drift` epochs, which is ~37 h at 0.7 ppm but every
    ~17 min at 100 ppm. Asking for that combination warns.

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
already free of it, so it is removed from the reported Doppler and from the accumulated
carrier phase as well, using the clock drift of the PVT solution. Every observable in the
file therefore lives on the GNSS time scale. Leaving it in would let code and phase diverge
at 220 m/s — kilometres over a minute — which breaks cycle-slip detection and any
carrier-phase processing.

The carrier phase is accumulated across every processed chunk rather than differentiated from
the Doppler, which is what keeps it at the millimetre precision that makes a phase observable
worth having. `Tracking` reports the phase wrapped into a single cycle; the whole cycles are
recovered by rounding the loops' own Doppler prediction onto the measured fractional change,
so none is lost or invented. The phase ambiguity is arbitrary in RINEX and is anchored on the
first pseudorange of each tracking arc. When a satellite drops out of lock the arc ends, and
the first epoch of the next one carries the **loss-of-lock indicator** so post-processing
knows not to connect the two.

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
    pvt_approximate_year = 2017,   # resolves the GPS week-number rollover for old data
    write_rinex = RinexConfig(;
        directory,
        interval = 1.0u"s",
        marker_name = "ION-RTLSDR",   # not a site identification, so it names the station
        country = "NLD",              # the recording is from the Netherlands
        observer = "GNSSReceiver.jl",
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
GNSSReceiver.rinex_ephemeris
GNSSReceiver.rinex_week
GNSSReceiver.ObsEpochTiming
```
