"""
    RinexConfig(; kwargs...)

Where and how [`receive`](@ref) writes RINEX 3.05 files when its `write_rinex` keyword is
enabled. Pass it as `write_rinex = RinexConfig(...)`; `write_rinex = true` uses the
defaults below.

# Keywords

  - `obs_file`, `nav_file`: output paths. `nothing` skips that file, so
    `RinexConfig(obs_file = nothing)` writes only the navigation (ephemeris) file.
  - `interval`: spacing of the observation epochs, which are written at integer multiples of
    it in GNSS system time (see [`RinexLogger`](@ref) for the epoch steering). Because that
    grid is in system time and not receiver time, the rate is exactly this interval whatever
    the receiver's clock drift — but it must be comfortably longer than `receive`'s
    `pvt_update_interval`, which is how often an epoch time becomes available (see
    `check_epoch_cadence`).
  - `marker_name`, `marker_type`, `observer`, `agency`, `receiver_number`,
    `receiver_type`, `antenna_number`, `antenna_type`: verbatim observation-header
    metadata. The receiver cannot know these, so they default to RINEX placeholders.
  - `leap_seconds`: current GPS-UTC leap-second count for the `LEAP SECONDS` header
    record. Left `nothing` it is filled from the navigation message *if* a satellite has
    broadcast it by the time the header is written (GPS sends it on subframe 4 page 18,
    only every 12.5 min, so usually it has not).
"""
Base.@kwdef struct RinexConfig
    obs_file::Union{Nothing,String} = "gnss.obs"
    nav_file::Union{Nothing,String} = "gnss.nav"
    interval::typeof(1.0s) = 1.0s
    marker_name::String = "UNKNOWN"
    marker_type::String = "GEODETIC"
    observer::String = ""
    agency::String = ""
    receiver_number::String = ""
    receiver_type::String = "GNSSReceiver.jl"
    antenna_number::String = ""
    antenna_type::String = "UNKNOWN"
    leap_seconds::Union{Nothing,Int} = nothing
end

# `write_rinex = false` / `true` / `RinexConfig(...)`: normalise the flag form to either
# "off" (`nothing`) or a configuration.
rinex_config(flag::Bool) = flag ? RinexConfig() : nothing
rinex_config(config::RinexConfig) = config

# Speed of light and the GPS time-scale origin, taken from the packages that own them
# rather than restated as literals. Every RINEX epoch and ephemeris record is stamped in
# GNSS system time counted from that origin.
const SPEED_OF_LIGHT = ustrip(u"m/s", Unitful.c0)
const GPS_EPOCH = DateTime(1980, 1, 6)
const SECONDS_PER_WEEK = 604800

# Galileo's system time starts at the beginning of GPS week 1024, so a GST week number
# becomes the continuous, GPS-aligned week RINEX asks for by adding this offset. The two
# time scales are within nanoseconds of each other, so a Galileo time of week is directly
# comparable with a GPS one — which is what lets one epoch clock and one week number serve
# a mixed file.
const GALILEO_WEEK_OFFSET = 1024

# The GPS time-scale origin on the atomic (TAI) scale: 1980-01-06 UTC placed on the atomic
# scale by AstroTime's leap-second-aware conversion. Computed on each call rather than
# cached in a global, both because the leap-second table cannot be baked in at precompile
# time and because this runs at most once per PVT solution.
gps_time_origin() = from_utc(get_system_start_time(GPSL1CA()); scale = TAI)

# RINEX satellite-system character per constellation (`get_constellation_id`).
const RINEX_SYSTEM_CHARS =
    Dict(:GPS => 'G', :Galileo => 'E', :GLONASS => 'R', :BeiDou => 'C')

# RINEX 3.05 observation-code suffix (Table A2: band digit plus signal attribute) of every
# signal this receiver can range on, keyed by `get_signal_id`. The four observables of a
# signal are "C"/"L"/"D"/"S" prefixed onto its suffix. The BOC(1,1)-approximation Galileo
# E1 variants are the same broadcast signal as the full-CBOC ones — the modulation
# approximation is a tracking-internal detail — so they share their codes.
const RINEX_SIGNAL_CODES = Dict(
    :GPSL1CA => "1C",
    :GPSL1C_D => "1S",
    :GPSL1C_P => "1L",
    :GPSL2CM => "2S",
    :GPSL2CL => "2L",
    :GPSL5I => "5I",
    :GPSL5Q => "5Q",
    :GalileoE1B => "1B",
    :GalileoE1C => "1C",
    :GalileoE1B_BOC11 => "1B",
    :GalileoE1C_BOC11 => "1C",
    :GalileoE5aI => "5I",
    :GalileoE5aQ => "5Q",
)

# The four RINEX observables this receiver produces per signal, in header order:
# pseudorange, carrier phase, Doppler, signal strength.
rinex_obs_types(code::AbstractString) = ["C" * code, "L" * code, "D" * code, "S" * code]

# Everything the RINEX writer needs to know about one tracking group (one `(signal, PRN)`
# namespace, keyed by `signal_group_key`): which observation types its observables are
# written under, and the constants that turn tracking state into those observables. Where
# each one sits in a satellite's record is RINEXParser's business — `SatObs` derives it from
# the header, so nothing here has to track column positions.
struct RinexSignalLayout
    system::Char
    code::String
    # The signal the group ranges on. Needed to rebuild the transmit time of a satellite from
    # its tracking state, whose code and carrier terms scale with this signal's frequencies.
    signal::AbstractGNSSSignal
    carrier_frequency::Float64
    # Receiver intermediate frequency of this signal's band: the tracked carrier phase
    # advances at `carrier_doppler + interm_freq`, so the IF ramp has to come back out to
    # leave the Doppler-only phase RINEX wants.
    interm_freq::Float64
end

# Metres per cycle of a signal's carrier — what converts between the range and phase
# observables, and between the Doppler and the range rate.
wavelength(layout::RinexSignalLayout) = SPEED_OF_LIGHT / layout.carrier_frequency

# RINEX system character of a signal, or a helpful error for a constellation RINEX does
# not name.
function rinex_system_char(signal)
    constellation = get_constellation_id(signal)
    haskey(RINEX_SYSTEM_CHARS, constellation) || throw(
        ArgumentError(
            "Constellation $constellation has no RINEX satellite-system character; " *
            "RINEX output supports " *
            join(sort!(string.(collect(keys(RINEX_SYSTEM_CHARS)))), ", ") *
            ".",
        ),
    )
    RINEX_SYSTEM_CHARS[constellation]
end

# Build the observation-header layout for every configured system: the header's `obs_types`
# (one entry per constellation, in order of first appearance, each carrying the four
# observables of each of its signals) plus the per-group [`RinexSignalLayout`](@ref) naming
# the observation types each group writes under.
function rinex_layout(band_systems::Tuple, interm_freqs::Tuple)
    obs_types = Pair{Char,Vector{String}}[]
    layouts = Dict{Symbol,RinexSignalLayout}()
    for (systems, interm_freq) in zip(band_systems, interm_freqs)
        for system in systems
            # We range on the ranging signal (the pilot of a `CombinedSignal`), so that is
            # the signal the observables belong to — and its id is the group key that
            # addresses the satellite in `track_state` and in `pvt.sats`.
            signal = ranging_signal(system)
            signal_id = get_signal_id(signal)
            haskey(RINEX_SIGNAL_CODES, signal_id) || throw(
                ArgumentError(
                    "Signal $signal_id has no RINEX 3.05 observation code in this " *
                    "receiver; add it to `GNSSReceiver.RINEX_SIGNAL_CODES` or disable " *
                    "RINEX output.",
                ),
            )
            code = RINEX_SIGNAL_CODES[signal_id]
            system_char = rinex_system_char(signal)
            index = findfirst(p -> first(p) == system_char, obs_types)
            isnothing(index) ? push!(obs_types, system_char => rinex_obs_types(code)) :
            append!(last(obs_types[index]), rinex_obs_types(code))
            layouts[signal_id] = RinexSignalLayout(
                system_char,
                code,
                signal,
                ustrip(Hz, get_center_frequency(signal)),
                ustrip(Hz, interm_freq),
            )
        end
    end
    obs_types, layouts
end

# ─────────────────────────────────────────────────────────────────────────────
# Continuous carrier phase
#
# `Tracking` reports the carrier phase of the local replica wrapped into a single cycle,
# whereas the RINEX phase observable is a continuously accumulated cycle count. Between two
# chunks the replica advances by `(carrier_doppler + interm_freq) · Δt` cycles, which the
# loops' own Doppler estimate predicts to far better than half a cycle over a few
# milliseconds — so the whole-cycle part of the advance is recovered by rounding the
# prediction onto the measured fractional change, and no cycle is ever lost or invented.
# Accumulating (rather than differentiating) is what keeps the observable at the millimetre
# precision that makes a phase observable worth writing at all.

"""
    CarrierPhaseAccumulator(fractional, doppler, clock_rate, runtime, chunk)

Continuous carrier phase of one tracking arc of one satellite, in cycles.

Advanced once per processed chunk by [`advance!`](@ref); `phase` is the accumulated
Doppler-only phase since the arc started. The arc ends when the satellite drops out of lock,
and the next one starts a fresh accumulator — which is exactly what the RINEX loss-of-lock
indicator reports.
"""
mutable struct CarrierPhaseAccumulator
    # Last fractional replica phase [cycles] and carrier Doppler [Hz] — the state the next
    # advance is measured against.
    fractional::Float64
    doppler::Float64
    # Last receiver-clock drift expressed as a carrier-frequency offset [Hz], the common-mode
    # term removed from the phase below.
    clock_rate::Float64
    runtime::Float64
    # Index of the chunk this arc was last advanced on, so an arc that ended (the satellite
    # was not advanced on the current chunk) is recognised without comparing times.
    chunk::Int
    # Continuous Doppler-only phase advance since this arc started [cycles], on the GNSS time
    # scale: the replica advance with the intermediate-frequency ramp and the receiver's own
    # clock drift taken back out.
    phase::Float64
    # `phase` fixes only the *change* of the observable. The RINEX phase ambiguity is
    # arbitrary, so it is anchored on the arc's first pseudorange (`NaN` until then) to
    # keep the written cycle counts in the range a receiver would report.
    anchor::Float64
    # True while the arc's first epoch has not been written yet, so that epoch carries the
    # loss-of-lock indicator.
    restarted::Bool
end

CarrierPhaseAccumulator(fractional, doppler, clock_rate, runtime, chunk) =
    CarrierPhaseAccumulator(fractional, doppler, clock_rate, runtime, chunk, 0.0, NaN, true)

"""
    advance!(acc::CarrierPhaseAccumulator, fractional, doppler, clock_rate, runtime, chunk,
             interm_freq)

Advance `acc` to `chunk`, which ended at `runtime` with fractional replica phase `fractional`
(cycles), carrier Doppler `doppler` (Hz) and receiver-clock drift `clock_rate` (Hz at the
carrier). Returns `acc`.
"""
function advance!(
    acc::CarrierPhaseAccumulator,
    fractional,
    doppler,
    clock_rate,
    runtime,
    chunk,
    interm_freq,
)
    Δt = runtime - acc.runtime
    # Predicted replica advance over the chunk, trapezoidal in the Doppler so that a linearly
    # changing Doppler is integrated exactly.
    predicted = (0.5 * (acc.doppler + doppler) + interm_freq) * Δt
    # Measured fractional change; `round` supplies the whole cycles a wrapped phase cannot
    # carry (and absorbs the wrap itself, so no separate unwrapping step is needed).
    fractional_change = fractional - acc.fractional
    advance = fractional_change + round(predicted - fractional_change)
    # Only the Doppler tracks the changing range, so the IF ramp comes back out — and with it
    # the receiver's own clock drift. The tracking loops measure against the receiver's
    # oscillator, so its frequency offset rides on every satellite's Doppler as a common-mode
    # term; the epoch times and pseudoranges are already free of it (the epochs are steered
    # onto GNSS system time), so leaving it in the phase would make code and phase diverge at
    # that rate — hundreds of metres per second of a difference that should be constant.
    acc.phase += advance - interm_freq * Δt + 0.5 * (acc.clock_rate + clock_rate) * Δt
    acc.fractional = fractional
    acc.doppler = doppler
    acc.clock_rate = clock_rate
    acc.runtime = runtime
    acc.chunk = chunk
    acc
end

# ─────────────────────────────────────────────────────────────────────────────
# Ephemerides
#
# The broadcast navigation message is decoded into GNSSDecoder's per-signal data types;
# RINEXParser consumes the record layout of the RINEX 3.05 navigation files (Tables A14
# and A15). The conversions below are the whole of that mapping: unit scaling, the
# binary-string issue-of-data fields, and the packed health words.

# RINEX navigation records give angles in radians, which is what the decoders already hold:
# GPS and Galileo broadcast them in semicircles, and each decoder applies the semicircle
# scaling as it parses the message (with the ICD's fixed, truncated π rather than `Base.π`,
# so the values reproduce the transmitted numbers exactly). Their field docstrings name the
# *broadcast* unit, not the stored one — the propagator in `PositionVelocityTime` consumes
# `M_0`, `Δn`, `i_0` and friends as radians with no further scaling. So every angle below is
# passed through unchanged; scaling here would multiply it by π a second time.

# The GPS issue-of-data and health fields arrive as binary strings straight from the
# subframes.
parse_binary(value::AbstractString) = parse(Int, value; base = 2)

"""
    rinex_week(decoder; approximate_year) -> Int

Continuous, GPS-aligned week number of a decoder's navigation message, as RINEX navigation
records count weeks.

GPS L1 C/A broadcasts only the 10-bit week, so the 1024-week cycle is resolved against
`approximate_year` — by `PositionVelocityTime`'s `get_week`, so a written ephemeris and the
PVT solution can never disagree about which cycle a recording is in. Galileo broadcasts a
12-bit week counted from the start of GPS week 1024, which becomes GPS-aligned by adding
that offset.
"""
rinex_week(decoder; approximate_year) =
    PositionVelocityTime.get_week(decoder; approximate_year)
rinex_week(
    decoder::GNSSDecoderState{
        <:Union{GNSSDecoder.GalileoE1BData,GNSSDecoder.GalileoE5aData},
    };
    approximate_year,
) = decoder.data.WN + GALILEO_WEEK_OFFSET

# Calendar time of a navigation-record reference epoch from its GPS-aligned week and
# seconds of week. Galileo's system time shares the GPS week alignment, so one conversion
# serves both.
week_seconds_to_datetime(week::Integer, seconds) =
    GPS_EPOCH + Week(week) + Millisecond(round(Int, seconds * 1000))

"""
    rinex_ephemeris(decoder; approximate_year)

Convert one satellite's decoded navigation message into the RINEX 3.05 broadcast ephemeris
record RINEXParser writes, or `nothing` when RINEX 3.05 cannot express that message.

Records exist for GPS LNAV (L1 C/A) and for Galileo I/NAV (E1B) and F/NAV (E5a). The GPS
CNAV and CNAV-2 messages (L5, L2C, L1C) broadcast a quasi-Keplerian ephemeris that RINEX
3.05 has no record layout for — it arrived with RINEX 4 — so those decoders yield `nothing`
even though they position perfectly well.
"""
rinex_ephemeris(decoder; approximate_year) = nothing

function rinex_ephemeris(
    decoder::GNSSDecoderState{<:GNSSDecoder.GPSL1CAData};
    approximate_year,
)
    data = decoder.data
    week = rinex_week(decoder; approximate_year)
    GPSEphemeris(;
        prn = decoder.prn,
        toc = week_seconds_to_datetime(week, data.t_0c),
        af0 = data.a_f0,
        af1 = data.a_f1,
        af2 = data.a_f2,
        iode = parse_binary(data.IODE_Sub_2),
        crs = data.C_rs,
        deltan = data.Δn,
        m0 = data.M_0,
        cuc = data.C_uc,
        e = data.e,
        cus = data.C_us,
        sqrt_a = data.sqrt_A,
        toe = data.t_0e,
        cic = data.C_ic,
        omega0 = data.Ω_0,
        cis = data.C_is,
        i0 = data.i_0,
        crc = data.C_rc,
        omega = data.ω,
        omegadot = data.Ω_dot,
        idot = data.i_dot,
        codes_on_l2 = something(data.codeonl2, 0),
        week = week,
        l2p_data_flag = something(data.l2pcode, false) ? 1.0 : 0.0,
        sv_accuracy = data.ura,
        sv_health = parse_binary(data.sv_health),
        tgd = data.T_GD,
        iodc = parse_binary(data.IODC),
        transmission_time = data.TOW,
        # The broadcast curve-fit flag only distinguishes the nominal four-hour interval
        # from a longer one; six hours is the extended value the constellation uses
        # (IS-GPS-200N Table 20-XII).
        fit_interval = something(data.fit_interval, false) ? 6.0 : 4.0,
    )
end

# Galileo signal-in-space accuracy: the broadcast SISA index maps onto metres in four
# linear segments (Galileo OS SIS ICD Table 92). Index 255 means "no accuracy prediction
# available" (NAPA), and the spare range above 125 is equally meaningless; RINEX reports
# both as a negative accuracy.
function sisa_metres(index::Integer)
    index <= 49 && return 0.01 * index
    index <= 74 && return 0.5 + 0.02 * (index - 50)
    index <= 99 && return 1.0 + 0.04 * (index - 75)
    index <= 125 && return 2.0 + 0.16 * (index - 100)
    -1.0
end
sisa_metres(::Nothing) = -1.0

# The broadcast health and data-validity fields as the plain integers the RINEX bit fields
# are packed from. A signal whose status a message does not carry is reported healthy and
# valid, which is what a RINEX file from a receiver tracking only that one band says too.
status_bits(status, healthy) = Int(something(status, healthy))
signal_health_bits(status) = status_bits(status, GNSSDecoder.signal_ok)
data_validity_bits(status) = status_bits(status, GNSSDecoder.navigation_data_valid)

# The two bit fields of a Galileo navigation record (RINEX 3.05 Table A15), assembled by
# RINEXParser from the per-signal statuses. An I/NAV (E1B) message carries the E1B and E5b
# health and the E5b·E1 clock parameters; an F/NAV (E5a) message the E5a health and the
# E5a·E1 clock parameters. RINEXParser keys a record's identity on `data_sources`, so the
# I/NAV and F/NAV messages of one satellite stay the separate records RINEX requires.
galileo_health(data::GNSSDecoder.GalileoE1BData) = galileo_sv_health(;
    e1b_dvs = data_validity_bits(data.data_validity_status_e1b),
    e1b_hs = signal_health_bits(data.signal_health_e1b),
    e5b_dvs = data_validity_bits(data.data_validity_status_e5b),
    e5b_hs = signal_health_bits(data.signal_health_e5b),
)
galileo_health(data::GNSSDecoder.GalileoE5aData) = galileo_sv_health(;
    e5a_dvs = data_validity_bits(data.data_validity_status_e5a),
    e5a_hs = signal_health_bits(data.signal_health_e5a),
)

galileo_sources(::GNSSDecoder.GalileoE1BData) =
    galileo_data_sources(; inav_e1b = true, clock_e5b_e1 = true)
galileo_sources(::GNSSDecoder.GalileoE5aData) =
    galileo_data_sources(; fnav_e5a = true, clock_e5a_e1 = true)

# I/NAV broadcasts the group delay of both Galileo band pairs, F/NAV only the E5a·E1 one;
# RINEX leaves the term a message does not carry at zero.
galileo_bgd_e5b_e1(data::GNSSDecoder.GalileoE1BData) = data.broadcast_group_delay_e1_e5b
galileo_bgd_e5b_e1(::GNSSDecoder.GalileoE5aData) = 0.0

galileo_sisa(data::GNSSDecoder.GalileoE1BData) = sisa_metres(data.SISA_e1_e5b)
galileo_sisa(data::GNSSDecoder.GalileoE5aData) = sisa_metres(data.SISA_e1_e5a)

function rinex_ephemeris(
    decoder::GNSSDecoderState{
        <:Union{GNSSDecoder.GalileoE1BData,GNSSDecoder.GalileoE5aData},
    };
    approximate_year,
)
    data = decoder.data
    week = rinex_week(decoder; approximate_year)
    GalileoEphemeris(;
        prn = decoder.prn,
        toc = week_seconds_to_datetime(week, data.t_0c),
        af0 = data.a_f0,
        af1 = data.a_f1,
        af2 = data.a_f2,
        iodnav = data.IOD_nav1,
        crs = data.C_rs,
        deltan = data.Δn,
        m0 = data.M_0,
        cuc = data.C_uc,
        e = data.e,
        cus = data.C_us,
        sqrt_a = data.sqrt_A,
        toe = data.t_0e,
        cic = data.C_ic,
        omega0 = data.Ω_0,
        cis = data.C_is,
        i0 = data.i_0,
        crc = data.C_rc,
        omega = data.ω,
        omegadot = data.Ω_dot,
        idot = data.i_dot,
        data_sources = galileo_sources(data),
        week = week,
        sisa = galileo_sisa(data),
        sv_health = galileo_health(data),
        bgd_e5a_e1 = data.broadcast_group_delay_e1_e5a,
        bgd_e5b_e1 = galileo_bgd_e5b_e1(data),
        transmission_time = data.TOW,
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# Navigation-file header
#
# The ionosphere, time-system and leap-second records are broadcast far less often than the
# ephemerides (GPS puts them on subframe 4 page 18, once every 12.5 min), so they are
# collected from whichever satellite has them and installed into the header for as long as
# RINEXParser allows it — up to the first ephemeris written.

# Klobuchar (GPS) and NeQuick (Galileo) ionosphere coefficients of one decoder, appended to
# `corrections` when the satellite has broadcast them.
nav_ionospheric_corrections!(corrections, data) = corrections
function nav_ionospheric_corrections!(corrections, data::GNSSDecoder.GPSL1CAData)
    isnothing(data.α_0) || push!(
        corrections,
        IonosphericCorrection("GPSA", (data.α_0, data.α_1, data.α_2, data.α_3)),
    )
    isnothing(data.β_0) || push!(
        corrections,
        IonosphericCorrection("GPSB", (data.β_0, data.β_1, data.β_2, data.β_3)),
    )
    corrections
end
function nav_ionospheric_corrections!(corrections, data::GNSSDecoder.AbstractGalileoData)
    # Galileo's NeQuick coefficients occupy a Klobuchar-shaped record whose fourth slot is
    # unused.
    isnothing(data.a_i0) || push!(
        corrections,
        IonosphericCorrection("GAL", (data.a_i0, data.a_i1, data.a_i2, 0.0)),
    )
    corrections
end

# Time-system corrections of one decoder: each constellation's offset to UTC, plus the
# broadcast Galileo-to-GPS offset (GGTO) once Galileo has sent it.
nav_time_system_corrections!(corrections, data) = corrections
function nav_time_system_corrections!(corrections, data::GNSSDecoder.GPSL1CAData)
    isnothing(data.A_0) || push!(
        corrections,
        TimeSystemCorrection("GPUT", data.A_0, data.A_1, data.t_ot, data.WN_t),
    )
    corrections
end
function nav_time_system_corrections!(corrections, data::GNSSDecoder.AbstractGalileoData)
    isnothing(data.A_0_utc) || push!(
        corrections,
        TimeSystemCorrection("GAUT", data.A_0_utc, data.A_1_utc, data.t_0t, data.WN_0t),
    )
    isnothing(data.A_0G) || push!(
        corrections,
        TimeSystemCorrection("GPGA", data.A_0G, data.A_1G, data.t_0G, data.WN_0G),
    )
    corrections
end

# Broadcast leap-second count, as the four-field form RINEX prefers (current count, count
# after the next event, and the week and day the event falls on) whenever the satellite
# sent the whole record. GPS and Galileo name these fields alike.
nav_leap_seconds(data) = nothing
function nav_leap_seconds(
    data::Union{GNSSDecoder.GPSL1CAData,GNSSDecoder.AbstractGalileoData},
)
    isnothing(data.Δt_LS) && return nothing
    any(isnothing, (data.Δt_LSF, data.WN_LSF, data.DN)) && return Int(data.Δt_LS)
    (Int(data.Δt_LS), Int(data.Δt_LSF), Int(data.WN_LSF), Int(data.DN))
end

# ─────────────────────────────────────────────────────────────────────────────
# The record handed over to the writer task

# One RINEX update: a finished observation epoch (`nothing` before the first fix), the
# ephemerides to offer the navigation writer (which drops the ones it has already written),
# and the header material that only exists once there is a fix. Immutable, so the writer
# task owns it outright while the tracking loop moves on.
struct RinexRecord
    epoch::Union{Nothing,ObsEpoch}
    ephemerides::Vector{Union{GPSEphemeris,GalileoEphemeris}}
    nav_header::RinexNavHeader
    approx_position::NTuple{3,Float64}
    leap_seconds::Union{Nothing,Int,NTuple{4,Int}}
end

# ─────────────────────────────────────────────────────────────────────────────
# The logger

"""
    RinexLogger(config, band_systems, interm_freqs; approximate_year)

Turns the receiver state of each processed chunk into RINEX 3.05 output. [`receive`](@ref)
builds one when its `write_rinex` keyword is enabled and calls [`log_rinex!`](@ref) once
per chunk; the files are written by a task this constructor spawns, so no file I/O happens
on the tracking loop. `close` stops it and blocks until the files are complete.

# Epoch times and pseudoranges

RINEX epochs are stamped in GNSS system time, which a software receiver only knows once the
PVT solution has resolved its clock offset — so observations start at the first fix. Each
epoch is then *steered* onto an integer multiple of `config.interval`: the observables are
propagated from the measurement instant to that nominal time with their own Doppler (tens
of milliseconds at most, over which the range rate is constant to well under a millimetre).
That is why no receiver clock offset is reported — this receiver's clock *is* GNSS system
time by construction, rather than a hardware clock that has to be steered onto it.

Because that grid is in system time rather than receiver time, the written **rate is exactly
`config.interval` regardless of the receiver's clock drift**: the drift decides only which
chunk lands closest to each grid point, never how far apart the grid points are. See
`check_epoch_cadence` for the one condition this needs.

Pseudoranges are formed against the steered epoch from the satellite transmit times of the
PVT solution, so they are exactly the ranges the reported fix was computed from. Only
satellites that contributed to that fix carry observations: without a decoded time of week
a satellite has no transmit time, and hence no pseudorange to report.

Ephemerides do not need a fix and are offered to the navigation writer on their own
`config.interval` cadence, so a run that never reaches four satellites still produces a
navigation file.
"""
mutable struct RinexLogger
    config::RinexConfig
    # The observation header of the file, shared with the writer that owns it. This side only
    # ever reads its `obs_types` — which never change — to let `SatObs` place each observable
    # in its own column; the writer completes the fields the data supplies late. So the two
    # tasks touch disjoint fields of it, and neither races the other.
    header::Union{Nothing,RinexObsHeader}
    layouts::Dict{Symbol,RinexSignalLayout}
    # Constellations the run can produce ephemerides for. Taken from the configuration
    # rather than from what has been decoded so far: it pins the navigation header's
    # satellite system, and a header written before a second constellation showed up would
    # make RINEXParser reject that constellation's records.
    systems::Vector{Char}
    approximate_year::Int
    channel::Channel{RinexRecord}
    task::Task
    # Continuous carrier phase per `(group key, PRN)`, i.e. per tracked signal of a
    # satellite. An entry lives as long as its tracking arc; a satellite that drops out of
    # lock loses its entry and comes back on a fresh, loss-of-lock-flagged arc.
    accumulators::Dict{Tuple{Symbol,Int},CarrierPhaseAccumulator}
    # Week and epoch-grid index of the last written epoch, so each nominal epoch is written
    # exactly once.
    last_epoch_index::Union{Nothing,Tuple{Int,Int}}
    # Epoch of the last PVT solution, to spot the chunk a fresh fix was computed on: only
    # then do its transmit times refer to the same instant as the carrier phases.
    last_pvt_time::Union{Nothing,TAIEpoch{Float64}}
    last_nav_runtime::Float64
    # Chunks processed so far, which identifies the arcs still alive (see
    # [`CarrierPhaseAccumulator`](@ref)).
    chunk::Int
    # Signals whose navigation message RINEX 3.05 cannot express, warned about once each.
    warned_signals::Set{Symbol}
end

# The epoch grid lives in GNSS system time, but the instants that can land on it are the
# chunks a fresh clock solution arrives on — one every `pvt_update_interval` of receiver
# time. As long as those candidates are comfortably closer together than the epoch interval,
# exactly one of them rounds onto each grid point and the written rate is exactly
# `interval`, whatever the receiver clock is doing. If the two are equal, the receiver's
# clock drift slowly slides the candidates against the grid until a grid point gets two
# candidates or none — one duplicate-or-gap per `1/drift` epochs, so ~37 h at 0.7 ppm but
# every ~17 min at 100 ppm. A post-processing tool that insists on a uniform rate would trip
# over that, so say so rather than let it be discovered downstream.
function check_epoch_cadence(interval, pvt_update_interval)
    isnothing(pvt_update_interval) && return nothing
    interval >= 2 * pvt_update_interval && return nothing
    @warn "A RINEX epoch interval of $interval is not comfortably longer than the PVT " *
          "update interval of $pvt_update_interval, which is how often an epoch time " *
          "becomes available. Epochs stay on exact multiples of the interval, but the " *
          "receiver's clock drift can make one occasionally repeat or be skipped, so the " *
          "rate is no longer strictly uniform. Use an interval of at least " *
          "$(2 * pvt_update_interval), or lower `pvt_update_interval`."
    nothing
end

function RinexLogger(
    config::RinexConfig,
    band_systems::Tuple,
    interm_freqs::Tuple;
    approximate_year::Integer = year(now(UTC)),
    pvt_update_interval = nothing,
)
    obs_types, layouts = rinex_layout(band_systems, interm_freqs)
    check_epoch_cadence(config.interval, pvt_update_interval)
    # The writers are opened here rather than inside the task so that an unwritable path
    # fails at the `receive` call, and so that a run producing no records at all still
    # leaves valid, header-only files.
    header = isnothing(config.obs_file) ? nothing : initial_obs_header(config, obs_types)
    obs_writer = isnothing(header) ? nothing : RinexObsWriter(config.obs_file, header)
    nav_writer = isnothing(config.nav_file) ? nothing : RinexNavWriter(config.nav_file)
    # Buffered, so a slow disk cannot rendezvous with — and stall — the tracking loop.
    channel = Channel{RinexRecord}(16)
    task = spawn_rinex_writer(channel, obs_writer, nav_writer)
    RinexLogger(
        config,
        header,
        layouts,
        unique(layout.system for layout in values(layouts)),
        approximate_year,
        channel,
        task,
        Dict{Tuple{Symbol,Int},CarrierPhaseAccumulator}(),
        nothing,
        nothing,
        -Inf,
        0,
        Set{Symbol}(),
    )
end

"""
    close(logger::RinexLogger)

Stop the RINEX output and block until the files are complete on disk. Closing also flushes
the header of a file that never saw a record, so even an aborted run leaves valid RINEX. A
writer that failed has already reported it, so this does not raise it again.
"""
function Base.close(logger::RinexLogger)
    close(logger.channel)
    try
        wait(logger.task)
    catch e
        e isa TaskFailedException || rethrow()
    end
    nothing
end

# Total GNSS system time of a PVT epoch, in seconds since the GPS time-scale origin. The
# solution's `time` is an atomic (TAI) epoch, so the difference against that origin is a
# leap-second-free count of system-time seconds.
#
# Only ever used to recover the *week*: at present-day week numbers this count is around
# 1.2e9, where a `Float64` resolves no better than 0.2 µs — 70 m of pseudorange. Everything
# that needs precision therefore works in times of week (below 6.05e5, resolved to 0.02 ns),
# which is also how the PVT solution reports its satellite transmit times.
gps_seconds(time::TAIEpoch) = AstroTime.value(AstroTime.seconds(time - gps_time_origin()))

# Calendar time plus sub-millisecond remainder of a RINEX epoch record, from a GPS week and
# a time of week. Both parts stay exact: the week contributes whole days, and the time of
# week is small enough to split without losing its fraction.
function epoch_datetime(week::Integer, time_of_week)
    whole_seconds = floor(Int, time_of_week)
    fraction = time_of_week - whole_seconds
    whole_milliseconds = floor(Int, fraction * 1000)
    (
        GPS_EPOCH + Week(week) + Second(whole_seconds) + Millisecond(whole_milliseconds),
        fraction - whole_milliseconds / 1000,
    )
end

# Difference of two times of week, resolved across the week boundary. A receive and a
# transmit instant lie a fraction of a second apart, so a difference approaching a whole
# week can only mean the week rolled over between them.
function week_difference(later, earlier)
    difference = later - earlier
    difference > SECONDS_PER_WEEK / 2 ? difference - SECONDS_PER_WEEK :
    difference < -SECONDS_PER_WEEK / 2 ? difference + SECONDS_PER_WEEK : difference
end

"""
    uncorrected_transmit_time(decoder, signal, sat_state) -> Float64

Time of week the satellite's own clock read when it transmitted the signal now arriving,
reconstructed from the decoded time of week and the tracking loops' code and carrier phase.

This is the *raw* transmit time, and it is what a RINEX pseudorange has to be measured
against. The PVT solution instead reports the **corrected** one — satellite clock
polynomial, relativistic term and group delay already removed — because that is what a
position solve needs. Handing a pre-corrected range to a RINEX consumer would have it
subtract the broadcast clock correction a second time, biasing every satellite by its own
clock error: tens of kilometres, differing satellite to satellite, so the fix collapses
rather than merely shifting.
"""
uncorrected_transmit_time(decoder, signal, sat_state) =
    PositionVelocityTime.calc_uncorrected_time(
        PositionVelocityTime.SatelliteState(decoder, signal, sat_state),
    )

# RINEX signal-strength indicator: the carrier-to-noise density ratio mapped onto the
# spec's 1-9 scale in 6 dB-Hz steps (1 for 12 dB-Hz or worse, 9 for 54 dB-Hz or better).
# A CN0 estimate that has not converged yet is not a strength, so it reports the spec's
# "unknown" value 0.
signal_strength_indicator(cn0) = isfinite(cn0) ? clamp(floor(Int, cn0 / 6), 1, 9) : 0

"""
    log_rinex!(logger::RinexLogger, receiver_state) -> Nothing

Advance the RINEX output by one processed chunk: keep every tracked satellite's continuous
carrier phase up to date and hand the writer task a finished observation epoch whenever a
fresh PVT solution crosses the next nominal epoch, plus the current ephemerides.

The carrier phase has to be accumulated on *every* chunk — that is what keeps its
whole-cycle count exact, see [`CarrierPhaseAccumulator`](@ref) — so this is called for all
chunks, not only the ones that produce an epoch.
"""
function log_rinex!(logger::RinexLogger, receiver_state)
    runtime = ustrip(s, receiver_state.runtime)
    logger.chunk += 1
    update_carrier_phases!(logger, receiver_state, runtime)

    timing = obs_epoch_timing(logger, receiver_state)
    # Ephemerides need no fix, so they run on their own cadence off the signal runtime.
    nav_due =
        !isnothing(logger.config.nav_file) &&
        runtime - logger.last_nav_runtime >= ustrip(s, logger.config.interval)
    (isnothing(timing) && !nav_due) && return nothing
    logger.last_nav_runtime = runtime

    epoch = isnothing(timing) ? nothing : build_obs_epoch(logger, receiver_state, timing)
    ephemerides, nav_header = build_nav_records(logger, receiver_state)
    position = receiver_state.pvt.position
    record = RinexRecord(
        epoch,
        ephemerides,
        nav_header,
        (position[1], position[2], position[3]),
        something(logger.config.leap_seconds, nav_header.leap_seconds, Some(nothing)),
    )
    # A closed channel means the writer task is gone (it only ends early on a write failure,
    # which it has already reported): drop the record rather than take the run down with it.
    try
        put!(logger.channel, record)
    catch e
        e isa InvalidStateException || rethrow()
    end
    nothing
end

"""
    ObsEpochTiming

When one observation epoch happens, in the GNSS system time the PVT solution reports.

  - `week`, `time_of_week`: the steered (nominal) epoch the record is stamped with.
  - `reference_time`: the time of week the PVT solution referenced its transmit times to,
    i.e. the latest of them. Pseudoranges are differences against this, so they never touch
    an absolute seconds count.
  - `clock_offset`: the receiver clock bias of that solution, in seconds. The measurement
    instant is `reference_time - clock_offset`.
  - `propagation`: how far the observables have to be carried, with their own Doppler, to
    land on the steered epoch — at most half an emission interval.
"""
struct ObsEpochTiming
    week::Int
    time_of_week::Float64
    reference_time::Float64
    clock_offset::Float64
    propagation::Float64
end

# The timing of this chunk's observation epoch, or `nothing` when it is not due to produce
# one.
function obs_epoch_timing(logger::RinexLogger, receiver_state)
    isnothing(logger.config.obs_file) && return nothing
    pvt = receiver_state.pvt
    # An epoch needs a clock solution computed from *this* chunk's measurements; a stale one
    # would time-stamp carrier phases that have since moved on.
    isnothing(pvt.time) && return nothing
    pvt.time == logger.last_pvt_time && return nothing
    logger.last_pvt_time = pvt.time
    isempty(pvt.sats) && return nothing

    # The solution's own reference: the latest transmit time it saw, and the clock bias it
    # solved for. Together they give the measurement instant as a time of week, which is
    # exactly how `calc_pvt` derived the epoch it reports.
    reference_time = maximum(info.time for info in pvt.sats)
    clock_offset = ustrip(m, pvt.time_correction) / SPEED_OF_LIGHT
    receive_time = reference_time - clock_offset

    interval = ustrip(s, logger.config.interval)
    index = round(Int, receive_time / interval)
    # The week comes from the reported epoch, which needs only coarse precision for that:
    # the absolute count minus the time of week is a whole number of weeks by construction.
    week = round(Int, (gps_seconds(pvt.time) - receive_time) / SECONDS_PER_WEEK)
    time_of_week = index * interval
    # An epoch that rounds up past the end of the week belongs to the next one.
    if time_of_week >= SECONDS_PER_WEEK
        week += 1
        index = 0
        time_of_week = 0.0
    end

    epoch_index = (week, index)
    epoch_index == logger.last_epoch_index && return nothing
    logger.last_epoch_index = epoch_index

    ObsEpochTiming(
        week,
        time_of_week,
        reference_time,
        clock_offset,
        week_difference(time_of_week, receive_time),
    )
end

# Keep one accumulator per tracked, in-lock satellite up to date, and forget the arcs that
# ended so their next appearance starts a fresh, loss-of-lock-flagged one.
function update_carrier_phases!(logger::RinexLogger, receiver_state, runtime)
    track_state = receiver_state.track_state
    drift = receiver_state.pvt.relative_clock_drift
    for (group_key, sat_states) in pairs(receiver_state.receiver_sat_states)
        layout = get(logger.layouts, group_key, nothing)
        isnothing(layout) && continue
        # The receiver's fractional clock-frequency error as a carrier-frequency offset: the
        # common-mode Doppler its oscillator adds to every satellite of this band.
        clock_rate = drift * layout.carrier_frequency
        for sat_state in get_sat_states(track_state, group_key)
            prn = get_prn(sat_state)
            (haskey(sat_states, prn) && is_in_lock(sat_states[prn])) || continue
            key = (group_key, prn)
            # `get_carrier_phase` reports the replica phase in radians, wrapped into one
            # cycle; the accumulator works in cycles, as RINEX does.
            fractional = get_carrier_phase(sat_state) / 2π
            doppler = ustrip(Hz, get_carrier_doppler(sat_state))
            accumulator = get(logger.accumulators, key, nothing)
            logger.accumulators[key] =
                isnothing(accumulator) ?
                CarrierPhaseAccumulator(
                    fractional,
                    doppler,
                    clock_rate,
                    runtime,
                    logger.chunk,
                ) :
                advance!(
                    accumulator,
                    fractional,
                    doppler,
                    clock_rate,
                    runtime,
                    logger.chunk,
                    layout.interm_freq,
                )
        end
    end
    # Anything not advanced on this chunk lost lock, so its arc ended.
    filter!(pair -> last(pair).chunk == logger.chunk, logger.accumulators)
    nothing
end

# Build one observation epoch, steered onto `timing`'s nominal epoch, from the satellites of
# the PVT solution.
function build_obs_epoch(logger::RinexLogger, receiver_state, timing::ObsEpochTiming)
    track_state = receiver_state.track_state
    propagation = timing.propagation
    # Group the per-signal observations by satellite: a satellite tracked on several signals
    # contributes one RINEX record carrying each signal's observation types. `SatObs` takes
    # their placement from the header, so they are collected by descriptor and in any order.
    satellites = Dict{Tuple{Char,Int},Vector{Pair{String,ObsValue}}}()
    # The satellites of the PVT solution are exactly those with a decoded time of week, and
    # so the only ones a transmit time — and hence a pseudorange — exists for.
    for (group_key, prn) in keys(receiver_state.pvt.sats)
        layout = get(logger.layouts, group_key, nothing)
        isnothing(layout) && continue
        accumulator = get(logger.accumulators, (group_key, prn), nothing)
        isnothing(accumulator) && continue
        sat_state = get_sat_state(track_state, group_key, prn)
        decoder = receiver_state.receiver_sat_states[group_key][prn].decoder
        # Doppler on the GNSS time scale, i.e. with the receiver oscillator's common-mode
        # offset removed — the same correction the accumulated phase carries, and the rate
        # that matches the epochs and pseudoranges (see `advance!`). Reporting the raw
        # tracking Doppler here would contradict both.
        doppler =
            ustrip(Hz, get_carrier_doppler(sat_state)) +
            receiver_state.pvt.relative_clock_drift * layout.carrier_frequency
        λ = wavelength(layout)
        # `estimate_cn0` reports a logarithmic quantity in dB-Hz, which is already the unit
        # of the RINEX signal-strength observable; stripping it yields that number. It is not
        # always finite — an estimator that has not converged reports an infinite ratio —
        # which RINEXParser writes as the blank field RINEX reads as "no measurement".
        cn0 = ustrip(estimate_cn0(sat_state, RANGING_SIGNAL_INDEX))

        # Pseudorange against the steered epoch. Formed as a difference of nearby times of
        # week and the (small) clock offset, never as a difference of absolute seconds — the
        # cancellation there would quantize the range to tens of metres. The range rate is
        # `-λ·f_d`, which carries the range from the measurement instant to the nominal one.
        transmit_time = uncorrected_transmit_time(decoder, layout.signal, sat_state)
        travel_time =
            week_difference(timing.reference_time, transmit_time) - timing.clock_offset
        range = travel_time * SPEED_OF_LIGHT

        # Carrier phase in whole cycles. RINEX has it change in the same sense as the range,
        # so its rate is the negated Doppler; the arbitrary ambiguity is anchored on the
        # arc's first range so the written cycle counts look like a receiver's. Anchored on
        # the range at the *measurement* instant, before the propagation below, so that
        # propagating each observable once leaves the two consistent.
        isnan(accumulator.anchor) && (accumulator.anchor = range / λ + accumulator.phase)

        # Carry both onto the steered epoch, each at its own rate — the range rate is `-λ·f_d`
        # and the phase rate `-f_d` — which keeps code and phase agreeing to a fraction of a
        # cycle rather than drifting apart by metres.
        pseudorange = range - λ * doppler * propagation
        carrier_phase = accumulator.anchor - accumulator.phase - doppler * propagation

        observations = get!(() -> Pair{String,ObsValue}[], satellites, (layout.system, prn))
        ssi = signal_strength_indicator(cn0)
        # The loss-of-lock indicator flags the first epoch of an arc; after it the phase is
        # continuous with the previous epoch's.
        lli = accumulator.restarted ? 1 : 0
        accumulator.restarted = false
        code = layout.code
        push!(observations, "C" * code => ObsValue(pseudorange; ssi))
        push!(observations, "L" * code => ObsValue(carrier_phase; lli, ssi))
        push!(observations, "D" * code => ObsValue(doppler))
        push!(observations, "S" * code => ObsValue(cn0))
    end
    isempty(satellites) && return nothing

    time, fractional_second = epoch_datetime(timing.week, timing.time_of_week)
    # Deterministic satellite order within the epoch, as RINEX files conventionally carry.
    sats = [
        SatObs(logger.header, system, prn, satellites[(system, prn)]) for
        (system, prn) in sort!(collect(keys(satellites)))
    ]
    ObsEpoch(time, sats; fractional_second)
end

# Every tracked satellite's current ephemeris, plus the navigation header assembled from
# whatever ionosphere, time-system and leap-second records have been decoded so far.
# RINEXParser deduplicates the ephemerides, so offering all of them every time is the
# documented way to write each one exactly once.
function build_nav_records(logger::RinexLogger, receiver_state)
    ephemerides = Vector{Union{GPSEphemeris,GalileoEphemeris}}()
    ionospheric_corrections = IonosphericCorrection[]
    time_system_corrections = TimeSystemCorrection[]
    leap_seconds = nothing
    if !isnothing(logger.config.nav_file)
        for (group_key, sat_states) in pairs(receiver_state.receiver_sat_states)
            haskey(logger.layouts, group_key) || continue
            for sat_state in sat_states
                decoder = sat_state.decoder
                # A partially decoded message would silently write zeros into the record.
                is_decoding_completed_for_positioning(decoder) || continue
                ephemeris =
                    rinex_ephemeris(decoder; approximate_year = logger.approximate_year)
                if isnothing(ephemeris)
                    warn_unsupported_ephemeris(logger, group_key)
                    continue
                end
                push!(ephemerides, ephemeris)
                nav_ionospheric_corrections!(ionospheric_corrections, decoder.data)
                nav_time_system_corrections!(time_system_corrections, decoder.data)
                leap_seconds =
                    something(leap_seconds, nav_leap_seconds(decoder.data), Some(nothing))
            end
        end
    end
    header = RinexNavHeader(;
        # A file that can only ever carry one constellation names it; anything else is left
        # unpinned so RINEXParser marks the file as mixed.
        satellite_system = length(logger.systems) == 1 ? only(logger.systems) : nothing,
        ionospheric_corrections = unique(ionospheric_corrections),
        time_system_corrections = unique(time_system_corrections),
        leap_seconds,
    )
    ephemerides, header
end

function warn_unsupported_ephemeris(logger::RinexLogger, group_key::Symbol)
    group_key in logger.warned_signals && return nothing
    push!(logger.warned_signals, group_key)
    @warn "RINEX 3.05 has no navigation record for the message decoded on $group_key, so " *
          "its ephemerides are not written. RINEX 3.05 covers GPS LNAV (L1 C/A) and " *
          "Galileo I/NAV (E1B) and F/NAV (E5a); the GPS CNAV and CNAV-2 messages need " *
          "RINEX 4. Observations of $group_key are unaffected."
    nothing
end

# ─────────────────────────────────────────────────────────────────────────────
# The writer task

# Drain the record channel into the RINEX files. All file I/O — and all of RINEXParser's
# state — lives here, off the tracking loop.
function spawn_rinex_writer(channel::Channel{RinexRecord}, obs_writer, nav_writer)
    Base.errormonitor(
        Threads.@spawn try
            for record in channel
                if !isnothing(nav_writer)
                    # The header may still be replaced until the first ephemeris is written,
                    # which is how the ionosphere and UTC records — broadcast minutes after
                    # the first ephemeris — reach the file at all.
                    nav_writer.header_written || (nav_writer.header = record.nav_header)
                    foreach(eph -> write_ephemeris!(nav_writer, eph), record.ephemerides)
                end
                (isnothing(obs_writer) || isnothing(record.epoch)) && continue
                # The receiver's own position and the broadcast leap seconds are known no
                # earlier than the first epoch — which is the epoch that writes the header,
                # so it can still take them.
                if !obs_writer.header_written
                    obs_writer.header.approx_position = record.approx_position
                    obs_writer.header.leap_seconds = record.leap_seconds
                end
                write_epoch!(obs_writer, record.epoch)
            end
        finally
            # Close the channel as well: on a write failure the producer would otherwise
            # block on a full channel forever, stalling the tracking loop.
            close(channel)
            foreach(close, filter(!isnothing, (obs_writer, nav_writer)))
        end
    )
end

initial_obs_header(config::RinexConfig, obs_types) = RinexObsHeader(;
    obs_types,
    marker_name = config.marker_name,
    marker_type = config.marker_type,
    observer = config.observer,
    agency = config.agency,
    receiver_number = config.receiver_number,
    receiver_type = config.receiver_type,
    receiver_version = string(pkgversion(@__MODULE__)),
    antenna_number = config.antenna_number,
    antenna_type = config.antenna_type,
    interval = ustrip(s, config.interval),
    leap_seconds = config.leap_seconds,
)
