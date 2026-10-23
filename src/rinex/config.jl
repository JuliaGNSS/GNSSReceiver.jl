# Configuration of the RINEX output: `RinexConfig`, its checks, and the file names and
# observation header it gives.

"""
    RinexConfig(; kwargs...)

Where and how [`receive`](@ref) writes RINEX 3.05 files when its `write_rinex` keyword is
enabled. Pass it as `write_rinex = RinexConfig(...)`; `write_rinex = true` uses the
defaults below.

# Keywords

  - `directory`: where the files are written. A relative `obs_file` or `nav_file` is taken
    relative to it, an absolute one ignores it, and a derived name is placed in it.
  - `obs_file`, `nav_file`: `:convention` to let RINEXParser name the file by the RINEX
    long filename convention (the default, see below), a path to name it yourself, or
    `nothing` to skip that file — `RinexConfig(obs_file = nothing)` writes only the
    navigation (ephemeris) file.
  - `interval`: spacing of the observation epochs, written at integer multiples of it in
    GNSS system time (see [`RinexLogger`](@ref)). It must be longer than a processed chunk.
  - `period`: the intended collection period a derived name reports, as a `Dates.Period` or
    a unitful time — `Day(1)`, `1u"hr"`. Left `nothing` the name says `00U`, not specified,
    which is what the length of a run nobody planned is.
  - `country`: ISO 3166-1 alpha-3 code of the site, for a derived name. Left `nothing` it
    comes from the marker name where that is a site identification, and is `XXX` otherwise.
  - `obs_header`: keywords of RINEXParser's `RinexObsHeader` for the metadata the receiver
    cannot know, written verbatim over the defaults `marker_name = "UNKNOWN"`,
    `marker_type = "GEODETIC"`, `receiver_type = "GNSSReceiver.jl"` and
    `antenna_type = "UNKNOWN"`, e.g. `(; marker_name = "ROOF12DEU", observer = "Me")`. The
    records the receiver fills in itself cannot be given. The marker name also names the
    site of a derived file name: a nine-character site identification `XXXXMRCCC` gives its
    station, monument, receiver and country, any other marker name the station alone.
  - `leap_seconds`: current GPS-UTC leap-second count for the `LEAP SECONDS` header
    record, as a count or a `RINEXParser.LeapSeconds` naming the event and the time system
    it counts in. Left `nothing` it is filled from the navigation message *if* a satellite
    has broadcast it by the time the header is written (GPS sends it on subframe 4 page 18,
    only every 12.5 min, so usually it has not).

# Derived file names

A long filename carries the start time of the file, which this receiver does not know until
its first fix — long after the files have to be open. So a file named by the convention is
written to a temporary `*.rnx.part` file in `directory` and named when it is closed
(RINEXParser's `RinexDirectory`): the observation file after its first epoch, the navigation
file after the observation file of the same run, or after its first ephemeris where there is
none. A rerun of the same recording lands on the same names and replaces the files. A file
with no data to name it by — no epoch, or no ephemeris — is not kept.
"""
Base.@kwdef struct RinexConfig
    directory::String = "."
    obs_file::Union{Nothing,Symbol,String} = :convention
    nav_file::Union{Nothing,Symbol,String} = :convention
    interval::typeof(1.0s) = 1.0s
    period::Union{Nothing,Dates.Period,Unitful.Time} = nothing
    country::Union{Nothing,String} = nothing
    obs_header::NamedTuple = (;)
    leap_seconds::Union{Nothing,Int,LeapSeconds} = nothing
end

# `write_rinex = false` / `true` / `RinexConfig(...)`: normalise the flag form to either
# "off" (`nothing`) or a configuration.
rinex_config(flag::Bool) = flag ? RinexConfig() : nothing
rinex_config(config::RinexConfig) = check_rinex_config(config)

# A mistyped target or an impossible interval says so at the `receive` call rather than
# inside the tracking loop. `:convention` is the only symbol a file target can be.
function check_rinex_config(config::RinexConfig)
    config.interval > 0s || throw(
        ArgumentError(
            "RinexConfig.interval is $(config.interval), but it must be positive",
        ),
    )
    owned = intersect(keys(config.obs_header), RECEIVER_OBS_HEADER_FIELDS)
    isempty(owned) || throw(
        ArgumentError(
            "RinexConfig.obs_header gives $(join(owned, ", ")), which the receiver fills in " *
            "itself" *
            (
                :leap_seconds in owned ?
                "; give leap seconds as RinexConfig.leap_seconds" : ""
            ),
        ),
    )
    unknown = setdiff(keys(config.obs_header), fieldnames(RinexObsHeader))
    isempty(unknown) || throw(
        ArgumentError(
            "RinexConfig.obs_header gives $(join(unknown, ", ")), which RinexObsHeader has " *
            "no field for",
        ),
    )
    for (field, target) in (:obs_file => config.obs_file, :nav_file => config.nav_file)
        target isa Symbol &&
            target !== :convention &&
            throw(
                ArgumentError(
                    "RinexConfig.$field is :$target; a file target is :convention to name " *
                    "the file by the RINEX long filename convention, a path to name it " *
                    "yourself, or nothing to skip it",
                ),
            )
    end
    config
end

# The intended collection period as RINEXParser takes it. `interval` in this configuration
# is a unitful time, so a period may be one too, rather than only the `Dates.Period` the
# name field is written from.
rinex_period(::Nothing) = nothing
rinex_period(period::Dates.Period) = period
rinex_period(period::Unitful.Time) = Second(round(Int, ustrip(s, period)))

# Fields of a derived name this configuration supplies; a `country` left `nothing` comes from
# the marker name. The site of an observation file is its header's marker name, and that of a
# navigation file the one of its session; only a navigation file without an observation file
# to go with it (`with_marker_name`) has it from here.
naming_keywords(config::RinexConfig; with_marker_name = false) =
    with_marker_name ?
    (;
        marker_name = marker_name(config),
        period = rinex_period(config.period),
        country = config.country,
    ) : (; period = rinex_period(config.period), country = config.country)

# Where a file is written: the configured name, taken relative to the configured directory,
# or a `RinexDirectory` that names the file by the long filename convention when it is
# closed. A navigation file takes its start time and site from the observation writer of
# its `session`. A rerun of the same recording is meant to replace its files, so a name that
# is taken is overwritten.
rinex_target(config::RinexConfig, file::String; kwargs...) =
    joinpath(config.directory, file)
rinex_target(config::RinexConfig, ::Symbol; session = nothing, with_marker_name = false) =
    RinexDirectory(
        config.directory;
        session,
        overwrite = true,
        naming_keywords(config; with_marker_name)...,
    )

# The navigation header a writer starts with, until the first record brings the decoded
# corrections. Its satellite system, which the name reports, is settled by the configuration
# before a single ephemeris is decoded (RINEXParser's `nav_satellite_system`) — the same
# choice [`build_nav_records`](@ref) makes.
initial_nav_header(systems) =
    RinexNavHeader(; satellite_system = nav_satellite_system(systems))

# The observation-header metadata a configuration that gives none writes over RINEXParser's
# own defaults (`marker_type = "GEODETIC"`): RINEX placeholders, and this receiver as the
# receiver type. The marker name is here too, since it also names the site of a derived file
# name (`marker_name`).
const DEFAULT_OBS_HEADER = (;
    marker_name = "UNKNOWN",
    receiver_type = "GNSSReceiver.jl",
    antenna_type = "UNKNOWN",
)

# The observation-header records the receiver fills in itself, from its configuration, its
# layout or its data.
const RECEIVER_OBS_HEADER_FIELDS = (
    :obs_codes,
    :phase_shifts,
    :receiver_version,
    :signal_strength_unit,
    :interval,
    :clock_offset_applied,
    :time_of_first_obs,
    :approx_position,
    :leap_seconds,
)

# The configured metadata over the defaults.
obs_header_metadata(config::RinexConfig) = merge(DEFAULT_OBS_HEADER, config.obs_header)

marker_name(config::RinexConfig) = obs_header_metadata(config).marker_name

initial_obs_header(config::RinexConfig, obs_codes, phase_shifts = PhaseShift[]) =
    RinexObsHeader(;
        obs_header_metadata(config)...,
        obs_codes,
        # The phases are aligned with their bands' reference signals; a reference
        # signal's own code is written blank.
        phase_shifts,
        receiver_version = string(pkgversion(@__MODULE__)),
        # The C/N₀ observations are in dB-Hz.
        signal_strength_unit = "DBHZ",
        interval = ustrip(s, config.interval),
        # Epochs are steered onto system time and code and phase propagated there, so the
        # receiver clock offset is applied to all three (see `build_obs_epoch`).
        clock_offset_applied = true,
        leap_seconds = config.leap_seconds,
    )
