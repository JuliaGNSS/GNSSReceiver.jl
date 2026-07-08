using UnicodePlots, Term
import REPL

struct GUIData{S<:SatelliteDataOfInterest}
    sat_data::Dictionary{Tuple{Symbol,Int},S}
    pvt::PVTSolution
    runtime::typeof(1.0s)
    # Whether this data carries a *new* PVT solution.
    pvt_fresh::Bool
end

# Satellite labels follow the RINEX-3 convention: the satellite is identified by
# its system letter + zero-padded PRN (`G30`, `E24`, `R05`, `C21` — the same code
# used in RINEX files, IGS products and most receivers), and the signal's frequency
# band is appended after a space (`G30 L5`). This keeps the satellite ID
# unambiguous — `G05` is GPS PRN 5, never "GPS L5" — and drops the I/Q/CA signal
# component for compactness. Band tokens use the familiar frequency-band names
# (GPS L1/L2/L5, Galileo E1/E5a); the modernized GPS L1C civil signal keeps "L1C"
# so it is not confused with L1 C/A ("L1"). The band token is right-padded to the
# widest name (3 chars: "E5a"/"L1C") so every label is the same width and the bars
# line up in a column regardless of band.
const CONSTELLATION_LETTERS =
    Dict(:GPS => "G", :Galileo => "E", :GLONASS => "R", :BeiDou => "C", :Other => "?")

const BAND_ABBREVIATIONS = Dict(
    :GPSL1CA => "L1",
    :GPSL1C_D => "L1C",
    :GPSL1C_P => "L1C",
    :GPSL2CM => "L2",
    :GPSL2CL => "L2",
    :GPSL5I => "L5",
    :GPSL5Q => "L5",
    :GalileoE1B => "E1",
    :GalileoE1C => "E1",
    # BOC(1,1)-approximation E1 variants share the E1 band, so they get the same
    # abbreviation as the full-CBOC E1B/E1C — the modulation approximation is a
    # tracking-internal detail, not a distinct band to the user.
    :GalileoE1B_BOC11 => "E1",
    :GalileoE1C_BOC11 => "E1",
    :GalileoE5aI => "E5a",
    :GalileoE5aQ => "E5a",
)

function sat_label(system_key::Symbol, prn::Integer)
    sys = get(CONSTELLATION_LETTERS, constellation_of(system_key), "?")
    band = get(BAND_ABBREVIATIONS, system_key, string(system_key))
    string(sys, lpad(prn, 2, '0'), " ", rpad(band, 3))
end

# Display order for the CN0 bar chart: by constellation (GPS, then Galileo, then the
# rest), then PRN, then band (ascending frequency within a constellation). Unlisted
# constellations/signals sort last (rank 99) but keep a stable order among themselves.
const CONSTELLATION_ORDER =
    Dict(:GPS => 1, :Galileo => 2, :GLONASS => 3, :BeiDou => 4, :Other => 5)

const BAND_ORDER = Dict(
    :GPSL1CA => 1,
    :GPSL1C_D => 2,
    :GPSL1C_P => 3,
    :GPSL2CM => 4,
    :GPSL2CL => 5,
    :GPSL5I => 6,
    :GPSL5Q => 7,
    :GalileoE1B => 1,
    :GalileoE1C => 2,
    :GalileoE5aI => 3,
    :GalileoE5aQ => 4,
)

# Sort key for a `(get_signal_id, prn)` satellite-data key: constellation, then PRN,
# then band.
sat_sort_key((system_key, prn)::Tuple{Symbol,Int}) = (
    get(CONSTELLATION_ORDER, constellation_of(system_key), 99),
    prn,
    get(BAND_ORDER, system_key, 99),
)

# Frequency-band display order for inter-frequency biases (L1 < L2 < L5 …), matching the
# per-constellation band order of the CN0 bar chart. Unlisted bands sort last (rank 99).
const IFB_BAND_ORDER = Dict(:L1 => 1, :L2 => 2, :L5 => 3)

# Time-system display order for inter-system biases (GPS < Galileo < …), mirroring
# `CONSTELLATION_ORDER`. Keyed by `nameof(typeof(time_system))` (`:GPST`, `:GST`, …).
const TIME_SYSTEM_ORDER = Dict(:GPST => 1, :GST => 2, :GLONASST => 3, :BDT => 4)

# Re-express every inter-frequency bias against the lowest-ordered band (by `IFB_BAND_ORDER`)
# in its connected coverage component, independent of the reference band `calc_pvt` picks.
# `calc_pvt` anchors each component's IFBs on a reference band it chooses per solve, and that
# choice can flip between solves (e.g. L1 ↔ L5) — which makes a plotted bias jump and its
# monitored line break. Re-anchoring on the lowest band gives a stable reference. Returns a
# `Vector` of `(band, value, reference_band)`, where `value` is `band`'s delay minus the
# reference band's.
function ifbs_vs_lowest_band(pvt)
    out = Tuple{Symbol,typeof(1.0m),Symbol}[]
    isempty(pvt.inter_frequency_biases) && return out
    # Difference graph: an undirected edge band—reference weighted by bias[band] - bias[ref].
    adj = Dict{Symbol,Vector{Tuple{Symbol,typeof(1.0m)}}}()
    for (band, ifb) in pvt.inter_frequency_biases
        push!(get!(() -> Tuple{Symbol,typeof(1.0m)}[], adj, band), (ifb.reference, ifb.value))
        push!(get!(() -> Tuple{Symbol,typeof(1.0m)}[], adj, ifb.reference), (band, -ifb.value))
    end
    rank(b) = get(IFB_BAND_ORDER, b, 99)
    visited = Set{Symbol}()
    for start in keys(adj)
        start in visited && continue
        # BFS one connected component, accumulating each band's bias relative to `start`.
        rel = Dict(start => 0.0m)
        queue = [start]
        push!(visited, start)
        while !isempty(queue)
            n = popfirst!(queue)
            for (nb, w) in adj[n]
                nb in visited && continue
                rel[nb] = rel[n] - w
                push!(visited, nb)
                push!(queue, nb)
            end
        end
        # Anchor the component on its lowest-ordered band; the anchor drops out (bias 0).
        ref = argmin(rank, keys(rel))
        for band in keys(rel)
            band == ref || push!(out, (band, rel[band] - rel[ref], ref))
        end
    end
    sort!(out; by = t -> (rank(t[3]), rank(t[1])))
    return out
end

# Re-express every inter-system bias against the lowest-ordered time system present (by
# `TIME_SYSTEM_ORDER`, i.e. GPS < Galileo < …) rather than `pvt.reference_system`, which
# `calc_pvt` can swap between solves (the same instability as the IFB reference). Every ISB
# is already referenced to the single `pvt.reference_system`. Returns a `Vector` of
# `(system, value, reference_system)`.
function isbs_vs_lowest_system(pvt)
    out = Tuple{Any,typeof(1.0m),Any}[]
    isempty(pvt.inter_system_biases) && return out
    bias = Dict{Any,typeof(1.0m)}(pvt.reference_system => 0.0m)
    for (sys, v) in pvt.inter_system_biases
        bias[sys] = v
    end
    rank(s) = get(TIME_SYSTEM_ORDER, nameof(typeof(s)), 99)
    ref = argmin(rank, keys(bias))
    for sys in keys(bias)
        sys == ref || push!(out, (sys, bias[sys] - bias[ref], ref))
    end
    sort!(out; by = t -> rank(t[1]))
    return out
end

# Marker for a satellite: the circled-number glyph of its PRN, so the glyph on the
# sky plot reads as the PRN itself and cross-references the CN0 bar chart's
# `G05`/`E05` labels directly. The constellation is carried by the point colour
# (`CONSTELLATION_COLORS`), not the glyph. Wraps around for PRNs beyond the glyph
# table (circled numbers end at 50).
# NB: the circled-number glyphs 11-50 use Ambiguous/Wide code points that can render
# two cells wide in some terminals/fonts (they may look squeezed); the PRN identity
# they carry is preferred here over that cosmetic risk.
sat_marker(prn::Integer) = PRNMARKERS[mod1(prn, length(PRNMARKERS))]

const PRNMARKERS = (
    '\U2780',
    '\U2781',
    '\U2782',
    '\U2783',
    '\U2784',
    '\U2785',
    '\U2786',
    '\U2787',
    '\U2788',
    '\U2789',
    '\U246A',
    '\U246B',
    '\U246C',
    '\U246D',
    '\U246E',
    '\U246F',
    '\U2470',
    '\U2471',
    '\U2472',
    '\U2473',
    '\U3251',
    '\U3252',
    '\U3253',
    '\U3254',
    '\U3255',
    '\U3256',
    '\U3257',
    '\U3258',
    '\U3259',
    '\U325A',
    '\U325B',
    '\U325C',
    '\U325D',
    '\U325E',
    '\U325F',
    '\U32B1',
    '\U32B2',
    '\U32B3',
    '\U32B4',
    '\U32B5',
    '\U32B6',
    '\U32B7',
    '\U32B8',
    '\U32B9',
    '\U32BA',
    '\U32BB',
    '\U32BC',
    '\U32BD',
    '\U32BE',
    '\U32BF',
)

# Constellation of a `get_signal_id` symbol, used to group and colour satellites in
# the DOA plot. Derived from the id prefix so it needs no signal-type instance.
function constellation_of(signal_id::Symbol)
    s = string(signal_id)
    startswith(s, "GPS") ? :GPS :
    startswith(s, "Galileo") ? :Galileo :
    startswith(s, "GLONASS") ? :GLONASS :
    startswith(s, "BeiDou") ? :BeiDou : :Other
end

# Per-constellation DOA marker colour (both the UnicodePlots points and the legend).
# Colours follow the common skyplot convention (Safran GNSS spectrum / Trimble
# GNSS planning): GPS green, Galileo blue, GLONASS red, BeiDou yellow.
const CONSTELLATION_COLORS = Dict(
    :GPS => :green,
    :Galileo => :blue,
    :GLONASS => :red,
    :BeiDou => :yellow,
    :Other => :white,
)

# Minimum ground speed (m/s) for the course-over-ground "Heading" to be shown as a
# trustworthy value. `PVTSolution.course_over_ground` is derived from the velocity
# vector alone, so at low speed it is dominated by velocity-solution noise and points
# in an essentially random direction — below this threshold the heading is greyed out
# rather than presented as a real bearing. ~0.5 m/s is well above typical static
# velocity noise yet below any real walking/driving pace.
const MIN_SPEED_FOR_HEADING = 0.5

"""
    get_gui_data_channel(data_channel, push_gui_data_roughly_every = 500u"ms")

Return a `Channel{GUIData}` that downsamples `data_channel` for display: a spawned task
consumes every [`ReceiverDataOfInterest`](@ref) but only forwards one roughly every
`push_gui_data_roughly_every` of signal runtime (plus the very first), so the GUI is
refreshed at a human rate rather than once per processed chunk.
"""
function get_gui_data_channel(
    data_channel::Channel{<:ReceiverDataOfInterest},
    push_gui_data_roughly_every = 500ms,
)
    gui_data_channel = Channel{GUIData}()
    # Reassigned closure captures would each lower to an untyped `Core.Box`,
    # making every access in the consume loop dynamic; typed `Ref`s captured once
    # keep the loop type-stable (same pattern as the processing loop in `receive`).
    last_gui_output = Ref(0.0ms)
    first = Ref(true)
    last_pvt_time = Ref{fieldtype(PVTSolution, :time)}(nothing)
    Base.errormonitor(
        Threads.@spawn begin
            consume_channel(data_channel) do data
                if (data.runtime - last_gui_output[]) > push_gui_data_roughly_every ||
                   first[]
                    # Fresh iff there is a fix whose epoch advanced since the last emission
                    # (a re-emitted stale solution keeps the same `pvt.time`).
                    pvt_fresh =
                        !isnothing(data.pvt.time) && data.pvt.time != last_pvt_time[]
                    last_pvt_time[] = data.pvt.time
                    push!(
                        gui_data_channel,
                        GUIData(data.sat_data, data.pvt, data.runtime, pvt_fresh),
                    )
                    last_gui_output[] = data.runtime
                    first[] = false
                end
            end
            close(gui_data_channel)
        end
    )
    gui_data_channel
end

_cursor_hide(io::IO) = print(io, "\x1b[?25l")
_cursor_show(io::IO) = print(io, "\x1b[?25h")
panel(plot; kw...) = Panel(string(plot; color = true); fit = true, kw...)

"""
    gui(gui_data_channel, io = stdout; construct_gui_panels = construct_gui_panels)

Render the receiver GUI to `io`, consuming each `GUIData` from `gui_data_channel` and
redrawing the terminal in place until the channel closes. `construct_gui_panels` builds
the panel layout for one frame (satellite CN0 bars, sky plot, PVT/position block) and can
be overridden to customise the display.
"""
function gui(gui_data_channel, io::IO = stdout; construct_gui_panels = construct_gui_panels)
    terminal = REPL.Terminals.TTYTerminal("", stdin, io, stderr)
    num_dots = 0
    consume_channel(gui_data_channel) do gui_data
        panels = construct_gui_panels(gui_data, num_dots)
        num_dots = mod(num_dots + 1, 4)
        out = string(panels)
        REPL.Terminals.clear(terminal)
        _cursor_hide(io)
        println(io, out)
        _cursor_show(io)
    end
end

# Root-mean-square of a collection (0 for an empty collection).
_rms(v) = isempty(v) ? 0.0 : sqrt(sum(abs2, v) / length(v))

# Format a real with exactly two decimals (trailing zeros kept) so decimal points
# line up when the values are right-aligned in a column.
function _fmt2(x)
    s = string(round(x; digits = 2))
    dot = findfirst('.', s)
    isnothing(dot) ? s * ".00" : s * '0'^max(0, 2 - (length(s) - dot))
end

# The PVT solution's internals as text lines: DOP, inter-system and inter-frequency
# biases (metres), and pseudorange-residual RMS (overall and per signal). Appended
# below the position/velocity/time block in the combined panel. The residuals are
# greyed out until the solution is over-determined — with only
# `3 + #time-systems + #extra-bands` satellites the least-squares residual is ~0 by
# construction, so its RMS is meaningless. Sections that don't apply (no
# inter-system bias for a single constellation, no inter-frequency bias for a single
# band) are omitted. Only called with a fix present, so no "waiting" fallback.
function pvt_details_lines(pvt)
    lines = String[]

    if !isnothing(pvt.dop)
        push!(lines, "GDOP: $(_fmt2(pvt.dop.GDOP))")
    end

    # Inter-system biases re-anchored on the lowest-ordered time system present (GPS <
    # Galileo < …) rather than `calc_pvt`'s reference system, so the displayed anchor is
    # stable across solves. All share that one anchor, so it goes in the heading.
    isbs = isbs_vs_lowest_system(pvt)
    if !isempty(isbs)
        ref = string(nameof(typeof(isbs[1][3])))
        push!(lines, "Inter-system biases (vs $ref):")
        for (sys, bias, _) in isbs
            push!(lines, "  $(nameof(typeof(sys))): $(_fmt2(ustrip(m, bias))) m")
        end
    end

    # Each IFB is the receiver's differential RF-chain delay of one band relative to a
    # reference band. Re-anchored on the lowest-ordered band of each coverage component
    # (see `ifbs_vs_lowest_band`) so the reference is stable across solves; shown per line
    # since disjoint components can still carry different (lowest) references.
    ifbs = ifbs_vs_lowest_band(pvt)
    if !isempty(ifbs)
        push!(lines, "Inter-frequency biases:")
        for (band, value, reference) in ifbs
            push!(lines, "  $band (vs $reference): $(_fmt2(ustrip(m, value))) m")
        end
    end

    if !isempty(pvt.sats)
        n = length(pvt.sats)
        # Estimated unknowns: 3 position + one clock per time system (reference +
        # each inter-system bias) + one per extra band.
        num_unknowns = 3 + (1 + length(pvt.inter_system_biases)) + length(pvt.inter_frequency_biases)
        residuals = [ustrip(m, info.residual) for info in values(pvt.sats)]
        # Table rows (label, RMS, count): overall first, then one per signal.
        rows = Tuple{String,Float64,Int}[("overall", _rms(residuals), n)]
        for sig in unique(first(key) for key in keys(pvt.sats))
            per_sig =
                [ustrip(m, pvt.sats[key].residual) for key in keys(pvt.sats) if first(key) == sig]
            # Label each per-signal row by its band abbreviation (e.g. `GalileoE1C_BOC11`
            # → "E1"), falling back to the raw id for an unlisted signal.
            push!(rows, (get(BAND_ABBREVIATIONS, sig, string(sig)), _rms(per_sig), length(per_sig)))
        end
        namew = maximum(length(first(row)) for row in rows)
        res_lines = ["Pseudorange residual RMS:", "  $(rpad("signal", namew))  $(lpad("RMS/m", 6))   n"]
        for (label, rms, cnt) in rows
            push!(res_lines, "  $(rpad(label, namew))  $(lpad(_fmt2(rms), 6))  $(lpad(cnt, 2))")
        end
        if n > num_unknowns
            append!(lines, res_lines)
        else
            # Grey out: no redundancy, residuals ~0 by construction.
            res_lines[1] *= " (insufficient redundancy)"
            for line in res_lines
                push!(lines, "{dim}$line{/dim}")
            end
        end
    end

    lines
end

function construct_gui_panels(gui_data, num_dots)
    # Bars sorted by constellation (GPS, then Galileo, …), then PRN, then band.
    sorted_keys = sort(collect(keys(gui_data.sat_data)); by = sat_sort_key)
    prn_strings = [sat_label(key...) for key in sorted_keys]
    cn0s = map(sorted_keys) do key
        x = gui_data.sat_data[key]
        round(10 * log10(linear(x.cn0 == Inf * Hz ? 1Hz : x.cn0) / Hz); digits = 1)
    end
    colors = map(key -> gui_data.sat_data[key].is_healthy ? :green : :red, sorted_keys)
    pvt = gui_data.pvt
    runtime_row = "Run time: $(round(ustrip(s, gui_data.runtime); digits = 1)) s"
    sat_doa_panel_title = "Satellite Direction-of-Arrival (DOA)"
    pvt_panel_title = "Position Velocity Time (PVT)"
    not_enough_sats_text = "Not enough satellites to calculate position."
    cn0_panel_title = "Carrier-to-Noise-Density-Ratio (CN0)"
    cn0_panel =
        !isempty(prn_strings) ?
        panel(
            barplot(
                prn_strings,
                cn0s;
                color = colors,
                ylabel = "Satellites",
                xlabel = "Carrier-to-Noise-Density-Ratio (CN0) [dBHz]",
            );
            fit = true,
            title = cn0_panel_title,
        ) :
        Panel(
            "Searching for satellites$(repeat(".", num_dots))";
            title = cn0_panel_title,
            width = length(cn0_panel_title) + 5,
        )
    if !isnothing(pvt.time)
        # One point per *physical* satellite: the same satellite tracked on several
        # signals (e.g. Galileo E1B + E5a) shares an az/el, so keying by
        # (constellation, PRN) — not (signal, PRN) — stops the duplicates piling
        # onto each other. Points are coloured by constellation.
        seen = Set{Tuple{Symbol,Int}}()
        azs = Float64[]
        els_deg = Float64[]
        markers = Char[]
        point_colors = Symbol[]
        for (key, info) in pairs(pvt.sats)
            signal_id, prn = key
            system = constellation_of(signal_id)
            (system, prn) in seen && continue
            push!(seen, (system, prn))
            enu = get_sat_enu(pvt.position, info.position)
            push!(azs, enu.θ)
            push!(els_deg, enu.ϕ * 180 / π)
            push!(markers, sat_marker(prn))
            push!(point_colors, get(CONSTELLATION_COLORS, system, :white))
        end
        present = sort(unique(constellation_of(first(key)) for key in keys(pvt.sats)))
        legend = join(
            [
                "{$(get(CONSTELLATION_COLORS, c, :white))}●{/$(get(CONSTELLATION_COLORS, c, :white))} $c"
                for c in present
            ],
            "   ",
        )
        doa_plot = polarplot(
            azs,
            els_deg;
            rlim = (0, 90),
            scatter = true,
            marker = markers,
            color = point_colors,
        )
        # Re-label the angular axis to the GNSS azimuth convention: 0° = North on
        # top, increasing clockwise (90° = East right, 180° = South bottom,
        # 270° = West left). `polarplot` labels the ring with math angles (0° right,
        # counter-clockwise), but the satellite az/el points are already placed
        # geographically (North up, East right), so only the labels need fixing.
        grid_color = UnicodePlots.BORDER_COLOR[]
        mid_row = ceil(Int, UnicodePlots.nrows(doa_plot.graphics) / 2)
        label!(doa_plot, :t, "0°"; color = grid_color)
        label!(doa_plot, :r, mid_row, "90°"; color = grid_color)
        label!(doa_plot, :b, "180°"; color = grid_color)
        label!(doa_plot, :l, mid_row, "270°"; color = grid_color)
        doa_panel = Panel(
            string(doa_plot; color = true) * "\n" * legend;
            fit = true,
            title = sat_doa_panel_title,
        )
        lla = get_LLA(pvt)
        lat_hem = lla.lat >= 0 ? "N" : "S"
        lon_hem = lla.lon >= 0 ? "E" : "W"
        speed = sqrt(sum(abs2, pvt.velocity))
        # Time solution first: `pvt.time` is a TAI epoch; show it as UTC (the civil
        # scale users expect) via AstroTime's leap-aware `to_utc`, to millisecond
        # precision — the clock-bias solution is far coarser than the sub-second
        # digits. AstroTime has no UTC epoch *type*, so `to_utc` yields a string.
        # Format is time-then-date (`HH:MM:SS.sss dd.mm.yyyy`), echoing NMEA's
        # time/date field ordering — GNSS has no single canonical display format.
        utc_str = to_utc(String, pvt.time, dateformat"HH:MM:SS.sss dd.mm.yyyy")
        # Heading = course over ground: the azimuth of the velocity vector (degrees
        # clockwise from true North), computed by PVT. Only trustworthy while moving,
        # so grey it out below `MIN_SPEED_FOR_HEADING` — the value stays visible but
        # dimmed to signal it is noise-dominated, matching how the residuals are greyed.
        heading = "$(round(ustrip(°, pvt.course_over_ground); digits = 1))°"
        heading_value = speed >= MIN_SPEED_FOR_HEADING ? heading :
            "{dim}$heading (low speed){/dim}"
        # Position/velocity/time rows as (label, value) pairs, Time first. Labels are
        # padded to a common width so the values line up in a column.
        pvt_rows = [
            ("Time", "$utc_str UTC"),
            (
                "Coordinates",
                "$(abs(round(lla.lat; digits = 6)))°$lat_hem, " *
                "$(abs(round(lla.lon; digits = 6)))°$lon_hem",
            ),
            ("Altitude", "$(round(lla.alt; digits = 1)) m"),
            ("Speed", "$(round(speed; digits = 2)) m/s"),
            ("Heading", heading_value),
            ("Run time", "$(round(ustrip(s, gui_data.runtime); digits = 1)) s"),
        ]
        labelw = maximum(length(first(r)) for r in pvt_rows) + 1  # +1 for the colon
        pvt_lines = ["$(rpad(first(r) * ":", labelw)) $(last(r))" for r in pvt_rows]
        # Append the solution internals (DOP, biases, residuals) below, separated by a
        # blank line, so position/velocity/time and diagnostics share one PVT block.
        details = pvt_details_lines(pvt)
        lines = isempty(details) ? pvt_lines : vcat(pvt_lines, "", details)
        # If the latest `calc_pvt` produced no new solution, this fix is stale (a re-emitted
        # old one): grey the whole block and flag it in the title, so a frozen position is
        # not read as live. `pvt_fresh` is set by `get_gui_data_channel`.
        if !gui_data.pvt_fresh
            lines = ["{dim}$line{/dim}" for line in lines]
        end
        pvt_title = gui_data.pvt_fresh ? pvt_panel_title : "$pvt_panel_title — stale (no new fix)"
        pvt_panel = Panel(join(lines, "\n"); fit = true, title = pvt_title)
    elseif length(prn_strings) < 4
        doa_panel = Panel(not_enough_sats_text; title = sat_doa_panel_title, fit = true)
        pvt_panel =
            Panel("$not_enough_sats_text\n$runtime_row"; title = pvt_panel_title, fit = true)
    else
        decoding_text = "Decoding satellites$(repeat(".", num_dots))"
        doa_panel = Panel(
            decoding_text;
            title = sat_doa_panel_title,
            width = length(not_enough_sats_text) + 5,
        )
        pvt_panel = Panel(
            "$decoding_text\n$runtime_row";
            title = pvt_panel_title,
            width = length(not_enough_sats_text) + 5,
        )
    end
    # Layout: CN0 | DOA | the combined Position/Velocity/Time block.
    cn0_panel * doa_panel * pvt_panel
end