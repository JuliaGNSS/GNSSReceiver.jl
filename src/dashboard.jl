# The live GUI is a Tachikoma app (Elm-style Model/update!/view). A background task drains
# the `GUIData` channel into the model; `view` lays out four panels — CN0 bars and a
# direction-of-arrival sky plot on top, a Position/Velocity/Time block and an OpenStreetMap
# map below — and paints them each frame. The CN0 bars and the sky plot are drawn by
# `UnicodePlots` (`barplot`/`polarplot`), the map by `UnicodeMaps`; all three arrive as an
# ANSI colour string that `_paint_plot!` paints into the panel. Interactivity: `d` toggles
# the PVT diagnostics, `+`/`-`/`hjkl`/`0` drive the map, `q`/Ctrl-C quit.
#
# It lives in its own module — the layout Tachikoma's `@tachikoma_app` docstring prescribes
# — because `using Tachikoma` plus that macro bring ~150 framework names into scope, and
# several of them collide with names the receiver itself uses: `Tachikoma.linear` shadows
# `Unitful.linear`, and `Tachikoma.view` is a *brand-new* generic function rather than a
# method of `Base.view`, so at package scope a plain `view(x, 1:n)` anywhere in the sample
# path would fail with a `MethodError` instead of taking a slice. Confining the framework
# here keeps `GNSSReceiver`'s namespace free of it; only the handful of names below crosses
# the boundary.
module Dashboard

using Tachikoma
@tachikoma_app

using UnicodePlots
using UnicodeMaps: TileSource, worldmap
using Dates: @dateformat_str
using AstroTime: to_utc
using PositionVelocityTime: get_LLA, get_sat_enu
using SignalChannels: consume_channel
# `Unitful` itself (not `using Unitful`) because Tachikoma exports an easing function also
# named `linear`, which would make the bare name ambiguous here.
import Unitful
using Unitful: ustrip, s, Hz, °

using ..GNSSReceiver:
    CONSTELLATION_COLORS,
    GUIData,
    MIN_SPEED_FOR_HEADING,
    constellation_of,
    pvt_details_lines,
    sat_label,
    sat_sort_key

# What the map panel needs rendered: the tile centre, the panel size in cells, the zoom
# level, and whether to pin the centre (only while the map is centred on the fix).
const MapRequest = @NamedTuple{
    lat::Float64, lon::Float64, width::Int, height::Int, zoom::Int, marker::Bool}

const MAP_DEFAULT_ZOOM = 13
const MAP_MIN_ZOOM = 1        # whole world
const MAP_MAX_ZOOM = 18       # OpenStreetMap's deepest tile level
const MAP_MIN_WIDTH = 8       # below this the tile is unreadable; show the text fallback
const MAP_MIN_HEIGHT = 4
const MAP_POLL_INTERVAL = 0.5 # seconds between checks for a new map request

mutable struct ReceiverModel <: Model
    quit::Bool
    tick::Int                       # frame counter, drives the "Searching…" dots
    lk::ReentrantLock
    gui::Union{GUIData,Nothing}     # latest frame from the receiver
    last_fix::Union{GUIData,Nothing}# last frame that carried a real PVT fix
    show_diagnostics::Bool          # PVT diagnostics section, shown by default (toggle with `d`)
    stream_ended::Bool              # set when the data channel closes (stream finished)
    # Map state. `map_want` is what `view` asks for; `_spawn_map` renders it in the
    # background and caches the ANSI string in `map_ansi`, tagged with `map_key`.
    map_zoom::Int
    map_dlon::Float64               # pan offset from the fix, degrees longitude
    map_dlat::Float64               # pan offset from the fix, degrees latitude
    map_ansi::Union{String,Nothing} # last rendered map, as an ANSI colour string
    map_key::Union{MapRequest,Nothing}  # the request `map_ansi` answers
    map_want::Union{MapRequest,Nothing} # the request the map panel wants rendered
end

ReceiverModel() = ReceiverModel(false, 0, ReentrantLock(), nothing, nothing, true, false,
    MAP_DEFAULT_ZOOM, 0.0, 0.0, nothing, nothing, nothing)

should_quit(m::ReceiverModel) = m.quit

# Docstrings in here are resolved by Documenter against `Dashboard`, not `GNSSReceiver`, so
# any `@ref` to something in the parent module must name it in full.
"""
    gui(gui_data_channel; fps = 12)

Display the receiver dashboard, consuming each `GUIData` from `gui_data_channel`. Runs a
Tachikoma terminal app: a background task keeps the model fed with the latest frame while
the app renders the CN0 bars, the direction-of-arrival sky plot, the Position/Velocity/Time
block and an OpenStreetMap map of the fix. When the stream ends the last frame stays on
screen (flagged "stream ended"); the app blocks until the user quits (`q` or `Ctrl-C`).
Keys: `d` toggles the PVT diagnostics; `+`/`-` zoom and `hjkl` pan the map, `0` recenters it.

Quitting closes `gui_data_channel`, which tears the pipeline down behind it (see
[`get_gui_data_channel`](@ref GNSSReceiver.get_gui_data_channel)) — `q` stops the run, it
does not just hide it.
"""
function gui(gui_data_channel; fps::Int = 12)
    m = ReceiverModel()
    # Feed the model from the receiver channel. When the channel closes, keep the last
    # frame on screen (flag it) rather than quitting — the user quits with `q`.
    Base.errormonitor(
        Threads.@spawn begin
            consume_channel(gui_data_channel) do gui_data
                @lock m.lk begin
                    m.gui = gui_data
                    isnothing(gui_data.pvt.time) || (m.last_fix = gui_data)
                end
            end
            @lock m.lk (m.stream_ended = true)
        end
    )
    _spawn_map(m)
    try
        # Prefer the interactive threadpool (`julia -t auto,1`) so the render loop is not
        # starved by the streaming/DSP tasks on the default pool.
        if Threads.nthreads(:interactive) > 0
            wait(Threads.@spawn :interactive app(m; fps))
        else
            app(m; fps)
        end
    finally
        # Quitting the dashboard ends the run: closing our end of the channel stops the
        # feeder, which closes the data channel behind it and so unwinds `receive` and the
        # sample reader too. Without this the user gets the prompt back while the pipeline
        # keeps churning through the rest of the recording.
        close(gui_data_channel)
        # Also stop the map task, which is otherwise only wound up by `q` — the app can
        # leave through an exception too, and nothing should keep fetching tiles for a
        # dashboard that is gone.
        @lock m.lk (m.quit = true)
    end
end

# ── Input ─────────────────────────────────────────────────────────────────────
function update!(m::ReceiverModel, e::KeyEvent)
    if e.key == :ctrl_c || (e.key == :char && (e.char == 'q' || e.char == 'Q'))
        m.quit = true
        return
    end
    if e.key == :char && (e.char == 'd' || e.char == 'D')
        @lock m.lk (m.show_diagnostics = !m.show_diagnostics)
        return
    end
    # Map controls: `+`/`-` zoom, `hjkl` pan (vim), `0` recenter. `←/→` are left free for
    # future navigation; the map uses hjkl so as not to clobber them.
    if e.key == :char
        c = e.char
        if c == '+' || c == '='
            @lock m.lk (m.map_zoom = min(m.map_zoom + 1, MAP_MAX_ZOOM))
        elseif c == '-' || c == '_'
            @lock m.lk (m.map_zoom = max(m.map_zoom - 1, MAP_MIN_ZOOM))
        elseif c == '0'
            @lock m.lk begin
                m.map_zoom = MAP_DEFAULT_ZOOM
                m.map_dlon = 0.0
                m.map_dlat = 0.0
            end
        elseif c == 'h' || c == 'j' || c == 'k' || c == 'l'
            @lock m.lk begin
                # ~⅓ of the view per press: the tile's span halves with every zoom level.
                step = 0.35 * 360.0 / 2.0^m.map_zoom
                c == 'h' && (m.map_dlon -= step)       # west
                c == 'l' && (m.map_dlon += step)       # east
                c == 'k' && (m.map_dlat += step)       # north
                c == 'j' && (m.map_dlat -= step)       # south
            end
        end
    end
    return
end

update!(::ReceiverModel, ::Event) = nothing

# ── View ────────────────────────────────────────────────────────────────────
const CN0_PANEL_TITLE = "Carrier-to-Noise-Density-Ratio (CN0) [dBHz]"
const DOA_PANEL_TITLE = "Satellite Direction-of-Arrival (DOA)"
const PVT_PANEL_TITLE = "Position Velocity Time (PVT)"
const MAP_PANEL_TITLE = "Map"
const NOT_ENOUGH_SATS_TEXT = "Not enough satellites to calculate position."

function view(m::ReceiverModel, f::Frame)
    m.tick += 1
    gui_data, last_fix, show_diag, ended =
        @lock m.lk (m.gui, m.last_fix, m.show_diagnostics, m.stream_ended)
    buf = f.buffer
    num_dots = mod(m.tick ÷ 4, 4)

    rows = split_layout(Layout(Vertical, [Fixed(1), Fill(), Fixed(1)]), f.area)
    header, body, footer = rows[1], rows[2], rows[3]

    fresh = gui_data !== nothing && gui_data.pvt_fresh
    # "stale" only once a fix is being held frozen (a re-emitted old solution) — not while
    # still searching/decoding, where there is simply no fix yet. Once the stream has ended
    # the frozen frame is expected, so show "stream ended" instead of "stale".
    has_fix = gui_data !== nothing && !isnothing(gui_data.pvt.time)
    rt = gui_data === nothing ? 0.0 : round(ustrip(s, gui_data.runtime); digits = 1)
    status = ended ? "  │  stream ended (press q to quit)" :
             (has_fix && !fresh ? "  │  stale (no new fix)" : "")
    hdr = " ● GNSSReceiver  │  run time $(rt) s" * status
    set_string!(buf, header.x, header.y, rpad(hdr, header.width),
        tstyle(:title, bold = true); max_x = right(header))

    # Body: CN0 | DOA (top), PVT | Map (bottom).
    toprow, botrow = split_layout(Layout(Vertical, [Percent(50), Fill()]), body)
    topcols = split_layout(Layout(Horizontal, [Percent(50), Fill()]), toprow)
    botcols = split_layout(Layout(Horizontal, [Percent(42), Fill()]), botrow)
    _render_cn0(buf, topcols[1], gui_data, num_dots)
    _render_skyplot(buf, topcols[2], gui_data, num_dots)
    _render_position(buf, botcols[1], gui_data, last_fix, show_diag, fresh, ended)
    _render_map(m, buf, botcols[2], last_fix)

    diaghint = show_diag ? "[d] hide diagnostics" : "[d] diagnostics"
    render(StatusBar(
            left = [Span(" [+/-] zoom  [hjkl] pan  [0] recenter  ", tstyle(:text_dim)),
                Span(diaghint, tstyle(:text_dim))],
            right = [Span("  [q] quit ", tstyle(:text_dim))],
        ), footer, buf)
    return
end

# Paint a UnicodePlots colour string (`string(plot; color=true)`) or any ANSI text into
# `area`: split into lines, parse each line's ANSI into spans, and lay the spans out
# left-to-right, clipping at the panel edges.
function _paint_plot!(buf, area::Rect, str::AbstractString)
    for (i, line) in enumerate(split(str, '\n'))
        y = area.y + i - 1
        y > bottom(area) && break
        x = area.x
        for sp in parse_ansi(String(line))
            x > right(area) && break
            # `set_string!` returns the next free column, which is the only reliable
            # advance: it strips ANSI escapes and bare control chars, segments graphemes and
            # substitutes a space for a wide glyph straddling `max_x`, so recomputing the
            # width here (e.g. `textwidth(sp.content)`) would drift — and a one-cell drift
            # misplaces satellites against the axis labels in the braille sky plot.
            x = set_string!(buf, x, y, sp.content, sp.style; max_x = right(area))
        end
    end
    return
end

# CN0 in dBHz as a plain rounded number, floored at the 0 dB (1 Hz) baseline — the same
# reference an `Inf` CN0 got in the old GUI. `barplot` throws on anything negative or
# non-finite ("all values have to be ≥ 0"), and both are reachable for a satellite the
# detectors are still holding: Tracking's NWPR estimator reports `-Inf dBHz` for "no
# detectable signal" and a negative dB-Hz figure just above that (any CN0 under 1 Hz),
# while `Inf`/`NaN` come from a degenerate prompt buffer. Such a satellite draws an empty
# bar instead of taking the whole panel — and with it the dashboard — down.
function _cn0_db(cn0)
    db = 10 * log10(Unitful.linear(cn0) / Hz)
    isfinite(db) ? max(round(db; digits = 1), 0.0) : 0.0
end

function _render_cn0(buf, area::Rect, gui_data, num_dots)
    inner = render(Block(; title = CN0_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    if gui_data === nothing || isempty(gui_data.sat_data)
        set_string!(buf, inner.x + 1, inner.y, "Searching for satellites$(repeat(".", num_dots))",
            tstyle(:text_dim); max_x = right(inner))
        return
    end
    # Bars sorted by constellation (GPS, then Galileo, …), then PRN, then band; coloured
    # green (healthy) / red (unhealthy).
    sorted_keys = sort(collect(keys(gui_data.sat_data)); by = sat_sort_key)
    labels = [sat_label(key...) for key in sorted_keys]
    cn0s = [_cn0_db(gui_data.sat_data[key].cn0) for key in sorted_keys]
    colors = [gui_data.sat_data[key].is_healthy ? :green : :red for key in sorted_keys]
    labelw = maximum(length, labels)
    # Chrome `barplot` draws around the bars themselves: the label column, then `" ┤"`
    # between label and bar, then the value printed after the bar (" 45.1" — a space plus up
    # to four characters for `NN.N`). That is 2 + 5 = 7 columns, plus one leading and one
    # trailing column of panel padding, so 9 in total is taken off the panel width.
    barwidth = clamp(inner.width - labelw - 9, 5, 60)
    plot = barplot(labels, cn0s; color = colors, border = :none,
        width = barwidth, maximum = 55)
    _paint_plot!(buf, inner, string(plot; color = true))
    return
end

function _render_skyplot(buf, area::Rect, gui_data, num_dots)
    inner = render(Block(; title = DOA_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    if gui_data === nothing || isnothing(gui_data.pvt.time)
        nsat = gui_data === nothing ? 0 : length(gui_data.sat_data)
        msg = nsat < 4 ? NOT_ENOUGH_SATS_TEXT : "Decoding satellites$(repeat(".", num_dots))"
        set_string!(buf, inner.x + 1, inner.y, msg, tstyle(:text_dim); max_x = right(inner))
        return
    end
    pvt = gui_data.pvt
    # One point per *physical* satellite: the same satellite tracked on several signals
    # (e.g. Galileo E1B + E5a) shares an az/el, so key by (constellation, PRN) — not
    # (signal, PRN) — to stop the duplicates piling onto each other. Colour by constellation.
    seen = Set{Tuple{Symbol,Int}}()
    azs = Float64[]
    els_deg = Float64[]
    prns = Int[]
    point_colors = Symbol[]
    for (key, info) in pairs(pvt.sats)
        signal_id, prn = key
        system = constellation_of(signal_id)
        (system, prn) in seen && continue
        push!(seen, (system, prn))
        enu = get_sat_enu(pvt.position, info.position)
        push!(azs, enu.θ)
        push!(els_deg, enu.ϕ * 180 / π)
        push!(prns, prn)
        push!(point_colors, get(CONSTELLATION_COLORS, system, :white))
    end
    # Reserve one row for the legend below the plot.
    plotarea = Rect(inner.x, inner.y, inner.width, max(1, inner.height - 1))
    # Size the polarplot canvas 2:1 (columns:rows) so it renders as a round circle: braille
    # cells are ~square on screen, so equal x/y data range needs twice as many columns as
    # rows. Leave margin for the axis labels UnicodePlots draws around the canvas (~13 cols,
    # ~4 rows) so the whole circle fits and is not clipped (which read as "not round").
    wcanvas = clamp(min(plotarea.width - 13, 2 * (plotarea.height - 4)), 8, 60)
    hcanvas = max(4, wcanvas ÷ 2)
    grid_color = UnicodePlots.BORDER_COLOR[]
    # The radius is the *zenith distance* (90° − elevation), the standard sky-plot
    # convention: the zenith sits at the centre and the horizon on the rim, so a satellite
    # moves outward as it sets. `polarplot`'s own radial labels would then read as zenith
    # distance, which is not what a GNSS user expects, so they are suppressed
    # (`num_rad_lab = 0`) and the rings are annotated with elevation below.
    zenith_dists = 90 .- els_deg
    doa_plot = polarplot(azs, zenith_dists; rlim = (0, 90), scatter = true,
        marker = :circle, color = point_colors, border = :none,
        num_rad_lab = 0, width = wcanvas, height = hcanvas)
    # Label each satellite with its PRN number (coloured by constellation) placed exactly
    # on its point. `polarplot` plots θ (az) counter-clockwise from +x at radius r, so the
    # point's Cartesian position is (r·cos az, r·sin az) — annotate there.
    for (az, r, prn, col) in zip(azs, zenith_dists, prns, point_colors)
        annotate!(doa_plot, r * cos(az), r * sin(az), string(prn); color = col)
    end
    # Elevation rings, labelled along the same π/4 diagonal `polarplot` uses for its own
    # radial labels, and — like those — as bare numbers: the `°` is reserved for the azimuth
    # labels around the rim, so a ring label can never be misread as a bearing. 90 (the
    # zenith) is left out, since at the centre it would sit on top of the satellites that are
    # highest in the sky.
    for el in (0, 30, 60)
        r = 90 - el
        annotate!(doa_plot, r * cos(π / 4), r * sin(π / 4), string(el); color = grid_color)
    end
    # Re-label the angular axis to the GNSS azimuth convention: 0° = North on top,
    # increasing clockwise (90° = East right, 180° = South bottom, 270° = West left).
    mid_row = ceil(Int, UnicodePlots.nrows(doa_plot.graphics) / 2)
    label!(doa_plot, :t, "0°"; color = grid_color)
    label!(doa_plot, :r, mid_row, "90°"; color = grid_color)
    label!(doa_plot, :b, "180°"; color = grid_color)
    label!(doa_plot, :l, mid_row, "270°"; color = grid_color)
    _paint_plot!(buf, plotarea, string(doa_plot; color = true))
    # Legend: a coloured ● per present constellation, matching the point colours exactly.
    present = sort(unique(constellation_of(first(key)) for key in keys(pvt.sats)))
    _paint_plot!(buf, Rect(inner.x + 1, bottom(inner), inner.width - 1, 1),
        _legend_ansi(present))
    return
end

# Constellation legend as an ANSI-coloured string (parsed back into spans by `_paint_plot!`),
# so the legend markers use exactly the same terminal colours as the plotted points.
function _legend_ansi(present)
    io = IOContext(IOBuffer(), :color => true)
    for (i, c) in enumerate(present)
        i == 1 || print(io, "   ")
        printstyled(io, "●"; color = get(CONSTELLATION_COLORS, c, :white))
        print(io, " ", c)
    end
    String(take!(io.io))
end

function _render_position(buf, area::Rect, gui_data, last_fix, show_diag, fresh, ended)
    inner = render(Block(; title = PVT_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    live = gui_data !== nothing && !isnothing(gui_data.pvt.time)
    fix = live ? gui_data : last_fix
    x, y = inner.x + 1, inner.y
    if fix === nothing
        nsat = gui_data === nothing ? 0 : length(gui_data.sat_data)
        rt = gui_data === nothing ? 0.0 : round(ustrip(s, gui_data.runtime); digits = 1)
        msg = nsat < 4 ? NOT_ENOUGH_SATS_TEXT : "Decoding satellites…"
        set_string!(buf, x, y, msg, tstyle(:text_dim); max_x = right(inner))
        set_string!(buf, x, y + 1, "$nsat satellites tracked   run time $rt s",
            tstyle(:text_dim); max_x = right(inner))
        return
    end
    pvt = fix.pvt
    lla = get_LLA(pvt)
    lat_hem = lla.lat >= 0 ? "N" : "S"
    lon_hem = lla.lon >= 0 ? "E" : "W"
    speed = sqrt(sum(abs2, pvt.velocity))
    # Time solution: `pvt.time` is a TAI epoch shown as UTC (leap-aware `to_utc`) to ms,
    # time-then-date (HH:MM:SS.sss dd.mm.yyyy).
    utc_str = to_utc(String, pvt.time, dateformat"HH:MM:SS.sss dd.mm.yyyy")
    heading = "$(round(ustrip(°, pvt.course_over_ground); digits = 1))°"
    low_speed = speed < MIN_SPEED_FOR_HEADING
    heading_value = low_speed ? "$heading (low speed)" : heading
    rt = gui_data === nothing ? 0.0 : round(ustrip(s, gui_data.runtime); digits = 1)
    pvt_rows = [
        ("Time", "$utc_str UTC"),
        ("Coordinates",
            "$(abs(round(lla.lat; digits = 6)))°$lat_hem, " *
            "$(abs(round(lla.lon; digits = 6)))°$lon_hem"),
        ("Altitude", "$(round(lla.alt; digits = 1)) m"),
        ("Speed", "$(round(speed; digits = 2)) m/s"),
        ("Heading", heading_value),
        ("Run time", "$rt s"),
    ]
    labelw = maximum(length(first(r)) for r in pvt_rows) + 1  # +1 for the colon
    # A stale (re-emitted) fix or a held last-fix is dimmed so a frozen position is not
    # read as live.
    base_style = (fresh && live) ? tstyle(:text) : tstyle(:text_dim)
    if !live
        # The displayed fix is older than the current frame. Say *why* it is frozen: after
        # the stream ends nothing is being re-acquired, and claiming otherwise contradicts
        # the header.
        note = ended ? "◦ last fix (stream ended)" : "◦ last fix (re-acquiring)"
        set_string!(buf, x, y, note, tstyle(:warning); max_x = right(inner))
        y += 1
    end
    for r in pvt_rows
        y > bottom(inner) && return
        st = (r[1] == "Heading" && low_speed) ? tstyle(:text_dim) : base_style
        set_string!(buf, x, y, "$(rpad(first(r) * ":", labelw)) $(last(r))", st; max_x = right(inner))
        y += 1
    end
    # Diagnostics (DOP, biases, pseudorange residuals): shown below on demand, dimmed as
    # secondary info. Clipped at the panel bottom.
    if show_diag
        y += 1
        for line in pvt_details_lines(pvt)
            y > bottom(inner) && return
            set_string!(buf, x, y, line, tstyle(:text_dim); max_x = right(inner))
            y += 1
        end
    end
    return
end

# ── Map ─────────────────────────────────────────────────────────────────────
# The fix on an OpenStreetMap tile. Fetching and rendering a tile takes far too long for a
# frame, so `view` only ever *asks* for a map (`map_want`) and paints whatever the
# background task has cached; until that arrives — and whenever it cannot be fetched at all
# (no network) — the panel falls back to the coordinates and a Google Maps link.
#
# `get_LLA` is called unguarded, exactly as in `_render_position`: both panels convert the
# same `pvt`, so swallowing a failure here would only blank this panel while the PVT panel
# above still crashed the frame.
function _render_map(m::ReceiverModel, buf, area::Rect, last_fix)
    inner = render(Block(; title = MAP_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    if last_fix === nothing
        set_string!(buf, inner.x + 1, inner.y, "awaiting fix…", tstyle(:text_dim);
            max_x = right(inner))
        return
    end
    lla = get_LLA(last_fix.pvt)
    zoom, dlon, dlat = @lock m.lk (m.map_zoom, m.map_dlon, m.map_dlat)
    lat, lon = round(lla.lat + dlat; digits = 5), round(lla.lon + dlon; digits = 5)
    ansi = if inner.width < MAP_MIN_WIDTH || inner.height < MAP_MIN_HEIGHT
        nothing   # panel too small for a tile; the text fallback still fits
    else
        # The pin marks the centre of the tile, which is the fix only while unpanned.
        want = MapRequest((lat, lon, Int(inner.width), Int(inner.height), zoom,
            dlon == 0.0 && dlat == 0.0))
        @lock m.lk begin
            m.map_want = want
            m.map_ansi
        end
    end
    if ansi === nothing
        set_string!(buf, inner.x + 1, inner.y, "$lat, $lon",
            tstyle(:success, bold = true); max_x = right(inner))
        # A real OSC 8 hyperlink (via the style's `hyperlink`), so the URL is click-to-open
        # in terminals that support it (and copy-pasteable everywhere).
        url = "https://www.google.com/maps?q=$lat,$lon"
        set_string!(buf, inner.x + 1, inner.y + 1, url,
            tstyle(:text_dim; hyperlink = url); max_x = right(inner))
        return
    end
    _paint_plot!(buf, inner, ansi)
    return
end

# Render the map on a background task (tile download + drawing, a good fraction of a
# second), once per (centre, panel size, zoom) — the map panel only ever paints the cached
# string. On any failure (offline, tile server error) the request is still marked done, so
# a failing one is not retried in a tight loop, and the panel keeps showing the coordinate
# fallback. The task ends with the app.
function _spawn_map(m::ReceiverModel)
    Base.errormonitor(
        Threads.@spawn begin
            # One tile source for the whole run: building it resolves the current
            # OpenFreeMap tile URL over the network, and it carries the decoded-tile cache,
            # so panning back over ground already covered costs no download at all. Built
            # on first use (and retried later) so being offline at startup is not fatal.
            source = nothing
            while !m.quit
                want, have = @lock m.lk (m.map_want, m.map_key)
                if want !== nothing && want != have
                    ansi = try
                        isnothing(source) && (source = TileSource())
                        img = worldmap(; center = (want.lon, want.lat), zoom = want.zoom,
                            size = (want.width, want.height), marker = want.marker, source)
                        sprint(show, img)
                    catch
                        nothing
                    end
                    @lock m.lk begin
                        m.map_key = want
                        isnothing(ansi) || (m.map_ansi = ansi)
                    end
                end
                sleep(MAP_POLL_INTERVAL)
            end
        end
    )
end

end # module Dashboard
