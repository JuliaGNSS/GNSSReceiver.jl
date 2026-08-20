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
    constellation_name,
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

# Columns `barplot` pads before its (right-justified) label column, so the CN0 panel can put
# its continuation marker in that column rather than guessing at the panel's own padding.
const CN0_LABEL_INDENT = 3

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
const CN0_PANEL_TITLE = "Carrier-to-Noise-Density Ratio (C/N₀) [dBHz]"
# Spelling out the marker legend costs more columns than the long title leaves, and a title
# clipped mid-legend is worse than a terse one — so the panel shortens its name to make room.
const CN0_PANEL_TITLE_SHORT = "C/N₀ [dBHz]"
const DOA_PANEL_TITLE = "Satellite Direction-of-Arrival (DOA)"
const PVT_PANEL_TITLE = "Position Velocity Time (PVT)"
const MAP_PANEL_TITLE = "Map"

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
    hx = set_string!(buf, header.x, header.y, hdr,
        tstyle(:title, bold = true); max_x = right(header))
    # Vector-tracking badge, coloured by state: green while the navigation filter is closing
    # the loops, yellow while enabled but not yet running ("armed"), and nothing at all when
    # VT is disabled. The count reads measured/loop — the satellites whose measurements
    # determined this solution (`pvt.sats`, the same set the fix-quality table counts) out of
    # the loop's total membership (the `in_vt_loop` flags of this same frame), the difference
    # being members coasted through an obscuration. One number for both would hide exactly
    # that.
    vt = gui_data === nothing ? nothing : gui_data.vt
    if vt !== nothing
        num_members = count(sat -> sat.in_vt_loop, gui_data.sat_data)
        # Non-zero only while the filter is (or has recently been) propagating on its dynamics
        # alone: the starvation watchdog grows this whenever an epoch could not determine the
        # navigation state and pays it back at half rate afterwards, so it is an "unsolvable
        # for about this long" figure rather than a stopwatch — shown only while it is running
        # up, and dropped from the badge as soon as it is back to zero.
        coasting = vt.time_with_insufficient_meas > 0.0s ?
            ", propagating $(round(ustrip(s, vt.time_with_insufficient_meas); digits = 1)) s" :
            ""
        badge, badge_style = vt.running ?
            (
                "  │  VT running ($(length(gui_data.pvt.sats))/$(num_members) sats$(coasting))",
                isempty(coasting) ? tstyle(:success, bold = true) : tstyle(:warning, bold = true),
            ) :
            ("  │  VT armed", tstyle(:warning))
        hx = set_string!(buf, hx, header.y, badge, badge_style; max_x = right(header))
    end
    # Pad the rest of the row so the title bar stays continuous.
    hx <= right(header) && set_string!(buf, hx, header.y,
        repeat(" ", right(header) - hx + 1), tstyle(:title, bold = true); max_x = right(header))

    # Body: CN0 | DOA (top), PVT | Map (bottom).
    toprow, botrow = split_layout(Layout(Vertical, [Percent(50), Fill()]), body)
    topcols = split_layout(Layout(Horizontal, [Percent(50), Fill()]), toprow)
    botcols = split_layout(Layout(Horizontal, [Percent(42), Fill()]), botrow)
    _render_cn0(buf, topcols[1], gui_data, num_dots)
    _render_skyplot(buf, topcols[2], gui_data, num_dots)
    _render_position(buf, botcols[1], gui_data, last_fix, show_diag, fresh, ended, num_dots)
    _render_map(m, buf, botcols[2], last_fix, num_dots)

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
# detectors are still holding: Tracking's CN0 estimator reports `-Inf dBHz` for "no
# detectable signal" and a negative dBHz figure just above that (any CN0 under 1 Hz),
# while `Inf`/`NaN` come from a degenerate prompt buffer. Such a satellite draws an empty
# bar instead of taking the whole panel — and with it the dashboard — down.
function _cn0_db(cn0)
    db = 10 * log10(Unitful.linear(cn0) / Hz)
    isfinite(db) ? max(round(db; digits = 1), 0.0) : 0.0
end

# Bar colour for one satellite in the CN0 panel. The three colours separate the reasons a
# tracked satellite may not be contributing to the fix:
#
#   * red    — the satellite declares itself unhealthy in its navigation message. Worst news
#              and unrelated to our own tracking, so it wins over the readiness stage.
#   * yellow — healthy, but the receiver is not ranging on it: either its loops have not
#              settled enough yet (the handover is deliberately tolerated, so this is normal
#              for the first second or so after acquisition — see `is_ranging_ready`), or it
#              has lost lock and is only still on screen because the vector-tracking filter
#              is carrying it through an outage. Both are what `collect_pvt_sat_states!`
#              withholds from the solve, and without this state such a satellite would be
#              indistinguishable from one the solve ignores for no visible reason.
#   * green  — healthy and contributing.
#
# Both conditions are needed. A scalar-tracked satellite is dropped from tracking the moment
# it loses lock, so `is_in_lock` is constant-true for it — but a vector-loop member is not
# dropped, and its `is_ranging_ready` stays latched from before the outage by design, so
# readiness alone would paint a coasting member green.
_sat_bar_color(sat) =
    !sat.is_healthy ? :red : (sat.is_ranging_ready && sat.is_in_lock) ? :green : :yellow

function _render_cn0(buf, area::Rect, gui_data, num_dots)
    # One marker column carries both memberships, which are different sets: the vector loop
    # (`in_vt_loop`) and the satellites whose measurements determined this fix (`pvt.sats`).
    # A loop member missing from `pvt.sats` was coasted through this update — the case worth
    # seeing — so it gets its own glyph rather than sharing `*`. Without VT only the fix
    # membership is left to show. Legends ride in the panel title and appear only once some
    # satellite carries the mark, so a quiet receiver adds no chrome.
    vt_on = gui_data !== nothing && gui_data.vt !== nothing
    in_fix = gui_data === nothing ? Set{Tuple{Symbol,Int}}() : Set(keys(gui_data.pvt.sats))
    title = if vt_on && any(sd -> sd.in_vt_loop, values(gui_data.sat_data))
        CN0_PANEL_TITLE_SHORT * "  (* VT+fix, ∘ VT coasted, · fix)"
    elseif !isempty(in_fix)
        CN0_PANEL_TITLE_SHORT * "  (· = in fix)"
    else
        CN0_PANEL_TITLE
    end
    inner = render(Block(; title = title, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    if gui_data === nothing || isempty(gui_data.sat_data)
        set_string!(buf, inner.x + 1, inner.y, "Searching for satellites$(repeat(".", num_dots))",
            tstyle(:text_dim); max_x = right(inner))
        return
    end
    # Bars sorted by constellation (GPS, then Galileo, …), then PRN, then band; coloured by
    # `_sat_bar_color`. The mark goes right after the PRN (`G03* L1`), with a
    # blank in that column for an unmarked bar, so no space precedes it, all labels stay one
    # width, and `barplot`'s right-justification keeps the PRN/band columns aligned.
    # `sat_label` is `<sys><2-digit prn> <3-char band>`, so the mark goes at index 4. The column
    # is spent only when something could occupy it — a receiver still searching keeps the space
    # for the bars.
    marking = vt_on || !isempty(in_fix)
    sorted_keys = sort(collect(keys(gui_data.sat_data)); by = sat_sort_key)
    # `barplot` draws one row per bar between a leading and a trailing blank row, and
    # `_paint_plot!` clips at the panel's bottom edge, so a panel shorter than the satellite
    # list would quietly show a shorter list — a satellite panel must not lie about how many
    # satellites there are. `inner.height - 1` bars fit (the trailing blank row is the one
    # that may be clipped without loss); past that the last row goes to a continuation marker
    # rather than to one more bar, so the number of missing satellites is always on screen.
    # The shown bars stay the leading slice of the same sort order, so a satellite being
    # acquired or lost never reshuffles the ones already on screen.
    max_bar_rows = max(1, inner.height - 1)
    num_shown = length(sorted_keys) > max_bar_rows ? max(1, max_bar_rows - 1) :
                length(sorted_keys)
    # Slices, not `view`s: at module scope `view` is `Tachikoma.view` (see the module
    # header), and a frame's worth of labels and bars is allocated below regardless.
    hidden_keys = sorted_keys[(num_shown+1):end]
    shown_keys = sorted_keys[1:num_shown]
    labels = map(shown_keys) do key
        lbl = sat_label(key...)
        marking || return lbl
        mark = if vt_on && gui_data.sat_data[key].in_vt_loop
            key in in_fix ? "*" : "∘"
        elseif key in in_fix
            "·"
        else
            " "
        end
        lbl[1:3] * mark * lbl[4:end]
    end
    cn0s = [_cn0_db(gui_data.sat_data[key].cn0) for key in shown_keys]
    colors = [_sat_bar_color(gui_data.sat_data[key]) for key in shown_keys]
    labelw = maximum(length, labels)
    # Chrome `barplot` draws around the bars themselves: the label column, then `" ┤"`
    # between label and bar, then the value printed after the bar (" 45.1" — a space plus up
    # to four characters for `NN.N`). That is 2 + 5 = 7 columns, plus one leading and one
    # trailing column of panel padding, so 9 in total is taken off the panel width.
    barwidth = clamp(inner.width - labelw - 9, 5, 60)
    plot = barplot(labels, cn0s; color = colors, border = :none,
        width = barwidth, maximum = 55)
    _paint_plot!(buf, inner, string(plot; color = true))
    # The marker sits on the row after the last bar (blank row, then one row per bar), in the
    # label column, so it reads as the bar list continuing. It is skipped when even that row
    # is past the panel — a panel with room for a single bar keeps the bar.
    isempty(hidden_keys) && return
    y = inner.y + num_shown + 1
    y > bottom(inner) && return
    x = inner.x + CN0_LABEL_INDENT
    set_string!(buf, x, y, _cn0_continuation(hidden_keys, right(inner) - x + 1),
        tstyle(:text_dim); max_x = right(inner))
    return
end

# The continuation row standing in for the satellites the panel had no room for: a vertical
# ellipsis under the bar labels, how many are hidden, and as many of their names as `width`
# takes. A list cut short ends in `…`, so it is never mistaken for the whole of what is
# hidden — the count ahead of it is what carries that. When not even one name fits, the count
# stands alone.
function _cn0_continuation(hidden_keys, width)
    counted = "⋮  $(length(hidden_keys)) more"
    labels = [strip(sat_label(key...)) for key in hidden_keys]
    # Longest prefix of the list that fits, measured by accumulating widths rather than by
    # rendering each candidate: `": "` before the first name, `", "` between names, and — for
    # every prefix but the whole list — `", …"` for the remainder.
    used = textwidth(counted) + 2
    shown = 0
    for (i, label) in enumerate(labels)
        used += textwidth(label) + (i == 1 ? 0 : 2)
        used + (i == lastindex(labels) ? 0 : 3) <= width || break
        shown = i
    end
    shown == 0 && return counted
    counted * ": " * join(labels[1:shown], ", ") * (shown < length(labels) ? ", …" : "")
end

function _render_skyplot(buf, area::Rect, gui_data, num_dots)
    inner = render(Block(; title = DOA_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    if gui_data === nothing || isnothing(gui_data.pvt.time)
        nsat = gui_data === nothing ? 0 : length(gui_data.sat_data)
        # `nsat` counts tracked (not necessarily decoded) satellites, and the count
        # a fix needs is layout-dependent (3 + one clock per time system + one IFB
        # per extra band), so the panel reports progress and lets the fix itself
        # signal success.
        msg = nsat == 0 ? "Searching for satellites$(repeat(".", num_dots))" :
              "Decoding satellites$(repeat(".", num_dots))"
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
# Each constellation is named by `constellation_name` (GNSSSignals'
# `get_constellation_name`) rather than printing the raw id symbol.
function _legend_ansi(present)
    io = IOContext(IOBuffer(), :color => true)
    for (i, c) in enumerate(present)
        i == 1 || print(io, "   ")
        printstyled(io, "●"; color = get(CONSTELLATION_COLORS, c, :white))
        print(io, " ", constellation_name(c))
    end
    String(take!(io.io))
end

function _render_position(buf, area::Rect, gui_data, last_fix, show_diag, fresh, ended, num_dots)
    inner = render(Block(; title = PVT_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    live = gui_data !== nothing && !isnothing(gui_data.pvt.time)
    fix = live ? gui_data : last_fix
    x, y = inner.x + 1, inner.y
    if fix === nothing
        nsat = gui_data === nothing ? 0 : length(gui_data.sat_data)
        # See `_render_skyplot`: no fixed satellite threshold — just report progress.
        # Run time is already shown in the header, so it is not repeated here. The animated
        # dots match the CN0 and DOA panels (`repeat(".", num_dots)`) so all three "wait" alike.
        msg = nsat == 0 ? "Searching for satellites$(repeat(".", num_dots))" :
              "Decoding satellites$(repeat(".", num_dots))"
        set_string!(buf, x, y, msg, tstyle(:text_dim); max_x = right(inner))
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
    # Run time is shown in the header, so it is not repeated in this panel.
    pvt_rows = [
        ("Time", "$utc_str UTC"),
        ("Coordinates",
            "$(abs(round(lla.lat; digits = 6)))°$lat_hem, " *
            "$(abs(round(lla.lon; digits = 6)))°$lon_hem"),
        ("Altitude", "$(round(lla.alt; digits = 1)) m"),
        ("Speed", "$(round(speed; digits = 2)) m/s"),
        ("Heading", heading_value),
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
function _render_map(m::ReceiverModel, buf, area::Rect, last_fix, num_dots)
    inner = render(Block(; title = MAP_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    if last_fix === nothing
        set_string!(buf, inner.x + 1, inner.y, "awaiting fix$(repeat(".", num_dots))",
            tstyle(:text_dim); max_x = right(inner))
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
