using UnicodePlots

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
            # No redundancy, residuals ~0 by construction: flag it in the header. The
            # whole diagnostics block is rendered dimmed anyway (it is secondary info).
            res_lines[1] *= " (insufficient redundancy)"
            append!(lines, res_lines)
        end
    end

    lines
end

# ─────────────────────────────────────────────────────────────────────────────
# Tachikoma dashboard
#
# The GUI is a Tachikoma app (Elm-style Model/update!/view). A background task drains
# the `GUIData` channel into the model; `view` lays out four panels — CN0 bars and a
# direction-of-arrival sky plot on top, a Position/Velocity/Time block and an
# OpenStreetMap map below — and paints them each frame. The CN0 bars and the sky plot
# are still drawn by `UnicodePlots` (`barplot`/`polarplot`); their colour string is
# painted into the panel via `_paint_plot!`. Interactivity: `d` toggles the PVT
# diagnostics, `+`/`-`/`hjkl`/`0` drive the map, `q`/Ctrl-C quit.

mutable struct ReceiverModel <: Model
    quit::Bool
    tick::Int                       # frame counter, drives the "Searching…" dots
    lk::ReentrantLock
    gui::Union{GUIData,Nothing}     # latest frame from the receiver
    last_fix::Union{GUIData,Nothing}# last frame that carried a real PVT fix
    show_diagnostics::Bool          # PVT diagnostics section (toggled with `d`)
    # Map state (PVT panel). `map_want` is what `view` requests; `_spawn_map` renders it
    # and caches the parsed span-lines in `map_lines`/`map_key`.
    map_zoom::Int
    map_dlon::Float64               # pan offset from the fix, degrees longitude
    map_dlat::Float64               # pan offset from the fix, degrees latitude
    map_lines::Any                  # Vector{Vector{Span}} | nothing
    map_key::Any                    # request key the cached map corresponds to | nothing
    map_want::Any                   # request key the PVT panel wants | nothing
end

ReceiverModel() = ReceiverModel(
    false, 0, ReentrantLock(), nothing, nothing, false, 13, 0.0, 0.0, nothing, nothing, nothing,
)

should_quit(m::ReceiverModel) = m.quit

"""
    gui(gui_data_channel; fps = 12)

Display the receiver dashboard, consuming each `GUIData` from `gui_data_channel`
until the channel closes. Runs a Tachikoma terminal app: a background task keeps the model
fed with the latest frame while the app renders the CN0 bars, the direction-of-arrival sky
plot and the Position/Velocity/Time block (with a live OpenStreetMap map). Blocks until the
stream ends or the user quits (`q`). Keys: `d` toggles the PVT diagnostics; `+`/`-` zoom and
`hjkl` pan the map, `0` recenters it.
"""
function gui(gui_data_channel; fps::Int = 12)
    m = ReceiverModel()
    # Feed the model from the receiver channel; quit the app when the stream ends.
    Base.errormonitor(
        Threads.@spawn begin
            consume_channel(gui_data_channel) do gui_data
                @lock m.lk begin
                    m.gui = gui_data
                    isnothing(gui_data.pvt.time) || (m.last_fix = gui_data)
                end
            end
            @lock m.lk (m.quit = true)
        end
    )
    _spawn_map(m)
    # Prefer the interactive threadpool (`julia -t auto,1`) so the render loop is not
    # starved by the streaming/DSP tasks on the default pool.
    if Threads.nthreads(:interactive) > 0
        wait(Threads.@spawn :interactive app(m; fps))
    else
        app(m; fps)
    end
end

# ── Input ─────────────────────────────────────────────────────────────────────
function update!(m::ReceiverModel, e::KeyEvent)
    if e.key == :ctrl_c || e.key == :escape || (e.key == :char && (e.char == 'q' || e.char == 'Q'))
        m.quit = true
        return
    end
    if e.key == :char && (e.char == 'd' || e.char == 'D')
        @lock m.lk (m.show_diagnostics = !m.show_diagnostics)
        return
    end
    # Map controls: `+`/`-` zoom, `hjkl` pan (vim), `0` recenter. `←/→` are free for
    # future navigation; the map uses hjkl to avoid clobbering them.
    if e.key == :char
        c = e.char
        if c == '+' || c == '='
            @lock m.lk (m.map_zoom = clamp(m.map_zoom + 1, 1, 18))
        elseif c == '-' || c == '_'
            @lock m.lk (m.map_zoom = clamp(m.map_zoom - 1, 1, 18))
        elseif c == '0'
            @lock m.lk begin
                m.map_zoom = 13
                m.map_dlon = 0.0
                m.map_dlat = 0.0
            end
        elseif c == 'h' || c == 'j' || c == 'k' || c == 'l'
            @lock m.lk begin
                step = 0.35 * 360.0 / 2.0^m.map_zoom   # ~⅓ view per press, scales with zoom
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
const CN0_PANEL_TITLE = "Carrier-to-Noise-Density-Ratio (CN0)"
const DOA_PANEL_TITLE = "Satellite Direction-of-Arrival (DOA)"
const PVT_PANEL_TITLE = "Position Velocity Time (PVT)"
const MAP_PANEL_TITLE = "Map"
const NOT_ENOUGH_SATS_TEXT = "Not enough satellites to calculate position."

function view(m::ReceiverModel, f::Frame)
    m.tick += 1
    gui_data, last_fix, show_diag = @lock m.lk (m.gui, m.last_fix, m.show_diagnostics)
    buf = f.buffer
    num_dots = mod(m.tick ÷ 4, 4)

    rows = split_layout(Layout(Vertical, [Fixed(1), Fill(), Fixed(1)]), f.area)
    header, body, footer = rows[1], rows[2], rows[3]

    fresh = gui_data !== nothing && gui_data.pvt_fresh
    # "stale" only once a fix is being held frozen (a re-emitted old solution) — not while
    # still searching/decoding, where there is simply no fix yet.
    has_fix = gui_data !== nothing && gui_data.pvt !== nothing && !isnothing(gui_data.pvt.time)
    rt = gui_data === nothing ? 0.0 : round(ustrip(s, gui_data.runtime); digits = 1)
    hdr = " ● GNSSReceiver  │  run time $(rt) s" *
          (has_fix && !fresh ? "  │  stale (no new fix)" : "")
    set_string!(buf, header.x, header.y, rpad(hdr, header.width),
        tstyle(:title, bold = true); max_x = right(header))

    # Body: CN0 | DOA (top), PVT | Map (bottom) — the same 2×2 proportions as the
    # presentation's PVT slide.
    toprow, botrow = split_layout(Layout(Vertical, [Percent(50), Fill()]), body)
    topcols = split_layout(Layout(Horizontal, [Percent(50), Fill()]), toprow)
    botcols = split_layout(Layout(Horizontal, [Percent(42), Fill()]), botrow)
    _render_cn0(buf, topcols[1], gui_data, num_dots)
    _render_skyplot(buf, topcols[2], gui_data, num_dots)
    _render_position(buf, botcols[1], gui_data, last_fix, show_diag, fresh)
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
# left-to-right, clipping at the panel edges. The same technique the map uses.
function _paint_plot!(buf, area::Rect, str::AbstractString)
    for (i, line) in enumerate(split(str, '\n'))
        y = area.y + i - 1
        y > bottom(area) && break
        x = area.x
        for sp in parse_ansi(String(line))
            x > right(area) && break
            set_string!(buf, x, y, sp.content, sp.style; max_x = right(area))
            x += max(1, textwidth(sp.content))
        end
    end
    return
end

# CN0 in dBHz as a plain rounded number (Inf CN0 → 0 dB reference), matching the old GUI.
_cn0_db(cn0) = round(10 * log10(Unitful.linear(cn0 == Inf * Hz ? 1Hz : cn0) / Hz); digits = 1)

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
    barwidth = clamp(inner.width - labelw - 9, 5, 60)
    plot = barplot(labels, cn0s; color = colors, border = :none,
        width = barwidth, maximum = 55)
    _paint_plot!(buf, inner, string(plot; color = true))
    return
end

function _render_skyplot(buf, area::Rect, gui_data, num_dots)
    inner = render(Block(; title = DOA_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    pvt = gui_data === nothing ? nothing : gui_data.pvt
    if pvt === nothing || isnothing(pvt.time)
        nsat = gui_data === nothing ? 0 : length(gui_data.sat_data)
        msg = nsat < 4 ? NOT_ENOUGH_SATS_TEXT : "Decoding satellites$(repeat(".", num_dots))"
        set_string!(buf, inner.x + 1, inner.y, msg, tstyle(:text_dim); max_x = right(inner))
        return
    end
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
    plotwidth = clamp(min(plotarea.width, 2 * plotarea.height) - 2, 10, 60)
    doa_plot = polarplot(azs, els_deg; rlim = (0, 90), scatter = true,
        marker = :circle, color = point_colors, border = :none, width = plotwidth)
    # Label each satellite with its PRN number (coloured by constellation) placed exactly
    # on its point. `polarplot` plots θ (az) counter-clockwise from +x at radius r (=el),
    # so the point's Cartesian position is (el·cos az, el·sin az) — annotate there.
    for (az, el, prn, col) in zip(azs, els_deg, prns, point_colors)
        annotate!(doa_plot, el * cos(az), el * sin(az), string(prn); color = col)
    end
    # Re-label the angular axis to the GNSS azimuth convention: 0° = North on top,
    # increasing clockwise (90° = East right, 180° = South bottom, 270° = West left).
    grid_color = UnicodePlots.BORDER_COLOR[]
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

function _render_position(buf, area::Rect, gui_data, last_fix, show_diag, fresh)
    inner = render(Block(; title = PVT_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    live = gui_data !== nothing && gui_data.pvt !== nothing && !isnothing(gui_data.pvt.time)
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
        set_string!(buf, x, y, "◦ last fix (re-acquiring)", tstyle(:warning); max_x = right(inner))
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

# ── Map ───────────────────────────────────────────────────────────────────────
function _render_map(m::ReceiverModel, buf, area::Rect, last_fix)
    inner = render(Block(; title = MAP_PANEL_TITLE, border_style = tstyle(:border),
            title_style = tstyle(:accent, bold = true)), area, buf)
    (inner.width < 8 || inner.height < 4) && return
    if last_fix === nothing
        set_string!(buf, inner.x + 1, inner.y, "awaiting fix…", tstyle(:text_dim);
            max_x = right(inner))
        return
    end
    lla = try
        get_LLA(last_fix.pvt)
    catch
        nothing
    end
    lla === nothing && return
    zoom, dlon, dlat = @lock m.lk (m.map_zoom, m.map_dlon, m.map_dlat)
    clon = lla.lon + dlon
    clat = lla.lat + dlat
    marker = dlon == 0.0 && dlat == 0.0          # the pin marks the map centre = the fix
    want = (round(clat; digits = 5), round(clon; digits = 5),
        inner.width, inner.height, zoom, marker)
    lines = @lock m.lk begin
        m.map_want = want
        m.map_lines
    end
    if lines === nothing
        # No cached tile yet: show the coordinates + a maps link as a graceful fallback
        # (also what stays on screen when there is no network).
        set_string!(buf, inner.x + 1, inner.y, "loading map…", tstyle(:text_dim);
            max_x = right(inner))
        set_string!(buf, inner.x + 1, inner.y + 1,
            "$(round(clat; digits = 5)), $(round(clon; digits = 5))",
            tstyle(:success, bold = true); max_x = right(inner))
        set_string!(buf, inner.x + 1, inner.y + 2,
            "maps.google.com/?q=$(round(clat; digits = 5)),$(round(clon; digits = 5))",
            tstyle(:text_dim); max_x = right(inner))
        return
    end
    for (i, spans) in enumerate(lines)
        yy = inner.y + i - 1
        yy > bottom(inner) && break
        xx = inner.x
        for sp in spans
            xx > right(inner) && break
            set_string!(buf, xx, yy, sp.content, sp.style; max_x = right(inner))
            xx += max(1, textwidth(sp.content))
        end
    end
    return
end

# Render the map on a background task (network tile download + ANSI parse, a few seconds),
# once per (position, panel-size, zoom); the PVT panel only ever paints the cached span
# lines. On any failure (offline, tile error) the panel keeps showing the coordinate
# fallback, and we mark the request done so a failing request is not retried in a tight loop.
function _spawn_map(m::ReceiverModel)
    Base.errormonitor(
        Threads.@spawn begin
            while !m.quit
                want, have = @lock m.lk (m.map_want, m.map_key)
                if want !== nothing && want != have
                    lat, lon, w, h, zoom, marker = want
                    if w >= 8 && h >= 4
                        try
                            img = worldmap(; center = (lon, lat), zoom = Int(zoom),
                                size = (Int(w), Int(h)), marker = marker)
                            parsed = [parse_ansi(String(l)) for l in split(sprint(show, img), "\n")]
                            @lock m.lk begin
                                m.map_lines = parsed
                                m.map_key = want
                            end
                        catch
                            @lock m.lk (m.map_key = want)   # don't retry a failing request
                        end
                    end
                end
                sleep(0.5)
            end
        end
    )
end