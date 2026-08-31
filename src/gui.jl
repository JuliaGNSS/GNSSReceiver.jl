using UnicodePlots

struct GUIData{S<:SatelliteDataOfInterest}
    sat_data::Dictionary{Tuple{Symbol,Int},S}
    pvt::PVTSolution
    runtime::typeof(1.0s)
    # Whether this data carries a *new* PVT solution.
    pvt_fresh::Bool
    # Vector-tracking status (see `VTStatus`), or `nothing` when VT is disabled.
    vt::Union{Nothing,VTStatus}
end

# Back-compat / convenience: GUI data built without VT status defaults to disabled.
GUIData(
    sat_data::Dictionary{Tuple{Symbol,Int},S},
    pvt,
    runtime,
    pvt_fresh,
) where {S<:SatelliteDataOfInterest} =
    GUIData{S}(sat_data, pvt, runtime, pvt_fresh, nothing)

# Satellite labels follow the RINEX-3 convention: the satellite is identified by
# its system letter + zero-padded PRN (`G30`, `E24`, `R05`, `C21` — the same code
# used in RINEX files, IGS products and most receivers), and the signal's frequency
# band is appended after a space (`G30 L5`). This keeps the satellite ID
# unambiguous — `G05` is GPS PRN 5, never "GPS L5" — and drops the I/Q/CA signal
# component for compactness. Band tokens use the familiar frequency-band names
# (GPS L1/L2/L5, Galileo E1/E5a/E5b/E6, BeiDou B1I/B1C/B2a/B2b/B3I); the modernized GPS
# L1C civil signal keeps "L1C" so it is not confused with L1 C/A ("L1"). The band token is
# right-padded to the widest name (3 chars — "E5a", "L1C", "B2a" and the rest) so every
# label is the same width and the bars line up in a column regardless of band. Every PRN
# still fits two digits, BeiDou's 63 included.
const CONSTELLATION_LETTERS =
    Dict(:GPS => "G", :Galileo => "E", :GLONASS => "R", :BeiDou => "C", :Other => "?")

# Band token per signal. Deliberately *not* `GNSSSignals.get_band_name`: band identity
# there is by RF frequency, so that names the band — `get_band_name(GalileoE1B)` is "L1",
# not "E1" — whereas a satellite label wants the constellation's own ICD label for the
# carrier. The ICD label is a per-(constellation, band) fact rather than a band fact
# (Galileo calls 1575.42 MHz E1 and 1176.45 MHz E5a), so it is stated here. The "L1C"
# entries are a further display choice of ours: IS-GPS-800 calls that band L1, but the bar
# chart spells the modernized signal "L1C" so it cannot be misread as L1 C/A.
#
# BeiDou is the one constellation whose band label alone would be ambiguous, so its
# tokens are the ICD's *signal* labels: BDS calls both 1561.098 MHz and 1575.42 MHz "B1"
# and both 1176.45 MHz and 1207.14 MHz "B2", so "B1"/"B2" would name two different
# carriers each. "B1I", "B1C", "B2a", "B2b" and "B3I" are the labels the five ICDs are
# titled with, and the ones RINEX 3 distinguishes those carriers by.
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
    :GalileoE5bI => "E5b",
    :GalileoE5bQ => "E5b",
    :GalileoE6B => "E6",
    :GalileoE6C => "E6",
    :BeiDouB1I => "B1I",
    :BeiDouB1C_D => "B1C",
    :BeiDouB1C_P => "B1C",
    :BeiDouB2aI => "B2a",
    :BeiDouB2aQ => "B2a",
    :BeiDouB2bI => "B2b",
    :BeiDouB3I => "B3I",
)

# Every signal the GUI knows how to display, listed in the order the CN0 bar chart shows
# them: grouped by constellation (see `CONSTELLATION_ORDER`), then by band (ascending band
# number), then data component before pilot. Listed as *types*, never instances —
# constructing a signal reads its code file from disk and bakes every PRN's table, while
# every GNSSSignals accessor takes the type just as happily.
#
# The tables below are derived from this one list through those accessors, so each signal's
# constellation (`get_constellation_id`) and display rank (its position here) is stated
# once: teaching the GUI about a new signal is one line here, not an entry in three tables.
const DISPLAYED_SIGNALS = (
    GPSL1CA,
    GPSL1C_D,
    GPSL1C_P,
    GPSL2CM,
    GPSL2CL,
    GPSL5I,
    GPSL5Q,
    GalileoE1B,
    GalileoE1C,
    # The BOC(1,1) approximations rank next to the full-CBOC E1B/E1C they stand in for,
    # for the same reason they share their band token.
    GalileoE1B_BOC11,
    GalileoE1C_BOC11,
    GalileoE5aI,
    GalileoE5aQ,
    # `GalileoE5aQP` is deliberately absent: the E5a quasi-pilot is an acquisition aid
    # (OS SIS ICD Issue 2.2 §2.3.1.4), not a component to range on — its 330-chip code
    # repeats every 2/31 ms — and this receiver has no acquisition path that uses it.
    GalileoE5bI,
    GalileoE5bQ,
    GalileoE6B,
    GalileoE6C,
    # BeiDou by ascending band number, which puts the legacy B1I before B1C on the other
    # B1 carrier just as GPS L1 C/A precedes L1C.
    BeiDouB1I,
    BeiDouB1C_D,
    BeiDouB1C_P,
    BeiDouB2aI,
    BeiDouB2aQ,
    BeiDouB2bI,
    BeiDouB3I,
)

# Constellation of each displayed signal, from GNSSSignals' `get_constellation_id`: which
# constellation broadcasts a signal is the signal's own fact, not something to be recovered
# from the spelling of its id.
const SIGNAL_CONSTELLATIONS =
    Dict(get_signal_id(S) => get_constellation_id(S) for S in DISPLAYED_SIGNALS)

# Display rank of each signal — its position in `DISPLAYED_SIGNALS`. Only ever compared
# within one constellation and PRN (see `sat_sort_key`), so one global ordering suffices.
const SIGNAL_ORDER = Dict(get_signal_id(S) => i for (i, S) in enumerate(DISPLAYED_SIGNALS))

# Display name per constellation, from `get_constellation_name` — so a constellation whose
# name differs from its id (a future `:NavIC` reading "NavIC (IRNSS)") reads correctly in
# the DOA legend without a table here.
const CONSTELLATION_NAMES =
    Dict(get_constellation_id(S) => get_constellation_name(S) for S in DISPLAYED_SIGNALS)

# Constellation of a `get_signal_id` symbol, used to group, order and colour satellites in
# the CN0 bars and the DOA plot. Keyed by symbol because that is what `sat_data` and
# `pvt.sats` carry — the signal *type* is long gone by the time the GUI runs. A signal the
# GUI does not know (e.g. one defined downstream) falls into `:Other`, which the letter,
# order and colour tables all carry an entry for, so the GUI degrades instead of erroring.
constellation_of(signal_id::Symbol) = get(SIGNAL_CONSTELLATIONS, signal_id, :Other)

# Display name of a constellation id, for the DOA legend. An unknown constellation prints
# its id.
constellation_name(constellation::Symbol) =
    get(CONSTELLATION_NAMES, constellation, string(constellation))

function sat_label(system_key::Symbol, prn::Integer)
    sys = get(CONSTELLATION_LETTERS, constellation_of(system_key), "?")
    band = get(BAND_ABBREVIATIONS, system_key, string(system_key))
    string(sys, lpad(prn, 2, '0'), " ", rpad(band, 3))
end

# Display order for the CN0 bar chart: by constellation (GPS, then Galileo, then the
# rest), then PRN, then signal (`SIGNAL_ORDER`: ascending band within a constellation, data
# component before pilot). Unlisted constellations/signals sort last (rank 99) but keep a
# stable order among themselves.
const CONSTELLATION_ORDER =
    Dict(:GPS => 1, :Galileo => 2, :GLONASS => 3, :BeiDou => 4, :Other => 5)

# Sort key for a `(get_signal_id, prn)` satellite-data key: constellation, then PRN,
# then signal.
sat_sort_key((system_key, prn)::Tuple{Symbol,Int}) = (
    get(CONSTELLATION_ORDER, constellation_of(system_key), 99),
    prn,
    get(SIGNAL_ORDER, system_key, 99),
)

# Every frequency band the GUI ranks and labels, used for the inter-frequency biases. As
# types, for the same reason as `DISPLAYED_SIGNALS`. The order written here is immaterial —
# `BAND_ORDER` sorts them — so this is simply the set of bands GNSSSignals defines.
const DISPLAYED_BANDS = (L1, L2, L5, B1I, B3I, E5b, E6)

# Rank and display name per band, keyed by `get_band_id` — exactly what
# `pvt.inter_frequency_biases` is keyed by. The name comes from `get_band_name`; here it is
# the band's own label that is wanted (an inter-frequency bias is one RF-chain delay of one
# shared carrier, so it has no constellation), unlike the per-signal `BAND_ABBREVIATIONS`.
# An unlisted band sorts last (rank 99) and prints its id.
#
# Ranked by *descending* carrier frequency — the only rule that survives bands from more
# than one constellation, since a band is one shared RF carrier with no constellation of
# its own (1207.14 MHz is E5b to Galileo and B2b to BeiDou). Sorted rather than
# hand-ordered, so adding a band above cannot put it in the wrong place. No tie-break is
# needed: band identity in GNSSSignals *is* the carrier, so two `Band`s on one frequency
# would be one `Band`; the test asserting `length(BAND_ORDER) == length(DISPLAYED_BANDS)`
# catches a collision on `get_band_id` should that invariant ever break.
const BAND_ORDER = Dict(
    get_band_id(B) => i for (i, B) in
    enumerate(sort(collect(DISPLAYED_BANDS); by = B -> -get_center_frequency(B)))
)
const BAND_NAMES = Dict(get_band_id(B) => get_band_name(B) for B in DISPLAYED_BANDS)

band_name(band::Symbol) = get(BAND_NAMES, band, string(band))

# Time systems in display order (GPS < Galileo < BeiDou), mirroring `CONSTELLATION_ORDER`,
# used to rank the inter-system biases. Keyed by `get_time_system_id`, which is also how a
# `pvt.inter_system_biases` key — a `TimeSystem` *instance* — reduces to a symbol.
const DISPLAYED_TIME_SYSTEMS = (GPST, GST, BDT)
const TIME_SYSTEM_ORDER =
    Dict(get_time_system_id(T) => i for (i, T) in enumerate(DISPLAYED_TIME_SYSTEMS))

# Re-express every inter-frequency bias against the lowest-ordered band (by `BAND_ORDER`)
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
    rank(b) = get(BAND_ORDER, b, 99)
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
    rank(s) = get(TIME_SYSTEM_ORDER, get_time_system_id(s), 99)
    ref = argmin(rank, keys(bias))
    for sys in keys(bias)
        sys == ref || push!(out, (sys, bias[sys] - bias[ref], ref))
    end
    sort!(out; by = t -> rank(t[1]))
    return out
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
    get_gui_data_channel(data_channel, push_gui_data_roughly_every = 500ms)

Return a `Channel{GUIData}` that downsamples `data_channel` for display: a spawned task
consumes every [`ReceiverDataOfInterest`](@ref) but only forwards one roughly every
`push_gui_data_roughly_every` of signal runtime (plus the very first), so the GUI is
refreshed at a human rate rather than once per processed chunk.

The forwarding task also propagates a *downstream* close upstream: if the consumer closes
the returned channel — which [`gui`](@ref GNSSReceiver.gui) does when the user quits — it
closes `data_channel`, ending the run instead of leaving the pipeline churning through the
rest of the stream with nobody watching.
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
            try
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
                            GUIData(data.sat_data, data.pvt, data.runtime, pvt_fresh, data.vt),
                        )
                        last_gui_output[] = data.runtime
                        first[] = false
                    end
                end
                close(gui_data_channel)
            catch e
                # The only expected failure is the `push!` above hitting a channel the
                # consumer has closed (the user quit the GUI). Tear the run down from here:
                # closing `data_channel` ends `receive`'s processing loop, which in turn
                # stops the sample reader.
                e isa InvalidStateException || rethrow()
                close(data_channel)
            end
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
# biases (metres), and residual RMS in both domains — range and range rate, overall
# and broken down per signal, which is why the block is named for the residuals
# rather than for the pseudoranges one of its two columns is about. Appended
# below the position/velocity/time block in the combined panel. The residuals are
# greyed out until the solution is over-determined — with only
# `3 + #time-systems + #extra-bands` satellites the least-squares residual is ~0 by
# construction, so its RMS is meaningless. Sections that don't apply (no
# inter-system bias for a single constellation, no inter-frequency bias for a single
# band) are omitted. Only called with a fix present, so no "waiting" fallback.
function pvt_details_lines(pvt)
    lines = String[]

    if !isnothing(pvt.dop)
        d = pvt.dop
        push!(lines, "GDOP: $(_fmt2(d.GDOP))   PDOP: $(_fmt2(d.PDOP))")
        push!(lines, "HDOP: $(_fmt2(d.HDOP))   VDOP: $(_fmt2(d.VDOP))   TDOP: $(_fmt2(d.TDOP))")
    end

    # Inter-system biases re-anchored on the lowest-ordered time system present (GPS <
    # Galileo < …) rather than `calc_pvt`'s reference system, so the displayed anchor is
    # stable across solves. All share that one anchor, so it goes in the heading.
    # Anchor and rows both use the compact `get_time_system_id` (`GPST`), not the spelled-out
    # `get_time_system_name` ("GPS Time"): this panel is 42% of the terminal, so at an
    # 80-column terminal it has 30 usable columns — exactly the width of this heading with an
    # id. A name overruns and is clipped ("(vs GPS Ti"), and the longer the scale's name the
    # worse it gets — "Galileo System Time" already clips at 100 columns, and it becomes the
    # anchor as soon as a third time system is tracked without GPS. Ids stay bounded.
    isbs = isbs_vs_lowest_system(pvt)
    if !isempty(isbs)
        push!(lines, "Inter-system biases (vs $(get_time_system_id(isbs[1][3]))):")
        for (sys, bias, _) in isbs
            push!(lines, "  $(get_time_system_id(sys)): $(_fmt2(ustrip(m, bias))) m")
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
            push!(
                lines,
                "  $(band_name(band)) (vs $(band_name(reference))): " *
                "$(_fmt2(ustrip(m, value))) m",
            )
        end
    end

    if !isempty(pvt.sats)
        n = length(pvt.sats)
        # Estimated unknowns: 3 position + one clock per time system (reference +
        # each inter-system bias) + one per extra band.
        num_unknowns = 3 + (1 + length(pvt.inter_system_biases)) + length(pvt.inter_frequency_biases)
        # Range and range-rate residuals side by side: the rate column is the Doppler-domain
        # counterpart, so a satellite consistent in range but not in rate (a cycle slip, or a
        # replica the loop is dragging) shows up as a rate RMS out of scale with the range one.
        residuals = [ustrip(m, info.residual) for info in values(pvt.sats)]
        rate_residuals = [ustrip(m/s, info.rate_residual) for info in values(pvt.sats)]
        # Table rows (label, RMS, rate RMS, count): overall first, then one per signal.
        rows = Tuple{String,Float64,Float64,Int}[
            ("overall", _rms(residuals), _rms(rate_residuals), n),
        ]
        # Drawn as children of the `overall` row above, so the table reads as one figure and
        # its breakdown — the per-signal counts sum to the overall `n` — rather than as a
        # column of unrelated numbers where `overall` is just the widest label.
        sigs = unique(first(key) for key in keys(pvt.sats))
        for (i, sig) in enumerate(sigs)
            sig_keys = [key for key in keys(pvt.sats) if first(key) == sig]
            # Label each per-signal row by its band abbreviation (e.g. `GalileoE1C_BOC11`
            # → "E1"), falling back to the raw id for an unlisted signal.
            push!(
                rows,
                (
                    (i == lastindex(sigs) ? "└ " : "├ ") *
                    get(BAND_ABBREVIATIONS, sig, string(sig)),
                    _rms([ustrip(m, pvt.sats[key].residual) for key in sig_keys]),
                    _rms([ustrip(m/s, pvt.sats[key].rate_residual) for key in sig_keys]),
                    length(sig_keys),
                ),
            )
        end
        # The title takes a line of its own rather than heading the label column: folding it
        # in would widen that column to the title's length (47 columns against 34 here) and
        # leave a gutter between every label and its figures, and the panel is 42% of the
        # terminal. The label column then needs no heading — the rows name themselves.
        # `textwidth`, not `length`, as `rpad`/`lpad` pad in display columns: the branch
        # glyphs are one column each, but a label counted in characters would misalign a
        # wider one. Both value columns are as wide as their own heading (`range/m` is one
        # column wider than the figures it heads), so heading and figures right-align on the
        # same edge.
        namew = maximum(textwidth(first(row)) for row in rows)
        res_lines = [
            # The statistic goes in the title, which is now free of the table's geometry: it
            # holds for both columns, and spelling it into the headings ("range RMS/m") would
            # widen them by the length of the word.
            "Measurement residuals (RMS):",
            "  $(" "^namew)  $(lpad("range/m", 7))  $(lpad("rate/(m/s)", 10))   n",
        ]
        for (label, rms, rate_rms, cnt) in rows
            push!(
                res_lines,
                "  $(rpad(label, namew))  $(lpad(_fmt2(rms), 7))  " *
                "$(lpad(_fmt2(rate_rms), 10))  $(lpad(cnt, 2))",
            )
        end
        # No redundancy, residuals ~0 by construction: flag it under the table rather than on
        # the title line, where the two together run to 53 columns and a narrow panel clips
        # the caveat off the end — the one part of this block that must not go missing. On
        # its own line it is 27. The whole diagnostics block is rendered dimmed anyway (it is
        # secondary info).
        n > num_unknowns || push!(res_lines, "  (insufficient redundancy)")
        append!(lines, res_lines)
    end

    lines
end
