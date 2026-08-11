@testset "Get GUI data from data channel" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{sat_data_type}}() do ch
        foreach(1:100) do i
            data = GNSSReceiver.ReceiverDataOfInterest{sat_data_type}(
                Dictionary(
                    Dict{Tuple{Symbol,Int},sat_data_type}(
                        (:GPSL1CA, 1) => sat_data_type(
                            45.0dBHz,
                            SVector(complex(1.0, 2.0), complex(2.0, 3.0)),
                            true,
                        ),
                    ),
                ),
                GNSSReceiver.PVTSolution(),
                (i - 1) * 0.004u"s",
            )
            put!(ch, data)
        end
    end

    gui_data_channel = get_gui_data_channel(data_channel, 200ms)

    gui_datas = collect(gui_data_channel)
    @test length(gui_datas) == 2
    @test length(gui_datas[1].sat_data) == 1
    @test first(gui_datas[1].sat_data[(:GPSL1CA, 1)].cn0) == 45dBHz
    @test length(gui_datas[2].sat_data) == 1
    @test first(gui_datas[2].sat_data[(:GPSL1CA, 1)].cn0) == 45dBHz
    @test isnothing(gui_datas[1].pvt.time)
end

@testset "Closing the GUI channel stops the pipeline" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    # An endless producer, standing in for a receiver replaying a long recording.
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{sat_data_type}}() do ch
        runtime = 0.0u"s"
        while true
            runtime += 1.0u"s"
            put!(
                ch,
                GNSSReceiver.ReceiverDataOfInterest{sat_data_type}(
                    Dictionary{Tuple{Symbol,Int},sat_data_type}(),
                    GNSSReceiver.PVTSolution(),
                    runtime,
                ),
            )
        end
    end

    gui_data_channel = get_gui_data_channel(data_channel, 200ms)
    take!(gui_data_channel)     # one frame reaches the GUI
    close(gui_data_channel)     # ... and then the user quits with `q`

    # `gui` closes its channel on exit, and the forwarding task propagates that close
    # upstream, so the producer is shut down instead of grinding on with nobody watching.
    timedwait(() -> !isopen(data_channel), 10.0)
    @test !isopen(data_channel)
end

# Render one frame of the Tachikoma dashboard for `model` into an off-screen buffer and
# return it as plain text. This drives the real `view` (CN0 bars, sky plot, PVT block, map
# fallback) without a TTY — the map background task is only spawned by `gui`, not `view`,
# so no network is touched and `map_ansi` stays empty (the map panel shows its coordinate
# fallback).
using Tachikoma: Rect, Buffer, Frame, GraphicsRegion, ColorRGBA, buffer_to_text

function render_gui_text(model; width = 140, height = 50)
    area = Rect(1, 1, width, height)
    buf = Buffer(area)
    frame = Frame(buf, area, GraphicsRegion[], Tuple{Int,Int,Matrix{ColorRGBA}}[])
    GNSSReceiver.Dashboard.view(model, frame)
    buffer_to_text(buf, area)
end

function gui_model(gui_data; show_diagnostics = false, last_fix = gui_data)
    m = GNSSReceiver.Dashboard.ReceiverModel()
    m.gui = gui_data
    isnothing(last_fix.pvt.time) || (m.last_fix = last_fix)
    m.show_diagnostics = show_diagnostics
    m
end

# A minimal `GUIData` carrying a real fix — four GPS L1 satellites plus a position/velocity/
# time solution — for the header- and held-fix tests below.
function fixed_gui_data(; runtime = 10.0u"s", pvt_fresh = true)
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    sat_keys = [(:GPSL1CA, 3), (:GPSL1CA, 12), (:GPSL1CA, 23), (:GPSL1CA, 10)]
    GNSSReceiver.GUIData(
        Dictionary(
            sat_keys,
            [sat_data_type(45.0dBHz, zeros(SVector{2,ComplexF64}), true) for _ in sat_keys],
        ),
        PositionVelocityTime.PVTSolution(;
            position = ECEF(4.0e6, 3.9e5, 4.9e6),
            velocity = ECEF(2.0e6, 2.9e5, 1.9e6),
            time = TAIEpoch(2022, 10, 8),
            dop = PositionVelocityTime.DOP(1.0, 1.0, 1.0, 1.0, 1.0),
            sats = Dictionary(
                sat_keys,
                [
                    PositionVelocityTime.SatInfo(
                        ECEF(5e6 + 1e5 * i, 3e6 - 1e5 * i, 1e6 + 2e5 * i),
                        0.0,
                        0.0u"m",
                    ) for i in eachindex(sat_keys)
                ],
            ),
        ),
        runtime,
        pvt_fresh,
    )
end

@testset "GUI input handling (update!)" begin
    using Tachikoma: KeyEvent
    using GNSSReceiver.Dashboard: ReceiverModel, update!

    # Quit on q / Q / Ctrl-C — the keys the docs advertise.
    for e in (KeyEvent('q'), KeyEvent('Q'), KeyEvent(:ctrl_c))
        m = ReceiverModel()
        update!(m, e)
        @test m.quit
    end

    # Esc is deliberately *not* bound to quit: it is far too easy to hit by reflex to tear
    # down a running receiver with.
    m = ReceiverModel()
    update!(m, KeyEvent(:escape))
    @test !m.quit

    # Diagnostics are shown by default; `d` toggles them; an unbound key is a no-op.
    m = ReceiverModel()
    @test m.show_diagnostics
    update!(m, KeyEvent('d'))
    @test !m.show_diagnostics
    update!(m, KeyEvent('d'))
    @test m.show_diagnostics
    update!(m, KeyEvent('x'))
    @test !m.quit && m.show_diagnostics
end

@testset "GUI map controls (update!)" begin
    using Tachikoma: KeyEvent
    using GNSSReceiver.Dashboard:
        ReceiverModel, update!, MAP_DEFAULT_ZOOM, MAP_MIN_ZOOM, MAP_MAX_ZOOM

    # Zoom: `+`/`-`, clamped to the tile levels that exist.
    m = ReceiverModel()
    update!(m, KeyEvent('+'))
    @test m.map_zoom == MAP_DEFAULT_ZOOM + 1
    update!(m, KeyEvent('-'))
    @test m.map_zoom == MAP_DEFAULT_ZOOM
    foreach(_ -> update!(m, KeyEvent('-')), 1:30)
    @test m.map_zoom == MAP_MIN_ZOOM
    foreach(_ -> update!(m, KeyEvent('+')), 1:30)
    @test m.map_zoom == MAP_MAX_ZOOM

    # Pan: hjkl move the centre offset, opposite keys cancel out.
    m = ReceiverModel()
    update!(m, KeyEvent('l'))
    @test m.map_dlon > 0
    update!(m, KeyEvent('h'))
    @test isapprox(m.map_dlon, 0.0; atol = 1e-9)
    update!(m, KeyEvent('k'))
    @test m.map_dlat > 0
    update!(m, KeyEvent('j'))
    @test isapprox(m.map_dlat, 0.0; atol = 1e-9)

    # `0` recenters on the fix and resets the zoom, whatever the map was showing.
    m.map_zoom = 5
    update!(m, KeyEvent('k'))
    update!(m, KeyEvent('0'))
    @test m.map_zoom == MAP_DEFAULT_ZOOM
    @test m.map_dlon == 0.0 && m.map_dlat == 0.0
    @test !m.quit   # panning and zooming never quit
end

@testset "GUI map panel paints a rendered tile" begin
    # Once the background task has rendered a tile, the panel paints it instead of the
    # coordinate fallback. Feeding `map_ansi` directly keeps the test off the network.
    m = gui_model(fixed_gui_data())
    m.map_ansi = join(("map row $i" for i in 1:6), "\n")
    out = render_gui_text(m)
    @test occursin("map row 1", out) && occursin("map row 6", out)
    @test !occursin("google.com/maps?q=", out)
end

@testset "GUI header flags stream ended" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    gui_data = GNSSReceiver.GUIData(
        Dictionary{Tuple{Symbol,Int},sat_data_type}(),
        GNSSReceiver.PVTSolution(),
        5.0u"s",
        true,
    )
    m = gui_model(gui_data)
    m.stream_ended = true   # channel closed: the last frame is held, not stale
    out = render_gui_text(m)
    @test occursin("stream ended", out)
    @test !occursin("stale", out)
end

@testset "GUI header flags a stale fix" begin
    # A fix is on screen, but the receiver produced no new one for this frame
    # (`pvt_fresh = false`) and the stream is still running: that — and only that — is
    # "stale". The stream-ended case above must not use this wording.
    out = render_gui_text(gui_model(fixed_gui_data(; pvt_fresh = false)))
    @test occursin("stale (no new fix)", out)
    @test !occursin("stream ended", out)
end

@testset "GUI holds the last fix when the current frame has none" begin
    fix = fixed_gui_data(; runtime = 10.0u"s")
    # Current frame carries no fix (`pvt.time === nothing`) but an earlier one did, so the
    # PVT panel falls back to the held fix and flags why it is frozen.
    searching =
        GNSSReceiver.GUIData(fix.sat_data, GNSSReceiver.PVTSolution(), 20.0u"s", false)
    m = gui_model(searching; last_fix = fix)

    out = render_gui_text(m)
    @test occursin("last fix (re-acquiring)", out)
    # The held fix's coordinates stay on screen, stamped with the *current* run time.
    @test occursin("Coordinates: 50.830895°N, 5.568737°E", out)
    @test occursin("Run time:", out) && occursin("20.0 s", out)

    # Once the stream has ended nothing is being re-acquired any more — the panel must not
    # claim otherwise while the header says "stream ended".
    m.stream_ended = true
    out = render_gui_text(m)
    @test occursin("last fix (stream ended)", out)
    @test !occursin("re-acquiring", out)
end

@testset "GUI with no data" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    gui_data = GNSSReceiver.GUIData(
        Dictionary{Tuple{Symbol,Int},sat_data_type}(),
        GNSSReceiver.PVTSolution(),
        0.0u"s",
        true,
    )
    out = render_gui_text(gui_model(gui_data))
    @test occursin("Searching for satellites", out)
    @test occursin("Not enough satellites to calculate position.", out)
end

@testset "GUI while decoding (enough sats but no PVT yet)" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    gui_data = GNSSReceiver.GUIData(
        Dictionary(
            [(:GPSL1CA, 3), (:GPSL1CA, 12), (:GPSL1CA, 23), (:GPSL1CA, 10)],
            [
                GNSSReceiver.SatelliteDataOfInterest(
                    45.0dBHz,
                    zeros(SVector{2,ComplexF64}),
                    true,
                ) for _ = 1:4
            ],
        ),
        GNSSReceiver.PVTSolution(),  # no fix yet: pvt.time === nothing
        10.0u"s",
        true,
    )
    out = render_gui_text(gui_model(gui_data))
    # Four tracked sats show up in the CN0 barplot (with their RINEX-style labels), but
    # with no PVT fix the DOA and position panels report that decoding is still going.
    @test occursin("Decoding satellites", out)
    @test !occursin("Not enough satellites", out)
    @test occursin("G03 L1", out)     # a CN0 bar label
end

@testset "GUI with data" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    gui_data = GNSSReceiver.GUIData(
        Dictionary(Dict{Tuple{Symbol,Int},sat_data_type}(
            (:GPSL1CA, 3) => GNSSReceiver.SatelliteDataOfInterest(
                46.3453dBHz,
                zeros(SVector{2,ComplexF64}),
                true,
            ),
            (:GPSL1CA, 12) => GNSSReceiver.SatelliteDataOfInterest(
                42.233dBHz,
                zeros(SVector{2,ComplexF64}),
                true,
            ),
            (:GPSL1CA, 23) => GNSSReceiver.SatelliteDataOfInterest(
                43.23123dBHz,
                zeros(SVector{2,ComplexF64}),
                true,
            ),
            (:GPSL1CA, 10) => GNSSReceiver.SatelliteDataOfInterest(
                45.123467dBHz,
                zeros(SVector{2,ComplexF64}),
                true,
            ),
        )),
        PositionVelocityTime.PVTSolution(;
            position = ECEF(4.0e6, 3.9e5, 4.9e6),
            velocity = ECEF(2.0e6, 2.9e5, 1.9e6),
            time_correction = 4.5e6u"m",
            time = TAIEpoch(2022, 10, 8),
            relative_clock_drift = 0.1e-6,
            dop = PositionVelocityTime.DOP(1.0, 1.0, 1.0, 1.0, 1.0),
            sats = Dictionary(
                [
                    (:GPSL1CA, 3),
                    (:GPSL1CA, 12),
                    (:GPSL1CA, 23),
                    (:GPSL1CA, 10),
                ],
                [
                    PositionVelocityTime.SatInfo(ECEF(5e6, 3e6, 1e6), 0.0, 0.0u"m"),
                    PositionVelocityTime.SatInfo(ECEF(3e6, 3e6, 2e6), 0.0, 0.0u"m"),
                    PositionVelocityTime.SatInfo(ECEF(2e6, 5e6, 1e6), 0.0, 0.0u"m"),
                    PositionVelocityTime.SatInfo(ECEF(3e6, 1e6, 1e6), 0.0, 0.0u"m"),
                ],
            ),
        ),
        10.0u"s",
        true,
    )

    # Diagnostics on so DOP / residual state is asserted alongside the core PVT block.
    out = render_gui_text(gui_model(gui_data; show_diagnostics = true))
    # CN0 bar values (dBHz, rounded to one decimal).
    @test occursin("45.1", out)
    @test occursin("42.2", out)
    @test occursin("43.2", out)
    @test occursin("46.3", out)
    # PVT block: single rounded "Coordinates" row with hemisphere letters, altitude in
    # metres, ground speed in m/s. Labels are padded to a common width; "Coordinates" is
    # the widest, so its value sits one space after the colon.
    @test occursin("Coordinates: 50.830895°N, 5.568737°E", out)
    @test occursin(r"Altitude:.* m", out)
    @test occursin("Speed:", out) && occursin(" m/s", out)
    # Time solution shown as UTC via AstroTime's leap-aware conversion, time-then-date
    # (HH:MM:SS.sss dd.mm.yyyy): TAI 2022-10-08T00:00:00 is UTC 2022-10-07T23:59:23
    # (37 leap seconds) ⇒ "23:59:23.000 07.10.2022".
    @test occursin("Time:", out) && occursin("23:59:23.000 07.10.2022", out) && occursin("UTC", out)
    # Heading (course over ground) shown as a real bearing — the (unphysical but large)
    # velocity is well above the low-speed threshold, so it is not flagged low-speed.
    @test occursin("Heading:", out) && occursin("°", out)
    @test !occursin("low speed", out)
    @test occursin("Run time:", out) && occursin("10.0 s", out)
    # The block is titled "Position Velocity Time (PVT)" and, with diagnostics on, folds in
    # the solution internals: the full DOP breakdown (GDOP/PDOP/HDOP/VDOP/TDOP) is shown;
    # residuals flagged (4 sats, single system/band ⇒ no redundancy), so the "insufficient
    # redundancy" note appears.
    @test occursin("Position Velocity Time (PVT)", out)
    @test occursin("GDOP", out) && occursin("HDOP", out) && occursin("VDOP", out) &&
          occursin("PDOP", out) && occursin("TDOP", out)
    @test occursin("insufficient redundancy", out)
    # With no tile rendered (no background task here), the Map panel falls back to the
    # coordinates and a Google Maps URL (rendered as a clickable OSC 8 link).
    @test occursin("Map", out)
    @test occursin("google.com/maps?q=", out)
end

@testset "GUI CN0 bars survive an unmeasurable satellite" begin
    # Tracking's default NWPR estimator reports `-Inf dBHz` for a satellite whose CN0 it
    # cannot measure — and a negative dB-Hz figure just above that — while the detectors may
    # still be holding it, so such a satellite does reach the bar chart.
    # `UnicodePlots.barplot` throws on any value below zero ("all values have to be ≥ 0"),
    # which would take the whole dashboard down, so the panel floors them at the 0 dB
    # baseline instead.
    _cn0_db = GNSSReceiver.Dashboard._cn0_db
    no_signal = uconvert(u"dBHz", 0.0u"Hz")
    @test ustrip(no_signal) == -Inf
    @test _cn0_db(45.1234dBHz) == 45.1
    @test _cn0_db(no_signal) == 0.0
    @test _cn0_db(uconvert(u"dBHz", 0.5u"Hz")) == 0.0     # ≈ -3 dBHz, still negative
    @test _cn0_db(uconvert(u"dBHz", Inf * u"Hz")) == 0.0
    @test _cn0_db(uconvert(u"dBHz", NaN * u"Hz")) == 0.0

    # End to end through the real `view`: the frame renders, with the measurable satellite's
    # bar intact next to the unmeasurable one's empty bar.
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    sat_keys = [(:GPSL1CA, 3), (:GPSL1CA, 12)]
    gui_data = GNSSReceiver.GUIData(
        Dictionary(
            sat_keys,
            [
                sat_data_type(cn0, zeros(SVector{2,ComplexF64}), true) for
                cn0 in (no_signal, 45.0dBHz)
            ],
        ),
        PositionVelocityTime.PVTSolution(),
        10.0u"s",
        true,
    )
    out = render_gui_text(gui_model(gui_data))
    @test occursin(r"G03 L1\s+0\s", out)          # empty bar, drawn at the baseline
    @test occursin(r"G12 L1\s+■+ 45\s", out)      # ... next to an intact one
end

@testset "GUI PVT diagnostics — multi-GNSS, multi-band, redundant" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{ComplexF64}
    keys_ = [
        (:GPSL1CA, 1), (:GPSL1CA, 8), (:GPSL1CA, 30),
        (:GalileoE1B, 3), (:GalileoE1B, 9), (:GalileoE1B, 24),
        (:GPSL5I, 8), (:GPSL5I, 30),
    ]
    sat_positions = [ECEF(5e6 + 1e5 * i, 3e6 - 1e5 * i, 1e6 + 2e5 * i) for i in eachindex(keys_)]
    # All residuals 2.0 m ⇒ overall and per-signal RMS are exactly 2.0 m.
    sats = Dictionary(keys_, [PositionVelocityTime.SatInfo(p, 0.0, 2.0u"m") for p in sat_positions])

    pvt = PositionVelocityTime.PVTSolution(;
        position = ECEF(4.0e6, 3.9e5, 4.9e6),
        time = TAIEpoch(2022, 10, 8),
        dop = PositionVelocityTime.DOP(2.5, 2.1, 1.3, 1.6, 0.9),
        sats,
        reference_system = GPST(),
        inter_system_biases = Dict{GNSSSignals.TimeSystem,typeof(1.0u"m")}(GST() => 12.34u"m"),
        inter_frequency_biases =
            Dict{Symbol,InterFrequencyBias}(:L5 => InterFrequencyBias(5.67u"m", :L1)),
    )

    gd = GNSSReceiver.GUIData(
        Dictionary(Dict{Tuple{Symbol,Int},sat_data_type}(
            k => sat_data_type(45.0dBHz, complex(1.0, 0.0), true) for k in keys_
        )),
        pvt,
        42.0u"s",
        true,
    )
    out = render_gui_text(gui_model(gd; show_diagnostics = true))

    @test occursin("Run time:", out) && occursin("42.0 s", out)
    @test occursin("DOP", out) && occursin("2.5", out)          # GDOP
    @test occursin("Inter-system biases", out) && occursin("GST", out) && occursin("12.34", out)
    # Inter-frequency bias names its reference band explicitly ("L5 (vs L1)").
    @test occursin("Inter-frequency biases", out) && occursin("L5", out) && occursin("5.67", out)
    @test occursin("vs L1", out)
    # No velocity given ⇒ zero speed ⇒ heading is flagged low-speed.
    @test occursin("Heading:", out) && occursin("low speed", out)
    # 8 sats, unknowns = 3 + 2 time systems + 1 extra band = 6 ⇒ redundant ⇒ not flagged.
    @test occursin("Pseudorange residual RMS", out)
    @test occursin("RMS/m", out)          # tabular header
    @test occursin("2.0", out)            # RMS value (all residuals 2.0 m)
    @test !occursin("insufficient redundancy", out)
    # DOA legend distinguishes constellations.
    @test occursin("GPS", out) && occursin("Galileo", out)
end

@testset "Display metadata is derived from GNSSSignals" begin
    # The GUI's per-signal display tables are built from `DISPLAYED_SIGNALS` through
    # GNSSSignals' accessors, so every entry must agree with the accessor it came from.
    # This is what replaced reading the constellation back out of the id's spelling.
    for S in GNSSReceiver.DISPLAYED_SIGNALS
        id = get_signal_id(S)
        constellation = get_constellation_id(S)
        @test GNSSReceiver.constellation_of(id) === constellation
        @test GNSSReceiver.constellation_name(constellation) == get_constellation_name(S)
        # Every displayed signal is orderable and labellable, and its constellation has a
        # RINEX letter, a marker colour and a rank — so no displayed signal can fall
        # through to a fallback.
        @test haskey(GNSSReceiver.SIGNAL_ORDER, id)
        @test haskey(GNSSReceiver.BAND_ABBREVIATIONS, id)
        @test haskey(GNSSReceiver.CONSTELLATION_LETTERS, constellation)
        @test haskey(GNSSReceiver.CONSTELLATION_COLORS, constellation)
        @test haskey(GNSSReceiver.CONSTELLATION_ORDER, constellation)
    end
    @test GNSSReceiver.constellation_of(:GPSL1CA) === :GPS
    @test GNSSReceiver.constellation_of(:GalileoE5aQ) === :Galileo
    @test GNSSReceiver.constellation_name(:Galileo) == "Galileo"
    # A signal the GUI does not know lands in `:Other` instead of erroring — the id no
    # longer has to begin with its constellation's name for this to work.
    @test GNSSReceiver.constellation_of(:SomeFutureSignal) === :Other
    @test GNSSReceiver.constellation_name(:Other) == "Other"
end

@testset "Satellite labels and display order" begin
    # RINEX-style system letter + zero-padded PRN + band token, every label the same
    # width (the token is padded to the widest, 3 chars).
    @test GNSSReceiver.sat_label(:GPSL1CA, 3) == "G03 L1 "
    @test GNSSReceiver.sat_label(:GPSL1C_P, 30) == "G30 L1C"
    @test GNSSReceiver.sat_label(:GalileoE1B, 24) == "E24 E1 "
    @test GNSSReceiver.sat_label(:GalileoE5aQ, 7) == "E07 E5a"
    @test allequal(
        length(GNSSReceiver.sat_label(id, 1)) for
        id in keys(GNSSReceiver.BAND_ABBREVIATIONS)
    )

    # Bars are ordered by constellation, then PRN, then signal — the signal rank being
    # each signal's position in `DISPLAYED_SIGNALS` (ascending band, data before pilot).
    # A BOC(1,1) approximation ranks with the E1 signal it stands in for, so it sorts
    # before E5a rather than after everything known.
    keys_ = [
        (:GalileoE5aI, 5),
        (:GPSL5I, 5),
        (:GPSL1CA, 5),
        (:GalileoE1B, 5),
        (:GPSL1CA, 3),
        (:GalileoE1B_BOC11, 5),
    ]
    @test sort(keys_; by = GNSSReceiver.sat_sort_key) == [
        (:GPSL1CA, 3),
        (:GPSL1CA, 5),
        (:GPSL5I, 5),
        (:GalileoE1B, 5),
        (:GalileoE1B_BOC11, 5),
        (:GalileoE5aI, 5),
    ]
    # An unknown signal sorts last (`:Other`) instead of erroring.
    unknown_last =
        sort([(:SomeFutureSignal, 1), (:GPSL1CA, 30)]; by = GNSSReceiver.sat_sort_key)
    @test last(unknown_last) == (:SomeFutureSignal, 1)
end

@testset "Band and time-system display tables" begin
    # Keyed by `get_band_id` — exactly what `pvt.inter_frequency_biases` is keyed by —
    # and named by `get_band_name`, which is the band's own label (no constellation).
    for B in GNSSReceiver.DISPLAYED_BANDS
        @test GNSSReceiver.band_name(get_band_id(B)) == get_band_name(B)
    end
    @test GNSSReceiver.band_name(get_band_id(L1)) == "L1"
    @test GNSSReceiver.BAND_ORDER[get_band_id(L1)] <
          GNSSReceiver.BAND_ORDER[get_band_id(L2)] <
          GNSSReceiver.BAND_ORDER[get_band_id(L5)]
    @test GNSSReceiver.band_name(:E6) == "E6"   # a band we don't rank still prints

    # Time systems are ranked by `get_time_system_id`, GPS before Galileo.
    @test GNSSReceiver.TIME_SYSTEM_ORDER[get_time_system_id(GPST())] <
          GNSSReceiver.TIME_SYSTEM_ORDER[get_time_system_id(GST())]
end

@testset "PVT diagnostics name time systems and bands" begin
    # Anchored on the lowest-ranked time system present (GPS < Galileo) rather than on
    # `calc_pvt`'s `reference_system`, which can flip between solves.
    pvt = PositionVelocityTime.PVTSolution(;
        reference_system = GST(),
        inter_system_biases =
            Dict{GNSSSignals.TimeSystem,typeof(1.0u"m")}(GPST() => -12.34u"m"),
        inter_frequency_biases =
            Dict{Symbol,InterFrequencyBias}(:L5 => InterFrequencyBias(5.67u"m", :L1)),
    )
    lines = GNSSReceiver.pvt_details_lines(pvt)
    # The anchor is spelled out in full (`get_time_system_name`) since it is stated once
    # and says what the rows are measured against; the rows use the compact ids
    # (`get_time_system_id`) to fit the narrow panel.
    @test "Inter-system biases (vs GPS Time):" in lines
    @test "  GST: 12.34 m" in lines
    # Bands are named through `get_band_name`.
    @test "  L5 (vs L1): 5.67 m" in lines
end
