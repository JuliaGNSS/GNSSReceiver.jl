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

# Render one frame of the Tachikoma dashboard for `model` into an off-screen buffer and
# return it as plain text. This drives the real `view` (CN0 bars, sky plot, PVT block, map
# fallback) without a TTY — the map background task is only spawned by `gui`, not `view`,
# so no network is touched and `map_lines` stays empty (the coordinate fallback shows).
using Tachikoma: Rect, Buffer, Frame, GraphicsRegion, ColorRGBA, buffer_to_text

function render_gui_text(model; width = 140, height = 50)
    area = Rect(1, 1, width, height)
    buf = Buffer(area)
    frame = Frame(buf, area, GraphicsRegion[], Tuple{Int,Int,Matrix{ColorRGBA}}[])
    GNSSReceiver.view(model, frame)
    buffer_to_text(buf, area)
end

function gui_model(gui_data; show_diagnostics = false)
    m = GNSSReceiver.ReceiverModel()
    m.gui = gui_data
    isnothing(gui_data.pvt.time) || (m.last_fix = gui_data)
    m.show_diagnostics = show_diagnostics
    m
end

@testset "GUI input handling (update!)" begin
    using Tachikoma: KeyEvent

    # Quit on q / Q / Esc.
    for e in (KeyEvent('q'), KeyEvent('Q'), KeyEvent(:escape))
        m = GNSSReceiver.ReceiverModel()
        GNSSReceiver.update!(m, e)
        @test m.quit
    end

    # `d` toggles the diagnostics section.
    m = GNSSReceiver.ReceiverModel()
    @test !m.show_diagnostics
    GNSSReceiver.update!(m, KeyEvent('d'))
    @test m.show_diagnostics
    GNSSReceiver.update!(m, KeyEvent('d'))
    @test !m.show_diagnostics

    # Map zoom: `+`/`-`, clamped to [1, 18].
    m = GNSSReceiver.ReceiverModel()
    z0 = m.map_zoom
    GNSSReceiver.update!(m, KeyEvent('+'))
    @test m.map_zoom == z0 + 1
    GNSSReceiver.update!(m, KeyEvent('-'))
    @test m.map_zoom == z0
    foreach(_ -> GNSSReceiver.update!(m, KeyEvent('-')), 1:30)
    @test m.map_zoom == 1
    foreach(_ -> GNSSReceiver.update!(m, KeyEvent('+')), 1:30)
    @test m.map_zoom == 18

    # Map pan: hjkl move the centre offset; `0` recenters and resets zoom.
    m = GNSSReceiver.ReceiverModel()
    GNSSReceiver.update!(m, KeyEvent('l'))
    @test m.map_dlon > 0
    GNSSReceiver.update!(m, KeyEvent('h'))
    @test isapprox(m.map_dlon, 0.0; atol = 1e-9)
    GNSSReceiver.update!(m, KeyEvent('k'))
    @test m.map_dlat > 0
    GNSSReceiver.update!(m, KeyEvent('j'))
    @test isapprox(m.map_dlat, 0.0; atol = 1e-9)
    m.map_zoom = 5
    GNSSReceiver.update!(m, KeyEvent('k'))
    GNSSReceiver.update!(m, KeyEvent('0'))
    @test m.map_zoom == 13 && m.map_dlon == 0.0 && m.map_dlat == 0.0
    @test !m.quit   # panning/zooming never quits
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
    # the solution internals: DOP shown; residuals flagged (4 sats, single system/band ⇒ no
    # redundancy), so the "insufficient redundancy" note appears.
    @test occursin("Position Velocity Time (PVT)", out)
    @test occursin("DOP", out)
    @test occursin("insufficient redundancy", out)
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
