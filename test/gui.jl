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

@testset "GUI with no data" begin
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch
        sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
        gui_data = GNSSReceiver.GUIData(
            Dictionary{Tuple{Symbol,Int},sat_data_type}(),
            GNSSReceiver.PVTSolution(),
            0.0u"s",
            true,
        )

        foreach(i -> put!(ch, gui_data), 1:20)
    end

    buf = IOBuffer()
    GNSSReceiver.gui(gui_data_channel, buf)
    out = String(take!(buf))
    @test occursin("Searching for satellites", out)
    @test occursin("Not enough satellites to calculate position.", out)
end

@testset "GUI while decoding (enough sats but no PVT yet)" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch
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
        foreach(i -> put!(ch, gui_data), 1:20)
    end

    buf = IOBuffer()
    GNSSReceiver.gui(gui_data_channel, buf)
    out = String(take!(buf))
    # Four tracked sats show up in the CN0 barplot, but with no PVT fix the DOA and
    # position panels report that decoding is still in progress.
    @test occursin("Decoding satellites", out)
    @test !occursin("Not enough satellites", out)
end

@testset "GUI with data" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch
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

        foreach(i -> put!(ch, gui_data), 1:20)
    end

    buf = IOBuffer()
    GNSSReceiver.gui(gui_data_channel, buf)
    out = String(take!(buf))
    #    println(out)
    @test occursin("Satellites", out)
    @test occursin("45.1", out)
    @test occursin("42.2", out)
    @test occursin("43.2", out)
    @test occursin("46.3", out)
    # Combined PVT block: single rounded "Coordinates" row with hemisphere letters,
    # altitude in metres, and ground speed in m/s. No Google-maps link. Labels are
    # padded to a common width; "Coordinates" is the widest, so its value sits one
    # space after the colon (this also pins the alignment column).
    @test occursin("Coordinates: 50.830895°N, 5.568737°E", out)
    # Altitude shown in metres. (Panels render side by side, so the metre suffix is
    # followed by padding + a border rather than a newline — match within the line.)
    @test occursin(r"Altitude:.* m", out)
    @test occursin("Speed:", out) && occursin(" m/s", out)
    # Time solution, first row, shown as UTC via AstroTime's leap-aware conversion,
    # in time-then-date form (HH:MM:SS.sss dd.mm.yyyy): TAI 2022-10-08T00:00:00 is
    # UTC 2022-10-07T23:59:23 (37 leap seconds) ⇒ "23:59:23.000 07.10.2022".
    @test occursin("Time:", out) && occursin("23:59:23.000 07.10.2022", out) && occursin("UTC", out)
    # Heading (course over ground) shown as a real bearing here — the (unphysical, but
    # large) velocity is well above the low-speed threshold, so it is not greyed out.
    @test occursin("Heading:", out) && occursin("°", out)
    @test !occursin("low speed", out)
    @test !occursin("google.com", out)
    # Run-time row is part of the aligned PVT block (padding ⇒ variable spacing).
    @test occursin(r"Run time:\s+10\.0 s", out)
    # The block is titled "Position Velocity Time (PVT)" and folds in the solution
    # internals: DOP shown; residuals greyed out here (4 sats, single system/band ⇒ no
    # redundancy), so the "insufficient redundancy" note appears.
    @test occursin("Position Velocity Time (PVT)", out)
    @test occursin("DOP", out)
    @test occursin("insufficient redundancy", out)
end

@testset "GUI PVT details panel — multi-GNSS, multi-band, redundant" begin
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

    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch
        gd = GNSSReceiver.GUIData(
            Dictionary(Dict{Tuple{Symbol,Int},sat_data_type}(
                k => sat_data_type(45.0dBHz, complex(1.0, 0.0), true) for k in keys_
            )),
            pvt,
            42.0u"s",
            true,
        )
        foreach(i -> put!(ch, gd), 1:5)
    end
    buf = IOBuffer()
    GNSSReceiver.gui(gui_data_channel, buf)
    out = String(take!(buf))

    @test occursin(r"Run time:\s+42\.0 s", out)
    @test occursin("DOP", out) && occursin("2.5", out)          # GDOP
    @test occursin("Inter-system biases", out) && occursin("GST", out) && occursin("12.34", out)
    # Inter-frequency bias names its reference band explicitly (":L5 (vs L1)").
    @test occursin("Inter-frequency biases", out) && occursin("L5", out) && occursin("5.67", out)
    @test occursin("vs L1", out)
    # No velocity given ⇒ zero speed ⇒ heading is greyed out with the "low speed" note.
    @test occursin("Heading:", out) && occursin("low speed", out)
    # 8 sats, unknowns = 3 + 2 time systems + 1 extra band = 6 ⇒ redundant ⇒ not greyed.
    @test occursin("Pseudorange residual RMS", out)
    @test occursin("RMS/m", out)          # tabular header
    @test occursin("2.0", out)            # RMS value (all residuals 2.0 m)
    @test !occursin("insufficient redundancy", out)
    # DOA legend distinguishes constellations.
    @test occursin("GPS", out) && occursin("Galileo", out)
end
