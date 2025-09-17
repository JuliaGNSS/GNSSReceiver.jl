@testset "Get GUI data from data channel" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{sat_data_type}}() do ch
        foreach(1:100) do i
            data = GNSSReceiver.ReceiverDataOfInterest{sat_data_type}(
                Dict{Int,sat_data_type}(
                    1 => sat_data_type(
                        45.0dBHz,
                        SVector(complex(1.0, 2.0), complex(2.0, 3.0)),
                        true,
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
    @test first(gui_datas[1].sat_data[1].cn0) == 45dBHz
    @test length(gui_datas[2].sat_data) == 1
    @test first(gui_datas[2].sat_data[1].cn0) == 45dBHz
    @test isnothing(gui_datas[1].pvt.time)
end

@testset "GUI with no data" begin
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch
        sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
        gui_data = GNSSReceiver.GUIData(
            Dict{Int,sat_data_type}(),
            GNSSReceiver.PVTSolution(),
            0.0u"s",
        )

        foreach(i -> put!(ch, gui_data), 1:20)
    end

    buf = IOBuffer()
    GNSSReceiver.gui(gui_data_channel, buf)
    out = String(take!(buf))
    @test occursin("Searching for satellites", out)
    @test occursin("Not enough satellites to calculate position.", out)
end

@testset "GUI with data" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch
        gui_data = GNSSReceiver.GUIData(
            Dict{Int,sat_data_type}(
                3 => GNSSReceiver.SatelliteDataOfInterest(
                    46.3453dBHz,
                    zeros(SVector{2,ComplexF64}),
                    true,
                ),
                12 => GNSSReceiver.SatelliteDataOfInterest(
                    42.233dBHz,
                    zeros(SVector{2,ComplexF64}),
                    true,
                ),
                23 => GNSSReceiver.SatelliteDataOfInterest(
                    43.23123dBHz,
                    zeros(SVector{2,ComplexF64}),
                    true,
                ),
                10 => GNSSReceiver.SatelliteDataOfInterest(
                    45.123467dBHz,
                    zeros(SVector{2,ComplexF64}),
                    true,
                ),
            ),
            PositionVelocityTime.PVTSolution(
                ECEF(4.0e6, 3.9e5, 4.9e6),
                ECEF(2.0e6, 2.9e5, 1.9e6),
                4.5e6,
                TAIEpoch(2022, 10, 8),
                0.1e-6,
                PositionVelocityTime.DOP(1.0, 1.0, 1.0, 1.0, 1.0),
                Dict(
                    3 => PositionVelocityTime.SatInfo(ECEF(5e6, 3e6, 1e6), 0.0),
                    12 => PositionVelocityTime.SatInfo(ECEF(3e6, 3e6, 2e6), 0.0),
                    23 => PositionVelocityTime.SatInfo(ECEF(2e6, 5e6, 1e6), 0.0),
                    10 => PositionVelocityTime.SatInfo(ECEF(3e6, 1e6, 1e6), 0.0),
                ),
            ),
            10.0u"s",
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
    @test occursin("50.830894797269906", out)
    @test occursin("5.568737077976637", out)
end