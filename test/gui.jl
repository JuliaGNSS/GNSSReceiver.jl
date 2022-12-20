@testset "Get GUI data from data channel" begin
    sat_data_type = GNSSReceiver.SatelliteDataOfInterest{SVector{2,ComplexF64}}
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{sat_data_type}}() do ch
        foreach(1:200) do i
            data = GNSSReceiver.ReceiverDataOfInterest{sat_data_type}(
                Dict{Int,sat_data_type}(
                    1 => sat_data_type(
                        45.0dBHz,
                        SVector(complex(1.0, 2.0), complex(2.0, 3.0)),
                    ),
                ),
                GNSSReceiver.PVTSolution(),
                (i - 1) * 1ms,
            )
            put!(ch, data)
        end
    end

    gui_data_channel = get_gui_data_channel(data_channel, 100ms)

    gui_datas = collect(gui_data_channel)
    @test length(gui_datas) == 2
    @test length(gui_datas[1].cn0s) == 1
    @test first(gui_datas[1].cn0s) == (1 => 45dBHz)
    @test length(gui_datas[2].cn0s) == 1
    @test first(gui_datas[2].cn0s) == (1 => 45dBHz)
    @test isnothing(gui_datas[1].pvt.time)
end

@testset "GUI with no data" begin
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch
        gui_data = GNSSReceiver.GUIData(Dict{Int,typeof(1.0dBHz)}(), GNSSReceiver.PVTSolution())

        foreach(i -> put!(ch, gui_data), 1:20)
    end

    buf = IOBuffer()
    GNSSReceiver.gui(gui_data_channel, buf)
    out = String(take!(buf))
    @test occursin("Searching for satellites", out)
    @test occursin("Not enough satellites to calculate position.", out)
end

@testset "GUI with data" begin
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch
        gui_data = GNSSReceiver.GUIData(
            Dict{Int,typeof(1.0dBHz)}(
                3 => 46.3453dBHz,
                12 => 42.233dBHz,
                23 => 43.23123dBHz,
                10 => 45.123467dBHz,
            ),
            GNSSReceiver.PVTSolution(
                ECEF(4.0e6, 3.9e5, 4.9e6),
                4.5e6,
                TAIEpoch(2022, 10, 8),
                PositionVelocityTime.DOP(1.0, 1.0, 1.0, 1.0, 1.0),
                [3, 12, 23, 10],
                [
                    ECEF(5e6, 3e6, 1e6),
                    ECEF(3e6, 3e6, 2e6),
                    ECEF(2e6, 5e6, 1e6),
                    ECEF(3e6, 1e6, 1e6),
                ],
            ),
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