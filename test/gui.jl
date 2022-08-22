@testset "Get GUI data from data channel" begin
    data_channel = Channel{GNSSReceiver.ReceiverDataOfInterest{4}}() do ch
        foreach(1:200) do i
            data = GNSSReceiver.ReceiverDataOfInterest{4}(
                Dict{Int, Vector{GNSSReceiver.SatelliteDataOfInterest{4}}}(),
                GNSSReceiver.PVTSolution(),
                (i - 1) * 1ms
            )
            put!(ch, data)
        end
    end

    gui_data_channel = get_gui_data_channel(data_channel, 100ms)

    gui_datas = collect(gui_data_channel)
    @test length(gui_datas) == 2
    @test length(gui_datas[1].cn0s) == 0
    @test length(gui_datas[2].cn0s) == 0
    @test isnothing(gui_datas[1].pvt.time)
end

@testset "GUI with no data" begin
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch

        gui_data = GNSSReceiver.GUIData(
            Dict{Int, typeof(1.0dBHz)}(),
            GNSSReceiver.PVTSolution()
        )

        foreach(i -> put!(ch, gui_data), 1:20)
    end

    buf = IOBuffer()
    GNSSReceiver.gui(gui_data_channel, buf)
    out = String(take!(buf))
    @test occursin("No satellites acquired.", out)
    @test occursin("Not enough satellites to calculate position.", out)
end

@testset "GUI with data" begin
    gui_data_channel = Channel{GNSSReceiver.GUIData}() do ch

        gui_data = GNSSReceiver.GUIData(
            Dict{Int, typeof(1.0dBHz)}(
                3 => 46.3453dBHz,
                12 => 42.233dBHz,
                23 => 43.23123dBHz,
                10 => 45.123467dBHz
            ),
            GNSSReceiver.PVTSolution(
                ECEF(4.0e6, 3.9e5, 4.9e6),
                4.5e6,
                TAIEpoch(2022, 10, 8),
                PositionVelocityTime.DOP(1.0,1.0,1.0,1.0,1.0),
                [3, 12, 23, 10],
                [ECEF(5e6, 3e6, 1e6), ECEF(3e6, 3e6, 2e6), ECEF(2e6, 5e6, 1e6), ECEF(3e6, 1e6, 1e6)]
            )
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
    @test occursin("Latitude: 50.830894797269906", out)
    @test occursin("Longitude: 5.568737077976637", out)
end