@testset "Per-GNSS PRN selection and signal-capability filtering" begin
    G = GNSSReceiver

    # L1 C/A and every Galileo signal broadcast on the whole constellation;
    # modernized GPS signals only on newer blocks.
    @test G.broadcasting_prns(GPSL1CA()) === nothing
    @test G.broadcasting_prns(GalileoE5aI()) === nothing
    @test !isnothing(G.broadcasting_prns(GPSL5I()))
    @test issubset(G.broadcasting_prns(GPSL5I()), G.broadcasting_prns(GPSL2CM()))

    gps = [1, 4, 7, 8, 9, 11, 13, 17, 18, 19, 28, 30]
    gal = [3, 9, 11, 12, 24, 25, 31, 33]
    prns = (GPS = gps, Galileo = gal)

    # L1 C/A: no capability restriction ⇒ full GPS request, order preserved.
    @test G.search_prns(prns, GPSL1CA()) == gps
    # L5: request restricted to L5-capable PRNs (drops IIR/IIR-M PRNs 7,13,17,19).
    l5 = G.search_prns(prns, GPSL5I())
    @test l5 == [1, 4, 8, 9, 11, 18, 28, 30]
    @test !any(in([7, 13, 17, 19]), l5)
    # L2C (data signal L2CM) additionally keeps the IIR-M PRNs 7 and 17.
    l2c = G.search_prns(prns, GPSL2CM())
    @test l2c == [1, 4, 7, 8, 9, 11, 17, 18, 28, 30]
    @test issubset(l5, l2c)
    # Galileo lists apply unchanged to both Galileo signals.
    @test G.search_prns(prns, GalileoE1B()) == gal
    @test G.search_prns(prns, GalileoE5aI()) == gal

    # A missing constellation falls back to the default range (filtered by capability).
    @test G.search_prns((GPS = gps,), GalileoE1B()) == collect(1:36)
    @test G.search_prns((Galileo = gal,), GPSL5I()) == G.broadcasting_prns(GPSL5I())

    # `nothing` ⇒ constellation default, still capability-filtered.
    @test G.search_prns(nothing, GPSL1CA()) == collect(1:32)
    @test G.search_prns(nothing, GPSL5I()) == G.broadcasting_prns(GPSL5I())

    # Backwards compatible: a plain collection applies to every system.
    @test G.search_prns([1, 8, 30, 7], GPSL5I()) == [1, 8, 30]  # 7 is not L5-capable
    @test G.search_prns([1, 8, 30, 7], GPSL1CA()) == [1, 8, 30, 7]

    # `Dict` form is accepted too.
    @test G.search_prns(Dict(:GPS => gps, :Galileo => gal), GalileoE1B()) == gal
end
