@testset "Acquisition-signal selection (pilot vs data)" begin
    G = GNSSReceiver

    # A `CombinedSignal(pilot, data)` acquires on the pilot only when the requested
    # Doppler resolution lands exactly on a valid pilot coherent length (a whole
    # number of secondary-code periods, or a single code period, within the
    # rotation-search cap); otherwise it falls back to the data component. See
    # `acquisition_signal`.
    l5 = G.CombinedSignal(GPSL5Q(), GPSL5I())
    l1c = G.CombinedSignal(GPSL1C_P(), GPSL1C_D())
    e1 = G.CombinedSignal(GalileoE1C(), GalileoE1B())
    e5a = G.CombinedSignal(GalileoE5aQ(), GalileoE5aI())
    l2c = G.CombinedSignal(GPSL2CL(), GPSL2CM())

    # L5Q: NH20 secondary, 1 ms code. 100 Hz needs N=10 (not a multiple of 20) ⇒
    # data; 50 Hz needs N=20 (one full secondary period) ⇒ pilot. With no resolution
    # constraint, N=1 (secondary irrelevant) ⇒ pilot.
    @test G.acquisition_signal(l5, 100Hz) isa GPSL5I
    @test G.acquisition_signal(l5, 50Hz) isa GPSL5Q
    @test G.acquisition_signal(l5, nothing) isa GPSL5Q

    # L1C-P: 10 ms code ⇒ N=1 already gives 100 Hz, so its long (L=1800) secondary is
    # irrelevant and the pilot is always chosen.
    @test G.acquisition_signal(l1c, 100Hz) isa GPSL1C_P
    @test G.acquisition_signal(l1c, nothing) isa GPSL1C_P

    # E1C: length-25 secondary, 4 ms code. 100 Hz needs N=3 (not a multiple of 25) ⇒
    # data; 10 Hz needs N=25 (one full secondary period) ⇒ pilot.
    @test G.acquisition_signal(e1, 100Hz) isa GalileoE1B
    @test G.acquisition_signal(e1, 10Hz) isa GalileoE1C

    # E5aQ: length-100 secondary (> the 32 rotation cap), 1 ms code. It can never be
    # acquired coherently at N>1, so it always falls back to the data component —
    # even at a fine resolution where N would be a multiple of 100.
    @test G.acquisition_signal(e5a, 100Hz) isa GalileoE5aI
    @test G.acquisition_signal(e5a, 10Hz) isa GalileoE5aI
    @test G.MAX_SECONDARY_CODE_ROTATIONS == 32

    # L2CL: no secondary code, but a 1.5 s primary code. A single code period
    # (nc=1) already gives ~0.67 Hz — ~150× finer than a 100 Hz target and far longer
    # than L2CM's 20 ms window — so acquisition stays on the data component. The
    # pilot is only chosen when the requested resolution is that fine: at 0.67 Hz
    # both components resolve to a 1.5 s window, so the pilot's power wins.
    @test G.acquisition_signal(l2c, 100Hz) isa GPSL2CM
    @test G.acquisition_signal(l2c, nothing) isa GPSL2CM
    @test G.acquisition_signal(l2c, 0.67Hz) isa GPSL2CL

    # The new pilots land on the branches `acquisition_signal`'s comment claims for them.
    # E5b-Q, E6-C and B2a-Q all carry a length-100 secondary over a 1 ms code, so they
    # behave exactly as E5a-Q does: unacquirable coherently at N>1, hence the data
    # component at any requested resolution.
    e5b = G.CombinedSignal(GalileoE5bQ(), GalileoE5bI())
    e6 = G.CombinedSignal(GalileoE6C(), GalileoE6B())
    b2a = G.CombinedSignal(BeiDouB2aQ(), BeiDouB2aI())
    @test G.acquisition_signal(e5b, 100Hz) isa GalileoE5bI
    @test G.acquisition_signal(e5b, 10Hz) isa GalileoE5bI
    @test G.acquisition_signal(e6, 100Hz) isa GalileoE6B
    @test G.acquisition_signal(b2a, 100Hz) isa BeiDouB2aI
    # B1C-P is the L1C-P case: a 10 ms code already resolves 100 Hz at N=1, so its
    # length-1800 secondary never comes into play and the pilot always wins.
    b1c = G.CombinedSignal(BeiDouB1C_P(), BeiDouB1C_D())
    @test G.acquisition_signal(b1c, 100Hz) isa BeiDouB1C_P
    @test G.acquisition_signal(b1c, nothing) isa BeiDouB1C_P

    # A plain (non-combined) signal is always its own acquisition signal (the exact
    # object passed in is returned unchanged). BeiDou B1I, B3I and B2b-I have no pilot
    # component at all, so they are only ever used this way.
    l1ca = GPSL1CA()
    e1b = GalileoE1B()
    b1i = BeiDouB1I()
    @test G.acquisition_signal(l1ca, 100Hz) === l1ca
    @test G.acquisition_signal(e1b, nothing) === e1b
    @test G.acquisition_signal(b1i, 100Hz) === b1i
end

@testset "signal_group_key uses the ranging signal (matches pvt.satellites)" begin
    G = GNSSReceiver
    # Plain signal: ranging == data == itself, so the key is the signal's own id.
    @test G.signal_group_key(GPSL1CA()) == get_signal_id(GPSL1CA())
    @test G.signal_group_key(GPSL5I()) == get_signal_id(GPSL5I())
    # CombinedSignal: keyed by the pilot (ranging) id, NOT the data id — matching what
    # PVT keys `pvt.satellites` by (`calc_pvt` is handed the ranging signal).
    l5 = G.CombinedSignal(GPSL5Q(), GPSL5I())
    e1 = G.CombinedSignal(GalileoE1C(), GalileoE1B())
    @test G.signal_group_key(l5) == get_signal_id(GPSL5Q()) == get_signal_id(G.ranging_signal(l5))
    @test G.signal_group_key(l5) != get_signal_id(G.data_signal(l5))
    @test G.signal_group_key(e1) == get_signal_id(GalileoE1C())
end

@testset "BeiDou GEO satellites are acquired on a plan of their own" begin
    # A BeiDou B1I or B3I GEO satellite (PRN 1-5, 59-63) broadcasts D2 bits of 2 code
    # periods without the NH20 overlay, where the other satellites' D1 bits span 20; one
    # plan holds one bit count, so the GEO PRNs get a plan of their own.
    G = GNSSReceiver
    geo = [1, 2, 3, 4, 5, 59, 60, 61, 62, 63]
    for (signal, sampling_freq) in ((BeiDouB1I(), 4e6Hz), (BeiDouB3I(), 24.552e6Hz))
        _, plans, num_samples = G.plan_band_acquisition((signal,), sampling_freq, (250Hz,))
        d1, d2 = plans[get_signal_id(signal)]
        @test d1.avail_prns == setdiff(1:63, geo)
        @test d2.avail_prns == geo
        # The D1 window spans the overlay; the D2 one the shortest whole number of bits
        # that meets the resolution, 4 ms for 250 Hz.
        @test d1.num_coherently_integrated_code_periods == 20
        @test d2.num_coherently_integrated_code_periods == 4
        @test num_samples == 20 * G.samples_per_code(signal, sampling_freq)
    end
    # Without a GEO PRN to search, and on every other signal, one plan serves them all.
    _, plans, _ = G.plan_band_acquisition((BeiDouB1I(),), 4e6Hz, (250Hz,); prns = 6:58)
    @test length(plans.BeiDouB1I) == 1
    _, plans, _ = G.plan_band_acquisition((GPSL1CA(),), 4e6Hz, (250Hz,))
    @test length(plans.GPSL1CA) == 1
end
