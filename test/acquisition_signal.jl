@testset "Acquisition-signal selection (pilot vs data)" begin
    G = GNSSReceiver
    fs = 5e6Hz

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
    @test G.acquisition_signal(l5, fs, 100Hz) isa GPSL5I
    @test G.acquisition_signal(l5, fs, 50Hz) isa GPSL5Q
    @test G.acquisition_signal(l5, fs, nothing) isa GPSL5Q

    # L1C-P: 10 ms code ⇒ N=1 already gives 100 Hz, so its long (L=1800) secondary is
    # irrelevant and the pilot is always chosen.
    @test G.acquisition_signal(l1c, fs, 100Hz) isa GPSL1C_P
    @test G.acquisition_signal(l1c, fs, nothing) isa GPSL1C_P

    # E1C: length-25 secondary, 4 ms code. 100 Hz needs N=3 (not a multiple of 25) ⇒
    # data; 10 Hz needs N=25 (one full secondary period) ⇒ pilot.
    @test G.acquisition_signal(e1, fs, 100Hz) isa GalileoE1B
    @test G.acquisition_signal(e1, fs, 10Hz) isa GalileoE1C

    # E5aQ: length-100 secondary (> the 32 rotation cap), 1 ms code. It can never be
    # acquired coherently at N>1, so it always falls back to the data component —
    # even at a fine resolution where N would be a multiple of 100.
    @test G.acquisition_signal(e5a, fs, 100Hz) isa GalileoE5aI
    @test G.acquisition_signal(e5a, fs, 10Hz) isa GalileoE5aI
    @test G.MAX_SECONDARY_CODE_ROTATIONS == 32

    # L2CL: no secondary code, but a 1.5 s primary code. A single code period
    # (nc=1) already gives ~0.67 Hz — ~150× finer than a 100 Hz target and far longer
    # than L2CM's 20 ms window — so acquisition stays on the data component. The
    # pilot is only chosen when the requested resolution is that fine: at 0.67 Hz
    # both components resolve to a 1.5 s window, so the pilot's power wins.
    @test G.acquisition_signal(l2c, fs, 100Hz) isa GPSL2CM
    @test G.acquisition_signal(l2c, fs, nothing) isa GPSL2CM
    @test G.acquisition_signal(l2c, fs, 0.67Hz) isa GPSL2CL

    # A plain (non-combined) signal is always its own acquisition signal (the exact
    # object passed in is returned unchanged).
    l1ca = GPSL1CA()
    e1b = GalileoE1B()
    @test G.acquisition_signal(l1ca, fs, 100Hz) === l1ca
    @test G.acquisition_signal(e1b, fs, nothing) === e1b
end

@testset "signal_group_key uses the ranging signal (matches pvt.sats)" begin
    G = GNSSReceiver
    # Plain signal: ranging == data == itself, so the key is the signal's own id.
    @test G.signal_group_key(GPSL1CA()) == get_signal_id(GPSL1CA())
    @test G.signal_group_key(GPSL5I()) == get_signal_id(GPSL5I())
    # CombinedSignal: keyed by the pilot (ranging) id, NOT the data id — matching what
    # PVT keys `pvt.sats` by (`calc_pvt` is handed the ranging signal).
    l5 = G.CombinedSignal(GPSL5Q(), GPSL5I())
    e1 = G.CombinedSignal(GalileoE1C(), GalileoE1B())
    @test G.signal_group_key(l5) == get_signal_id(GPSL5Q()) == get_signal_id(G.ranging_signal(l5))
    @test G.signal_group_key(l5) != get_signal_id(G.data_signal(l5))
    @test G.signal_group_key(e1) == get_signal_id(GalileoE1C())
end
