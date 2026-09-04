# Note: this file is included from runtests.jl which provides all `using` statements.

# The Galileo decoder caches hold a concretely-typed `Aff3ct.ConvViterbiDecoder`, so
# `GNSSDecoderState(::GalileoE1B, prn)` cannot be built at all wherever the AFF3CT binary is
# missing — `aff3ct_jll` ships no `libaff3ct_jl` for Windows, and building the cache throws
# there. `GNSSDecoderState` parameterises its cache (`CA<:AbstractGNSSCache`) though, and
# none of these testsets decode a bit: they only read ephemeris, health and GGTO, all of
# which dispatch on the data or the constants (e.g. `get_data_frequency`,
# `time_offset_available`)
# and never touch the cache. So build the decoder states here with a cache that does nothing
# but subtype `AbstractGNSSCache` — on every platform, not just the ones without AFF3CT.
struct StubGNSSCache <: GNSSDecoder.AbstractGNSSCache end

stub_cache_decoder(prn, data, constants) =
    GNSSDecoderState(prn, data, data, constants, StubGNSSCache(), nothing, false)

# GPS decoders build fine everywhere, so only Galileo needs the stub.
test_decoder_state(system, prn) = GNSSDecoderState(system, prn)
test_decoder_state(::Union{GalileoE1B,GalileoE1B_BOC11}, prn) = stub_cache_decoder(
    prn,
    GNSSDecoder.GalileoINAVData(),
    GNSSDecoder.GalileoE1BConstants(),
)
test_decoder_state(::GalileoE5aI, prn) =
    stub_cache_decoder(prn, GNSSDecoder.GalileoE5aData(), GNSSDecoder.GalileoE5aConstants())

# A `VTMember` with hand-picked geometry for the measurement-model tests; the
# defaults describe a GPS L1 C/A satellite straight overhead along +x.
function _test_member(;
    group_key = :GPSL1CA,
    prn = 1,
    clock_bias_index = 1,
    ifb_index = 0,
    signal = GPSL1CA(),
    available = true,
    time = 0.0,
    sat_position = SVector(2.6e7, 0.0, 0.0),
    sat_velocity = SVector(0.0, 0.0, 0.0),
    sat_clock_drift = 0.0,
    pseudorange = 0.0,
    pseudorange_rate = 0.0,
    code_discriminator = 0.0,
    carrier_discriminator = 0.0,
    cn0 = 10^4.5, # 45 dB-Hz
    early_late_spacing = 1.0,
    coherent_integration_time = nothing,
    decoder = test_decoder_state(signal, prn),
)
    c = SPEED_OF_LIGHT
    code_frequency = ustrip(u"Hz", get_code_frequency(signal))
    tcoh = something(coherent_integration_time, get_code_length(signal) / code_frequency)
    sat_state = SatelliteState(;
        decoder,
        system = signal,
        code_phase = 0.0,
        carrier_doppler = 0.0u"Hz",
    )
    GNSSReceiver.VTMember(
        group_key,
        prn,
        clock_bias_index,
        ifb_index,
        c / code_frequency,
        c / ustrip(u"Hz", get_center_frequency(signal)),
        code_frequency,
        available,
        sat_state,
        time,
        # The same instant on the GPS Time count: differs from `time` by the
        # signal's defined scale offset (0 for GPST/GST, −14 s for BDT).
        time - time_scale_offset_to_gpst(get_time_system(signal)),
        sat_position,
        sat_velocity,
        sat_clock_drift,
        pseudorange,
        pseudorange_rate,
        code_discriminator,
        carrier_discriminator,
        cn0,
        early_late_spacing,
        tcoh,
    )
end

@testset "Vector-loop times difference on the GPS Time count" begin
    # One physical instant: a BDT seconds-of-week reads 14 s below the GPS time
    # of week (GPST is TAI−19, BDT is TAI−33). Everything the vector loop
    # *differences* across members — `reference_time`, the pseudoranges — must
    # therefore use `time_gpst_count`, not `time`; mixing raw counts hands every
    # BeiDou member 14 s × c ≈ 4.2×10⁹ m of structural pseudorange offset that
    # the ns-scale BGTO collapse constraint then fights rather than absorbs.
    gps = _test_member(; time = 100.0)
    bds = _test_member(;
        group_key = :BeiDouB2aI,
        signal = BeiDouB2aI(),
        clock_bias_index = 2,
        time = 86.0,
    )
    @test gps.time_gpst_count == 100.0
    @test bds.time_gpst_count ≈ gps.time_gpst_count
    # The own-scale transmit time stays untouched — it is what the ephemeris,
    # the clock polynomial and `calc_steering_offset` are evaluated at.
    @test bds.time == 86.0
end

@testset "VectorTracking configuration" begin
    config = VectorTracking()
    @test config.use_pseudorange_rates # VDFLL by default
    @test config.motion_model_order == 2
    @test config.clock_model_order == 2
    @test config.insufficient_meas_timeout == 10.0u"s"

    vdll = VectorTracking(use_pseudorange_rates = false)
    @test !vdll.use_pseudorange_rates

    @test_throws ArgumentError VectorTracking(motion_model_order = 0)
    @test_throws ArgumentError VectorTracking(motion_model_order = 4)
    @test_throws ArgumentError VectorTracking(clock_model_order = 0)
    @test_throws ArgumentError VectorTracking(clock_model_order = 3)
end

@testset "Navigation filter layout from the configured systems" begin
    # Single constellation, single band: one clock bias, no IFBs.
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(),))
    @test layout.time_systems == [GPST()]
    @test isempty(layout.extra_bands)
    @test layout.clock_bias_index_by_group[:GPSL1CA] == 1
    @test layout.ifb_index_by_group[:GPSL1CA] == 0

    # Two constellations sharing L1: two clock biases, still no IFBs.
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(), GalileoE1B()))
    @test layout.time_systems == [GPST(), GST()]
    @test isempty(layout.extra_bands)
    @test layout.clock_bias_index_by_group[:GPSL1CA] == 1
    @test layout.clock_bias_index_by_group[:GalileoE1B] == 2

    # GPS on two bands: one clock bias, one inter-frequency bias for the band
    # beyond the reference.
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(), GPSL5I()))
    @test layout.time_systems == [GPST()]
    @test length(layout.extra_bands) == 1
    @test layout.ifb_index_by_group[:GPSL1CA] == 0 ||
          layout.ifb_index_by_group[:GPSL5I] == 0
    @test layout.ifb_index_by_group[:GPSL1CA] + layout.ifb_index_by_group[:GPSL5I] == 1

    # A band stranded alone on its constellation gets no IFB column — its
    # delay folds into that constellation's clock (PositionVelocityTime's
    # observability-driven layout).
    layout = GNSSReceiver.NavFilterLayout((
        GPSL1CA(),
        CombinedSignal(GalileoE5aQ(), GalileoE5aI()),
    ))
    @test layout.time_systems == [GPST(), GST()]
    @test isempty(layout.extra_bands)
end

@testset "Navigation filter state indices" begin
    idxs = GNSSReceiver.NavFilterIndices(2, 2, 1, 0)
    @test idxs.pos == [1, 3, 5]
    @test idxs.vel == [2, 4, 6]
    @test isempty(idxs.acc)
    @test idxs.clock_biases == [7]
    @test idxs.clock_drift == 8
    @test isempty(idxs.ifb)

    # Two clock biases sharing one drift, plus one inter-frequency bias.
    idxs = GNSSReceiver.NavFilterIndices(2, 2, 2, 1)
    @test idxs.clock_biases == [7, 8]
    @test idxs.clock_drift == 9
    @test idxs.ifb == [10]

    idxs = GNSSReceiver.NavFilterIndices(1, 1, 1, 0)
    @test idxs.pos == [1, 2, 3]
    @test isempty(idxs.vel)
    @test idxs.clock_biases == [4]
    @test idxs.clock_drift == 0

    idxs = GNSSReceiver.NavFilterIndices(3, 2, 1, 0)
    @test idxs.pos == [1, 4, 7]
    @test idxs.vel == [2, 5, 8]
    @test idxs.acc == [3, 6, 9]
    @test idxs.clock_biases == [10]
    @test idxs.clock_drift == 11

    idxs = GNSSReceiver.NavFilterIndices(2, 2, 2, 1)
    x = zeros(10)
    x[idxs.pos] = [1.0, 2.0, 3.0]
    x[idxs.vel] = [4.0, 5.0, 6.0]
    x[idxs.clock_biases] = [7.0, 8.0]
    x[idxs.clock_drift] = 9.0
    x[idxs.ifb] = [10.0]
    pos, vel, clock_drift = GNSSReceiver.nav_filter_states(x, idxs)
    @test pos == [1.0, 2.0, 3.0]
    @test vel == [4.0, 5.0, 6.0]
    @test clock_drift == 9.0
    # The [x, y, z, tc₁.., ifb₁..] sub-vector PositionVelocityTime consumes.
    @test GNSSReceiver.position_and_bias_vector(x, idxs) ==
          [1.0, 2.0, 3.0, 7.0, 8.0, 10.0]

    # Unmodelled derivatives read as zero.
    idxs1 = GNSSReceiver.NavFilterIndices(1, 1, 1, 0)
    pos, vel, clock_drift = GNSSReceiver.nav_filter_states([1.0, 2.0, 3.0, 4.0], idxs1)
    @test pos == [1.0, 2.0, 3.0]
    @test vel == zeros(3)
    @test clock_drift == 0.0
end

@testset "Navigation filter process model" begin
    config = VectorTracking()
    T = 0.1
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(), GalileoE1B()))
    nav = GNSSReceiver.NavFilterModel(config, layout, 100.0u"ms")
    @test size(nav.F) == (9, 9)
    # Per-axis constant-velocity blocks propagate x += T * ẋ.
    for i in [1, 3, 5]
        @test nav.F[i, i] == 1.0
        @test nav.F[i, i+1] == T
        @test nav.F[i+1, i+1] == 1.0
        @test nav.F[i+1, i] == 0.0
    end
    # Both clock biases integrate the single oscillator drift.
    @test nav.F[7, 9] == T
    @test nav.F[8, 9] == T
    @test nav.F[9, 9] == 1.0
    # Process noise is symmetric positive semi-definite with positive variances,
    # and the drift random walk is fully correlated across the biases (one
    # oscillator).
    @test nav.Q ≈ nav.Q'
    @test all(diag(nav.Q) .> 0)
    @test all(eigvals(Symmetric(nav.Q)) .> -1e-12)
    @test nav.Q[7, 8] > 0

    # Position-only, bias-only model: pure identity dynamics.
    nav1 = GNSSReceiver.NavFilterModel(
        VectorTracking(; motion_model_order = 1, clock_model_order = 1),
        GNSSReceiver.NavFilterLayout((GPSL1CA(),)),
        100.0u"ms",
    )
    @test nav1.F == I(4)
    @test size(nav1.Q) == (4, 4)

    # An inter-frequency bias state is (nearly) constant.
    nav_ifb = GNSSReceiver.NavFilterModel(
        config,
        GNSSReceiver.NavFilterLayout((GPSL1CA(), GPSL5I())),
        100.0u"ms",
    )
    ifb_index = nav_ifb.idxs.ifb[1]
    @test nav_ifb.F[ifb_index, ifb_index] == 1.0
    @test all(nav_ifb.F[ifb_index, 1:end .!= ifb_index] .== 0.0)
    # The IFB random walk is driven by the configured density (m/√s), so a front end whose
    # bands drift apart can be described without touching the source.
    @test nav_ifb.Q[ifb_index, ifb_index] ≈
          ustrip(u"m/sqrt(s)", config.ifb_noise_density)^2 * T
    loose_ifb = GNSSReceiver.NavFilterModel(
        VectorTracking(; ifb_noise_density = 0.05u"m/sqrt(s)"),
        GNSSReceiver.NavFilterLayout((GPSL1CA(), GPSL5I())),
        100.0u"ms",
    )
    @test loose_ifb.Q[ifb_index, ifb_index] ≈ 25 * nav_ifb.Q[ifb_index, ifb_index]

    # Integration-time management: minor deviations are tolerated, larger ones
    # rebuild the process model.
    vt = GNSSReceiver.VectorTrackingState(config, GNSSReceiver.NavFilterLayout((GPSL1CA(),)))
    @test vt.nav_filter.integration_time == 100.0u"ms"
    @test GNSSReceiver.ensure_nav_filter_integration_time(vt, 100.5u"ms") === vt
    adapted = GNSSReceiver.ensure_nav_filter_integration_time(vt, 200.0u"ms")
    @test adapted.nav_filter.integration_time == 200.0u"ms"
    @test adapted.nav_filter.F[1, 2] ≈ 0.2
end

@testset "Motion process noise models the first unmodelled derivative" begin
    # `acceleration_noise_std` is the platform's agility at every motion model order: the
    # axis block is always `Γ Γᵀ σ²` for the first derivative the state does *not* carry,
    # with the orders that do not model the acceleration deriving their figure from it
    # through `MANOEUVRE_TIME`. Checked against the gain vectors written out by hand.
    acc = 5.0u"m/s^2"
    σ_a = ustrip(u"m/s^2", acc)
    τ = ustrip(u"s", GNSSReceiver.MANOEUVRE_TIME)
    σ_v = σ_a * τ   # velocity change over one manoeuvre
    σ_j = σ_a / τ   # jerk making up one manoeuvre
    T = 0.1
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(),))
    config(order) =
        VectorTracking(; motion_model_order = order, acceleration_noise_std = acc)
    axis_block(order) =
        GNSSReceiver.NavFilterModel(config(order), layout, T * u"s").Q[1:order, 1:order]

    @test GNSSReceiver.MANOEUVRE_TIME == 2.0u"s"
    @test axis_block(1) ≈ [T]  * [T]'  * σ_v^2
    @test axis_block(2) ≈ [T^2/2, T] * [T^2/2, T]' * σ_a^2
    @test axis_block(3) ≈ [T^3/6, T^2/2, T] * [T^3/6, T^2/2, T]' * σ_j^2

    # Order 2 is the one whose unmodelled derivative *is* the acceleration, so the manoeuvre
    # time constant does not enter: its driving noise is the configured figure itself.
    @test GNSSReceiver.motion_noise_model(config(2), T)[2] == σ_a
    # The two derived figures sit either side of it by the same factor, so one number
    # describes the platform at all three orders.
    @test GNSSReceiver.motion_noise_model(config(1), T)[2] / σ_a ≈ τ
    @test σ_a / GNSSReceiver.motion_noise_model(config(3), T)[2] ≈ τ

    # The physical figure is what is held fixed across filter intervals, not the resulting
    # variance: at every order the noise recovered from `Q` is the same number at 100 ms as
    # at 1 s. This is what a constant rescaling factor could not do — it is only right at the
    # interval it was tuned for.
    for (order, expected) in ((1, σ_v), (2, σ_a), (3, σ_j))
        recovered(interval) = let
            nav = GNSSReceiver.NavFilterModel(config(order), layout, interval)
            # The last modelled derivative's own gain is `T` at every order.
            sqrt(nav.Q[order, order]) / ustrip(u"s", interval)
        end
        @test recovered(0.1u"s") ≈ expected
        @test recovered(1.0u"s") ≈ expected
    end
end

@testset "Measurement prediction" begin
    config = VectorTracking()
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(), GalileoE1B()))
    nav = GNSSReceiver.NavFilterModel(config, layout, 100.0u"ms")
    idxs = nav.idxs

    # One GPS satellite along +x, one Galileo along +z: each pseudorange
    # carries its own system's clock bias.
    members = [
        _test_member(; prn = 1, clock_bias_index = 1, sat_position = SVector(2.6e7, 0.0, 0.0)),
        _test_member(;
            group_key = :GalileoE1B,
            prn = 2,
            clock_bias_index = 2,
            signal = GalileoE1B(),
            sat_position = SVector(0.0, 0.0, 2.6e7),
        ),
    ]
    bias_columns = GNSSReceiver.vt_bias_columns(members, layout)
    @test bias_columns.clock_bias_indices == [1, 2]
    @test bias_columns.num_clock_biases == 2
    @test bias_columns.num_ifb == 0

    user_pos = SVector(6.378e6, 0.0, 0.0)
    x = zeros(9)
    x[idxs.pos] = user_pos
    x[idxs.clock_biases] = [100.0, 200.0]
    ξ = GNSSReceiver.position_and_bias_vector(x, idxs)
    sat_positions_mat = stack(member.sat_position for member in members)

    psr = GNSSReceiver.predict_pseudoranges(ξ, sat_positions_mat, bias_columns)
    @test psr[1] ≈ 2.6e7 - 6.378e6 + 100.0
    @test psr[2] ≈ norm(user_pos - [0.0, 0.0, 2.6e7]) + 200.0 rtol = 1e-5

    # A satellite closing head-on with 1000 m/s reads +1000 m/s; a receiver
    # clock drift of +10 m/s reduces the predicted rate accordingly.
    rate = GNSSReceiver.predict_pseudorange_rates(
        user_pos,
        SVector(0.0, 0.0, 0.0),
        0.0,
        [SVector(2.6e7, 0.0, 0.0)],
        [SVector(-1000.0, 0.0, 0.0)],
        [0.0],
    )
    @test rate[1] ≈ 1000.0 atol = 1e-6
    rate_with_drift = GNSSReceiver.predict_pseudorange_rates(
        user_pos,
        SVector(0.0, 0.0, 0.0),
        10.0,
        [SVector(2.6e7, 0.0, 0.0)],
        [SVector(-1000.0, 0.0, 0.0)],
        [0.0],
    )
    @test rate_with_drift[1] ≈ 990.0 atol = 1e-6

    J = GNSSReceiver.nav_filter_jacobian(nav, x, members, sat_positions_mat, bias_columns)
    @test size(J) == (4, 9)
    # Pseudorange rows: line-of-sight in the position columns (satellite 1
    # along +x ⇒ ∂ρ/∂x = -1), 1 in the member's own clock-bias column only.
    @test J[1, idxs.pos] ≈ [-1.0, 0.0, 0.0] atol = 1e-3
    @test J[1, idxs.clock_biases[1]] == 1.0
    @test J[1, idxs.clock_biases[2]] == 0.0
    @test J[2, idxs.clock_biases[1]] == 0.0
    @test J[2, idxs.clock_biases[2]] == 1.0
    @test J[1, idxs.clock_drift] == 0.0
    # Pseudorange-rate rows: negated line of sight in the velocity columns, -1
    # in the (single, shared) clock-drift column.
    @test J[3, idxs.vel] ≈ [1.0, 0.0, 0.0] atol = 1e-3
    @test J[3, idxs.clock_biases[1]] == 0.0
    @test J[3, idxs.clock_drift] == -1.0
    @test J[4, idxs.clock_drift] == -1.0

    # An inter-frequency-bias member additionally carries a 1 in its band's
    # IFB column.
    layout_l5 = GNSSReceiver.NavFilterLayout((GPSL1CA(), GPSL5I()))
    nav_l5 = GNSSReceiver.NavFilterModel(config, layout_l5, 100.0u"ms")
    l5_member = _test_member(;
        group_key = :GPSL5I,
        signal = GPSL5I(),
        clock_bias_index = 1,
        ifb_index = 1,
    )
    x_l5 = zeros(size(nav_l5.F, 1))
    x_l5[nav_l5.idxs.pos] = user_pos
    J_l5 = GNSSReceiver.nav_filter_jacobian(
        nav_l5,
        x_l5,
        [l5_member],
        stack([l5_member.sat_position]),
        GNSSReceiver.vt_bias_columns([l5_member], layout_l5),
    )
    @test J_l5[1, nav_l5.idxs.ifb[1]] == 1.0
end

@testset "Measurement noise and innovation gates" begin
    members = [
        _test_member(; prn = 1),
        _test_member(; prn = 2, signal = GPSL5I(), group_key = :GPSL5I),
    ]
    R = GNSSReceiver.vt_measurement_noise_covariance(members, 100.0u"ms")
    @test size(R) == (4, 4)
    @test isdiag(R)
    # The DLL thermal noise scales with the squared chip length: GPS L5's chips
    # are 10× shorter than L1 C/A's, so its thermal variance is 100× smaller.
    # (L1 C/A and L5I share a 1 ms coherent dump, so the squaring loss cancels here.)
    @test R[1, 1] / R[2, 2] ≈ (members[1].chip_length / members[2].chip_length)^2
    # The rate rows scale with the squared carrier wavelength.
    @test R[3, 3] / R[4, 4] ≈ (members[1].wavelength / members[2].wavelength)^2

    # Both rows use each member's coherent integration time: GPS L1 C/A integrates 1 ms per
    # dump, Galileo E1B 4 ms, so C/A is the noisier measurement in both — the rate through
    # the ATAN FLL phase noise, the range through the noncoherent DLL squaring loss.
    ca = _test_member(; prn = 1, signal = GPSL1CA(), group_key = :GPSL1CA)
    e1b = _test_member(; prn = 1, signal = GalileoE1B(), group_key = :GalileoE1B)
    @test ca.coherent_integration_time ≈ 1e-3
    @test e1b.coherent_integration_time ≈ 4e-3
    Tf = 0.1
    Rte = GNSSReceiver.vt_measurement_noise_covariance([ca, e1b], Tf * u"s")
    # Averaged noncoherent DLL variance d/(4·C/N0·T)·(1 + 2/((2−d)·C/N0·T_coh)), in metres².
    function range_var(m, T = Tf)
        d = m.early_late_spacing
        squaring_loss = 1 + 2 / ((2 - d) * m.cn0 * m.coherent_integration_time)
        d / (4 * T * m.cn0) * squaring_loss * m.chip_length^2
    end
    # Mean-FLL variance 2·σ_φ²/(2π·T)² with σ_φ² = 1/(2·C/N0·T_coh)(1+1/(2·C/N0·T_coh)),
    # expressed as a rate (λ²). No inflation factor: the derived single-cycle variance is the
    # right one for this estimator, and the lag-1 correlation the telescoping creates makes a
    # white-noise filter pessimistic rather than overconfident.
    rate_var(m) = let ct = m.cn0 * m.coherent_integration_time
        m.wavelength^2 * (1 / (2 * ct) * (1 + 1 / (2 * ct))) / (2 * π^2 * Tf^2)
    end
    @test Rte[1, 1] ≈ range_var(ca)
    @test Rte[2, 2] ≈ range_var(e1b)
    @test Rte[3, 3] ≈ rate_var(ca)
    @test Rte[4, 4] ≈ rate_var(e1b)
    @test Rte[3, 3] > Rte[4, 4]   # shorter-T_coh C/A is the noisier rate measurement
    # E1B's chips are 293 m against C/A's 293 m, so at equal C/N0 and spacing the only
    # difference in the range rows is the squaring loss — C/A's shorter dump costs it more.
    @test ca.chip_length ≈ e1b.chip_length
    @test Rte[1, 1] > Rte[2, 2]

    # The squaring loss is pinned to the coherent dump, not the filter interval: lengthening
    # the filter interval averages the thermal term down but cannot buy back the loss, so the
    # variance falls more slowly than 1/T.
    R_short = GNSSReceiver.vt_measurement_noise_covariance([ca], 0.1u"s")
    R_long = GNSSReceiver.vt_measurement_noise_covariance([ca], 1.0u"s")
    @test R_short[1, 1] / R_long[1, 1] ≈ 10.0
    # A weak-signal member is dominated by the squaring loss, which no filter interval fixes.
    weak = _test_member(; cn0 = 10^2.0) # 20 dB-Hz
    R_weak = GNSSReceiver.vt_measurement_noise_covariance([weak], 0.1u"s")
    weak_loss = 1 + 2 / ((2 - weak.early_late_spacing) * weak.cn0 * weak.coherent_integration_time)
    @test weak_loss > 10
    @test R_weak[1, 1] ≈ range_var(weak, 0.1)

    # The model is evaluated at each member's configured spacing as given: it assumes the early
    # and late taps bracket the correlation peak (`d < 2` chips), which every real correlator
    # does — Tracking's own default is 0.5. Narrowing the correlator at equal C/N0 lowers the
    # thermal term proportionally, which is the whole point of a narrow spacing.
    narrow = _test_member(; early_late_spacing = 0.5)
    R_narrow = GNSSReceiver.vt_measurement_noise_covariance([narrow], 0.1u"s")
    @test R_narrow[1, 1] ≈ range_var(narrow, 0.1)
    @test R_narrow[1, 1] < range_var(ca, 0.1)   # same C/N0 and dump, wider spacing

    # With a confident filter the gate is the physical two-chip bracket; with
    # a large prior uncertainty (e.g. an unobserved clock bias) it widens to
    # the innovation's own predicted std so the Kalman update can absorb it.
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(),))
    nav = GNSSReceiver.NavFilterModel(VectorTracking(), layout, 100.0u"ms")
    member = _test_member(;)
    x = zeros(8)
    x[nav.idxs.pos] = [6.378e6, 0.0, 0.0]
    bias_columns = GNSSReceiver.vt_bias_columns([member], layout)
    J = GNSSReceiver.nav_filter_jacobian(
        nav,
        x,
        [member],
        stack([member.sat_position]),
        bias_columns,
    )
    R1 = GNSSReceiver.vt_measurement_noise_covariance([member], 100.0u"ms")
    P_confident = Matrix(1.0I, 8, 8)
    gates = GNSSReceiver.innovation_gates([member], J[1:1, :], P_confident, R1[1:1, 1:1])
    @test gates[1] ≈ 2 * member.chip_length
    P_unsure = copy(P_confident)
    P_unsure[nav.idxs.clock_biases[1], nav.idxs.clock_biases[1]] = 1e6^2
    gates = GNSSReceiver.innovation_gates([member], J[1:1, :], P_unsure, R1[1:1, 1:1])
    @test gates[1] > 4e6
end

@testset "Bias observability and the broadcast-clock collapse" begin
    # GPS on L1 + Galileo on E1B and E5a: 2 clock biases, 1 inter-frequency
    # bias, so a full independent layout needs 3 + 2 + 1 = 6 pseudoranges.
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(), GalileoE1B(), GalileoE5aI()))
    vt = GNSSReceiver.VectorTrackingState(VectorTracking(), layout)
    gps_clock = layout.clock_bias_index_by_group[:GPSL1CA]
    gal_clock = layout.clock_bias_index_by_group[:GalileoE1B]
    l5_ifb = layout.ifb_index_by_group[:GalileoE5aI]
    @test l5_ifb != 0

    gps(prn) = _test_member(; prn, group_key = :GPSL1CA, clock_bias_index = gps_clock)
    e1b(prn) = _test_member(;
        prn,
        group_key = :GalileoE1B,
        signal = GalileoE1B(),
        clock_bias_index = gal_clock,
    )
    e5a(prn) = _test_member(;
        prn,
        group_key = :GalileoE5aI,
        signal = GalileoE5aI(),
        clock_bias_index = gal_clock,
        ifb_index = l5_ifb,
    )

    # Only the bias states a measurement actually touches are unknowns: GPS
    # alone on L1 leaves the Galileo clock and the L5 bias out of the count.
    members = [gps(i) for i = 1:5]
    obs = GNSSReceiver.assess_bias_observability(vt, members, 1:5)
    @test obs.num_unknowns == 4          # 3 position + 1 GPS clock
    @test isempty(obs.hub_offset_constraints) # nothing to collapse

    # All three signals present and plentiful: everything is estimated
    # independently, with no broadcast GGTO error entering the solution.
    members = [gps(1), gps(2), gps(3), gps(4), e1b(5), e1b(6), e5a(7), e5a(8)]
    obs = GNSSReceiver.assess_bias_observability(vt, members, 1:8)
    @test obs.num_unknowns == 6          # 3 + 2 clocks + 1 IFB
    @test obs.num_distinct_sats_required == 5 # 3 position + 2 clocks
    @test obs.num_distinct_sats == 8
    @test GNSSReceiver.is_epoch_solvable(obs, 8)
    @test isempty(obs.hub_offset_constraints)

    # The same layout with only five measurements is one short. Without a
    # decoded GGTO there is nothing to collapse onto, so the epoch stays
    # under-determined and the count says so — this is what grows the
    # starvation timer where the old flat `>= 4` test saw a healthy epoch.
    members = [gps(1), gps(2), gps(3), e1b(4), e5a(5)]
    obs = GNSSReceiver.assess_bias_observability(vt, members, 1:5)
    @test obs.num_unknowns == 6
    @test length(members) < obs.num_unknowns
    @test !GNSSReceiver.is_epoch_solvable(obs, length(members))
    @test isempty(obs.hub_offset_constraints)

    # With the GGTO decoded, the Galileo clock collapses onto the GPS one: one
    # unknown fewer, and the collapse is handed back as the pseudo-measurement
    # `clk_GST - clk_GPST = -c * GGTO`.
    function with_ggto(decoder, a_0g)
        data = decoder.data
        data = @set data.A_0G = a_0g
        data = @set data.A_1G = 0.0
        data = @set data.t_0G = 0
        data = @set data.WN_0G = 0
        data = @set data.WN = 0
        @set decoder.data = data
    end
    ggto_e1b = _test_member(;
        prn = 4,
        group_key = :GalileoE1B,
        signal = GalileoE1B(),
        clock_bias_index = gal_clock,
        decoder = with_ggto(test_decoder_state(GalileoE1B(), 4), 1e-9),
    )
    @test time_offset_available(ggto_e1b.sat_state.decoder, GPST())
    members = [gps(1), gps(2), gps(3), ggto_e1b, e5a(5)]
    obs = GNSSReceiver.assess_bias_observability(vt, members, 1:5)
    @test obs.num_unknowns == 5
    @test length(members) >= obs.num_unknowns
    gst_state, gpst_state, isb = only(obs.hub_offset_constraints)
    @test gst_state == gal_clock
    @test gpst_state == gps_clock
    @test isb ≈ -SPEED_OF_LIGHT * 1e-9

    # A disconnected coverage graph — GPS only on L1, Galileo only on E5a, no
    # E1B to link the bands — leaves the L5 bias collinear with the Galileo
    # clock however many satellites there are. The collapse is what reconnects
    # it, so it is applied even though the plain count would have passed.
    #
    # Without a collapse the unknowns are counted on the epoch's own coverage graph, as
    # `decide_bias_layout` counts them: with the two bands in separate components neither
    # carries an observable inter-frequency bias (each is its component's reference), so the
    # E5a delay folds into the Galileo clock and the epoch has 3 + 2 unknowns, not 6. The
    # bias decomposition is ambiguous there, but the epoch is solvable and must not grow the
    # starvation timer.
    members = [gps(1), gps(2), gps(3), gps(4), gps(5), gps(6), e5a(7)]
    obs = GNSSReceiver.assess_bias_observability(vt, members, 1:7)
    @test obs.num_unknowns == 5
    @test isempty(obs.hub_offset_constraints) # this E5a member carries no GGTO
    members[7] = _test_member(;
        prn = 7,
        group_key = :GalileoE5aI,
        signal = GalileoE5aI(),
        clock_bias_index = gal_clock,
        ifb_index = l5_ifb,
        decoder = with_ggto(test_decoder_state(GalileoE5aI(), 7), 2e-9),
    )
    # Merging Galileo onto GPS reconnects the two bands, which both removes a clock unknown
    # *and* makes the E5a inter-frequency bias observable (and countable) again — so the
    # merged count is recomputed on the merged graph rather than decremented: 3 + 1 clock +
    # 1 IFB. The two effects happen to cancel here; they are separately real.
    obs = GNSSReceiver.assess_bias_observability(vt, members, 1:7)
    @test obs.num_unknowns == 5
    @test length(obs.hub_offset_constraints) == 1

    # A band whose component reference carries no measurement this cycle is not an unknown
    # of the cycle: with the L1 satellites gone, the L5 delay is collinear with the (single)
    # GPS clock and folds into it, so four L5 satellites determine the epoch — counting the
    # *configured* layout's IFB column instead would demand five and grow the starvation
    # timer through an L1 outage the filter is handling perfectly well.
    dual_band = GNSSReceiver.NavFilterLayout((GPSL1CA(), GPSL5I()))
    vt_dual = GNSSReceiver.VectorTrackingState(VectorTracking(), dual_band)
    @test dual_band.ifb_index_by_group[:GPSL5I] != 0
    l5(prn) = _test_member(;
        prn,
        group_key = :GPSL5I,
        signal = GPSL5I(),
        clock_bias_index = dual_band.clock_bias_index_by_group[:GPSL5I],
        ifb_index = dual_band.ifb_index_by_group[:GPSL5I],
    )
    obs = GNSSReceiver.assess_bias_observability(vt_dual, [l5(i) for i = 1:4], 1:4)
    @test obs.num_unknowns == 4
    @test obs.num_distinct_sats == 4
    @test GNSSReceiver.is_epoch_solvable(obs, 4)
    @test isempty(obs.hub_offset_constraints)

    # With both bands present the bias is observable again and does count.
    l1(prn) = _test_member(; prn, group_key = :GPSL1CA)
    both_bands = [l1(1), l1(2), l1(3), l5(1), l5(2)]
    obs = GNSSReceiver.assess_bias_observability(vt_dual, both_bands, 1:5)
    @test obs.num_unknowns == 5

    # The measurement count alone is not enough, and this is the case that shows it: three
    # satellites tracked on two bands each make six measurements against five unknowns, so
    # the count passes — but a satellite's second band is not a second line of sight, and
    # the six design rows take only four distinct values outside the IFB column. The epoch
    # is rank deficient and must not pay the starvation timer down. `decide_bias_layout`
    # makes the same distinction with the same `(time system, PRN)` identification.
    three_sats_two_bands = [l1(1), l1(2), l1(3), l5(1), l5(2), l5(3)]
    obs = GNSSReceiver.assess_bias_observability(vt_dual, three_sats_two_bands, 1:6)
    @test obs.num_unknowns == 5
    @test 6 >= obs.num_unknowns              # the plain count is satisfied …
    @test obs.num_distinct_sats == 3         # … by three satellites' worth of geometry
    @test obs.num_distinct_sats_required == 4
    @test !GNSSReceiver.is_epoch_solvable(obs, 6)

    # A fourth satellite on either band supplies the missing line of sight.
    obs = GNSSReceiver.assess_bias_observability(
        vt_dual,
        [three_sats_two_bands; l1(4)],
        1:7,
    )
    @test obs.num_distinct_sats == 4
    @test GNSSReceiver.is_epoch_solvable(obs, 7)

    # A mixed epoch collapses each non-GPS clock through its own broadcast offset —
    # Galileo through the GGTO, BeiDou through the BGTO — independently and in one cycle,
    # which is what `decide_bias_layout` does on the scalar side. B1I is on its own
    # carrier, so the independent layout's coverage graph is disconnected (GPS and Galileo
    # on L1, BeiDou alone on B1I) and the collapse is what reconnects it: two clock
    # unknowns go and the B1I inter-frequency bias becomes observable, and countable,
    # again — 3 + 3 clocks + 0 IFBs recounted as 3 + 1 clock + 1 IFB.
    tri = GNSSReceiver.NavFilterLayout((GPSL1CA(), GalileoE1B(), BeiDouB1I()))
    vt_tri = GNSSReceiver.VectorTrackingState(VectorTracking(), tri)
    gal_tri = tri.clock_bias_index_by_group[:GalileoE1B]
    bds_tri = tri.clock_bias_index_by_group[:BeiDouB1I]
    # The legacy D1 message broadcasts the BGTO as a bare `A_0GPS`/`A_1GPS` pair with no
    # reference epoch, so at `time = 0.0` the offset is `A_0GPS` itself.
    function with_bgto(decoder, a_0gps)
        data = decoder.data
        data = @set data.A_0GPS = a_0gps
        data = @set data.A_1GPS = 0.0
        data = @set data.WN = 0
        @set decoder.data = data
    end
    gal_member = _test_member(;
        prn = 4,
        group_key = :GalileoE1B,
        signal = GalileoE1B(),
        clock_bias_index = gal_tri,
        decoder = with_ggto(test_decoder_state(GalileoE1B(), 4), 1e-9),
    )
    bds_member = _test_member(;
        prn = 21,
        group_key = :BeiDouB1I,
        signal = BeiDouB1I(),
        clock_bias_index = bds_tri,
        decoder = with_bgto(test_decoder_state(BeiDouB1I(), 21), 3e-9),
    )
    @test time_offset_available(bds_member.sat_state.decoder, GPST())
    gps_tri = tri.clock_bias_index_by_group[:GPSL1CA]
    gps_of(prn) = _test_member(; prn, group_key = :GPSL1CA, clock_bias_index = gps_tri)
    mixed = [gps_of(1), gps_of(2), gps_of(3), gal_member, bds_member]
    obs = GNSSReceiver.assess_bias_observability(vt_tri, mixed, 1:5)
    @test obs.num_unknowns == 5              # 3 position + the surviving GPS clock + 1 IFB
    @test obs.num_distinct_sats_required == 4
    @test GNSSReceiver.is_epoch_solvable(obs, 5)
    @test length(obs.hub_offset_constraints) == 2
    by_state = Dict(
        state => (ref, isb) for (state, ref, isb) in obs.hub_offset_constraints
    )
    @test by_state[gal_tri][1] == gps_tri
    @test by_state[gal_tri][2] ≈ -SPEED_OF_LIGHT * 1e-9
    @test by_state[bds_tri][1] == gps_tri
    # Explicit tolerance: `calc_steering_offset` recovers the ~ns BDT steering residual by
    # subtracting the defined 14 s back out of `A_0`, which costs one ULP at 14
    # (~1.8e-15 s, ≈5.5e-7 m of range) — above the default relative tolerance on a
    # value this small. Same reasoning as PVT's own BGTO tests.
    @test by_state[bds_tri][2] ≈ -SPEED_OF_LIGHT * 3e-9 atol = 1e-5

    # Only the systems that actually carry an offset collapse: drop the BeiDou one's BGTO
    # and its clock stays an unknown of its own while Galileo's still merges.
    mixed[5] = _test_member(;
        prn = 21,
        group_key = :BeiDouB1I,
        signal = BeiDouB1I(),
        clock_bias_index = bds_tri,
    )
    obs = GNSSReceiver.assess_bias_observability(vt_tri, mixed, 1:5)
    @test length(obs.hub_offset_constraints) == 1
    @test only(obs.hub_offset_constraints)[1] == gal_tri
    @test obs.num_unknowns == 5               # 3 position + GPS clock + BeiDou clock

    # The hub is not GPS-specific: with GPS absent, the same cycle collapses onto
    # Galileo instead — the D1 message broadcasts a BDT–GST offset (`A_0Gal`)
    # alongside the GPS one, and `assess_bias_observability` tries GST as the hub
    # once GPST is not in play, in the same fixed hub order as `decide_bias_layout`.
    duo = GNSSReceiver.NavFilterLayout((GalileoE1B(), BeiDouB1I()))
    vt_duo = GNSSReceiver.VectorTrackingState(VectorTracking(), duo)
    gal_duo = duo.clock_bias_index_by_group[:GalileoE1B]
    bds_duo = duo.clock_bias_index_by_group[:BeiDouB1I]
    function with_gal_bgto(decoder, a_0gal)
        data = decoder.data
        data = @set data.A_0Gal = a_0gal
        data = @set data.A_1Gal = 0.0
        data = @set data.WN = 0
        @set decoder.data = data
    end
    bds_gal_member = _test_member(;
        prn = 22,
        group_key = :BeiDouB1I,
        signal = BeiDouB1I(),
        clock_bias_index = bds_duo,
        decoder = with_gal_bgto(test_decoder_state(BeiDouB1I(), 22), 4e-9),
    )
    @test time_offset_available(bds_gal_member.sat_state.decoder, GST())
    gal_of(prn) = _test_member(;
        prn, group_key = :GalileoE1B, signal = GalileoE1B(), clock_bias_index = gal_duo)
    gps_free = [gal_of(1), gal_of(2), gal_of(3), bds_gal_member]
    obs = GNSSReceiver.assess_bias_observability(vt_duo, gps_free, 1:4)
    @test length(obs.hub_offset_constraints) == 1
    gst_constraint = only(obs.hub_offset_constraints)
    @test gst_constraint[1] == bds_duo
    @test gst_constraint[2] == gal_duo
    # The BDT count offset against GST is the same defined 14 s as against GPST, so
    # the recovered steering residual carries the same one-ULP-at-14 tolerance.
    @test gst_constraint[3] ≈ -SPEED_OF_LIGHT * 4e-9 atol = 1e-5
    # The collapse reconnects the disjoint L1/B1I split: 3 position + one merged
    # clock + the now-observable B1I inter-frequency bias.
    @test obs.num_unknowns == 5

    # The position uncertainty reported with the solution is the root sum of the position
    # variances, read off the position block of P.
    idxs = vt.nav_filter.idxs
    P = zeros(size(vt.covariance))
    P[idxs.pos, idxs.pos] = Diagonal([9.0, 16.0, 144.0])
    @test GNSSReceiver.position_uncertainty(P, idxs) ≈ sqrt(169.0)
end

@testset "Reported DOP never carries calc_DOP's sentinel" begin
    # `calc_DOP` reports a rank-deficient geometry as an all-`-1` `DOP` rather than throwing.
    # `calc_pvt` never emits one (it rejects the epoch instead), so the vector loop must not
    # either: a consumer is entitled to read `pvt.dop` as either a real geometry or `nothing`,
    # and a negative DOP on a logarithmic axis takes a plot down with a `DomainError`.
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(),))
    vt = GNSSReceiver.VectorTrackingState(VectorTracking(), layout)
    state = copy(vt.state)
    state[vt.nav_filter.idxs.pos] = [6.378e6, 0.0, 0.0]
    vt = GNSSReceiver.VectorTrackingState(vt; state)

    # One measurement cannot determine three position components and a clock, so the design is
    # rank deficient and `calc_DOP` returns its sentinel.
    lone = [_test_member(; prn = 1, sat_position = SVector(2.6e7, 0.0, 0.0))]
    H_lone = calc_H(
        stack(m.sat_position for m in lone),
        GNSSReceiver.position_and_bias_vector(vt.state, vt.nav_filter.idxs),
        first(GNSSReceiver.dense_bias_columns(lone, vt.primary_clock_index)),
    )
    @test calc_DOP(H_lone, ECEF(6.378e6, 0.0, 0.0), 1).GDOP < 0
    pvt = GNSSReceiver.vt_pvt_solution(vt, lone, [1], [1.0u"m"], [0.1u"m/s"])
    @test isnothing(pvt.dop)

    # A geometry that *is* determined reports a real DOP, so the guard above does not simply
    # suppress every DOP.
    spread = [
        _test_member(; prn = 1, sat_position = SVector(2.6e7, 0.0, 0.0)),
        _test_member(; prn = 2, sat_position = SVector(0.0, 2.6e7, 0.0)),
        _test_member(; prn = 3, sat_position = SVector(0.0, 0.0, 2.6e7)),
        _test_member(; prn = 4, sat_position = SVector(1.5e7, 1.5e7, 1.5e7)),
    ]
    pvt = GNSSReceiver.vt_pvt_solution(
        vt, spread, 1:4, fill(1.0u"m", 4), fill(0.1u"m/s", 4),
    )
    @test !isnothing(pvt.dop)
    @test pvt.dop.GDOP > 0
    # And an update that measured nothing has no geometry to report at all.
    @test isnothing(
        GNSSReceiver.vt_pvt_solution(
            vt, spread, Int[], fill(1.0u"m", 4), fill(0.1u"m/s", 4),
        ).dop,
    )
end

@testset "Solution epoch from the cached week offset" begin
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(),))
    vt = GNSSReceiver.VectorTrackingState(VectorTracking(), layout)

    # Until a decoded primary-system satellite has been seen there is no epoch to report, so
    # the solution carries no timestamp and reads as "no fix" — as it must.
    @test isnothing(vt.time_epoch_offset)
    @test isnothing(GNSSReceiver.vt_time(vt))

    # With the offset known, the epoch is the clock-bias-corrected reference time on top of it:
    # a receiver clock one light-second fast puts the same reference time one second earlier.
    week_offset = 2052 * GNSSReceiver.SECONDS_PER_WEEK
    vt = GNSSReceiver.VectorTrackingState(
        vt;
        reference_time = 123.25u"s",
        time_epoch_offset = week_offset,
    )
    epoch = GNSSReceiver.vt_time(vt)
    @test !isnothing(epoch)
    biased_state = copy(vt.state)
    biased_state[vt.nav_filter.idxs.clock_biases[1]] = SPEED_OF_LIGHT
    biased_epoch =
        GNSSReceiver.vt_time(GNSSReceiver.VectorTrackingState(vt; state = biased_state))
    @test AstroTime.value(AstroTime.seconds(epoch - biased_epoch)) ≈ 1.0 rtol = 1e-9

    # The offset is cached: once resolved it is reused even on a cycle that could not resolve
    # one (no receiver satellite states are passed here at all), which is what keeps a valid
    # filter solution from silently losing its timestamp.
    @test GNSSReceiver.resolve_time_epoch_offset(vt, (GPSL1CA(),), (;), 2019) == week_offset

    # A week rollover takes a week off the time of week, so the offset — the only other place
    # the run's absolute epoch is held — has to gain one, or every later solution would be
    # timestamped exactly one week in the past.
    @test GNSSReceiver.rolled_over_time_epoch_offset(week_offset, week_offset, true) ==
          week_offset + GNSSReceiver.SECONDS_PER_WEEK
    @test GNSSReceiver.rolled_over_time_epoch_offset(week_offset, week_offset, false) ==
          week_offset
    # An offset first resolved on the very cycle that wraps comes from a decoder that has
    # rolled its own week number over already, so it must not be advanced a second time.
    @test GNSSReceiver.rolled_over_time_epoch_offset(week_offset, nothing, true) ==
          week_offset
    @test isnothing(GNSSReceiver.rolled_over_time_epoch_offset(nothing, nothing, true))

    # End to end across the boundary: one cycle before the wrap and one after must be a
    # single integration time apart, not a week.
    before = GNSSReceiver.VectorTrackingState(
        vt;
        reference_time = (GNSSReceiver.SECONDS_PER_WEEK - 0.1)u"s",
    )
    after = GNSSReceiver.VectorTrackingState(
        before;
        reference_time = 0.0u"s",
        time_epoch_offset = GNSSReceiver.rolled_over_time_epoch_offset(
            before.time_epoch_offset,
            before.time_epoch_offset,
            true,
        ),
    )
    step = GNSSReceiver.vt_time(after) - GNSSReceiver.vt_time(before)
    @test AstroTime.value(AstroTime.seconds(step)) ≈ 0.1 rtol = 1e-6
end

@testset "Reporting primary clock selection" begin
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(), GalileoE1B()))
    gps_clock = layout.clock_bias_index_by_group[:GPSL1CA]
    gal_clock = layout.clock_bias_index_by_group[:GalileoE1B]
    gps(prn) = _test_member(; prn, group_key = :GPSL1CA, clock_bias_index = gps_clock)
    e1b(prn) = _test_member(; prn, group_key = :GalileoE1B, signal = GalileoE1B(),
        clock_bias_index = gal_clock)
    both = [gps(1), gps(2), e1b(3)]

    # Kept while its own time system still contributes a measurement — no flicker.
    @test GNSSReceiver.report_primary_clock_index(layout, both, 1:3, gal_clock) == gal_clock
    @test GNSSReceiver.report_primary_clock_index(layout, both, 1:3, gps_clock) == gps_clock

    # Re-picked when the current primary's system is absent (index 0 is never
    # present): GPST is preferred when GPS is in the fix.
    @test GNSSReceiver.report_primary_clock_index(layout, both, 1:3, 0) == gps_clock

    # Else the most-populated time system (GPS absent ⇒ Galileo).
    @test GNSSReceiver.report_primary_clock_index(layout, [e1b(2), e1b(3)], 1:2, gps_clock) ==
          gal_clock

    # No included measurements at all: keep whatever it was.
    @test GNSSReceiver.report_primary_clock_index(layout, both, Int[], gps_clock) == gps_clock
end

@testset "Dense bias columns for DOP" begin
    # The filter's fixed bias layout carries a clock for every configured
    # constellation, but a DOP design matrix must only carry the columns the
    # epoch can determine — an empty column would make `HᵀH` singular and
    # `calc_DOP` report -1 whenever a constellation is missing.
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(), GalileoE1B(), GalileoE5aI()))
    gal_clock = layout.clock_bias_index_by_group[:GalileoE1B]
    l5_ifb = layout.ifb_index_by_group[:GalileoE5aI]
    members = [
        _test_member(;
            prn,
            group_key = :GalileoE1B,
            signal = GalileoE1B(),
            clock_bias_index = gal_clock,
        ) for prn = 1:5
    ]
    columns, primary = GNSSReceiver.dense_bias_columns(members, 1)
    @test columns.num_clock_biases == 1
    @test columns.num_ifb == 0
    @test all(columns.clock_bias_indices .== 1)
    @test primary == 1 # the GPS primary is not present; falls back to the first

    # Adding an E5a member brings its band's bias back as a real column.
    push!(
        members,
        _test_member(;
            prn = 9,
            group_key = :GalileoE5aI,
            signal = GalileoE5aI(),
            clock_bias_index = gal_clock,
            ifb_index = l5_ifb,
        ),
    )
    columns, _ = GNSSReceiver.dense_bias_columns(members, gal_clock)
    @test columns.num_clock_biases == 1
    @test columns.num_ifb == 1
    @test columns.ifb_indices == [0, 0, 0, 0, 0, 1]
end

@testset "NCO loop-closure corrections" begin
    layout = GNSSReceiver.NavFilterLayout((GPSL1CA(),))
    nav = GNSSReceiver.NavFilterModel(VectorTracking(), layout, 100.0u"ms")
    user_pos = SVector(6.378e6, 0.0, 0.0)
    member = _test_member(; prn = 7, pseudorange_rate = 100.0)
    x = zeros(8)
    x[nav.idxs.pos] = user_pos
    sat_positions_mat = stack([member.sat_position])
    bias_columns = GNSSReceiver.vt_bias_columns([member], layout)
    J = GNSSReceiver.nav_filter_jacobian(nav, x, [member], sat_positions_mat, bias_columns)
    c = SPEED_OF_LIGHT

    # A predicted pseudorange 10 m beyond the measured one must speed the code
    # NCO up by f_code · 10 m / (T · c): the replica is 10 m late. A predicted
    # rate 5 m/s above the measured one maps to +5/λ Hz on the carrier NCO.
    code_updates, carrier_updates = GNSSReceiver.nco_corrections(
        [member],
        [1],
        [1000.0 + 10.0],
        [1000.0],
        [105.0],
        100.0u"ms",
    )
    @test ustrip(u"Hz", code_updates[7]) ≈ -10.0 * member.code_frequency / (0.1 * c)
    @test ustrip(u"Hz", carrier_updates[7]) ≈ 5.0 / member.wavelength

    # The prediction is evaluated at the updated state, so it *is* the whole
    # correction — the measurement update's state correction must not be added
    # on top of it a second time. A satellite the filter moved 10 m towards
    # therefore gets the same single-counted NCO update as above, not double.
    x_updated = copy(x)
    x_updated[nav.idxs.pos] = user_pos .+ [10.0, 0.0, 0.0] # +x is towards the satellite
    predicted_at_updated = GNSSReceiver.predict_pseudoranges(
        GNSSReceiver.position_and_bias_vector(x_updated, nav.idxs),
        sat_positions_mat,
        bias_columns,
    )
    predicted_at_seed = GNSSReceiver.predict_pseudoranges(
        GNSSReceiver.position_and_bias_vector(x, nav.idxs),
        sat_positions_mat,
        bias_columns,
    )
    @test predicted_at_updated[1] - predicted_at_seed[1] ≈ -10.0 rtol = 1e-3
    code_updates, _ = GNSSReceiver.nco_corrections(
        [member],
        [1],
        predicted_at_updated,
        predicted_at_seed,
        [100.0],
        100.0u"ms",
    )
    @test ustrip(u"Hz", code_updates[7]) ≈ 10.0 * member.code_frequency / (0.1 * c) rtol =
        1e-3
end

@testset "Post-fit pseudorange and range-rate residuals" begin
    # Two members: one whose replica the loop has steered onto the navigation solution
    # (the model at the updated state reproduces both its measured pseudorange and its
    # replica's own Doppler), and one the solution predicts 30 m short and 2 m/s slow of
    # what it measures.
    steered = _test_member(;
        prn = 4,
        pseudorange_rate = 800.0,
        code_discriminator = 0.02,   # chips
        carrier_discriminator = 5.0, # Hz
    )
    diverged = _test_member(; prn = 9, pseudorange_rate = -600.0)
    members = [steered, diverged]
    measured_pseudoranges = [2.05e7, 2.10e7]
    predicted_pseudoranges = [measured_pseudoranges[1], measured_pseudoranges[2] - 30.0]
    predicted_rates = [steered.pseudorange_rate, diverged.pseudorange_rate - 2.0]

    residuals, rate_residuals = GNSSReceiver.vt_post_fit_residuals(
        members,
        measured_pseudoranges,
        predicted_pseudoranges,
        predicted_rates,
    )

    # Both go straight into `SatInfo`'s unit-typed fields, so the units are part of the
    # contract: metres and metres per second.
    @test eltype(residuals) == typeof(1.0u"m")
    @test eltype(rate_residuals) == typeof(1.0u"m/s")

    # The steered member's residuals are its discriminators alone — chips × chip
    # length and Hz × wavelength — which is what makes them the vector loop's
    # per-satellite tracking-error readout.
    @test ustrip(u"m", residuals[1]) ≈ 0.02 * steered.chip_length
    @test ustrip(u"m/s", rate_residuals[1]) ≈ -5.0 * steered.wavelength
    # The other member keeps the full disagreement. Both are observed − computed in
    # `calc_pvt`'s conventions, and its rate observable (the geometric range rate) runs
    # opposite to this loop's Doppler-signed one, so a measurement beyond the prediction
    # reads positive in the range domain and negative in the rate domain. Pins the
    # pairing that keeps a vector-tracking `SatInfo` sign-comparable with a scalar one.
    @test ustrip(u"m", residuals[2]) ≈ 30.0
    @test ustrip(u"m/s", rate_residuals[2]) ≈ -2.0

    # A cycle with no members at all still yields the two correctly typed (empty)
    # vectors rather than untyped ones.
    empty_residuals, empty_rate_residuals = GNSSReceiver.vt_post_fit_residuals(
        GNSSReceiver.VTMember[],
        Float64[],
        Float64[],
        Float64[],
    )
    @test isempty(empty_residuals) && eltype(empty_residuals) == typeof(1.0u"m")
    @test isempty(empty_rate_residuals) &&
          eltype(empty_rate_residuals) == typeof(1.0u"m/s")

    # `vt_member_sats` turns those into the reported `SatInfo`s. Selecting all members is
    # the filter's own report (`VectorTrackingState.member_sats`); selecting the update's
    # `included_indices` is the solution's `pvt.sats`, so a coasted member appears in the
    # first and not in the second — that difference is what tells a consumer which
    # satellites were measured rather than predicted.
    all_members = GNSSReceiver.vt_member_sats(members, residuals, rate_residuals)
    measured_only = GNSSReceiver.vt_member_sats(members, residuals, rate_residuals, [1])
    @test issetequal(keys(all_members), [(:GPSL1CA, 4), (:GPSL1CA, 9)])
    @test issetequal(keys(measured_only), [(:GPSL1CA, 4)])
    # A member's entry is identical in both — same constructor, same residuals.
    @test all_members[(:GPSL1CA, 4)] == measured_only[(:GPSL1CA, 4)]
    @test all_members[(:GPSL1CA, 9)].residual == residuals[2]
    @test all_members[(:GPSL1CA, 9)].rate_residual == rate_residuals[2]
    @test all_members[(:GPSL1CA, 4)].position == steered.sat_position
    # An update that measured nothing (everything coasted) reports no satellites at all,
    # still correctly typed: that is the empty `pvt.sats` of a purely predicted solution.
    none = GNSSReceiver.vt_member_sats(members, residuals, rate_residuals, Int[])
    @test isempty(none) && none isa Dictionary{Tuple{Symbol,Int},SatInfo}
end

@testset "FLL discriminator sign into the rate measurement" begin
    # `fll_disc` measures f_incoming − f_replica, so a positive mean residual must
    # ADD to the pseudorange-rate measurement (`λ · carrier_doppler + λ · mean_fll`);
    # the discriminator enters with its own sign, not flipped. Pins the sign so a
    # re-introduced negation is caught.
    sat = GNSSReceiver.create_tracked_sat(
        GNSSReceiver.tracking_signals(GPSL1CA()),
        1,
        0.0,
        20.0u"Hz",
        NumAnts(1),
        VectorPLLAndDLL(),
    )
    base = Tracking.get_doppler_estimator_state(sat)
    # Mean = sum / count: (2, 6 Hz) → +3 Hz, (1, −4 Hz) → −4 Hz.
    pos = SatVectorPLLAndDLL(base; carrier_discr_acc = (2, 6.0u"Hz"))
    neg = SatVectorPLLAndDLL(base; carrier_discr_acc = (1, -4.0u"Hz"))
    empty_acc = SatVectorPLLAndDLL(base; carrier_discr_acc = (0, 0.0u"Hz"))
    @test GNSSReceiver.accumulated_carrier_discriminator(pos) ≈ 3.0
    @test GNSSReceiver.accumulated_carrier_discriminator(neg) ≈ -4.0
    @test GNSSReceiver.accumulated_carrier_discriminator(empty_acc) == 0.0
end

@testset "A cycle without an accumulated discriminator withholds the member" begin
    # The `accumulated_*` helpers substitute a zero when nothing was accumulated, which the
    # navigation filter cannot distinguish from a genuine zero residual measured at full
    # weight. `has_accumulated_discriminators` is the guard that keeps such a member out of
    # the measurement set.
    sat = GNSSReceiver.create_tracked_sat(
        GNSSReceiver.tracking_signals(GPSL1CA()),
        1,
        0.0,
        20.0u"Hz",
        NumAnts(1),
        VectorPLLAndDLL(),
    )
    base = Tracking.get_doppler_estimator_state(sat)
    accumulated = SatVectorPLLAndDLL(
        base;
        code_discr_acc = (3, 0.06),
        carrier_discr_acc = (3, 6.0u"Hz"),
    )
    nothing_accumulated =
        SatVectorPLLAndDLL(base; code_discr_acc = (0, 0.0), carrier_discr_acc = (0, 0.0u"Hz"))
    @test GNSSReceiver.has_accumulated_discriminators(accumulated)
    @test !GNSSReceiver.has_accumulated_discriminators(nothing_accumulated)
    # The zero the helpers would otherwise hand to the filter is indistinguishable from a
    # measured zero — which is exactly why the guard is needed rather than the fallback.
    @test GNSSReceiver.accumulated_carrier_discriminator(nothing_accumulated) == 0.0
    @test GNSSReceiver.accumulated_code_discriminator(nothing_accumulated, 100.0u"ms") == 0.0
end

@testset "Pseudorange differencing across a GNSS week rollover" begin
    c = SPEED_OF_LIGHT
    # Receive just after the week rollover (TOW 0.05 s), transmit just before it
    # (TOW 604799.97 s): the 0.08 s light-travel difference straddles 604800 s and
    # must fold back rather than read as a ~604800 s (≈1.8e11 m) pseudorange.
    @test GNSSReceiver.pseudorange_from_tows(0.05, 604799.97) ≈ 0.08 * c rtol = 1e-6
    # Mid-week (no wrap) is unaffected.
    @test GNSSReceiver.pseudorange_from_tows(100.08, 100.0) ≈ 0.08 * c rtol = 1e-6
end

@testset "Mixed sampling/IF units normalise to Hz for the tracking pass" begin
    # `process` builds `Tracking.BandMeasurement(m, sampling_freq, interm_freq)`. When the two
    # carry different units — a MHz sampling frequency and an Hz IF, as in a real front end —
    # Unitful's `promote` collapses both to the SI base `s^-1`, and vector tracking's
    # `Hz`-typed discriminator accumulator then rejects the value at the first loop closure
    # (scalar tracking has no such field and silently tolerates it). `process` guards this by
    # normalising both to `Hz`; document the trap and the fix here.
    m = zeros(Complex{Int16}, 8, 1)
    raw = Tracking.BandMeasurement(m, 2.048u"MHz", 0.0u"Hz")
    @test Unitful.unit(Tracking.get_sampling_frequency(raw)) == u"s^-1"        # the trap
    fixed = Tracking.BandMeasurement(m, uconvert(u"Hz", 2.048u"MHz"), uconvert(u"Hz", 0.0u"Hz"))
    @test Unitful.unit(Tracking.get_sampling_frequency(fixed)) == u"Hz"        # the normalisation
end

@testset "Doppler estimator selection and receiver-state wiring" begin
    # The tracking-loop estimator follows from the mode alone.
    @test GNSSReceiver.doppler_estimator_for(false) isa ConventionalPLLAndDLL
    @test GNSSReceiver.doppler_estimator_for(true) isa VectorPLLAndDLL

    # The vector estimator's scalar fallback is FLL-assisted, so the acquisition
    # pull-in range matches the conventional assisted estimator's.
    @test GNSSReceiver.carrier_doppler_pull_in_range(VectorPLLAndDLL(), GPSL1CA()) ==
          GNSSReceiver.carrier_doppler_pull_in_range(
        ConventionalAssistedPLLAndDLL(),
        GPSL1CA(),
    )

    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    @test receiver_state.vt isa GNSSReceiver.VectorTrackingState
    @test !receiver_state.vt.running
    @test receiver_state.track_state.doppler_estimator isa VectorPLLAndDLL

    scalar_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 20000,
        vector_tracking = false,
    )
    @test isnothing(scalar_state.vt)

    # A `VectorTracking` in place of `true` both enables the filter and configures it, so a
    # known platform and front end can be described without editing the defaults.
    configured_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 20000,
        vector_tracking = VectorTracking(;
            h0 = 1.3e-22,
            hm2 = 2e-22,
            acceleration_noise_std = 1.0u"m/s^2",
            insufficient_meas_timeout = 25.0u"s",
        ),
    )
    @test configured_state.vt isa GNSSReceiver.VectorTrackingState
    @test configured_state.vt.config.h0 == 1.3e-22
    @test configured_state.vt.config.hm2 == 2e-22
    @test configured_state.vt.config.insufficient_meas_timeout == 25.0u"s"
    # The tracking-loop estimator follows from *whether* vector tracking is on, so passing a
    # configuration must select it exactly as `true` does.
    @test configured_state.track_state.doppler_estimator isa VectorPLLAndDLL
    # Untouched fields keep their defaults.
    @test configured_state.vt.config.use_pseudorange_rates ==
          VectorTracking().use_pseudorange_rates
    @test configured_state.vt.config.ifb_noise_density ==
          VectorTracking().ifb_noise_density

    # Multi-constellation vector tracking: two clock-bias states.
    multi_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        (GPSL1CA(), GalileoE1B());
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    @test multi_state.vt isa GNSSReceiver.VectorTrackingState
    @test multi_state.vt.layout.time_systems == [GPST(), GST()]
    @test length(multi_state.vt.state) == 9
end

@testset "Vector-loop membership on the tracking state" begin
    system = GPSL1CA()
    key = get_signal_id(system)
    prn = 9
    receiver_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        system;
        num_samples_for_acquisition = 20000,
        vector_tracking = true,
    )
    track_state = merge_sats(
        receiver_state.track_state,
        key,
        [GNSSReceiver.create_tracked_sat(
            GNSSReceiver.tracking_signals(system),
            prn,
            0.0,
            20.0u"Hz",
            NumAnts(1),
            receiver_state.track_state.doppler_estimator,
        )],
    )
    receiver_sat_states = (; key => Dictionary([prn], [GNSSReceiver.ReceiverSatState(system, prn)]))

    # Fresh from acquisition: scalar fallback, not in the vector loop.
    sat = get_sat_state(track_state, key, prn)
    @test !GNSSReceiver.in_vt_loop(sat)

    # Promote (group-scoped) and sync the receiver-side flag.
    Tracking.enable_vt!(track_state, key, [prn])
    @test GNSSReceiver.in_vt_loop(get_sat_state(track_state, key, prn))
    synced = GNSSReceiver.sync_vt_flags(receiver_sat_states, track_state)
    @test synced[key][prn].in_vt_loop

    # A member out of code lock is neither reacquired nor removed.
    out_of_lock_member = GNSSReceiver.ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        out_of_lock_code_detector(),
        GNSSReceiver.CarrierLockDetector(),
        0.0u"s",
        1.0u"s",
        0,
        true,
    )
    @test !GNSSReceiver.is_in_lock(out_of_lock_member)
    @test !GNSSReceiver.should_reacquire(out_of_lock_member)
    member_states = (; key => Dictionary([prn], [out_of_lock_member]))
    @test length(get_sat_states(
        GNSSReceiver.remove_lost_satellites(member_states, track_state),
    )) == 1

    # While in the vector loop, lock follows the code detector alone.
    code_locked_member = GNSSReceiver.ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        GNSSReceiver.CodeLockDetector(),
        GNSSReceiver.set_out_of_lock(GNSSReceiver.CarrierLockDetector()),
        0.0u"s",
        0.0u"s",
        0,
        true,
    )
    @test GNSSReceiver.is_in_lock(code_locked_member)
    @test !GNSSReceiver.is_in_lock(GNSSReceiver.ReceiverSatState(
        code_locked_member.prn,
        code_locked_member.decoder,
        code_locked_member.code_lock_detector,
        code_locked_member.carrier_lock_detector,
        code_locked_member.time_in_lock,
        code_locked_member.time_out_of_lock,
        0,
        false,
    ))

    # Releasing hands the satellite back to the scalar fallback with reset
    # loop filters and NCO corrections.
    Tracking.set_code_freq_updates!(track_state, key, Dictionary([prn], [10.0u"Hz"]))
    GNSSReceiver.release_from_vector_tracking!(track_state, key, [prn])
    released = get_sat_state(track_state, key, prn)
    @test !GNSSReceiver.in_vt_loop(released)
    estimator_state = Tracking.get_doppler_estimator_state(released)
    @test estimator_state.code_freq_update == 0.0u"Hz"

    # Forcing out of lock trips both detectors so the satellite is removed and
    # reacquired through the normal path.
    forced = GNSSReceiver.force_out_of_lock(synced, key, [prn])
    @test !GNSSReceiver.is_in_lock(forced[key][prn].code_lock_detector)
    @test !GNSSReceiver.is_in_lock(forced[key][prn].carrier_lock_detector)
end

@testset "set_out_of_lock lock detectors" begin
    code_detector = GNSSReceiver.CodeLockDetector()
    @test GNSSReceiver.is_in_lock(code_detector)
    @test !GNSSReceiver.is_in_lock(GNSSReceiver.set_out_of_lock(code_detector))
    carrier_detector = GNSSReceiver.CarrierLockDetector()
    @test GNSSReceiver.is_in_lock(carrier_detector)
    @test !GNSSReceiver.is_in_lock(GNSSReceiver.set_out_of_lock(carrier_detector))
end
