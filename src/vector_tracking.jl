# ─────────────────────────────────────────────────────────────────────────────
# Vector tracking (VDLL / VDFLL)
#
# In scalar tracking every satellite closes its own code/carrier loops from its
# own discriminators. In vector tracking a central navigation Kalman filter
# closes all loops at once: each satellite's accumulated DLL (and FLL)
# discriminator outputs become pseudorange (and pseudorange-rate) measurements,
# the filter fuses them into a position/velocity/clock state, and the predicted
# line-of-sight dynamics are fed back as per-satellite NCO corrections — a
# vector delay lock loop (VDLL), or a vector delay/frequency lock loop (VDFLL)
# when the pseudorange rates are measured too (`use_pseudorange_rates`). Weak
# or briefly obscured satellites are carried through outages by the collective
# solution instead of losing lock on their own.
#
# The receiver may track several constellations and frequency bands at once;
# the filter follows `PositionVelocityTime`'s bias model: one receiver clock
# bias per GNSS time system (all driven by the one oscillator's clock drift)
# and one receiver inter-frequency bias per band beyond a reference band. The
# measured pseudoranges are corrected for the broadcast ionospheric model and
# the Saastamoinen tropospheric delay, as in the scalar PVT solve. Which of
# those biases a given epoch can actually determine is decided per cycle, the
# way `decide_bias_layout` decides it for the scalar solve — including the
# broadcast-offset collapse of a Galileo or BeiDou clock onto the GPS one (the
# GGTO / BGTO) when the measurements do not support both (see
# `assess_bias_observability`).
#
# The loop split with `Tracking`: the `VectorPLLAndDLL` Doppler estimator
# accumulates each satellite's discriminators and applies the NCO corrections
# (the group-scoped `set_code_freq_updates!` / `set_carrier_freq_updates!`);
# everything above — the navigation filter, measurement construction,
# satellite selection and the loop-closure math — lives here. A satellite
# fresh from acquisition runs `VectorPLLAndDLL`'s scalar fallback loop until
# it is locked, healthy and decoded, then `enable_vt!` puts it into the vector
# loop (and `disable_vt!` hands it back on release).

"""
    VectorTracking(; use_pseudorange_rates = true,
                   motion_model_order = 2, clock_model_order = 2,
                   acceleration_noise_std = 5.0m/s^2,
                   h0 = 2e-19, hm2 = 2e-20,
                   ifb_noise_density = 0.01m/sqrt(1.0s),
                   insufficient_meas_timeout = 10.0s)

Configuration of the vector-tracking navigation filter. Pass an instance as the
`vector_tracking` keyword of [`receive`](@ref) / [`ReceiverState`](@ref) to
describe the platform and the receiver's oscillator; `vector_tracking = true`
uses these defaults, which suit a vehicle carrying a consumer-grade front end.

# Fields
- `use_pseudorange_rates`: with `true` (VDFLL) the navigation filter measures
  both the pseudoranges (DLL discriminators) and the pseudorange rates (FLL
  discriminators on the carrier Doppler); with `false` (VDLL) only the
  pseudoranges enter the filter — the carrier NCO corrections are then derived
  purely from the filter's velocity/clock-drift prediction.
- `motion_model_order`: order of the per-axis motion model — `1` position only,
  `2` position + velocity, `3` position + velocity + acceleration.
- `clock_model_order`: order of the receiver clock model — `1` biases only,
  `2` biases + a common drift. One clock bias is modelled per GNSS time
  system, all integrating the single oscillator drift.
- `acceleration_noise_std`: white-acceleration process-noise standard deviation
  driving the motion model — how hard the platform manoeuvres. The `5 m/s²`
  default suits automotive / vehicular dynamics (≈0.5 g of manoeuvring); use
  ~`1 m/s²` for pedestrian or ship dynamics and tens of `m/s²` for aircraft or
  launch vehicles. It means the same thing at every `motion_model_order`: the
  process noise always models the platform's first *unmodelled* derivative, so
  the orders that do not model the acceleration (1 and 3) derive their velocity
  and jerk figures from this one through the manoeuvre time constant
  `MANOEUVRE_TIME` — see `motion_noise_model`, which is also where to change that
  constant for a platform whose manoeuvres are much shorter or longer than the
  couple of seconds a road vehicle takes.
- `h0`, `hm2`: Allan-variance coefficients ``h_0`` (seconds) and ``h_{-2}``
  (1/seconds) of the receiver oscillator, sizing the clock process noise —
  smaller coefficients for a more stable oscillator. They follow from the
  oscillator's Allan deviation: ``σ_y²(τ) = h_0 / 2τ`` at short averaging times
  gives ``h_0 = 2τσ_y²(τ)``, and ``σ_y²(τ) = (2π²/3)·h_{-2}·τ`` at long ones
  gives ``h_{-2} = 3σ_y²(τ)/(2π²τ)``. The defaults model a TCXO-grade clock (a
  typical consumer/automotive receiver oscillator); an OCXO is orders of magnitude
  tighter, a bare crystal looser.
- `ifb_noise_density`: random-walk process-noise density of the inter-frequency
  biases, in `m/√s`. Per-band RF-chain delays are nearly constant, so this only
  has to cover thermal drift of the front end: the default lets a bias wander
  about `0.01 m` in a second of elapsed time. Raise it for a front end whose
  bands are less thermally coupled, lower it for one that is stable or
  externally calibrated.
- `insufficient_meas_timeout`: how long the navigation filter may coast on
  epochs it cannot solve before vector tracking is abandoned and the receiver
  falls back to scalar tracking. An epoch counts as unsolvable when it has fewer
  measurements than the bias layout in force has unknowns. How certain the filter
  is of the solution it did produce is reported rather than policed — see
  [`VTStatus`](@ref)'s `position_std`.
"""
struct VectorTracking
    use_pseudorange_rates::Bool
    motion_model_order::Int
    clock_model_order::Int
    acceleration_noise_std::typeof(1.0m/s^2)
    h0::Float64
    hm2::Float64
    ifb_noise_density::typeof(1.0m/sqrt(1.0s))
    insufficient_meas_timeout::typeof(1.0s)
end

function VectorTracking(;
    use_pseudorange_rates = true,
    motion_model_order = 2,
    clock_model_order = 2,
    acceleration_noise_std = 5.0m/s^2,
    h0 = 2e-19,
    hm2 = 2e-20,
    ifb_noise_density = 0.01m/sqrt(1.0s),
    insufficient_meas_timeout = 10.0s,
)
    1 <= motion_model_order <= 3 ||
        throw(ArgumentError("motion_model_order must be 1, 2 or 3"))
    1 <= clock_model_order <= 2 ||
        throw(ArgumentError("clock_model_order must be 1 or 2"))
    VectorTracking(
        use_pseudorange_rates,
        motion_model_order,
        clock_model_order,
        acceleration_noise_std,
        h0,
        hm2,
        ifb_noise_density,
        insufficient_meas_timeout,
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# Navigation filter model

# Which clock-bias and inter-frequency-bias state each tracking group maps to.
# Established once at receiver construction from the *configured* systems (not
# from the currently tracked satellites), so the filter's state dimension is
# fixed for the whole run; a constellation without current measurements simply
# coasts on its process noise. The inter-frequency-bias columns follow
# `PositionVelocityTime.band_ifb_layout`, which creates a column only where the
# bias is observable (a band stranded alone on its constellation folds its
# delay into that constellation's clock instead).
struct NavFilterLayout
    time_systems::Vector{GNSSSignals.TimeSystem} # clock-bias state order
    extra_bands::Vector{Symbol}                  # inter-frequency-bias state order
    reference_bands::Vector{Symbol}              # anchor band per IFB state
    clock_bias_index_by_group::Dict{Symbol,Int}       # group key ⇒ clock-bias state
    ifb_index_by_group::Dict{Symbol,Int}         # group key ⇒ IFB state (0 = reference band)
    band_by_group::Dict{Symbol,Symbol}           # group key ⇒ frequency band
end

function NavFilterLayout(systems)
    systems_vector = collect(systems)
    time_system_per_group =
        GNSSSignals.TimeSystem[get_time_system(data_signal(s)) for s in systems_vector]
    band_per_group = Symbol[get_band_id(system_band(s)) for s in systems_vector]
    time_systems = unique(time_system_per_group)
    ifb_indices, extra_bands, reference_bands, _ =
        PositionVelocityTime.band_ifb_layout(time_system_per_group, band_per_group)
    clock_bias_index_by_group = Dict(
        signal_group_key(s) => findfirst(==(ts), time_systems) for
        (s, ts) in zip(systems_vector, time_system_per_group)
    )
    ifb_index_by_group = Dict(
        signal_group_key(s) => ifb_indices[i] for (i, s) in enumerate(systems_vector)
    )
    band_by_group = Dict(
        signal_group_key(s) => band_per_group[i] for (i, s) in enumerate(systems_vector)
    )
    NavFilterLayout(
        time_systems,
        collect(Symbol, extra_bands),
        collect(Symbol, reference_bands),
        clock_bias_index_by_group,
        ifb_index_by_group,
        band_by_group,
    )
end

num_clock_biases(layout::NavFilterLayout) = length(layout.time_systems)
num_ifb(layout::NavFilterLayout) = length(layout.extra_bands)

# Indices of the navigation-filter state vector
# `[x, ẋ, (ẍ), y, ẏ, (ÿ), z, ż, (z̈), clock_bias_1..M, (clock_drift), ifb_1..B]`
# — positions and clock/inter-frequency biases in metres, derivatives in m/s
# (m/s²), so the clock terms are directly commensurable with the pseudorange
# measurements.
struct NavFilterIndices
    pos::Vector{Int}
    vel::Vector{Int}          # empty for motion_model_order == 1
    acc::Vector{Int}          # empty for motion_model_order <= 2
    clock_biases::Vector{Int} # one per GNSS time system
    clock_drift::Int          # 0 for clock_model_order == 1; shared by all biases
    ifb::Vector{Int}          # one per frequency band beyond its reference
end

function NavFilterIndices(
    motion_model_order::Int,
    clock_model_order::Int,
    num_clock_biases::Int,
    num_ifb::Int,
)
    pos = [1 + (i - 1) * motion_model_order for i = 1:3]
    vel = motion_model_order >= 2 ? pos .+ 1 : Int[]
    acc = motion_model_order >= 3 ? pos .+ 2 : Int[]
    clock_biases = 3 * motion_model_order .+ (1:num_clock_biases)
    clock_drift = clock_model_order >= 2 ? clock_biases[end] + 1 : 0
    ifb_offset = max(clock_drift, clock_biases[end])
    ifb = ifb_offset .+ (1:num_ifb)
    NavFilterIndices(pos, vel, acc, collect(clock_biases), clock_drift, collect(ifb))
end

num_nav_states(config::VectorTracking, layout::NavFilterLayout) =
    3 * config.motion_model_order +
    num_clock_biases(layout) +
    (config.clock_model_order >= 2 ? 1 : 0) +
    num_ifb(layout)

# Read (position, velocity, common clock drift) out of a state vector,
# substituting zeros for unmodelled derivatives. The per-system clock biases
# and per-band IFBs are indexed directly via `idxs.clock_biases` / `idxs.ifb`.
function nav_filter_states(x, idxs::NavFilterIndices)
    user_pos = SVector{3,Float64}(x[idxs.pos])
    user_vel =
        isempty(idxs.vel) ? zero(SVector{3,Float64}) : SVector{3,Float64}(x[idxs.vel])
    user_clock_drift = idxs.clock_drift == 0 ? 0.0 : x[idxs.clock_drift]
    user_pos, user_vel, user_clock_drift
end

# The `[x, y, z, tc₁..tc_M, ifb₁..ifb_B]` sub-vector PositionVelocityTime's
# measurement helpers (`calc_ρ_hat!`, `calc_H`) expect.
position_and_bias_vector(x, idxs::NavFilterIndices) =
    vcat(x[idxs.pos], x[idxs.clock_biases], x[idxs.ifb])

# Discrete-time process model F: a constant-velocity (or -position /
# -acceleration) block per axis; every clock bias integrates the single
# oscillator drift; the inter-frequency biases are constant.
function nav_filter_process_model(
    config::VectorTracking,
    layout::NavFilterLayout,
    integration_time,
)
    T = ustrip(s, integration_time)
    m_ord = config.motion_model_order
    idxs = NavFilterIndices(
        m_ord,
        config.clock_model_order,
        num_clock_biases(layout),
        num_ifb(layout),
    )
    n = num_nav_states(config, layout)
    F = Matrix{Float64}(I, n, n)
    F_axis = [1 T T^2/2; 0 1 T; 0 0 1][1:m_ord, 1:m_ord]
    for d = 0:2
        block = (d * m_ord + 1):((d + 1) * m_ord)
        F[block, block] = F_axis
    end
    if idxs.clock_drift != 0
        for bias in idxs.clock_biases
            F[bias, idxs.clock_drift] = T
        end
    end
    F
end

# The per-axis process-noise gain vector `Γ` of one motion model order, and the standard
# deviation of the scalar per-interval noise driving it: the axis block of `Q` is `Γ Γᵀ σ²`,
# one unmodelled derivative acting over the interval and propagated into every modelled
# state (the discrete white-noise model of the order above the highest state — Bar-Shalom,
# Li & Kirubarajan, "Estimation with Applications to Tracking and Navigation", Wiley 2001,
# §6.3.2).
#
# Which derivative is unmodelled is what the order changes, and with it what `σ` physically
# is:
#
#   order 1 (p)     → the platform's *velocity* is unmodelled: Γ = [T],             σ = σ_v
#   order 2 (p,v)   → its *acceleration* is:                   Γ = [T²/2, T],       σ = σ_a
#   order 3 (p,v,a) → its *jerk* is:                           Γ = [T³/6, T²/2, T], σ = σ_j
#
# The configuration carries one dynamics figure, `acceleration_noise_std`, so the other two
# are derived from it through the manoeuvre time constant `MANOEUVRE_TIME` (τ) below. That
# keeps `acceleration_noise_std` meaning the same thing ("how hard the platform manoeuvres")
# at every order — and, unlike a fixed rescaling factor, it stays right when the filter
# interval changes, because each order's `Γ` already carries the `T` dependence its own
# derivative implies. A constant factor is only correct at one `T`: matching a fixed
# velocity or jerk with the order-2 gain vector needs a factor going as `1/T`.
#
# How long one manoeuvre lasts. A manoeuvre of this duration changes the velocity by `σ_a·τ`
# — what order 1, modelling no velocity, has to absorb — and is built out of a jerk of
# `σ_a/τ`, what order 3, modelling the acceleration, has to absorb. Two seconds is a road
# vehicle's: a lane change, or a 0.5 g stop from 36 km/h, which is `acceleration_noise_std`'s
# own default read as a manoeuvre. Deliberately a constant and not a keyword: it is inert at
# the default `motion_model_order = 2`, where the acceleration itself is the unmodelled
# derivative, so as a keyword it would sit on everyone's constructor to serve only the two
# rarely-chosen orders. A platform whose manoeuvres are much shorter or longer changes it
# here.
const MANOEUVRE_TIME = 2.0s

function motion_noise_model(config::VectorTracking, T)
    acc_std = ustrip(m/s^2, config.acceleration_noise_std)
    τ = ustrip(s, MANOEUVRE_TIME)
    config.motion_model_order == 1 && return [T], acc_std * τ
    config.motion_model_order == 2 && return [T^2/2, T], acc_std
    [T^3/6, T^2/2, T], acc_std / τ
end

# Process noise Q: the motion noise of `motion_noise_model` above, a clock model
# from the oscillator's Allan-variance coefficients, and a small random walk on
# the inter-frequency biases. All clock biases ride the *same* oscillator, so the
# drift random walk (Sg) is fully correlated across them; only the white
# frequency noise (Sf) is applied per bias.
function nav_filter_process_noise_covariance(
    config::VectorTracking,
    layout::NavFilterLayout,
    integration_time,
)
    T = ustrip(s, integration_time)
    m_ord = config.motion_model_order
    idxs = NavFilterIndices(
        m_ord,
        config.clock_model_order,
        num_clock_biases(layout),
        num_ifb(layout),
    )
    c = PositionVelocityTime.SPEEDOFLIGHT

    n = num_nav_states(config, layout)
    Q = zeros(n, n)

    Γ, driving_std = motion_noise_model(config, T)
    Q_axis = (Γ * Γ') .* driving_std^2
    for d = 0:2
        block = (d * m_ord + 1):((d + 1) * m_ord)
        Q[block, block] = Q_axis
    end

    # Allan variance to clock process noise (biases in m, drift in m/s):
    # Sf [m²/s] from white frequency noise h0, Sg [m²/s³] from the frequency
    # random walk h-2.
    Sf = c^2 * config.h0 / 2
    Sg = c^2 * 2 * π^2 * config.hm2
    for bias_i in idxs.clock_biases, bias_j in idxs.clock_biases
        Q[bias_i, bias_j] = Sg * T^3 / 3 + (bias_i == bias_j ? Sf * T : 0.0)
    end
    if idxs.clock_drift != 0
        for bias in idxs.clock_biases
            Q[bias, idxs.clock_drift] = Sg * T^2 / 2
            Q[idxs.clock_drift, bias] = Sg * T^2 / 2
        end
        Q[idxs.clock_drift, idxs.clock_drift] = Sg * T
    end

    # The inter-frequency biases are near-constant RF-chain delays; their random walk only
    # has to cover the front end's thermal drift (`ifb_noise_density`, in m/√s).
    ifb_density = ustrip(m/sqrt(s), config.ifb_noise_density)
    for ifb in idxs.ifb
        Q[ifb, ifb] = ifb_density^2 * T
    end
    Q
end

# The navigation filter's linear process model for one integration interval.
struct NavFilterModel
    integration_time::typeof(1.0s)
    F::Matrix{Float64}
    Q::Matrix{Float64}
    idxs::NavFilterIndices
end

function NavFilterModel(config::VectorTracking, layout::NavFilterLayout, integration_time)
    NavFilterModel(
        integration_time,
        nav_filter_process_model(config, layout, integration_time),
        nav_filter_process_noise_covariance(config, layout, integration_time),
        NavFilterIndices(
            config.motion_model_order,
            config.clock_model_order,
            num_clock_biases(layout),
            num_ifb(layout),
        ),
    )
end

"""
    VectorTrackingState

Runtime state of the vector-tracking loop, carried on the [`ReceiverState`](@ref):
the [`VectorTracking`](@ref) configuration, the clock/inter-frequency-bias
layout derived from the configured systems, the navigation filter's process
model and Kalman state/covariance, whether the vector loop is `running` (it
starts running with the first scalar PVT fix and falls back on prolonged
measurement starvation), which clock-bias state is the reporting reference
(`primary_clock_index`, seeded from the fix's reference system and maintained by
`report_primary_clock_index`), the epoch `reference_time` the pseudoranges are
referenced to — always on the *GPS Time count* (see `VTMember.time_gpst_count`),
so it does not move when the primary clock changes — how long the filter has been
coasting on epochs it could
not solve (see `run_vt_iteration`), and the per-member report of the latest
update (`member_sats`).
"""
struct VectorTrackingState
    config::VectorTracking
    layout::NavFilterLayout
    nav_filter::NavFilterModel
    state::Vector{Float64}
    covariance::Matrix{Float64}
    running::Bool
    primary_clock_index::Int
    reference_time::typeof(1.0s)
    time_with_insufficient_meas::typeof(1.0s)
    # Every member of the loop as of the latest update — measured *and* coasted — keyed
    # exactly as `pvt.sats` is, each with the satellite position, transmit time and post-fit
    # residuals that update produced. `pvt.sats` carries only the members the update
    # measured (the ones that determined the solution, matching what `calc_pvt` reports for
    # a scalar solve), so the difference between the two key sets is what the filter
    # predicted through an obscuration; their residuals are the divergence indicator.
    # Reporting only: nothing in the filter reads this back. Emptied while the scalar solve
    # is in control, so it never outlives the update it describes.
    member_sats::Dictionary{Tuple{Symbol,Int},SatInfo}
    # Constant part of the reported epoch (seconds), cached once resolved — see
    # `resolve_time_epoch_offset`. `nothing` until a decoded primary-system satellite has
    # been seen; from then on the solution always carries a timestamp.
    time_epoch_offset::Union{Nothing,Int}
end

function VectorTrackingState(
    config::VectorTracking,
    layout::NavFilterLayout;
    integration_time = 100.0ms,
)
    nav_filter = NavFilterModel(config, layout, integration_time)
    n = num_nav_states(config, layout)
    VectorTrackingState(
        config,
        layout,
        nav_filter,
        zeros(n),
        zeros(n, n),
        false,
        1,
        0.0s,
        0.0s,
        empty_member_sats(),
        nothing,
    )
end

# The empty per-member report, for a filter that has not updated yet (or has handed the
# loops back). One helper so the (long) element type is written once.
empty_member_sats() =
    Dictionary{Tuple{Symbol,Int},SatInfo}()

# Kwarg-update constructor.
function VectorTrackingState(
    vt::VectorTrackingState;
    nav_filter::Union{Nothing,NavFilterModel} = nothing,
    state = nothing,
    covariance = nothing,
    running = nothing,
    primary_clock_index = nothing,
    reference_time = nothing,
    time_with_insufficient_meas = nothing,
    member_sats = nothing,
    # `nothing` doubles as this one's "unknown" value, which costs nothing: the only caller
    # passes back what `rolled_over_time_epoch_offset` returned, and that is `nothing`
    # exactly when the cached value was `nothing` too.
    time_epoch_offset = nothing,
)
    VectorTrackingState(
        vt.config,
        vt.layout,
        isnothing(nav_filter) ? vt.nav_filter : nav_filter,
        isnothing(state) ? vt.state : state,
        isnothing(covariance) ? vt.covariance : covariance,
        isnothing(running) ? vt.running : running,
        isnothing(primary_clock_index) ? vt.primary_clock_index : primary_clock_index,
        isnothing(reference_time) ? vt.reference_time : reference_time,
        isnothing(time_with_insufficient_meas) ? vt.time_with_insufficient_meas :
        time_with_insufficient_meas,
        isnothing(member_sats) ? vt.member_sats : member_sats,
        isnothing(time_epoch_offset) ? vt.time_epoch_offset : time_epoch_offset,
    )
end

# The PVT cadence is the navigation filter's update interval. It is nominally
# `pvt_update_interval` but measured each cycle; rebuild the process model only
# when it drifts by more than 5 %. The normal deviation is the chunk-size
# quantisation — a cycle runs on the first chunk boundary at or past
# `pvt_update_interval`, so the interval overshoots by up to one chunk length
# (e.g. 4 ms on 100 ms ≈ 4 %); the 5 % band absorbs that without rebuilding
# `F`/`Q` every cycle, while a genuine cadence change (a different
# `pvt_update_interval`) still triggers a rebuild.
function ensure_nav_filter_integration_time(vt::VectorTrackingState, integration_time)
    nav_integration_time = vt.nav_filter.integration_time
    abs(integration_time - nav_integration_time) / nav_integration_time <= 0.05 &&
        return vt
    VectorTrackingState(
        vt;
        nav_filter = NavFilterModel(vt.config, vt.layout, integration_time),
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# Tracking-facing helpers

# Whether a tracked satellite's loops are closed by the navigation filter.
# Generic fallback for non-vector estimator states.
_vt_on(_estimator_state) = false
_vt_on(estimator_state::SatVectorPLLAndDLL) = estimator_state.vt_on

in_vt_loop(sat::Tracking.TrackedSat) = _vt_on(get_doppler_estimator_state(sat))

# Refresh each `ReceiverSatState`'s `in_vt_loop` flag from the tracking state.
# The flag is cached on the receiver side so lock handling and reacquisition
# (which run without the estimator states at hand) can consult it.
function sync_vt_flags(receiver_sat_states, track_state)
    new_vals = map(keys(receiver_sat_states)) do group_key
        tracked_sats = get_sat_states(track_state, group_key)
        map(receiver_sat_states[group_key]) do state
            flag = haskey(tracked_sats, state.prn) && in_vt_loop(tracked_sats[state.prn])
            @set state.in_vt_loop = flag
        end
    end
    NamedTuple{keys(receiver_sat_states)}(new_vals)
end

# Force the lock detectors of the given PRNs (one group) out of lock. Used on
# satellites released from the vector loop for cause (diverged innovation,
# below the horizon, unhealthy): out of lock and out of the vector loop, they
# are removed from tracking on the next chunk and recovered through normal
# reacquisition.
function force_out_of_lock(receiver_sat_states, group_key, prns)
    isempty(prns) && return receiver_sat_states
    group = map(receiver_sat_states[group_key]) do state
        state.prn in prns ?
        ReceiverSatState(
            state.prn,
            state.decoder,
            set_out_of_lock(state.code_lock_detector),
            set_out_of_lock(state.carrier_lock_detector),
            state.time_in_lock,
            state.time_out_of_lock,
            state.num_unsuccessful_reacquisition,
            state.in_vt_loop,
        ) : state
    end
    merge(receiver_sat_states, NamedTuple{(group_key,)}((group,)))
end

# Hand one group's satellites back to the scalar fallback loop: clear the
# vector-loop membership and reset the loop filters — `reset_loop_filters!`
# re-seeds the scalar loop from the current (vector-controlled) Dopplers and
# zeroes the stale NCO corrections and discriminator accumulators, so the
# handoff is transient-free.
function release_from_vector_tracking!(track_state, group_key, prns)
    isempty(prns) && return track_state
    disable_vt!(track_state, group_key, prns)
    tracked_prns = keys(get_sat_states(track_state, group_key))
    for prn in prns
        prn in tracked_prns && reset_loop_filters!(track_state, group_key, prn)
    end
    track_state
end

# ─────────────────────────────────────────────────────────────────────────────
# Measurement gathering

# Everything the navigation filter needs to know about one vector-loop member
# for one cycle. Gathered once per cycle by `collect_vt_members`; all
# per-satellite vectors downstream (measurements, Jacobian rows, noise) are
# aligned with the member vector.
struct VTMember
    group_key::Symbol
    prn::Int
    clock_bias_index::Int       # clock-bias state of this member's time system
    ifb_index::Int         # IFB state of this member's band (0 = reference)
    chip_length::Float64   # m
    wavelength::Float64    # m
    code_frequency::Float64 # Hz
    available::Bool        # usable as a measurement this cycle (in code lock)
    sat_state::SatelliteState # decoder + phase snapshot (atmosphere, transmit time)
    time::Float64          # corrected transmit time (own system's time of week, s)
    # The same instant expressed on the GPS Time count: `time` minus the system's
    # defined scale offset (0 for GPST/GST, −14 s for BDT — a BDT second-of-week reads
    # 14 s below the GPS time of week for the same instant). Everything that
    # *differences* times across members — `reference_time` and the pseudoranges —
    # uses this field, mirroring `calc_pvt`'s `calc_time_scale_offsets`; everything
    # that evaluates a broadcast polynomial (ephemeris, clock, `calc_gpst_offset`)
    # keeps the own-scale `time`.
    time_gpst_count::Float64
    sat_position::SVector{3,Float64}
    sat_velocity::SVector{3,Float64}
    sat_clock_drift::Float64 # s/s
    pseudorange::Float64     # measured, atmosphere-corrected (m)
    pseudorange_rate::Float64 # measured λ·doppler (m/s)
    code_discriminator::Float64    # accumulated DLL output (chips)
    carrier_discriminator::Float64 # accumulated FLL output (Hz, sign-flipped)
    cn0::Float64             # linear carrier-to-noise density (Hz)
    early_late_spacing::Float64 # chips
    coherent_integration_time::Float64 # s, per coherent correlator dump (sets DLL/FLL noise)
end

# Mean DLL discriminator (chips) over the last filter interval, corrected by
# half the NCO code-frequency correction applied over it (the discriminator
# measures the average error while the NCO was already steering it out).
# `mean_code_discr` returns `nothing` when nothing was accumulated.
function accumulated_code_discriminator(estimator_state, integration_time)
    mean = mean_code_discr(estimator_state)
    isnothing(mean) ? 0.0 :
    -mean + ustrip(Hz, estimator_state.code_freq_update) * ustrip(s, integration_time) / 2
end

# Mean FLL discriminator (Hz) over the last filter interval. Tracking's `fll_disc`
# measures f_incoming − f_replica, so the true carrier Doppler is the replica
# Doppler plus this residual; the pseudorange-rate measurement is therefore
# `λ · carrier_doppler + λ · mean_fll` — the discriminator enters with its own
# sign. (The code discriminator carries the opposite sign because `dll_disc`'s
# sense is reversed relative to its own observable.)
#
# Deliberately NOT corrected by half the NCO frequency correction, unlike
# `accumulated_code_discriminator` — the asymmetry is load-bearing, not an oversight.
# The code observable is a *delay* read at the end of the cycle combined with the *mean*
# delay error over it, so the two refer to epochs `T/2` apart and the mean has to be
# advanced to the end. The rate observable has no such gap: `get_carrier_doppler` is the
# replica frequency at the end of the cycle, and a loop lagging a Doppler ramp by `τ` has
# both a final replica low by `Ḋ·τ` and a mean residual high by `Ḋ·τ` — the lag cancels
# exactly, so `carrier_doppler + mean_fll` already lands on the *end-of-cycle* incoming
# Doppler, which is the epoch the navigation filter's predicted state is referenced to.
# Verified: under a 5 m/s² line-of-sight acceleration the sum tracks the end-of-cycle
# Doppler to <0.001 m/s while sitting 0.25 m/s (= a·T/2) away from the mid-cycle value.
# Adding a `T/2` term here would introduce exactly that 0.25 m/s of error.
#
# The cancellation is first-order in the loop's lag, so it is exact only for NCO motion
# that tracks real Doppler motion. NCO motion the incoming signal does not back — the
# navigation filter correcting its own past error — leaves a residue of ≈1% of the applied
# correction, whose sign follows the carrier loop's transient rather than the correction, so
# it does not accumulate across cycles.
function accumulated_carrier_discriminator(estimator_state)
    mean = mean_carrier_discr(estimator_state)
    isnothing(mean) ? 0.0 : ustrip(Hz, mean)
end

# Whether a member accumulated any discriminator at all this cycle. Tracking's
# `mean_code_discr` / `mean_carrier_discr` return `nothing` while their accumulator count is
# zero, which is what a cycle without a single fully integrated correlator dump for this
# satellite looks like (a member admitted at the very end of a cycle, or one whose samples
# were starved). The `accumulated_*` helpers substitute a zero there, and a zero
# discriminator is not a missing measurement to the navigation filter — it is a *confidently
# zero* residual carrying the full measurement weight of `R`, which would pull the state
# towards the current NCO instead of leaving it to coast. Members without an accumulation are
# therefore withheld from the measurement set; they stay in the vector loop and keep getting
# NCO corrections, exactly like a member out of code lock.
#
# Both accumulators are incremented in the same branch of Tracking's fold, so the two counts
# never disagree; the carrier one is checked as well so this stays true if that changes.
has_accumulated_discriminators(estimator_state) =
    !isnothing(mean_code_discr(estimator_state)) &&
    !isnothing(mean_carrier_discr(estimator_state))

# Linear carrier-to-noise density (Hz) floored to 1, from a CN0 estimate in
# dB-Hz. The floor keeps a starved estimator from producing a degenerate
# measurement weight; the `isnan` guard keeps a NaN estimate (an empty
# correlator returns NaN dB-Hz) from putting a NaN into the measurement-noise
# covariance and corrupting the whole Kalman update.
function linear_cn0_floor(cn0_dbhz)
    cn0_linear = 10^(cn0_dbhz / 10)
    isnan(cn0_linear) ? 1.0 : max(cn0_linear, 1.0)
end

# Length of a GNSS week in seconds — the modulus of every time of week in this file.
const SECONDS_PER_WEEK = 7 * 24 * 60 * 60

# Pseudorange (m) from the receive and transmit times-of-week. Both are
# seconds-of-week that wrap at 604800 s, so the small receive − transmit
# light-travel difference is folded modulo the week (`correct_week_crossovers`
# maps a near-±week difference back to near zero) to stay correct when the two
# straddle a GNSS week rollover.
pseudorange_from_tows(receive_tow, transmit_tow) =
    PositionVelocityTime.correct_week_crossovers(receive_tow - transmit_tow) *
    PositionVelocityTime.SPEEDOFLIGHT

# Gather one group's vector-loop members. `prns` must be fully decoded (their
# positions come from their ephemerides). `available_prns` are the members
# currently usable as measurements (in code lock); a member not in it stays in
# the loop and keeps getting NCO corrections, but its discriminators are
# withheld from the navigation filter this cycle. A member that accumulated no
# discriminator at all this cycle is withheld the same way — see
# `has_accumulated_discriminators`.
function collect_vt_members!(
    members::Vector{VTMember},
    track_state,
    system,
    group_key,
    receiver_group_states,
    layout::NavFilterLayout,
    prns,
    available_prns,
    reference_time,
    integration_time,
    sampling_freq,
)
    ranging = ranging_signal(system)
    code_frequency = ustrip(Hz, get_code_frequency(ranging))
    # Seconds this system's time-of-week count reads above the GPS Time count for the
    # same instant (0 for GPST/GST, −14 for BDT). Subtracted from the transmit time to
    # put every member on one count before anything differences them — without it a
    # BeiDou pseudorange is 14 s × c ≈ 4.2×10⁹ m off against a GPS-referenced
    # `reference_time`, the exact error `calc_pvt` removes via
    # `calc_time_scale_offsets`.
    scale_offset = PositionVelocityTime.time_scale_offset_to_gpst(get_time_system(ranging))
    chip_length = PositionVelocityTime.SPEEDOFLIGHT / code_frequency
    wavelength =
        PositionVelocityTime.SPEEDOFLIGHT / ustrip(Hz, get_center_frequency(ranging))
    clock_bias_index = layout.clock_bias_index_by_group[group_key]
    ifb_index = layout.ifb_index_by_group[group_key]
    for prn in prns
        tracked_sat = get_sat_state(track_state, group_key, prn)
        estimator_state = get_doppler_estimator_state(tracked_sat)
        # Actual coherent integration time of the last correlator dump (num_code_blocks ·
        # code period, from Tracking) — one code period per dump at the default integration
        # length (GPS L1 C/A 1 ms, Galileo E1B 4 ms), but this reads whatever Tracking used.
        # It sets the per-dump phase-noise variance and the DLL squaring loss, so both the
        # range and the range-rate measurement noise scale with it.
        #
        # Note this is the *last* dump's length, taken as representative of all `N` dumps in
        # the cycle. Tracking times each record individually
        # (`output.integrated_samples / sampling_frequency`), so a cycle in which the coherent
        # length changes — bit or secondary-code sync promoting 1 ms to 4/20 ms — is modelled
        # with the post-change length for every dump. That also breaks the telescoping the rate
        # measurement relies on (`mean_carrier_discr` divides each dump's phase difference by
        # its *own* `T_coh`, so the interior phases only cancel when every `T_coh` is equal),
        # leaving both the rate measurement's gain and its variance approximate for that one
        # cycle. Accepted: it is a single-cycle transient at sync, long before which the
        # cadence is settled for the rest of the run.
        coherent_integration_time =
            ustrip(s, get_last_fully_integrated_integration_time(tracked_sat, RANGING_SIGNAL_INDEX))
        sat_state =
            SatelliteState(receiver_group_states[prn].decoder, ranging, tracked_sat)
        time = PositionVelocityTime.calc_corrected_time(sat_state)
        time_gpst_count = time - scale_offset
        sat_pv = calc_satellite_position_and_velocity(sat_state.decoder, time)
        pseudorange = pseudorange_from_tows(ustrip(s, reference_time), time_gpst_count)
        pseudorange_rate = wavelength * ustrip(Hz, get_carrier_doppler(tracked_sat))
        # Early-late spacing in chips at the current code rate (measurement
        # noise model input).
        early_late_spacing = upreferred(
            get_early_late_sample_spacing(
                get_last_fully_integrated_correlator(tracked_sat, RANGING_SIGNAL_INDEX),
                sampling_freq,
                get_code_frequency(ranging),
            ) / sampling_freq * (get_code_doppler(tracked_sat) + get_code_frequency(ranging)),
        )
        push!(
            members,
            VTMember(
                group_key,
                prn,
                clock_bias_index,
                ifb_index,
                chip_length,
                wavelength,
                code_frequency,
                prn in available_prns && has_accumulated_discriminators(estimator_state),
                sat_state,
                time,
                time_gpst_count,
                SVector{3,Float64}(PositionVelocityTime.get_sat_position(sat_pv)),
                SVector{3,Float64}(PositionVelocityTime.get_sat_velocity(sat_pv)),
                PositionVelocityTime.calc_satellite_clock_drift(sat_state.decoder, time),
                pseudorange,
                pseudorange_rate,
                accumulated_code_discriminator(estimator_state, integration_time),
                accumulated_carrier_discriminator(estimator_state),
                linear_cn0_floor(ustrip(estimate_cn0(tracked_sat, RANGING_SIGNAL_INDEX))),
                early_late_spacing,
                coherent_integration_time,
            ),
        )
    end
    members
end

# The per-satellite ionospheric + tropospheric delay (m) at the filter's
# position, subtracted from the measured pseudoranges — the same models the
# scalar `calc_pvt` applies (NTCM-G / Klobuchar from the decoded broadcast
# coefficients, blind Saastamoinen with the Niell mapping functions). The
# delays are insensitive to metre-level position error, so one prediction per
# cycle suffices.
function vt_atmospheric_delays(
    members::Vector{VTMember},
    user_pos,
    reference_time,
    enable_ionospheric_correction,
    enable_tropospheric_correction,
    approximate_year,
)
    isempty(members) && return Float64[]
    (enable_ionospheric_correction || enable_tropospheric_correction) ||
        return zeros(length(members))
    sat_states = [member.sat_state for member in members]
    ionospheric_correction =
        enable_ionospheric_correction ?
        PositionVelocityTime.select_ionospheric_correction(sat_states) : nothing
    # The Niell mapping's seasonal term takes the day of year, derived the way `calc_pvt`
    # derives it: from a decoded satellite's absolute week plus the time of week. Any
    # member's decoder dates the epoch — every member has finished decoding (that is what
    # made it a member) and the time systems differ by at most their defined scale offset,
    # 14 s for BDT, against a one-year period — so the first one serves.
    # `vt.time_epoch_offset` would date it too, but it
    # resolves at the END of a cycle, so it is still `nothing` on the first one.
    first_state = first(sat_states)
    week = PositionVelocityTime.get_week(first_state.decoder; approximate_year)
    doy = PositionVelocityTime._day_of_year(
        first_state.system,
        week,
        ustrip(s, reference_time),
    )
    PositionVelocityTime.predict_atmospheric_delays(
        user_pos,
        sat_states,
        [member.sat_position for member in members],
        ionospheric_correction,
        ustrip(s, reference_time),
        doy,
        enable_tropospheric_correction,
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# Measurement prediction

# Per-member clock/IFB column assignment in PositionVelocityTime's
# `BiasColumns` form, so its `calc_ρ_hat!` / `calc_H` do the pseudorange
# modelling (including the earth-rotation correction).
vt_bias_columns(members::AbstractVector{VTMember}, layout::NavFilterLayout) =
    PositionVelocityTime.BiasColumns(
        [member.clock_bias_index for member in members],
        num_clock_biases(layout),
        [member.ifb_index for member in members],
        num_ifb(layout),
    )

# The same assignment restricted to the bias states these members actually
# occupy, renumbered densely, together with `primary_clock_index` translated
# into the dense numbering. The navigation filter carries a fixed set of bias
# states, but a design matrix built for DOP must only carry the columns the
# measurement set can determine: an all-zero column for a constellation with no
# satellites this epoch makes `HᵀH` singular, and `calc_DOP` then reports −1 for
# every epoch a constellation happens to be missing from.
function dense_bias_columns(members::AbstractVector{VTMember}, primary_clock_index::Int)
    clock_states = sort!(unique(member.clock_bias_index for member in members))
    ifb_states =
        sort!(unique(member.ifb_index for member in members if member.ifb_index != 0))
    clock_column = Dict(state => i for (i, state) in enumerate(clock_states))
    ifb_column = Dict(state => i for (i, state) in enumerate(ifb_states))
    columns = PositionVelocityTime.BiasColumns(
        [clock_column[member.clock_bias_index] for member in members],
        length(clock_states),
        [get(ifb_column, member.ifb_index, 0) for member in members],
        length(ifb_states),
    )
    columns, get(clock_column, primary_clock_index, 1)
end

# Predicted pseudoranges (m) for the given satellite positions from the
# position + clock/IFB sub-vector `ξ = [x, y, z, tc₁.., ifb₁..]`.
function predict_pseudoranges(ξ, sat_positions_mat, bias_columns)
    num_sats = size(sat_positions_mat, 2)
    PositionVelocityTime.calc_ρ_hat!(
        Vector{Float64}(undef, num_sats),
        sat_positions_mat,
        ξ,
        bias_columns,
    )
end

# Predicted pseudorange rates (m/s): line-of-sight closing speed plus satellite
# clock drift minus the (single, common) receiver clock drift — the same sign
# convention as the measured `λ · carrier_doppler`.
function predict_pseudorange_rates(
    user_pos,
    user_vel,
    user_clock_drift,
    sat_positions,
    sat_velocities,
    sat_clock_drifts,
)
    map(sat_positions, sat_velocities, sat_clock_drifts) do sat_pos, sat_vel, sat_drift
        e = PositionVelocityTime.calc_e(sat_pos, user_pos)
        dot(e, sat_vel - user_vel) + sat_drift * PositionVelocityTime.SPEEDOFLIGHT -
        user_clock_drift
    end
end

# Jacobian of the full `[pseudoranges; pseudorange rates]` measurement vector
# with respect to the navigation filter state, evaluated at `x`. The rate rows
# are always built — even under VDLL they drive the carrier NCO closure.
function nav_filter_jacobian(
    nav_filter::NavFilterModel,
    x,
    members::AbstractVector{VTMember},
    sat_positions_mat,
    bias_columns,
)
    idxs = nav_filter.idxs
    num_sats = length(members)
    H = PositionVelocityTime.calc_H(
        sat_positions_mat,
        position_and_bias_vector(x, idxs),
        bias_columns,
    )
    J = zeros(2 * num_sats, size(nav_filter.F, 1))
    for (j, member) in enumerate(members)
        J[j, idxs.pos] = H[j, 1:3]
        J[j, idxs.clock_biases[member.clock_bias_index]] = 1.0
        if member.ifb_index != 0
            J[j, idxs.ifb[member.ifb_index]] = 1.0
        end
        if !isempty(idxs.vel)
            J[num_sats + j, idxs.vel] = -H[j, 1:3]
        end
        if idxs.clock_drift != 0
            J[num_sats + j, idxs.clock_drift] = -1.0
        end
    end
    J
end

# The rate rows are the derived single-cycle variance, deliberately carrying no inflation
# factor. Consecutive cycles are not independent: Tracking chains the FLL's `previous_prompt`
# across cycles (each chunk's first record reads the carried-over
# `last_fully_integrated_filtered_prompt`) while `reset_carrier_discr_acc!` fires every cycle,
# so cycle `i` measures `(θ_N − θ_0)/(2π·T)` and cycle `i+1` measures `(θ_2N − θ_N)/(2π·T)`.
# They share the boundary phase estimate with opposite signs, giving
#     cov = −σ_φ²/(2π·T)²,   var = 2·σ_φ²/(2π·T)²   ⇒   ρ(lag 1) = −1/2,
# which a Kalman update cannot represent. Two consequences, neither of them a reason to inflate
# `R`:
#
#  - The correlation is *negative*, so noise averages out faster across cycles than a
#    white-noise filter credits. The filter is therefore already pessimistic about the rate
#    channel, not overconfident — inflating `R` moves further in the direction it already errs,
#    and widens the pseudorange innovation gates as a side effect. What the correlation does
#    cost is a pessimistic reported velocity / clock-drift uncertainty, which only measurement
#    differencing (Bryson-Henrikson) or carrying the boundary phase as a state would fix.
#  - The rate residuals carry a ≈ −0.5 lag-1 autocorrelation *by construction*. That is the
#    telescoping, not a tracking fault, and it must not be tuned against.
#
# Note also that the derived variance is not conservative by accident: treating all `N`
# per-dump discriminators as independent would give `N` times this value, and it is exactly the
# −1/2 adjacency correlation making the interior phases telescope away that earns the tighter
# figure. It is the right variance for the estimator `mean_carrier_discr` actually is.

# Measurement-noise covariance: CN0-driven DLL thermal-noise variance for the pseudoranges,
# and the pseudorange-rate variance for the ATAN frequency-lock discriminator used in Tracking
# (`atan(cross/dot)/(2π·T_coh)`; see Tracking's `fll_disc`).
#
# Both rows are built the same way: the per-dump discriminator variance for a coherent
# integration time `T_coh` (`member.coherent_integration_time`), propagated through the
# averaging of the `N = T/T_coh` dumps that the filter's measurement is a mean of.
#
# Code. Tracking's `dll_disc` is the *noncoherent* envelope-normalized early-minus-late
# discriminator, whose per-dump jitter carries a squaring loss set by the coherent
# integration time (Kaplan & Hegarty, "Understanding GPS: Principles and Applications",
# 2nd ed., Artech House 2006, §5.5.2, noncoherent early-late DLL tracking jitter; derived in
# general form by Betz & Kolodziejski, "Generalized Theory of Code Tracking with an
# Early-Late Discriminator, Part II: Noncoherent Processing and Numerical Results", IEEE
# Trans. Aerospace and Electronic Systems 45(4), 2009, pp. 1557-1564):
#     σ_τ,dump² = d/(4·C/N0·T_coh) · (1 + 2/((2 − d)·C/N0·T_coh))   [chips²],
# the first factor being the coherent early-late variance (the reference formulas carry a
# loop noise bandwidth `B_n`; a single dump is the open-loop case `B_n = 1/(2·T_coh)`) and
# the bracket the squaring loss from multiplying two noisy envelopes. The filter's code
# measurement is `mean_code_discr`, the mean of the `N` per-dump discriminators, whose
# noise is white across dumps (successive dumps share no samples), so
#     var(mean) = σ_τ,dump²/N = d/(4·C/N0·T) · (1 + 2/((2 − d)·C/N0·T_coh))   [chips²],
# i.e. the thermal term averages down over the whole filter interval `T` while the squaring
# loss stays pinned to `T_coh` — a short coherent dump inflates the code variance no matter
# how long the filter interval is. The pseudorange variance is `chip_length²` times that.
# For the BOC VEML correlator `d` is the inner early-late pair's spacing, so the model is
# the EPL approximation of it: the extra very-early/very-late taps average a little more
# noise away, making the model mildly conservative there.
#
# Rate. A coherent dump of length `T_coh` estimates carrier phase with the ATAN
# discriminator jitter
#     σ_φ² = 1/(2·C/N0·T_coh)·(1 + 1/(2·C/N0·T_coh))   [rad²].
# The filter's rate measurement is `mean_carrier_discr`, the mean of the `N` per-dump
# discriminators in `carrier_discr_acc`. Each dump is a frequency — a phase difference over
# one `T_coh`, `(θ_k − θ_{k-1})/(2π·T_coh)` — so the mean reduces to `(θ_N − θ_0)/(2π·T)`:
# the interior phases cancel, leaving only the two endpoint phase estimates, giving
#     var(mean) = 2·σ_φ² / (2π·T)²   [Hz²],
# and the pseudorange-rate variance is λ² times that.
function vt_measurement_noise_covariance(
    members::AbstractVector{VTMember},
    integration_time,
)
    T = ustrip(s, integration_time)
    num_sats = length(members)
    R = zeros(2 * num_sats, 2 * num_sats)
    for (j, member) in enumerate(members)
        cn0_tcoh = member.cn0 * member.coherent_integration_time
        # The noise model assumes the early and late taps still sit inside the correlation
        # triangle, i.e. `d < 2` chips — which every real correlator configuration is well
        # under (Tracking's own default is 0.5).
        d = member.early_late_spacing
        squaring_loss = 1 + 2 / ((2 - d) * cn0_tcoh)
        R[j, j] = d / (4 * T * member.cn0) * squaring_loss * member.chip_length^2
        sigma_phi2 = 1 / (2 * cn0_tcoh) * (1 + 1 / (2 * cn0_tcoh))
        R[num_sats + j, num_sats + j] = member.wavelength^2 * sigma_phi2 / (2 * π^2 * T^2)
    end
    R
end

# Innovation gate per member: a pseudorange innovation beyond twice the
# correlation triangle width means the code loop no longer brackets the true
# delay — the satellite's NCO has diverged from the navigation solution and it
# must be released rather than allowed to drag the filter. The gate is widened
# to the innovation's own predicted uncertainty when the filter is genuinely
# unsure (e.g. the first measurements of a constellation whose clock bias has
# not been observed yet), so a large-but-explained innovation is absorbed by
# the Kalman update instead of releasing a healthy satellite.
function innovation_gates(members, J, P, R)
    map(enumerate(members)) do (j, member)
        innovation_std = sqrt(dot(view(J, j, :), P, view(J, j, :)) + R[j, j])
        max(2 * member.chip_length, 4 * innovation_std)
    end
end

# ─────────────────────────────────────────────────────────────────────────────
# Bias observability

# Accuracy (m) credited to a broadcast offset to GPS Time — the Galileo GGTO or the
# BeiDou BGTO — when it is used to collapse that constellation's clock state onto the GPS
# one. Both are broadcast to a few nanoseconds, so a few metres of pseudorange — good
# enough to remove a clock unknown under satellite starvation, but far too coarse to
# constrain a solution whose geometry can observe the offset directly, which is why the
# collapse is only applied when the independent layout is not supported.
const GPST_OFFSET_STD = 3.0

# The bias-layout decision for one measurement set, mirroring
# `PositionVelocityTime.decide_bias_layout` for the navigation filter.
#
# `num_unknowns` is the number of parameters this cycle's pseudoranges have to
# determine — 3 position components plus the clock biases and the *observable*
# inter-frequency biases of the layout in force, the merged one where the clock
# collapse applies (see `epoch_bias_unknowns`). It is the measurement count the
# epoch must reach to be solvable, and so accounts for the bias unknowns a
# multi-GNSS, multi-band configuration adds beyond the bare four.
#
# `num_distinct_sats_required` is the second, independent condition
# `decide_bias_layout` makes, and reaching `num_unknowns` measurements does not
# imply it: the measurement count may be made up of extra *bands* of satellites
# already counted, and a second band of an already-tracked satellite carries no
# new line of sight — it constrains the inter-frequency biases, not the position
# and clock unknowns. Those need `3 + clock biases` measurements from distinct
# satellites (`num_distinct_sats`, identified as `decide_bias_layout` identifies
# them). Three satellites on two bands is the case the plain count misses: six
# measurements against five unknowns passes, while the design's rows take only
# four distinct values outside the inter-frequency-bias columns.
#
# Both conditions are necessary and neither is sufficient — the surviving
# geometry can still be degenerate in a way no satellite count can see. The
# scalar solve leaves that to `calc_pvt`'s checks on the assembled design; here
# it is reported as `VTStatus`'s `position_std` rather than policed.
#
# `gpst_offset_constraints` holds one `(state, gpst_state, isb)` per collapsed time
# system: the collapse, expressed as the linear pseudo-measurement
# `x[state] - x[gpst_state] = isb` rather than as a merged design-matrix column, so the
# filter keeps a fixed state dimension and that constellation's clock stays available
# (and keeps tracking the shared oscillator drift) for the epochs where the geometry does
# observe it. It is a vector rather than one entry because a mixed epoch can collapse
# several systems at once — Galileo through its GGTO and BeiDou through its BGTO,
# independently — which is what `decide_bias_layout` does on the scalar side. Empty when
# nothing is collapsed.
struct BiasObservability
    num_unknowns::Int
    num_distinct_sats_required::Int
    num_distinct_sats::Int
    gpst_offset_constraints::Vector{Tuple{Int,Int,Float64}}
end

# Whether a measurement set can determine the navigation state under the layout it was
# assessed for: both of `decide_bias_layout`'s conditions, the second of which the
# assessment already holds the operands of.
is_epoch_solvable(obs::BiasObservability, num_measurements) =
    num_measurements >= obs.num_unknowns &&
    obs.num_distinct_sats >= obs.num_distinct_sats_required

# The three quantities `decide_bias_layout` decides the scalar layout from, evaluated for
# one measurement set: how many parameters its pseudoranges have to determine, how many of
# those need a distinct satellite each (the position and clock unknowns — see
# `BiasObservability`), and how many connected components its (constellation × band)
# coverage graph has.
#
# Both are read off the *epoch's own* coverage graph rather than off the configured layout,
# exactly as `decide_bias_layout` reads them off the epoch's satellites. `band_ifb_layout`
# creates an inter-frequency-bias column only where the bias is observable, so a band whose
# component reference carries no measurement this cycle folds its delay into the clock and
# is not an unknown of this epoch — counting the configured layout's columns instead would
# demand a measurement for a parameter this epoch cannot and need not determine.
function epoch_bias_unknowns(time_systems, bands)
    _, extra_bands, _, num_components =
        PositionVelocityTime.band_ifb_layout(time_systems, bands)
    num_distinct_sats_required = 3 + length(unique(time_systems))
    num_distinct_sats_required + length(extra_bands),
    num_distinct_sats_required,
    num_components
end

# Decide the bias layout for this cycle's measurement set, following
# `decide_bias_layout`: estimate every bias independently when the
# (constellation × band) coverage graph is connected and there are enough
# measurements, and otherwise fall back to the broadcast clock collapse — which
# both removes a clock unknown (the scarce-satellite case) and reconnects a
# disjoint band split (the disconnected case, where a band's inter-frequency
# bias is collinear with the stranded constellation's clock).
function assess_bias_observability(
    vt::VectorTrackingState,
    members::AbstractVector{VTMember},
    candidate_indices,
)
    layout = vt.layout
    num_measurements = length(candidate_indices)
    # Only the constellations and bands some measurement touches are unknowns of this
    # epoch; a constellation or band without measurements simply coasts.
    time_systems =
        [layout.time_systems[members[j].clock_bias_index] for j in candidate_indices]
    bands = [layout.band_by_group[members[j].group_key] for j in candidate_indices]
    # Distinct physical satellites, identified by `(time system, PRN)` exactly as
    # `decide_bias_layout` identifies them — a PRN is only unique within its GNSS, and a
    # satellite tracked on several bands is one line of sight however many measurements it
    # contributes. The clock-bias state and the time system are in bijection here, so
    # keying on either identifies the same satellites.
    num_distinct_sats = length(
        unique(zip(time_systems, (members[j].prn for j in candidate_indices))),
    )

    num_unknowns, num_distinct_required, num_components =
        epoch_bias_unknowns(time_systems, bands)
    independent = BiasObservability(
        num_unknowns,
        num_distinct_required,
        num_distinct_sats,
        Tuple{Int,Int,Float64}[],
    )
    if num_components == 1 && is_epoch_solvable(independent, num_measurements)
        return independent
    end

    # Connected-but-scarce or disconnected: collapse every non-GPS clock that this cycle
    # has a broadcast offset to GPS Time for onto the GPS one — Galileo through its GGTO,
    # BeiDou through its BGTO, and each independently of the other, exactly as
    # `decide_bias_layout` does. Worth doing only for a system whose clock and the GPS one
    # are both in play this cycle; otherwise it would replace a well-observed clock state
    # with the coarser broadcast value for nothing.
    gpst_state = findfirst(==(GNSSSignals.GPST()), layout.time_systems)
    constraints = Tuple{Int,Int,Float64}[]
    collapsed = GNSSSignals.TimeSystem[]
    if !isnothing(gpst_state) && GNSSSignals.GPST() in time_systems
        for state in eachindex(layout.time_systems)
            time_system = layout.time_systems[state]
            (time_system == GNSSSignals.GPST() || time_system ∉ time_systems) && continue
            # The offset is one constellation-wide value whichever of the system's
            # satellites reports it, so the first decoded copy per system converts all of
            # that system's measurements — the rule `calc_gpst_range_offsets` follows.
            offset_index = findfirst(
                j ->
                    members[j].clock_bias_index == state &&
                        PositionVelocityTime.gpst_offset_available(
                            members[j].sat_state.decoder,
                        ),
                candidate_indices,
            )
            isnothing(offset_index) && continue
            member = members[candidate_indices[offset_index]]
            # The broadcast offset is Δt_systems = (that system's time) − GPST, and the
            # clock states are in metres of pseudorange, so that system's clock sits
            # −c·Δt_systems from the GPS one — the same sign convention
            # `decide_bias_layout` gives its `inter_system_biases`.
            isb =
                -PositionVelocityTime.SPEEDOFLIGHT *
                PositionVelocityTime.calc_gpst_offset(member.sat_state.decoder, member.time)
            push!(constraints, (state, gpst_state, isb))
            push!(collapsed, time_system)
        end
    end
    if !isempty(constraints)
        # The unknowns have to be recounted on the merged graph rather than simply
        # decremented: dropping a clock removes one unknown, but merging two
        # constellations can also reconnect two coverage components — which is the
        # point of the collapse in the disconnected case — and every band that stops
        # being a component reference then becomes an observable, and countable,
        # inter-frequency bias again. `decide_bias_layout` recounts for the same reason.
        merged_unknowns, merged_distinct_required, _ = epoch_bias_unknowns(
            [ts ∈ collapsed ? GNSSSignals.GPST() : ts for ts in time_systems],
            bands,
        )
        # Reported even when the merged layout is still short of its conditions, where
        # `decide_bias_layout` would fall back to the independent one and call the epoch
        # unsolvable: the merge can only lower both requirements (it drops a clock unknown
        # per collapsed system and can add back at most one inter-frequency bias each), so
        # the two agree on solvability, and applying the constraints on a starved epoch is
        # free information rather than a decision.
        return BiasObservability(
            merged_unknowns,
            merged_distinct_required,
            num_distinct_sats,
            constraints,
        )
    end

    # No collapse available: the layout stays independent, and the epoch is solvable only
    # if the measurements and the distinct satellites among them suffice on their own.
    independent
end

# 1σ 3-D position uncertainty (m) of a navigation-filter covariance.
position_uncertainty(P, idxs::NavFilterIndices) = sqrt(tr(view(P, idxs.pos, idxs.pos)))

# The filter's own 1σ uncertainties, as reported with a receiver snapshot: the 3-D position
# and the clock bias the solution is referenced to (`primary_clock_index`). Reading them
# through these accessors keeps the covariance's state layout — which state index a clock
# bias lives at — inside this file.
position_uncertainty(vt::VectorTrackingState) =
    position_uncertainty(vt.covariance, vt.nav_filter.idxs)

function clock_uncertainty(vt::VectorTrackingState)
    index = vt.nav_filter.idxs.clock_biases[vt.primary_clock_index]
    sqrt(vt.covariance[index, index])
end

# ─────────────────────────────────────────────────────────────────────────────
# Loop closure

# NCO corrections for the selected members: the post-update prediction residual
# — the difference between the pseudorange the filter now predicts for the
# satellite and the one its replica currently realises — converted into the code
# / carrier frequency offsets that remove it over the next interval. Returns two
# PRN-keyed dictionaries (the members must all belong to one group). The carrier
# corrections feed the FLL branch of each satellite's carrier loop.
#
# `predicted_pseudoranges` / `predicted_pseudorange_rates` must be evaluated at
# the *updated* state, which alone makes them the complete correction. Adding the
# measurement update's state correction projected onto the line of sight on top
# would count it twice and command double the required slew — a loop gain of 2
# that leaves the replica oscillating about the solution instead of settling.
function nco_corrections(
    members::Vector{VTMember},
    member_indices,
    predicted_pseudoranges,
    measured_pseudoranges,
    predicted_pseudorange_rates,
    integration_time,
)
    T = ustrip(s, integration_time)
    c = PositionVelocityTime.SPEEDOFLIGHT
    prns = [members[j].prn for j in member_indices]
    code_updates = map(member_indices) do j
        range_error = predicted_pseudoranges[j] - measured_pseudoranges[j]
        -range_error * members[j].code_frequency / (T * c) * Hz
    end
    carrier_updates = map(member_indices) do j
        rate_error = predicted_pseudorange_rates[j] - members[j].pseudorange_rate
        rate_error / members[j].wavelength * Hz
    end
    Dictionary(prns, code_updates), Dictionary(prns, carrier_updates)
end

# Post-fit residuals of one cycle, for EVERY member (not only those used in the
# update): evaluated from the predictions at the *updated* state, so these are
# post-fit residuals and not the pre-fit innovations. Returns `(pseudorange residuals
# in m, range-rate residuals in m/s)`, both in member order and both measured −
# modelled ("observed minus computed") — the orientation the scalar `calc_pvt` reports
# its own two residuals in, and RTKLIB before it, so a vector-tracking solution and a
# scalar one are directly comparable, sign included.
#
# Mind that the two are therefore written with opposite subtraction order here. The
# pseudorange residual is `z - h(x)`, straightforwardly. The rate residual is
# `h(x) - z` because the rate *observable* differs: this loop measures
# `+λ · carrier_doppler`, positive while the satellite closes, whereas `calc_pvt` (and
# RTKLIB's `resdop`) residuate the geometric range rate, positive while it recedes.
# Observed − computed of that quantity is `h(x) - z` of this one — verified against
# `calc_pvt`'s own numbers, not just its wording. Making both subtractions read alike
# would silently invert the reported rate residual against every scalar fix.
#
# Because the vector loop steers every replica onto the navigation solution, a
# well-tracked member's residuals reduce to its own discriminators — the code residual
# to `+code_discriminator · chip_length`, the rate residual to
# `-carrier_discriminator · wavelength` — while a member the solution predicts poorly
# (one diverging, or one out of lock and coasting) keeps a large residual. That is
# what makes them worth reporting for members outside the update too: such a member
# is monitored rather than dropped as a missing satellite.
#
# The rate residual is a least-squares post-fit residual proper only under VDFLL,
# where the rates entered the update. Under VDLL (`use_pseudorange_rates = false`)
# the rates are measured but never fused, so it tests the velocity/clock-drift
# solution against a measurement it never saw — the more searching check of the two.
# Either way it flags a satellite whose Doppler disagrees with the solution
# independently of its pseudorange.
function vt_post_fit_residuals(
    members::AbstractVector{VTMember},
    measured_pseudoranges,
    predicted_pseudoranges,
    predicted_pseudorange_rates,
)
    residuals = [
        (
            measured_pseudoranges[j] +
            members[j].code_discriminator * members[j].chip_length -
            predicted_pseudoranges[j]
        ) * m for j in eachindex(members)
    ]
    rate_residuals = [
        (
            predicted_pseudorange_rates[j] - members[j].pseudorange_rate -
            members[j].carrier_discriminator * members[j].wavelength
        ) * (m / s) for j in eachindex(members)
    ]
    residuals, rate_residuals
end

# Per-member `SatInfo` report, keyed as `pvt.sats` is. `indices` selects the members to
# report: `eachindex(members)` for the filter's own full report (`VectorTrackingState`'s
# `member_sats`, coasted members included), the update's `included_indices` for the solution
# (`pvt.sats`, the satellites that determined it). The two share this constructor so a
# member's entry is identical in both.
function vt_member_sats(
    members::AbstractVector{VTMember},
    residuals,
    rate_residuals,
    indices = eachindex(members),
)
    Dictionary(
        Tuple{Symbol,Int}[(members[j].group_key, members[j].prn) for j in indices],
        SatInfo[
            SatInfo(
                members[j].sat_position,
                members[j].time,
                residuals[j],
                rate_residuals[j],
            ) for j in indices
        ],
    )
end

function set_nco_corrections!(
    track_state,
    group_key,
    members::Vector{VTMember},
    member_indices,
    predicted_pseudoranges,
    measured_pseudoranges,
    predicted_pseudorange_rates,
    integration_time,
)
    isempty(member_indices) && return track_state
    code_updates, carrier_updates = nco_corrections(
        members,
        member_indices,
        predicted_pseudoranges,
        measured_pseudoranges,
        predicted_pseudorange_rates,
        integration_time,
    )
    set_code_freq_updates!(track_state, group_key, code_updates)
    set_carrier_freq_updates!(track_state, group_key, carrier_updates)
    track_state
end

# ─────────────────────────────────────────────────────────────────────────────
# PVT solution from the navigation filter

# The clock-bias state to report the solution against. Kept while its time
# system still contributes an included measurement, so the reference does not
# flicker; re-picked on loss following `PositionVelocityTime`'s reference-system
# convention — GPST when GPS is present, otherwise the time system with the most
# measurements. Re-picking only from time systems that are present this cycle is
# what keeps `vt_time` able to read a week number from a decoded satellite of the
# primary system, so the solution always carries a timestamp.
function report_primary_clock_index(layout, members, included_indices, current_index)
    present = unique(members[j].clock_bias_index for j in included_indices)
    (isempty(present) || current_index in present) && return current_index
    gpst_index = findfirst(==(GNSSSignals.GPST()), layout.time_systems)
    !isnothing(gpst_index) && gpst_index in present && return gpst_index
    # Most measurements, ties broken by clock-state order (first-appearance).
    argmin(index -> (-count(==(index), (members[j].clock_bias_index for j in included_indices)), index), present)
end

# The constant part of the solution's epoch: the primary system's week count in seconds plus
# that system's start epoch — everything in `vt_time` that does not come from the filter
# itself. Resolving it needs a satellite that has finished decoding for positioning in a group
# whose clock is the primary one, and a cycle can lack that: the primary clock is adaptive, so
# its constellation may briefly hold no decoded satellite while the other one carries the
# solution. It is a constant of the run, so it is resolved once and cached on the state —
# after which a solution can no longer lose its timestamp, and a timestamp is how every
# consumer tells a fix from a non-fix.
#
# Caching it survives a change of primary clock: `reference_time` runs on the GPS Time
# count regardless of which clock reports (`VTMember.time_gpst_count`), and the scale
# offset folded in below puts every system's `week·604800 + start epoch` onto that same
# count — BDT's, for instance, lands 14 s below GPST's, exactly compensating the 14 s its
# seconds-of-week read low. What remains between two systems' cached offsets is their
# broadcast steering — nanoseconds, against a quantity used to stamp a 100 ms cycle.
function resolve_time_epoch_offset(
    vt::VectorTrackingState,
    systems,
    receiver_sat_states,
    pvt_approximate_year,
)
    isnothing(vt.time_epoch_offset) || return vt.time_epoch_offset
    for system in systems
        group_key = signal_group_key(system)
        vt.layout.clock_bias_index_by_group[group_key] == vt.primary_clock_index || continue
        receiver_group_states = receiver_sat_states[group_key]
        prn = findfirst(
            state -> is_decoding_completed_for_positioning(state.decoder),
            receiver_group_states,
        )
        isnothing(prn) && continue
        week = PositionVelocityTime.get_week(
            receiver_group_states[prn].decoder;
            approximate_year = pvt_approximate_year,
        )
        start_time = PositionVelocityTime.system_start_epoch(data_signal(system))
        # `reference_time` is a GPS-Time-count time of week, so the epoch that anchors it
        # must absorb the system's scale offset: a BDT week·604800 + start epoch pairs
        # with BDT seconds-of-week, which read 14 s below the GPST count.
        scale_offset = round(
            Int,
            PositionVelocityTime.time_scale_offset_to_gpst(
                get_time_system(data_signal(system)),
            ),
        )
        return week * SECONDS_PER_WEEK + start_time.second + scale_offset
    end
    nothing
end

# The epoch offset to carry into the next cycle: the cached one (or a freshly resolved one),
# advanced by a week when `reference_time` has just wrapped at the 604800 s boundary. Without
# that advance a mid-run Saturday→Sunday rollover would leave every reported `pvt.time`
# exactly one week in the past for the rest of the run — the wrap takes a week off the time
# of week and the offset, being a constant of the run everywhere else, never puts it back.
# Only an offset that was *already* cached is advanced: one resolved on this very cycle was
# read from a decoder that has rolled its own week number over too, so it is already current.
rolled_over_time_epoch_offset(resolved, cached, week_rollover) =
    week_rollover && !isnothing(cached) ? cached + SECONDS_PER_WEEK : resolved

# Absolute epoch of the vector-tracking solution: the clock-bias-corrected reference time on
# the primary system's time-of-week axis, anchored by the cached epoch offset above
# (`nothing` only until the filter has once seen a decoded primary-system satellite).
function vt_time(vt::VectorTrackingState)
    isnothing(vt.time_epoch_offset) && return nothing
    idxs = vt.nav_filter.idxs
    primary_clock_bias = vt.state[idxs.clock_biases[vt.primary_clock_index]]
    corrected_reference_time =
        ustrip(s, vt.reference_time) - primary_clock_bias / PositionVelocityTime.SPEEDOFLIGHT
    TAIEpoch(
        vt.time_epoch_offset + floor(Int, corrected_reference_time),
        corrected_reference_time - floor(corrected_reference_time),
    )
end

function vt_pvt_solution(
    vt::VectorTrackingState,
    members::Vector{VTMember},
    included_indices,
    residuals,
    rate_residuals,
)
    idxs = vt.nav_filter.idxs
    layout = vt.layout
    user_pos, user_vel, user_clock_drift = nav_filter_states(vt.state, idxs)
    position = ECEF(user_pos...)
    velocity = ECEF(user_vel...)
    primary_clock_bias = vt.state[idxs.clock_biases[vt.primary_clock_index]]

    # Geometry of the satellites this update measured, or `nothing` when there is none to
    # report: no measurements at all, or a rank-deficient design for which `calc_DOP` returns
    # its all-`-1` sentinel. That sentinel stays inside the receiver — `calc_pvt` never emits
    # one either (it rejects the epoch instead), so a solution's `dop` is always either a real
    # geometry or absent, and a consumer plotting DOP on a log axis cannot be handed a
    # negative number.
    dop = if isempty(included_indices)
        nothing
    else
        included = members[included_indices]
        columns, primary_column = dense_bias_columns(included, vt.primary_clock_index)
        H = PositionVelocityTime.calc_H(
            stack(member.sat_position for member in included),
            position_and_bias_vector(vt.state, idxs),
            columns,
        )
        candidate = PositionVelocityTime.calc_DOP(H, position, primary_column)
        candidate.GDOP < 0 ? nothing : candidate
    end

    # The solution reports the satellites that determined it — the members this update
    # measured — which is what `calc_pvt` reports for a scalar solve, so a consumer reads
    # `pvt.sats` the same way under either tracking mode and `pvt.dop` above describes
    # exactly this set. The coasted members are not lost: they are reported (with their own
    # post-fit residuals) through the filter's `member_sats`, built by `vt_member_sats`.
    sats = vt_member_sats(members, residuals, rate_residuals, included_indices)

    inter_system_biases = Dict{GNSSSignals.TimeSystem,typeof(1.0m)}()
    for (index, time_system) in enumerate(layout.time_systems)
        index == vt.primary_clock_index && continue
        inter_system_biases[time_system] =
            (vt.state[idxs.clock_biases[index]] - primary_clock_bias) * m
    end
    inter_frequency_biases = Dict{Symbol,PositionVelocityTime.InterFrequencyBias}()
    for (index, band) in enumerate(layout.extra_bands)
        inter_frequency_biases[band] = PositionVelocityTime.InterFrequencyBias(
            vt.state[idxs.ifb[index]] * m,
            layout.reference_bands[index],
        )
    end

    PVTSolution(
        position,
        velocity,
        PositionVelocityTime.calc_course_over_ground(position, velocity),
        primary_clock_bias * m,
        vt_time(vt),
        user_clock_drift / PositionVelocityTime.SPEEDOFLIGHT,
        dop,
        sats,
        layout.time_systems[vt.primary_clock_index],
        inter_system_biases,
        inter_frequency_biases,
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# Initialization

# Initial navigation state and covariance from a scalar PVT fix. The primary
# system's clock bias starts at zero (the pseudorange reference epoch is
# already corrected by the fix's clock bias); the other systems' biases and
# the inter-frequency biases are seeded from the fix where it observed them,
# and start at zero with a generous variance where it did not — their first
# measurements then pull them in through the Kalman update (see
# `innovation_gates`).
function initial_nav_state(vt::VectorTrackingState, pvt)
    nav_filter = vt.nav_filter
    layout = vt.layout
    idxs = nav_filter.idxs
    n = size(nav_filter.F, 1)
    c = PositionVelocityTime.SPEEDOFLIGHT

    init_std_pos = 1.0                # m
    init_std_vel = 1.0                # m/s
    init_std_acc = 1.0                # m/s²
    init_std_clock_bias = 5e-9 * c    # 5 ns in m
    init_std_clock_drift = 5e-9 * c   # 5 ns/s in m/s
    init_std_unseeded_clock_bias = 100.0 # m — inter-system offsets are tens of m at most
    init_std_seeded_ifb = 1.0         # m
    init_std_unseeded_ifb = 30.0      # m — RF-chain delays are below ~100 ns

    x = zeros(n)
    P = zeros(n, n)
    x[idxs.pos] = [pvt.position.x, pvt.position.y, pvt.position.z]
    P[idxs.pos, idxs.pos] = init_std_pos^2 * I(3)
    if !isempty(idxs.vel)
        x[idxs.vel] = [pvt.velocity.x, pvt.velocity.y, pvt.velocity.z]
        P[idxs.vel, idxs.vel] = init_std_vel^2 * I(3)
    end
    if !isempty(idxs.acc)
        P[idxs.acc, idxs.acc] = init_std_acc^2 * I(3)
    end
    if idxs.clock_drift != 0
        x[idxs.clock_drift] = pvt.relative_clock_drift * c
        P[idxs.clock_drift, idxs.clock_drift] = init_std_clock_drift^2
    end

    primary_clock_index =
        something(findfirst(==(pvt.reference_system), layout.time_systems), 1)
    for (index, time_system) in enumerate(layout.time_systems)
        state_index = idxs.clock_biases[index]
        if index == primary_clock_index
            P[state_index, state_index] = init_std_clock_bias^2
        elseif haskey(pvt.inter_system_biases, time_system)
            x[state_index] = ustrip(m, pvt.inter_system_biases[time_system])
            P[state_index, state_index] = init_std_clock_bias^2
        else
            P[state_index, state_index] = init_std_unseeded_clock_bias^2
        end
    end
    for (index, band) in enumerate(layout.extra_bands)
        state_index = idxs.ifb[index]
        fix_ifb = get(pvt.inter_frequency_biases, band, nothing)
        # Only take the fix's bias when it was measured against the same
        # reference band as the filter's layout — otherwise it refers to a
        # different quantity.
        if !isnothing(fix_ifb) && fix_ifb.reference == layout.reference_bands[index]
            x[state_index] = ustrip(m, fix_ifb.value)
            P[state_index, state_index] = init_std_seeded_ifb^2
        else
            P[state_index, state_index] = init_std_unseeded_ifb^2
        end
    end

    x, P, primary_clock_index
end

# Switch from scalar to vector tracking off a fresh scalar PVT fix: promote the
# fix's satellites into the vector loop (per group), seed the navigation filter
# from the fix, and close the loops a first time so the NCOs already steer
# toward the navigation solution when the next chunk is tracked. `pvt` is the
# latest solve and `previous_pvt` the one before it. Returns `(track_state,
# receiver_sat_states, vt)` unchanged (still not running) when there is nothing
# to start from — a stale/failed solve, or a fix that has kept no tracked
# satellite.
function initialize_vector_tracking(
    vt::VectorTrackingState,
    systems,
    track_state,
    receiver_sat_states,
    previous_pvt,
    pvt,
    sampling_freq,
    integration_time;
    enable_ionospheric_correction = true,
    enable_tropospheric_correction = true,
    pvt_approximate_year::Integer = year(now(UTC)),
)
    # This cycle's solution comes from the scalar solve (that is why the loop is not
    # running), so the filter has no per-member report to make: drop the one the last update
    # left, rather than let the emitted status carry it alongside a solution it does not
    # describe. Also covers the enabling cycle below — the seeding fix is a scalar solve too.
    isempty(vt.member_sats) ||
        (vt = VectorTrackingState(vt; member_sats = empty_member_sats()))

    # Only a fresh fix seeds the loop. `calc_pvt` returns the *same* object on a
    # failed/unsolvable epoch, so identity is exactly the freshness test.
    pvt === previous_pvt && return track_state, receiver_sat_states, vt

    # The fix's satellites, per group, restricted to those still tracked.
    prns_by_group = map(systems) do system
        group_key = signal_group_key(system)
        tracked_prns = keys(get_sat_states(track_state, group_key))
        [
            prn for (signal_id, prn) in keys(pvt.sats) if
            signal_id == group_key &&
            prn in tracked_prns &&
            haskey(receiver_sat_states[group_key], prn)
        ]
    end
    # Need at least one still-tracked fix satellite to seed the loop from: the
    # guard sits before `enable_vt!` puts anything in the loop and keeps the
    # reference-epoch `maximum(...)` reduction below defined. The navigation
    # filter and its per-cycle observability watchdog (`assess_bias_observability`)
    # take over the geometry check from there.
    any(!isempty, prns_by_group) || return track_state, receiver_sat_states, vt

    @info "Enabling vector tracking" prns_by_signal = Dict(
        signal_group_key(system) => prns for
        (system, prns) in zip(systems, prns_by_group) if !isempty(prns)
    )
    vt = ensure_nav_filter_integration_time(vt, integration_time)
    nav_filter = vt.nav_filter
    idxs = nav_filter.idxs

    x, P, primary_clock_index = initial_nav_state(vt, pvt)
    vt = VectorTrackingState(vt; state = x, covariance = P, primary_clock_index)
    user_pos, user_vel, user_clock_drift = nav_filter_states(x, idxs)

    # Promote the fix's satellites into the vector loop (per group) and gather
    # their measurement data. The pseudorange reference epoch is the latest
    # transmit time corrected by the fix's receiver clock bias (matching how
    # `calc_pvt` timestamps the fix).
    members = VTMember[]
    for (system, prns) in zip(systems, prns_by_group)
        isempty(prns) && continue
        group_key = signal_group_key(system)
        enable_vt!(track_state, group_key, prns)
        # The fix's satellites are all in lock, so all are available.
        collect_vt_members!(
            members,
            track_state,
            system,
            group_key,
            receiver_sat_states[group_key],
            vt.layout,
            prns,
            prns,
            0.0s, # placeholder reference epoch; the pseudoranges are rebuilt below
            nav_filter.integration_time,
            sampling_freq,
        )
    end
    reference_time =
        (
            maximum(member.time_gpst_count for member in members) -
            ustrip(m, pvt.time_correction) / PositionVelocityTime.SPEEDOFLIGHT
        ) * s
    measured_pseudoranges =
        [
            pseudorange_from_tows(ustrip(s, reference_time), member.time_gpst_count) for
            member in members
        ] .- vt_atmospheric_delays(
            members,
            user_pos,
            reference_time,
            enable_ionospheric_correction,
            enable_tropospheric_correction,
            pvt_approximate_year,
        )

    # First loop closure: the filter is seeded exactly at the fix, so the NCO
    # corrections are the prediction residuals at the seeded state.
    bias_columns = vt_bias_columns(members, vt.layout)
    sat_positions_mat = stack(member.sat_position for member in members)
    ξ = position_and_bias_vector(x, idxs)
    predicted = predict_pseudoranges(ξ, sat_positions_mat, bias_columns)
    predicted_rates = predict_pseudorange_rates(
        user_pos,
        user_vel,
        user_clock_drift,
        [member.sat_position for member in members],
        [member.sat_velocity for member in members],
        [member.sat_clock_drift for member in members],
    )
    for group_key in unique(member.group_key for member in members)
        member_indices = findall(member -> member.group_key == group_key, members)
        set_nco_corrections!(
            track_state,
            group_key,
            members,
            member_indices,
            predicted,
            measured_pseudoranges,
            predicted_rates,
            nav_filter.integration_time,
        )
    end

    receiver_sat_states = sync_vt_flags(receiver_sat_states, track_state)
    vt = VectorTrackingState(
        vt;
        running = true,
        reference_time,
        time_with_insufficient_meas = 0.0s,
    )
    track_state, receiver_sat_states, vt
end

# ─────────────────────────────────────────────────────────────────────────────
# The vector-tracking iteration

# PRNs of one group eligible for the vector loop: tracked, fully decoded and
# healthy.
function vt_eligible_prns(receiver_group_states, tracked_prns)
    [
        state.prn for state in receiver_group_states if
        state.prn in tracked_prns &&
        is_decoding_completed_for_positioning(state.decoder) &&
        is_sat_healthy(state.decoder)
    ]
end

# One navigation-filter cycle: predict, fuse the accumulated discriminators as
# pseudorange (and, for VDFLL, pseudorange-rate) measurements — atmosphere
# corrected and innovation gated — close every group's tracking loops with
# fresh NCO corrections, and manage the vector-loop membership (promotion,
# availability, release, fallback to scalar tracking).
# Returns `(pvt, vt, track_state, receiver_sat_states)`.
function run_vt_iteration(
    vt::VectorTrackingState,
    systems,
    track_state,
    receiver_sat_states,
    sampling_freq,
    integration_time;
    enable_ionospheric_correction = true,
    enable_tropospheric_correction = true,
    pvt_approximate_year::Integer = year(now(UTC)),
)
    vt = ensure_nav_filter_integration_time(vt, integration_time)
    nav_filter = vt.nav_filter
    idxs = nav_filter.idxs
    # Advance the receive time-of-week, wrapping at the 604800 s week boundary so
    # it stays a valid seconds-of-week for the pseudorange differencing, the
    # atmospheric time-of-week and the `vt_time` week/second split. The wrap is
    # remembered: the week it drops has to be added to the cached epoch offset
    # (`rolled_over_time_epoch_offset`), which is the only other place the run's
    # absolute epoch is held.
    advanced_reference_time = vt.reference_time + integration_time
    week_rollover = advanced_reference_time >= SECONDS_PER_WEEK * s
    reference_time = mod(advanced_reference_time, SECONDS_PER_WEEK * s)

    # Membership management per group: promote eligible locked satellites into
    # the vector loop, and mark members that lost eligibility (unhealthy or no
    # longer tracked) for release. Members that survive are
    # gathered into the flat member vector every per-satellite array below is
    # aligned with.
    members = VTMember[]
    releasable = Dict{Symbol,Vector{Int}}()
    for system in systems
        group_key = signal_group_key(system)
        receiver_group_states = receiver_sat_states[group_key]
        tracked_sats = get_sat_states(track_state, group_key)
        eligible_prns = vt_eligible_prns(receiver_group_states, keys(tracked_sats))
        # NOTE: this list is deliberately gated on `is_in_lock` and not on
        # `is_ranging_ready`, even though admitting a satellite whose code phase is still
        # tens of metres out into the navigation filter is a real question. It serves two
        # purposes at once — the membership list handed to `enable_vt!` and the
        # measurement-eligibility list handed to `collect_vt_members!` below — so gating it
        # would also withhold discriminators from *existing* members, which is wrong. The
        # two uses need separating first; the filter meanwhile has its own innovation-based
        # release.
        prns_in_lock = filter(prn -> is_in_lock(receiver_group_states[prn]), eligible_prns)
        enable_vt!(track_state, group_key, prns_in_lock)
        member_prns = [get_prn(sat) for sat in tracked_sats if in_vt_loop(sat)]
        ineligible = setdiff(member_prns, eligible_prns)
        if !isempty(ineligible)
            @info "Releasing no longer eligible satellites from the vector loop" group_key ineligible
            releasable[group_key] = ineligible
        end
        # A member is usable as a measurement this cycle iff it is currently in
        # code lock (`prns_in_lock`); the rest stay in the loop and keep getting
        # NCO corrections but their discriminators are withheld.
        collect_vt_members!(
            members,
            track_state,
            system,
            group_key,
            receiver_group_states,
            vt.layout,
            setdiff(member_prns, ineligible),
            prns_in_lock,
            reference_time,
            integration_time,
            sampling_freq,
        )
    end
    active = trues(length(members))

    # Navigation filter prediction.
    tu = time_update(vt.state, vt.covariance, nav_filter.F, nav_filter.Q)
    x_predicted = get_state(tu)
    P_predicted = get_covariance(tu)
    user_pos, user_vel, user_clock_drift = nav_filter_states(x_predicted, idxs)

    # Atmosphere-corrected pseudorange measurements, and the full-member-set
    # model pieces (predictions, Jacobian, noise) at the predicted state.
    measured_pseudoranges =
        [member.pseudorange for member in members] .- vt_atmospheric_delays(
            members,
            user_pos,
            reference_time,
            enable_ionospheric_correction,
            enable_tropospheric_correction,
            pvt_approximate_year,
        )
    bias_columns = vt_bias_columns(members, vt.layout)
    sat_positions_mat =
        isempty(members) ? zeros(3, 0) : stack(member.sat_position for member in members)
    R = vt_measurement_noise_covariance(members, integration_time)

    # Measurement candidates: members whose signal is currently available (the
    # discriminators of an obscured satellite carry no information).
    candidate_indices = findall(member -> member.available, members)

    x_updated, P_updated = x_predicted, P_predicted
    included_indices = Int[]
    # What this epoch must supply to determine the state, decided below from the bias
    # layout actually in force. Until a measurement set is gathered it is the bare floor —
    # 3 position components and one clock, from four distinct satellites none of which are
    # there — so an epoch with no candidates at all counts as unsolvable.
    observability = BiasObservability(4, 4, 0, Tuple{Int,Int,Float64}[])
    if !isempty(candidate_indices)
        z_pseudoranges = [
            measured_pseudoranges[j] +
            members[j].code_discriminator * members[j].chip_length for
            j in candidate_indices
        ]
        # Innovation gate disabled for now: the observability watchdog
        # (`assess_bias_observability` and the starvation timer it feeds) is what
        # catches a diverging solution, and this extra gate could otherwise
        # release a healthy satellite whose large-but-explained innovation the
        # Kalman update would have absorbed (e.g. the first measurements of a
        # not-yet-observed constellation). Kept for reference; re-enabling it
        # also restores the `nav_filter_jacobian` and `innovation_gates` uses.
        #=
        J = nav_filter_jacobian(nav_filter, x_predicted, members, sat_positions_mat, bias_columns)
        z_pre_pseudoranges = predict_pseudoranges(
            position_and_bias_vector(x_predicted, idxs),
            sat_positions_mat[:, candidate_indices],
            vt_bias_columns(members[candidate_indices], vt.layout),
        )
        gates = innovation_gates(members[candidate_indices], J[candidate_indices, :], P_predicted, R[candidate_indices, candidate_indices])
        within_gate = abs.(z_pseudoranges .- z_pre_pseudoranges) .< gates
        if !all(within_gate)
            for j in candidate_indices[.!within_gate]
                member = members[j]
                @info "Releasing satellite with diverged innovation from the vector loop" member.group_key member.prn
                push!(get!(releasable, member.group_key, Int[]), member.prn)
                active[j] = false
            end
            z_pseudoranges = z_pseudoranges[within_gate]
            candidate_indices = candidate_indices[within_gate]
        end
        =#

        if !isempty(candidate_indices)
            # Bias layout for this measurement set: what it has to determine, and
            # which non-GPS clocks have to be collapsed onto the GPS one through
            # their broadcast offset to GPS Time to get there.
            observability = assess_bias_observability(vt, members, candidate_indices)

            included = members[candidate_indices]
            included_positions = [member.sat_position for member in included]
            included_velocities = [member.sat_velocity for member in included]
            included_clock_drifts = [member.sat_clock_drift for member in included]
            included_positions_mat = sat_positions_mat[:, candidate_indices]
            included_bias_columns = vt_bias_columns(included, vt.layout)
            use_rates = vt.config.use_pseudorange_rates
            z = if use_rates
                vcat(
                    z_pseudoranges,
                    [
                        members[j].pseudorange_rate +
                        members[j].carrier_discriminator * members[j].wavelength
                        for j in candidate_indices
                    ],
                )
            else
                z_pseudoranges
            end
            h = function (x)
                pos, vel, clock_drift = nav_filter_states(x, idxs)
                pseudoranges = predict_pseudoranges(
                    position_and_bias_vector(x, idxs),
                    included_positions_mat,
                    included_bias_columns,
                )
                use_rates ?
                vcat(
                    pseudoranges,
                    predict_pseudorange_rates(
                        pos,
                        vel,
                        clock_drift,
                        included_positions,
                        included_velocities,
                        included_clock_drifts,
                    ),
                ) : pseudoranges
            end
            num_members = length(members)
            R_rows =
                use_rates ?
                vcat(candidate_indices, num_members .+ candidate_indices) :
                candidate_indices
            R_meas = R[R_rows, R_rows]

            # Each clock collapse rides along as one extra measurement row — the
            # broadcast offset between two clock states — appended after the
            # satellite rows so the residual slice below is unaffected.
            if !isempty(observability.gpst_offset_constraints)
                constraints = observability.gpst_offset_constraints
                z = vcat(z, [isb for (_, _, isb) in constraints])
                h_sats = h
                h =
                    x -> vcat(
                        h_sats(x),
                        [
                            x[idxs.clock_biases[state]] -
                            x[idxs.clock_biases[gpst_state]] for
                            (state, gpst_state, _) in constraints
                        ],
                    )
                R_meas = cat(
                    R_meas,
                    Diagonal(fill(GPST_OFFSET_STD^2, length(constraints)));
                    dims = (1, 2),
                )
            end

            mu = measurement_update(x_predicted, P_predicted, z, h, R_meas)
            x_updated = get_state(mu)
            P_updated = get_covariance(mu)
            included_indices = candidate_indices
        end
    end

    user_pos, user_vel, user_clock_drift = nav_filter_states(x_updated, idxs)

    # The measurement model at the *updated* state, for every member. Evaluated once
    # for both of its consumers: the post-fit residuals just below, and — unless the
    # loop falls back to scalar tracking — the NCO corrections that close the loops
    # onto the updated solution further down.
    predicted_pseudoranges = predict_pseudoranges(
        position_and_bias_vector(x_updated, idxs),
        sat_positions_mat,
        bias_columns,
    )
    predicted_pseudorange_rates = predict_pseudorange_rates(
        user_pos,
        user_vel,
        user_clock_drift,
        [member.sat_position for member in members],
        [member.sat_velocity for member in members],
        [member.sat_clock_drift for member in members],
    )

    # Post-fit pseudorange and range-rate residuals of every member, reported with
    # the solution below (see `vt_post_fit_residuals`).
    residuals, rate_residuals = vt_post_fit_residuals(
        members,
        measured_pseudoranges,
        predicted_pseudoranges,
        predicted_pseudorange_rates,
    )

    # Starvation watchdog: grow the timer whenever the epoch could not determine the
    # navigation state — too few measurements, or too few distinct satellites among them,
    # for the bias layout in force (`is_epoch_solvable`) — and pay it back down (at half
    # rate) otherwise. How certain the filter is of the solution it did produce is not
    # policed here; it is reported as `VTStatus`'s `position_std`, so a degenerate geometry
    # (whose post-fit residuals stay small while an unobserved direction goes uncorrected)
    # is visible to the consumer rather than acted on by the receiver.
    epoch_is_solvable = is_epoch_solvable(observability, length(included_indices))
    time_with_insufficient_meas =
        epoch_is_solvable ?
        max(0.0s, vt.time_with_insufficient_meas - integration_time / 2) :
        vt.time_with_insufficient_meas + integration_time

    # Elevation mask: release members that dropped below the horizon. One ENU
    # transform for the (shared) user position, reused per satellite.
    enu_from_ecef = Geodesy.ENUfromECEF(ECEF(user_pos...), Geodesy.wgs84)
    for (j, member) in enumerate(members)
        active[j] || continue
        if get_sat_enu(enu_from_ecef, ECEF(member.sat_position...)).ϕ < 0
            @info "Releasing satellite below the horizon from the vector loop" member.group_key member.prn
            push!(get!(releasable, member.group_key, Int[]), member.prn)
            active[j] = false
        end
    end

    # Release the satellites that were dropped for cause *before* closing the
    # loops: `set_code_freq_updates!` requires an NCO entry for every satellite
    # still in the vector loop of its group. Released satellites additionally
    # go out of lock so they are removed from tracking and recovered through
    # reacquisition.
    for (group_key, prns) in releasable
        release_from_vector_tracking!(track_state, group_key, prns)
        receiver_sat_states = force_out_of_lock(receiver_sat_states, group_key, prns)
    end

    # Fall back to scalar tracking when the filter has coasted too long or no
    # members remain; otherwise close every group's loops with fresh NCO
    # corrections toward the updated solution.
    active_indices = findall(active)
    running = true
    if time_with_insufficient_meas > vt.config.insufficient_meas_timeout ||
       isempty(active_indices)
        remaining = [(members[j].group_key, members[j].prn) for j in active_indices]
        @info "Falling back to scalar tracking" time_with_insufficient_meas num_measurements =
            length(included_indices) num_required_measurements = observability.num_unknowns num_distinct_sats =
            observability.num_distinct_sats num_distinct_sats_required =
            observability.num_distinct_sats_required position_uncertainty =
            position_uncertainty(P_updated, idxs) remaining
        for group_key in unique(first.(remaining))
            release_from_vector_tracking!(
                track_state,
                group_key,
                [prn for (key, prn) in remaining if key == group_key],
            )
        end
        running = false
    else
        for group_key in unique(members[j].group_key for j in active_indices)
            member_indices =
                [j for j in active_indices if members[j].group_key == group_key]
            set_nco_corrections!(
                track_state,
                group_key,
                members,
                member_indices,
                predicted_pseudoranges,
                measured_pseudoranges,
                predicted_pseudorange_rates,
                integration_time,
            )
        end
    end

    # The navigation filter consumed this interval's discriminators.
    reset_code_discr_acc!(track_state)
    reset_carrier_discr_acc!(track_state)

    receiver_sat_states = sync_vt_flags(receiver_sat_states, track_state)

    vt = VectorTrackingState(
        vt;
        state = x_updated,
        covariance = P_updated,
        running,
        # The full per-member report of this update, coasted members included. It stands
        # even on a cycle that falls back: the solution below is still this update's, and a
        # coasted member's residual is exactly what explains why the filter is giving up.
        member_sats = vt_member_sats(members, residuals, rate_residuals),
        time_epoch_offset = rolled_over_time_epoch_offset(
            resolve_time_epoch_offset(
                vt,
                systems,
                receiver_sat_states,
                pvt_approximate_year,
            ),
            vt.time_epoch_offset,
            week_rollover,
        ),
        primary_clock_index = report_primary_clock_index(
            vt.layout,
            members,
            included_indices,
            vt.primary_clock_index,
        ),
        reference_time,
        time_with_insufficient_meas,
    )

    pvt = vt_pvt_solution(vt, members, included_indices, residuals, rate_residuals)

    track_state, receiver_sat_states, pvt, vt
end
