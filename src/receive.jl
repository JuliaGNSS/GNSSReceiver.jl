"""
    SatelliteDataOfInterest

Per-satellite summary emitted for each processed chunk: the estimated carrier-to-noise
density ratio `cn0`, the latest fully integrated `prompt` correlator value (a scalar
for single-antenna, an `SVector` for multi-antenna) and whether the satellite reports
itself `is_healthy`.
"""
struct SatelliteDataOfInterest{P<:Union{<:Complex,<:AbstractVector{<:Complex}}}
    cn0::typeof(1.0dBHz)
    prompt::P
    is_healthy::Bool
end

"""
    ReceiverDataOfInterest

Snapshot of the receiver after a processed chunk: `sat_data` maps each tracked satellite
to its [`SatelliteDataOfInterest`](@ref), `pvt` is the current PVT solution and `runtime`
is the elapsed signal time. This is the element type produced by [`receive`](@ref).

`sat_data` is a `Dictionaries.Dictionary` (GNSSReceiver.jl PR #96) keyed by
`(signal_id, prn)` — the ranging signal's id and PRN — so the same PRN can appear on
several constellations and bands without colliding, and the key matches `pvt.sats`
(which `PositionVelocityTime` keys by the same ranging signal).
"""
struct ReceiverDataOfInterest{S<:SatelliteDataOfInterest}
    sat_data::Dictionary{Tuple{Symbol,Int},S}
    pvt::PVTSolution
    runtime::typeof(1.0s)
end

is_sat_healthy_at(sat_state_dict, prn) =
    haskey(sat_state_dict, prn) && is_sat_healthy(sat_state_dict[prn].decoder)

# Prompt correlator value of a satellite's ranging signal (the pilot, for a combined
# system) — the quantity `SatelliteDataOfInterest` reports.
_ranging_prompt(sat_state) =
    get_prompt(get_last_fully_integrated_correlator(sat_state, RANGING_SIGNAL_INDEX))

# Concrete `SatelliteDataOfInterest` element type for `build_sat_data`. All of a receiver's
# groups share one correlator backend and antenna count, so the prompt type `P` is identical
# across them — derive it from the first group's satellite-state type. Using the *type*
# (not a value) keeps the result concretely typed even when no satellite is tracked yet,
# where a `dictionary` comprehension over the runtime-keyed, heterogeneous groups would
# instead infer `Dictionary{Any,Any}` and be rejected by the `ReceiverDataOfInterest` field
# type. `first(keys(...))` is a compile-time symbol (a `NamedTuple` key), so this stays
# inferrable.
function _sat_data_value_type(track_state, receiver_sat_states)
    sat_type = eltype(get_sat_states(track_state, first(keys(receiver_sat_states))))
    SatelliteDataOfInterest{Base.promote_op(_ranging_prompt, sat_type)}
end

# Build the `(signal_id, prn) => SatelliteDataOfInterest` dictionary from a receiver state.
# The tracking-group keys are exactly `keys(receiver_sat_states)`, so no `systems` argument
# is needed. Built as an explicitly-typed dictionary so it is a concrete, uniform
# `Dictionary{Tuple{Symbol,Int},SatelliteDataOfInterest{P}}` whether or not a satellite is
# tracked and across single- or multi-band states.
function build_sat_data(receiver_state)
    track_state = receiver_state.track_state
    receiver_sat_states = receiver_state.receiver_sat_states
    sat_data = Dictionary{Tuple{Symbol,Int},_sat_data_value_type(track_state, receiver_sat_states)}()
    for group_key in keys(receiver_sat_states)
        sat_state_dict = receiver_sat_states[group_key]
        for sat_state in get_sat_states(track_state, group_key)
            prn = get_prn(sat_state)
            insert!(
                sat_data,
                (group_key, prn),
                SatelliteDataOfInterest(
                    estimate_cn0(sat_state, RANGING_SIGNAL_INDEX),
                    _ranging_prompt(sat_state),
                    is_sat_healthy_at(sat_state_dict, prn),
                ),
            )
        end
    end
    sat_data
end

"""
    default_data_of_interest(receiver_state) -> ReceiverDataOfInterest

Condense a `ReceiverState` into the default per-chunk payload emitted by [`receive`](@ref):
each tracked satellite's CN0, prompt correlator value and health — keyed by
`(signal_id, prn)` — together with the current PVT solution and the runtime.

This is the default `extract` function of [`receive`](@ref). Pass your own
`extract(receiver_state)` to emit a different payload (e.g. raw carrier Doppler, code
phase or decoded navigation data); the returned channel's element type is inferred from
what it returns. `extract` runs inside the tracking loop on a `ReceiverState` that the
next chunk mutates in place, so it must be read-only and return an immutable value.
"""
function default_data_of_interest(receiver_state)
    ReceiverDataOfInterest(
        build_sat_data(receiver_state),
        receiver_state.pvt,
        receiver_state.runtime,
    )
end

# Backend from the sample element type: `Complex{Int16}` with a `max_meas` (front-end
# full-scale, e.g. `2^11` for a 12-bit ADC) uses Tracking's fast integer backend, everything
# else the float CPU backend (which ignores `max_meas`). Without `max_meas`, `Complex{Int16}`
# falls back to the float backend (logged) rather than erroring. `max_meas` must be the true
# peak |real|/|imag| — under-declaring it corrupts the correlation. Override via the
# `downconvert_and_correlator` keyword.
default_downconvert_and_correlator(::Type, max_meas) = CPUThreadedDownconvertAndCorrelator()
function default_downconvert_and_correlator(::Type{Complex{Int16}}, max_meas)
    if max_meas === nothing
        @info "`Complex{Int16}` measurements without `max_meas`: using the float CPU " *
              "downconvert-and-correlator. Pass `max_meas` (the front-end full-scale, " *
              "e.g. `2^11` for a 12-bit ADC) to use Tracking's faster integer backend."
        return CPUThreadedDownconvertAndCorrelator()
    end
    Int16ThreadedDownconvertAndCorrelator(max_meas)
end

# Start-of-tracking coherent integration time of `signal`: the loop update period
# the carrier discriminator sees at the acquisition→tracking handover. Before
# bit/secondary-code sync the loops integrate a single primary code block, so this
# is one primary-code period (get_code_length / get_code_frequency).
handover_coherent_integration_time(signal::AbstractGNSSSignal) =
    get_code_length(signal) / get_code_frequency(signal)

# Carrier-Doppler pull-in range: the largest carrier-Doppler error the tracking
# loop can pull into lock from a fresh acquisition handover. `receive` sizes each
# constellation's acquisition Doppler bin from this (bin = 2·margin·pull_in) so the
# worst-case post-acquisition residual lands inside the loop's capture range.
#
# This mirrors the capture behaviour of `Tracking`'s `ConventionalPLLAndDLL`
# estimator using only the released `Tracking` / `GNSSSignals` API — the loop lives
# in `Tracking`, but the sizing decision is the receiver's, so the range is derived
# here rather than queried from the loop.

# FLL-assisted carrier loop — the `ConventionalAssistedPLLAndDLL` default, a
# `ThirdOrderAssistedBilinearLF`. Pull-in comes from the FLL frequency
# discriminator `atan(cross / dot) / (2π·T)`. Its two-quadrant `atan` recovers the
# inter-prompt phase advance `Δφ = 2π·Δf·T` unambiguously only within ±π/2; beyond
# that it folds and drives the loop the wrong way. So the loop pulls in a Doppler
# error only while `2π·|Δf|·T ≤ π/2`, i.e. `|Δf| ≤ 1 / (4·T)` — 250 Hz for a 1 ms
# code (GPS L1 C/A, L5I), 62.5 Hz for Galileo E1B (4 ms), 25 Hz for L1C (10 ms).
function carrier_doppler_pull_in_range(
    ::ConventionalPLLAndDLL{<:Tracking.ThirdOrderAssistedBilinearLF},
    signal::AbstractGNSSSignal,
)
    T = handover_coherent_integration_time(signal)
    uconvert(Hz, 1 / (4 * T))
end

# Pure PLL — any other carrier loop filter. A pure PLL has no frequency
# discriminator, so there is no static frequency pull-in; fast lock-in is instead
# governed by the carrier loop bandwidth `B_L`, capped by the coherent-integration
# decorrelation limit `1 / (2·T)` (beyond which the prompt correlation nulls out):
# `|Δf| ≈ min(B_L, 1 / (2·T))`. `B_L` is the estimator's configured bandwidth, or
# the signal's default when it is auto (`nothing`) — matching how `Tracking` seeds
# each satellite. This is an order-of-magnitude estimate, not the crisp
# discriminator bound of the FLL-assisted case.
function carrier_doppler_pull_in_range(
    estimator::ConventionalPLLAndDLL,
    signal::AbstractGNSSSignal,
)
    B_L = something(
        estimator.carrier_loop_filter_bandwidth,
        default_carrier_loop_filter_bandwidth(signal),
    )
    T = handover_coherent_integration_time(signal)
    uconvert(Hz, min(B_L, 1 / (2 * T)))
end

# Build the per-constellation acquisition plans and the acquisition-buffer sample
# count for one band from the caller-supplied per-system target acquisition Doppler
# resolutions. Returns `(systems, acq_plans, num_samples_for_acquisition)`.
function plan_band_acquisition(
    systems,
    sampling_freq,
    acq_doppler_resolutions;
    prns = nothing,
)
    systems = as_systems(systems)

    # Ensure that systems is single band
    assert_single_band(systems)

    # Acquisition runs on each system's acquisition signal: the pilot when the
    # required resolution lands exactly on a valid pilot coherent length, else the
    # data component (see `acquisition_signal`). Plan sizing, buffer length and the
    # plans themselves are all derived from these signals.
    acq_systems = map(
        (system, acq_doppler_resolution) ->
            acquisition_signal(system, sampling_freq, acq_doppler_resolution),
        systems,
        acq_doppler_resolutions,
    )

    # Coherent code periods per system from the required Doppler resolution
    # (bin spacing = 1 / (nc · T_code)), snapped to a length `plan_acquire` accepts.
    ncoh = map(acq_systems, acq_doppler_resolutions) do system, acq_doppler_resolution
        # `ideal` is the minimum coherent length meeting the required resolution
        # (`ceil`, so the achieved resolution never exceeds the required maximum).
        ideal = coherent_code_periods_for_resolution(system, acq_doppler_resolution)
        snap_coherent_code_periods(system, sampling_freq, ideal)
    end

    # Size this band's buffer to the largest coherent plan window
    # (nc · samples_per_code).
    num_samples_for_acquisition = maximum(
        map((s, nc) -> nc * samples_per_code(s, sampling_freq), acq_systems, ncoh),
    )

    # One acquisition plan per system, keyed like the tracking groups. The plan is
    # built for (and PRNs restricted by) the acquisition signal; the group key is
    # the ranging signal's id.
    group_keys = map(signal_group_key, systems)
    acq_plans = NamedTuple{group_keys}(map(systems, acq_systems, ncoh) do system, acq_sys, nc
        # Per-GNSS candidate PRNs restricted to those that broadcast this signal.
        prns_for_system = search_prns(prns, data_signal(system))
        plan_acquire(
            acq_sys,
            float(sampling_freq),
            collect(prns_for_system);
            num_coherently_integrated_code_periods = nc,
            # Pass our own cap rather than relying on `plan_acquire`'s default: this
            # is the same value the `acquisition_signal` chooser gates on, so the
            # selection decision and the plan's actual rotation-search cap can never
            # drift apart (Acquisition exposes no queryable constant for its default).
            max_secondary_code_rotations = MAX_SECONDARY_CODE_ROTATIONS,
        )
    end)

    return systems, acq_plans, num_samples_for_acquisition
end

"""
    receive(measurement_channel, systems, sampling_freq; num_ants = NumAnts(1), kwargs...)
    receive(measurement_channels::Tuple, systems_per_band::Tuple, sampling_freq; kwargs...)

Run the full acquire → track → decode → PVT pipeline over the samples arriving on
`measurement_channel` and return a channel of per-chunk [`ReceiverDataOfInterest`](@ref).

`systems` is a single GNSS system, a [`CombinedSignal`](@ref) pilot+data pair, or a tuple
of these that share one RF band and sample stream; every constellation is acquired,
tracked and decoded, and all are fused into a single multi-GNSS PVT solution. The
multi-band method takes a tuple of measurement channels (one per RF band), a tuple of
per-band system groups and a tuple of `interm_freqs`, all aligned band-by-band, and fuses
every band into one solution with per-constellation clock biases and per-band
inter-frequency biases. The band channels must deliver equal-length frames from one shared
time base (e.g. a single capture split by front-end channel) so one frame per band stays
aligned each step. The number of antenna channels in each `SignalChannel` must equal `N`
in `num_ants`.

Sampled at `sampling_freq`, each chunk is processed by [`process`](@ref) in a spawned
task; one `ReceiverDataOfInterest` is emitted per `pvt_update_interval`. Acquisition
(Acquisition 2 FM-DBZP with CFAR) runs at most every `acquire_every`; its coherent length
and Doppler resolution are derived per system from the tracking loop pull-in range, and
`prns` restricts the search (`nothing` ⇒ each constellation's default range, a per-GNSS
`NamedTuple`/`Dict` keyed by `get_constellation_id`, or a plain collection applied to
every system). A satellite is declared locked once its CN0 exceeds `code_lock_cn0_threshold` and
contributes to the PVT solution — recomputed every `pvt_update_interval` — after
`time_in_lock_before_calculating_pvt`. `enable_ionospheric_correction`,
`enable_tropospheric_correction` and `pvt_approximate_year` (which resolves the GPS
week-number rollover for old recordings) are passed through to `calc_pvt`.

The downconvert-and-correlator backend is auto-selected from the sample element type:
`Complex{Int16}` inputs use Tracking's fast integer backend when `max_meas` (the front-end
full-scale, e.g. `2^11` for a 12-bit ADC) is given, and otherwise fall back to the float
CPU backend (logged once); every other element type uses the float CPU backend and ignores
`max_meas`. Pass `downconvert_and_correlator` to override the choice.
"""
function receive(
    measurement_channel::SignalChannel,
    systems,
    sampling_freq;
    interm_freq = 0.0u"Hz",
    kwargs...,
)
    receive(
        (measurement_channel,),
        (systems,),
        sampling_freq;
        interm_freqs = (interm_freq,),
        kwargs...,
    )
end

function receive(
    measurement_channels::Tuple{Vararg{SignalChannel}},
    systems_per_band::Tuple,
    sampling_freq;
    num_ants::NumAnts{N} = NumAnts(1),
    interm_freqs::Tuple = map(_ -> 0.0u"Hz", measurement_channels),
    acquire_every = 10u"s",
    # PRNs to acquire. `nothing` ⇒ each constellation's default range; a per-GNSS
    # `NamedTuple`/`Dict` keyed by `get_constellation_id` (`(GPS = …, Galileo = …)`);
    # or a plain collection applied to every system. Each system's search is further
    # restricted to the PRNs that broadcast its signal (see `broadcasting_prns`).
    prns = nothing,
    # Front-end full-scale for `Complex{Int16}` measurements (integer backend); omit it to
    # fall back to the float backend. Ignored for float samples or when
    # `downconvert_and_correlator` is given.
    max_meas = nothing,
    # `nothing` ⇒ auto-select from the sample element type (integer backend for
    # `Complex{Int16}`, float CPU backend otherwise); see `default_downconvert_and_correlator`.
    downconvert_and_correlator = nothing,
    code_lock_cn0_threshold = nothing,
    time_in_lock_before_calculating_pvt = 2u"s",
    pvt_update_interval = 100u"ms",
    enable_ionospheric_correction = true,
    enable_tropospheric_correction = true,
    pvt_approximate_year::Integer = year(now(UTC)),
    # Per-chunk payload builder run inside the tracking loop; see
    # [`default_data_of_interest`](@ref). Pass your own `extract(receiver_state)` to emit
    # a custom payload — it must be read-only and return an immutable value, since the
    # `ReceiverState` it sees is mutated in place by the next chunk.
    extract = default_data_of_interest,
) where {N}
    n_bands = length(measurement_channels)
    (
        length(systems_per_band) == n_bands &&
        length(interm_freqs) == n_bands
    ) || throw(
        ArgumentError(
            "measurement_channels, systems_per_band and interm_freqs must have equal length",
        ),
    )
    all(ch -> num_antenna_channels(ch) == N, measurement_channels) ||
        throw(ArgumentError("The number of antenna channels must match num_ants"))

    # Resolve the downconvert-and-correlator backend from the sample element type unless
    # the caller passed one explicitly. All bands share one capture, so one backend.
    resolved_dc =
        isnothing(downconvert_and_correlator) ?
        default_downconvert_and_correlator(
            eltype(eltype(first(measurement_channels))),
            max_meas,
        ) : downconvert_and_correlator

    # Normalise the systems band-by-band and derive each band's key up front.
    band_systems = map(as_systems, systems_per_band)
    band_keys = map(s -> get_band_id(system_band(first(s))), band_systems)

    # Acquisition Doppler resolution derived per system from the carrier tracking
    # loops' *pull-in range*: the worst-case post-acquisition residual (≈ half a
    # Doppler bin) is held to at most `pull_in_margin` of that range
    # (bin = 2·margin·pull_in) — a smaller margin gives finer bins (more sensitive,
    # more compute) and more clearance from the FLL discriminator's aliasing edge.
    # The pull-in is a property of `doppler_estimator` and each group's *ranging*
    # (driver) signal — the pilot for a combined system, i.e. the signal the loop
    # tracks. `doppler_estimator` must match the loops' estimator; it is baked into
    # the freshly-built receiver state below.
    doppler_estimator = ConventionalAssistedPLLAndDLL()
    pull_in_margin = 0.5
    band_acq_doppler_resolutions = map(band_systems) do systems
        map(systems) do system
            2 * pull_in_margin *
            carrier_doppler_pull_in_range(doppler_estimator, ranging_signal(system))
        end
    end

    # CFAR false-alarm probability for acquisition detection — fixed internally,
    # not a receiver-level tuning knob.
    acq_pfa = DEFAULT_ACQ_PFA

    # Per-band acquisition plans and buffer sizes (each band is validated as
    # single-band by `plan_band_acquisition`).
    setups = map(band_systems, band_acq_doppler_resolutions) do systems, acq_doppler_resolutions
        plan_band_acquisition(systems, sampling_freq, acq_doppler_resolutions; prns)
    end
    # One acquisition-plan NamedTuple across all bands, keyed by group key (unique
    # across bands). `merge` of the per-band NamedTuples flattens them.
    acq_plans = merge(map(s -> s[2], setups)...)

    # Build the single multi-band receiver state.
    buffers = NamedTuple{band_keys}(
        # `eltype(ch)` of a `SignalChannel` is the per-chunk `Matrix{T}`; the sample
        # buffer is sized in scalar samples, so unwrap to the scalar element type `T`.
        map((ch, s) -> SampleBuffer(eltype(eltype(ch)), s[3]), measurement_channels, setups),
    )
    initial_state = ReceiverState(band_systems, buffers; num_ants, doppler_estimator)

    # The channel carries whatever `extract` returns. Infer that type without running
    # user code where possible (`promote_op`); for the default this is a concrete
    # `ReceiverDataOfInterest`. Fall back to calling `extract` on the (empty) initial
    # state only if inference can't pin a concrete type.
    payload_type = Base.promote_op(extract, typeof(initial_state))
    isconcretetype(payload_type) || (payload_type = typeof(extract(initial_state)))
    # A small buffer decouples the real-time processing task from its consumers: an
    # unbuffered channel would rendezvous on every `push!`, stalling chunk processing
    # (and backing up the SDR stream) for as long as e.g. a GUI redraw or a JLD2
    # write takes.
    data_channel = Channel{payload_type}(16)

    # `state` and `last_output` are updated every frame. Captured in a reassigned
    # closure variable Julia would lower each to an untyped `Core.Box`, making every
    # field access dynamic; a typed `Ref` captured once (only its contents change)
    # keeps the loop type-stable (see GNSSReceiver.jl PR #90).
    state_ref = Ref(initial_state)
    last_output_ref = Ref(-Inf * 1.0s)
    Base.errormonitor(
        Threads.@spawn try
            while true
                # Take one frame from each band in lock-step; a closed stream ends
                # the run (`InvalidStateException` from an exhausted, closed channel).
                measurements = try
                    map(take!, measurement_channels)
                catch e
                    e isa InvalidStateException && break
                    rethrow(e)
                end
                state_ref[] = process(
                    state_ref[],
                    acq_plans,
                    measurements,
                    band_systems,
                    sampling_freq,
                    interm_freqs;
                    downconvert_and_correlator = resolved_dc,
                    num_ants,
                    acquire_every,
                    acq_pfa,
                    code_lock_cn0_threshold,
                    time_in_lock_before_calculating_pvt,
                    pvt_update_interval,
                    enable_ionospheric_correction,
                    enable_tropospheric_correction,
                    pvt_approximate_year,
                )
                # Emit one payload per PVT update: PVT is recomputed every
                # `pvt_update_interval`, so output is produced at that same rate. Running
                # `extract` and pushing every raw frame would allocate at the frame rate
                # (~kHz), far more often than consumers — GUI, PVT solve, logger,
                # `save_data` — need it.
                state = state_ref[]
                if state.runtime - last_output_ref[] >= pvt_update_interval
                    push!(data_channel, extract(state))
                    last_output_ref[] = state.runtime
                end
            end
            # The stream may end between interval emissions; flush the final state
            # so `last(collect_data(...))` is genuinely the end-of-run snapshot.
            # The `isfinite` guard skips the flush for an empty stream, where
            # nothing was ever emitted.
            state = state_ref[]
            if isfinite(last_output_ref[]) && state.runtime > last_output_ref[]
                push!(data_channel, extract(state))
            end
        finally
            # Close even when a chunk throws: consumers block in `take!` until the
            # channel closes, so leaving it open would hang them after a crash
            # (`errormonitor` only logs the failure).
            close(data_channel)
        end
    )
    data_channel
end
