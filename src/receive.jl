"""
    SatelliteDataOfInterest

Per-satellite summary emitted for each processed chunk: the estimated carrier-to-noise
density ratio `cn0`, the latest fully integrated `prompt` correlator value (a scalar
for single-antenna, an `SVector` for multi-antenna) and whether the satellite reports
itself `is_healthy`.
"""
struct SatelliteDataOfInterest{P<:Union{<:Complex,<:AbstractVector{<:Complex}}}
    cn0::typeof(1.0u"dBHz")
    prompt::P
    is_healthy::Bool
end

"""
    ReceiverDataOfInterest

Snapshot of the receiver after a processed chunk: `sat_data` maps each tracked PRN to
its [`SatelliteDataOfInterest`](@ref), `pvt` is the current PVT solution and `runtime`
is the elapsed signal time. This is the element type produced by [`receive`](@ref).
"""
struct ReceiverDataOfInterest{S<:SatelliteDataOfInterest}
    sat_data::Dictionary{Int,S}
    pvt::PVTSolution
    runtime::typeof(1.0u"s")
end

# Pick the downconvert-and-correlator backend from the sample element type at
# compile time (the measurement channel's `T`). `Complex{Int16}` inputs get
# Tracking's fast integer backend automatically; every other element type (float
# samples) keeps the general CPU backend. The type is known upfront from the
# channel, so this needs no runtime probing and stays type-stable — and callers
# can still override the whole choice via the `downconvert_and_correlator` keyword.
#
# The integer backend needs `max_meas` (the front-end full-scale, e.g. `2^11` for
# a 12-bit ADC) — the largest `|real|`/`|imag|` any sample takes. It can't be
# inferred from the element type (an `Int16` buffer may be 8-bit, 12-bit or
# full-scale data), and Tracking deliberately gives it no default because
# under-declaring it silently overflows the Int16 carrier wipe and corrupts the
# correlation. So require it explicitly for `Complex{Int16}` and fail loudly when
# it's missing rather than guessing.
default_downconvert_and_correlator(::Type, max_meas) = CPUThreadedDownconvertAndCorrelator()
function default_downconvert_and_correlator(::Type{Complex{Int16}}, max_meas)
    max_meas === nothing && throw(
        ArgumentError(
            "Complex{Int16} measurements use Tracking's fast integer downconvert " *
            "and correlator, which needs `max_meas` — the front end's full-scale " *
            "(largest |real|/|imag| of any sample, e.g. `2^11` for a 12-bit ADC). " *
            "Pass it as `receive(...; max_meas = ...)`, or pass an explicit " *
            "`downconvert_and_correlator`. It has no default because under-" *
            "declaring it silently corrupts the correlation.",
        ),
    )
    Int16ThreadedDownconvertAndCorrelator(max_meas)
end

"""
    default_data_of_interest(receiver_state) -> ReceiverDataOfInterest

Condense a [`ReceiverState`](@ref) into the default per-chunk summary emitted by
[`receive`](@ref): each tracked satellite's CN0, prompt correlator value and health, the
current PVT solution and the runtime.

This is the default `extract` function of [`receive`](@ref). Pass your own
`extract(receiver_state)` to emit a different payload (see the `extract` keyword of
[`receive`](@ref)); it must be read-only and return an immutable value, since it runs
inside the tracking loop on a `ReceiverState` that the next chunk mutates in place.
"""
function default_data_of_interest(receiver_state)
    # Tracking's `get_sat_states` is a `Dictionary` keyed by PRN, so `map` over it keeps
    # those PRN keys and only transforms each satellite into the data of interest — no
    # keys to rebuild, and the result is already the `Dictionary{Int,…}` the payload
    # wants.
    #
    # Note: `map` *shares* the source's key `Indices` with this payload (only the values
    # vector is fresh), and the payload outlives this iteration (it queues in the data
    # channel and is re-forwarded downstream). This is safe only because `process`
    # mutates the sat set exclusively through the functional `remove_satellite` /
    # `merge_sats`, which build a fresh `Dictionary` rather than mutating the shared
    # `Indices` in place. See the load-bearing comment in `filter_in_lock_sats`
    # (process.jl).
    sat_data = map(get_sat_states(receiver_state.track_state)) do sat_state
        SatelliteDataOfInterest(
            estimate_cn0(sat_state),
            get_prompt(get_last_fully_integrated_correlator(sat_state)),
            is_sat_healthy(receiver_state.receiver_sat_states[1][sat_state.prn].decoder),
        )
    end
    ReceiverDataOfInterest(sat_data, receiver_state.pvt, receiver_state.runtime)
end

"""
    receive(measurement_channel, system, sampling_freq; num_ants = NumAnts(1), kwargs...)

Run the full acquire → track → decode → PVT pipeline over the samples arriving on
`measurement_channel` and return a channel of per-chunk results (by default
[`ReceiverDataOfInterest`](@ref); see `extract` below).

Sampled at `sampling_freq`, each chunk read from `measurement_channel` is processed by
[`process`](@ref) in a spawned task; the result of `extract(receiver_state)` is pushed
onto the returned channel. The number of antenna channels in `measurement_channel` must
equal `N` in `num_ants`.

`extract` defaults to [`default_data_of_interest`](@ref), which emits the per-chunk
satellite CN0s, prompts, health and PVT solution as a [`ReceiverDataOfInterest`](@ref).
To collect other quantities (e.g. raw carrier Doppler, code phase or decoded navigation
data) pass your own `extract(receiver_state)`; the returned channel's element type is
inferred from what it returns. It runs inside the tracking loop on a `ReceiverState` that
the next chunk mutates in place, so it must be read-only and return an immutable value.

The downconvert-and-correlator backend is selected from the sample element type `T`:
`Complex{Int16}` inputs use Tracking's fast integer backend, which requires `max_meas`
(the front-end full-scale); every other element type uses the general CPU backend and
ignores `max_meas`. Pass `downconvert_and_correlator` to override this choice.

Acquisition runs every `acquire_every` over `acquisition_num_coherent_code_periods`
coherently integrated code periods (times `acquisition_num_noncoherent_accumulations`),
searching `prns` and detecting at `acquisition_false_alarm_probability`. A satellite is
declared locked once its CN0 exceeds `code_lock_cn0_threshold`, and contributes to the
PVT solution — recomputed every `pvt_update_interval` — after
`time_in_lock_before_calculating_pvt`. `approximate_year` resolves the GPS week-number
rollover for old recordings. Reuse a prior `receiver_state` to continue a run.
"""
function receive(
    measurement_channel::SignalChannel{T},
    system,
    sampling_freq;
    num_ants::NumAnts{N} = NumAnts(1),
    acquisition_num_coherent_code_periods = 4,
    acquisition_num_noncoherent_accumulations = 1,
    bit_edge_search_steps = 1,
    acquire_every = 10u"s",
    receiver_state = ReceiverState(
        T,
        system;
        num_ants,
        num_samples_for_acquisition = round(
            Int,
            get_code_length(system) *
            upreferred(sampling_freq / get_code_frequency(system)) *
            acquisition_num_coherent_code_periods *
            acquisition_num_noncoherent_accumulations,
        ),
    ),
    max_meas = nothing,
    downconvert_and_correlator = default_downconvert_and_correlator(T, max_meas),
    acquisition_false_alarm_probability = 1e-4,
    code_lock_cn0_threshold = get_default_code_lock_cn0_threshold(system),
    time_in_lock_before_calculating_pvt = 2u"s",
    pvt_update_interval = 100u"ms",
    interm_freq = 0.0u"Hz",
    always_buffer = false,
    prns = 1:32,
    approximate_year::Integer = year(now(UTC)),
    extract = default_data_of_interest,
) where {N,T}
    num_channels = num_antenna_channels(measurement_channel)
    num_channels == N ||
        throw(ArgumentError("The number of antenna channels must match num_ants"))

    acq_plan = plan_acquire(
        system,
        float(sampling_freq),
        collect(Int, prns);
        num_coherently_integrated_code_periods = acquisition_num_coherent_code_periods,
        num_noncoherent_accumulations = acquisition_num_noncoherent_accumulations,
        bit_edge_search_steps,
    )

    # The channel carries whatever `extract` returns. Infer that type without running
    # user code where possible (`promote_op`); for the default `extract` this is the
    # concrete `ReceiverDataOfInterest{SatelliteDataOfInterest{…}}`. Fall back to
    # actually calling `extract` on the (empty) initial state only if inference can't pin
    # a concrete type, so the channel element type stays concrete.
    payload_type = Base.promote_op(extract, typeof(receiver_state))
    if !isconcretetype(payload_type)
        payload_type = typeof(extract(receiver_state))
    end
    data_channel = PipeChannel{payload_type}(100)

    # Thread `receiver_state` through the per-chunk loop via a *typed* `Ref` rather
    # than reassigning a captured variable inside the spawned task. A variable
    # captured by that closure and reassigned would be lowered to an untyped
    # `Core.Box` (static type `Any`) — `Core.Box` is not parameterised, so it discards
    # the fact that `receiver_state`'s type never actually changes. Every
    # `receiver_state.…` access in the per-chunk `sat_data` build would then be a
    # dynamic, allocating `getproperty` (~71 KB/chunk — the receiver's dominant
    # allocation). A `Ref{typeof(receiver_state)}` is instead a *typed* cell: it is
    # captured but never reassigned (only its contents are), so it is not boxed; its
    # `[]` reads are type-stable; and `[] =` `convert`s to that type, so if `process`
    # ever returned a different `ReceiverState` type it would error loudly (surfacing
    # the type-instability regression) instead of silently deoptimising.
    receiver_state_ref = Ref(receiver_state)

    # Iterate the lock-free measurement channel directly in a spawned tracking task.
    task = Threads.@spawn begin
        for measurement in measurement_channel
            receiver_state_ref[] = process(
                receiver_state_ref[],
                acq_plan,
                num_channels == N == 1 ? vec(measurement) : measurement,
                system,
                sampling_freq;
                downconvert_and_correlator,
                num_ants,
                acquire_every,
                acquisition_false_alarm_probability,
                code_lock_cn0_threshold,
                time_in_lock_before_calculating_pvt,
                pvt_update_interval,
                interm_freq,
                always_buffer,
                approximate_year,
            )
            # Condense the (in-place-mutated) ReceiverState into an immutable payload to
            # queue. `extract` defaults to `default_data_of_interest`; see its comment for
            # why the built payload is safe to forward downstream.
            put!(data_channel, extract(receiver_state_ref[]))
        end
        close(data_channel)
    end
    # `bind` closes `data_channel` when tracking finishes (propagating any error to
    # the consumer as a `TaskFailedException`) and closes `measurement_channel` on
    # error to stop the upstream producer. `errormonitor` additionally logs a
    # tracking-task failure even if nothing is consuming `data_channel`.
    bind(data_channel, task)
    bind(measurement_channel, task)
    Base.errormonitor(task)
    data_channel
end
