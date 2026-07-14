struct SatelliteDataOfInterest{P<:Union{<:Complex,<:AbstractVector{<:Complex}}}
    cn0::typeof(1.0u"dBHz")
    prompt::P
    is_healthy::Bool
end

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
default_downconvert_and_correlator(::Type, max_meas) =
    CPUThreadedDownconvertAndCorrelator()
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

    sat_data_type =
        N == 1 ? SatelliteDataOfInterest{ComplexF64} :
        SatelliteDataOfInterest{SVector{N,ComplexF64}}
    data_channel = PipeChannel{ReceiverDataOfInterest{sat_data_type}}(100)

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
            rs = receiver_state_ref[]
            # Tracking's `get_sat_states` is a `Dictionary` keyed by PRN, so `map`
            # over it keeps those PRN keys and only transforms each satellite into
            # the data of interest — no keys to rebuild, and the result is already
            # the `Dictionary{Int,sat_data_type}` the channel expects.
            #
            # Note: `map` *shares* the source's key `Indices` with this payload
            # (only the values vector is fresh), and the payload outlives this
            # iteration (it queues in `data_channel` and is re-forwarded downstream).
            # This is safe only because `process` mutates the sat set exclusively
            # through the functional `remove_satellite` / `merge_sats`, which build a
            # fresh `Dictionary` rather than mutating the shared `Indices` in place.
            # See the load-bearing comment in `filter_in_lock_sats` (process.jl).
            sat_data = map(get_sat_states(rs.track_state)) do sat_state
                SatelliteDataOfInterest(
                    estimate_cn0(sat_state),
                    get_prompt(get_last_fully_integrated_correlator(sat_state)),
                    is_sat_healthy(rs.receiver_sat_states[1][sat_state.prn].decoder),
                )
            end
            put!(data_channel, ReceiverDataOfInterest(sat_data, rs.pvt, rs.runtime))
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
