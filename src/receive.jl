struct SatelliteDataOfInterest{P<:Union{<:Complex,<:AbstractVector{<:Complex}}}
    cn0::typeof(1.0u"dBHz")
    prompt::P
    is_healthy::Bool
end

struct ReceiverDataOfInterest{S<:SatelliteDataOfInterest}
    sat_data::Dict{Int,S}
    pvt::PVTSolution
    runtime::typeof(1.0u"s")
end

function receive(
    measurement_channel::MatrixSizedChannel{T},
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
    downconvert_and_correlator = CPUThreadedDownconvertAndCorrelator(),
    acquisition_false_alarm_probability = 1e-4,
    code_lock_cn0_threshold = get_default_code_lock_cn0_threshold(system),
    time_in_lock_before_calculating_pvt = 2u"s",
    pvt_update_interval = 100u"ms",
    interm_freq = 0.0u"Hz",
    always_buffer = false,
    prns = 1:32,
    approximate_year::Integer = year(now(UTC)),
) where {N,T}
    num_channels = measurement_channel.num_antenna_channels
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
    data_channel = Channel{ReceiverDataOfInterest{sat_data_type}}()

    Base.errormonitor(
        Threads.@spawn begin
            try
                consume_channel(measurement_channel) do measurement
                    receiver_state = process(
                    receiver_state,
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
                sat_data = Dict{Int,sat_data_type}(
                    sat_state.prn => SatelliteDataOfInterest(
                        estimate_cn0(sat_state),
                        get_prompt(get_last_fully_integrated_correlator(sat_state)),
                        is_sat_healthy(
                            receiver_state.receiver_sat_states[1][sat_state.prn].decoder,
                        ),
                    ) for sat_state in get_sat_states(receiver_state.track_state)
                )
                push!(
                    data_channel,
                    ReceiverDataOfInterest(
                        sat_data,
                        receiver_state.pvt,
                        receiver_state.runtime,
                    ),
                )
                end
            finally
                close(data_channel)
            end
        end
    )
    data_channel
end
