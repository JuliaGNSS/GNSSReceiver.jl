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
    acquire_every = 10u"s",
    acq_time::typeof(1u"ms") = 4u"ms",
    receiver_state = ReceiverState(
        T,
        measurement_channel.num_samples,
        acq_time,
        system,
        num_ants,
    ),
    acq_threshold = get_default_acq_threshold(system),
    time_in_lock_before_pvt = 2u"s",
    interm_freq = 0.0u"Hz",
) where {N,T}
    num_channels = measurement_channel.num_antenna_channels
    num_channels == N ||
        throw(ArgumentError("The number of antenna channels must match num_ants"))
    signal_duration = measurement_channel.num_samples / sampling_freq
    isapprox(signal_duration, 4u"ms"; atol = 1u"μs") && signal_duration >= 4u"ms" || throw(
        ArgumentError(
            "Signal length must be close to 1ms and above 4ms. Use $(ceil(4u"ms" * sampling_freq)) samples instead.",
        ),
    )

    acq_num_samples = receiver_state.acq_buffer.size * measurement_channel.num_samples
    acq_plan = CoarseFineAcquisitionPlan(system, acq_num_samples, sampling_freq)
    coarse_step = 1 / (acq_num_samples / sampling_freq)
    fine_step = 1 / 12 / (acq_num_samples / sampling_freq)
    fine_doppler_range = -2*coarse_step:fine_step:2*coarse_step
    fast_re_acq_plan = AcquisitionPlan(
        system,
        acq_num_samples,
        sampling_freq;
        dopplers = fine_doppler_range,
    )

    sat_data_type =
        N == 1 ? SatelliteDataOfInterest{ComplexF64} :
        SatelliteDataOfInterest{SVector{N,ComplexF64}}
    data_channel = Channel{ReceiverDataOfInterest{sat_data_type}}()

    Base.errormonitor(
        Threads.@spawn begin
            consume_channel(measurement_channel) do measurement
                receiver_state, track_results = process(
                    receiver_state,
                    acq_plan,
                    fast_re_acq_plan,
                    num_channels == N == 1 ? vec(measurement) : measurement,
                    system,
                    sampling_freq;
                    num_ants,
                    acquire_every,
                    acq_threshold,
                    time_in_lock_before_pvt,
                    interm_freq,
                )
                sat_data = Dict{Int,sat_data_type}(
                    prn => SatelliteDataOfInterest(get_cn0(res), get_prompt(res), is_sat_healthy(receiver_state.sat_channel_states[prn].decoder)) for
                    (prn, res) in track_results
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
            close(data_channel)
        end
    )
    data_channel
end