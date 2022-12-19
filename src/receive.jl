struct SatelliteDataOfInterest{P<:Union{<:Complex,<:AbstractVector{<:Complex}}}
    cn0::typeof(1.0dBHz)
    prompt::P
end

struct ReceiverDataOfInterest{S<:SatelliteDataOfInterest}
    sat_data::Dict{Int,Vector{S}}
    pvt::PVTSolution
    runtime::typeof(1ms)
end

function receive(
    measurement_channel::AbstractChannel,
    system,
    sampling_freq;
    num_samples,
    num_ants::NumAnts{N} = NumAnts(1),
    receiver_state = ReceiverState(system, num_ants),
    acquire_every = 10000ms,
    acq_threshold = get_default_acq_threshold(system),
    time_in_lock_before_pvt = 2000ms,
    interm_freq = 0.0u"Hz"
) where {N}
    acq_plan = CoarseFineAcquisitionPlan(system, num_samples, sampling_freq)
    coarse_step = 1 / (num_samples / sampling_freq)
    fine_step = 1 / 12 / (num_samples / sampling_freq)
    fine_doppler_range = -2*coarse_step:fine_step:2*coarse_step
    fast_re_acq_plan = AcquisitionPlan(
        system,
        num_samples,
        sampling_freq,
        dopplers = fine_doppler_range
    )

    sat_data_type =
        N == 1 ? SatelliteDataOfInterest{ComplexF64} :
        SatelliteDataOfInterest{SVector{N,ComplexF64}}
    data_channel = Channel{ReceiverDataOfInterest{sat_data_type}}()

    Base.errormonitor(
        Threads.@spawn begin
            consume_channel(measurement_channel) do measurement
                num_channels = size(measurement, 2)
                num_channels == N || throw(
                    ArgumentError("The number of antenna channels must match num_ants"),
                )
                signal_duration =
                    convert(typeof(1ms), size(measurement, 1) / sampling_freq)
                signal_duration % 1ms == 0ms ||
                    throw(ArgumentError("Signal length must be multiples of 1ms"))
                track_results = process!(
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
                    interm_freq
                )
                sat_data = Dict{Int,Vector{sat_data_type}}(
                    prn => map(
                        x -> SatelliteDataOfInterest(get_cn0(x), get_prompt(x)),
                        res,
                    ) for (prn, res) in track_results
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