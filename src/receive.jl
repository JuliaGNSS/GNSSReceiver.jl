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
    measurement_channel::Channel{T},
    system,
    sampling_freq;
    num_samples,
    num_ants::NumAnts{N} = NumAnts(1),
    receiver_state = ReceiverState(),
    acquire_every = 10000ms,
    acq_threshold = get_default_acq_threshold(system),
    time_in_lock_before_pvt = 2000ms,
) where {N,T<:AbstractArray}
    T <: AbstractMatrix &&
        N == 1 &&
        throw(
            ArgumentError(
                "Measurement channel contains a matrix. Please specify num_ants to the number of used antennas with num_ants = NumAnts(N)",
            ),
        )
    T <: AbstractVector &&
        N > 1 &&
        throw(
            ArgumentError(
                "Measurement channel contains a vector. In this case number of antennas should be one: num_ants = NumAnts(1) (default)",
            ),
        )

    acq_plan = CoarseFineAcquisitionPlan(system, num_samples, sampling_freq)

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
                receiver_state, track_results = process(
                    receiver_state,
                    acq_plan,
                    num_channels == N == 1 ? vec(measurement) : measurement,
                    system,
                    sampling_freq;
                    num_ants,
                    acquire_every,
                    acq_threshold,
                    time_in_lock_before_pvt,
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