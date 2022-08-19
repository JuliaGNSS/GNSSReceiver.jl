struct SatelliteDataOfInterest{N}
    cn0::typeof(1.0dBHz)
    prompts::SVector{N,ComplexF64}
end

struct ReceiverDataOfInterest{N}
    sat_data::Dict{Int, Vector{SatelliteDataOfInterest{N}}}
    pvt::PVTSolution
end

function receive(
    measurement_channel::Channel{T},
    system,
    sampling_freq;
    num_ants::NumAnts{N} = NumAnts(1),
    receiver_state = ReceiverState(),
    push_gui_data_every = 500ms,
    acquire_every = 10000ms,
    acq_threshold = get_default_acq_threshold(system),
    time_in_lock_before_pvt = 2000ms
) where {N, T <:AbstractArray}
    T <: AbstractMatrix && N == 1 && throw(ArgumentError("Measurement channel contains a matrix. Please specify num_ants to the number of used antennas with num_ants = NumAnts(N)"))
    T <: AbstractVector && N > 1 && throw(ArgumentError("Measurement channel contains a vector. In this case number of antennas should be one: num_ants = NumAnts(1) (default)"))
    
    data_channel = Channel{ReceiverDataOfInterest{N}}()
    gui_data_channel = Channel{GUIData}()

    Base.errormonitor(Threads.@spawn begin
        consume_channel(measurement_channel) do measurement
            size(measurement, 2) == N || throw(ArgumentError("The number of antenna channels must match num_ants"))
            signal_duration = convert(typeof(1ms), size(measurement, 1) / sampling_freq)
            signal_duration % 1ms == 0ms || throw(ArgumentError("Signal length must be multiples of 1ms"))
            push_gui_data_every % signal_duration == 0ms || throw(ArgumentError("push_gui_data_every must be multiples of signal duration"))
            receiver_state, track_results = process(
                receiver_state,
                measurement,
                system,
                sampling_freq;
                num_ants,
                acquire_every,
                acq_threshold,
                time_in_lock_before_pvt
            )
            sat_data = Dict(
                prn => map(x -> SatelliteDataOfInterest(get_cn0(x), get_prompt(x)), res)
                for (prn, res) in track_results
            )
            push!(data_channel, ReceiverDataOfInterest(sat_data, receiver_state.pvt))
            if receiver_state.runtime % push_gui_data_every == 0ms
                cn0s = Dict(
                    prn => get_cn0(last(res))
                    for (prn, res) in track_results
                )
                push!(gui_data_channel, GUIData(cn0s, receiver_state.pvt))
            end
        end
        close(data_channel)
        close(gui_data_channel)
    end)    
    data_channel, gui_data_channel
end

function receive(
    streams,
    system,
    sampling_freq;
    measurement = get_default_measurement(streams, sampling_freq),
    receiver_state = ReceiverState(),
    gui_data_channel = nothing,
    push_gui_data_every = 500ms,
    acquire_every = 10000ms,
    acq_threshold = get_default_acq_threshold(system),
    time_in_lock_before_pvt = 2000ms
)
    num_ants = streams isa AbstractVector ? NumAnts(length(streams)) : NumAnts(1)
    signal_duration = convert(typeof(1ms), size(measurement, 1) / sampling_freq)
    signal_duration % 1ms == 0ms || throw(ArgumentError("Signal length must be multiples of 1ms"))
    isnothing(gui_data_channel) || push_gui_data_every % signal_duration == 0ms || throw(ArgumentError("push_gui_data_every must be multiples of signal duration"))
    data = Any[]
    try
        while true
            read_measurement!(streams, measurement)
            receiver_state, track_results = process(
                receiver_state,
                measurement,
                system,
                sampling_freq;
                num_ants,
                acquire_every,
                acq_threshold,
                time_in_lock_before_pvt
            )
            partial_track_results = Dict(
                prn => (cn0s = 10*log10.(linear.(get_cn0.(res)) / Hz), prompts = get_prompt.(res))
                for (prn, res) in track_results
            )
            push!(data, (pvt = receiver_state.pvt, track_results = partial_track_results))
            if receiver_state.runtime % push_gui_data_every == 0ms && !isnothing(gui_data_channel)
                cn0s = Dict(
                    prn => 10*log10(linear(get_cn0(res[end])) / Hz)
                    for (prn, res) in track_results
                )
                push!(gui_data_channel, GUIData(cn0s, receiver_state.pvt))
                sleep(0.0001)
            end
        end
    catch e
        if e isa EOFError
            println("Reached end of file.")
        else
            rethrow(e)
        end
    end
    close.(streams)
    receiver_state, data
end