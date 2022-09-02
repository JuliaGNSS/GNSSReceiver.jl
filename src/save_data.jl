function save_data(data_channel::Channel{T}) where T <: ReceiverDataOfInterest
    data_over_time = Vector{T}()

    consume_channel(data_channel) do data
        push!(data_over_time, data)
    end
    return data_over_time
end