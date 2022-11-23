function save_data(data_channel::Channel{T}; filename) where {T<:ReceiverDataOfInterest}
    data_over_time = Vector{T}()
    Base.errormonitor(Threads.@spawn begin
        consume_channel(data_channel) do data
            push!(data_over_time, data)
        end
        jldsave(filename; data_over_time)
    end)
end