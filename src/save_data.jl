function save_data(data_channel::Channel{T}) where T <: ReceiverDataOfInterest
    data_over_time = Vector{T}()
    Base.errormonitor(Threads.@spawn begin
        consume_channel(data_channel) do data
            push!(data_over_time, data)
        end
        jldsave("data.jld2"; data_over_time)
    end)
end