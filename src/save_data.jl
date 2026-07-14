"""
    save_data(data_channel; filename)

Consume every [`ReceiverDataOfInterest`](@ref) from `data_channel` on a spawned task
and, once the channel closes, write the collected vector to `filename` as a JLD2 file
under the key `"data_over_time"`. Returns immediately; the write happens when the
producer finishes.
"""
function save_data(
    data_channel::AbstractChannel{T};
    filename,
) where {T<:ReceiverDataOfInterest}
    data_over_time = Vector{T}()
    Base.errormonitor(Threads.@spawn begin
        consume_channel(data_channel) do data
            push!(data_over_time, data)
        end
        jldsave(filename; data_over_time)
    end)
end

"""
    collect_data(data_channel) -> Vector{ReceiverDataOfInterest}

Consume every [`ReceiverDataOfInterest`](@ref) from `data_channel`, blocking until the
channel closes, and return them in arrival order.

This is the in-memory counterpart to [`save_data`](@ref): use it to gather a whole run
for offline analysis or plotting (e.g. `last(collect_data(data_channel))` for the final
snapshot), and use [`save_data`](@ref) to persist directly to a JLD2 file instead.
"""
function collect_data(
    data_channel::AbstractChannel{T},
) where {T<:ReceiverDataOfInterest}
    data_over_time = Vector{T}()
    consume_channel(data_channel) do data
        push!(data_over_time, data)
    end
    return data_over_time
end
