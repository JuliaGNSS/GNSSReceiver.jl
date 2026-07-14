"""
    save_data(data_channel; filename) -> Task

Consume every element of `data_channel` on a spawned task and, once the channel closes,
write the collected vector to `filename` as a JLD2 file under the key `"data_over_time"`.

This is the on-disk counterpart to [`collect_data`](@ref): like it, `save_data` works
with the default [`ReceiverDataOfInterest`](@ref) channel from [`receive`](@ref) as well
as a channel of a custom payload produced via `receive`'s `extract` keyword. Returns the
writer task immediately (the write happens when the producer finishes); `wait` on it if
you need to block until the file is on disk.
"""
function save_data(data_channel::AbstractChannel; filename)
    data_over_time = Vector{eltype(data_channel)}()
    Base.errormonitor(Threads.@spawn begin
        consume_channel(data_channel) do data
            push!(data_over_time, data)
        end
        jldsave(filename; data_over_time)
    end)
end

"""
    collect_data(data_channel) -> Vector

Consume every element of `data_channel`, blocking until the channel closes, and return
them in arrival order as a `Vector` of the channel's element type.

This is the in-memory counterpart to [`save_data`](@ref): use it to gather a whole run
for offline analysis or plotting (e.g. `last(collect_data(data_channel))` for the final
snapshot), and use [`save_data`](@ref) to persist directly to a JLD2 file instead. It
works with the default [`ReceiverDataOfInterest`](@ref) channel from [`receive`](@ref)
as well as a channel of a custom payload produced via `receive`'s `extract` keyword.
"""
function collect_data(data_channel::AbstractChannel)
    data_over_time = Vector{eltype(data_channel)}()
    consume_channel(data_channel) do data
        push!(data_over_time, data)
    end
    return data_over_time
end
