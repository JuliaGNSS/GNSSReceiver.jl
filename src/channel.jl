"""
    consume_channel(f::Function, c::Channel, args...)

Consumes the given channel, calling `f(data, args...)` where `data` is what is
taken from the given channel.  Returns when the channel closes.
"""
function consume_channel(f::Function, c::AbstractChannel, args...)
    while !isempty(c) || isopen(c)
        local data
        try
            data = take!(c)
        catch e
            if isa(e, InvalidStateException)
                continue
            end
            rethrow(e)
        end
        f(data, args...)
    end
end

"""
    tee(in::Channel)

Returns two channels that synchronously output what comes in from `in`.
"""
function tee(in::Channel{T}) where {T}
    out1 = Channel{T}()
    out2 = Channel{T}()
    Base.errormonitor(Threads.@spawn begin
        consume_channel(in) do data
            put!(out1, data)
            put!(out2, data)
        end
        close(out1)
        close(out2)
    end)
    return (out1, out2)
end

"""
    rechunk(in::Channel, chunk_size::Int)

Converts a stream of chunks with size A to a stream of chunks with size B.
"""
function rechunk(in::Channel{Matrix{T}}, chunk_size::Integer) where {T<:Number}
    return spawn_channel_thread(; T) do out
        chunk_filled = 0
        chunk_idx = 1
        # We'll alternate between filling up these three chunks, then sending
        # them down the channel.  We have three so that we can have:
        # - One that we're modifying,
        # - One that was sent out to a downstream,
        # - One that is being held by an intermediary
        chunks = [Matrix{T}(undef, 0, 0), Matrix{T}(undef, 0, 0), Matrix{T}(undef, 0, 0)]
        function make_chunks!(num_channels)
            if size(chunks[1], 2) != num_channels
                for idx in eachindex(chunks)
                    chunks[idx] = Matrix{T}(undef, chunk_size, num_channels)
                end
                global chunk_filled = 0
                global chunk_idx = 1
            end
        end
        consume_channel(in) do data
            # Make the loop type-stable
            data = view(data, 1:size(data, 1), :)

            # Generate chunks until this data is done
            while !isempty(data)
                make_chunks!(size(data, 2))

                # How many samples are we going to consume from this buffer?
                samples_wanted = (chunk_size - chunk_filled)
                samples_taken = min(size(data, 1), samples_wanted)

                # Copy as much of `data` as we can into `chunks`
                chunks[chunk_idx][chunk_filled+1:chunk_filled+samples_taken, :] =
                    data[1:samples_taken, :]
                chunk_filled += samples_taken

                # Move our view of `data` forward:
                data = view(data, samples_taken+1:size(data, 1), :)

                # If we filled the chunk completely, then send it off and flip `chunk_idx`:
                if chunk_filled >= chunk_size
                    put!(out, chunks[chunk_idx])
                    chunk_idx = mod1(chunk_idx + 1, length(chunks))
                    chunk_filled = 0
                end
            end
        end
    end
end

"""
    vectorize_data(in::Channel)

Returns channels with vectorized data.
"""
function vectorize_data(in::Channel{<:AbstractMatrix{T}}) where {T}
    vec_c = Channel{Vector{T}}()
    Base.errormonitor(Threads.@spawn begin
        consume_channel(in) do buff
            put!(vec_c, vec(buff))
        end
        close(vec_c)
    end)
    return vec_c
end

"""
    write_to_file(in::Channel, file_path)

Consume a channel and write to file(s). Multiple channels will
be written to different files. The channel number is appended
to the filename.
"""
function write_to_file(in::Channel{Matrix{T}}, file_path::String; compress=false, compression_rati) where {T<:Number}
    type_string = string(T)
    try
        consume_channel(in) do buffs
            if compress
                streams = if length(streams) != size(buffs, 2)
                    [GZip.open("$file_path$type_string$i.dat.gz", "w") for i = 1:size(buffs, 2)]
                else
                    GZipStream[]
                end

                foreach(eachcol(buffs), streams) do buff, stream
                    write(stream, buff)
                end
            else
                streams = if length(streams) != size(buffs, 2)
                    [open("$file_path$type_string$i.dat", "w") for i = 1:size(buffs, 2)]
                else
                    IOStream[]
                end

                foreach(eachcol(buffs), streams) do buff, stream
                    write(stream, buff)
                end
            end
        end
    finally
        close.(streams)
    end
end