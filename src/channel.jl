import Base.close, Base.put!, Base.close, Base.isempty

struct MatrixSizedChannel{T} <: AbstractChannel{T}
    num_samples::Int
    num_antenna_channels::Int
    channel::Channel{Matrix{T}}
    function MatrixSizedChannel{T}(
        num_samples::Integer,
        num_antenna_channels::Integer,
        sz::Integer = 0,
    ) where {T}
        return new(num_samples, num_antenna_channels, Channel{Matrix{T}}(sz))
    end
end

function MatrixSizedChannel{T}(
    func::Function,
    num_samples::Integer,
    num_antenna_channels::Integer,
    size = 0;
    taskref = nothing,
    spawn = false,
) where {T}
    chnl = MatrixSizedChannel{T}(num_samples, num_antenna_channels, size)
    task = Task(() -> func(chnl))
    task.sticky = !spawn
    bind(chnl, task)
    if spawn
        schedule(task) # start it on (potentially) another thread
    else
        yield(task) # immediately start it, yielding the current thread
    end
    isa(taskref, Ref{Task}) && (taskref[] = task)
    return chnl
end

function Base.put!(c::MatrixSizedChannel, v::AbstractMatrix)
    if size(v, 1) != c.num_samples || size(v, 2) != c.num_antenna_channels
        throw(
            ArgumentError(
                "First dimension must be the number of samples and second dimension number of channels",
            ),
        )
    end
    Base.put!(c.channel, v)
end

Base.bind(c::MatrixSizedChannel, task::Task) = Base.bind(c.channel, task)
Base.take!(c::MatrixSizedChannel) = Base.take!(c.channel)
Base.close(c::MatrixSizedChannel, excp::Exception = Base.closed_exception()) =
    Base.close(c.channel, excp)
Base.isopen(c::MatrixSizedChannel) = Base.isopen(c.channel)
Base.close_chnl_on_taskdone(t::Task, c::MatrixSizedChannel) =
    Base.close_chnl_on_taskdone(t, c.channel)
Base.isready(c::MatrixSizedChannel) = Base.isready(c.channel)
Base.isempty(c::MatrixSizedChannel) = Base.isempty(c.channel)
Base.n_avail(c::MatrixSizedChannel) = Base.n_avail(c.channel)

Base.lock(c::MatrixSizedChannel) = Base.lock(c.channel)
Base.lock(f, c::MatrixSizedChannel) = Base.lock(f, c.channel)
Base.unlock(c::MatrixSizedChannel) = Base.unlock(c.channel)
Base.trylock(c::MatrixSizedChannel) = Base.trylock(c.channel)
Base.wait(c::MatrixSizedChannel) = Base.wait(c.channel)
Base.eltype(c::MatrixSizedChannel) = Base.eltype(c.channel)
Base.show(io::IO, c::MatrixSizedChannel) = Base.show(io, c.channel)
Base.iterate(c::MatrixSizedChannel, state = nothing) = Base.iterate(c.channel, state)

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
function tee(in::MatrixSizedChannel{T}) where {T<:Number}
    out1 = MatrixSizedChannel{T}(in.num_samples, in.num_antenna_channels)
    out2 = MatrixSizedChannel{T}(in.num_samples, in.num_antenna_channels)
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
function rechunk(in::MatrixSizedChannel{T}, chunk_size::Integer) where {T<:Number}
    return spawn_channel_thread(;
        T,
        num_samples = chunk_size,
        in.num_antenna_channels,
    ) do out
        chunk_filled = 0
        chunk_idx = 1
        # We'll alternate between filling up these three chunks, then sending
        # them down the channel.  We have three so that we can have:
        # - One that we're modifying,
        # - One that was sent out to a downstream,
        # - One that is being held by an intermediary
        chunks = [
            Matrix{T}(undef, chunk_size, in.num_antenna_channels),
            Matrix{T}(undef, chunk_size, in.num_antenna_channels),
            Matrix{T}(undef, chunk_size, in.num_antenna_channels),
        ]
        consume_channel(in) do data
            # Make the loop type-stable
            data = view(data, 1:size(data, 1), :)

            # Generate chunks until this data is done
            while !isempty(data)

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
    write_to_file(in::Channel, file_path)

Consume a channel and write to file(s). Multiple channels will
be written to different files. The channel number is appended
to the filename.
"""
function write_to_file(in::MatrixSizedChannel{T}, file_path::String) where {T<:Number}
    Base.errormonitor(
        Threads.@spawn begin
            type_string = string(T)
            streams = [
                open("$file_path$type_string$i.dat", "w") for i = 1:in.num_antenna_channels
            ]
            try
                consume_channel(in) do buffs
                    foreach(eachcol(buffs), streams) do buff, stream
                        write(stream, buff)
                    end
                end
            finally
                close.(streams)
            end
        end
    )
end