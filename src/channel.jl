import Base.close, Base.put!, Base.isempty

"""
    SignalChannel{T,N} <: AbstractChannel{T}

A specialized channel type that enforces matrix dimensions for multi-channel
signal data. The number of antenna channels `N` is a type parameter, enabling
compile-time specialization for zero-allocation performance in tight loops.

Data is always stored as a `Matrix{T}` with dimensions `(num_samples, N)`. For
single-channel signals (`N = 1`) this is a column vector represented as an
`(num_samples, 1)` matrix.

Uses a lock-free [`PipeChannel`](@ref) internally, which eliminates the
allocations Julia's `Channel` incurs in the hot path.

**Thread Safety**: Exactly ONE producer thread may call `put!` and exactly ONE
consumer thread may call `take!`. Multiple producers or consumers cause data races.

# Type Parameters
- `T`: Element type (e.g. `ComplexF32`, `Complex{Int16}`)
- `N`: Number of antenna channels (compile-time constant)
"""
struct SignalChannel{T,N} <: AbstractChannel{T}
    num_samples::Int
    channel::PipeChannel{Matrix{T}}
    function SignalChannel{T,N}(num_samples::Integer, sz::Integer = 16) where {T,N}
        return new{T,N}(num_samples, PipeChannel{Matrix{T}}(sz))
    end
end

# Convenience constructor: SignalChannel{T}(num_samples) defaults to N=1
SignalChannel{T}(num_samples::Integer, sz::Integer = 16) where {T} =
    SignalChannel{T,1}(num_samples, sz)

# Accessor for number of antenna channels (from type parameter)
num_antenna_channels(::SignalChannel{T,N}) where {T,N} = N

"""
    SignalChannel{T,N}(func::Function, num_samples, size=16; taskref=nothing, spawn=false)

Construct a `SignalChannel{T,N}` and execute `func(channel)` in a task, similar to
`Channel(func)`. If `spawn` is `true` the task is scheduled on any thread;
otherwise the current thread yields to it immediately.
"""
function SignalChannel{T,N}(
    func::Function,
    num_samples::Integer,
    size = 16;
    taskref = nothing,
    spawn = false,
) where {T,N}
    chnl = SignalChannel{T,N}(num_samples, size)
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

# Convenience: SignalChannel{T}(func, num_samples, size) defaults to N=1
SignalChannel{T}(
    func::Function,
    num_samples::Integer,
    size = 16;
    taskref = nothing,
    spawn = false,
) where {T} = SignalChannel{T,1}(func, num_samples, size; taskref, spawn)

"""
    put!(c::SignalChannel{T,N}, v::Matrix{T})

Put a matrix into the channel, validating that its dimensions match the channel's
`num_samples` and `N`.
"""
function Base.put!(c::SignalChannel{T,N}, v::Matrix{T}) where {T,N}
    if size(v, 1) != c.num_samples || size(v, 2) != N
        throw(
            ArgumentError(
                "Matrix dimensions $(size(v)) do not match expected ($(c.num_samples), $N)",
            ),
        )
    end
    Base.put!(c.channel, v)
end

# The channel stores `Matrix{T}` (a producer must hand ownership of a fresh, densely
# stored buffer — the lock-free channel is buffered, so it may still be in flight).
# Reject other `AbstractMatrix{T}` (views, reshapes of non-Arrays, adjoints) with a
# clear error rather than a bare `MethodError`.
function Base.put!(c::SignalChannel{T,N}, v::AbstractMatrix{T}) where {T,N}
    throw(
        ArgumentError(
            "SignalChannel stores Matrix{$T}; got $(typeof(v)). " *
            "Materialise it with Matrix{$T}(your_matrix).",
        ),
    )
end

"""
    put!(c::SignalChannel{T,N}, values::AbstractVector{<:Matrix{T}})

Add multiple matrices to the channel in a single batch operation, using the
underlying `PipeChannel`'s batch `put!` to reduce atomic overhead.
"""
function Base.put!(
    c::SignalChannel{T,N},
    values::AbstractVector{<:Matrix{T}},
) where {T,N}
    for (i, v) in enumerate(values)
        if size(v, 1) != c.num_samples || size(v, 2) != N
            throw(
                ArgumentError(
                    "Matrix $i dimensions $(size(v)) do not match expected ($(c.num_samples), $N)",
                ),
            )
        end
    end
    Base.put!(c.channel, values)
end

# Delegate Base methods to the underlying channel
Base.bind(c::SignalChannel, task::Task) = Base.bind(c.channel, task)
Base.take!(c::SignalChannel) = Base.take!(c.channel)
Base.take!(c::SignalChannel, n::Integer) = Base.take!(c.channel, n)
Base.take!(c::SignalChannel{T,N}, output::AbstractVector{<:Matrix{T}}) where {T,N} =
    Base.take!(c.channel, output)
Base.close(c::SignalChannel, excp::Exception = Base.closed_exception()) =
    Base.close(c.channel, excp)
Base.isopen(c::SignalChannel) = Base.isopen(c.channel)
Base.isready(c::SignalChannel) = Base.isready(c.channel)
Base.isempty(c::SignalChannel) = Base.isempty(c.channel)
Base.n_avail(c::SignalChannel) = Base.n_avail(c.channel)
isfull(c::SignalChannel) = isfull(c.channel)
Base.wait(c::SignalChannel) = Base.wait(c.channel)
Base.eltype(::Type{SignalChannel{T,N}}) where {T,N} = Matrix{T}

# Iterator support: allows `for buffer in channel` syntax. The @inline annotation
# is critical to avoid heap allocation of the (value, state) tuple.
@inline Base.iterate(c::SignalChannel, state = nothing) = Base.iterate(c.channel, state)
Base.IteratorSize(::Type{<:SignalChannel}) = Base.SizeUnknown()

# `similar` is used by `tee`/`membuffer` to create matching output channels.
Base.similar(c::SignalChannel{T,N}, size::Int = 16) where {T,N} =
    SignalChannel{T,N}(c.num_samples, size)
Base.similar(c::PipeChannel{T}, size::Int = 16) where {T} = PipeChannel{T}(size)

"""
    consume_channel(f::Function, c::AbstractChannel, args...)

Consume the given channel, calling `f(data, args...)` for each element taken from
it. Returns when the channel closes.
"""
function consume_channel(f::Function, c::AbstractChannel, args...)
    for data in c
        f(data, args...)
    end
end

"""
    tee(in::AbstractChannel, channel_size::Integer=16)

Split a channel into two synchronized outputs; both receive identical copies of
the data. Returns a tuple `(out1, out2)` of channels with the same type as `in`.
"""
function tee(in::AbstractChannel, channel_size::Integer = 16)
    out1 = similar(in, channel_size)
    out2 = similar(in, channel_size)
    task = Threads.@spawn begin
        for data in in
            put!(out1, data)
            put!(out2, data)
        end
        close(out1)
        close(out2)
    end
    bind(out1, task)
    bind(out2, task)
    bind(in, task)
    return (out1, out2)
end

# ============================================================================
# Rechunking
# ============================================================================

"""
    RechunkState{T,N}

Mutable state for rechunking operations, holding pre-allocated buffers and the
current fill position. Enables zero-allocation rechunking in hot loops. `N`
(number of antenna channels) is a type parameter so the per-channel copy loop
unrolls at compile time.
"""
mutable struct RechunkState{T,N}
    output_chunk_size::Int
    buffer_pool::Vector{Matrix{T}}
    buffer_idx::Int
    chunk_filled::Int
    output_vector::Vector{Matrix{T}}
    output_count::Int

    function RechunkState{T,N}(
        output_chunk_size::Integer,
        num_buffers::Integer,
        max_outputs_per_input::Integer,
    ) where {T,N}
        buffer_pool =
            [Matrix{T}(undef, output_chunk_size, N) for _ = 1:num_buffers]
        output_vector = Vector{Matrix{T}}(undef, max_outputs_per_input)
        return new{T,N}(output_chunk_size, buffer_pool, 1, 0, output_vector, 0)
    end
end

# Per-channel copy using ntuple for compile-time unrolling (zero allocations)
@inline function copy_channels!(
    output::AbstractMatrix{T},
    input::AbstractMatrix{T},
    dst_offset::Int,
    src_offset::Int,
    nsamples::Int,
    ::Val{N},
) where {T,N}
    ntuple(Val(N)) do ch
        @inbounds copyto!(
            view(output, :, ch),
            dst_offset,
            view(input, :, ch),
            src_offset,
            nsamples,
        )
    end
    nothing
end

# Core rechunk step: copy samples and check for chunk completion.
# Returns (completed_buffer_or_nothing, samples_consumed).
@inline function rechunk_step!(
    state::RechunkState{T,N},
    input::AbstractMatrix{T},
    data_offset::Int,
    data_remaining::Int,
) where {T,N}
    samples_taken = min(data_remaining, state.output_chunk_size - state.chunk_filled)

    # Per-channel copy with compile-time unrolling
    output_buff = state.buffer_pool[state.buffer_idx]
    copy_channels!(
        output_buff,
        input,
        state.chunk_filled + 1,
        data_offset + 1,
        samples_taken,
        Val(N),
    )

    state.chunk_filled += samples_taken

    # Check if we completed a chunk
    if state.chunk_filled >= state.output_chunk_size
        state.buffer_idx = mod1(state.buffer_idx + 1, length(state.buffer_pool))
        state.chunk_filled = 0
        return (output_buff, samples_taken)
    end

    return (nothing, samples_taken)
end

# Process a single input matrix and append completed buffers to output_vector.
@inline function rechunk_one!(
    state::RechunkState{T,N},
    input::AbstractMatrix{T},
    output_count::Int,
) where {T,N}
    # Zero-copy passthrough: if input exactly matches output size and no partial
    # data is buffered, include input directly without copying.
    if state.chunk_filled == 0 && size(input, 1) == state.output_chunk_size
        output_count += 1
        state.output_vector[output_count] = input
        return output_count
    end

    data_offset = 0
    data_remaining = size(input, 1)

    while data_remaining > 0
        completed_buffer, samples_taken =
            rechunk_step!(state, input, data_offset, data_remaining)
        data_offset += samples_taken
        data_remaining -= samples_taken

        if completed_buffer !== nothing
            output_count += 1
            state.output_vector[output_count] = completed_buffer
        end
    end

    return output_count
end

"""
    rechunk!(state::RechunkState{T,N}, input::Matrix{T})

Process an input buffer through the rechunk state, returning a view of completed
output buffers (valid until the next call). Uses per-channel `copyto!` with
compile-time loop unrolling for zero-allocation performance.

**Zero-copy passthrough**: when `input` exactly matches the output chunk size and
no partial data is buffered, `input` is passed through without copying, so the
caller must not reuse it while the output is still in use.
"""
@inline function rechunk!(
    state::RechunkState{T,N},
    input::Matrix{T},
) where {T,N}
    output_count = rechunk_one!(state, input, 0)
    state.output_count = output_count
    return view(state.output_vector, 1:output_count)
end

"""
    rechunk(in::SignalChannel{T,N}, chunk_size::Integer, channel_size=16)

Convert a stream of chunks with one size to a stream of chunks with a different
size, preserving the antenna-channel count `N`. Uses batch `put!` on the output
channel for improved throughput.
"""
function rechunk(
    in::SignalChannel{T,N},
    chunk_size::Integer,
    channel_size = 16,
) where {T<:Number,N}
    out = SignalChannel{T,N}(chunk_size, channel_size)

    # Estimate max outputs per input: input can complete partial + produce full chunks.
    max_outputs = cld(in.num_samples, chunk_size) + 1

    # channel_size buffers may sit in the output channel, max_outputs in the output
    # vector, plus one being written and one being read by the consumer.
    num_buffers = channel_size + max_outputs + 2

    task = Threads.@spawn _rechunk_task(T, Val(N), in, out, chunk_size, num_buffers, max_outputs)
    bind(out, task)
    bind(in, task)  # Propagate errors upstream
    return out
end

# Inner task function with compile-time N for zero-allocation rechunking
function _rechunk_task(::Type{T}, ::Val{N}, in, out, chunk_size, num_buffers, max_outputs) where {T,N}
    state = RechunkState{T,N}(chunk_size, num_buffers, max_outputs)
    for data in in
        outputs = rechunk!(state, data)
        if !isempty(outputs)
            put!(out.channel, outputs)
        end
    end
    close(out)
end

# ============================================================================
# Stream helpers
# ============================================================================

"""
    spawn_signal_channel_thread(f::Function; T=ComplexF32, num_samples, num_antenna_channels=1, buffers_in_flight=16)

Invoke `f(out_channel)` on a separate thread, closing `out_channel` when `f`
finishes. Returns the `SignalChannel`.
"""
function spawn_signal_channel_thread(
    f::Function;
    T::DataType = ComplexF32,
    num_samples,
    num_antenna_channels::Integer = 1,
    buffers_in_flight::Int = 16,
)
    _spawn_signal_channel_thread(f, T, Val(num_antenna_channels), num_samples, buffers_in_flight)
end

function _spawn_signal_channel_thread(
    f::Function,
    ::Type{T},
    ::Val{N},
    num_samples,
    buffers_in_flight,
) where {T,N}
    SignalChannel{T,N}(num_samples, buffers_in_flight; spawn = true) do out
        f(out)
    end
end

"""
    membuffer(in::AbstractChannel, max_size::Int = 16)

Provide buffering for realtime applications by forwarding `in` into a new channel
that can hold up to `max_size` items in flight.
"""
function membuffer(in::AbstractChannel, max_size::Int = 16)
    out = similar(in, max_size)
    task = Threads.@spawn begin
        for data in in
            put!(out, data)
        end
        close(out)
    end
    bind(out, task)
    bind(in, task)
    return out
end

# ============================================================================
# File I/O
# ============================================================================

"""
    write_to_file(in::SignalChannel{T,N}, file_path::String)

Consume a channel and write each antenna channel to its own file. Files are named
`{file_path}{Type}{channel_number}.dat`. Returns the writer task.
"""
function write_to_file(in::SignalChannel{T,N}, file_path::String) where {T<:Number,N}
    task = Threads.@spawn begin
        type_string = string(T)
        streams = [open("$file_path$type_string$i.dat", "w") for i = 1:N]
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
    Base.errormonitor(task)
    return task
end

"""
    read_from_file(file_path::String, num_samples, num_antenna_channels; T=ComplexF32)

Read data from files (the inverse of [`write_to_file`](@ref)) and stream it through
a `SignalChannel`. Files are expected to follow the
`{file_path}{Type}{channel_number}.dat` naming pattern.
"""
function read_from_file(
    file_path::String,
    num_samples::Integer,
    num_antenna_channels::Integer;
    T::Type = ComplexF32,
)
    type_string = string(T)

    for i = 1:num_antenna_channels
        filepath = "$file_path$type_string$i.dat"
        isfile(filepath) || error("File not found: $filepath")
    end

    return spawn_signal_channel_thread(; T, num_samples, num_antenna_channels) do out
        streams = [open("$file_path$type_string$i.dat", "r") for i = 1:num_antenna_channels]
        try
            while !any(eof, streams)
                buff = Matrix{T}(undef, num_samples, num_antenna_channels)
                all_complete = true
                for (idx, stream) in enumerate(streams)
                    column = view(buff, :, idx)
                    bytes_read = readbytes!(stream, reinterpret(UInt8, column))
                    samples_read = bytes_read ÷ sizeof(T)
                    if samples_read < num_samples
                        all_complete = false
                        break
                    end
                end
                all_complete && put!(out, buff)
            end
        finally
            close.(streams)
        end
    end
end
