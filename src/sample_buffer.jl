"""
    SampleBuffers

A fixed-capacity, allocation-free FIFO ring buffer of signal samples used to hold the
most recent window for satellite (re)acquisition. Supports single-antenna (vector) and
multi-antenna (matrix) sample layouts and tracks an absolute counter of the first
retained sample so acquisition results can be aligned with the live sample stream.
"""
module SampleBuffers

export SampleBuffer,
    buffer, get_samples, isfull, get_first_sample_counter, reset_first_sample_counter, reset

"""
    SampleBuffer{T,A}

Fixed-capacity ring buffer holding up to `max_length` of the most recent samples of
type `T` in the backing store `buffer` (a `Vector` for one antenna, a `Matrix` whose
columns are antennas otherwise). `fifo_buffer` is a scratch store of equal size used to
return wrapped data as a contiguous view without allocating. `current_length` samples
start at `start_index` (wrapping around the end), and `first_sample_counter` is the
absolute index of the oldest retained sample in the original stream.
"""
struct SampleBuffer{T,A}
    buffer::A
    fifo_buffer::A
    max_length::Int
    current_length::Int
    start_index::Int
    first_sample_counter::Int
end

function create_buffer(::Type{T}, max_length::Int, num_antennas::Val{1}) where {T}
    Vector{T}(undef, max_length)
end

function create_buffer(::Type{T}, max_length::Int, num_antennas::Val{N}) where {T,N}
    Matrix{T}(undef, max_length, N)
end

"""
    SampleBuffer(T, max_length, num_antennas = Val(1))

Create an empty [`SampleBuffer`](@ref) holding up to `max_length` samples of type `T`.
`num_antennas` is a `Val`: `Val(1)` backs the buffer with a `Vector`, `Val(N)` with a
`max_length × N` `Matrix`.
"""
function SampleBuffer(::Type{T}, max_length::Int, num_antennas::Val{N} = Val(1)) where {T,N}
    buffer = create_buffer(T, max_length, num_antennas)
    fifo_buffer = similar(buffer)
    SampleBuffer{T,typeof(buffer)}(buffer, fifo_buffer, max_length, 0, 1, 1)
end

"""
    buffer(sample_buffer, samples)

Append `samples` (rows are samples, columns are antennas) to `sample_buffer`, evicting
the oldest samples in FIFO order once `max_length` is exceeded, and return the updated
buffer. Writes into the existing backing store rather than allocating. Empty `samples`
return the buffer unchanged; a mismatch in the non-sample dimensions throws an
`ArgumentError`.
"""
function buffer(sample_buffer::SampleBuffer{T,A}, samples) where {T,A<:AbstractArray}
    num_new_samples, = size(samples)

    if size(samples)[2:end] != size(sample_buffer.buffer)[2:end]
        throw(
            ArgumentError(
                "All dimension of the buffer ($(size(sample_buffer.buffer))) except the first must have the same size as the incoming samples ($(size(samples)))",
            ),
        )
    end

    if num_new_samples == 0
        return sample_buffer
    end

    max_length = sample_buffer.max_length

    if num_new_samples >= max_length
        new_first_sample_counter =
            sample_buffer.first_sample_counter +
            sample_buffer.current_length +
            num_new_samples - max_length
        sample_buffer.buffer[:, :] = @view samples[(end-max_length+1):end, :]
        return SampleBuffer{T,A}(
            sample_buffer.buffer,
            sample_buffer.fifo_buffer,
            max_length,
            max_length,
            1,
            new_first_sample_counter,
        )
    end

    new_length = min(sample_buffer.current_length + num_new_samples, max_length)

    if sample_buffer.current_length + num_new_samples <= max_length
        end_index = sample_buffer.start_index + sample_buffer.current_length - 1
        if end_index + num_new_samples <= max_length
            sample_buffer.buffer[(end_index+1):(end_index+num_new_samples), :] = samples
            new_start_index = sample_buffer.start_index
        else
            first_part_length = max_length - end_index
            sample_buffer.buffer[(end_index+1):max_length, :] =
                @view samples[1:first_part_length, :]
            sample_buffer.buffer[1:(num_new_samples-first_part_length), :] =
                @view samples[(first_part_length+1):end, :]
            new_start_index = sample_buffer.start_index
        end
        new_first_sample_counter = sample_buffer.first_sample_counter
    else
        samples_to_remove = sample_buffer.current_length + num_new_samples - max_length
        new_start_index = mod1(sample_buffer.start_index + samples_to_remove, max_length)
        new_first_sample_counter = sample_buffer.first_sample_counter + samples_to_remove

        end_index =
            mod1(sample_buffer.start_index + sample_buffer.current_length - 1, max_length)

        if end_index + num_new_samples <= max_length
            sample_buffer.buffer[(end_index+1):(end_index+num_new_samples), :] = samples
        else
            first_part_length = max_length - end_index
            sample_buffer.buffer[(end_index+1):max_length, :] =
                @view samples[1:first_part_length, :]
            sample_buffer.buffer[1:(num_new_samples-first_part_length), :] =
                @view samples[(first_part_length+1):end, :]
        end
    end

    SampleBuffer{T,A}(
        sample_buffer.buffer,
        sample_buffer.fifo_buffer,
        max_length,
        new_length,
        new_start_index,
        new_first_sample_counter,
    )
end

"""
    get_samples(sample_buffer)

Return a view of the buffered samples in chronological order (oldest first). When the
retained data wraps around the end of the backing store it is first copied into the
scratch `fifo_buffer` so the result is always a contiguous view and no allocation is
made.
"""
function get_samples(sample_buffer::SampleBuffer)
    if sample_buffer.current_length == 0
        return _empty_view(sample_buffer.fifo_buffer)
    end

    end_index = sample_buffer.start_index + sample_buffer.current_length - 1
    if end_index <= sample_buffer.max_length
        # Data doesn't wrap around
        @inbounds return _slice_view(
            sample_buffer.buffer,
            sample_buffer.start_index:end_index,
        )
    else
        # Data wraps around - copy to fifo_buffer to avoid vcat allocation
        first_part_length = sample_buffer.max_length - sample_buffer.start_index + 1
        second_part_length = sample_buffer.current_length - first_part_length

        # Copy first part (from start_index to end of buffer)
        sample_buffer.fifo_buffer[1:first_part_length, :] = @view sample_buffer.buffer[
            sample_buffer.start_index:sample_buffer.max_length,
            :,
        ]

        # Copy second part (from beginning of buffer)
        sample_buffer.fifo_buffer[(first_part_length+1):sample_buffer.current_length, :] =
            @view sample_buffer.buffer[1:second_part_length, :]

        @inbounds return _slice_view(
            sample_buffer.fifo_buffer,
            1:sample_buffer.current_length,
        )
    end
end

# Helper functions to dispatch on buffer type
_empty_view(buffer::Vector) = view(buffer, 1:0)
_empty_view(buffer::Matrix) = view(buffer, 1:0, :)

_slice_view(buffer::Vector, range) = view(buffer, range)
_slice_view(buffer::Matrix, range) = view(buffer, range, :)

"""
    isfull(sample_buffer)

Return `true` once the buffer holds `max_length` samples.
"""
function isfull(sample_buffer::SampleBuffer)
    sample_buffer.current_length == sample_buffer.max_length
end

"""
    get_first_sample_counter(sample_buffer)

Return the absolute stream index of the oldest sample currently retained in the buffer.
"""
function get_first_sample_counter(sample_buffer::SampleBuffer)
    sample_buffer.first_sample_counter
end

"""
    reset(sample_buffer)

Return an emptied copy of `sample_buffer` (length zero) that keeps the backing store and
advances `first_sample_counter` past the discarded samples, so the absolute counter
stays consistent with the stream.
"""
function reset(sample_buffer::SampleBuffer{T}) where {T}
    SampleBuffer{T,typeof(sample_buffer.buffer)}(
        sample_buffer.buffer,
        sample_buffer.fifo_buffer,
        sample_buffer.max_length,
        0,
        1,
        sample_buffer.first_sample_counter + sample_buffer.current_length,
    )
end

"""
    reset_first_sample_counter(sample_buffer)

Return a copy of `sample_buffer` with `first_sample_counter` rewound to `1`, keeping the
retained samples and all other state, to restart absolute sample counting from zero.
"""
function reset_first_sample_counter(sample_buffer::SampleBuffer{T}) where {T}
    SampleBuffer{T,typeof(sample_buffer.buffer)}(
        sample_buffer.buffer,
        sample_buffer.fifo_buffer,
        sample_buffer.max_length,
        sample_buffer.current_length,
        sample_buffer.start_index,
        1,
    )
end

end
