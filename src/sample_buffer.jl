module SampleBuffers

export SampleBuffer,
    buffer, get_samples, isfull, get_first_sample_counter, reset_first_sample_counter

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

function SampleBuffer(::Type{T}, max_length::Int, num_antennas::Val{N} = Val(1)) where {T,N}
    buffer = create_buffer(T, max_length, num_antennas)
    fifo_buffer = similar(buffer)
    SampleBuffer{T,typeof(buffer)}(buffer, fifo_buffer, max_length, 0, 1, 1)
end

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
        sample_buffer.buffer[:, :] = @view samples[end-max_length+1:end, :]
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
            sample_buffer.buffer[end_index+1:end_index+num_new_samples, :] = samples
            new_start_index = sample_buffer.start_index
        else
            first_part_length = max_length - end_index
            sample_buffer.buffer[end_index+1:max_length, :] =
                @view samples[1:first_part_length, :]
            sample_buffer.buffer[1:num_new_samples-first_part_length, :] =
                @view samples[first_part_length+1:end, :]
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
            sample_buffer.buffer[end_index+1:end_index+num_new_samples, :] = samples
        else
            first_part_length = max_length - end_index
            sample_buffer.buffer[end_index+1:max_length, :] =
                @view samples[1:first_part_length, :]
            sample_buffer.buffer[1:num_new_samples-first_part_length, :] =
                @view samples[first_part_length+1:end, :]
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
        sample_buffer.fifo_buffer[first_part_length+1:sample_buffer.current_length, :] =
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

function isfull(sample_buffer::SampleBuffer)
    sample_buffer.current_length == sample_buffer.max_length
end

function get_first_sample_counter(sample_buffer::SampleBuffer)
    sample_buffer.first_sample_counter
end

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