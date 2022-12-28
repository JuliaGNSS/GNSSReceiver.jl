struct AcquisitionBuffer{T}
    buffer::Vector{T}
    size::Int
    current_index::Int
end

function AcquisitionBuffer(T::Type, num_samples, size)
    AcquisitionBuffer(Vector{T}(undef, num_samples * size), size, 0)
end

function put(acq_buffer::AcquisitionBuffer, measurement)
    current_index = mod(acq_buffer.current_index, acq_buffer.size)
    num_samples = size(measurement, 1)
    acq_buffer.buffer[current_index*num_samples+1:(current_index+1)*num_samples] =
        view(measurement, :, 1)
    AcquisitionBuffer(acq_buffer.buffer, acq_buffer.size, current_index + 1)
end

function isfull(acq_buffer)
    acq_buffer.size == acq_buffer.current_index
end

get_buffer(acq_buffer::AcquisitionBuffer) = acq_buffer.buffer