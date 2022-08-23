# Helper for turning a matrix into a tuple of views, for use with the SoapySDR API.
split_matrix(m::Matrix{ComplexF32}) = tuple(collect(view(m, :, idx) for idx in 1:size(m,2))...)

"""
    generate_stream(gen_buff!::Function, buff_size, num_channels)
Returns a `Channel` that allows multiple buffers to be 
"""
function generate_stream(gen_buff!::Function, buff_size::Integer, num_channels::Integer;
                         wrapper::Function = (f) -> f(),
                         buffers_in_flight::Integer = 1)
    c = Channel{Matrix{ComplexF32}}(buffers_in_flight)

    Base.errormonitor(Threads.@spawn begin
        buff_idx = 1
        try
            wrapper() do
                buff = Matrix{ComplexF32}(undef, buff_size, num_channels)

                # Keep on generating buffers until `gen_buff!()` returns `false`.
                while gen_buff!(buff)
                    put!(c, copy(buff))
                end
                # Put the last one too
                put!(c, buff)
            end
        finally
            close(c)
        end
    end)
    return c
end
function generate_stream(f::Function, s::SoapySDR.Stream; kwargs...)
    return generate_stream(f, s.mtu, s.nchannels; kwargs...)
end

"""
    stream_data(s_rx::SoapySDR.Stream, num_samples::Integer)
Returns a `Channel` which will yield buffers of data to be processed of size `s_rx.mtu`.
Starts an asynchronous task that does the reading from the stream, until the requested
number of samples are read.
"""
function stream_data(s_rx::SoapySDR.Stream, num_samples::Integer;
                     leadin_buffers::Integer = 16,
                     kwargs...)
    # Wrapper to activate/deactivate `s_rx`
    wrapper = (f) -> begin
        SoapySDR.activate!(s_rx) do
            # Let the stream come online for a bit
            buff = Matrix{ComplexF32}(undef, s_rx.mtu, s_rx.nchannels)
            for _ in 1:leadin_buffers
                read!(s_rx, split_matrix(buff); timeout=1u"s")
            end

            # Invoke the rest of `generate_stream()`
            f()
        end
    end

    # Read streams until we exhaust the number of buffs
    buff_idx = 0
    return generate_stream(s_rx.mtu, s_rx.nchannels; wrapper, kwargs...) do buff
        read!(s_rx, split_matrix(buff); timeout=1u"s")
        buff_idx += 1
        return buff_idx*s_rx.mtu < num_samples
    end
end