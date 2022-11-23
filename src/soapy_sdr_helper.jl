# Helper for turning a matrix into a tuple of views, for use with the SoapySDR API.
function split_matrix(m::Matrix{T}) where {T<:Number}
    return tuple(collect(view(m, :, idx) for idx = 1:size(m, 2))...)
end

"""
    spawn_channel_thread(f::Function)

Use this convenience wrapper to invoke `f(out_channel)` on a separate thread, closing
`out_channel` when `f()` finishes.
"""
function spawn_channel_thread(
    f::Function;
    T::DataType = ComplexF32,
    buffers_in_flight::Int = 0,
)
    out = Channel{Matrix{T}}(buffers_in_flight)
    Base.errormonitor(Threads.@spawn begin
        try
            f(out)
        finally
            close(out)
        end
    end)
    return out
end

"""
    membuffer(in, max_size = 16)

Provide some buffering for realtime applications.
"""
function membuffer(in::Channel{Matrix{T}}, max_size::Int = 16) where {T<:Number}
    spawn_channel_thread(; T, buffers_in_flight = max_size) do out
        consume_channel(in) do buff
            put!(out, buff)
        end
    end
end

"""
    generate_stream(gen_buff!::Function, buff_size, num_channels)

Returns a `Channel` that allows multiple buffers to be
"""
function generate_stream(
    gen_buff!::Function,
    buff_size::Integer,
    num_channels::Integer;
    wrapper::Function = (f) -> f(),
    buffers_in_flight::Integer = 1,
    T = ComplexF32,
)
    return spawn_channel_thread(; T, buffers_in_flight) do c
        wrapper() do
            buff = Matrix{T}(undef, buff_size, num_channels)

            # Keep on generating buffers until `gen_buff!()` returns `false`.
            while gen_buff!(buff)
                put!(c, copy(buff))
            end
        end
    end
end
function generate_stream(f::Function, s::SoapySDR.Stream{T}; kwargs...) where {T<:Number}
    return generate_stream(f, s.mtu, s.nchannels; T, kwargs...)
end

"""
    stream_data(s_rx::SoapySDR.Stream, end_condition::Union{Integer,Event})

Returns a `Channel` which will yield buffers of data to be processed of size `s_rx.mtu`.
Starts an asynchronous task that does the reading from the stream, until the requested
number of samples are read, or the given `Event` is notified.
"""
function stream_data(
    s_rx::SoapySDR.Stream{T},
    end_condition::Union{Integer,Base.Event};
    leadin_buffers::Integer = 16,
    kwargs...,
) where {T<:Number}
    # Wrapper to activate/deactivate `s_rx`
    wrapper = (f) -> begin
        buff = Matrix{T}(undef, s_rx.mtu, s_rx.nchannels)

        # Let the stream come online for a bit
        SoapySDR.activate!(s_rx) do
            while leadin_buffers > 0
                read!(s_rx, split_matrix(buff))
                leadin_buffers -= 1
            end

            # Invoke the rest of `generate_stream()`
            f()
        end
    end

    # Read streams until we read the number of samples, or the given event
    # is triggered
    buff_idx = 0
    return generate_stream(s_rx.mtu, s_rx.nchannels; wrapper, T, kwargs...) do buff
        if isa(end_condition, Integer)
            if buff_idx * s_rx.mtu >= end_condition
                return false
            end
        else
            if end_condition.set
                return false
            end
        end

        flags = Ref{Int}(0)
        try
            read!(s_rx, split_matrix(buff); flags, throw_error = true)
        catch e
            if e isa SoapySDR.SoapySDRDeviceError
                if e.status == SoapySDR.SOAPY_SDR_OVERFLOW
                    @warn("RX buffer overflowed.")
                elseif e.status == SoapySDR.SOAPY_SDR_TIMEOUT
                    println("Tᵣ")
                else
                    println("Eᵣ")
                end
            else
                rethrow(e)
            end
        end

        buff_idx += 1
        return true
    end
end