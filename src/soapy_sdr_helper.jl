# Helper for turning a matrix into a tuple of views, for use with the SoapySDR API.
function split_matrix(m::AbstractMatrix{T}) where {T<:Number}
    return [view(m, :, idx) for idx = 1:size(m, 2)]
end

"""
    generate_stream(gen_buff!::Function, num_samples, num_antenna_channels; kwargs...)

Return a `SignalChannel` that yields buffers produced by `gen_buff!`. Keeps
generating buffers until `gen_buff!(buff)` returns `false`.
"""
function generate_stream(
    gen_buff!::Function,
    num_samples::Integer,
    num_antenna_channels::Integer;
    wrapper::Function = (f) -> f(),
    buffers_in_flight::Integer = 16,
    T = ComplexF32,
)
    return spawn_signal_channel_thread(;
        T,
        num_samples,
        num_antenna_channels,
        buffers_in_flight,
    ) do c
        wrapper() do
            buff = Matrix{T}(undef, num_samples, num_antenna_channels)

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

Return a `SignalChannel` which yields buffers of data of size `s_rx.mtu`. Starts an
asynchronous task that reads from the stream until the requested number of samples
are read, or the given `Event` is notified.
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
            read!(s_rx, split_matrix(buff); flags, timeout = 0.9u"s", throw_error = true)
        catch e
            if e isa SoapySDR.SoapySDRDeviceError
                if e.status == SoapySDR.SOAPY_SDR_OVERFLOW
                    print("O")
                elseif e.status == SoapySDR.SOAPY_SDR_TIMEOUT
                    print("Tᵣ")
                else
                    print("Eᵣ")
                end
            else
                rethrow(e)
            end
        end

        buff_idx += 1
        return true
    end
end
