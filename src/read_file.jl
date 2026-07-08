# `read_measurement!` is the per-frame parser: `read_measurement!(streams, chunk)`
# must *completely* fill `chunk` (a `num_samples × num_ants` matrix of `type`) from
# the open `streams`. Contract at end of data: it MUST throw `EOFError` (or otherwise
# error) rather than fill the chunk only partially and return normally — `read_files`
# reuses two chunk buffers, so a short frame that returned would be pushed downstream
# still carrying stale samples from an earlier frame. A read that throws is caught
# below and cleanly ends the stream ("Reached end of file."). The default parser is a
# raw `read!`-per-antenna (one file per antenna column), and `read!` already throws on
# a short read; a custom parser for a packed or otherwise-encoded format should
# likewise build on `read!` (or throw `EOFError` itself) so the contract holds. The
# parser owns any temporary buffers it needs.
"""
    read_files(files, num_samples, end_condition = nothing; type = Complex{Int16},
               read_measurement! = read_measurement!)

Return a `SignalChannel` that yields `num_samples`-long chunks read from `files`, one
file per antenna channel (a single path is treated as one channel). Samples are read
as `type` elements.

The read runs on a spawned task that stops when `end_condition` is reached: an
`Integer` sample count, a notified `Base.Event`, or end-of-file (`nothing` reads until
EOF). Each chunk is a freshly allocated buffer, since the lock-free channel may still
hold a previously enqueued chunk.

A custom `read_measurement!(streams, chunk)` can be supplied to decode packed or
otherwise-encoded sample formats; see the note above for the contract it must honour.
"""
function read_files(
    files,
    num_samples,
    end_condition::Union{Nothing,Integer,Base.Event} = nothing;
    type = Complex{Int16},
    read_measurement! = read_measurement!,
)
    # Normalize a single path to a one-element vector so `open.(files)` yields a
    # vector of streams (a bare `String` broadcasts to a scalar `IOStream`, which
    # `read_measurement!` does not accept).
    files isa AbstractVector || (files = [files])
    num_ants = length(files)
    return spawn_signal_channel_thread(;
        T = type,
        num_samples,
        num_antenna_channels = num_ants,
    ) do out
        streams = open.(files)
        num_read_samples = 0
        try
            while true
                if end_condition isa Integer && num_read_samples > end_condition ||
                   end_condition isa Base.Event && end_condition.set
                    break
                end
                # Allocate a fresh buffer per chunk: the lock-free SignalChannel is
                # buffered, so the consumer may still hold a previously enqueued
                # buffer while we fill the next one.
                chunk = Matrix{type}(undef, num_samples, num_ants)
                read_measurement!(streams, chunk)
                num_read_samples += num_samples
                put!(out, chunk)
            end
        catch e
            if e isa EOFError
                println("Reached end of file.")
            else
                rethrow(e)
            end
        finally
            close.(streams)
        end
    end
end

"""
    read_uint8_iq_file(file, num_samples, end_condition = nothing; center = 128,
                       type = Complex{Int16})

Return a `SignalChannel` that yields `num_samples`-long chunks read from a single raw
recording of interleaved 8-bit **unsigned offset-binary** in-phase/quadrature samples —
the native sample format of many SDRs (e.g. RTL-SDR) and of public GNSS sample sets such
as the [ION SDR test data](https://sdr.ion.org/api-sample-data.html). Each unsigned byte
pair `(I, Q)` is recentred on `center` (128 for standard offset binary, where 128 ≈
zero) and returned as a baseband sample of element type `type`.

This is the single-file, offset-binary counterpart to [`read_files`](@ref), which reads
samples that are already stored in the target element type. Because the recentred samples
satisfy `|real|, |imag| ≤ center`, pass `max_meas = center` to [`receive`](@ref) when
`type` is `Complex{Int16}` (the default) so it selects Tracking's fast integer
downconvert-and-correlator. For a float `type` (e.g. `ComplexF32`) use `center = 127.5`
for exact midscale recentring and drop `max_meas`.

`end_condition` stops the read exactly like [`read_files`](@ref): an `Integer` sample
count, a notified `Base.Event`, or `nothing` to read until end-of-file.
"""
function read_uint8_iq_file(
    file,
    num_samples,
    end_condition::Union{Nothing,Integer,Base.Event} = nothing;
    center = 128,
    type = Complex{Int16},
)
    RT = real(type)
    c = RT(center)
    return spawn_signal_channel_thread(;
        T = type,
        num_samples,
        num_antenna_channels = 1,
    ) do out
        io = open(file)
        raw = Vector{UInt8}(undef, 2 * num_samples)
        num_read_samples = 0
        try
            while true
                if end_condition isa Integer && num_read_samples > end_condition ||
                   end_condition isa Base.Event && end_condition.set
                    break
                end
                n = readbytes!(io, raw)
                # Stop on a short (final) read so every emitted chunk is full.
                n < 2 * num_samples && break
                # Allocate a fresh buffer per chunk: the lock-free SignalChannel is
                # buffered, so the consumer may still hold a previously enqueued buffer.
                chunk = Matrix{type}(undef, num_samples, 1)
                @inbounds for i = 1:num_samples
                    chunk[i, 1] = complex(RT(raw[2i-1]) - c, RT(raw[2i]) - c)
                end
                num_read_samples += num_samples
                put!(out, chunk)
            end
        finally
            close(io)
        end
    end
end

function read_measurement!(streams::AbstractVector, measurements)
    foreach(
        (stream, measurement) -> read!(stream, measurement),
        streams,
        eachcol(measurements),
    )
end
