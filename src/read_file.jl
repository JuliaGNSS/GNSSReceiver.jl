"""
    read_files(files, num_samples, end_condition = nothing; type = Complex{Int16})

Return a `SignalChannel` that yields `num_samples`-long chunks read from `files`, one
file per antenna channel (a single path is treated as one channel). Samples are read
as `type` elements.

The read runs on a spawned task that stops when `end_condition` is reached: an
`Integer` sample count, a notified `Base.Event`, or end-of-file (`nothing` reads until
EOF). Each chunk is a freshly allocated buffer, since the lock-free channel may still
hold a previously enqueued chunk.
"""
function read_files(
    files,
    num_samples,
    end_condition::Union{Nothing,Integer,Base.Event} = nothing;
    type = Complex{Int16},
)
    num_ants = files isa AbstractVector ? length(files) : 1
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

function read_measurement!(streams::AbstractVector, measurements)
    foreach(
        (stream, measurement) -> read!(stream, measurement),
        streams,
        eachcol(measurements),
    )
end
