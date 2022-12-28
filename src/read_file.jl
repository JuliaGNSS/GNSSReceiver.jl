function read_files(
    files,
    num_samples,
    end_condition::Union{Nothing,Integer,Base.Event} = nothing;
    type = Complex{Int16},
)
    num_ants = files isa AbstractVector ? length(files) : 1
    return spawn_channel_thread(; T = type, num_samples, num_antenna_channels = num_ants) do out
        # Use two chunks to avoid race condition (One to fill and one to read)
        chunk_idx = 1
        chunks = [
            Matrix{type}(undef, num_samples, num_ants),
            Matrix{type}(undef, num_samples, num_ants),
        ]
        streams = open.(files)
        num_read_samples = 0
        try
            while true
                if end_condition isa Integer && num_read_samples > end_condition ||
                   end_condition isa Base.Event && end_condition.set
                    break
                end
                read_measurement!(streams, chunks[chunk_idx])
                num_read_samples += num_samples
                push!(out, chunks[chunk_idx])
                chunk_idx = mod1(chunk_idx + 1, length(chunks))
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