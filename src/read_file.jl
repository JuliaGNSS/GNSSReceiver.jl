function read_files(files, num_samples; type = Complex{Int16})
    measurement = get_measurement(files, num_samples, type)
    is_compressed = typeof(files) <: AbstractVector ? all(f -> endswith(f, ".gz"), files) : endswith(files, ".gz")
    streams_gz = if is_compressed
        GZip.open(files)
    else
        GZipStream[]
    end
    streams = if !is_compressed
        open.(files)
    else
        IOStream[]
    end
    measurement_channel = Channel{typeof(measurement)}()
    Base.errormonitor(Threads.@spawn begin
        try
            while true
                if is_compressed
                    read_measurement!(streams_gz, measurement)
                else
                    read_measurement!(streams, measurement)
                end
                push!(measurement_channel, measurement)
            end
        catch e
            if e isa EOFError
                println("Reached end of file.")
            else
                rethrow(e)
            end
        finally
            close(measurement_channel)
        end
    end)
    return measurement_channel
end

function get_measurement(files, num_samples, type)
    files isa AbstractVector ? Matrix{type}(undef, num_samples, length(files)) :
    Vector{type}(undef, num_samples)
end

function read_measurement!(streams::AbstractVector, measurements)
    foreach(
        (stream, measurement) -> read!(stream, measurement),
        streams,
        eachcol(measurements),
    )
end

function read_measurement!(stream, measurement)
    read!(stream, measurement)
end