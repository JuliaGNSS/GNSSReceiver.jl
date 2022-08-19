function read_files(files, sampling_freq, measurement = get_default_measurement(files, sampling_freq))
    streams = open.(files)
    measurement_channel = Channel{typeof(measurement)}()
    Base.errormonitor(Threads.@spawn begin
        try
            while true
                read_measurement!(streams, measurement)
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

function get_default_measurement(files, sampling_freq)
    num_samples = Int(upreferred(sampling_freq * 4ms))
    files isa AbstractVector ? Matrix{Complex{Int16}}(undef, num_samples, length(files)) : Vector{Complex{Int16}}(undef, num_samples)
end

function read_measurement!(streams::AbstractVector, measurements)
    foreach((stream, measurement) -> read!(stream, measurement), streams, eachcol(measurements))
end

function read_measurement!(stream, measurement)
    read!(stream, measurement)
end