function read_files(files, num_samples, end_condition::Union{Nothing,Integer,Base.Event} = nothing; type = Complex{Int16})
    measurement_channel = MatrixSizedChannel{type}(num_samples, length(files))
    Base.errormonitor(Threads.@spawn begin
        measurement = get_measurement(files, num_samples, type)
        streams = open.(files)
        num_read_samples = 0
        try
            while true
                if end_condition isa Integer && num_read_samples > end_condition ||
                    end_condition isa Base.Event && end_condition.set
                    break
                end
                read_measurement!(streams, measurement)
                num_read_samples += num_samples
                push!(measurement_channel, copy(measurement))
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
    Matrix{type}(undef, num_samples, 1)
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