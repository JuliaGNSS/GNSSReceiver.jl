using BenchmarkTools
include("src/sample_buffer.jl")
using .SampleBuffers

# Test scenario similar to your requirements: max_length=4000, chunk_size=1000, 2 antennas
const max_length = 4000
const chunk_size = 1000
const num_antennas = 2

# Helper functions
function create_full_buffer()
    buf_full = SampleBuffer(Float64, max_length, Val(num_antennas))
    for i = 1:4
        buf_full = buffer(buf_full, randn(chunk_size, num_antennas))
    end
    return buf_full
end

function create_wrapped_buffer()
    buf_wrapped = SampleBuffer(Float64, max_length, Val(num_antennas))
    for i = 1:6  # This will cause wrapping
        buf_wrapped = buffer(buf_wrapped, randn(chunk_size, num_antennas))
    end
    return buf_wrapped
end

function full_workflow()
    buf = SampleBuffer(Float64, 4000, Val(2))
    for i = 1:6
        chunk_data = randn(1000, 2)
        buf = buffer(buf, chunk_data)
    end
    return get_samples(buf)
end

# Pre-create test data to avoid measuring random number generation
const test_chunk = randn(chunk_size, num_antennas)
const buf_fresh = SampleBuffer(Float64, max_length, Val(num_antennas))
const buf_full = create_full_buffer()
const buf_simple =
    buffer(SampleBuffer(Float64, max_length, Val(num_antennas)), randn(2000, num_antennas))
const buf_wrapped = create_wrapped_buffer()

# Create benchmark suite
suite = BenchmarkGroup()

suite["constructor"] =
    @benchmarkable SampleBuffer(Float64, $max_length, $(Val(num_antennas)))

suite["buffer"] = BenchmarkGroup()
suite["buffer"]["within_capacity"] = @benchmarkable buffer($buf_fresh, $test_chunk)
suite["buffer"]["fifo_case"] = @benchmarkable buffer($buf_full, $test_chunk)

suite["get_samples"] = BenchmarkGroup()
suite["get_samples"]["non_wrapped"] = @benchmarkable get_samples($buf_simple)
suite["get_samples"]["wrapped"] = @benchmarkable get_samples($buf_wrapped)

suite["full_workflow"] = @benchmarkable full_workflow()

println("Running SampleBuffer Performance Benchmarks...")
println("="^60)

# Run the benchmark suite
results = run(suite; verbose = true)

println("\nBenchmark Results:")
println("="^60)
for (group_name, group_results) in results
    println("\n$(uppercase(group_name)):")
    if isa(group_results, BenchmarkGroup)
        for (subtest_name, result) in group_results
            println("  $subtest_name:")
            println("    Time: $(BenchmarkTools.prettytime(median(result).time))")
            println("    Memory: $(BenchmarkTools.prettymemory(median(result).memory))")
            println("    Allocations: $(median(result).allocs)")
        end
    else
        println("  Time: $(BenchmarkTools.prettytime(median(group_results).time))")
        println("  Memory: $(BenchmarkTools.prettymemory(median(group_results).memory))")
        println("  Allocations: $(median(group_results).allocs)")
    end
end