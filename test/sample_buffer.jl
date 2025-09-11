module TestSampleBuffer

using Test
using GNSSReceiver.SampleBuffers

@testset "SampleBuffer Tests" begin
    @testset "Constructor" begin
        buf = SampleBuffer(Float64, 100, Val(2))
        @test size(buf.buffer) == (100, 2)
        @test buf.max_length == 100
        @test buf.current_length == 0
        @test buf.start_index == 1
        @test buf.first_sample_counter == 1
    end

    @testset "Empty buffer operations" begin
        buf = SampleBuffer(Float64, 100, Val(2))
        samples = get_samples(buf)
        @test size(samples) == (0, 2)
        @test eltype(samples) == Float64
    end

    @testset "Basic buffer operations" begin
        buf = SampleBuffer(Float64, 10, Val(2))

        # Add first chunk
        chunk1 = [1.0 2.0; 3.0 4.0; 5.0 6.0]  # 3x2
        buf = buffer(buf, chunk1)
        @test buf.current_length == 3
        samples = get_samples(buf)
        @test samples == chunk1

        # Add second chunk
        chunk2 = [7.0 8.0; 9.0 10.0]  # 2x2
        buf = buffer(buf, chunk2)
        @test buf.current_length == 5
        samples = get_samples(buf)
        @test samples == [1.0 2.0; 3.0 4.0; 5.0 6.0; 7.0 8.0; 9.0 10.0]
    end

    @testset "FIFO behavior when exceeding max_length" begin
        buf = SampleBuffer(Float64, 5, Val(2))

        # Fill buffer to capacity
        chunk1 = [1.0 2.0; 3.0 4.0; 5.0 6.0; 7.0 8.0; 9.0 10.0]  # 5x2
        buf = buffer(buf, chunk1)
        @test buf.current_length == 5

        # Add more samples - should remove oldest
        chunk2 = [11.0 12.0; 13.0 14.0]  # 2x2
        buf = buffer(buf, chunk2)
        @test buf.current_length == 5
        samples = get_samples(buf)
        @test samples == [5.0 6.0; 7.0 8.0; 9.0 10.0; 11.0 12.0; 13.0 14.0]
    end

    @testset "Large chunk replacement" begin
        buf = SampleBuffer(Float64, 5, Val(2))

        # Add initial samples
        chunk1 = [1.0 2.0; 3.0 4.0]  # 2x2
        buf = buffer(buf, chunk1)

        # Add chunk larger than max_length
        large_chunk = [10.0 20.0; 30.0 40.0; 50.0 60.0; 70.0 80.0; 90.0 100.0; 110.0 120.0]  # 6x2
        buf = buffer(buf, large_chunk)
        @test buf.current_length == 5
        samples = get_samples(buf)
        @test samples == [30.0 40.0; 50.0 60.0; 70.0 80.0; 90.0 100.0; 110.0 120.0]
    end

    @testset "Empty samples handling" begin
        buf = SampleBuffer(Float64, 10, Val(2))
        empty_samples = Matrix{Float64}(undef, 0, 2)
        buf_new = buffer(buf, empty_samples)
        @test buf_new === buf  # Should return same buffer unchanged
    end

    @testset "Antenna count mismatch" begin
        buf = SampleBuffer(Float64, 10, Val(2))
        wrong_samples = [1.0 2.0 3.0; 4.0 5.0 6.0]  # 2x3 (3 antennas instead of 2)
        @test_throws ArgumentError buffer(buf, wrong_samples)
    end

    @testset "Sample counter functionality" begin
        buf = SampleBuffer(Float64, 10, Val(2))

        # Initial state
        @test buf.first_sample_counter == 1

        # Add first chunk
        chunk1 = [1.0 2.0; 3.0 4.0; 5.0 6.0]  # 3x2
        buf = buffer(buf, chunk1)
        @test buf.first_sample_counter == 1  # Still pointing to first sample

        # Add second chunk - still within capacity
        chunk2 = [7.0 8.0; 9.0 10.0]  # 2x2
        buf = buffer(buf, chunk2)
        @test buf.first_sample_counter == 1  # Still pointing to first sample

        # Fill to capacity
        chunk3 = [11.0 12.0; 13.0 14.0; 15.0 16.0; 17.0 18.0; 19.0 20.0]  # 5x2
        buf = buffer(buf, chunk3)
        @test buf.first_sample_counter == 1  # Still pointing to first sample
        @test buf.current_length == 10

        # Add more samples - should trigger FIFO
        chunk4 = [21.0 22.0; 23.0 24.0]  # 2x2
        buf = buffer(buf, chunk4)
        @test buf.first_sample_counter == 3  # First sample is now the 3rd original sample
        @test buf.current_length == 10

        # Add more samples
        chunk5 = [25.0 26.0; 27.0 28.0; 29.0 30.0]  # 3x2
        buf = buffer(buf, chunk5)
        @test buf.first_sample_counter == 6  # First sample is now the 6th original sample
        @test buf.current_length == 10
    end

    @testset "Large chunk counter behavior" begin
        buf = SampleBuffer(Float64, 5, Val(2))

        # Add initial samples
        chunk1 = [1.0 2.0; 3.0 4.0]  # 2x2
        buf = buffer(buf, chunk1)
        @test buf.first_sample_counter == 1

        # Add chunk larger than max_length
        large_chunk = [10.0 20.0; 30.0 40.0; 50.0 60.0; 70.0 80.0; 90.0 100.0; 110.0 120.0]  # 6x2
        buf = buffer(buf, large_chunk)
        @test buf.current_length == 5
        @test buf.first_sample_counter == 4  # 2 (initial) + 6 (large chunk) - 5 (max_length) + 1 = 4
    end

    @testset "isfull function" begin
        buf = SampleBuffer(Float64, 5, Val(2))

        # Initially empty
        @test !isfull(buf)

        # Add some samples but not full
        chunk = [1.0 2.0; 3.0 4.0]  # 2x2
        buf = buffer(buf, chunk)
        @test !isfull(buf)

        # Fill to capacity
        chunk_full = [5.0 6.0; 7.0 8.0; 9.0 10.0]  # 3x2
        buf = buffer(buf, chunk_full)
        @test isfull(buf)

        # Add more samples - should still be full (FIFO)
        chunk_more = [11.0 12.0]  # 1x2
        buf = buffer(buf, chunk_more)
        @test isfull(buf)
    end

    @testset "Vector (single antenna) functionality" begin
        # Test constructor for single antenna
        buf = SampleBuffer(Float64, 10, Val(1))
        @test buf.buffer isa Vector{Float64}
        @test buf.fifo_buffer isa Vector{Float64}
        @test length(buf.buffer) == 10
        @test buf.max_length == 10
        @test buf.current_length == 0
        @test buf.start_index == 1
        @test buf.first_sample_counter == 1

        # Test basic operations with vectors
        chunk1 = [1.0, 2.0, 3.0]
        buf = buffer(buf, chunk1)
        @test buf.current_length == 3
        samples = get_samples(buf)
        @test samples isa AbstractVector
        @test samples == chunk1

        # Test adding more samples
        chunk2 = [4.0, 5.0]
        buf = buffer(buf, chunk2)
        @test buf.current_length == 5
        samples = get_samples(buf)
        @test samples == [1.0, 2.0, 3.0, 4.0, 5.0]

        # Test FIFO behavior with vectors
        chunk3 = [6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]  # 7 more samples
        buf = buffer(buf, chunk3)
        @test buf.current_length == 10
        @test isfull(buf)
        samples = get_samples(buf)
        @test samples == [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]  # FIFO removed first 2 samples

        # Test large chunk replacement with vectors
        large_chunk =
            [20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0]
        buf = buffer(buf, large_chunk)
        @test buf.current_length == 10
        samples = get_samples(buf)
        @test samples == [22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0]

        # Test empty buffer
        buf_empty = SampleBuffer(Float64, 5, Val(1))
        samples = get_samples(buf_empty)
        @test samples isa AbstractVector
        @test length(samples) == 0

        # Test sample counter with vectors
        buf_counter = SampleBuffer(Float64, 5, Val(1))
        chunk = [1.0, 2.0, 3.0]
        buf_counter = buffer(buf_counter, chunk)
        @test buf_counter.first_sample_counter == 1

        chunk_overflow = [4.0, 5.0, 6.0, 7.0]  # This will cause FIFO
        buf_counter = buffer(buf_counter, chunk_overflow)
        @test buf_counter.first_sample_counter == 3  # 2 samples removed
        @test isfull(buf_counter)
    end

    @testset "Helper functions for first sample counter" begin
        buf = SampleBuffer(Float64, 10, Val(2))

        # Test initial counter value
        @test get_first_sample_counter(buf) == 1

        # Add some samples and check counter doesn't change (within capacity)
        chunk1 = [1.0 2.0; 3.0 4.0; 5.0 6.0]
        buf = buffer(buf, chunk1)
        @test get_first_sample_counter(buf) == 1

        # Fill buffer and cause FIFO behavior
        large_chunk = [
            7.0 8.0
            9.0 10.0
            11.0 12.0
            13.0 14.0
            15.0 16.0
            17.0 18.0
            19.0 20.0
            21.0 22.0
        ]
        buf = buffer(buf, large_chunk)
        @test get_first_sample_counter(buf) > 1  # Counter should have advanced

        # Test reset functionality
        original_counter = get_first_sample_counter(buf)
        @test original_counter > 1

        buf_reset = reset_first_sample_counter(buf)
        @test get_first_sample_counter(buf_reset) == 1

        # Verify other properties unchanged after reset
        @test buf_reset.current_length == buf.current_length
        @test buf_reset.max_length == buf.max_length
        @test buf_reset.start_index == buf.start_index
        @test get_samples(buf_reset) == get_samples(buf)  # Same data

        # Test reset on Vector buffer
        buf_vec = SampleBuffer(Float64, 5, Val(1))
        chunk_vec = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]  # Cause FIFO
        buf_vec = buffer(buf_vec, chunk_vec)
        @test get_first_sample_counter(buf_vec) > 1

        buf_vec_reset = reset_first_sample_counter(buf_vec)
        @test get_first_sample_counter(buf_vec_reset) == 1
    end
end

end