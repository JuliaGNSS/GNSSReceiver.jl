@testset "read_uint8_iq_file" begin
    # 24 samples of interleaved 8-bit offset-binary I/Q = 48 bytes.
    tmp = tempname()
    write(tmp, UInt8[UInt8(i % 256) for i = 0:47])

    @testset "Complex{Int16} (default)" begin
        ch = read_uint8_iq_file(tmp, 4)   # 4 samples/chunk -> 6 chunks
        chunks = collect(ch)
        @test length(chunks) == 6
        @test eltype(chunks[1]) == Complex{Int16}
        @test size(chunks[1]) == (4, 1)
        # First (I, Q) byte pair is (0, 1), recentred on 128.
        @test chunks[1][1, 1] == complex(Int16(-128), Int16(-127))
    end

    @testset "Integer end_condition" begin
        # Stops once more than 4 samples have been read: two full chunks.
        ch = read_uint8_iq_file(tmp, 4, 4)
        @test length(collect(ch)) == 2
    end

    @testset "Float element type with midscale center" begin
        ch = read_uint8_iq_file(tmp, 4; center = 127.5, type = ComplexF32)
        chunks = collect(ch)
        @test eltype(chunks[1]) == ComplexF32
        @test chunks[1][1, 1] == ComplexF32(-127.5, -126.5)
    end

    rm(tmp; force = true)
end
