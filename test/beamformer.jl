@testset "EigenBeamformer" begin
    post_corr_filter = GNSSReceiver.EigenBeamformer(4)

    @test post_corr_filter.beamformer == [0, 0, 0, 1]
    @test post_corr_filter.counter == 0

    next_post_corr_filter = GNSSReceiver.Tracking.update(post_corr_filter, [1, 1, 1, 1])
    @test next_post_corr_filter.beamformer == [0, 0, 0, 1]
    @test next_post_corr_filter.counter == 1
    @test all(next_post_corr_filter.covariance .== 1)

    post_corr_filter = GNSSReceiver.EigenBeamformer(4, 1)
    next_post_corr_filter = GNSSReceiver.Tracking.update(post_corr_filter, [1, 1, 1, 1])
    @test next_post_corr_filter.beamformer ≈ [1, 1, 1, 1]
    @test next_post_corr_filter.counter == 1
    @test all(next_post_corr_filter.covariance .== 0)

    # Tracking combines the antennas itself, so what the filter owes it is its current
    # weights — the same vector `Tracking.update` evolves, and the one the C/N₀ path
    # reduces the signal's noise covariance through (`wᴴR̂w`).
    @test Tracking.get_weights(GNSSReceiver.EigenBeamformer(4), NumAnts(4)) ==
          SVector{4,ComplexF64}(0, 0, 0, 1)
    @test Tracking.get_weights(next_post_corr_filter, NumAnts(4)) ===
          next_post_corr_filter.beamformer
    # Before its first recomputation the beamformer selects the last antenna, exactly as
    # `DefaultPostCorrFilter` does, so a fresh multi-antenna receiver starts out reporting
    # the same C/N₀ as the single-antenna default would.
    @test Tracking.get_weights(GNSSReceiver.EigenBeamformer(4), NumAnts(4)) ==
          Tracking.get_weights(Tracking.DefaultPostCorrFilter(), NumAnts(4))
    # A weight vector is only meaningful for the array it was computed for; asking for
    # another count is a wiring mistake rather than something to silently reshape.
    @test_throws MethodError Tracking.get_weights(
        GNSSReceiver.EigenBeamformer(4),
        NumAnts(2),
    )
end
