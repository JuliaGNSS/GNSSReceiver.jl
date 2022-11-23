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
end