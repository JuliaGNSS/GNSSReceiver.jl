using Aqua

@testset "Aqua quality assurance" begin
    Aqua.test_all(GNSSReceiver; ambiguities = false)
end
