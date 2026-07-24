using Aqua

@testset "Aqua quality assurance" begin
    # `persistent_tasks` with a longer `tmax`: no dependency actually spawns a persistent
    # task (`Aqua.find_persistent_tasks_deps(GNSSReceiver)` is empty), and the receiver only
    # spawns tasks at runtime inside `gui`/`receive`, never at load. But loading the heavier
    # UI dependency graph (Tachikoma, UnicodeMaps, …) can take longer than Aqua's default
    # 10 s settling window on slow CI (Windows), tripping a false positive — so give it more.
    Aqua.test_all(GNSSReceiver; ambiguities = false, persistent_tasks = (tmax = 60,))
end
