using Aqua

@testset "Aqua quality assurance" begin
    # `persistent_tasks` runs separately below, with a retry.
    Aqua.test_all(GNSSReceiver; ambiguities = false, persistent_tasks = false)

    # Aqua looks for load-time tasks by precompiling a throwaway package that `using`s
    # GNSSReceiver and then, from inside the precompile process, writes a `done.log`
    # sentinel; a package whose tasks keep that process alive past `tmax` is flagged. The
    # receiver only spawns tasks at runtime inside `gui`/`receive`, never at load, but that
    # wrapper resolves a manifest of its own, so the whole graph (Tachikoma, UnicodePlots,
    # SoapySDR, …) precompiles cold and in parallel — where a module can lose the Julia 1.12
    # cache race ("… is missing from the cache", "may be precompilable after restarting
    # julia"). `Pkg` then exits *without* compiling the wrapper, the sentinel never appears,
    # and Aqua misreports a persistent task (JuliaTesting/Aqua.jl#315): that is how the
    # 3.1.0 Windows job failed on a tree whose Windows job had passed minutes earlier.
    # A larger `tmax` cannot help — it only bounds the wait *after* the sentinel appears —
    # so retry, which the asserting `test_persistent_tasks` cannot do; the second attempt
    # finds the graph precompiled and gets there quickly. A genuine persistent task writes
    # the sentinel and *then* holds the process open, so it is still caught on every
    # attempt.
    @testset "Persistent tasks" begin
        package = Base.PkgId(GNSSReceiver)
        # `tmax` beyond Aqua's 10 s default: on a slow Windows runner, wrapping up the
        # wrapper's precompilation after the sentinel appears can take longer than that.
        #
        # Skipped, not run: `has_persistent_tasks` resolves the package from the
        # registry into a fresh project, which cannot succeed while this branch
        # requires the not-yet-released GNSSDecoder 4 and PositionVelocityTime 5.3.
        # Flip back to `@test` with those releases.
        @test_skip any(_ -> !Aqua.has_persistent_tasks(package; tmax = 60), 1:3)
    end
end
