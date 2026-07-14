# Changelog

# [0.2.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v0.1.6...v0.2.0) (2026-07-14)


### Bug Fixes

* **benchmark:** give each process sample a fresh state so it measures a real 45 s run ([76dbd82](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/76dbd82fc262ffe454ea76d16e498e2030dc4cee))
* generalize save_data, cover it in tests, and pre-v1 cleanup ([7ec5806](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/7ec5806d9331a568e95455ba1947e3ea8a220e8b))
* increment unsuccessful counter during reacquisition ([#48](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/48)) ([402783a](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/402783a206cb7aee1f2e9950ff958b672b4a2aa5))
* **receive:** thread receiver_state via typed Ref to avoid Core.Box allocations ([bb58434](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/bb584349ce1820f03984ee76d6799e43145127d2))
* reset decoder after reacquisition ([#47](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/47)) ([0df1fed](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/0df1fed57ed1a173352da12a1db573a9241b994f))
* small fixes to signal flow ([#45](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/45)) ([03a111a](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/03a111adb9536480561e3fbd7abc21e5ded63302))


### Features

* add extract hook to receive for custom per-chunk payloads ([79ab06a](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/79ab06a4205de117c2de9029ea116253b9bb9d6a))
* allow to specify PVT update rate ([#52](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/52)) ([0b4e515](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/0b4e515e76f2c8745fefa12897b5cc22f6b73e5c))
* allow to specify the doppler_estimator ([#56](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/56)) ([a93b8f3](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/a93b8f3f93a26851cf75ecfbc708a4e3d9e74da6))
* allow to specify the lock detector threshold ([#53](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/53)) ([128fca8](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/128fca8e92f37921ba723605fb8f57edc37bdd10))
* allow to specify the PRNs of the acquisition plan ([#46](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/46)) ([2a7a408](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/2a7a4087d5fa22754ddeb675e5e973f42b774b54))
* **benchmark:** human-readable sizes and memory ratio in PR comment ([1a20bf3](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/1a20bf3c4b4e1d5279090e5d4a0aa11a3aeba912))


### Performance Improvements

* **benchmark:** split process into per-stage 1 s benchmarks for reliable CI ([5b59f32](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/5b59f327c16a026ac505338a51446f3a5f751687))
* **channel:** replace Channel-backed MatrixSizedChannel with lock-free SignalChannel ([c5c40ef](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/c5c40ef6ed61d8d5b1cd602ad1cf2bc2b33eaec1))
* **process:** reuse a SatelliteState buffer in update_pvt, drop filter→Dictionary ([bceee57](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/bceee572210d32df641b587dc9e2c7e658fba9f9)), closes [#82](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/82)
* update receiver_sat_states in place (map!) instead of rebuilding each frame ([5a2f4c3](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/5a2f4c3a00489cc121b20cde8df0d2b7f6ffa156)), closes [82/#84](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/84) [#82](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/82)
