# Changelog

# [4.5.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v4.4.0...v4.5.0) (2026-09-03)


### Features

* support GNSSSignals 4 ([3356c5e](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/3356c5ebd389736f61ae05ecee2c3ba15b0d541a))

# [4.4.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v4.3.0...v4.4.0) (2026-09-03)


### Features

* precompile the multi-system and two-band receive shapes too ([47c70bf](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/47c70bfcf14f78321d18cd5f0d94d5409d5539b4))
* precompile the receive pipeline ([816777b](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/816777b17b79a06cab50c6eac59fe1eeaff45345)), closes [#107](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/107)

# [4.3.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v4.2.0...v4.3.0) (2026-08-20)


### Bug Fixes

* enforce the lock dwell's documented invariants in the code ([568dad6](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/568dad6a11ed220cf787ec9bbe7a0f9ce77c6ea9)), closes [#118](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/118)
* **gui:** treat a coasting vector-loop member as not contributing ([9a97b65](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/9a97b6554151e8aacf020d8f285a66f58b146c0c))


### Features

* **gui:** draw still-settling satellites yellow in the CN0 panel ([874f402](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/874f4028d4b115b161af11a33173a0d27242e13c))
* read every prompt into a Van Dierendonck carrier phase-lock indicator ([38a71cb](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/38a71cb2da3c4d07879dd591708335912cf64cd0))
* scale the lock detectors' timings to the ranging signal's code period ([cdf3b4b](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/cdf3b4b4cde02c9e31506edfbcef61464c44c1c2))
* stage the lock detectors through the acquisition handover ([9b89bff](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/9b89bff12ecb4d9497ee12978ad2d57b1d80bb70)), closes [#118](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/118) [#118](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/118) [#118](https://github.com/JuliaGNSS/GNSSReceiver.jl/issues/118)

# [4.2.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v4.1.0...v4.2.0) (2026-08-20)


### Features

* support Tracking 8; reference the C/N₀ to the beamformer's own noise ([1dcc956](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/1dcc9568c300c62c6d82fed5d80c74d2aec8ba6f)), closes [JuliaGNSS/PositionVelocityTime.jl#73](https://github.com/JuliaGNSS/PositionVelocityTime.jl/issues/73)

# [4.1.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v4.0.0...v4.1.0) (2026-08-18)


### Features

* support Tracking 7; measure the noise floor per tracked signal ([7c1d962](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/7c1d962bee9330ddfbef017fe82c0e5da6f0e7d0))

# [4.0.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v3.3.1...v4.0.0) (2026-08-13)


* feat(vt)!: vector tracking (VDLL / VDFLL) through a navigation Kalman filter ([01b4a79](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/01b4a79ddb4f487ed445a9b2a53dab951012cb3f))


### BREAKING CHANGES

* `receive` no longer accepts `code_lock_cn0_threshold`, and
`ReceiverState` no longer accepts `doppler_estimator`. Both entry points take
`vector_tracking::Union{Bool,VectorTracking} = false` instead, from which the
Doppler estimator is derived — `VectorPLLAndDLL` when
true, the conventional FLL-assisted PLL/DLL when false — so that the estimator
sizing acquisition is always the one the receiver state bakes in. The C/N0 code
lock threshold now always comes from the per-system
`get_default_code_lock_cn0_threshold`. The GUI's fixed "not enough satellites" threshold
is likewise gone — the minimum count is layout-dependent (3 + one clock per
GNSS time system + one IFB per extra band), so the dashboard reports decoding
progress and lets the fix's own appearance signal success.

The dependency floors move with it: PositionVelocityTime 5.0.3 replacing `4.0.1`, and
KalmanFilters and Geodesy become direct dependencies. Tracking needs no bump — 3.3.0
already pins the `Tracking = "6"` major that `VectorPLLAndDLL` requires.

Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>

## [3.3.1](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v3.3.0...v3.3.1) (2026-08-12)


### Bug Fixes

* **gui:** keep the inter-system-bias anchor as a compact id ([cc38edb](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/cc38edb8bab76e156f14952cd96e7216cf289874))

# [3.3.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v3.2.0...v3.3.0) (2026-08-05)


### Features

* support Tracking 6 ([ea83490](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/ea834901861a3f843c39efd2f8efb443caa5d409))

# [3.2.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v3.1.0...v3.2.0) (2026-08-04)


### Features

* support Tracking 5; credit integration time in the code lock detector ([e077c33](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/e077c33e40e4df19ee1d8a934828a4c7a3e4fe65))

# [3.1.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v3.0.0...v3.1.0) (2026-07-27)


### Features

* OpenStreetMap map view in the Map panel ([8c2b9c8](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/8c2b9c856e72fb274e2147931a4b9cb4419929fa))

# [3.0.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v2.1.0...v3.0.0) (2026-07-27)


* feat!: replace terminal GUI with an interactive Tachikoma dashboard ([d3b9850](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/d3b9850d858c9d0547d548b6956c59c01e0d542d))


### Bug Fixes

* pace real-time file replay against an absolute deadline ([5d0ebee](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/5d0ebeec9eadff14baa363582200dad72122960d))
* UnicodeMaps 1.0 compat, docs xref, and Aqua persistent-tasks timeout ([eb2d77f](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/eb2d77fbf100e157f71b2cb53e742d8c120d365c))


### Features

* drop UnicodeMaps map; show coordinates + Google Maps link ([5d4d2f6](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/5d4d2f649ed3aba7bbb174486807235bd8b977ab))
* full DOP breakdown by default, clickable maps link, round sky plot ([a72e3cd](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/a72e3cde2094ff0f3b1c2109b95ae81a9f0050bf))
* real-time file replay + keep the TUI open after the stream ends ([dd3fd7d](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/dd3fd7d05fa6e717d3863e7f017a0c3613e11463))


### BREAKING CHANGES

* `GNSSReceiver.gui` no longer accepts the documented `io`
positional argument or the `construct_gui_panels` keyword; its signature is now
`gui(channel; fps = 12)`. Output always goes to the terminal that Tachikoma
owns, and the "Customising the display" hook for supplying your own panel
constructor is gone — panels are laid out by the dashboard's `view`. The
exported `gnss_receiver_gui` entry point is unchanged.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>

# [2.1.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v2.0.0...v2.1.0) (2026-07-26)


### Features

* support Tracking 4 ([76338f7](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/76338f7))

# [2.0.0](https://github.com/JuliaGNSS/GNSSReceiver.jl/compare/v1.0.0...v2.0.0) (2026-07-19)


* feat!: multi-constellation, multi-band reception ([deb6956](https://github.com/JuliaGNSS/GNSSReceiver.jl/commit/deb695644185ae1bb29bc4e6f4350de58001de75))


### BREAKING CHANGES

* receive/process/ReceiverState now take signal objects and new
acquisition/PVT keywords.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
