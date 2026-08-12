# Changelog

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
