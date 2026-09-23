using Test,
    GNSSReceiver,
    GNSSSignals,
    GNSSDecoder,
    Tracking,
    Unitful,
    Geodesy,
    AstroTime,
    PositionVelocityTime,
    StaticArrays,
    Random,
    Accessors,
    Acquisition,
    Dictionaries,
    LinearAlgebra,
    Scratch
# Unexported PositionVelocityTime internals the vector-tracking tests exercise
# directly (see "The Measurement-Model Surface" in that package's API docs).
using PositionVelocityTime:
    DOP, SPEED_OF_LIGHT, calc_DOP, calc_H, time_offset_available, time_scale_offset_to_gpst

using JLD2: load

# The navigation-filter configuration (also accepted by `receive`'s `vector_tracking`
# keyword in place of `true`); the tests exercise its parameters directly.
using GNSSReceiver: VectorTracking

using Unitful: Hz, dBHz, ms

# The reference harness (deterministic synthetic signals plus the noise-free
# reference every check is measured against) and the per-signal support matrix
# are shared by more than one file now — test/secondary_code_removal.jl
# generates its signals with the harness and records its evidence in the matrix,
# and test/signal_validation.jl runs the per-signal checks and the matrix
# integrity tests over everything recorded. So both modules are defined here,
# once, before anything that uses them.
include("reference_harness.jl")
include("signal_support.jl")

include("aqua.jl")
include("read_file.jl")
include("beamformer.jl")
include("lock_detector.jl")
include("acquisition_signal.jl")
include("process.jl")
include("vector_tracking.jl")
include("prn_selection.jl")
include("gui.jl")
include("save_data.jl")
include("receive.jl")
include("async_acquisition.jl")
include("hardware_capabilities.jl")
include("nco_referenced_loop.jl")
include("remote_hardware_loop.jl")
include("sample_buffer.jl")
include("signal_validation.jl")
include("ion_rtlsdr_integration.jl")
