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
include("sample_buffer.jl")
include("ion_rtlsdr_integration.jl")
