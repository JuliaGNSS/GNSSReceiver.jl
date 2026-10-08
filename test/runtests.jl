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
# The loop core Tracking no longer re-exports: the Doppler and C/N₀ estimators the tests
# build and inspect. Imported by name, since TrackingLoops' wholesale exports (e.g.
# `normalize`, `VTStatus`) would clash with names the tests use from elsewhere.
import TrackingLoops
using TrackingLoops:
    ConventionalAssistedPLLAndDLL,
    ConventionalPLLAndDLL,
    MomentsCN0Estimator,
    NoiseRefCN0Estimator,
    VectorPLLAndDLL

# Unexported PositionVelocityTime types the GUI tests build solutions from.
using PositionVelocityTime: DOP, SupportedTimeSystem

using JLD2: load

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
