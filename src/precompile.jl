# Precompile workload (PrecompileTools). A first `receive` on a fresh session
# costs ~18 s of compilation on a workstation and three to four times that on
# an embedded ARM host — the hardware-correlator receiver in issue #107 spent
# 43 s warming the pipeline up before it dared to start the stream, and the
# paths the warm-up missed then compiled live, each stall holding every tracking
# loop open. Running the standard pipeline here, on a short burst of noise
# through the integer and the float sample paths, moves that cost into
# `Pkg.precompile`. Nothing is detected, which is the point: acquisition, the
# tracking pass, lock detection, the decode consumer and the PVT cadence gate
# all execute, and nothing depends on a signal. The pipeline is specialised on
# the system tuple, so it runs for one system, for the two default
# constellations in one band, and for two bands in lock-step.
using PrecompileTools: @setup_workload, @compile_workload

# Satellites for one real PVT solve, or an empty vector when this cannot be
# built.
#
# `PositionVelocityTime` precompiles `calc_pvt` itself, and for its own callers
# that works — 0.006 s for a first solve. It does not survive reaching this
# package, for two independent reasons, and neither of them is PVT's to fix.
#
# **Invalidation, from `Tracking`'s dependencies.** Measured (x86, first
# `calc_pvt` on the fixtures below, in a fresh session):
#
#   PVT alone                                  0.006 s
#   + Static                                   0.626 s
#   + Polyester                                0.452 s
#   + Tracking                                 0.472 s
#
# `Static` alone accounts for all of it: `Tracking` reaches it through
# `Polyester`'s `@batch`. `Static` and its siblings add methods to Base generics
# for their static-integer types — `abs2(::Union{StaticBool,StaticFloat64,
# StaticInt})`, `length(::Type{<:NDIndex})`, `(:)(::Integer, ::StaticInt)`,
# `ifelse`, `IteratorSize` — and any precompiled code that called those
# generically is discarded when they appear. `SnoopCompile`'s invalidation trees
# attribute **74 PositionVelocityTime `MethodInstance`s to 7 such insertions**,
# and the list is exactly what the board's `--trace-compile` named at the first
# fix: `calc_pvt`, `user_position`, `decide_bias_layout`, `calc_H`,
# `band_ifb_layout`, `calc_user_velocity_and_clock_drift`.
#
# **Specialisation.** The receiver hands `calc_pvt` its own
# `pvt_sat_state_buffer`, whose element type is a union over the configured
# systems (see `pvt_sat_state_type`) — not one of the concrete vectors PVT
# caches for itself.
#
# On an Orin the two together cost **2.6-2.8 s inside the fold at the first
# fix** — for a while the largest stall left in a live hardware run, and one
# that released satellites every time (issue #107).
#
# This package is where it has to be fixed: the invalidation comes from
# `Tracking`'s dependency tree rather than from PVT, so PVT cannot precompile
# around it, and the buffer type is the receiver's own. By the time *this*
# workload runs, `Static` and everything else is already loaded, so what it
# caches is compiled in the world the receiver actually runs in and nothing
# invalidates it afterwards.
#
# The satellites are PVT's own precompile fixtures rather than a copy of them:
# duplicating thirty-five ephemeris fields per satellite here would rot against
# the originals. It is a workload, so it degrades to a no-op if those internals
# are ever renamed — the receiver is slower to its first fix, nothing breaks.
function _precompile_pvt_states()
    empty = PositionVelocityTime.SatelliteState[]
    isdefined(PositionVelocityTime, :_precompile_states) || return empty
    isdefined(PositionVelocityTime, :_PRECOMPILE_GPS_L1CA_STATES) || return empty
    try
        # Concretely typed, exactly as a GPS-only receiver's buffer is.
        PositionVelocityTime._precompile_states(
            GPSL1CA(),
            PositionVelocityTime._PRECOMPILE_GPS_L1CA_STATES,
            identity,
            GPSL1CA(),
        )
    catch
        empty
    end
end

function _precompile_noise_channel(T, num_samples, num_chunks)
    spawn_signal_channel_thread(; T, num_samples, num_antenna_channels = 1) do channel
        for _ = 1:num_chunks
            put!(channel, T.(round.(randn(ComplexF32, num_samples, 1) .* 512)))
        end
    end
end

@setup_workload begin
    pvt_states = _precompile_pvt_states()
    pvt_states_abstract = Vector{PositionVelocityTime.SatelliteState}(pvt_states)
    @compile_workload begin
        # One real navigation solution, cold and warm started and with the
        # atmospheric corrections both ways — the shapes `update_pvt` calls.
        # Run on the *concrete* element type a single-constellation receiver's
        # `pvt_sat_state_buffer` has and on the abstract `SatelliteState`
        # fallback, because those are two specialisations and a receiver hits
        # one or the other depending on whether its satellite type could be
        # inferred (see `pvt_sat_state_type`). A multi-constellation receiver's
        # union is a third, and is left to its own first solve.
        for states in (pvt_states, pvt_states_abstract)
            if !isempty(states)
                pvt = calc_pvt(states; approximate_year = 2021)
                calc_pvt(states, pvt; approximate_year = 2021)
                calc_pvt(
                    states;
                    approximate_year = 2021,
                    enable_ionospheric_correction = false,
                    enable_tropospheric_correction = false,
                )
            end
        end
        # Integer front end: the common live case, with the Int16 backend
        # `max_meas` selects — one system, then the two default constellations
        # in one band (the multi-system tracking state, decoder and PVT paths).
        # Galileo E1B's CBOC replica needs at least twelve samples per chip, so
        # that band is sampled at 24 chips per sample period.
        for (systems, sampling_freq, num_samples) in (
            (GPSL1CA(), 4e6Hz, 4000),
            ((GPSL1CA(), GalileoE1B()), 24.552e6Hz, 24552),
        )
            data = receive(
                _precompile_noise_channel(Complex{Int16}, num_samples, 12),
                systems,
                sampling_freq;
                max_meas = 2^12,
                acquire_every = 4u"ms",
                pvt_update_interval = 4u"ms",
            )
            collect_data(data)
            # Float samples (file replay, simulations): the float CPU backend.
            data = receive(
                _precompile_noise_channel(ComplexF32, num_samples, 12),
                systems,
                sampling_freq;
                acquire_every = 4u"ms",
                pvt_update_interval = 4u"ms",
            )
            collect_data(data)
        end
        # Two RF bands in lock-step: L1 (GPS + Galileo) and L5, one code period
        # per 1 ms chunk at the shared sampling frequency.
        data = receive(
            (
                _precompile_noise_channel(Complex{Int16}, 24552, 6),
                _precompile_noise_channel(Complex{Int16}, 24552, 6),
            ),
            ((GPSL1CA(), GalileoE1B()), (GPSL5I(),)),
            24.552e6Hz;
            max_meas = 2^12,
            acquire_every = 4u"ms",
            pvt_update_interval = 4u"ms",
        )
        collect_data(data)
    end
end
