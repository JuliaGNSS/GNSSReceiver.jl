# Precompile workload (PrecompileTools). A first `receive` on a fresh session
# costs ~18 s of compilation on a workstation and three to four times that on
# an embedded ARM host — the hardware-correlator harness in issue #107 spends
# 43 s warming the pipeline up before it dares to start the stream, and the
# paths the warm-up misses then compile live, each stall holding every tracking
# loop open. Running the standard pipeline here, on a short burst of noise
# through the integer and the float sample paths, moves that cost into
# `Pkg.precompile`. Nothing is detected, which is the point: acquisition, the
# tracking pass, lock detection, the decode consumer and the PVT cadence gate
# all execute, and nothing depends on a signal. The pipeline is specialised on
# the system tuple, so it runs for one system, for the two default
# constellations in one band, and for two bands in lock-step.
using PrecompileTools: @setup_workload, @compile_workload

# ── A device to compile the hardware-correlator pipeline against ─────────────
#
# The hardware receiver is a second, separate pipeline: `process` takes its
# correlator outputs from a [`HardwareCorrelatorLink`](@ref) rather than from a
# `Tracking` backend, and every method from `receive`'s processing closure down
# to the epoch fold is specialised on that source. Compiling it live costs 1.7 s
# on an Orin — 1.7 s inside one chunk, during which nothing drains the device's
# dump ring, so it overruns and every satellite's records for the next second
# and a half are simply gone (issue #107).
#
# Nothing here talks to hardware: the three streams are ordinary channels the
# workload fills itself, which is all the pipeline ever sees of a device. The
# link erases the device's type (see `HardwareCorrelatorLink`), so what is
# cached here is what a *real* vendor device runs — this stub and an M2SDR
# produce the same `HardwareCorrelatorLink{EarlyPromptLateCorrelator{…}}` and
# therefore the same specialisations.
struct _PrecompileHardwareSDR{C<:Tracking.AbstractCorrelator} <:
       AbstractHardwareCorrelatorSDR
    raw::SignalChannel{Complex{Int16},1}
    dumps::PipeChannel{CorrelatorDump{C}}
    ncos::PipeChannel{NCOUpdate}
end

raw_sample_channel(sdr::_PrecompileHardwareSDR) = sdr.raw
correlator_dump_channel(sdr::_PrecompileHardwareSDR) = sdr.dumps
nco_update_channel(sdr::_PrecompileHardwareSDR) = sdr.ncos
num_hardware_channels(::_PrecompileHardwareSDR) = 4
assign_channel!(::_PrecompileHardwareSDR, args...; kwargs...) = nothing
release_channel!(::_PrecompileHardwareSDR, hw_channel) = nothing

# The correlator a single-antenna Early/Prompt/Late device dumps. Both the
# element type and the tap count are part of the link's type parameter, so this
# is what pins the cached specialisations to the ones a vendor package's device
# will hit.
_precompile_epl(early, prompt, late) =
    EarlyPromptLateCorrelator(SVector{3,ComplexF64}(early, prompt, late), 1)

const _PRECOMPILE_EPL = typeof(_precompile_epl(0, 0, 0))

# Feed the stub device: one raw chunk and one epoch's worth of dumps per
# iteration, on a sample axis that advances exactly as a real device's does, and
# drain whatever NCO updates the receiver pushes back so the ring cannot fill.
function _precompile_drive_hardware_sdr(sdr, num_samples, num_chunks)
    Threads.@spawn begin
        try
            chunk = Complex{Int16}.(round.(randn(ComplexF32, num_samples, 1) .* 512))
            for i = 1:num_chunks
                base = i * num_samples
                batch = CorrelatorDump{_PRECOMPILE_EPL}[]
                for hw_channel = 1:2
                    push!(
                        batch,
                        CorrelatorDump(
                            hw_channel,
                            hw_channel,
                            CorrelatorOutput(
                                _precompile_epl(400 + 0im, 1000 + 10im, 400 + 0im),
                                num_samples,
                                base + hw_channel,
                            ),
                            mod(0.001 * base, 1023.0),
                        ),
                    )
                end
                # The strobe is what closes an epoch on a device whose channels
                # are momentarily silent, so the fold path that depends on it is
                # compiled here too.
                push!(batch, epoch_strobe(_precompile_epl(0, 0, 0), base))
                put!(sdr.dumps, batch)
                put!(sdr.raw, chunk)
                while Base.n_avail(sdr.ncos) > 0
                    take!(sdr.ncos)
                end
            end
        finally
            close(sdr.raw)
            close(sdr.dumps)
        end
    end
end

# Satellites for one real PVT solve, or an empty vector when this cannot be
# built.
#
# `PositionVelocityTime` precompiles `calc_pvt` itself, and for its own callers
# that works — 0.006 s for a first solve. It does not survive reaching this
# package. Two things break it, measured (x86, first `calc_pvt` on the states
# below):
#
#   PVT alone                            0.006 s
#   + Tracking (loads its PVT extension) 0.448 s
#   + GNSSReceiver                       1.285 s
#
# The extension's methods are added *after* PVT precompiled, so they invalidate
# the cached solve; and the receiver hands `calc_pvt` a `Vector{SatelliteState}`
# with an abstract element type (it pools constellations), which is a different
# specialisation from the concrete vectors PVT caches. On an Orin the two
# together cost **2.6-2.8 s inside the fold at the first fix** — the largest
# stall left in a live hardware run, and the one that still releases satellites
# (issue #107). Only this package sees all the pieces at once, so this is where
# the solve has to be cached.
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

        # The hardware-correlator receiver: the same pipeline taking its
        # correlator outputs off a device instead of computing them. Run with
        # the live defaults — asynchronous acquisition on the interactive pool —
        # so what is cached is the shape a hardware receiver actually runs,
        # including the scan-merge path. The logger is silenced because
        # `Pkg.precompile` runs single-threaded, where asynchronous acquisition
        # rightly warns that it cannot overlap a scan with tracking.
        Base.CoreLogging.with_logger(Base.CoreLogging.NullLogger()) do
            sdr = _PrecompileHardwareSDR(
                SignalChannel{Complex{Int16},1}(4000, 8),
                PipeChannel{CorrelatorDump{_PRECOMPILE_EPL}}(1 << 12),
                PipeChannel{NCOUpdate}(1 << 8),
            )
            driver = _precompile_drive_hardware_sdr(sdr, 4000, 12)
            data = receive(
                sdr,
                GPSL1CA(),
                4e6Hz;
                max_meas = 2^11,
                acquire_every = 4u"ms",
                pvt_update_interval = 4u"ms",
            )
            collect_data(data)
            wait(driver)
        end
    end
end
