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

function _precompile_noise_channel(T, num_samples, num_chunks)
    spawn_signal_channel_thread(; T, num_samples, num_antenna_channels = 1) do channel
        for _ = 1:num_chunks
            put!(channel, T.(round.(randn(ComplexF32, num_samples, 1) .* 512)))
        end
    end
end

@setup_workload begin
    @compile_workload begin
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
