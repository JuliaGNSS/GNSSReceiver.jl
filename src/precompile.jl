# Precompile workload (PrecompileTools). A first `receive` on a fresh session
# costs ~18 s of compilation on a workstation and three to four times that on
# an embedded ARM host — the hardware-correlator harness in issue #107 spends
# 43 s warming the pipeline up before it dares to start the stream, and the
# paths the warm-up misses then compile live, each stall holding every tracking
# loop open. Running the standard pipeline here, on a short burst of noise
# through the integer and the float sample paths, moves that cost into
# `Pkg.precompile`. Nothing is detected, which is the point: acquisition, the
# tracking pass, lock detection, the decode consumer and the PVT cadence gate
# all execute, and nothing depends on a signal.
using PrecompileTools: @setup_workload, @compile_workload

function _precompile_noise_channel(T, num_samples, num_chunks)
    spawn_signal_channel_thread(; T, num_samples, num_antenna_channels = 1) do channel
        for _ = 1:num_chunks
            put!(channel, T.(round.(randn(ComplexF32, num_samples, 1) .* 512)))
        end
    end
end

@setup_workload begin
    system = GPSL1CA()
    sampling_freq = 4e6Hz
    num_samples = 4000
    @compile_workload begin
        # Integer front end: the common live case, with the Int16 backend
        # `max_meas` selects.
        data = receive(
            _precompile_noise_channel(Complex{Int16}, num_samples, 12),
            system,
            sampling_freq;
            max_meas = 2^12,
            acquire_every = 4u"ms",
            pvt_update_interval = 4u"ms",
        )
        collect_data(data)
        # Float samples (file replay, simulations): the float CPU backend.
        data = receive(
            _precompile_noise_channel(ComplexF32, num_samples, 12),
            system,
            sampling_freq;
            acquire_every = 4u"ms",
            pvt_update_interval = 4u"ms",
        )
        collect_data(data)
    end
end
