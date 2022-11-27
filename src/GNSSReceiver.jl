module GNSSReceiver

using StaticArrays,
    GNSSDecoder,
    Tracking,
    PositionVelocityTime,
    GNSSSignals,
    Acquisition,
    Unitful,
    JLD2,
    LinearAlgebra,
    SoapySDR

using Unitful: dBHz, ms, Hz

export receive,
    reset_but_keep_decoders_and_pvt,
    read_files,
    save_data,
    get_gui_data_channel,
    tee,
    rechunk,
    stream_data,
    vectorize_data,
    membuffer,
    write_to_file,
    gnss_receiver_gui,
    gnss_write_to_file

include("lock_detector.jl")
include("beamformer.jl")

struct SatelliteChannelState{
    DS<:GNSSDecoderState,
    TS<:TrackingState,
    COLD<:CodeLockDetector,
    CALD<:CarrierLockDetector,
}
    track_state::TS
    decoder::DS
    code_lock_detector::COLD
    carrier_lock_detector::CALD
    time_in_lock::typeof(1ms)
end

function is_in_lock(state::SatelliteChannelState)
    is_in_lock(state.code_lock_detector) && is_in_lock(state.carrier_lock_detector)
end

function mark_out_of_lock(state::SatelliteChannelState)
    SatelliteChannelState(
        state.track_state,
        state.decoder,
        mark_out_of_lock(state.code_lock_detector),
        mark_out_of_lock(state.carrier_lock_detector),
        0ms,
    )
end

Base.@kwdef struct ReceiverState{DS<:SatelliteChannelState,P<:PVTSolution}
    sat_channel_states::Dict{Int,DS} = Dict{Int,SatelliteChannelState}()
    pvt::P = PVTSolution()
    runtime::typeof(1ms) = 0ms
end

function reset_but_keep_decoders_and_pvt(rec_state::ReceiverState)
    sat_channel_states = Dict(
        prn => mark_out_of_lock(state) for (prn, state) in rec_state.sat_channel_states
    )
    ReceiverState(sat_channel_states, rec_state.pvt, 0ms)
end

include("channel.jl")
include("process.jl")
include("read_file.jl")
include("receive.jl")
include("gui.jl")
include("save_data.jl")
include("soapy_sdr_helper.jl")

function gnss_receiver_gui(;
    system = GPSL1(),
    sampling_freq = 2e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    run_time = 40u"s",
    num_ants = NumAnts(2)
)
    num_samples_acquisition = Int(upreferred(sampling_freq * acquisition_time))
    eval_num_samples = Int(upreferred(sampling_freq * run_time))
    Device(first(Devices())) do dev

        for crx in dev.rx
            crx.frequency = get_center_frequency(system)
            crx.sample_rate = sampling_freq
            crx.bandwidth = sampling_freq
            crx.gain_mode = true
        end

        stream = SoapySDR.Stream(ComplexF32, dev.rx)
        # Getting samples in chunks of `mtu`
        data_stream = stream_data(stream, eval_num_samples)

        # Satellite acquisition takes about 1s to process on a recent laptop
        # Let's take a buffer length of 5s to be on the safe side
        buffer_length = 5u"s"
        buffered_stream = membuffer(data_stream, ceil(Int, buffer_length * sampling_freq / stream.mtu))

        # Resizing the chunks to acquisition length
        reshunked_stream = rechunk(buffered_stream, num_samples_acquisition)

        # Performing GNSS acquisition and tracking
        data_channel = receive(reshunked_stream, system, sampling_freq; num_ants, num_samples = num_samples_acquisition)

        gui_channel = get_gui_data_channel(data_channel)

        # Display the GUI and block
        GNSSReceiver.gui(gui_channel)
    end
end

function gnss_write_to_file(;
    system = GPSL1(),
    sampling_freq = 2e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    run_time = 4u"s"
)
    num_samples_acquisition = Int(upreferred(sampling_freq * acquisition_time))
    eval_num_samples = Int(upreferred(sampling_freq * run_time))
    Device(first(Devices())) do dev

        for crx in dev.rx
            crx.frequency = get_center_frequency(system)
            crx.sample_rate = sampling_freq
            crx.bandwidth = sampling_freq
            crx.gain_mode = true
        end

        stream = SoapySDR.Stream(ComplexF32, dev.rx)
        # Getting samples in chunks of `mtu`
        data_stream = stream_data(stream, eval_num_samples)

        # Satellite acquisition takes about 1s to process on a recent laptop
        # Let's take a buffer length of 5s to be on the safe side
        buffer_length = 5u"s"
        buffered_stream = membuffer(data_stream, ceil(Int, buffer_length * sampling_freq / stream.mtu))

        # Resizing the chunks to acquisition length
        reshunked_stream = rechunk(buffered_stream, num_samples_acquisition)

        write_to_file(reshunked_stream, "/home/schoenbrod/Messungen/testdata")
    end
end

end
