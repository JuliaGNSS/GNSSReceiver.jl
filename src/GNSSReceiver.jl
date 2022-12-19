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
    write_to_file,
    gnss_receiver_gui,
    gnss_write_to_file

include("lock_detector.jl")
include("beamformer.jl")

struct SatelliteChannelState{
    DS<:GNSSDecoderState,
    TS<:TrackingState,
}
    track_state::TS
    decoder::DS
    code_lock_detector::CodeLockDetector
    carrier_lock_detector::CarrierLockDetector
    time_in_lock::typeof(1ms)
    time_out_of_lock::typeof(1ms)
    num_unsuccessful_reacquisition::Int
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
        0ms,
        0
    )
end

function increase_time_out_of_lock(state::SatelliteChannelState, time::typeof(1ms))
    SatelliteChannelState(
        state.track_state,
        state.decoder,
        state.code_lock_detector,
        state.carrier_lock_detector,
        0ms,
        state.time_out_of_lock + time,
        0,
    )
end

struct ReceiverState{DS<:SatelliteChannelState,P<:PVTSolution}
    sat_channel_states::Dict{Int,DS}
    pvt::P
    runtime::typeof(1ms)
end

function ReceiverState(system, num_ants::NumAnts{N}) where N
    track_state = TrackingState(1, system, 1.0Hz, 1.0; num_ants,
    post_corr_filter = N == 1 ? Tracking.DefaultPostCorrFilter() :
                       EigenBeamformer(N))
    decoder = GNSSDecoderState(system, 1)
    pvt = PositionVelocityTime.PVTSolution()
    sat_channel_type = SatelliteChannelState{typeof(decoder), typeof(track_state)}
    ReceiverState{sat_channel_type, typeof(pvt)}(
        Dict{Int, sat_channel_type}(),
        pvt,
        0ms
    )
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
    num_ants = NumAnts(2),
    dev_args = first(Devices()),
    interm_freq = 0.0u"Hz",
    gain::Union{Nothing, <:Unitful.Gain} = nothing,
    antenna = nothing
)
    num_samples_acquisition = Int(upreferred(sampling_freq * acquisition_time))
    eval_num_samples = Int(upreferred(sampling_freq * run_time))
    Device(dev_args) do dev

        for crx in dev.rx
            if !isnothing(antenna)
                crx.antenna = antenna
            end
            crx.sample_rate = sampling_freq
            crx.bandwidth = sampling_freq
            if isnothing(gain)
                crx.gain_mode = true
            else
                crx.gain = gain
            end
            crx.frequency = get_center_frequency(system)
        end

        stream = SoapySDR.Stream(first(dev.rx).native_stream_format, dev.rx)

        # Getting samples in chunks of `mtu`
        data_stream = stream_data(stream, eval_num_samples)

        # Satellite acquisition takes about 1s to process on a recent laptop
        # Let's take a buffer length of 5s to be on the safe side
        buffer_length = 5u"s"
        buffered_stream = membuffer(data_stream, ceil(Int, buffer_length * sampling_freq / stream.mtu))

        # Resizing the chunks to acquisition length
        reshunked_stream = rechunk(buffered_stream, num_samples_acquisition)

        # Performing GNSS acquisition and tracking
        data_channel = receive(
            reshunked_stream,
            system,
            sampling_freq;
            num_ants,
            num_samples = num_samples_acquisition,
            interm_freq
        )

        gui_channel = get_gui_data_channel(data_channel)

        # Display the GUI and block
        GNSSReceiver.gui(gui_channel)
    end
end

function gnss_write_to_file(;
    system = GPSL1(),
    sampling_freq = 2e6u"Hz",
    run_time = 4u"s",
    dev_args = first(Devices()),
    output_file = "gnss_test_data"
)
    eval_num_samples = Int(upreferred(sampling_freq * run_time))
    Device(dev_args) do dev

        for crx in dev.rx
            crx.frequency = get_center_frequency(system)
            crx.sample_rate = sampling_freq
            crx.bandwidth = sampling_freq
            crx.gain_mode = true
        end

        stream = SoapySDR.Stream(first(dev.rx).native_stream_format, dev.rx)
        
        # Getting samples in chunks of `mtu`
        data_stream = stream_data(stream, eval_num_samples)

        write_to_file(data_stream, output_file)
    end
end

end
