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
    SoapySDR,
    AstroTime

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
include("acquisition_buffer.jl")

struct SatelliteChannelState{
    DS<:GNSSDecoderState,
    TS<:TrackingState,
}
    track_state::TS
    decoder::DS
    code_lock_detector::CodeLockDetector
    carrier_lock_detector::CarrierLockDetector
    time_in_lock::typeof(1.0u"s")
    time_out_of_lock::typeof(1.0u"s")
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
        0.0u"s",
        0.0u"s",
        0
    )
end

function increase_time_out_of_lock(state::SatelliteChannelState, time::Unitful.Time)
    SatelliteChannelState(
        state.track_state,
        state.decoder,
        state.code_lock_detector,
        state.carrier_lock_detector,
        0.0u"s",
        state.time_out_of_lock + time,
        0,
    )
end

function increment_num_unsuccessful_reacquisition(state::SatelliteChannelState)
    SatelliteChannelState(
        state.track_state,
        state.decoder,
        state.code_lock_detector,
        state.carrier_lock_detector,
        state.time_in_lock,
        state.time_out_of_lock,
        state.num_unsuccessful_reacquisition + 1,
    )
end

struct ReceiverState{T, DS<:SatelliteChannelState,P<:PVTSolution}
    sat_channel_states::Dict{Int,DS}
    pvt::P
    runtime::typeof(1.0u"s")
    acq_counter::Int
    acq_buffer::AcquisitionBuffer{T}
end

function ReceiverState(T::Type, num_samples, acq_time::typeof(1u"ms"), system, num_ants::NumAnts{N}) where N
    acq_time.val >= 4 || throw(ArgumentError("Acquisition time must be at least 4ms"))
    track_state = TrackingState(1, system, 1.0u"Hz", 1.0; num_ants,
    post_corr_filter = N == 1 ? Tracking.DefaultPostCorrFilter() :
                       EigenBeamformer(N))
    decoder = GNSSDecoderState(system, 1)
    pvt = PositionVelocityTime.PVTSolution()
    sat_channel_type = SatelliteChannelState{typeof(decoder), typeof(track_state)}
    acq_buffer = AcquisitionBuffer(T, num_samples, acq_time.val >> 2)
    ReceiverState{T, sat_channel_type, typeof(pvt)}(
        Dict{Int, sat_channel_type}(),
        pvt,
        0.0u"s",
        0,
        acq_buffer
    )
end

function reset_but_keep_decoders_and_pvt(rec_state::ReceiverState)
    sat_channel_states = Dict(
        prn => mark_out_of_lock(state) for (prn, state) in rec_state.sat_channel_states
    )
    acq_buffer = AcquisitionBuffer(rec_state.acq_buffer.buffer, rec_state.acq_buffer.size, 0)
    ReceiverState(sat_channel_states, rec_state.pvt, 0.0ms, 0, acq_buffer)
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

function receive_and_gui(files;clock_drift = 0.0, system = GPSL1(), sampling_freq = 5e6u"Hz", type = Complex{Int16}, num_ants = NumAnts(4))
    #    files = map(i -> "/mnt/data_disk/measurementComplex{Int16}$i.dat", 1:2)
    close_stream_event = Base.Event()
    adjusted_sample_freq = sampling_freq * (1 - clock_drift)
    
    num_samples_to_receive = Int(upreferred(sampling_freq * 4u"ms"))
    measurement_channel = read_files(files, num_samples_to_receive, close_stream_event; type)

    # Let's receive GPS L1 signals
    data_channel = receive(measurement_channel, system, adjusted_sample_freq; num_ants, interm_freq = clock_drift * get_center_frequency(system))
    # Get gui channel from data channel
    gui_channel = get_gui_data_channel(data_channel)
    # Hook up GUI
    Base.errormonitor(@async GNSSReceiver.gui(gui_channel; construct_gui_panels = make_construct_gui_panels()))
    
        # Read any input to close
    t = REPL.TerminalMenus.terminal
    REPL.Terminals.raw!(t, true)
    char = read(stdin, Char) 
    REPL.Terminals.raw!(t, false)
    notify(close_stream_event)
end
function make_construct_gui_panels()
    function construct_gui_panels(gui_data, num_dots)
        panels = GNSSReceiver.construct_gui_panels(gui_data, num_dots)
        nanoseconds = isnothing(gui_data.pvt.time) ? nothing : nanosecond(gui_data.pvt.time)
        microseconds = isnothing(gui_data.pvt.time) ? nothing : microsecond(gui_data.pvt.time)
        milliseconds = isnothing(gui_data.pvt.time) ? nothing : millisecond(gui_data.pvt.time)
        panels / Panel("Runtime: $(gui_data.runtime)\nTime: $(gui_data.pvt.time)\nMilliseconds: $milliseconds\nMicroseconds: $microseconds\nNanoseconds: $nanoseconds", fit = true)
    end
end

end
