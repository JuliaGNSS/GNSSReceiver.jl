"""
    GNSSReceiver

A software GNSS receiver: it acquires, tracks, decodes and computes a
position/velocity/time (PVT) solution from GNSS signal samples, whether streamed live
from a SoapySDR device or replayed from recorded files.

The high-level entry points are [`gnss_receiver_gui`](@ref) (live device + terminal
GUI), [`gnss_write_to_file`](@ref) (record raw samples) and [`receive`](@ref) (the
acquire → track → decode → PVT pipeline over a sample channel). Results can be shown
with [`get_gui_data_channel`](@ref) or persisted with [`save_data`](@ref).
"""
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
    AstroTime,
    Accessors,
    Dictionaries,
    Dates

# Lock-free channel primitives now live in their own packages. The SoapySDR device
# streaming (`stream_data` / `SDRChannelConfig`) comes from SignalChannels' SoapySDR
# extension, which `using SoapySDR` above loads.
using PipeChannels: PipeChannel
using SignalChannels:
    SignalChannel,
    SDRChannelConfig,
    consume_channel,
    num_antenna_channels,
    spawn_signal_channel_thread,
    stream_data,
    write_to_file

export ReceiverState,
    receive,
    read_files,
    read_uint8_iq_file,
    save_data,
    collect_data,
    get_gui_data_channel,
    write_to_file,
    gnss_receiver_gui,
    gnss_write_to_file

include("lock_detector.jl")
include("beamformer.jl")
include("sample_buffer.jl")

using GNSSReceiver.SampleBuffers

struct ReceiverSatState{DS<:GNSSDecoderState}
    prn::Int
    decoder::DS
    code_lock_detector::CodeLockDetector
    carrier_lock_detector::CarrierLockDetector
    time_in_lock::typeof(1.0u"s")
    time_out_of_lock::typeof(1.0u"s")
    num_unsuccessful_reacquisition::Int
    carrier_doppler_for_reacquisition::typeof(1.0u"Hz")
end

function ReceiverSatState(
    acq::Acquisition.AcquisitionResults,
    decoder::Tracking.Maybe{<:GNSSDecoderState} = nothing,
    code_lock_cn0_threshold::typeof(1.0u"dBHz") = get_default_code_lock_cn0_threshold(
        acq.system,
    ),
)
    ReceiverSatState(
        acq.prn,
        isnothing(decoder) ? GNSSDecoderState(acq.system, acq.prn) : decoder,
        CodeLockDetector(; cn0_threshold = code_lock_cn0_threshold),
        CarrierLockDetector(),
        0.0u"s",
        0.0u"s",
        0,
        acq.carrier_doppler,
    )
end

function ReceiverSatState(
    system::AbstractGNSSSignal,
    prn::Int,
    code_lock_cn0_threshold::typeof(1.0u"dBHz") = get_default_code_lock_cn0_threshold(
        system,
    ),
)
    ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        CodeLockDetector(; cn0_threshold = code_lock_cn0_threshold),
        CarrierLockDetector(),
        0.0u"s",
        0.0u"s",
        0,
        0.0u"Hz",
    )
end

function is_in_lock(state::ReceiverSatState)
    is_in_lock(state.code_lock_detector) && is_in_lock(state.carrier_lock_detector)
end

function increase_time_out_of_lock(state::ReceiverSatState, time::Unitful.Time)
    @reset state.time_in_lock = 0.0u"s"
    @reset state.time_out_of_lock = state.time_out_of_lock + time
    @reset state.num_unsuccessful_reacquisition = 0
    return state
end

function increment_num_unsuccessful_reacquisition(state::ReceiverSatState)
    @reset state.num_unsuccessful_reacquisition = state.num_unsuccessful_reacquisition + 1
    return state
end

const MultipleReceiverSatStates{N,I,DS} = NTuple{N,Dictionary{I,ReceiverSatState{DS}}}

struct ReceiverState{
    RS<:MultipleReceiverSatStates,
    TS<:TrackState,
    AB<:SampleBuffer,
    P<:PVTSolution,
    PB<:AbstractVector{<:SatelliteState},
}
    track_state::TS
    receiver_sat_states::RS
    acquisition_buffer::AB
    pvt::P
    # Reused across PVT cycles: `update_pvt` refills this in place each cycle
    # instead of allocating a fresh `Vector{SatelliteState}` (see `update_pvt`).
    pvt_sat_state_buffer::PB
    runtime::typeof(1.0u"s")
    last_time_acquisition_ran::typeof(1.0u"s")
    last_time_pvt_ran::typeof(1.0u"s")
    num_samples_processed::Int
end

get_num_ants(num_ants::NumAnts{N}) where {N} = N

create_post_corr_filter(num_ants::NumAnts{N}) where {N} =
    EigenBeamformer(get_num_ants(num_ants))
create_post_corr_filter(num_ants::NumAnts{1}) = Tracking.DefaultPostCorrFilter()

"""
    ReceiverState(T, system; num_samples_for_acquisition, num_ants = NumAnts(1), kwargs...)

Build the initial receiver state for tracking `system`, where `T` is the element type
of the incoming signal samples (e.g. `ComplexF64` or `Complex{Int16}`).

`num_samples_for_acquisition` sizes the acquisition sample buffer. `num_ants` selects
single- versus multi-antenna processing and, together with `correlator`,
`post_corr_filter` and `doppler_estimator`, pins the concrete satellite-slot type so
that satellites handed over from acquisition merge into the track state without
introducing type instability.
"""
function ReceiverState(
    ::Type{T}, # Must be the same type as the incoming signal
    system;
    num_samples_for_acquisition,
    num_ants::NumAnts{N} = NumAnts(1),
    acquisition_buffer = SampleBuffer(T, num_samples_for_acquisition),
    correlator::Tracking.AbstractCorrelator = Tracking.get_default_correlator(
        system,
        num_ants,
    ),
    post_corr_filter = create_post_corr_filter(num_ants),
    doppler_estimator::Tracking.AbstractDopplerEstimator = ConventionalPLLAndDLL(),
) where {T,N}
    # Build an empty TrackState whose satellite slot type is pinned to the
    # requested correlator / post-corr-filter / estimator. Sats handed over
    # from acquisition later (`create_sat_state_from_acq`) are built the same
    # way, so `merge_sats` sees a matching slot type.
    template_sat = TrackedSat(
        system,
        0,
        0.0,
        0.0u"Hz";
        num_ants,
        correlator,
        post_corr_filter,
        doppler_estimator,
    )
    satellites = Dictionary{Int,typeof(template_sat)}(Int[], typeof(template_sat)[])
    signal_group = SignalGroup(get_band(system), satellites, (system,), num_ants)
    track_state = TrackState((; default = signal_group), doppler_estimator)
    decoder = GNSSDecoderState(system, 1)
    receiver_sat_states = (Dictionary{Int64,ReceiverSatState{typeof(decoder)}}(),)
    pvt = PositionVelocityTime.PVTSolution()
    # Preallocated (empty) buffer reused by `update_pvt`. `SatelliteState`'s code
    # phase / carrier phase are `Float64`; it embeds the decoder by value, so the
    # element type is pinned to this receiver's decoder and system types.
    pvt_sat_state_buffer = SatelliteState{Float64,typeof(decoder),typeof(system)}[]
    ReceiverState(
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        pvt,
        pvt_sat_state_buffer,
        0.0u"s",
        -Inf * 1.0u"s",
        -Inf * 1.0u"s",
        0,
    )
end

include("read_file.jl")
include("receive.jl")
include("process.jl")
include("gui.jl")
include("save_data.jl")

"""
    gnss_receiver_gui(; system = GPSL1CA(), sampling_freq = 2e6u"Hz", kwargs...)

Acquire, track and compute a PVT solution from a live SoapySDR device and display the
result in a live terminal GUI. Blocks until the stream ends.

The device selected by `dev_args` is configured for `sampling_freq`, tuned to
`system`'s centre frequency and streamed for `run_time` — all handled by SignalChannels'
`stream_data`, which also buffers a few seconds of signal for acquisition headroom and
rechunks to the acquisition length. Satellite acquisition uses `acquisition_time` of
signal — a longer time improves the acquisition SNR at a higher computational cost and
must exceed one code period. Set `gain` to a fixed value or leave it `nothing` for
automatic gain control; `antenna`, `interm_freq` and `num_ants` override the RF front end
and antenna-channel count (one identical channel config is used per antenna).

Samples are streamed as `stream_type` (default `ComplexF32`). Pass `stream_type =
Complex{Int16}` to use Tracking's fast integer backend on an Int16-native device, in
which case `max_meas` (the front-end full-scale) is required; it is ignored for float
stream types.
"""
function gnss_receiver_gui(;
    system = GPSL1CA(),
    sampling_freq = 2e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    run_time = 40u"s",
    num_ants = NumAnts(2),
    dev_args = first(Devices()),
    interm_freq = 0.0u"Hz",
    gain::Union{Nothing,<:Unitful.Gain} = nothing,
    antenna = nothing,
    stream_type = ComplexF32,
    # Front-end full-scale for the integer downconvert-and-correlator, required only
    # when `stream_type` is `Complex{Int16}` (see `receive`).
    max_meas = nothing,
)
    num_samples_acquisition = Int(upreferred(sampling_freq * acquisition_time))
    eval_num_samples = Int(upreferred(sampling_freq * run_time))

    # One identical RX config per antenna channel. SignalChannels' `stream_data` opens
    # and configures the device (sample rate, bandwidth, centre frequency, gain / AGC,
    # antenna), buffers `buffer_time` of signal for acquisition headroom, and rechunks
    # the stream to the acquisition length.
    channel_config = SDRChannelConfig(;
        sample_rate = sampling_freq,
        frequency = get_center_frequency(system),
        bandwidth = sampling_freq,
        gain,
        antenna,
    )
    configs = ntuple(_ -> channel_config, get_num_ants(num_ants))

    data_stream, warning_channel = stream_data(
        stream_type,
        dev_args,
        configs,
        eval_num_samples;
        chunk_size = num_samples_acquisition,
        buffer_time = 5u"s",
    )
    # Surface SDR overflow/timeout warnings without blocking the pipeline.
    warning_task = Threads.@spawn for w in warning_channel
        @warn "SDR stream warning" type = w.type time = w.time_str
    end
    Base.errormonitor(warning_task)

    # Performing GNSS acquisition and tracking
    data_channel = receive(
        data_stream,
        system,
        sampling_freq;
        num_ants,
        interm_freq,
        max_meas,
    )

    gui_channel = get_gui_data_channel(data_channel)

    # Display the GUI and block
    GNSSReceiver.gui(gui_channel)
end

"""
    gnss_write_to_file(; system = GPSL1CA(), sampling_freq = 2e6u"Hz", run_time = 4u"s",
                       dev_args = first(Devices()), output_file = "gnss_test_data",
                       gain = 50u"dB")

Stream `run_time` of raw `Complex{Int16}` samples from a SoapySDR device (tuned to
`system`'s centre frequency, at `gain` or AGC when `gain === nothing`) to `output_file`,
for later offline replay with [`read_files`](@ref) and [`receive`](@ref). Blocks until the
recording finishes.
"""
function gnss_write_to_file(;
    system = GPSL1CA(),
    sampling_freq = 2e6u"Hz",
    run_time = 4u"s",
    dev_args = first(Devices()),
    output_file = "gnss_test_data",
    gain::Union{Nothing,<:Unitful.Gain} = 50u"dB",
)
    eval_num_samples = Int(upreferred(sampling_freq * run_time))

    channel_config = SDRChannelConfig(;
        sample_rate = sampling_freq,
        frequency = get_center_frequency(system),
        bandwidth = sampling_freq,
        gain,
    )
    # Record raw Complex{Int16} samples, matching `read_files`' default element type.
    data_stream, warning_channel =
        stream_data(Complex{Int16}, dev_args, channel_config, eval_num_samples)

    wait(write_to_file(data_stream, output_file))

    # Recording is done and the stream is closed; drain any SDR warnings.
    for w in warning_channel
        @warn "SDR stream warning" type = w.type time = w.time_str
    end
end

"""
    receive_and_gui(files; system = GPSL1CA(), sampling_freq = 5e6u"Hz", kwargs...)

Replay recorded sample `files` through the receiver and show the live terminal GUI,
returning once any key is pressed (which stops the stream).

`files` holds one path per antenna channel (`num_ants` must match). Samples are read
as `type` elements; `max_meas` (the front-end full-scale) is required for the default
`Complex{Int16}` recordings and ignored for float types. `clock_drift` rescales the
sampling and intermediate frequencies to compensate for a known front-end clock
offset.
"""
function receive_and_gui(
    files;
    clock_drift = 0.0,
    system = GPSL1CA(),
    sampling_freq = 5e6u"Hz",
    type = Complex{Int16},
    num_ants = NumAnts(4),
    # Front-end full-scale for the integer downconvert-and-correlator, required
    # for the default `Complex{Int16}` file type (see `receive`). Leave `nothing`
    # for float file types.
    max_meas = nothing,
)
    #    files = map(i -> "/mnt/data_disk/measurementComplex{Int16}$i.dat", 1:2)
    close_stream_event = Base.Event()
    adjusted_sample_freq = sampling_freq * (1 - clock_drift)

    num_samples_to_receive = Int(upreferred(sampling_freq * 4u"ms"))
    measurement_channel =
        read_files(files, num_samples_to_receive, close_stream_event; type)

    # Let's receive GPS L1 signals
    data_channel = receive(
        measurement_channel,
        system,
        adjusted_sample_freq;
        num_ants,
        interm_freq = clock_drift * get_center_frequency(system),
        max_meas,
    )
    # Get gui channel from data channel
    gui_channel = get_gui_data_channel(data_channel)
    # Hook up GUI
    Base.errormonitor(
        @async GNSSReceiver.gui(
            gui_channel;
            construct_gui_panels = make_construct_gui_panels(),
        )
    )

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
        microseconds =
            isnothing(gui_data.pvt.time) ? nothing : microsecond(gui_data.pvt.time)
        milliseconds =
            isnothing(gui_data.pvt.time) ? nothing : millisecond(gui_data.pvt.time)
        panels / Panel(
            "Runtime: $(gui_data.runtime)\nTime: $(gui_data.pvt.time)\nMilliseconds: $milliseconds\nMicroseconds: $microseconds\nNanoseconds: $nanoseconds";
            fit = true,
        )
    end
end

end
