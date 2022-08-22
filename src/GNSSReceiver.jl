module GNSSReceiver

    using StaticArrays, GNSSDecoder, Tracking, PositionVelocityTime, GNSSSignals, Acquisition, Unitful, LinearAlgebra, JLD2
    using Unitful:dBHz, ms, Hz

    export 
        receive,
        reset_but_keep_decoders_and_pvt,
        read_files,
        save_data,
        get_gui_data_channel,
        tee

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
            0ms
        )
    end

    Base.@kwdef struct ReceiverState{
        DS<:SatelliteChannelState,
        P<:PVTSolution
    }
        sat_channel_states::Dict{Int,DS} = Dict{Int, SatelliteChannelState}()
        pvt::P = PVTSolution()
        runtime::typeof(1ms) = 0ms
    end

    function reset_but_keep_decoders_and_pvt(rec_state::ReceiverState)
        sat_channel_states = Dict(
            prn => mark_out_of_lock(state)
            for (prn, state) in rec_state.sat_channel_states
        )
        ReceiverState(
            sat_channel_states,
            rec_state.pvt,
            0ms
        )
    end

    include("channel.jl")
    include("process.jl")
    include("read_file.jl")
    include("receive.jl")
    include("gui.jl")
    include("save_data.jl")

end
