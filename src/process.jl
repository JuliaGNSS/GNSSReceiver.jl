get_default_acq_threshold(system::GPSL1) = 43
get_default_acq_threshold(system::GalileoE1B) = 37

function process(
    receiver_state::ReceiverState{DS},
    acq_plan,
    fast_re_acq_plan,
    measurement,
    system::AbstractGNSS,
    sampling_freq;
    num_ants::NumAnts{N} = NumAnts(1),
    acquire_every = 10000ms,
    acq_threshold = get_default_acq_threshold(system),
    time_in_lock_before_pvt = 2000ms,
    interm_freq = 0.0u"Hz",
) where {N, DS <: SatelliteChannelState}
    signal_duration = convert(typeof(1ms), size(measurement, 1) / sampling_freq)
    signal_duration % 1ms == 0ms ||
        throw(ArgumentError("Signal length must be multiples of 1ms"))
    sat_channel_states = receiver_state.sat_channel_states
    sat_channel_states = try_to_reacquire_lost_satellites(
        fast_re_acq_plan,
        sat_channel_states,
        measurement,
        interm_freq,
        acq_threshold,
        num_ants
    )
    if receiver_state.runtime % acquire_every == 0ms
        missing_satellites = vcat(
            filter(prn -> !(prn in keys(sat_channel_states)), 1:32),
            collect(
                keys(filter(((prn, state),) -> !is_in_lock(state), sat_channel_states)),
            ),
        )::Vector{Int}
        acq_res = acquire!(acq_plan, view(measurement, :, 1), missing_satellites; interm_freq)
        acq_res_valid = filter(res -> res.CN0 > acq_threshold, acq_res)
        new_sat_channel_states = Dict{Int,DS}(
            res.prn => SatelliteChannelState(
                TrackingState(
                    res;
                    num_ants,
                    post_corr_filter = N == 1 ? Tracking.DefaultPostCorrFilter() :
                                       EigenBeamformer(N),
                ),
                res.prn in keys(sat_channel_states) ?
                sat_channel_states[res.prn].decoder :
                GNSSDecoderState(system, res.prn),
                CodeLockDetector(),
                CarrierLockDetector(),
                0ms,
                0ms,
                0,
            ) for res in acq_res_valid
        )
        merge!(sat_channel_states, new_sat_channel_states)::Dict{Int,DS}
    end
    sat_channel_states_in_lock = filter(((prn, state),) -> is_in_lock(state), sat_channel_states)::Dict{Int,DS}
    track_results = Dict{Int,Vector{Tracking.TrackingResults}}(
        prn => track_measurement_parts(
            state.track_state,
            measurement,
            sampling_freq,
            signal_duration,
            interm_freq,
        ) for
        (prn, state) in sat_channel_states_in_lock
    )
    sat_channel_states = Dict{Int,DS}(
        prn =>
            is_in_lock(state) ?
            SatelliteChannelState(
                get_state(track_results[prn][end]),
                foldl(
                    track_results[prn];
                    init = sat_channel_states[prn].decoder,
                ) do prev_decoder, track_res
                    decode(prev_decoder, get_bits(track_res), get_num_bits(track_res))
                end,
                foldl(
                    (prev_detector, cn0) -> update(prev_detector, cn0),
                    get_cn0.(track_results[prn]);
                    init = sat_channel_states[prn].code_lock_detector,
                ),
                foldl(
                    (prev_detector, prompt) -> update(prev_detector, prompt),
                    get_filtered_prompt.(track_results[prn]);
                    init = sat_channel_states[prn].carrier_lock_detector,
                ),
                state.time_in_lock + signal_duration,
                0ms,
                0,
            ) : increase_time_out_of_lock(state, signal_duration) for (prn, state) in sat_channel_states
    )
    sat_channel_states_for_pvt = filter(
        ((prn, state),) ->
            is_in_lock(state) && state.time_in_lock > time_in_lock_before_pvt,
        sat_channel_states,
    )::Dict{Int,DS}
    sat_states = SatelliteState[
        SatelliteState(sat_channel_states[prn].decoder, track_results[prn][end]) for
        prn in keys(sat_channel_states_for_pvt)
    ]
    pvt = receiver_state.pvt
    if length(sat_states) >= 4
        pvt = calc_pvt(sat_states, pvt)
    end
    ReceiverState{DS, typeof(pvt)}(sat_channel_states, pvt, receiver_state.runtime + signal_duration),
    track_results
end

function view_part(measurement::AbstractMatrix, sample_range)
    view(measurement, sample_range, :)
end

function view_part(measurement::AbstractVector, sample_range)
    view(measurement, sample_range)
end

function track_measurement_parts(track_state, measurement, sampling_freq, signal_duration, intermediate_frequency)
    samples = Int(upreferred(1ms * sampling_freq))
    num_parts = Int(upreferred(signal_duration / 1ms))
    first_track_result =
        track(view_part(measurement, 1:samples), track_state, sampling_freq; intermediate_frequency)
    track_results = Vector{typeof(first_track_result)}(undef, num_parts)
    track_results[1] = first_track_result
    track_state = get_state(first_track_result)
    for i = 2:num_parts
        track_results[i] = track(
            view_part(measurement, (i-1)*samples+1:i*samples),
            track_state,
            sampling_freq;
            intermediate_frequency
        )
        track_state = get_state(track_results[i])
    end
    track_results
end

function try_to_reacquire_lost_satellites(
    fast_re_acq_plan,
    sat_channel_states::Dict{Int, DS},
    measurement,
    interm_freq,
    acq_threshold,
    num_ants
) where {DS <: SatelliteChannelState}
    out_of_lock_sat_states = filter(sat_channel_states) do (prn, state)
        !is_in_lock(state) && state.num_unsuccessful_reacquisition <= 10 &&
            state.num_unsuccessful_reacquisition^2 * 100ms >= state.time_out_of_lock
    end
    acq_res = Dict(
        prn => acquire!(
            fast_re_acq_plan,
            view(measurement, :, 1),
            prn;
            interm_freq,
            doppler_offset = get_carrier_doppler(sat_state.track_state),
        ) for (prn, sat_state) in out_of_lock_sat_states
    )
    acq_res_valid = filter(((prn, res),) -> res.CN0 > acq_threshold, acq_res)
    new_sat_channel_states = Dict{Int,DS}(
        prn => SatelliteChannelState(
            TrackingState(
                res;
                num_ants,
                post_corr_filter = 
                    sat_channel_states[prn].track_state.post_corr_filter,
            ),
            sat_channel_states[prn].decoder,
            CodeLockDetector(),
            CarrierLockDetector(),
            0ms,
            0ms,
            0,
        ) for (prn, res) in acq_res_valid
    )
    merge!(sat_channel_states, new_sat_channel_states)
end