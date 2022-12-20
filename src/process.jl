get_default_acq_threshold(system::GPSL1) = 43
get_default_acq_threshold(system::GalileoE1B) = 37

function process(
    receiver_state::ReceiverState{T, DS},
    acq_plan,
    fast_re_acq_plan,
    measurement,
    system::AbstractGNSS,
    sampling_freq;
    num_ants::NumAnts{N} = NumAnts(1),
    acquire_every = 10u"s",
    acq_threshold = get_default_acq_threshold(system),
    time_in_lock_before_pvt = 2u"s",
    interm_freq = 0.0u"Hz",
) where {N, T <: Complex, DS<:SatelliteChannelState}
    signal_duration = size(measurement, 1) / sampling_freq
    sat_channel_states = receiver_state.sat_channel_states
    acq_buffer = put(receiver_state.acq_buffer, measurement)
    acq_counter = receiver_state.acq_counter
    sat_channel_states = try_to_reacquire_lost_satellites(
        fast_re_acq_plan,
        sat_channel_states,
        acq_buffer,
        interm_freq,
        acq_threshold,
        num_ants,
    )
    if receiver_state.runtime >= acq_counter * acquire_every && isfull(acq_buffer)
        missing_satellites = vcat(
            filter(prn -> !(prn in keys(sat_channel_states)), 1:32),
            collect(
                keys(filter(((prn, state),) -> !is_in_lock(state), sat_channel_states)),
            ),
        )::Vector{Int}
        acq_res =
            acquire!(acq_plan, get_buffer(acq_buffer), missing_satellites; interm_freq)
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
                0.0u"s",
                0.0u"s",
                0,
            ) for res in acq_res_valid
        )
        sat_channel_states = merge(sat_channel_states, new_sat_channel_states)::Dict{Int,DS}
        acq_counter += 1
    end
    sat_channel_states_in_lock =
        filter(((prn, state),) -> is_in_lock(state), sat_channel_states)::Dict{Int,DS}
    track_results = Dict{Int, Tracking.TrackingResults}(
        prn => track(
            measurement,
            state.track_state,
            sampling_freq;
            intermediate_frequency = interm_freq,
        ) for (prn, state) in sat_channel_states_in_lock
    )
    sat_channel_states = Dict{Int,DS}(
        prn =>
            is_in_lock(state) ?
            SatelliteChannelState(
                get_state(track_results[prn]),
                decode(sat_channel_states[prn].decoder, get_bits(track_results[prn]), get_num_bits(track_results[prn])),
                update(sat_channel_states[prn].code_lock_detector, get_cn0(track_results[prn])),
                update(sat_channel_states[prn].carrier_lock_detector, get_filtered_prompt(track_results[prn])),
                state.time_in_lock + signal_duration,
                0.0u"s",
                0,
            ) : increase_time_out_of_lock(state, signal_duration) for
        (prn, state) in sat_channel_states
    )
    sat_channel_states_for_pvt = filter(
        ((prn, state),) ->
            is_in_lock(state) && state.time_in_lock > time_in_lock_before_pvt,
        sat_channel_states,
    )::Dict{Int,DS}
    sat_states = SatelliteState[
        SatelliteState(sat_channel_states[prn].decoder, track_results[prn]) for
        prn in keys(sat_channel_states_for_pvt)
    ]
    pvt = receiver_state.pvt
    if length(sat_states) >= 4
        pvt = calc_pvt(sat_states, pvt)
    end
    ReceiverState{T, DS, typeof(pvt)}(
        sat_channel_states,
        pvt,
        receiver_state.runtime + signal_duration,
        acq_counter,
        acq_buffer
    ),
    track_results
end

function view_part(measurement::AbstractMatrix, sample_range)
    view(measurement, sample_range, :)
end

function view_part(measurement::AbstractVector, sample_range)
    view(measurement, sample_range)
end

function try_to_reacquire_lost_satellites(
    fast_re_acq_plan,
    sat_channel_states::Dict{Int,DS},
    acq_buffer,
    interm_freq,
    acq_threshold,
    num_ants,
) where {DS<:SatelliteChannelState}
    return if isfull(acq_buffer)
        out_of_lock_sat_states = filter(sat_channel_states) do (prn, state)
            !is_in_lock(state) &&
                state.num_unsuccessful_reacquisition <= 10 &&
                state.num_unsuccessful_reacquisition^2 * 100u"ms" >= state.time_out_of_lock
        end
        acq_res = Dict(
            prn => acquire!(
                fast_re_acq_plan,
                get_buffer(acq_buffer),
                prn;
                interm_freq,
                doppler_offset = get_carrier_doppler(sat_state.track_state),
            ) for (prn, sat_state) in out_of_lock_sat_states
        )
        acq_res_valid = filter(((prn, res),) -> res.CN0 > acq_threshold, acq_res)
        new_sat_channel_states = Dict{Int,DS}(
            prn =>
                prn in keys(acq_res_valid) ?
                SatelliteChannelState(
                    TrackingState(
                        acq_res_valid[prn];
                        num_ants,
                        post_corr_filter = sat_state.track_state.post_corr_filter,
                    ),
                    sat_state.decoder,
                    CodeLockDetector(),
                    CarrierLockDetector(),
                    0.0u"s",
                    0.0u"s",
                    0,
                ) : increment_num_unsuccessful_reacquisition(sat_state) for
            (prn, sat_state) in out_of_lock_sat_states
        )
        merge(sat_channel_states, new_sat_channel_states)
    else
        sat_channel_states
    end
end