get_default_acq_threshold(system::GPSL1) = 43
get_default_acq_threshold(system::GalileoE1B) = 37

function process(
    receiver_state::ReceiverState{RS,TS,AB,P},
    acq_plan,
    fast_re_acq_plan,
    measurement,
    system::AbstractGNSS,
    sampling_freq;
    num_ants::NumAnts{N} = NumAnts(1),
    acquire_every = 10u"s",
    acq_threshold = get_default_acq_threshold(system),
    time_in_lock_before_calculating_pvt = 2u"s",
    interm_freq = 0.0u"Hz",
) where {N,RS,TS,AB,P}
    num_samples = size(measurement, 1)
    signal_duration = num_samples / sampling_freq
    # Currently this only supports a single system. Hence [1]
    receiver_sat_states = receiver_state.receiver_sat_states[1]
    track_state = receiver_state.track_state
    acquisition_buffer = buffer(receiver_state.acquisition_buffer, @view(measurement[:, 1]))
    runtime = receiver_state.runtime
    last_time_acquisition_ran = receiver_state.last_time_acquisition_ran

    track_state, receiver_sat_states, last_time_acquisition_ran = acquire_satellites(
        acq_plan,
        fast_re_acq_plan,
        acquisition_buffer,
        track_state,
        receiver_sat_states,
        interm_freq,
        acq_threshold,
        runtime,
        num_ants,
        last_time_acquisition_ran,
        acquire_every,
        receiver_state.num_samples_processed,
    )

    track_state =
        track(measurement, track_state, sampling_freq; intermediate_frequency = interm_freq)

    receiver_sat_states =
        update_receiver_sat_states(receiver_sat_states, track_state, signal_duration)

    pvt = update_pvt(
        system,
        receiver_state.pvt,
        receiver_sat_states,
        track_state,
        time_in_lock_before_calculating_pvt,
    )

    track_state = filter_in_lock_sats(receiver_sat_states, track_state)

    ReceiverState{RS,TS,AB,P}(
        track_state,
        (receiver_sat_states,),
        acquisition_buffer,
        pvt,
        receiver_state.runtime + signal_duration,
        last_time_acquisition_ran,
        receiver_state.num_samples_processed + num_samples,
    )
end

function filter_in_lock_sats(receiver_sat_states, track_state)
    tracked_prns_out_of_lock =
        filter(receiver_sat_states) do receiver_sat_state
            !is_in_lock(receiver_sat_state) &&
                receiver_sat_state.prn in keys(get_sat_states(track_state))
        end |> get_prns

    filter_out_sats(track_state, tracked_prns_out_of_lock)
end

function update_pvt(
    system,
    pvt,
    receiver_sat_states,
    track_state,
    time_in_lock_before_calculating_pvt,
)
    receiver_sat_states_ready_for_pvt = filter(receiver_sat_states) do receiver_sat_state
        is_in_lock(receiver_sat_state) &&
            receiver_sat_state.time_in_lock > time_in_lock_before_calculating_pvt
    end

    pvt_satellite_states =
        map(receiver_sat_states_ready_for_pvt.values) do receiver_sat_state
            SatelliteState(
                receiver_sat_state.decoder,
                system,
                get_sat_state(track_state, receiver_sat_state.prn),
            )
        end

    if length(pvt_satellite_states) >= 4
        pvt = calc_pvt(pvt_satellite_states, pvt)
    end
    return pvt
end

function update_receiver_sat_states(receiver_sat_states, track_state, signal_duration)
    return map(receiver_sat_states) do receiver_sat_state
        if is_in_lock(receiver_sat_state)
            prn = receiver_sat_state.prn
            ReceiverSatState(
                prn,
                decode(
                    receiver_sat_state.decoder,
                    get_bits(track_state, prn),
                    get_num_bits(track_state, prn),
                ),
                update(
                    receiver_sat_state.code_lock_detector,
                    estimate_cn0(track_state, prn),
                ),
                update(
                    receiver_sat_state.carrier_lock_detector,
                    get_last_fully_integrated_filtered_prompt(track_state, prn),
                ),
                receiver_sat_state.time_in_lock + signal_duration,
                0.0u"s",
                0,
                get_carrier_doppler(track_state, prn),
            )
        else
            increase_time_out_of_lock(receiver_sat_state, signal_duration)
        end
    end
end

get_prns(acquisition_results::AbstractVector{<:Acquisition.AcquisitionResults}) =
    map(res -> res.prn, acquisition_results)
get_prns(receiver_sat_states::Dictionary{<:Any,<:ReceiverSatState}) =
    map(receiver_sat_state -> receiver_sat_state.prn, collect(receiver_sat_states))

function create_sat_state_from_acq(
    acq::Acquisition.AcquisitionResults,
    track_state::TrackState,
    num_ants::NumAnts{N},
) where {N}
    correlator = Tracking.get_default_correlator(acq.system, num_ants)
    eltype(track_state.multiple_system_sats_state[1].states)(
        acq.prn,
        acq.code_phase,
        acq.carrier_doppler * get_code_center_frequency_ratio(acq.system),
        0.0,
        acq.carrier_doppler,
        0,
        1,
        correlator,
        correlator,
        complex(0.0, 0.0),
        Tracking.MomentsCN0Estimator(100),
        Tracking.BitBuffer(),
        create_post_corr_filter(num_ants),
    )
end

function update_states_from_acquisition_results(
    acquisition_results,
    acquisition_threshold,
    track_state,
    receiver_sat_states,
    num_ants,
)
    acq_res_valids = filter(res -> res.CN0 > acquisition_threshold, acquisition_results)

    new_receiver_sat_states = map(acq_res_valids) do res
        ReceiverSatState(
            res,
            res.prn in keys(receiver_sat_states) ?
            receiver_sat_states[res.prn].decoder : nothing,
        )
    end

    new_sat_states = eltype(track_state.multiple_system_sats_state[1].states)[
        create_sat_state_from_acq(acq, track_state, num_ants) for acq in acq_res_valids
    ]

    new_track_state = merge_sats(track_state, 1, new_sat_states)

    new_receiver_sat_states_dictionary = merge(
        receiver_sat_states,
        Dictionary(get_prns(acq_res_valids), new_receiver_sat_states),
    )
    new_track_state, new_receiver_sat_states_dictionary
end

function acquire_satellites(
    acq_plan,
    fast_re_acq_plan,
    acquisition_buffer,
    track_state,
    receiver_sat_states,
    interm_freq,
    acq_threshold,
    runtime,
    num_ants,
    last_time_acquisition_ran,
    acquire_every,
    num_samples_processed,
)
    track_state, receiver_sat_states = try_to_reacquire_lost_satellites(
        fast_re_acq_plan,
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        interm_freq,
        acq_threshold,
        num_ants,
        num_samples_processed,
    )

    if SampleBuffers.isfull(acquisition_buffer) &&
       runtime - last_time_acquisition_ran >= acquire_every
        missing_satellites = vcat(
            filter(prn -> !(prn in keys(receiver_sat_states)), 1:32),
            collect(keys(filter(state -> !is_in_lock(state), receiver_sat_states))),
        )
        acq_res = acquire!(
            acq_plan,
            get_samples(acquisition_buffer),
            missing_satellites;
            interm_freq,
        )

        corrected_acq_res = map(acq_res) do res
            advance_code_phase(
                res,
                num_samples_processed - (get_first_sample_counter(acquisition_buffer) - 1),
            )
        end

        track_state, receiver_sat_states = update_states_from_acquisition_results(
            corrected_acq_res,
            acq_threshold,
            track_state,
            receiver_sat_states,
            num_ants,
        )

        last_time_acquisition_ran = runtime
    end
    track_state, receiver_sat_states, last_time_acquisition_ran
end

function advance_code_phase(acq_res::Acquisition.AcquisitionResults, num_samples)
    code_phase = mod(
        (
            get_code_frequency(acq_res.system) +
            acq_res.carrier_doppler * get_code_center_frequency_ratio(acq_res.system)
        ) * num_samples / acq_res.sampling_frequency + acq_res.code_phase,
        get_code_length(acq_res.system),
    )
    Acquisition.AcquisitionResults(
        acq_res.system,
        acq_res.prn,
        acq_res.sampling_frequency,
        acq_res.carrier_doppler,
        code_phase,
        acq_res.CN0,
        acq_res.noise_power,
        acq_res.power_bins,
        acq_res.dopplers,
    )
end

function try_to_reacquire_lost_satellites(
    fast_re_acq_plan,
    track_state,
    receiver_sat_states,
    acquisition_buffer,
    interm_freq,
    acq_threshold,
    num_ants,
    num_samples_processed,
)
    if SampleBuffers.isfull(acquisition_buffer)
        out_of_lock_receiver_sat_states = filter(receiver_sat_states) do state
            !is_in_lock(state) &&
                state.num_unsuccessful_reacquisition <= 10 &&
                state.num_unsuccessful_reacquisition^2 * 100u"ms" >= state.time_out_of_lock
        end

        acq_res = map(out_of_lock_receiver_sat_states.values) do receiver_sat_state
            acquire!(
                fast_re_acq_plan,
                get_samples(acquisition_buffer),
                receiver_sat_state.prn;
                interm_freq,
                doppler_offset = receiver_sat_state.carrier_doppler_for_reacquisition,
            )
        end

        corrected_acq_res = map(acq_res) do res
            advance_code_phase(
                res,
                num_samples_processed - (get_first_sample_counter(acquisition_buffer) - 1),
            )
        end

        return update_states_from_acquisition_results(
            corrected_acq_res,
            acq_threshold,
            track_state,
            receiver_sat_states,
            num_ants,
        )
    else
        return track_state, receiver_sat_states
    end
end
