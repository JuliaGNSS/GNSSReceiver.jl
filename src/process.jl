get_default_code_lock_cn0_threshold(system::GPSL1CA) = 30.0u"dBHz"
get_default_code_lock_cn0_threshold(system::GalileoE1B) = 30.0u"dBHz"

function process(
    receiver_state::ReceiverState{RS,TS,AB,P},
    acq_plan,
    measurement,
    system::AbstractGNSSSignal,
    sampling_freq;
    downconvert_and_correlator = CPUThreadedDownconvertAndCorrelator(),
    num_ants::NumAnts{N} = NumAnts(1),
    acquire_every = 10u"s",
    acquisition_false_alarm_probability = 1e-4,
    code_lock_cn0_threshold = get_default_code_lock_cn0_threshold(system),
    time_in_lock_before_calculating_pvt = 2u"s",
    pvt_update_interval = 100u"ms",
    interm_freq = 0.0u"Hz",
    always_buffer = false,
    approximate_year::Integer = year(now(UTC)),
) where {N,RS,TS,AB,P}
    num_samples = size(measurement, 1)
    signal_duration = num_samples / sampling_freq
    # Currently this only supports a single system. Hence [1]
    receiver_sat_states = receiver_state.receiver_sat_states[1]
    track_state = receiver_state.track_state
    runtime = receiver_state.runtime
    last_time_acquisition_ran = receiver_state.last_time_acquisition_ran

    # When always_buffer is false, only fill the acquisition buffer when
    # acquisition could plausibly fire. This avoids ~46 μs of memcpy on
    # every steady-state frame. When always_buffer is true, the buffer is
    # kept up to date for fast reacquisition of lost satellites.
    needs_buffering = always_buffer ||
        runtime - last_time_acquisition_ran >= acquire_every ||
        any(should_reacquire, receiver_sat_states)
    acquisition_buffer = if needs_buffering
        buffer(receiver_state.acquisition_buffer, @view(measurement[:, 1]))
    else
        receiver_state.acquisition_buffer
    end
    last_time_pvt_ran = receiver_state.last_time_pvt_ran

    prev_last_time_acquisition_ran = last_time_acquisition_ran
    track_state, receiver_sat_states, last_time_acquisition_ran = acquire_satellites(
        acq_plan,
        acquisition_buffer,
        track_state,
        receiver_sat_states,
        interm_freq,
        acquisition_false_alarm_probability,
        code_lock_cn0_threshold,
        runtime,
        num_ants,
        last_time_acquisition_ran,
        acquire_every,
        receiver_state.num_samples_processed,
    )
    # Reset buffer after periodic acquisition so stale samples aren't reused.
    # Skip reset when always_buffer is true — the buffer stays fresh every frame.
    if !always_buffer && last_time_acquisition_ran != prev_last_time_acquisition_ran
        acquisition_buffer = SampleBuffers.reset(acquisition_buffer)
    end

    # Use the in-place `track!` rather than the immutable `track`: the latter
    # detaches (copies) the satellite slot vectors and rebuilds the TrackedSat
    # wrappers each call, which dominated per-chunk allocations. The receiver
    # discards the previous ReceiverState every chunk and reuses one correlator
    # (hoisted in `receive`), so mutating the track state in place is safe and
    # is Tracking v3's documented allocation-free real-time pattern.
    track_state = track!(
        measurement,
        track_state,
        sampling_freq;
        intermediate_frequency = interm_freq,
        downconvert_and_correlator,
    )

    receiver_sat_states =
        update_receiver_sat_states(receiver_sat_states, track_state, signal_duration)

    pvt, last_time_pvt_ran = update_pvt(
        system,
        receiver_state.pvt,
        receiver_sat_states,
        track_state,
        runtime,
        time_in_lock_before_calculating_pvt,
        last_time_pvt_ran,
        pvt_update_interval,
        approximate_year,
    )

    track_state = filter_in_lock_sats(receiver_sat_states, track_state)

    ReceiverState{RS,TS,AB,P}(
        track_state,
        (receiver_sat_states,),
        acquisition_buffer,
        pvt,
        receiver_state.runtime + signal_duration,
        last_time_acquisition_ran,
        last_time_pvt_ran,
        receiver_state.num_samples_processed + num_samples,
    )
end

function filter_in_lock_sats(receiver_sat_states, track_state)
    isempty(receiver_sat_states) && return track_state

    tracked_prns = keys(get_sat_states(track_state))
    should_remove(rs) = !is_in_lock(rs) && rs.prn in tracked_prns

    # Use lazy iterator - only allocates if any satellites need removal
    out_of_lock = Iterators.filter(should_remove, receiver_sat_states)
    prns_to_remove = Int[rs.prn for rs in out_of_lock]

    # Tracking v3 dropped `filter_out_sats`; remove PRNs one at a time,
    # threading the returned TrackState through each removal.
    foldl(prns_to_remove; init = track_state) do ts, prn
        remove_satellite(ts; prn)
    end
end

function update_pvt(
    system,
    pvt,
    receiver_sat_states,
    track_state,
    runtime,
    time_in_lock_before_calculating_pvt,
    last_time_pvt_ran,
    pvt_update_interval,
    approximate_year::Integer = year(now(UTC)),
)
    if runtime - last_time_pvt_ran >= pvt_update_interval
        receiver_sat_states_ready_for_pvt =
            filter(receiver_sat_states) do receiver_sat_state
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
            pvt = calc_pvt(pvt_satellite_states, pvt; approximate_year)
        end

        last_time_pvt_ran = runtime
    end

    return pvt, last_time_pvt_ran
end

function update_receiver_sat_states(receiver_sat_states, track_state, signal_duration)
    # In-place `map!` reuses the storage `receiver_sat_states` already owns
    # instead of `map`, which calls `similar(d)` and allocates a fresh
    # `Memory{ReceiverSatState}` (~20 KB / frame) every call. Input and output
    # alias the same dictionary: `map!` reads the old value at each token,
    # computes the new one, then writes it back at that token, so aliasing is
    # safe. The immutable `ReceiverSatState` is built in registers and memcpied
    # into the slot it already occupies — no per-frame heap allocation. Mirrors
    # the in-place `track!` adopted in c1c8b26; the previous frame's state is
    # discarded each frame, so mutating in place is not observable downstream.
    return map!(receiver_sat_states, receiver_sat_states) do receiver_sat_state
        if is_in_lock(receiver_sat_state)
            prn = receiver_sat_state.prn
            ReceiverSatState(
                prn,
                # GNSSDecoder decodes soft symbols; Tracking v3's `get_bits`
                # returns packed hard bits, so hand it the soft-bit buffer.
                decode(
                    receiver_sat_state.decoder,
                    get_soft_bits(track_state, prn),
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
    # Build the TrackedSat with the same estimator the TrackState uses, so its
    # doppler_estimator_state type matches the group's slot type and `merge_sats`
    # accepts it. Correlator / post-corr-filter default to the same values the
    # empty TrackState was templated with (see the ReceiverState constructor).
    TrackedSat(
        acq.system,
        acq.prn,
        acq.code_phase,
        acq.carrier_doppler;
        num_ants,
        post_corr_filter = create_post_corr_filter(num_ants),
        doppler_estimator = track_state.doppler_estimator,
    )
end

function update_states_from_acquisition_results(
    acquisition_results,
    acquisition_false_alarm_probability,
    code_lock_cn0_threshold,
    track_state,
    receiver_sat_states,
    num_ants,
)
    # Early return if no results to process
    isempty(acquisition_results) && return track_state, receiver_sat_states

    # Acquisition v2 uses a CFAR detector: a result is a detection when its
    # peak-to-noise ratio exceeds the CFAR threshold for the given false-alarm
    # probability (rather than a fixed CN0 threshold).
    acq_res_valids = filter(
        res -> is_detected(res; pfa = acquisition_false_alarm_probability),
        acquisition_results,
    )
    isempty(acq_res_valids) && return track_state, receiver_sat_states

    new_receiver_sat_states = map(acq_res_valids) do res
        ReceiverSatState(
            res,
            res.prn in keys(receiver_sat_states) ?
            reset_decoder_state(receiver_sat_states[res.prn].decoder) : nothing,
            code_lock_cn0_threshold,
        )
    end

    new_sat_states = eltype(get_sat_states(track_state))[
        create_sat_state_from_acq(acq, track_state, num_ants) for acq in acq_res_valids
    ]

    new_track_state = merge_sats(track_state, 1, new_sat_states)

    new_receiver_sat_states_dictionary = merge(
        receiver_sat_states,
        Dictionary(get_prns(acq_res_valids), new_receiver_sat_states),
    )
    new_track_state, new_receiver_sat_states_dictionary
end

get_available_prn_channels(acq_plan::AcquisitionPlan) = acq_plan.avail_prns

function acquire_satellites(
    acq_plan,
    acquisition_buffer,
    track_state,
    receiver_sat_states,
    interm_freq,
    acquisition_false_alarm_probability,
    code_lock_cn0_threshold,
    runtime,
    num_ants,
    last_time_acquisition_ran,
    acquire_every,
    num_samples_processed,
)
    track_state, receiver_sat_states = try_to_reacquire_lost_satellites(
        acq_plan,
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        interm_freq,
        acquisition_false_alarm_probability,
        code_lock_cn0_threshold,
        num_ants,
        num_samples_processed,
    )

    if SampleBuffers.isfull(acquisition_buffer) &&
       runtime - last_time_acquisition_ran >= acquire_every
        missing_satellites = vcat(
            filter(
                prn -> !(prn in keys(receiver_sat_states)),
                get_available_prn_channels(acq_plan),
            ),
            collect(keys(filter(state -> !is_in_lock(state), receiver_sat_states))),
        )
        acq_res = acquire!(
            acq_plan,
            SampleBuffers.get_samples(acquisition_buffer),
            missing_satellites;
            interm_freq,
        )

        corrected_acq_res = eltype(acq_res)[
            advance_code_phase(
                res,
                num_samples_processed - (get_first_sample_counter(acquisition_buffer) - 1),
            ) for res in acq_res
        ]

        track_state, receiver_sat_states = update_states_from_acquisition_results(
            corrected_acq_res,
            acquisition_false_alarm_probability,
            code_lock_cn0_threshold,
            track_state,
            receiver_sat_states,
            num_ants,
        )

        last_time_acquisition_ran = runtime
    end
    track_state, receiver_sat_states, last_time_acquisition_ran
end

function advance_code_phase(acq_res::Acquisition.AcquisitionResults, num_samples)
    advanced_code_phase = mod(
        (
            get_code_frequency(acq_res.system) +
            acq_res.carrier_doppler * get_code_center_frequency_ratio(acq_res.system)
        ) * num_samples / acq_res.sampling_frequency + acq_res.code_phase,
        get_code_length(acq_res.system),
    )
    # AcquisitionResults gained fields in Acquisition v2; copy all of them and only
    # override the advanced code phase instead of reconstructing positionally.
    @set acq_res.code_phase = advanced_code_phase
end

function should_reacquire(state)
    !is_in_lock(state) &&
    state.num_unsuccessful_reacquisition <= 10 &&
    state.num_unsuccessful_reacquisition^2 * 100u"ms" >= state.time_out_of_lock
end

function try_to_reacquire_lost_satellites(
    acq_plan,
    track_state,
    receiver_sat_states,
    acquisition_buffer,
    interm_freq,
    acquisition_false_alarm_probability,
    code_lock_cn0_threshold,
    num_ants,
    num_samples_processed,
)
    SampleBuffers.isfull(acquisition_buffer) || return track_state, receiver_sat_states

    any(should_reacquire, receiver_sat_states) || return track_state, receiver_sat_states

    # Only now allocate the filtered Dictionary (preserves type stability)
    out_of_lock_receiver_sat_states = filter(should_reacquire, receiver_sat_states)

    prns_to_reacquire = collect(Int, keys(out_of_lock_receiver_sat_states))
    acq_res = acquire!(
        acq_plan,
        SampleBuffers.get_samples(acquisition_buffer),
        prns_to_reacquire;
        interm_freq,
    )

    corrected_acq_res = eltype(acq_res)[
        advance_code_phase(
            res,
            num_samples_processed - (get_first_sample_counter(acquisition_buffer) - 1),
        ) for res in acq_res
    ]

    invalid_acq_res_prns = get_prns(
        filter(res -> !is_detected(res; pfa = acquisition_false_alarm_probability), acq_res),
    )
    if !isempty(invalid_acq_res_prns)
        receiver_sat_states = map(receiver_sat_states) do state
            state.prn in invalid_acq_res_prns ?
            increment_num_unsuccessful_reacquisition(state) : state
        end
    end

    return update_states_from_acquisition_results(
        corrected_acq_res,
        acquisition_false_alarm_probability,
        code_lock_cn0_threshold,
        track_state,
        receiver_sat_states,
        num_ants,
    )
end
