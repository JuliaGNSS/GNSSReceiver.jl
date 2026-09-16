# ─────────────────────────────────────────────────────────────────────────────
# A delay-aware tracking loop for hardware correlators (GNSSReceiver.jl #107)
#
# The conventional PLL/DLL assumes its output acts before the next measurement.
# A hardware NCO is written milliseconds after the record that motivated the
# correction ended, and holds the word until the next one lands. At the 18 Hz
# bandwidth GPS L1 C/A is tuned for, a correction that acts 3–4 ms late instead
# of 1 ms overshoots and the carrier limit-cycles (±80 Hz, 25 ms period) while
# C/N₀ and code lock look perfect — see docs/plans/2026-09-14-delay-aware-
# hardware-loop.md for the measurements.
#
# The estimator below closes the same loop, with the same filter and the same
# gains, but referenced to what actually happened at the NCO — and to what is
# already committed to happen there. It is a Smith predictor around Tracking's
# `ThirdOrderAssistedBilinearLF`: with no delay it is the conventional loop to
# the bit; with delay, it emulates that loop acting at the sample where its
# command will land.
# ─────────────────────────────────────────────────────────────────────────────

"""
    NCOReferencedPLLAndDLL(; carrier_loop_filter_bandwidth = nothing,
                             code_loop_filter_bandwidth = nothing,
                             predict_landing = true)

FLL-assisted PLL and DLL Doppler estimator for a replica whose NCO words are
applied with a known delay — the hardware-correlator receiver's default.

It is `Tracking`'s `ConventionalAssistedPLLAndDLL` — the same third-order
assisted bilinear carrier filter, the same second-order code filter, the same
gains and the same bandwidth defaults (auto-sized per signal when `nothing`, 18 Hz
for GPS L1 C/A) — with its loop internals referenced to the device NCO instead of
to the word the filter last computed:

 1. **Every record is attributed to the word that ran under it.** The phase
    discriminator is measured against the applied replica by construction; the
    frequency discriminator is re-based onto it too, so `applied word + FLL
    discriminator` is an *absolute* measurement of the signal's Doppler, whatever
    the filter assumed was applied. The DLL is normalised with the applied code
    word. The link supplies the words per hardware channel
    ([`NCOTimeline`](@ref)).
 2. **The correction is sized for the moment it lands.** The filter is stepped
    with the discriminators *predicted at the landing sample of the new
    command*: the measured phase error advanced by `2π ∫ (f̂ − w(τ)) dτ` over the
    words already scheduled at the NCO, and the frequency measurement taken
    relative to the word that will be running there. If a command already in
    flight removes most of the error before this one lands, this one is sized
    for the remainder — no correction of a correction.

With zero delay both steps are the identity and the estimator *is* the
conventional loop, so the software receiver's noise performance is inherited
rather than re-tuned; with delay, the loop's response is that of the delay-free
loop acting at the landing sample, up to the error of the frequency estimate
the phase is propagated with. Measured on the recording that defeats the
conventional loop at three epochs of delay, this holds lock at one to six.

`predict_landing = false` keeps step 1 and drops step 2. It is the documented
**negative control**: re-basing on the applied word alone fails exactly like the
conventional loop, because each new command then restates the whole correction
the scheduled words are already about to make.

The estimator reads the words from a [`HardwareCorrelatorLink`](@ref); through
the software receiver (`track!`) it runs with the chunk's own replica Doppler
and no delay, i.e. as the conventional loop.

The acquisition Doppler bins `receive` derives from the estimator's pull-in
range are the conventional assisted loop's: the FLL bound `1/(4T)`.
"""
struct NCOReferencedPLLAndDLL{CO<:Tracking.AbstractLoopFilter} <:
       Tracking.AbstractDopplerEstimator
    carrier_loop_filter_bandwidth::Union{Nothing,typeof(1.0Hz)}
    code_loop_filter_bandwidth::Union{Nothing,typeof(1.0Hz)}
    predict_landing::Bool
end

function NCOReferencedPLLAndDLL(
    ::Type{CO} = Tracking.SecondOrderBilinearLF;
    carrier_loop_filter_bandwidth::Union{Nothing,typeof(1.0Hz)} = nothing,
    code_loop_filter_bandwidth::Union{Nothing,typeof(1.0Hz)} = nothing,
    predict_landing::Bool = true,
) where {CO<:Tracking.AbstractLoopFilter}
    NCOReferencedPLLAndDLL{CO}(
        carrier_loop_filter_bandwidth,
        code_loop_filter_bandwidth,
        predict_landing,
    )
end

"""
    SatNCOReferencedPLLAndDLL

Per-satellite state of an [`NCOReferencedPLLAndDLL`](@ref): the handover
Dopplers the loop filters' outputs are offsets from, both filters, their
bandwidths, and the centre sample of the last record folded (the FLL measures
the mean frequency offset between two prompts' centres, so that is the span its
replica word is averaged over).
"""
struct SatNCOReferencedPLLAndDLL{
    CA<:Tracking.ThirdOrderAssistedBilinearLF,
    CO<:Tracking.AbstractLoopFilter,
}
    init_carrier_doppler::typeof(1.0Hz)
    init_code_doppler::typeof(1.0Hz)
    carrier_loop_filter::CA
    code_loop_filter::CO
    carrier_loop_filter_bandwidth::typeof(1.0Hz)
    code_loop_filter_bandwidth::typeof(1.0Hz)
    # Device sample at the centre of the last record folded; `NaN` before the
    # first.
    previous_record_center::Float64
end

function SatNCOReferencedPLLAndDLL(
    state::SatNCOReferencedPLLAndDLL{CA,CO};
    carrier_loop_filter::Union{Nothing,CA} = nothing,
    code_loop_filter::Union{Nothing,CO} = nothing,
    previous_record_center::Union{Nothing,Float64} = nothing,
) where {CA,CO}
    SatNCOReferencedPLLAndDLL{CA,CO}(
        state.init_carrier_doppler,
        state.init_code_doppler,
        something(carrier_loop_filter, state.carrier_loop_filter),
        something(code_loop_filter, state.code_loop_filter),
        state.carrier_loop_filter_bandwidth,
        state.code_loop_filter_bandwidth,
        something(previous_record_center, state.previous_record_center),
    )
end

function Tracking.init_estimator_state(
    estimator::NCOReferencedPLLAndDLL{CO},
    sat::TrackedSat,
) where {CO}
    driver_signal = first(sat.signals).signal
    SatNCOReferencedPLLAndDLL(
        sat.carrier_doppler,
        sat.code_doppler,
        Tracking.ThirdOrderAssistedBilinearLF(),
        Accessors.constructorof(CO)(),
        something(
            estimator.carrier_loop_filter_bandwidth,
            Tracking.default_carrier_loop_filter_bandwidth(driver_signal),
        ),
        something(
            estimator.code_loop_filter_bandwidth,
            Tracking.default_code_loop_filter_bandwidth(driver_signal),
        ),
        NaN,
    )
end

# `reset_loop_filters!`: zero the integrators and re-seed from the converged
# Dopplers, keeping the per-satellite bandwidths (as the conventional estimator
# does).
function Tracking._reset_estimator_state(
    ::NCOReferencedPLLAndDLL,
    sat::TrackedSat{<:Tuple{Vararg{TrackedSignal}},<:SatNCOReferencedPLLAndDLL},
)
    state = sat.doppler_estimator_state
    SatNCOReferencedPLLAndDLL(
        sat.carrier_doppler,
        sat.code_doppler,
        Accessors.constructorof(typeof(state.carrier_loop_filter))(),
        Accessors.constructorof(typeof(state.code_loop_filter))(),
        state.carrier_loop_filter_bandwidth,
        state.code_loop_filter_bandwidth,
        NaN,
    )
end

# Pull-in is the FLL discriminator's, exactly as for the conventional assisted
# loop (see `carrier_doppler_pull_in_range` in receive.jl).
function carrier_doppler_pull_in_range(::NCOReferencedPLLAndDLL, signal::AbstractGNSSSignal)
    T = handover_coherent_integration_time(signal)
    uconvert(Hz, 1 / (4 * T))
end

# "No landing sample": the command acts at each record's end, i.e. no delay.
const NO_LANDING_SAMPLE = typemin(Int64)

# A BPSK prompt fixes the carrier phase modulo π, and so does `pll_disc`; a
# predicted phase error has to be folded into the same (−π/2, π/2] range. Exact
# for anything already inside it.
wrap_half_cycle(phase) = rem(phase, π, RoundNearest)

# Fold the estimator-driver signal's records: Tracking's per-record advance
# (prompt filter, C/N₀, bit buffer) plus the NCO-referenced loop-filter step.
# Mirrors `Tracking._process_estimator_driver_signal`; the only differences are
# the two re-referencing steps documented on `NCOReferencedPLLAndDLL`.
#
# `words` answers `mean_nco_word(words, a, b)` for the replica word over device
# samples `[a, b)`; `landing_sample` is the device sample the command computed
# from this fold takes effect at, or `NO_LANDING_SAMPLE` for "at each record's
# end".
@inline function _nco_fold_driver(
    tracked_signal::TrackedSignal,
    sat::TrackedSat,
    state::SatNCOReferencedPLLAndDLL,
    estimator::NCOReferencedPLLAndDLL,
    sampling_frequency,
    noise_density,
    noise_density_ready::Bool,
    driver_carrier_phase::Real,
    words,
    landing_sample::Int64,
)
    outputs = tracked_signal.correlator_outputs
    if isempty(outputs)
        return tracked_signal, state, sat.carrier_doppler, sat.code_doppler
    end
    signal = tracked_signal.signal
    ts = tracked_signal
    carrier_loop_filter = state.carrier_loop_filter
    code_loop_filter = state.code_loop_filter
    carrier_doppler = sat.carrier_doppler
    code_doppler = sat.code_doppler
    previous_center = state.previous_record_center
    sampling_freq_hz = Float64(ustrip(Hz, uconvert(Hz, sampling_frequency)))
    found_before_fold = has_bit_or_secondary_code_been_found(ts.bit_buffer)
    # The command this fold produces is computed after its last record and
    # lands at `landing_sample`; every record of the fold maps onto the
    # delay-free loop's record that far ahead of it.
    fold_end = last(outputs).sample_index
    shift = landing_sample == NO_LANDING_SAMPLE ? 0 : landing_sample - fold_end
    @inbounds for k in eachindex(outputs)
        output = outputs[k]
        record_end = output.sample_index
        record_samples = output.integrated_samples
        record_start = record_end - record_samples
        center = record_end - record_samples / 2
        # Per-record integration time — the block time, not the chunk time.
        integration_time = record_samples / sampling_frequency
        # The FLL chains from the previous record's filtered prompt; read it
        # before the advance overwrites it.
        previous_prompt = get_last_fully_integrated_filtered_prompt(ts)
        synced_earlier_in_fold =
            !found_before_fold && has_bit_or_secondary_code_been_found(ts.bit_buffer)
        ts, filtered_correlator, integrated_code_blocks = Tracking._apply_correlator_output(
            ts,
            output,
            sat.prn,
            sampling_frequency,
            noise_density,
            noise_density_ready,
            driver_carrier_phase;
            correlated_pre_sync = synced_earlier_in_fold,
        )

        # The words this record really ran on.
        applied_carrier, applied_code = mean_nco_word(words, record_start, record_end)
        # Discriminators against the applied replica. The phase error is that by
        # construction; the frequency error is the mean offset from the replica
        # between the two prompts' centres.
        phase_error = Tracking.pll_disc(signal, filtered_correlator)
        frequency_error =
            Tracking.fll_disc(signal, filtered_correlator, previous_prompt, integration_time)
        fll_word =
            isnan(previous_center) ? applied_carrier :
            first(mean_nco_word(words, previous_center, center))

        if estimator.predict_landing && shift > 0
            # The phase error this record would show `shift` samples later,
            # under the words the NCO will run in between: the mean phase sits
            # at the record's centre, so the ramp is integrated from there. The
            # signal's Doppler is the filter's own estimate, before this
            # record's innovation.
            f_hat = ustrip(
                Hz,
                uconvert(
                    Hz,
                    state.init_carrier_doppler +
                    carrier_loop_filter.x1 +
                    integration_time / 2 * carrier_loop_filter.x2,
                ),
            )
            ramp_word = first(mean_nco_word(words, center, center + shift))
            phase_error = wrap_half_cycle(
                phase_error + 2π * shift * (f_hat - ramp_word) / sampling_freq_hz,
            )
            # The absolute frequency measurement, relative to the word that
            # will be running under the record `shift` samples ahead.
            landing_word =
                first(mean_nco_word(words, record_start + shift, record_end + shift))
            frequency_error += (fll_word - landing_word) * Hz
        end

        # Bandwidths exactly as the conventional loop: carrier scaled by the
        # blocks the record actually covered, code capped by its stability
        # product against the record's integration time.
        carrier_bandwidth = state.carrier_loop_filter_bandwidth / integrated_code_blocks
        code_bandwidth = Tracking.effective_code_loop_filter_bandwidth(
            state.code_loop_filter_bandwidth,
            integration_time,
        )
        carrier_freq_update, carrier_loop_filter = Tracking.filter_loop(
            carrier_loop_filter,
            (phase_error, frequency_error),
            integration_time,
            carrier_bandwidth,
        )
        # The DLL normalises with the code word the replica actually ran on.
        code_freq_update, code_loop_filter = Tracking.calculate_code_frequency_update(
            signal,
            code_loop_filter,
            filtered_correlator,
            applied_code * Hz,
            sampling_frequency,
            integration_time,
            code_bandwidth,
        )
        carrier_doppler, code_doppler = Tracking.aid_dopplers(
            signal,
            state.init_carrier_doppler,
            state.init_code_doppler,
            carrier_freq_update,
            code_freq_update,
        )
        previous_center = center
    end
    empty!(outputs)
    new_state = SatNCOReferencedPLLAndDLL(
        state;
        carrier_loop_filter,
        code_loop_filter,
        previous_record_center = previous_center,
    )
    return ts, new_state, carrier_doppler, code_doppler
end

# Per-satellite update: the driver fold above, Tracking's passenger fold, and
# the secondary-code phase snap on the fold that first finds sync — the same
# sequence as `Tracking._update_tracked_sat_doppler`.
function _nco_update_tracked_sat(
    sat::TrackedSat,
    estimator::NCOReferencedPLLAndDLL,
    sampling_frequency,
    noise::Tuple,
    words,
    landing_sample::Int64,
)
    head = first(sat.signals)
    tail_signals = Base.tail(sat.signals)
    driver_carrier_phase = Tracking.get_carrier_phase_offset(head.signal)
    driver_noise_density, driver_noise_density_ready = first(noise)
    new_head, new_state, new_carrier_doppler, new_code_doppler = _nco_fold_driver(
        head,
        sat,
        sat.doppler_estimator_state,
        estimator,
        sampling_frequency,
        driver_noise_density,
        driver_noise_density_ready,
        driver_carrier_phase,
        words,
        landing_sample,
    )
    new_tail = Tracking._process_passenger_signals(
        tail_signals,
        sat.prn,
        sampling_frequency,
        Base.tail(noise),
        driver_carrier_phase,
    )
    new_signals = (new_head, new_tail...)
    just_synced = Tracking._any_signal_just_synced(sat.signals, new_signals)
    snapped_code_phase =
        just_synced ? Tracking._snap_code_phase_from_synced_signal(new_signals, sat.code_phase) :
        sat.code_phase
    final_signals =
        just_synced ? map(Tracking._reset_inflight_integration, new_signals) : new_signals
    TrackedSat(
        sat;
        code_phase = snapped_code_phase,
        carrier_doppler = new_carrier_doppler,
        code_doppler = new_code_doppler,
        signals = final_signals,
        doppler_estimator_state = new_state,
    )
end

# ── Through the software receiver: no device, no delay ───────────────────────

# `track!` regenerates every replica from the satellite's Doppler each chunk
# and applies the new Doppler to the next chunk, so the word each record ran on
# is the satellite's own and the command acts at the record's end.
@inline function _nco_est_software_group!(
    g::Tracking.SignalGroup,
    sampling_frequencies,
    noise_estimators::NamedTuple,
    estimator::NCOReferencedPLLAndDLL,
)
    vals = g.satellites.values
    isempty(vals) && return nothing
    sampling_frequency =
        Tracking._band_sampling_frequency(sampling_frequencies, get_band_id(g.band))
    noise = Tracking._signal_noise_densities(noise_estimators, eltype(g.satellites))
    Tracking._warn_noise_density_missing(eltype(g.satellites), noise, noise_estimators)
    @inbounds for i in eachindex(vals)
        sat = vals[i]
        words = FixedNCOWord(
            ustrip(Hz, uconvert(Hz, sat.carrier_doppler)),
            ustrip(Hz, uconvert(Hz, sat.code_doppler)),
        )
        vals[i] = _nco_update_tracked_sat(
            sat,
            estimator,
            sampling_frequency,
            noise,
            words,
            NO_LANDING_SAMPLE,
        )
    end
    nothing
end

function Tracking.estimate_dopplers_and_filter_prompt!(
    track_state::TrackState{<:Tracking.SignalGroups,<:NCOReferencedPLLAndDLL},
    sampling_frequencies::Union{Tracking.BandMeasurements,NamedTuple,AbstractDict},
)
    Tracking._foreach_group!(
        _nco_est_software_group!,
        track_state.groups,
        sampling_frequencies,
        track_state.noise_estimators,
        track_state.doppler_estimator,
    )
    return track_state
end

function Tracking.estimate_dopplers_and_filter_prompt(
    track_state::TrackState{<:Tracking.SignalGroups,<:NCOReferencedPLLAndDLL},
    sampling_frequencies::Union{Tracking.BandMeasurements,NamedTuple,AbstractDict},
)
    new_track_state = TrackState(
        track_state;
        groups = Tracking._copy_groups_slot_vectors(track_state.groups),
    )
    Tracking.estimate_dopplers_and_filter_prompt!(new_track_state, sampling_frequencies)
end

# ── Through a hardware-correlator link: the device's words and landing sample ─

# Walk the groups with their keys, which the link's channel table is indexed by.
# A recursive tuple walk, like `Tracking._foreach_group!`, so a heterogeneous
# group tuple does not box.
@inline _nco_foreach_group!(f::F, ::NamedTuple{()}, args::Vararg{Any,N}) where {F,N} =
    nothing
@inline function _nco_foreach_group!(f::F, groups::NamedTuple, args::Vararg{Any,N}) where {F,N}
    f(first(keys(groups)), first(values(groups)), args...)
    _nco_foreach_group!(f, Base.tail(groups), args...)
end

@inline function _nco_est_hardware_group!(
    group_key::Symbol,
    g::Tracking.SignalGroup,
    link::HardwareCorrelatorLink,
    band_measurements,
    noise_estimators::NamedTuple,
    estimator::NCOReferencedPLLAndDLL,
)
    vals = g.satellites.values
    isempty(vals) && return nothing
    sampling_frequency =
        Tracking._band_sampling_frequency(band_measurements, get_band_id(g.band))
    noise = Tracking._signal_noise_densities(noise_estimators, eltype(g.satellites))
    Tracking._warn_noise_density_missing(eltype(g.satellites), noise, noise_estimators)
    landing = link.scheduled_apply_at_sample
    @inbounds for i in eachindex(vals)
        sat = vals[i]
        hw_channel = get(
            link.channel_of,
            HardwareChannelAssignment(group_key, sat.prn, RANGING_SIGNAL_INDEX),
            0,
        )
        # A satellite without a channel has no records; its words are never read.
        words = hw_channel == 0 ? link.unassigned_timeline : link.nco_timelines[hw_channel]
        # The landing sample is decided once, on the receiver timebase, and read
        # here on the *channel's own band counter* — the axis its records'
        # `sample_index`es and its timeline's words are on. On a band counted at
        # another rate the two axes differ by that ratio, and a shift measured
        # across them would mis-size every predicted phase error by it.
        landing_sample = hw_channel == 0 ? landing : _band_sample(link, hw_channel, landing)
        vals[i] = _nco_update_tracked_sat(
            sat,
            estimator,
            sampling_frequency,
            noise,
            words,
            landing_sample,
        )
    end
    nothing
end

function estimate_dopplers!(
    link::HardwareCorrelatorLink,
    track_state::TrackState{<:Tracking.SignalGroups,<:NCOReferencedPLLAndDLL},
    band_measurements,
)
    _nco_foreach_group!(
        _nco_est_hardware_group!,
        track_state.groups,
        link,
        band_measurements,
        track_state.noise_estimators,
        track_state.doppler_estimator,
    )
    return track_state
end
