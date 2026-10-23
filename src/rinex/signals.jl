# The signals this receiver ranges on, as the RINEX observation header names them: their
# observation codes, satellite systems and the header's `obs_codes` come from RINEXParser's
# GNSSSignals extension (`observation_code`, `RINEXParser.satellite_system`,
# `observation_codes`); the carrier-phase alignment and the per-group layout the logger
# writes them by are the receiver's.

# RINEX 3.05 requires every carrier phase in a file aligned with its band's reference signal
# (section 5.2.12, Table A23), and the header to state the correction that aligned each code
# (`SYS / PHASE SHIFT`). The tracked phase is the ranging signal's own carrier: Tracking
# steers the prompt of a group's ranging signal onto the real axis, which for a pilot is not
# the band's reference. RINEXParser carries the table (`phase_correction`). For the BDS-3
# pilots the table only says they must be aligned to their data components: the direction
# of the quarter cycle between them depends on how a receiver tracks them, so it is this
# receiver's own correction to state, here with the sign RTKLIB writes.
const BDS_PILOT_PHASE_CORRECTIONS = Dict(('C', "L1P") => 0.25, ('C', "L5P") => 0.25)

# The correction [cycles] that aligns the carrier phase `code` of `system` with its band's
# reference signal, `nothing` where it needs none (the reference itself, or a code Table A23
# aligns with "None").
function rinex_phase_shift(system::Char, code::AbstractString)
    correction = phase_correction(system, code)
    if isnothing(correction)
        haskey(BDS_PILOT_PHASE_CORRECTIONS, (system, code)) || throw(
            ArgumentError(
                "RINEX 3.05 Table A23 gives no phase alignment for $system $code, so its " *
                "carrier phase cannot be written",
            ),
        )
        return BDS_PILOT_PHASE_CORRECTIONS[(system, code)]
    end
    iszero(correction) ? nothing : correction
end

# The observation codes of the four observables this receiver produces per signal:
# pseudorange, carrier phase, Doppler, signal strength — in header order.
const RinexObsCodes = @NamedTuple{C::String, L::String, D::String, S::String}
rinex_obs_codes(signal) =
    RinexObsCodes(observation_code.(('C', 'L', 'D', 'S'), Ref(signal)))

# Everything the RINEX writer needs to know about one tracking group (one `(signal, PRN)`
# namespace, keyed by `signal_group_key`): the observation codes its observables are written
# under, and the constants that turn tracking state into those observables.
struct RinexSignalLayout
    system::Char
    codes::RinexObsCodes
    # The signal the group ranges on. Needed to rebuild the transmit time of a satellite from
    # its tracking state, whose code and carrier terms scale with this signal's frequencies.
    signal::AbstractGNSSSignal
    carrier_frequency::Float64
    # Receiver intermediate frequency of this signal's band: the tracked carrier phase
    # advances at `carrier_doppler + interm_freq`, so the IF ramp has to come back out to
    # leave the Doppler-only phase RINEX wants.
    interm_freq::Float64
    # Index of the data signal among the group's tracked signals: its bit buffer and the
    # decoder fed from it resolve the Costas half-cycle of the carrier phase.
    data_index::Int
    # Sampling frequency of this signal's band, which reading the vector loop's carrier
    # phase correction takes (`get_carrier_phase_correction`).
    sampling_frequency::typeof(1.0Hz)
    # The correction [cycles] that aligns the phase with its band's reference signal,
    # `nothing` where it needs none (`rinex_phase_shift`).
    phase_shift::Union{Nothing,Float64}
end

# The `SYS / PHASE SHIFT` records of the corrections `layouts` apply, one per code.
rinex_phase_shifts(layouts) = sort!(
    unique(
        PhaseShift(layout.system, layout.codes.L, layout.phase_shift) for
        layout in values(layouts) if !isnothing(layout.phase_shift)
    );
    by = shift -> (shift.system, shift.code),
)

# Metres per cycle of a signal's carrier — what converts between the range and phase
# observables, and between the Doppler and the range rate.
wavelength(layout::RinexSignalLayout) = SPEED_OF_LIGHT / layout.carrier_frequency

# The observation codes of the signal the receiver ranges on, or RINEXParser's reason why the
# format cannot name it, with what to do about it here.
function ranging_obs_codes(signal)
    try
        rinex_obs_codes(signal)
    catch e
        e isa ArgumentError || rethrow()
        throw(
            ArgumentError(
                "$(e.msg). Track it without RINEX output, or range on a signal the " *
                "format names.",
            ),
        )
    end
end

# Build the observation-header layout for every configured system: the header's `obs_codes`
# (RINEXParser's `observation_codes` of the ranging signals, in band and system order) plus
# the per-group [`RinexSignalLayout`](@ref). Every band is sampled at `sampling_freq`.
function rinex_layout(band_systems::Tuple, interm_freqs::Tuple, sampling_freq)
    signals = AbstractGNSSSignal[]
    layouts = Dict{Symbol,RinexSignalLayout}()
    for (systems, interm_freq) in zip(band_systems, interm_freqs)
        for system in systems
            # We range on the ranging signal (the pilot of a `CombinedSignal`), so that is
            # the signal the observables belong to — and its id is the group key that
            # addresses the satellite in `track_state` and in `pvt.sats`.
            signal = ranging_signal(system)
            codes = ranging_obs_codes(signal)
            system_char = RINEXParser.satellite_system(signal)
            # The header declares a shared code once, so a satellite tracked on both
            # signals would give it twice in one record, which RINEX cannot hold.
            for (key, layout) in layouts
                layout.system == system_char && layout.codes == codes && throw(
                    ArgumentError(
                        "$key and $(get_signal_id(signal)) are both written as the " *
                        "RINEX observations $(join(codes, ", ")) of system " *
                        "$system_char, so a satellite tracked on both cannot be " *
                        "written. Range on one of them for RINEX output.",
                    ),
                )
            end
            push!(signals, signal)
            layouts[get_signal_id(signal)] = RinexSignalLayout(
                system_char,
                codes,
                signal,
                ustrip(Hz, get_center_frequency(signal)),
                ustrip(Hz, interm_freq),
                data_signal_index(system),
                float(sampling_freq),
                rinex_phase_shift(system_char, codes.L),
            )
        end
    end
    observation_codes(signals), layouts
end
