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

using Unitful: m, s, ms, Hz, dBHz, dB, °

# Lock-free channel primitives and SoapySDR device streaming now live in their own
# packages (they replaced the vendored `channel.jl` / `soapy_sdr_helper.jl`). The
# SDR streaming (`stream_data` / `SDRChannelConfig`) comes from SignalChannels'
# SoapySDR extension, which `using SoapySDR` above loads.
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
    CombinedSignal,
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

# Throughout the receiver, a constellation is identified by GNSSSignals'
# `get_signal_id` (e.g. `:GPSL1CA`, `:GalileoE1B`) — the same per-signal symbol
# `PositionVelocityTime` keys its `pvt.sats` by, so one key addresses a satellite
# across tracking, decoding and PVT.

# Number of samples spanning one primary code period of `system` at `sampling_freq`.
# Uses the same `ceil` convention as Acquisition's `plan_acquire`, so buffer sizing
# matches the plan's sample requirement exactly.
samples_per_code(system::AbstractGNSSSignal, sampling_freq) =
    ceil(Int, get_code_length(system) * upreferred(sampling_freq / get_code_frequency(system)))

# Code periods per navigation data bit, matching Acquisition's `plan_acquire`.
# `nothing` for dataless (pilot) signals, which carry no bit-period constraint.
function code_periods_per_data_bit(system::AbstractGNSSSignal, sampling_freq)
    data_freq = get_data_frequency(system)
    (isfinite(ustrip(data_freq)) && ustrip(data_freq) > 0) || return nothing
    ceil(Int, upreferred(sampling_freq / data_freq)) ÷ samples_per_code(system, sampling_freq)
end

# Is a coherent window of `nc` code periods a length `plan_acquire` accepts? A
# window spanning ≥ one data bit must cover whole bits, and (for tiered/pilot
# signals) whole secondary-code periods; sub-bit windows are unconstrained. This
# mirrors the divisibility constraints `plan_acquire` enforces — it validates
# these rather than deriving them, so the caller must land on a valid `nc`.
function is_valid_coherent_length(system::AbstractGNSSSignal, sampling_freq, nc::Int)
    nc <= 1 && return nc == 1
    L = get_secondary_code_length(system)
    (L > 1 && nc % L != 0) && return false
    bpc = code_periods_per_data_bit(system, sampling_freq)
    (!isnothing(bpc) && nc >= bpc && nc % bpc != 0) && return false
    true
end

# Coherent code-period count needed to reach a requested acquisition Doppler
# resolution. The Doppler bin spacing of a coherent integration is 1 / (nc · T_code)
# = get_code_frequency / (nc · get_code_length), so nc = get_code_frequency /
# (Δf · get_code_length), rounded UP (Δf is treated as a maximum). A `nothing`
# resolution means "no constraint" ⇒ a single code period. This is set by the
# coherent integration *time* alone and is independent of the carrier frequency
# (Fourier duality); the carrier only affects the Doppler search *span*, not its
# resolution. Shared by the acquisition-signal chooser and `plan_band_acquisition` so the two
# can never disagree on the required coherent length.
function coherent_code_periods_for_resolution(system::AbstractGNSSSignal, acq_doppler_resolution)
    isnothing(acq_doppler_resolution) && return 1
    max(
        1,
        ceil(
            Int,
            upreferred(
                get_code_frequency(system) / (acq_doppler_resolution * get_code_length(system)),
            ),
        ),
    )
end

# Wall-clock duration of a coherent integration of `nc` code periods of `system`
# (nc · T_code = nc · get_code_length / get_code_frequency). Used to compare the
# pilot's and data component's acquisition windows on the same (time) axis.
coherent_integration_time(system::AbstractGNSSSignal, nc::Integer) =
    nc * get_code_length(system) / get_code_frequency(system)

# Snap a resolution-derived coherent code-period count UP to the smallest value
# `plan_acquire` accepts (see `is_valid_coherent_length`). Rounding up (never down)
# keeps the achieved Doppler resolution (∝ 1/nc) at least as fine as requested —
# `acq_doppler_resolution` is treated as a maximum, so snapping must not coarsen it.
# A valid length always exists above `ideal` (multiples of the secondary-code /
# data-bit period), so the search terminates.
function snap_coherent_code_periods(system::AbstractGNSSSignal, sampling_freq, ideal::Int)
    ideal <= 1 && return 1
    nc = ideal
    while !is_valid_coherent_length(system, sampling_freq, nc)
        nc += 1
    end
    nc
end

# Default PRNs to search per system. Kept conservative to bound the
# acquisition cost; override via the `prns` keyword of `receive`. The
# per-constellation supertypes let us default to each system's real PRN range:
# GPS allocates 1:32, Galileo 1:36 (searching only 1:32 would silently skip the
# higher Galileo PRNs).
default_prns(::AbstractGNSSSignal) = 1:32
default_prns(::AbstractGalileoSignal) = 1:36

# PRNs that actually broadcast a given signal. The modernized GPS civil signals
# live only on newer satellite blocks — L5 on Block IIF and III, L2C on Block
# IIR-M, IIF and III — whereas L1 C/A (and every non-GPS signal) is on the whole
# constellation. Restricting acquisition to these PRNs avoids blind-searching
# satellites that cannot carry the signal (the GPS L5 search drops from 32 PRNs to
# ~20). `nothing` means "no restriction — search all requested PRNs".
#
# The block membership is from the Wikipedia "List of GPS satellites"; PRN↔satellite
# assignments change as satellites are launched and retired, so this reflects the
# constellation at time of writing. Pass an explicit `prns` list to override for a
# historical capture.
const GPS_L5_PRNS =
    [1, 3, 4, 6, 8, 9, 10, 11, 14, 18, 20, 21, 23, 24, 25, 26, 27, 28, 30, 32]
const GPS_L2C_PRNS = sort(union(GPS_L5_PRNS, [5, 7, 12, 15, 17, 29, 31]))

broadcasting_prns(::AbstractGNSSSignal) = nothing
broadcasting_prns(::GPSL5I) = GPS_L5_PRNS
broadcasting_prns(::GPSL5Q) = GPS_L5_PRNS
broadcasting_prns(::GPSL2CM) = GPS_L2C_PRNS
broadcasting_prns(::GPSL2CL) = GPS_L2C_PRNS

# The candidate PRNs the caller asked to search for `system`. `prns` may be:
#   * `nothing`                       ⇒ the constellation default (`default_prns`);
#   * a per-GNSS `NamedTuple` / `Dict` keyed by GNSSSignals' `get_constellation_id`
#     (`:GPS`, …), falling back to the default for a constellation not listed;
#   * any other collection            ⇒ used for every system (backwards compatible).
requested_prns(::Nothing, system) = default_prns(system)
requested_prns(prns::NamedTuple, system) =
    get(prns, get_constellation_id(system), default_prns(system))
requested_prns(prns::AbstractDict, system) =
    get(prns, get_constellation_id(system), default_prns(system))
requested_prns(prns, system) = prns

# PRNs to actually acquire for `system`: the requested candidates restricted to the
# PRNs that broadcast the signal (request order preserved). Empty is legal — the
# band simply has nothing to acquire for that system.
function search_prns(prns, system)
    requested = requested_prns(prns, system)
    capable = broadcasting_prns(system)
    isnothing(capable) ? collect(requested) : intersect(requested, capable)
end

get_default_code_lock_cn0_threshold(::AbstractGNSSSignal) = 30.0dBHz

# CFAR false-alarm probability for acquisition detection — the sole acquisition
# detector (no CN0 floor). Matches the last released `acquisition_false_alarm_probability`
# default. Kept as a const because it is shared between `process`'s keyword default and
# the internal value `receive` sets before calling `process`.
const DEFAULT_ACQ_PFA = 1e-4

struct ReceiverSatState{DS<:GNSSDecoderState}
    prn::Int
    decoder::DS
    code_lock_detector::CodeLockDetector
    carrier_lock_detector::CarrierLockDetector
    time_in_lock::typeof(1.0s)
    time_out_of_lock::typeof(1.0s)
    num_unsuccessful_reacquisition::Int
end

function ReceiverSatState(
    acq::Acquisition.AcquisitionResults,
    decoder::GNSSDecoderState,
    code_lock_cn0_threshold::typeof(1.0dBHz) = get_default_code_lock_cn0_threshold(acq.system),
)
    ReceiverSatState(
        acq.prn,
        decoder,
        CodeLockDetector(; cn0_threshold = code_lock_cn0_threshold),
        CarrierLockDetector(),
        0.0s,
        0.0s,
        0,
    )
end

function ReceiverSatState(
    system::AbstractGNSSSignal,
    prn::Int,
    code_lock_cn0_threshold::typeof(1.0dBHz) = get_default_code_lock_cn0_threshold(system),
)
    ReceiverSatState(
        prn,
        GNSSDecoderState(system, prn),
        CodeLockDetector(; cn0_threshold = code_lock_cn0_threshold),
        CarrierLockDetector(),
        0.0s,
        0.0s,
        0,
    )
end

function is_in_lock(state::ReceiverSatState)
    is_in_lock(state.code_lock_detector) && is_in_lock(state.carrier_lock_detector)
end

function increase_time_out_of_lock(state::ReceiverSatState, time::Unitful.Time)
    @reset state.time_in_lock = 0.0s
    @reset state.time_out_of_lock = state.time_out_of_lock + time
    return state
end

function increment_num_unsuccessful_reacquisition(state::ReceiverSatState)
    @reset state.num_unsuccessful_reacquisition = state.num_unsuccessful_reacquisition + 1
    return state
end

# One receiver state spanning every RF band. `Tracking` tracks all bands from a
# single `TrackState` (its groups carry their own band tag; `track` takes one
# `BandMeasurement` per band — see `process`), so `track_state`,
# `receiver_sat_states`, `pvt`, `runtime` and `last_time_pvt_ran` are shared
# across bands and stored once. Only acquisition is intrinsically per-band — each
# band buffers its own RF stream — so `acquisition_buffers` and
# `last_time_acquisition_ran` are NamedTuples keyed by `band_key` (`:l1`, `:l5`,
# …), one entry per band. `receiver_sat_states` is a NamedTuple keyed by each
# system's group key (`get_signal_id`, the ranging signal's id, unique across
# bands), each value a per-constellation dictionary of `ReceiverSatState` by PRN.
struct ReceiverState{
    TS<:TrackState,
    RS<:NamedTuple,
    AB<:NamedTuple,
    LT<:NamedTuple,
    P<:PVTSolution,
    PB<:AbstractVector{<:SatelliteState},
}
    track_state::TS
    receiver_sat_states::RS
    acquisition_buffers::AB
    last_time_acquisition_ran::LT
    pvt::P
    # Reused across PVT cycles: `update_pvt` refills it in place instead of allocating a
    # fresh `Vector{SatelliteState}` each cycle. Pooling every constellation and band
    # makes its element type the abstract `SatelliteState` when more than one signal is
    # tracked, but reuse still saves the per-cycle allocation.
    pvt_sat_state_buffer::PB
    runtime::typeof(1.0s)
    last_time_pvt_ran::typeof(1.0s)
end

# Flatten a tuple of per-band system tuples into one flat tuple of systems.
# Recursion (rather than `reduce`/splat) keeps every step concretely typed.
_flatten_systems(::Tuple{}) = ()
_flatten_systems(band_systems::Tuple) =
    (first(band_systems)..., _flatten_systems(Base.tail(band_systems))...)

get_num_ants(num_ants::NumAnts{N}) where {N} = N

create_post_corr_filter(num_ants::NumAnts{N}) where {N} =
    EigenBeamformer(get_num_ants(num_ants))
create_post_corr_filter(::NumAnts{1}) = Tracking.DefaultPostCorrFilter()

# The single canonical way this receiver builds a `TrackedSat`: each signal's
# default correlator, the beamformer injected as post-correlation filter for the
# multi-antenna case, and the receiver's Doppler estimator seeding the per-sat
# estimator state. A `TrackState`'s satellite-slot types are frozen at
# construction, so the empty slot-type template in `ReceiverState` and the
# acquisition handover in `process` must both construct sats through here —
# otherwise `merge_sats` rejects the handed-over sats for their slot type.
function create_tracked_sat(
    signals::Tuple{AbstractGNSSSignal,Vararg{AbstractGNSSSignal}},
    prn,
    code_phase,
    carrier_doppler,
    num_ants::NumAnts,
    doppler_estimator::Tracking.AbstractDopplerEstimator,
)
    TrackedSat(
        signals,
        prn,
        code_phase,
        carrier_doppler;
        num_ants,
        post_corr_filter = create_post_corr_filter(num_ants),
        doppler_estimator,
    )
end

# ─────────────────────────────────────────────────────────────────────────────
# Pilot + data combined tracking
#
# Many GNSS signals split into a data component carrying the navigation message
# (e.g. Galileo E1B, GPS L5I) and a dataless pilot (e.g. Galileo E1C, GPS L5Q).
# `Tracking` tracks both in one group off a single shared carrier/code loop: the
# receiver ranges on the *pilot* (dataless, so its prompt has no navigation-bit
# transitions to squash the discriminator) and decodes the message from the *data*.
#
# `CombinedSignal(pilot, data)` declares such a pair. Everywhere the receiver takes
# a *signal system* it is either a plain `AbstractGNSSSignal` (data-only) or a
# `CombinedSignal`; the helpers below map a system to the concrete signals each
# stage needs, keeping the rest of the receiver system-agnostic.
"""
    CombinedSignal(pilot, data)

A pilot + data signal pair tracked jointly in one group off a shared carrier/code loop: the
receiver ranges on the dataless `pilot` (its prompt has no navigation-bit transitions to
squash the discriminator) and decodes the navigation message from the `data` component —
e.g. `CombinedSignal(GalileoE1C(), GalileoE1B())` or `CombinedSignal(GPSL5Q(), GPSL5I())`.

Pass it anywhere [`receive`](@ref) / [`ReceiverState`](@ref) take a signal system; a plain
`AbstractGNSSSignal` (e.g. `GPSL1CA()`) is the data-only alternative.
"""
struct CombinedSignal{P<:AbstractGNSSSignal,D<:AbstractGNSSSignal}
    pilot::P
    data::D
end

# The signal whose navigation message is decoded, and that identifies the
# satellite everywhere downstream (tracking-group key, PVT `system`,
# constellation, `sat_data` key).
data_signal(system::AbstractGNSSSignal) = system
data_signal(system::CombinedSignal) = system.data

# The signal the code/carrier loops range on — the `Tracking` estimator-driver
# (first entry of `tracking_signals`), and the one the CN0 and lock detectors
# read.
ranging_signal(system::AbstractGNSSSignal) = system
ranging_signal(system::CombinedSignal) = system.pilot

# Cap on Acquisition's secondary-code rotation search. `plan_band_acquisition` passes this
# explicitly to `plan_acquire` (rather than leaning on its matching default, which
# Acquisition exposes no constant for), so this is the single source of truth for
# both the plan's actual cap and the `acquisition_signal` gate below — they cannot
# drift. A pilot whose secondary code is longer than this cannot have its (unknown)
# code stripped by the rotation search, so it is only acquirable coherently at a
# single code period (`nc == 1`), where no rotation search runs.
const MAX_SECONDARY_CODE_ROTATIONS = 32

# The signal acquisition runs on. For a CombinedSignal the (dataless) pilot is the
# better target — more power, and its known secondary code can be stripped
# (`use_secondary_code=true`) to integrate past the data component's navigation-bit
# cap; tracking ranges on the pilot regardless and its code phase seeds the group.
#
# But Acquisition's rotation search accepts a coherent length `nc` only at whole
# multiples of the secondary-code length `L` (a partial period gives a ±Doppler
# sign-ambiguity mirror, Acquisition #68) and only for `L ≤ MAX_SECONDARY_CODE_ROTATIONS`.
# So prefer the pilot ONLY when it needs no more coherent integration *time* than the
# data component for the same resolution (no over-integration) and its window is within
# the rotation cap. Over-integration comes from snapping `nc` up to a multiple of `L`
# (L5Q needs 20 ms vs L5I's 10 ms at 100 Hz) or a long primary code (L2CL's 1.5 s code
# overshoots even at `nc == 1`); comparing windows in absolute time catches both.
#
# Net: L1C-P / B1C-P take the pilot; L5Q pilot at ≤50 Hz else L5I data; E1C pilot at
# ≤10 Hz else E1B data; L2CL falls to L2CM data above ~0.67 Hz; E5aQ / B2a-Q (L=100>32)
# always fall back to data. A plain (non-combined) signal is its own acquisition signal.
acquisition_signal(system::AbstractGNSSSignal, sampling_freq, acq_doppler_resolution) = system
function acquisition_signal(system::CombinedSignal, sampling_freq, acq_doppler_resolution)
    pilot, data = system.pilot, system.data
    # Coherent length each component needs to meet the requested resolution, snapped
    # to a length `plan_acquire` accepts (the same value `plan_band_acquisition` will build).
    snap(s) = snap_coherent_code_periods(
        s, sampling_freq, coherent_code_periods_for_resolution(s, acq_doppler_resolution))
    nc_pilot = snap(pilot)
    within_cap =
        nc_pilot == 1 || get_secondary_code_length(pilot) <= MAX_SECONDARY_CODE_ROTATIONS
    no_over_integration =
        coherent_integration_time(pilot, nc_pilot) <= coherent_integration_time(data, snap(data))
    (within_cap && no_over_integration) ? pilot : data
end

# The signal tuple handed to a `Tracking` group: the ranging (driver) signal
# first, the data signal last. For a plain signal the two coincide, so the
# group tracks that single signal.
tracking_signals(system::AbstractGNSSSignal) = (system,)
tracking_signals(system::CombinedSignal) = (system.pilot, system.data)

# Per-signal selectors into `tracking_signals(system)` for the `Tracking`
# accessors: ranging is always the first signal, data always the last. Both are
# `1` for a plain signal.
const RANGING_SIGNAL_INDEX = 1
data_signal_index(system) = length(tracking_signals(system))

# Tracking-group key / `sat_data` key for a system: the id of the signal the loops
# range on — the pilot for a `CombinedSignal`, and the signal itself (its data
# component) for a plain signal. This is exactly what PVT keys `pvt.sats` by (we hand
# `calc_pvt` the ranging signal), so tracking groups, `receiver_sat_states`, acq
# plans, `sat_data` and `pvt.sats` all share one `(signal, prn)` namespace. Decoding
# still runs on the group's data component regardless of the key.
signal_group_key(system) = get_signal_id(ranging_signal(system))

# RF band of a system, taken from its ranging signal (a pilot/data pair shares one
# band). Used by `assert_single_band`.
system_band(system) = get_band(ranging_signal(system))

# Normalise a single system or a collection of systems to a tuple of systems.
as_systems(systems::Tuple) = systems
as_systems(system::AbstractGNSSSignal) = (system,)
as_systems(system::CombinedSignal) = (system,)

# A single sample stream (SDR front-end or file) can only carry one RF band, so
# every requested system must share one. `get_band` exposes the shared carrier
# at the type level — GPS L1 C/A and Galileo E1B both report `L1()`, so multi-GNSS
# L1 works — letting us reject a genuinely un-tunable mix (e.g. L1 + L5) instead
# of silently dropping the rest of the systems.
function assert_single_band(systems)
    bands = map(system_band, systems)
    allequal(bands) || throw(
        ArgumentError(
            "All systems must share one RF band; got bands $bands",
        ),
    )
    nothing
end

# A pilot component carries no navigation data, so a system is only trackable if its
# data component is decodable — i.e. `GNSSDecoderState` has a method for it. The
# decoder method table is the authoritative definition of "data-carrying", so no
# separate pilot trait is needed.
is_decodable(system) = hasmethod(GNSSDecoderState, Tuple{typeof(data_signal(system)),Int})

# Reject a system whose data component cannot be decoded — e.g. a bare pilot such as
# `GPSL5Q()` passed as a system, or a `CombinedSignal` with a non-data `.data` slot.
# Tracking a pilot alone yields no navigation message, ephemeris or PVT, and would
# otherwise fail deep in decoder construction with a cryptic `MethodError`.
function assert_decodable(systems)
    for system in systems
        is_decodable(system) || throw(
            ArgumentError(
                "System with data component $(get_signal_id(data_signal(system))) is not " *
                "decodable; tracking a pilot-only signal is not supported — pair it with " *
                "its data component via `CombinedSignal(pilot, data)`.",
            ),
        )
    end
    nothing
end

# RF centre frequency to tune the front-end to for a single-band group of systems:
# the shared band's carrier, taken from the ranging signal. Used to set the SDR RX
# frequency (see `rx_center_frequency` caller in the SoapySDR setup).
function rx_center_frequency(systems)
    assert_single_band(systems)
    get_center_frequency(ranging_signal(first(systems)))
end

# Primary constructor: build one multi-band receiver state from the per-band
# system tuples and pre-built per-band acquisition buffers (keyed by `band_key`).
# All systems across all bands become tracking groups in a single `TrackState`,
# keyed by their (unique) group key.
function ReceiverState(
    band_systems::Tuple,
    acquisition_buffers::NamedTuple;
    num_ants::NumAnts = NumAnts(1),
    doppler_estimator::Tracking.AbstractDopplerEstimator = ConventionalAssistedPLLAndDLL(),
)
    systems = _flatten_systems(band_systems)
    assert_decodable(systems)
    group_keys = map(signal_group_key, systems)
    # One tracking group per system: a plain signal alone, a `CombinedSignal` as its
    # pilot (ranging driver) + data component (see `tracking_signals`). Each group
    # carries its own band, so a single `TrackState` spans every band. Each group's
    # empty satellite dictionary is typed after a template built through
    # `create_tracked_sat` — the same constructor the acquisition handover uses — so
    # acquired sats merge without a slot-type mismatch (see `create_tracked_sat`).
    groups = NamedTuple{group_keys}(map(systems) do system
        sigs = tracking_signals(system)
        template = create_tracked_sat(sigs, 0, 0.0, 0.0Hz, num_ants, doppler_estimator)
        sats = Dictionary{Int,typeof(template)}(Int[], typeof(template)[])
        SignalGroup(get_band(first(sigs)), sats, sigs, num_ants)
    end)
    track_state = TrackState(groups, doppler_estimator)
    receiver_sat_states = NamedTuple{group_keys}(map(systems) do system
        DS = typeof(GNSSDecoderState(data_signal(system), 1))
        Dictionary{Int,ReceiverSatState{DS}}()
    end)
    # One acquisition timer per band, keyed like the buffers.
    band_keys = keys(acquisition_buffers)
    last_time_acquisition_ran = NamedTuple{band_keys}(map(_ -> -Inf * 1.0s, band_keys))
    pvt = PositionVelocityTime.PVTSolution()
    pvt_sat_state_buffer = PositionVelocityTime.SatelliteState[]
    ReceiverState(
        track_state,
        receiver_sat_states,
        acquisition_buffers,
        last_time_acquisition_ran,
        pvt,
        pvt_sat_state_buffer,
        0.0s,
        -Inf * 1.0s,
    )
end

"""
    ReceiverState(T, systems; num_samples_for_acquisition, num_ants = NumAnts(1), kwargs...)

Build the initial receiver state for tracking `systems`, where `T` is the element type of
the incoming signal samples (e.g. `ComplexF64` or `Complex{Int16}`).

`systems` is a single GNSS system, a [`CombinedSignal`](@ref) pilot+data pair, or a tuple
of these sharing one RF band; each becomes a tracking group in a single `TrackState`,
keyed by its ranging signal's id. `num_samples_for_acquisition` sizes the acquisition
sample buffer, and `num_ants` selects single- versus multi-antenna processing.
`doppler_estimator` pins the tracking loops' estimator (it must match the one whose
pull-in range sizes acquisition). One `ReceiverState` spans every band; pass the per-band
system tuples and pre-built acquisition buffers to the primary constructor for the
multi-band case.
"""
function ReceiverState(
    ::Type{T}, # Must be the same type as the incoming signal
    systems;
    num_samples_for_acquisition,
    num_ants::NumAnts = NumAnts(1),
    doppler_estimator::Tracking.AbstractDopplerEstimator = ConventionalAssistedPLLAndDLL(),
) where {T}
    systems = as_systems(systems)
    band_key = get_band_id(system_band(first(systems)))
    buffers = NamedTuple{(band_key,)}((SampleBuffer(T, num_samples_for_acquisition),))
    ReceiverState((systems,), buffers; num_ants, doppler_estimator)
end

include("read_file.jl")
include("receive.jl")
include("process.jl")
include("gui.jl")
include("save_data.jl")

"""
    gnss_receiver_gui(; system = GPSL1CA(), sampling_freq = 2e6Hz, kwargs...)

Acquire, track and compute a PVT solution from a live SoapySDR device and display the
result in a live terminal GUI. Blocks until the stream ends.

`system` is a single system or a tuple of systems sharing one RF band (see
[`receive`](@ref) / [`CombinedSignal`](@ref)). The device selected by `dev_args` is
configured for `sampling_freq`, tuned to the shared band's centre frequency and streamed
for `run_time` — all handled by SignalChannels' `stream_data`, which buffers a few seconds
of signal for acquisition headroom and rechunks to the `chunk_time` processing length. Set
`gain` to a fixed value or leave it `nothing` for automatic gain control; `antenna`,
`interm_freq` and `num_ants` override the RF front end and antenna-channel count (one
identical channel config is used per antenna). Samples are streamed as `stream_type`
(default `ComplexF32`); pass `stream_type = Complex{Int16}` to use Tracking's fast integer
backend on an Int16-native device, in which case `max_meas` (the front-end full-scale) is
required. It is ignored for float stream types.
"""
function gnss_receiver_gui(;
    system = GPSL1CA(),
    sampling_freq = 2e6Hz,
    # Duration of each processing chunk fed to `receive` (tracking granularity /
    # latency). The acquisition coherent-integration length is chosen internally by
    # `receive`, not by this value.
    chunk_time = 4ms,
    run_time = 40s,
    num_ants = NumAnts(2),
    dev_args = first(Devices()),
    interm_freq = 0.0Hz,
    gain::Union{Nothing,<:Unitful.Gain} = nothing,
    antenna = nothing,
    stream_type = ComplexF32,
    # Front-end full-scale, required only when `stream_type` is `Complex{Int16}` (the
    # integer downconvert-and-correlator); ignored for float stream types (see `receive`).
    max_meas = nothing,
)
    systems = as_systems(system)
    num_samples_per_chunk = Int(upreferred(sampling_freq * chunk_time))
    eval_num_samples = Int(upreferred(sampling_freq * run_time))

    # One identical RX config per antenna channel. SignalChannels' `stream_data` opens
    # and configures the device (sample rate, bandwidth, centre frequency, gain / AGC,
    # antenna), buffers `buffer_time` of signal for acquisition headroom, and rechunks
    # the stream to the processing chunk length.
    channel_config = SDRChannelConfig(;
        sample_rate = sampling_freq,
        frequency = rx_center_frequency(systems),
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
        chunk_size = num_samples_per_chunk,
        buffer_time = 5s,
    )
    # Surface SDR overflow/timeout warnings without blocking the pipeline.
    warning_task = Threads.@spawn for w in warning_channel
        @warn "SDR stream warning" type = w.type time = w.time_str
    end
    Base.errormonitor(warning_task)

    # Performing GNSS acquisition and tracking. `receive`'s single-channel convenience
    # method wraps the stream and systems into the one-band tuple form.
    data_channel = receive(data_stream, systems, sampling_freq; num_ants, interm_freq, max_meas)

    gui_channel = get_gui_data_channel(data_channel)

    # Display the GUI and block
    GNSSReceiver.gui(gui_channel)
end

"""
    gnss_write_to_file(; system = GPSL1CA(), sampling_freq = 2e6Hz, run_time = 4s,
                       dev_args = first(Devices()), output_file = "gnss_test_data",
                       gain = 50dB)

Stream `run_time` of raw `Complex{Int16}` samples from a SoapySDR device (tuned to the
shared band's centre frequency, at `gain` or AGC when `gain === nothing`) to disk, for
later offline replay with [`read_files`](@ref) and [`receive`](@ref). SignalChannels'
`write_to_file` names each antenna channel's file `"\$(output_file)\$(type)\$(channel).dat"`
— e.g. `"gnss_test_dataComplex{Int16}1.dat"` with the defaults — so pass that full name
(not `output_file` itself) to `read_files` when replaying. Blocks until the recording
finishes.
"""
function gnss_write_to_file(;
    system = GPSL1CA(),
    sampling_freq = 2e6Hz,
    run_time = 4s,
    dev_args = first(Devices()),
    output_file = "gnss_test_data",
    gain::Union{Nothing,<:Unitful.Gain} = 50dB,
)
    eval_num_samples = Int(upreferred(sampling_freq * run_time))

    channel_config = SDRChannelConfig(;
        sample_rate = sampling_freq,
        frequency = rx_center_frequency(as_systems(system)),
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

function receive_and_gui(
    files;
    clock_drift = 0.0,
    system = GPSL1CA(),
    sampling_freq = 5e6Hz,
    type = Complex{Int16},
    num_ants = NumAnts(4),
    # Required for the default `Complex{Int16}` file type (integer backend); leave
    # `nothing` for float file types (see `receive`).
    max_meas = nothing,
)
    #    files = map(i -> "/mnt/data_disk/measurementComplex{Int16}$i.dat", 1:2)
    systems = as_systems(system)
    close_stream_event = Base.Event()
    adjusted_sample_freq = sampling_freq * (1 - clock_drift)

    num_samples_to_receive = Int(upreferred(sampling_freq * 4ms))
    measurement_channel =
        read_files(files, num_samples_to_receive, close_stream_event; type)

    # Let's receive GPS L1 signals
    data_channel = receive(
        measurement_channel,
        systems,
        adjusted_sample_freq;
        num_ants,
        interm_freq = clock_drift * get_center_frequency(first(systems)),
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
