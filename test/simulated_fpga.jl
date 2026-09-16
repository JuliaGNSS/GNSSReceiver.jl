# ─────────────────────────────────────────────────────────────────────────────
# A simulated hardware correlator: the software stand-in for what the
# LiteX-M2SDR gateware does. It correlates the samples it is handed with the
# replicas its channels hold, dumps a record per code period, strobes the epoch
# clock, and applies the NCO updates the host sends back at their scheduled
# sample — a device that honours `apply_at_sample` from a real queue, so the
# feedback delay is exactly the `feedback_delay_epochs` the link asked for.
#
# Shared by the closed-loop tests (test/hardware_correlator.jl,
# test/nco_referenced_loop.jl) and the offline replay of a recorded stream
# (examples/analysis/replay_simfpga.jl), so the two cannot drift apart. It
# depends on nothing but GNSSReceiver: include it after `using GNSSReceiver`.
#
# Nothing here is a mock: the loop really has to close through it, so if the
# ingest path fed the estimator the wrong accumulator order, the wrong spacing,
# a wrong epoch tag or dropped the feedback, the satellite would lose lock.
# ─────────────────────────────────────────────────────────────────────────────

using GNSSReceiver:
    CorrelatorDump,
    NCOUpdate,
    AbstractHardwareCorrelatorSDR,
    HardwareCorrelatorCapabilities,
    epoch_strobe
using GNSSReceiver.Tracking: CorrelatorOutput, EarlyPromptLateCorrelator
using GNSSReceiver.GNSSSignals:
    AbstractGNSSSignal,
    get_band,
    get_band_id,
    get_code,
    get_code_length,
    get_code_frequency,
    get_modulation,
    get_signal_id
using GNSSReceiver.StaticArrays: SVector, MVector
using GNSSReceiver.Unitful: Hz, ustrip, uconvert

# The single-antenna E/P/L correlator the device dumps, accumulators in
# Tracking's order: [late, prompt, early].
sim_epl(late, prompt, early) =
    EarlyPromptLateCorrelator(SVector{3,ComplexF64}(late, prompt, early), 1)
const SimEPL = typeof(sim_epl(0, 0, 0))

# One channel's replica state, i.e. what the gateware's NCOs hold.
mutable struct SimulatedChannel
    prn::Int
    carrier_phase::Float64      # cycles
    carrier_doppler::Float64    # Hz
    code_phase::Float64         # chips
    code_doppler::Float64       # Hz
    el_offset_samples::Float64  # prompt→early lead, in samples
    accumulators::MVector{3,ComplexF64}   # [late, prompt, early]
    integrated_samples::Int
    active::Bool
end

SimulatedChannel() =
    SimulatedChannel(0, 0.0, 0.0, 0.0, 0.0, 0.0, zero(MVector{3,ComplexF64}), 0, false)

# Every field is concretely typed, including `raw` and `system`. That is not
# cosmetic: `GPSL1CA` is a *parametric* type (`GPSL1CA{Matrix{Int16}}` — it
# carries its code table), so a bare `system::GPSL1CA` field is abstract, and
# the per-sample `get_code(sdr.system, …)` in `correlate_chunk!` then goes
# through dynamic dispatch and boxes its result: ~430 B per sample, i.e.
# ~1.7 GB of garbage per second of replayed signal, which is most of the
# closed-loop test's runtime.
mutable struct SimulatedFPGA{
    C,
    R<:GNSSReceiver.SignalChannel,
    S<:AbstractGNSSSignal,
} <: AbstractHardwareCorrelatorSDR
    const raw::R
    const dumps::GNSSReceiver.PipeChannel{CorrelatorDump{C}}
    const ncos::GNSSReceiver.PipeChannel{NCOUpdate}
    const channels::Vector{SimulatedChannel}
    const lock::ReentrantLock
    const system::S
    const sampling_freq::Float64
    const epoch_length::Int
    # The device's free-running sample counter, shared by both streams.
    sample_count::Int
    # NCO updates accepted but not yet due.
    const scheduled::Vector{NCOUpdate}
    # Every update applied, and the device sample it took effect at. A device
    # that honours the schedule has `applied_at[i] == applied[i].apply_at_sample`;
    # an update that arrived late is applied on arrival, later than scheduled.
    const applied::Vector{NCOUpdate}
    const applied_at::Vector{Int}
    const handovers::Vector{Any}
    # A deliberate handover code-phase error, in chips. Real handovers are never
    # exact, and it is what forces the DLL to actually do something: without it
    # the replica starts on truth and the code loop's sign is unobservable over a
    # short run.
    const handover_code_phase_error::Float64
end

"""
    SimulatedFPGA(system; sampling_freq, chunk, n_channels = 20, sample_type = ComplexF64,
                  epoch_length = chunk, handover_code_phase_error = 0.0)

A simulated hardware correlator for `system` at `sampling_freq` (Hz, plain or
Unitful), fed `chunk`-sample raw chunks of `sample_type` through
[`correlate_chunk!`](@ref) and strobing the epoch clock every `epoch_length`
samples.

Twenty channels by default, as the gnss-m2sdr gateware has: with fewer, the
false alarms a first scan produces can occupy every channel and starve the real
satellites of a handover (measured with eight: seven false alarms blocked every
real satellite for a minute).
"""
function SimulatedFPGA(
    system::AbstractGNSSSignal;
    sampling_freq,
    chunk::Integer,
    n_channels::Integer = 20,
    sample_type::Type = ComplexF64,
    epoch_length::Integer = chunk,
    handover_code_phase_error::Real = 0.0,
    raw_capacity::Integer = 4,
    dump_capacity::Integer = 1 << 16,
    nco_capacity::Integer = 1 << 12,
)
    SimulatedFPGA(
        GNSSReceiver.SignalChannel{sample_type,1}(chunk, raw_capacity),
        GNSSReceiver.PipeChannel{CorrelatorDump{SimEPL}}(dump_capacity),
        GNSSReceiver.PipeChannel{NCOUpdate}(nco_capacity),
        [SimulatedChannel() for _ = 1:n_channels],
        ReentrantLock(),
        system,
        sampling_freq isa Real ? Float64(sampling_freq) :
        Float64(ustrip(Hz, uconvert(Hz, sampling_freq))),
        Int(epoch_length),
        0,
        NCOUpdate[],
        NCOUpdate[],
        Int[],
        Any[],
        Float64(handover_code_phase_error),
    )
end

GNSSReceiver.raw_sample_channel(sdr::SimulatedFPGA) = sdr.raw
GNSSReceiver.correlator_dump_channel(sdr::SimulatedFPGA) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::SimulatedFPGA) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::SimulatedFPGA) = length(sdr.channels)

# What this device can do, declared the way a real one has to (issue #131).
# Without it the link takes any device for the legacy GPS L1 C/A one and refuses
# to arm a channel for anything else — so a simulated device built for GPS L5I
# could never be handed a satellite. Declared from the system it was built for,
# which for `GPSL1CA()` reproduces `LEGACY_GPS_L1CA_CAPABILITIES` exactly: one
# three-tap bank per channel reaching one chip either side of prompt, one
# antenna, one band, the primary code only, and the replica's code phase latched
# alongside the accumulators.
function GNSSReceiver.hardware_capabilities(sdr::SimulatedFPGA)
    code_freq = ustrip(Hz, uconvert(Hz, get_code_frequency(sdr.system)))
    HardwareCorrelatorCapabilities(;
        signals = [get_signal_id(sdr.system)],
        modulations = [nameof(typeof(get_modulation(sdr.system)))],
        max_primary_code_length = get_code_length(sdr.system),
        code_frequency_limits = (code_freq, code_freq),
        tap_layouts = [3],
        max_tap_offset_chips = 1.0,
        num_antennas = 1,
        bands = [get_band_id(get_band(sdr.system))],
        num_rf_inputs = 1,
        max_secondary_code_length = 1,
        reports_code_phase = true,
    )
end

function GNSSReceiver.release_channel!(sdr::SimulatedFPGA, hw_channel)
    @lock sdr.lock sdr.channels[hw_channel].active = false
    nothing
end

function GNSSReceiver.assign_channel!(
    sdr::SimulatedFPGA,
    hw_channel,
    prn,
    carrier_doppler,
    code_doppler,
    code_phase,
    valid_at_sample;
    el_sample_spacing,
    signal,
)
    @lock sdr.lock begin
        # The handover describes the satellite at `valid_at_sample` on the host's
        # raw-sample count. This device's counter is the same stream, so the
        # phase only has to be propagated over the samples generated since.
        carrier_doppler_hz = ustrip(uconvert(Hz, carrier_doppler))
        code_doppler_hz = ustrip(uconvert(Hz, code_doppler))
        code_freq = ustrip(uconvert(Hz, get_code_frequency(sdr.system))) + code_doppler_hz
        elapsed = sdr.sample_count - valid_at_sample
        ch = sdr.channels[hw_channel]
        ch.prn = prn
        ch.carrier_doppler = carrier_doppler_hz
        ch.carrier_phase = 0.0
        ch.code_doppler = code_doppler_hz
        ch.code_phase = mod(
            code_phase + sdr.handover_code_phase_error +
            code_freq * elapsed / sdr.sampling_freq,
            get_code_length(sdr.system),
        )
        # Program exactly the spacing the host quantised; half of the E-to-L
        # distance is the prompt→early lead.
        ch.el_offset_samples = el_sample_spacing / 2
        ch.accumulators .= 0
        ch.integrated_samples = 0
        ch.active = true
        push!(sdr.handovers, (; hw_channel, prn, el_sample_spacing, valid_at_sample, signal))
    end
    nothing
end

# Apply every scheduled update whose sample has arrived. This is the deterministic
# apply point that makes the feedback delay a constant.
function apply_due_ncos!(sdr::SimulatedFPGA)
    while Base.n_avail(sdr.ncos) > 0
        push!(sdr.scheduled, take!(sdr.ncos))
    end
    isempty(sdr.scheduled) && return sdr
    due = filter(u -> u.apply_at_sample <= sdr.sample_count, sdr.scheduled)
    filter!(u -> u.apply_at_sample > sdr.sample_count, sdr.scheduled)
    for u in due
        ch = sdr.channels[u.channel]
        (ch.active && ch.prn == u.prn) || continue
        ch.carrier_doppler = u.carrier_doppler
        ch.code_doppler = u.code_doppler
        push!(sdr.applied, u)
        push!(sdr.applied_at, sdr.sample_count)
    end
    sdr
end

"""
    correlate_chunk!(sdr::SimulatedFPGA, samples) -> Vector{CorrelatorDump}

Correlate one chunk of raw samples with every active channel's replica, push the
records it completed (and the epoch strobes) onto the dump stream, and return
them. Call it *before* handing the same chunk to the raw channel, exactly like
the gateware's observer sees a sample only once DMA0 accepted it.
"""
function correlate_chunk!(sdr::SimulatedFPGA{C}, samples) where {C}
    code_length = get_code_length(sdr.system)
    nominal_code_freq = ustrip(uconvert(Hz, get_code_frequency(sdr.system)))
    out = CorrelatorDump{C}[]
    @lock sdr.lock begin
        for k in eachindex(samples)
            apply_due_ncos!(sdr)
            sample = ComplexF64(samples[k])
            for (index, ch) in enumerate(sdr.channels)
                ch.active || continue
                code_freq = nominal_code_freq + ch.code_doppler
                el_chips = ch.el_offset_samples * code_freq / sdr.sampling_freq
                wipeoff = sample * cis(-2π * ch.carrier_phase)
                # [late, prompt, early] — Tracking's accumulator order.
                for (slot, offset) in ((1, -el_chips), (2, 0.0), (3, el_chips))
                    code = get_code(sdr.system, ch.code_phase + offset, ch.prn)
                    ch.accumulators[slot] += wipeoff * code
                end
                ch.carrier_phase += ch.carrier_doppler / sdr.sampling_freq
                ch.code_phase += code_freq / sdr.sampling_freq
                ch.integrated_samples += 1
                if ch.code_phase >= code_length
                    ch.code_phase -= code_length
                    push!(
                        out,
                        CorrelatorDump(
                            index,
                            ch.prn,
                            CorrelatorOutput(
                                EarlyPromptLateCorrelator(
                                    SVector{3,ComplexF64}(ch.accumulators),
                                    1,
                                ),
                                ch.integrated_samples,
                                sdr.sample_count + 1,
                            ),
                            # The replica's code phase at the dump sample — the
                            # absolute anchor a real device latches alongside the
                            # accumulators (`dump_code_phase` on the M2SDR).
                            ch.code_phase,
                        ),
                    )
                    ch.accumulators .= 0
                    ch.integrated_samples = 0
                end
            end
            sdr.sample_count += 1
            # The timebase marker, emitted regardless of what the channels did.
            if sdr.sample_count % sdr.epoch_length == 0
                push!(out, epoch_strobe(sim_epl(0, 0, 0), sdr.sample_count))
            end
        end
    end
    isempty(out) || put!(sdr.dumps, out)
    out
end

"""
    replay_raw_file!(sdr::SimulatedFPGA, path; chunk, seconds = Inf, words_per_sample = 4)

Stream a recorded `Int16` interleaved raw file (the LiteX-M2SDR's 2R2T sc16
layout: `I₁ Q₁ I₂ Q₂` per sample, antenna 1 = the first two words) through the
simulated device: each `chunk` of antenna-1 samples is correlated and then put
on the raw channel as `Complex{Int16}`. Returns a task; the raw channel is
closed when the file (or `seconds` of it) has been consumed.
"""
function replay_raw_file!(
    sdr::SimulatedFPGA,
    path::AbstractString;
    chunk::Integer,
    seconds::Real = Inf,
    words_per_sample::Integer = 4,
)
    task = Threads.@spawn begin
        try
            open(path) do io
                raw8 = Vector{UInt8}(undef, 2 * words_per_sample * chunk)
                n = 0
                while n * chunk / sdr.sampling_freq < seconds
                    try
                        read!(io, raw8)
                    catch e
                        e isa EOFError ? break : rethrow()
                    end
                    raw = reinterpret(Int16, raw8)
                    buf = Matrix{Complex{Int16}}(undef, chunk, 1)
                    @inbounds for k = 1:chunk
                        buf[k, 1] = Complex(
                            raw[words_per_sample*(k-1)+1],
                            raw[words_per_sample*(k-1)+2],
                        )
                    end
                    correlate_chunk!(sdr, view(buf, :, 1))
                    put!(sdr.raw, buf)
                    n += 1
                end
            end
        finally
            close(sdr.raw)
        end
    end
    Base.errormonitor(task)
    task
end
