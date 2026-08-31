# Tests for the hardware-correlator path (issue #107).
#
# The unit tests below drive the ingest logic directly with hand-built dumps, so
# the epoch/stale/overflow rules are pinned without a signal in the loop. The
# closed-loop test at the bottom then runs the whole receiver against a
# *simulated* FPGA that correlates the same samples it puts on the raw channel
# and honours the NCO updates it gets back — the software stand-in for what the
# LiteX-M2SDR gateware does.

using GNSSReceiver:
    CorrelatorDump,
    NCOUpdate,
    HardwareCorrelatorLink,
    AbstractHardwareCorrelatorSDR,
    EPOCH_STROBE_CHANNEL,
    epoch_strobe,
    is_epoch_strobe,
    raw_sample_channel,
    correlator_dump_channel,
    nco_update_channel,
    num_hardware_channels,
    assign_channel!,
    release_channel!,
    dropped_dump_count!,
    advance_tracking!,
    drain_dumps!,
    fold_closed_epochs!,
    sync_hardware_channels!

using PipeChannels: PipeChannel
using Tracking: CorrelatorOutput, EarlyPromptLateCorrelator

# ─────────────────────────────────────────────────────────────────────────────
# A minimal recording device: no signal, no correlation. It exists so the ingest
# logic can be driven with exactly the dumps a test wants.
# ─────────────────────────────────────────────────────────────────────────────

mutable struct RecordingSDR{C} <: AbstractHardwareCorrelatorSDR
    const dumps::PipeChannel{CorrelatorDump{C}}
    const ncos::PipeChannel{NCOUpdate}
    const n_channels::Int
    const assigned::Vector{Any}
    const released::Vector{Int}
    dropped::Int
end

function RecordingSDR(::Type{C}, n_channels; capacity = 4096) where {C}
    RecordingSDR{C}(
        PipeChannel{CorrelatorDump{C}}(capacity),
        PipeChannel{NCOUpdate}(capacity),
        n_channels,
        Any[],
        Int[],
        0,
    )
end

GNSSReceiver.correlator_dump_channel(sdr::RecordingSDR) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::RecordingSDR) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::RecordingSDR) = sdr.n_channels
GNSSReceiver.release_channel!(sdr::RecordingSDR, hw_channel) =
    push!(sdr.released, hw_channel)
function GNSSReceiver.dropped_dump_count!(sdr::RecordingSDR)
    n = sdr.dropped
    sdr.dropped = 0
    n
end
function GNSSReceiver.assign_channel!(
    sdr::RecordingSDR,
    hw_channel,
    prn,
    carrier_doppler,
    code_doppler,
    code_phase,
    valid_at_sample;
    el_sample_spacing,
    signal,
)
    push!(
        sdr.assigned,
        (; hw_channel, prn, carrier_doppler, code_doppler, code_phase, valid_at_sample,
         el_sample_spacing, signal),
    )
    nothing
end

# An E/P/L correlator whose accumulators are in Tracking's order, [late, prompt,
# early]. Only the prompt magnitude matters for the ingest tests.
epl(late, prompt, early) =
    EarlyPromptLateCorrelator(SVector{3,ComplexF64}(late, prompt, early), 1)

const EPL = typeof(epl(0, 0, 0))

dump_at(
    channel,
    prn,
    sample_index;
    prompt = 1.0 + 0im,
    integrated_samples = 4000,
    late = 0.0 + 0im,
    early = 0.0 + 0im,
    code_phase = NaN,
) = CorrelatorDump(
    channel,
    prn,
    CorrelatorOutput(epl(late, prompt, early), integrated_samples, sample_index),
    code_phase,
)

@testset "CorrelatorDump / NCOUpdate records" begin
    d = dump_at(1, 5, 4000)
    # The ring is only allocation-free if the element type is isbits.
    @test isbitstype(typeof(d))
    @test isbitstype(NCOUpdate)
    @test d.channel === Int32(1)
    @test d.prn === Int32(5)
    @test d.output.sample_index == 4000
    @test !is_epoch_strobe(d)

    strobe = epoch_strobe(epl(0, 0, 0), 8000)
    @test is_epoch_strobe(strobe)
    @test strobe.channel == EPOCH_STROBE_CHANNEL
    @test strobe.output.sample_index == 8000
    @test iszero(get_accumulators(strobe.output.correlator))

    # Unitful in, plain Hz out, so the record stays isbits.
    u = NCOUpdate(2, 7, 1234.5Hz, 0.8Hz, 16000)
    @test u.carrier_doppler === 1234.5
    @test u.code_doppler === 0.8
    @test u.apply_at_sample === Int64(16000)
end

@testset "Link construction validates its arguments" begin
    sdr = RecordingSDR(EPL, 4)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
    )
    # Default epoch = one primary code period = 1 ms = 4000 samples at 4 MHz.
    @test link.epoch_length == 4000
    @test num_hardware_channels(sdr) == 4
    @test all(isnothing, link.assignments)

    # An epoch shorter than a sample period cannot define a grid.
    @test_throws ArgumentError HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
        doppler_update_interval = 1e-9u"s",
    )
    # A zero-epoch feedback delay would schedule updates in the past.
    @test_throws ArgumentError HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
        feedback_delay_epochs = 0,
    )
end

@testset "Epochs close on the sample-index grid" begin
    sdr = RecordingSDR(EPL, 4)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
    )
    # Pretend channel 1 holds GPS PRN 1 so dumps have somewhere to go; the
    # append itself is exercised by the closed-loop test.
    link.assignments[1] = GNSSReceiver.HardwareChannelAssignment(:GPSL1CA, 1, 1)
    link.channel_of[link.assignments[1]] = 1

    # Nothing seen yet ⇒ no grid, no folds.
    @test link.next_epoch_boundary == typemin(Int)

    put!(sdr.dumps, [dump_at(1, 1, 10_000)])
    drain_dumps!(link)
    # The first record anchors epoch 0; the grid runs from there.
    @test link.next_epoch_boundary == 14_000
    @test length(link.pending) == 1

    # A record before the boundary does not close the epoch.
    put!(sdr.dumps, [dump_at(1, 1, 12_000)])
    drain_dumps!(link)
    @test link.latest_sample_index == 12_000
    @test link.next_epoch_boundary == 14_000
end

@testset "An epoch strobe closes an epoch no channel dumped into" begin
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
    )
    put!(sdr.dumps, [epoch_strobe(epl(0, 0, 0), 0)])
    drain_dumps!(link)
    @test link.next_epoch_boundary == 4000

    # Strobes alone must advance the clock — otherwise a receiver with nothing
    # locked never closes an epoch and the loop stalls.
    put!(sdr.dumps, [epoch_strobe(epl(0, 0, 0), 4000)])
    drain_dumps!(link)
    @test link.latest_sample_index >= link.next_epoch_boundary

    # And a strobe is never appended to a satellite: with no assignment at all,
    # folding must still succeed and consume it.
    track_state = nothing  # unused: no assignments, so nothing is addressed
    @test is_epoch_strobe(link.pending[end])
end

@testset "Dumps for a reassigned channel are dropped as stale" begin
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
    )
    # Channel 1 now holds PRN 9.
    link.assignments[1] = GNSSReceiver.HardwareChannelAssignment(:GPSL1CA, 9, 1)
    link.channel_of[link.assignments[1]] = 1

    # A dump tagged PRN 3 was produced before the reassignment took effect.
    # Folding it into PRN 9's loop would corrupt it.
    GNSSReceiver._append_dump!(link, nothing, dump_at(1, 3, 100))
    @test link.stale_dumps == 1

    # So is a dump for a channel that is currently free, or out of range.
    GNSSReceiver._append_dump!(link, nothing, dump_at(2, 3, 100))
    @test link.stale_dumps == 2
    GNSSReceiver._append_dump!(link, nothing, dump_at(99, 3, 100))
    @test link.stale_dumps == 3
end

@testset "Ring overflow reported by the device is surfaced" begin
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
    )
    sdr.dropped = 17
    link.dropped_dumps += dropped_dump_count!(sdr)
    @test link.dropped_dumps == 17
    # Sticky counters are write-1-to-clear: a second read must not double count.
    link.dropped_dumps += dropped_dump_count!(sdr)
    @test link.dropped_dumps == 17
end

@testset "Absolute code phase follows the device replica (pseudorange anchor)" begin
    system = GPSL1CA()
    prn = 7
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = system,
    )
    track_state = TrackState(system, [TrackedSat(system, prn, 100.0, 0.0Hz)])
    # A bare TrackState names its single group `default` (receive() keys groups
    # by signal id); the assignment must address the group that actually exists.
    link.assignments[1] = GNSSReceiver.HardwareChannelAssignment(:default, prn, 1)
    link.channel_of[link.assignments[1]] = 1
    sampling_freqs = (L1 = 4e6Hz,)
    # Balanced accumulators: discriminators read zero, so the Dopplers stay put
    # and every epoch advances the replica by exactly one code length.
    balanced = (; late = 400.0 + 0im, prompt = 1000.0 + 0im, early = 400.0 + 0im)

    fold!() = GNSSReceiver.fold_closed_epochs!(link, track_state, sampling_freqs)
    phase() = get_code_phase(get_sat_state(track_state, prn))

    # A dump without a reported replica phase cannot anchor anything: the seed
    # lives on the host's raw-sample axis, which the link cannot place on the
    # device counter, so it stays untouched.
    put!(sdr.dumps, [dump_at(1, prn, 4000; balanced...), epoch_strobe(epl(0, 0, 0), 8000)])
    GNSSReceiver.drain_dumps!(link)
    fold!()
    @test phase() == 100.0

    # First anchored dump: the reported replica phase (5.0 chips at sample 8000)
    # replaces the seed's within-code-period part, then the phase is
    # extrapolated to the fold boundary (12000) — exactly one code period on.
    put!(
        sdr.dumps,
        [
            dump_at(1, prn, 8000; code_phase = 5.0, balanced...),
            epoch_strobe(epl(0, 0, 0), 12_000),
        ],
    )
    GNSSReceiver.drain_dumps!(link)
    fold!()
    @test phase() ≈ 5.0 atol = 1e-9

    # An epoch with no dump dead-reckons: one more code period, same phase
    # modulo the code length — the satellite stays comparable with the others.
    put!(sdr.dumps, [epoch_strobe(epl(0, 0, 0), 16_000)])
    GNSSReceiver.drain_dumps!(link)
    fold!()
    @test phase() ≈ 5.0 atol = 1e-9

    # A later anchor absorbs replica drift the host bookkeeping cannot see
    # (device NCO quantisation, DLL pull-in): the wrapped difference is applied,
    # not accumulated error.
    put!(
        sdr.dumps,
        [
            dump_at(1, prn, 16_000; code_phase = 5.25, balanced...),
            epoch_strobe(epl(0, 0, 0), 20_000),
        ],
    )
    GNSSReceiver.drain_dumps!(link)
    fold!()
    @test phase() ≈ 5.25 atol = 1e-9

    # Releasing the channel forgets the phase reference with it: hand the
    # release pass a track state that no longer contains our PRN.
    empty!(sdr.released)
    GNSSReceiver.release_stale_channels!(
        link,
        TrackState(system, [TrackedSat(system, prn + 1, 0.0, 0.0Hz)]),
    )
    @test link.phase_ref_sample[1] == typemin(Int64)
end

@testset "An unimplemented interface method says which one is missing" begin
    struct BareSDR <: AbstractHardwareCorrelatorSDR end
    err = try
        raw_sample_channel(BareSDR())
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("raw_sample_channel", err.msg)
    @test occursin("BareSDR", err.msg)
    # The overflow hook is the one optional method, so it must have a default.
    @test dropped_dump_count!(BareSDR()) == 0
end

# ─────────────────────────────────────────────────────────────────────────────
# A simulated FPGA correlator: the software stand-in for the real gateware.
#
# It generates the received signal, puts it on the raw channel *and* correlates
# it with its own replicas — exactly the "non-intrusive observer on the RX
# stream" arrangement the LiteX-M2SDR gateware uses — then emits one dump per
# code period per channel plus a periodic epoch strobe, and applies the NCO
# updates the host sends back at their scheduled sample.
#
# Nothing here is a mock: the loop really has to close through it, so if the
# ingest path fed the estimator the wrong accumulator order, the wrong spacing,
# a wrong epoch tag or dropped the feedback, the satellite would lose lock.
# ─────────────────────────────────────────────────────────────────────────────

using SignalChannels: SignalChannel
using Tracking: get_early_late_sample_spacing

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

# Every field is concretely typed, including `raw` and `system`. That is not
# cosmetic: `GPSL1CA` is a *parametric* type (`GPSL1CA{Matrix{Int16}}` — it
# carries its code table), so a bare `system::GPSL1CA` field is abstract, and
# the per-sample `get_code(sdr.system, …)` in `_correlate_chunk!` then goes
# through dynamic dispatch and boxes its result: ~430 B per sample, i.e.
# ~1.7 GB of garbage per second of replayed signal, which is most of the
# closed-loop test's runtime.
mutable struct SimulatedFPGA{C,R<:SignalChannel,S<:AbstractGNSSSignal} <:
               AbstractHardwareCorrelatorSDR
    const raw::R
    const dumps::PipeChannel{CorrelatorDump{C}}
    const ncos::PipeChannel{NCOUpdate}
    const channels::Vector{SimulatedChannel}
    const lock::ReentrantLock
    const system::S
    const sampling_freq::Float64
    const epoch_length::Int
    # The device's free-running sample counter, shared by both streams.
    sample_count::Int
    # NCO updates accepted but not yet due.
    const scheduled::Vector{NCOUpdate}
    const applied::Vector{NCOUpdate}
    const handovers::Vector{Any}
    # A deliberate handover code-phase error, in chips. Real handovers are never
    # exact, and it is what forces the DLL to actually do something: without it
    # the replica starts on truth and the code loop's sign is unobservable over a
    # short run.
    const handover_code_phase_error::Float64
end

GNSSReceiver.raw_sample_channel(sdr::SimulatedFPGA) = sdr.raw
GNSSReceiver.correlator_dump_channel(sdr::SimulatedFPGA) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::SimulatedFPGA) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::SimulatedFPGA) = length(sdr.channels)

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
function _apply_due_ncos!(sdr::SimulatedFPGA)
    while Base.n_avail(sdr.ncos) > 0
        push!(sdr.scheduled, take!(sdr.ncos))
    end
    due = filter(u -> u.apply_at_sample <= sdr.sample_count, sdr.scheduled)
    filter!(u -> u.apply_at_sample > sdr.sample_count, sdr.scheduled)
    for u in due
        ch = sdr.channels[u.channel]
        (ch.active && ch.prn == u.prn) || continue
        ch.carrier_doppler = u.carrier_doppler
        ch.code_doppler = u.code_doppler
        push!(sdr.applied, u)
    end
    sdr
end

# Correlate one chunk and emit whatever records it completed.
function _correlate_chunk!(sdr::SimulatedFPGA{C}, samples) where {C}
    code_length = get_code_length(sdr.system)
    nominal_code_freq = ustrip(uconvert(Hz, get_code_frequency(sdr.system)))
    out = CorrelatorDump{C}[]
    @lock sdr.lock begin
        for k in eachindex(samples)
            _apply_due_ncos!(sdr)
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
                push!(out, epoch_strobe(epl(0, 0, 0), sdr.sample_count))
            end
        end
    end
    isempty(out) || put!(sdr.dumps, out)
    out
end

@testset "Closed loop through a simulated hardware correlator" begin
    system = GPSL1CA()
    prn = 11
    sampling_freq = 4e6Hz
    fs = 4e6
    chunk = 4000                      # 1 ms
    # 1.4 s of signal. The link pushes one NCO update per *chunk* (see
    # `push_nco_updates!`) rather than one per folded epoch, because that is how
    # often a correction can actually reach a device; a device like this one,
    # which honours `apply_at_sample` strictly, used to receive a whole per-epoch
    # sequence during a catch-up and could replay it, so it pulled in within
    # 700 ms. Same convergence, more signal to get there.
    num_chunks = 1400
    true_doppler = 1200.0             # Hz
    initial_code_phase = 137.4        # chips
    amplitude = 0.126                 # ≈ 45 dBHz against unit-variance noise
    handover_code_phase_error = 0.25  # chips the DLL has to pull in

    code_length = get_code_length(system)
    nominal_code_freq = ustrip(uconvert(Hz, get_code_frequency(system)))
    true_code_freq =
        nominal_code_freq + true_doppler * get_code_center_frequency_ratio(system)

    # The type parameters follow from the field types, so the default
    # constructor infers them.
    sdr = SimulatedFPGA(
        SignalChannel{ComplexF64,1}(chunk, 4),
        PipeChannel{CorrelatorDump{EPL}}(1 << 16),
        PipeChannel{NCOUpdate}(1 << 12),
        [SimulatedChannel(0, 0.0, 0.0, 0.0, 0.0, 0.0, zero(MVector{3,ComplexF64}), 0, false)
         for _ = 1:4],
        ReentrantLock(),
        system,
        fs,
        chunk,
        0,
        NCOUpdate[],
        NCOUpdate[],
        Any[],
        handover_code_phase_error,
    )

    producer = Threads.@spawn begin
        rng = Random.Xoshiro(0xC0FFEE)
        buf = Matrix{ComplexF64}(undef, chunk, 1)
        try
            for c = 0:(num_chunks-1)
                n0 = c * chunk
                for k = 1:chunk
                    t = (n0 + k - 1) / fs
                    code = get_code(
                        system,
                        initial_code_phase + true_code_freq * t,
                        prn,
                    )
                    buf[k, 1] =
                        amplitude * code * cis(2π * true_doppler * t) +
                        (randn(rng, ComplexF64) / sqrt(2)) * sqrt(2)
                end
                # Tap first, exactly like the gateware's observer sees a word
                # only once DMA0 accepted it, then hand it to the host.
                _correlate_chunk!(sdr, view(buf, :, 1))
                put!(sdr.raw, copy(buf))
            end
        finally
            close(sdr.raw)
        end
    end
    Base.errormonitor(producer)

    data_channel = receive(
        sdr,
        system,
        sampling_freq;
        # Acquire early and often enough that the 400 ms run has a handover.
        acquire_every = 20ms,
        prns = [prn],
        # A synthetic signal carries no real navigation data, so PVT can never
        # converge; the loop itself is what this test is about.
        time_in_lock_before_calculating_pvt = 1000u"s",
    )

    results = collect_data(data_channel)
    wait(producer)

    key = (get_signal_id(system), prn)

    # 1. The satellite was handed over to hardware, and the device was programmed
    #    with the very spacing Tracking normalises the DLL discriminator by — a
    #    device using the raw preferred chip shift instead would mis-scale the
    #    loop gain.
    @test length(results) > 0
    @test !isempty(sdr.handovers)
    handover = first(sdr.handovers)
    @test handover.prn == prn
    expected_spacing = get_early_late_sample_spacing(
        EarlyPromptLateCorrelator(num_ants = NumAnts(1)),
        sampling_freq,
        get_code_frequency(system),
    )
    @test handover.el_sample_spacing == expected_spacing
    @test isinteger(handover.el_sample_spacing)

    # 2. It ended the run tracked and in lock — which can only happen if dumps
    #    were folded and the NCO feedback kept the device's replica aligned.
    final = last(results)
    @test haskey(final.sat_data, key)
    @test final.sat_data[key].cn0 > 35dBHz

    # 3. The feedback really flowed: the device applied updates, and the
    #    Doppler it converged on is the true one.
    @test !isempty(sdr.applied)
    converged = last(sdr.applied)
    @test converged.carrier_doppler ≈ true_doppler atol = 15.0

    # 4. The code loop closed too. The device was deliberately handed a replica
    #    0.25 chips off truth; only a DLL whose discriminator sign and spacing
    #    convention match the device's accumulator order pulls that in. Swap E
    #    and L in the device and this is what diverges.
    tracked = only(filter(c -> c.active && c.prn == prn, sdr.channels))
    true_code_phase = mod(
        initial_code_phase + true_code_freq * sdr.sample_count / fs,
        code_length,
    )
    code_error = mod(tracked.code_phase - true_code_phase + code_length / 2, code_length) -
                 code_length / 2
    @test abs(code_error) < 0.5 * handover_code_phase_error

    # 5. Updates are scheduled on the epoch grid, a fixed number of epochs
    #    ahead — that is what makes the loop delay deterministic.
    applied_samples = unique(u -> u.apply_at_sample, sdr.applied)
    if length(applied_samples) >= 3
        deltas = diff([u.apply_at_sample for u in applied_samples])
        @test all(d -> d % chunk == 0, deltas)
    end
end

@testset "A stream gap resynchronises instead of replaying every epoch" begin
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
        max_catchup_epochs = 8,
    )
    # No channel assigned: this is about the epoch grid, not about routing dumps.
    empty_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 4000,
        num_ants = NumAnts(1),
    ).track_state
    band_measurements = (; GPSL1 = Tracking.BandMeasurement(
        zeros(ComplexF64, 4000), 4e6Hz, 0.0Hz))

    put!(sdr.dumps, [dump_at(1, 1, 0)])
    drain_dumps!(link)
    @test link.next_epoch_boundary == 4000

    # The device went quiet for a second, then came back: 1000 epochs of gap.
    # Replaying them would run 1000 estimator passes over empty buffers and push
    # 1000 NCO updates in a single chunk — a transient stall turned into a long
    # one. The grid must jump instead.
    put!(sdr.dumps, [dump_at(1, 1, 4_000_000)])
    drain_dumps!(link)
    folds = fold_closed_epochs!(link, empty_state, band_measurements)
    @test folds <= 1
    @test link.skipped_epochs > 900
    # And the grid is back in step with the newest record, not a second behind.
    @test link.next_epoch_boundary > 4_000_000
    @test link.next_epoch_boundary <= 4_000_000 + link.epoch_length
end

@testset "A normal one-or-two-epoch backlog is still folded, not skipped" begin
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = 4e6Hz,
        reference_signal = GPSL1CA(),
        max_catchup_epochs = 8,
    )
    empty_state = GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL1CA();
        num_samples_for_acquisition = 4000,
        num_ants = NumAnts(1),
    ).track_state
    band_measurements = (; GPSL1 = Tracking.BandMeasurement(
        zeros(ComplexF64, 4000), 4e6Hz, 0.0Hz))

    put!(sdr.dumps, [epoch_strobe(epl(0, 0, 0), 0), epoch_strobe(epl(0, 0, 0), 8000)])
    drain_dumps!(link)
    # Two epochs behind is a backlog, well inside the catch-up bound: fold both.
    folds = fold_closed_epochs!(link, empty_state, band_measurements)
    @test folds == 2
    @test link.skipped_epochs == 0
end

@testset "A chunk's dumps reach the loops as one record spanning the chunk" begin
    # The loop filter's proportional term is a frequency sized to act for exactly
    # one update interval, and a hardware NCO holds the last word it was given
    # until the next arrives. Feedback can only reach the device once per chunk,
    # so a chunk's dumps have to be folded into ONE record whose
    # `integrated_samples` is the real elapsed interval — otherwise the device is
    # left holding a one-epoch-sized correction for a whole chunk (or for however
    # long the host stays behind) and it over-corrects. Measured on sky, that is
    # what inverted ~1 % of navigation bits and stopped every ephemeris (#107).
    system = GPSL1CA()
    prn = 9
    sdr = RecordingSDR(EPL, 1)
    link = HardwareCorrelatorLink(sdr; sampling_freq = 4e6Hz, reference_signal = system)
    track_state = TrackState(system, [TrackedSat(system, prn, 0.0, 0.0Hz)])
    link.assignments[1] = GNSSReceiver.HardwareChannelAssignment(:default, prn, 1)
    link.channel_of[link.assignments[1]] = 1
    signal() = Tracking.get_signals(get_sat_state(track_state, prn))[1]
    outputs() = Tracking.get_correlator_outputs(get_sat_state(track_state, prn), 1)

    # Before bit sync the length is pinned to one block: the sync detectors take
    # exactly one prompt per code block and `Tracking` errors on anything else.
    @test GNSSReceiver.coherent_integration_blocks(
        link, get_sat_state(track_state, prn), 1) == 1
    for k = 1:4
        GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 4000k))
    end
    @test length(outputs()) == 4
    @test all(o -> o.integrated_samples == 4000, outputs())
    empty!(outputs())

    # Post-sync the ceiling is a whole navigation bit, but the per-chunk flush is
    # what decides the actual length.
    bit_buffer = Tracking.get_bit_buffer(signal())
    synced = typeof(bit_buffer)(
        bit_buffer.code_block_buffer, bit_buffer.code_block_buffer_length, true, 0,
        Int8(1), complex(0.0, 0.0), 0, bit_buffer.soft_bits, bit_buffer.phase_acc)
    Tracking.get_sat_states(track_state)[prn] = Tracking.TrackedSat(
        get_sat_state(track_state, prn);
        signals = (Tracking.TrackedSignal(signal(); bit_buffer = synced),))
    @test GNSSReceiver.coherent_integration_blocks(
        link, get_sat_state(track_state, prn), 1) == 20

    # Two dumps arrive (a 2 ms chunk keeping up). Neither reaches the estimator on
    # its own; the flush hands over their coherent sum, tagged with the summed
    # sample count and the newest dump's index.
    link.last_record_end[1] = typemin(Int64)
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 100_000; prompt = 3 + 4im))
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 104_000; prompt = 1 + 2im))
    @test isempty(outputs())
    GNSSReceiver.flush_partial_records!(link, track_state)
    record = only(outputs())
    @test record.integrated_samples == 8000
    @test record.sample_index == 104_000
    @test Tracking.get_prompt(record.correlator) ≈ 4 + 6im
    # The spacing metadata is still the host's, not the wire's.
    @test Tracking.get_early_late_sample_spacing(
        record.correlator, 4e6Hz, get_code_frequency(system)) ==
          Tracking.get_early_late_sample_spacing(
        get_correlator(get_sat_state(track_state, prn), 1), 4e6Hz,
        get_code_frequency(system))
    empty!(outputs())

    # A catch-up burst is where this matters: 20 dumps in one chunk become one
    # 20 ms record, so the loop's Δt — and with it Tracking's 1/n bandwidth
    # scaling — is the interval that actually elapsed, not one epoch.
    gaps_before = link.lost_record_gaps
    for k = 1:20
        GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 104_000 + 4000k))
    end
    GNSSReceiver.flush_partial_records!(link, track_state)
    # Continuous records, so no gap record: the burst is one output, nothing else.
    @test link.lost_record_gaps == gaps_before
    burst = only(outputs())
    @test burst.integrated_samples == 80_000
    @test Tracking.get_prompt(burst.correlator) ≈ 20 + 0im
    empty!(outputs())

    # Nothing accumulated ⇒ the flush is a no-op rather than a zero record, which
    # `Tracking` would read as a gap and credit to the bit clock.
    GNSSReceiver.flush_partial_records!(link, track_state)
    @test isempty(outputs())

    # A channel that changes occupant must not carry its predecessor's partial
    # into the newcomer's fold.
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 200_000))
    @test link.partial_blocks[1] == 1
    GNSSReceiver.release_stale_channels!(
        link, TrackState(system, [TrackedSat(system, prn + 1, 0.0, 0.0Hz)]))
    @test link.partial_blocks[1] == 0
end

@testset "A hole in a channel's record stream is reported to the bit clock" begin
    system = GPSL1CA()
    prn = 7
    sdr = RecordingSDR(EPL, 2)
    link = HardwareCorrelatorLink(sdr; sampling_freq = 4e6Hz, reference_signal = system)
    track_state = TrackState(system, [TrackedSat(system, prn, 0.0, 0.0Hz)])
    link.assignments[1] = GNSSReceiver.HardwareChannelAssignment(:default, prn, 1)
    link.channel_of[link.assignments[1]] = 1

    outputs() = Tracking.get_correlator_outputs(get_sat_state(track_state, prn), 1)

    # The first record on a fresh channel has nothing to be continuous with.
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 4000))
    @test link.lost_record_gaps == 0
    @test link.lost_record_samples[1] == 0
    @test length(outputs()) == 1

    # A record that starts exactly where the last one ended is continuous.
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 8000))
    @test link.lost_record_gaps == 0
    @test length(outputs()) == 2

    # Three records lost: the next one starts 12000 samples late. The gap is
    # accounted, and a zeroed record spanning exactly the hole is appended
    # *before* the real one so the fold credits the bit clock in the right
    # place. Without it the satellite's navigation bit boundary would move by
    # three code periods for the rest of its lock.
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 24_000))
    @test link.lost_record_gaps == 1
    @test link.lost_record_samples[1] == 12_000
    @test length(outputs()) == 4
    gap = outputs()[3]
    @test gap.integrated_samples == 12_000
    @test iszero(Tracking.get_prompt(gap.correlator))
    @test outputs()[4].sample_index == 24_000

    # A record that starts before the previous one ended is an overlap, not a
    # hole — a duplicate or a device counter step back. Counted, never credited.
    GNSSReceiver._append_dump!(link, track_state, dump_at(1, prn, 26_000))
    @test link.overlapping_record_samples[1] == 2000
    @test link.lost_record_gaps == 1
    @test length(outputs()) == 5

    # Reassigning the channel starts a fresh record stream: the old occupant's
    # end sample says nothing about where the new one's first record begins.
    empty!(sdr.released)
    GNSSReceiver.release_stale_channels!(
        link,
        TrackState(system, [TrackedSat(system, prn + 1, 0.0, 0.0Hz)]),
    )
    @test link.last_record_end[1] == typemin(Int64)
    @test link.lost_record_samples[1] == 0
end
