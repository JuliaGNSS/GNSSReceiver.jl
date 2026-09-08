# The vendor half of GNSSReceiver.jl#107 for the LiteX-M2SDR running the
# gnss-m2sdr gateware.
#
# GNSSReceiver owns the records, the epoch clock and the loop; this file owns
# everything device-specific: draining DMA1 into `CorrelatorDump`s, turning
# `NCOUpdate`s into scheduled CSR commits, and programming a channel on
# acquisition handover.

# The correlator type a dump carries. `EarlyPromptLateCorrelator{M,T}` has
# M = antennas and T = accumulator element type, which is a bare `ComplexF64`
# for one antenna and an `SVector` for more — so it cannot be spelled as one
# parametric alias.
correlator_type(::Val{1}) = Tracking.EarlyPromptLateCorrelator{1,ComplexF64}
correlator_type(::Val{N}) where {N} =
    Tracking.EarlyPromptLateCorrelator{N,SVector{N,ComplexF64}}

mutable struct M2SDRCorrelator{N,C} <: GNSSReceiver.AbstractHardwareCorrelatorSDR
    const csr::LiteXCSR
    const bank::GNSSBank
    const raw::SignalChannel
    const dumps::PipeChannel{GNSSReceiver.CorrelatorDump{C}}
    const ncos::PipeChannel{GNSSReceiver.NCOUpdate}
    const fs::Float64
    const handover_margin::Int
    const dma_device::String
    const codes::Dict{Int,Vector{Int}}
    # The device counter reading that corresponds to host raw-sample count 0.
    # Both streams count the same samples, so the mapping is this one constant
    # (see `_device_sample`).
    device_origin::Int64
    reader::Union{Task,Nothing}
    writer::Union{Task,Nothing}
    running::Bool
    # `dump_source = :csr` only: dumps the poller provably skipped (dump_count
    # advanced by more than one). A skipped dump is a missing bit-buffer prompt,
    # which scrambles that satellite's decoded bit stream — watch this when
    # decoding matters.
    missed_csr_dumps::Int
    # Which channels currently hold a satellite, and its PRN — maintained by
    # `assign_channel!` / `release_channel!`. The CSR poller only services
    # active channels (every channel dumps at ~1 kHz whether assigned or not,
    # and a full dump readout is ~11 CSR ioctls: polling all of them pushes the
    # pass time past the dump period and *guarantees* missed dumps), and it
    # tags dumps with the assigned PRN rather than paying one more ioctl.
    const active::Vector{Bool}
    const assigned_prns::Vector{Int32}
end

"""
    M2SDRCorrelator(csr_csv, raw; fs, n_channels = :detect, n_ants = 1, kwargs...)

A LiteX-M2SDR with the gnss-m2sdr tracking gateware, as an
`AbstractHardwareCorrelatorSDR`.

`raw` is the device's raw sample stream as a `SignalChannel` — this package does
not own it, because how you get I/Q off the board (SoapySDR, `LiteXM2SDR.jl`'s
shared-memory streamer, …) is independent of the correlator. It must keep
running for the whole session: the tracking bank is a *non-intrusive observer*
on the RX datapath, so it only sees samples while DMA0 is draining. Stop the raw
stream and the correlators stop, silently.

Keywords:

  - `csr_device` / `dma_device` — the litepcie char devices (`/dev/m2sdr0` for
    CSRs, `/dev/m2sdr1` for the DMA1 record stream).
  - `n_channels` — hardware tracking channels to drive. Defaults to counting
    the `gnss_ch<i>_` banks in the CSR map, i.e. whatever the flashed gateware
    actually has ([`detect_num_channels`](@ref)).
  - `n_ants` — antenna blocks to read per record (≤ 2, the AD9361's 2T2R limit).
  - `handover_margin` — how far ahead of the device's current sample counter an
    acquisition handover is scheduled. Must exceed the CSR write round trip, or
    the commit lands late (visible through `apply_status`).
"""
function M2SDRCorrelator(
    csr_csv::AbstractString,
    raw::SignalChannel;
    fs,
    n_channels::Union{Integer,Symbol} = :detect,
    n_ants::Integer = 1,
    csr_device::AbstractString = "/dev/m2sdr0",
    dma_device::AbstractString = "/dev/m2sdr1",
    handover_margin::Integer = 0,
    dump_capacity::Integer = 1 << 16,
    nco_capacity::Integer = 1 << 12,
)
    1 <= n_ants <= N_ANTS_MAX ||
        throw(ArgumentError("n_ants must be 1..$N_ANTS_MAX (the AD9361 is 2T2R)"))
    fs_hz = Float64(ustrip(uconvert(Hz, fs)))
    csr = LiteXCSR(csr_csv; device = csr_device)
    resolved_channels = n_channels === :detect ? detect_num_channels(csr) : Int(n_channels)
    bank = GNSSBank(csr; fs = fs_hz, n_channels = resolved_channels)

    device_ants = num_ants(bank)
    device_ants == n_ants || @warn(
        "gateware reports $device_ants antenna block(s) per record but $n_ants " *
        "requested; the host will read $n_ants"
    )

    # Default the handover margin to 10 ms — comfortably more than a CSR write
    # round trip over PCIe, and short enough that the acquisition's code phase
    # has not aged much by the time it commits.
    margin = handover_margin > 0 ? Int(handover_margin) : round(Int, fs_hz / 100)
    C = correlator_type(Val(Int(n_ants)))

    M2SDRCorrelator{Int(n_ants),C}(
        csr,
        bank,
        raw,
        PipeChannel{GNSSReceiver.CorrelatorDump{C}}(dump_capacity),
        PipeChannel{GNSSReceiver.NCOUpdate}(nco_capacity),
        fs_hz,
        margin,
        String(dma_device),
        Dict{Int,Vector{Int}}(),
        Int64(0),
        nothing,
        nothing,
        false,
        0,
        fill(false, resolved_channels),
        zeros(Int32, resolved_channels),
    )
end

# ── The GNSSReceiver interface ───────────────────────────────────────────────

GNSSReceiver.raw_sample_channel(sdr::M2SDRCorrelator) = sdr.raw
GNSSReceiver.correlator_dump_channel(sdr::M2SDRCorrelator) = sdr.dumps
GNSSReceiver.nco_update_channel(sdr::M2SDRCorrelator) = sdr.ncos
GNSSReceiver.num_hardware_channels(sdr::M2SDRCorrelator) = length(sdr.bank.channels)

# The gateware wipes the carrier off with a ±127 sin/cos ROM where a host
# correlator uses a unit-amplitude replica, so its accumulators are 127× the
# prompt the same samples would give on the CPU (measured on sky: 126.9–128.1
# across six satellites, issue #107). The ingest divides it out so the prompt
# and the noise density Tracking measures from the raw stream share one scale.
GNSSReceiver.correlator_gain(::M2SDRCorrelator) = 127

# The gateware's overflow status is a sticky per-channel bitmap, not a count, so
# report the number of channels that overflowed since the last read and clear.
function GNSSReceiver.dropped_dump_count!(sdr::M2SDRCorrelator)
    bits = overflow(sdr.bank)
    bits == 0 && return 0
    clear_overflow!(sdr.bank)
    count_ones(bits)
end

function GNSSReceiver.release_channel!(sdr::M2SDRCorrelator, hw_channel)
    sdr.active[hw_channel] = false
    ch = sdr.bank.channels[hw_channel]
    write(sdr.csr, ch.prefix * "control", 0)
    nothing
end

function GNSSReceiver.assign_channel!(
    sdr::M2SDRCorrelator,
    hw_channel,
    prn,
    carrier_doppler,
    code_doppler,
    code_phase,
    valid_at_sample;
    el_sample_spacing,
    signal,
)
    ch = sdr.bank.channels[hw_channel]
    carrier_hz = Float64(ustrip(uconvert(Hz, carrier_doppler)))
    code_doppler_hz = Float64(ustrip(uconvert(Hz, code_doppler)))

    # The code RAM only has to be rewritten when the channel's PRN changes:
    # 1023 back-to-back CSR writes per handover is not just slow, the ioctl
    # storm has been observed to wedge the board's MSI delivery (DMA0 then
    # starves while the sample counter keeps counting). Cache per channel.
    if sdr.assigned_prns[hw_channel] != Int32(prn)
        code = get!(sdr.codes, Int(prn)) do
            Int[get_code(signal, chip, prn) > 0 ? 1 : 0 for chip = 0:(CA_CODE_LENGTH-1)]
        end
        load_code!(ch, prn, code)
    end

    # `el_sample_spacing` is the Early-to-Late distance in whole input samples,
    # already quantised the way Tracking quantises it. The CSR wants the
    # prompt→Early half of that.
    sample_shift = max(1, round(Int, el_sample_spacing / 2))
    write(sdr.csr, ch.prefix * "spacing", spacing_word(ch, sample_shift, code_doppler_hz))

    # Schedule the handover far enough ahead that the CSR writes land first, and
    # propagate the code phase from the sample it was valid at to the sample it
    # will be committed on. Retry on a late commit: the first handover of a
    # session pays JIT compilation between reading the counter and the final
    # apply write, which can push the commit past its target — with a stale
    # phase, the DLL never sees the peak. The retry runs hot and lands.
    code_freq = GPS_CA_CHIP_RATE * (1.0 + carrier_hz / GPS_L1_HZ)
    for attempt = 1:3
        target = sample_count(sdr.bank) + sdr.handover_margin
        elapsed = target - _device_sample(sdr, valid_at_sample)
        code_phase_at_target = mod(
            Float64(code_phase) + code_freq * elapsed / sdr.fs,
            CA_CODE_LENGTH,
        )
        schedule!(
            ch,
            target;
            carrier_hz,
            code_doppler_hz,
            carrier_phase_cycles = 0.0,
            code_phase_chips = code_phase_at_target,
        )
        # `late` is only meaningful once the commit fired; polling `armed`
        # right after the writes reads the pre-commit status.
        deadline = time() + 2 * sdr.handover_margin / sdr.fs + 0.1
        while apply_status(ch).armed && time() < deadline
            # Keep this wait yield-friendly: a hot ioctl spin on the libuv
            # event-loop thread freezes every async IO in the process,
            # including the raw-sample pipe feeding the whole receiver.
            yield()
        end
        status = apply_status(ch)
        if !status.armed && !status.late
            sdr.assigned_prns[hw_channel] = Int32(prn)
            sdr.active[hw_channel] = true
            return nothing
        end
        attempt == 3 && @warn(
            "handover for PRN $prn on channel $hw_channel kept committing late — " *
            "increase handover_margin (currently $(sdr.handover_margin) samples)"
        )
    end
    sdr.assigned_prns[hw_channel] = Int32(prn)
    sdr.active[hw_channel] = true
    nothing
end

# Host raw-sample count → the bank's free-running counter. Both count the same
# samples off the same RX datapath, so they differ by one constant, latched when
# streaming started. A dropped raw buffer would break this; the driver reports
# that as an overrun.
_device_sample(sdr::M2SDRCorrelator, host_sample) = sdr.device_origin + Int64(host_sample)

# ── Session lifecycle ────────────────────────────────────────────────────────

"""
    start!(sdr; epoch_period = 0, device_origin = nothing, dump_source = :dma)

Latch the host↔device sample-counter offset, enable the bank and the epoch
strobe, and spawn the dump reader and the NCO writer.

Call once the raw stream is already flowing: the offset is latched against the
device counter *now*, so it has to be taken when the host's raw sample count is
still zero. That latch is only as exact as the raw path's in-flight buffering;
pass `device_origin` (e.g. from a code-phase sweep calibration against an
acquisition) to override it with a measured value — required for acquisition
handovers programmed straight from the raw stream's code phases.

`epoch_period` is the timebase-strobe period in samples (`0` ⇒ 1 kHz). Strobes
also pace the DMA1 stream: the litepcie driver only completes whole 8 KiB
buffers (64 records), so the strobe rate bounds the dump latency the tracking
loop sees — at the default 1 kHz an idle bank delivers records ~64 ms late,
far beyond what the loop filters tolerate. Raise it (e.g. `fs ÷ 16000`) when
the correlator drives a live feedback loop over DMA.

`dump_source` picks how dumps reach the receiver: `:dma` drains the DMA1
record ring; `:csr` polls every channel's dump CSRs instead (no DMA1 device
needed, latency of one polling pass, but it can miss dumps under load and
fabricates its own strobes from the sample counter).
"""
function start!(
    sdr::M2SDRCorrelator;
    epoch_period::Integer = 0,
    device_origin::Union{Nothing,Integer} = nothing,
    dump_source::Symbol = :dma,
)
    sdr.running && return sdr
    dump_source in (:dma, :csr) ||
        throw(ArgumentError("dump_source must be :dma or :csr, got $dump_source"))
    sdr.device_origin =
        isnothing(device_origin) ? sample_count(sdr.bank) : Int64(device_origin)
    period = epoch_period > 0 ? epoch_period : round(Int, sdr.fs / 1000)
    set_epoch_period!(sdr.bank, period)
    clear_overflow!(sdr.bank)
    enable!(sdr.bank, true)
    sdr.running = true
    # The service tasks live on the interactive pool: the CSR poller is a
    # busy loop and the writer parks in take!, and a receiver stack that uses
    # Polyester for acquisition (sticky per-thread worker tasks) deadlocks when
    # long-running default-pool tasks occupy the threads its workers are pinned
    # to. With no interactive threads (-t N alone) they fall back to :default —
    # start julia with `-t N,M` when acquisition runs concurrently.
    if dump_source === :dma
        sdr.reader = Base.errormonitor(Threads.@spawn :interactive _read_dumps!(sdr))
        # The CSR poller drains the NCO ring inline (deadline-bound commits first,
    # and a single consumer — PipeChannel is SPSC); only the DMA source needs a
    # separate writer task.
    sdr.writer = dump_source === :dma ?
        Base.errormonitor(Threads.@spawn :interactive _write_ncos!(sdr)) : nothing
    else
        # One spin loop owns all CSR traffic: the NCO drain runs between dump
        # polls, so commits land within a poll pass of being pushed and never
        # contend with the poller for the ioctl lock.
        sdr.reader = Base.errormonitor(Threads.@spawn :interactive _poll_dumps!(sdr, period))
        sdr.writer = nothing
    end
    sdr
end

function stop!(sdr::M2SDRCorrelator)
    sdr.running || return sdr
    sdr.running = false
    enable!(sdr.bank, false)
    close(sdr.dumps)
    close(sdr.ncos)
    sdr
end

# Drain DMA1 and push `CorrelatorDump`s. Reads whole DMA buffers: the driver
# only completes a buffer when it is full, so anything smaller just blocks.
#
# `DMAWriterStream` starts the channel's DMA writer over ioctl before the first
# read. Without that the driver's read path waits on a buffer counter the
# gateware is never told to advance, so the drain blocks forever and no dump ever
# reaches the receiver — see dma.jl.
function _read_dumps!(sdr::M2SDRCorrelator{N,C}) where {N,C}
    stream = DMAWriterStream(sdr.dma_device)
    records = M2SDRRecord{N}[]
    batch = GNSSReceiver.CorrelatorDump{C}[]
    # The ring occasionally re-delivers a whole buffer, so the same records
    # (identical channel/seq/sample_index) arrive twice. A duplicated dump
    # folded twice double-steps the tracking loops and doubles the prompts per
    # navigation bit — satellites drop within a minute and bit sync never
    # happens. Sample indices of genuine new dumps are strictly increasing per
    # channel (and strobes on their own slot), so drop anything not newer.
    last_sidx = fill(typemin(Int64), length(sdr.bank.channels) + 1)
    try
        while sdr.running
            # Only read when the driver reports a completed buffer: the read
            # itself blocks its OS thread in a ccall, and a task that loops
            # straight into the next blocking read never reaches a yield point
            # — a Polyester sticky worker pinned to this thread then starves
            # and acquisition deadlocks. Parking in `sleep` keeps the thread
            # schedulable; with DMA_BUFFER_PER_IRQ = 1 the counters advance
            # per buffer, so the poll adds at most ~1 ms of dump latency.
            hw_count, sw_count = dma_writer_counts(stream)
            if hw_count == sw_count
                sleep(0.001)
                continue
            end
            data = read_buffers!(stream)
            isempty(data) && break
            empty!(records)
            parse_records!(records, data, Val(N))
            isempty(records) && continue
            empty!(batch)
            for record in records
                slot = is_strobe(record) ? length(last_sidx) : Int(record.channel) + 1
                if 1 <= slot <= length(last_sidx)
                    record.sample_index <= last_sidx[slot] && continue
                    last_sidx[slot] = record.sample_index
                end
                push!(batch, _to_dump(record, Val(N)))
            end
            # Never block the device reader on a full ring: dropping here would
            # be silent, so the bank's own sticky overflow status is what the
            # receiver sees (`dropped_dump_count!`).
            Base.n_avail(sdr.dumps) + length(batch) <= sdr.dumps.capacity - 1 || continue
            put!(sdr.dumps, batch)
        end
    finally
        close(stream)
    end
end

# Wire record → `CorrelatorDump`. Two conventions have to be honoured here and
# nowhere else: the accumulators go in as `[late, prompt, early]` (Tracking's
# order, since `get_prompt_index` is 2 — E/P/L order inverts the DLL), and the
# strobe's reserved channel id becomes GNSSReceiver's sentinel.
#
# The record's `code_phase` is the code NCO's fractional register latched on the
# dump sample. A dump fires on the sample whose advance wraps the last chip, so
# on that sample the replica sits at chip `CA_CODE_LENGTH - 1` plus that
# fraction — the absolute anchor GNSSReceiver's pseudorange bookkeeping wants.
function _to_dump(record::M2SDRRecord{N}, ::Val{N}) where {N}
    accumulators = if N == 1
        SVector{3,ComplexF64}(record.late[1], record.prompt[1], record.early[1])
    else
        SVector{3,SVector{N,ComplexF64}}(
            SVector{N,ComplexF64}(record.late),
            SVector{N,ComplexF64}(record.prompt),
            SVector{N,ComplexF64}(record.early),
        )
    end
    # The spacing argument is placeholder metadata: GNSSReceiver replaces it
    # with the tracked satellite's before the estimator sees it, so a mismatch
    # here cannot mis-normalise `dll_disc`.
    correlator = Tracking.EarlyPromptLateCorrelator(accumulators, 1)
    output = Tracking.CorrelatorOutput(
        correlator,
        Int(record.integrated_samples),
        Int(record.sample_index),
    )
    channel = is_strobe(record) ? GNSSReceiver.EPOCH_STROBE_CHANNEL :
              Int32(record.channel + 1)   # gateware is 0-based, the host 1-based
    code_phase = is_strobe(record) ? NaN :
                 (CA_CODE_LENGTH - 1) + record.code_phase / (1 << CODE_FRAC_BITS)
    GNSSReceiver.CorrelatorDump(channel, Int32(record.prn), output, code_phase)
end

# CSR-polling dump source: read every channel's dump CSRs whenever its
# `dump_count` moves, and fabricate the timebase strobes from the sample
# counter. No DMA1 device needed and a latency of one polling pass, but unlike
# the DMA ring it can *miss* dumps when the poller falls behind — a missed dump
# is a missing bit-buffer prompt, which scrambles the decoder's bit stream — so
# this is a bring-up/diagnostic source; use `:dma` for decoding and PVT.
# Pin the calling OS thread to one CPU core. The poller's pass must run every
# dump period; any preemption is a burst of missed dumps, and a missed dump is
# a broken subframe. Launch julia under `taskset -c 0-(core-1)` and set
# GNSS_POLLER_CORE=<core>: every other thread (Julia pools, GC, FFTW, Polyester)
# inherits the reduced process mask, and the poller alone re-pins itself onto
# the reserved core — CPU storms elsewhere can then never touch it.
function _pin_current_thread(core::Integer)
    tid = ccall(:gettid, Cint, ())
    mask = zeros(UInt8, 128)
    mask[core ÷ 8 + 1] = UInt8(1) << (core % 8)
    rc = ccall(
        (:sched_setaffinity, "libc"),
        Cint,
        (Cint, Csize_t, Ptr{UInt8}),
        tid,
        length(mask),
        mask,
    )
    rc == 0 || @warn "could not pin the dump poller to core $core (errno $(Libc.errno()))"
    rc == 0
end

function _poll_dumps!(sdr::M2SDRCorrelator{N,C}, strobe_period::Integer) where {N,C}
    poller_core = get(ENV, "GNSS_POLLER_CORE", "")
    if !isempty(poller_core)
        # The task must stop migrating between pool threads before the OS-level
        # pin means anything.
        current_task().sticky = true
        _pin_current_thread(parse(Int, poller_core))
    end
    channels = sdr.bank.channels
    prev_counts = fill(-1, length(channels))
    prototype = _prototype_correlator(Val(N))
    last_strobe = sample_count(sdr.bank)
    last_carrier = fill(NaN, length(channels))
    last_code = fill(NaN, length(channels))
    rotate = 0
    while sdr.running
        # NCO commits first: they are deadline-bound (apply_at is only
        # feedback_delay_epochs ahead), dump readout is not.
        try
            _drain_ncos!(sdr, last_carrier, last_code)
        catch e
            e isa InvalidStateException || rethrow(e)
        end
        # Rotate the scan origin every pass: when a pass overruns the dump
        # period, the channels scanned last are the ones that lose dumps, and
        # a fixed order starves the same satellites' bit streams every time.
        rotate = mod1(rotate + 1, length(channels))
        for k in eachindex(channels)
            i = mod1(rotate + k - 1, length(channels))
            ch = channels[i]
            if !sdr.active[i]
                prev_counts[i] = -1
                continue
            end
            count = Int(read(sdr.csr, ch.prefix * "dump_count"))
            count == prev_counts[i] && continue
            dump = _read_dump_csrs(sdr, ch, Val(N))
            isnothing(dump) && continue
            # dump_count is 32-bit and monotonic while the channel runs; a jump
            # of more than one means the poller was outrun and dumps are gone.
            prev_counts[i] >= 0 &&
                (sdr.missed_csr_dumps += mod(dump.count - prev_counts[i] - 1, 1 << 32))
            prev_counts[i] = dump.count
            Base.n_avail(sdr.dumps) < sdr.dumps.capacity - 1 || continue
            put!(
                sdr.dumps,
                GNSSReceiver.CorrelatorDump(
                    Int32(i),
                    sdr.assigned_prns[i],
                    Tracking.CorrelatorOutput(dump.correlator, dump.n, dump.sample_index),
                    dump.code_phase,
                ),
            )
        end
        now = sample_count(sdr.bank)
        if now - last_strobe >= strobe_period
            last_strobe = now
            Base.n_avail(sdr.dumps) < sdr.dumps.capacity - 1 &&
                put!(sdr.dumps, GNSSReceiver.epoch_strobe(prototype, now))
        end
        yield()
    end
end

_prototype_correlator(::Val{1}) =
    Tracking.EarlyPromptLateCorrelator(zero(SVector{3,ComplexF64}), 1)
_prototype_correlator(::Val{N}) where {N} =
    Tracking.EarlyPromptLateCorrelator(zero(SVector{3,SVector{N,ComplexF64}}), 1)

# One coherent CSR dump read: retried until `dump_count` is stable around the
# field reads, so a dump firing mid-read cannot mix two integrations.
function _read_dump_csrs(sdr::M2SDRCorrelator, ch, ::Val{N}; tries::Integer = 10) where {N}
    csr = sdr.csr
    p = ch.prefix
    for _ = 1:tries
        c0 = read(csr, p * "dump_count")
        accumulators = _read_accumulators(csr, p, Val(N))
        n = read(csr, p * "integrated_samples")
        sample_index = read(csr, p * "sample_index")
        frac = read(csr, p * "dump_code_phase")
        if read(csr, p * "dump_count") == c0
            n == 0 && return nothing
            return (
                count = Int(c0),
                n = Int(n),
                sample_index = Int(sample_index),
                code_phase = (CA_CODE_LENGTH - 1) + Int(frac) / (1 << CODE_FRAC_BITS),
                correlator = Tracking.EarlyPromptLateCorrelator(accumulators, 1),
            )
        end
    end
    nothing
end

_acc_suffix(a::Integer) = a == 0 ? "" : "_ant$(a)"

function _read_accumulators(csr, prefix, ::Val{1})
    late = ComplexF64(read_signed(csr, prefix * "il", 32), read_signed(csr, prefix * "ql", 32))
    prompt = ComplexF64(read_signed(csr, prefix * "ip", 32), read_signed(csr, prefix * "qp", 32))
    early = ComplexF64(read_signed(csr, prefix * "ie", 32), read_signed(csr, prefix * "qe", 32))
    SVector{3,ComplexF64}(late, prompt, early)
end

function _read_accumulators(csr, prefix, ::Val{N}) where {N}
    per_ant = ntuple(Val(N)) do ant
        s = _acc_suffix(ant - 1)
        (
            late = ComplexF64(
                read_signed(csr, prefix * "il" * s, 32),
                read_signed(csr, prefix * "ql" * s, 32),
            ),
            prompt = ComplexF64(
                read_signed(csr, prefix * "ip" * s, 32),
                read_signed(csr, prefix * "qp" * s, 32),
            ),
            early = ComplexF64(
                read_signed(csr, prefix * "ie" * s, 32),
                read_signed(csr, prefix * "qe" * s, 32),
            ),
        )
    end
    SVector{3,SVector{N,ComplexF64}}(
        SVector{N,ComplexF64}(map(a -> a.late, per_ant)),
        SVector{N,ComplexF64}(map(a -> a.prompt, per_ant)),
        SVector{N,ComplexF64}(map(a -> a.early, per_ant)),
    )
end

# Turn NCO updates into scheduled CSR commits at their named sample.
#
# Latency here is loop-critical: an update scheduled `feedback_delay_epochs`
# (a few ms) ahead must reach the CSRs before its `apply_at_sample` passes, or
# it commits late with stale values. `PipeChannel`'s blocking single-item
# `take!` parks in a ~10 ms sleep-poll, which batches updates into bursts that
# all land late — the carrier keeps frequency lock but the PLL's phase
# corrections apply at random delays and phase never locks (no data bits).
# Hence the non-blocking batch drain; `:csr` mode calls it from the poller's
# spin loop, `:dma` mode gets a dedicated yield-loop task.
function _drain_ncos!(
    sdr::M2SDRCorrelator,
    last_carrier::Vector{Float64} = Float64[],
    last_code::Vector{Float64} = Float64[],
)
    n = Base.n_avail(sdr.ncos)
    n == 0 && return 0
    # A correction computed for a sample far in the past describes a loop state
    # that no longer exists: committing it steers the NCO with stale data and
    # throws the loop (observed while the host catches up a start-up backlog —
    # every satellite lost within a minute). Let the channel free-run instead
    # (a handover-seeded NCO drifts ~0.5 Hz/s, harmless for tens of seconds)
    # and resume with the first fresh correction.
    now = sample_count(sdr.bank)
    max_stale = Int64(round(0.02 * sdr.fs))   # 20 ms
    for _ = 1:n
        update = take!(sdr.ncos)
        update.apply_at_sample < now - max_stale && continue
        ch = sdr.bank.channels[update.channel]
        # An update that quantizes to the NCO words the channel already runs
        # is a no-op on the device; committing it anyway costs ~6 serialized
        # CSR writes that delay the dump scan (missed dumps = scrambled bits).
        cw = Float64(carrier_word(ch, update.carrier_doppler))
        kw = Float64(code_word_from_code_doppler(ch, update.code_doppler))
        if update.channel <= length(last_carrier) &&
           cw == last_carrier[update.channel] &&
           kw == last_code[update.channel]
            continue
        end
        # Do NOT use the staged commit for streaming frequency updates: the
        # gateware has a single staging register with cancel-on-re-arm
        # semantics, and Tracking's apply_at (fold boundary + 1 epoch) is
        # ~1-2 ms in the future — the next 1 kHz update re-arms and cancels
        # the pending commit before it matures, so NO update ever applied and
        # every channel silently free-ran on its handover words (the
        # walk-off-and-die disease). The immediate CSRs apply on the next
        # sample with no arming; +-2 ms of application jitter is irrelevant
        # at these loop bandwidths. The staged path remains for handovers,
        # which need the sample-exact phase load and are one-shot.
        write(
            sdr.csr,
            ch.prefix * "carrier_freq",
            carrier_word(ch, update.carrier_doppler),
        )
        write(
            sdr.csr,
            ch.prefix * "code_freq",
            code_word_from_code_doppler(ch, update.code_doppler),
        )
        if update.channel <= length(last_carrier)
            last_carrier[update.channel] = cw
            last_code[update.channel] = kw
        end
    end
    n
end

function _write_ncos!(sdr::M2SDRCorrelator)
    while sdr.running
        drained = try
            _drain_ncos!(sdr)
        catch e
            e isa InvalidStateException && break
            rethrow(e)
        end
        # Park when idle: a yield-spin monopolises its (interactive) pool
        # thread and starves the raw reader and drainer sharing the pool. In
        # :dma mode dumps arrive in whole-buffer batches anyway, so a 1 ms nap
        # costs nothing against the fold cadence.
        drained == 0 ? sleep(0.001) : yield()
    end
end
