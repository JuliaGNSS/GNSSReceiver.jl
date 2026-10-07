# Milestone 0, the device rung: open the CSR handle, read DMA1 directly, parse
# the records, measure the read-path latency against the device's own sample
# counter, and write one NCO word per channel — everything the loop process
# will do at the device boundary, built `--trim=safe` with no package but Base.
#
# Run with a raw stream already draining DMA0 (`m2sdr_record -c 0 -q - 0`): the
# bank only sees samples, and only counts them, while DMA0 is being drained.
#
#   rung_device CSR_CSV SECONDS FS_HZ [DMA_DEVICE] [EPOCH_PERIOD]

include(joinpath(@__DIR__, "..", "device", "csr.jl"))
include(joinpath(@__DIR__, "..", "device", "dma.jl"))
include(joinpath(@__DIR__, "..", "device", "record.jl"))
include(joinpath(@__DIR__, "..", "device", "minimal_bank.jl"))

# Latency histogram edges in microseconds; the last bucket is the overflow.
const LATENCY_EDGES_US = (250, 500, 1000, 2000, 4000, 8000, 16000, 50000)

# `Core.println` with several arguments is not resolvable under `--trim`; one
# value per call is. Every report line is therefore `label` then `value`.
report(label::String, value) = (Core.println(Core.stdout, label); Core.println(Core.stdout, value))

function bucket_index(latency_us::Float64)
    for (i, edge) in enumerate(LATENCY_EDGES_US)
        latency_us < edge && return i
    end
    length(LATENCY_EDGES_US) + 1
end

function run_spike(csr_csv::String, seconds::Float64, fs::Float64, dma_device::String, epoch_period::Int)
    csr = LiteXCSR(csr_csv)
    version = has_register(csr, "gnss_version") ? read(csr, "gnss_version") : UInt64(0)
    report("gnss_version_csr", Int(version & 0xFF))
    report("gnss_version_record", Int((version >> 8) & 0xFF))
    caps = read(csr, "gnss_capabilities")
    n_channels = Int(caps & 0xFF)
    report("channels", n_channels)
    set_epoch_period!(csr, epoch_period)
    enable!(csr, true)
    t0_count = sample_count(csr)
    report("sample_count_at_start", t0_count)

    stream = DMAWriterStream(dma_device; buffers = 16)
    buf = Vector{UInt8}(undef, 1 << 20)
    filled = 0
    records = M2SDRRecord{1}[]
    sizehint!(records, 1 << 14)
    hist = zeros(Int, length(LATENCY_EDGES_US) + 1)
    n_records = 0
    n_strobes = 0
    n_reads = 0
    max_read_bytes = 0
    max_loop_ns = 0
    sum_latency_us = 0.0
    min_seq_gap = 0
    dup = 0
    last_sample = typemin(Int64)
    gc_before = Base.gc_num()
    deadline = time_ns() + round(UInt64, seconds * 1e9)
    while time_ns() < deadline
        loop_start = time_ns()
        _poll_readable(stream.fd, 10) || continue
        n = _read_into!(stream.fd, buf, filled)
        n <= 0 && continue
        n_reads += 1
        max_read_bytes = max(max_read_bytes, n)
        filled += n
        empty!(records)
        filled = _take_records!(records, buf, filled, Val(1))
        # The device counter *now*: what the newest record is measured against.
        now = sample_count(csr)
        for r in records
            n_records += 1
            is_strobe(r) && (n_strobes += 1)
            if r.sample_index <= last_sample
                dup += 1
            end
            last_sample = max(last_sample, r.sample_index)
            latency_us = (now - r.sample_index) / fs * 1e6
            sum_latency_us += latency_us
            hist[bucket_index(latency_us)] += 1
        end
        max_loop_ns = max(max_loop_ns, Int(time_ns() - loop_start))
    end
    gc_after = Base.gc_num()
    report("records", n_records)
    report("strobes", n_strobes)
    report("reads", n_reads)
    report("max_read_bytes", max_read_bytes)
    report("duplicates_or_reorders", dup)
    if n_records > 0
        report("mean_latency_us", round(Int, sum_latency_us / n_records))
    end
    report("max_loop_iteration_us", div(max_loop_ns, 1000))
    for i in eachindex(LATENCY_EDGES_US)
        report("latency_bucket_upper_us", LATENCY_EDGES_US[i])
        report("latency_bucket_count", hist[i])
    end
    report("latency_overflow_count", hist[end])
    report("allocated_bytes_in_loop", Int(gc_after.allocd - gc_before.allocd))
    report("gc_pauses_in_loop", Int(gc_after.pause - gc_before.pause))

    # One NCO word per channel, timed: the CSR write path the loop process pays
    # on every commit.
    max_write_ns = 0
    sum_write_ns = 0
    n_writes = 0
    for rep = 1:20, ch = 0:(n_channels-1)
        t = time_ns()
        write_channel_words!(csr, ch, 1000.0 + ch, 1.023e6 + 1.0, fs)
        dt = Int(time_ns() - t)
        max_write_ns = max(max_write_ns, dt)
        sum_write_ns += dt
        n_writes += 1
    end
    report("nco_writes", n_writes)
    report("nco_write_mean_us", div(sum_write_ns, max(1, n_writes) * 1000))
    report("nco_write_max_us", div(max_write_ns, 1000))
    enable!(csr, false)
    close(stream)
    close(csr)
    nothing
end

function (@main)(args::Vector{String})::Cint
    if length(args) < 3
        Core.println(Core.stderr, "usage: rung_device CSR_CSV SECONDS FS_HZ [DMA_DEVICE] [EPOCH_PERIOD]")
        return Cint(2)
    end
    csr_csv = args[1]
    seconds = parse(Float64, args[2])
    fs = parse(Float64, args[3])
    dma_device = length(args) >= 4 ? args[4] : "/dev/m2sdr1"
    epoch_period = length(args) >= 5 ? parse(Int, args[5]) : 50
    run_spike(csr_csv, seconds, fs, dma_device, epoch_period)
    return Cint(0)
end

