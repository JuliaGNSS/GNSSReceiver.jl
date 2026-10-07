# The DMA1 record stream: starting the litepcie DMA writer and reading buffers.
#
# Opening `/dev/m2sdr1` and calling `read` is not enough. In litepcie's naming
# the *writer* is the FPGA-to-host direction, and the driver's read path is
#
#     wait_event_interruptible(chan->wait_rd,
#                              (chan->dma.writer_hw_count - chan->dma.writer_sw_count) > 0)
#
# where `writer_hw_count` only ever advances while that channel's DMA writer is
# enabled. So a plain `read` on a channel whose writer was never started does not
# return empty or error — it blocks forever, and the correlator-dump drain simply
# never produces a record. `LITEPCIE_IOCTL_DMA_WRITER` is what starts it.
#
# The same read path only moves whole DMA buffers (`while (len >= DMA_BUFFER_SIZE)`),
# so reads must be a multiple of `DMA_BUFFER_SIZE`; a smaller request copies
# nothing.

# struct litepcie_ioctl_dma_writer { uint8_t enable; int64_t hw_count; int64_t sw_count; }
# — the int64s force 8-byte alignment, so it is 8 + 8 + 8 = 24 bytes, not 17.
const DMA_WRITER_STRUCT_SIZE = 24
const LITEPCIE_IOCTL_DMA_WRITER = _iowr('S', 21, DMA_WRITER_STRUCT_SIZE)

# struct litepcie_ioctl_lock { 6 x uint8_t } — advisory, one owner per direction.
const LOCK_STRUCT_SIZE = 6
const LITEPCIE_IOCTL_LOCK = _iowr('S', 25, LOCK_STRUCT_SIZE)

# The driver's DMA buffer granularity (kernel config.h). Reads must be a whole
# multiple of this.
const DMA_BUFFER_SIZE = 8192

"""
    DMAWriterStream(device; buffers = 1)

The record stream of a litepcie DMA channel, with its DMA writer started.

Takes the channel's advisory writer lock, enables the writer, and reads in
`buffers * DMA_BUFFER_SIZE` chunks. `close` stops the writer and releases the
lock again, so the channel is reusable without reloading the driver.
"""
mutable struct DMAWriterStream
    const fd::Cint
    const device::String
    const buffer::Vector{UInt8}
    const ioctl_buffer::Vector{UInt8}
    open::Bool
end

function DMAWriterStream(device::AbstractString; buffers::Integer = 1)
    buffers >= 1 || throw(ArgumentError("buffers must be >= 1"))
    # O_RDWR: the ioctls are writes to the device even though the data only ever
    # flows towards the host. O_NONBLOCK so that a read never parks the thread
    # indefinitely: `read_buffers!` waits in `poll(2)` with a timeout instead,
    # which the kernel ends on the next completed buffer and which lets the
    # reader notice a closed stream.
    fd = ccall(:open, Cint, (Cstring, Cint), device, 2 | 0o4000 #= O_RDWR | O_NONBLOCK =#)
    fd < 0 && systemerror("open($device)", Libc.errno())
    stream = DMAWriterStream(
        fd,
        String(device),
        Vector{UInt8}(undef, Int(buffers) * DMA_BUFFER_SIZE),
        zeros(UInt8, max(DMA_WRITER_STRUCT_SIZE, LOCK_STRUCT_SIZE)),
        true,
    )
    try
        _request_writer_lock!(stream)
        _set_dma_writer!(stream, true)
    catch
        close(stream)
        rethrow()
    end
    stream
end

# Advisory lock: whoever holds it is the one driving this channel's writer. Not
# enforced by the ioctl that starts the writer, so failing to get it means
# another process is already draining this channel and the records would be split
# between the two readers.
#
# The driver does not clear the flag when its holder's fd is closed, so a
# crashed reader leaves it stuck with no living owner. Since this host runs a
# single DMA1 consumer by design, a denied request is treated as such a stale
# flag: release it and retry once, and only a second denial (a genuinely live
# concurrent reader re-taking it) is an error.
function _request_writer_lock!(stream::DMAWriterStream)
    for attempt = 1:2
        got = _try_writer_lock!(stream)
        got && return nothing
        attempt == 1 && _release_writer_lock!(stream)
    end
    error(
        "another process already holds the DMA writer lock on $(stream.device); " *
        "stop it before draining the correlator records",
    )
end

function _try_writer_lock!(stream::DMAWriterStream)
    buf = stream.ioctl_buffer
    fill!(buf, 0x00)
    GC.@preserve buf begin
        p = pointer(buf)
        unsafe_store!(Ptr{UInt8}(p + 1), 0x01)   # dma_writer_request
        rc = ccall(
            :ioctl,
            Cint,
            (Cint, Culong, Ptr{UInt8}),
            stream.fd,
            LITEPCIE_IOCTL_LOCK,
            p,
        )
        rc < 0 && systemerror("ioctl(LITEPCIE_IOCTL_LOCK)", Libc.errno())
        # dma_writer_status is byte 5; 0 means somebody else holds the lock.
        unsafe_load(Ptr{UInt8}(p + 5)) != 0x00
    end
end

function _release_writer_lock!(stream::DMAWriterStream)
    buf = stream.ioctl_buffer
    fill!(buf, 0x00)
    GC.@preserve buf begin
        p = pointer(buf)
        unsafe_store!(Ptr{UInt8}(p + 3), 0x01)   # dma_writer_release
        ccall(:ioctl, Cint, (Cint, Culong, Ptr{UInt8}), stream.fd, LITEPCIE_IOCTL_LOCK, p)
    end
    nothing
end

"""
    _set_dma_writer!(stream, enable) -> (hw_count, sw_count)

Enable or disable the channel's DMA writer, returning the driver's buffer
counters (their difference is how many filled buffers are waiting).
"""
function _set_dma_writer!(stream::DMAWriterStream, enable::Bool)
    buf = stream.ioctl_buffer
    fill!(buf, 0x00)
    GC.@preserve buf begin
        p = pointer(buf)
        unsafe_store!(Ptr{UInt8}(p), enable ? 0x01 : 0x00)
        rc = ccall(
            :ioctl,
            Cint,
            (Cint, Culong, Ptr{UInt8}),
            stream.fd,
            LITEPCIE_IOCTL_DMA_WRITER,
            p,
        )
        rc < 0 && systemerror("ioctl(LITEPCIE_IOCTL_DMA_WRITER)", Libc.errno())
        return unsafe_load(Ptr{Int64}(p + 8)), unsafe_load(Ptr{Int64}(p + 16))
    end
end

"""
    dma_writer_counts(stream) -> (hw_count, sw_count)

The driver's filled/consumed buffer counters, without changing the enable state
— `hw_count == sw_count` after a while means the gateware is not producing
records (bank disabled, or no dumps and no epoch strobe).
"""
dma_writer_counts(stream::DMAWriterStream) = _set_dma_writer!(stream, true)

# struct pollfd { int fd; short events; short revents; } — 8 bytes.
const POLLIN = Cshort(0x0001)

# Block in the kernel until the device has a completed buffer, or `timeout_ms`
# passes; return whether it is readable.
#
# This is a `gc_safe` ccall on purpose. Julia services its event loop — every
# `sleep`, `Timer` and libuv read — from thread 1 unless that thread is blocked,
# so a sleep-based poll here stopped for as long as thread 1 was busy: measured
# on the board, 550 ms per acquisition scan run on the main task, and the whole
# of any compilation the main task did, each time long enough for the driver to
# discard the ring (GNSSReceiver.jl#107). `poll(2)` is woken by the driver's
# interrupt directly, so this wait depends on nothing in the Julia runtime, and
# `gc_safe` lets a collection proceed while the thread is parked in the kernel.
function _poll_readable(fd::Cint, timeout_ms::Integer)
    pfd = Ref{NTuple{2,Cint}}((fd, Cint(POLLIN)))   # events in the low half, revents zeroed
    rc = GC.@preserve pfd @ccall gc_safe = true poll(
        Base.unsafe_convert(Ptr{Cvoid}, pfd)::Ptr{Cvoid},
        1::Culong,
        Cint(timeout_ms)::Cint,
    )::Cint
    if rc < 0
        err = Libc.errno()
        err == Libc.EINTR && return false
        systemerror("poll(fd $fd)", err)
    end
    rc > 0
end
_poll_readable(stream::DMAWriterStream, timeout_ms::Integer) =
    _poll_readable(stream.fd, timeout_ms)

# Read whatever `fd` has into `buf` after its first `filled` bytes, as a
# `gc_safe` ccall. Returns the byte count: `0` is end of file, `-1` means
# nothing was available yet (EAGAIN / EINTR).
function _read_into!(fd::Cint, buf::Vector{UInt8}, filled::Int)
    n = GC.@preserve buf @ccall gc_safe = true read(
        fd::Cint,
        (pointer(buf) + filled)::Ptr{UInt8},
        (length(buf) - filled)::Csize_t,
    )::Cssize_t
    if n < 0
        err = Libc.errno()
        (err == Libc.EAGAIN || err == Libc.EINTR) && return -1
        systemerror("read(fd $fd)", err)
    end
    Int(n)
end

"""
    _take_records!(records, buf, filled, ::Val{N}) -> filled

Parse every whole record in the first `filled` bytes of `buf` into `records`
(appending), move the unparsed tail — a record cut by a read boundary, or bytes
before the next magic — to the front of `buf`, and return how many bytes that
tail is. A pipe delivers the record stream at arbitrary cut points; the DMA
device delivers whole buffers, where the tail is normally empty.
"""
function _take_records!(records, buf::Vector{UInt8}, filled::Int, ::Val{N}) where {N}
    consumed = parse_records!(records, view(buf, 1:filled), Val(N))
    # Whatever the parser could not place a record at is kept for the next
    # read, except that a run of non-record bytes longer than a record can never
    # become one: drop all but the last `RECORD_BYTES - 1` bytes of it.
    remainder = filled - consumed
    if remainder >= RECORD_BYTES
        consumed += remainder - (RECORD_BYTES - 1)
        remainder = RECORD_BYTES - 1
    end
    remainder > 0 && consumed > 0 && copyto!(buf, 1, buf, consumed + 1, remainder)
    remainder
end

"""
    read_available!(stream) -> AbstractVector{UInt8}

Read whatever completed buffers the driver has right now, without waiting;
empty when there are none. Pair it with [`_poll_readable`](@ref) to wait in the
kernel between reads while keeping a foot in the loop (see `_service_dma!`).
"""
function read_available!(stream::DMAWriterStream)
    stream.open || return @view stream.buffer[1:0]
    buf = stream.buffer
    n = GC.@preserve buf ccall(
        :read,
        Cssize_t,
        (Cint, Ptr{UInt8}, Csize_t),
        stream.fd,
        pointer(buf),
        length(buf),
    )
    if n < 0
        err = Libc.errno()
        (err == Libc.EAGAIN || err == Libc.EINTR) && return @view buf[1:0]
        stream.open || return @view buf[1:0]
        systemerror("read($(stream.device))", err)
    end
    @view buf[1:Int(n)]
end

"""
    read_buffers!(stream; timeout_ms = 100) -> AbstractVector{UInt8}

Wait until at least one DMA buffer is available and return a view of the bytes
read. Empty only when the stream was closed while waiting.

The wait is a `poll(2)` in the kernel, ended by the driver's completion
interrupt: it does not involve Julia's scheduler or event loop, so it keeps
draining while thread 1 is busy. `timeout_ms` only bounds how long a closed
stream takes to be noticed.
"""
function read_buffers!(stream::DMAWriterStream; timeout_ms::Integer = 100)
    stream.open || throw(ArgumentError("stream is closed"))
    buf = stream.buffer
    while true
        n = GC.@preserve buf ccall(
            :read,
            Cssize_t,
            (Cint, Ptr{UInt8}, Csize_t),
            stream.fd,
            pointer(buf),
            length(buf),
        )
        if n < 0
            err = Libc.errno()
            if err == Libc.EAGAIN || err == Libc.EINTR
                stream.open || return @view buf[1:0]
                _poll_readable(stream, timeout_ms)
                continue
            end
            stream.open || return @view buf[1:0]
            systemerror("read($(stream.device))", err)
        end
        if n == 0
            stream.open || return @view buf[1:0]
            _poll_readable(stream, timeout_ms)
            continue
        end
        return @view buf[1:Int(n)]
    end
end

function Base.close(stream::DMAWriterStream)
    stream.open || return stream
    stream.open = false
    # Best effort: the fd is going away regardless, and throwing here would mask
    # whatever sent us into the cleanup path.
    try
        _set_dma_writer!(stream, false)
    catch
    end
    try
        _release_writer_lock!(stream)
    catch
    end
    ccall(:close, Cint, (Cint,), stream.fd)
    stream
end
