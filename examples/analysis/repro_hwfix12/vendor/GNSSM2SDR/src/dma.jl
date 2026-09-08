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
    # flows towards the host. O_NONBLOCK because a blocking read parks a whole OS
    # thread for as long as the record stream is quiet — if that thread hosts
    # Polyester's sticky workers the tracking folds deadlock, and if it is the
    # libuv event-loop thread the raw-sample pipe (and with it the recorder)
    # freezes. The driver returns EAGAIN instead and `read_buffers!` polls.
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

"""
    read_buffers!(stream) -> AbstractVector{UInt8}

Wait until at least one DMA buffer is available and return a view of the bytes
read. Empty only when the stream was closed while waiting.

The fd is non-blocking, so waiting is a poll loop with millisecond sleeps: the
task yields between attempts and never parks its OS thread, which keeps
Polyester's sticky workers and the libuv event loop runnable no matter which
thread this task lands on. The ~1 ms poll granularity is well inside the dump
feedback budget.
"""
function read_buffers!(stream::DMAWriterStream)
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
                sleep(0.001)
                continue
            end
            stream.open || return @view buf[1:0]
            systemerror("read($(stream.device))", err)
        end
        n == 0 && (stream.open ? (sleep(0.001); continue) : return @view buf[1:0])
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
