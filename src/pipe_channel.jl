"""
    PipeChannel{T}

A lock-free single-producer single-consumer channel using a ring buffer.

This implementation uses atomic operations for the head and tail indices,
allowing one thread to write and another to read without any locks.
This eliminates allocations in the hot path that would otherwise occur
with Julia's `Channel` type.

The API matches Julia's `Channel`:
- `put!` blocks when full, throws `InvalidStateException` when closed
- `take!` blocks when empty, throws `InvalidStateException` when closed and empty
- Iteration works with `for x in ch` syntax
- `bind` connects a task to the channel for error propagation

# Type Parameters
- `T`: Element type stored in the channel

# Thread Safety
- Exactly ONE producer thread may call `put!`
- Exactly ONE consumer thread may call `take!`
- Multiple producers or consumers will cause data races
"""
mutable struct PipeChannel{T} <: AbstractChannel{T}
    buffer::Vector{T}
    capacity::Int
    head::Threads.Atomic{Int}  # Write position (producer only)
    tail::Threads.Atomic{Int}  # Read position (consumer only)
    closed::Threads.Atomic{Bool}
    excp::Union{Exception,Nothing}  # Exception from bound task

    function PipeChannel{T}(capacity::Integer) where {T}
        capacity > 0 || throw(ArgumentError("Capacity must be positive"))
        # Allocate one extra slot to distinguish full from empty
        buffer = Vector{T}(undef, capacity + 1)
        return new{T}(
            buffer,
            capacity + 1,
            Threads.Atomic{Int}(1),
            Threads.Atomic{Int}(1),
            Threads.Atomic{Bool}(false),
            nothing,
        )
    end
end

Base.isopen(ch::PipeChannel) = !ch.closed[]

function Base.close(ch::PipeChannel, excp::Exception = Base.closed_exception())
    ch.excp = excp
    ch.closed[] = true
    return nothing
end

# Helper to throw the appropriate exception
function check_closed_and_throw(ch::PipeChannel)
    if ch.excp !== nothing && !isa(ch.excp, InvalidStateException)
        throw(ch.excp)
    end
    throw(InvalidStateException("PipeChannel is closed.", :closed))
end

"""
    bind(ch::PipeChannel, task::Task)

Bind a task to the channel. When the task terminates the channel is
automatically closed. If the task failed with an exception, that exception is
thrown on subsequent `put!` or `take!` operations, propagating producer/consumer
errors.
"""
function Base.bind(ch::PipeChannel, task::Task)
    # Register a callback that runs when the task completes
    @async begin
        try
            wait(task)
        catch
            # Task failed - will be handled below
        end
        # Close the buffer when task completes
        if istaskfailed(task)
            close(ch, TaskFailedException(task))
        elseif !ch.closed[]
            close(ch)
        end
    end
    return nothing
end

"""
    isfull(ch::PipeChannel) -> Bool

Check if the buffer is full. Only guaranteed accurate from the producer thread;
from the consumer thread a `true` result may be stale (safe, just an unnecessary
wait), while a `false` result from the producer guarantees space to write.

Defined as a module-local function rather than extending `Base.isfull`, which
does not exist on Julia 1.10.
"""
function isfull(ch::PipeChannel)
    head = ch.head[]
    tail = ch.tail[]
    next_head = head == ch.capacity ? 1 : head + 1
    return next_head == tail
end

"""
    isempty(ch::PipeChannel) -> Bool

Check if the buffer is empty. Only guaranteed accurate from the consumer thread;
a `false` result from the consumer guarantees data is available.
"""
Base.isempty(ch::PipeChannel) = ch.head[] == ch.tail[]

"""
    isready(ch::PipeChannel) -> Bool

Return `true` when `take!` would not block (i.e. the buffer is not empty).
Matches the `Channel` API. Same thread-safety caveats as `isempty`.
"""
Base.isready(ch::PipeChannel) = !isempty(ch)

"""
    wait(ch::PipeChannel)

Block until data is available in the buffer or the channel is closed. Does not
consume the data. Throws if the channel is closed and empty. Consumer thread only.
"""
function Base.wait(ch::PipeChannel)
    while true
        # Check if data is available
        if ch.head[] != ch.tail[]
            return nothing
        end
        # Check if closed and empty
        if ch.closed[]
            check_closed_and_throw(ch)
        end
        # Spin-wait
        yield()
    end
end

"""
    n_avail(ch::PipeChannel) -> Int

Return the number of elements available to read. An approximation that may be
slightly stale; most accurate from the consumer thread.
"""
function Base.n_avail(ch::PipeChannel)
    head = ch.head[]
    tail = ch.tail[]
    if head >= tail
        return head - tail
    else
        return ch.capacity - tail + head
    end
end

"""
    put!(ch::PipeChannel{T}, value::T)

Add an element to the buffer. Blocks if the buffer is full. Throws
`InvalidStateException` (or the bound task's exception) if the channel is closed.
Single producer thread only.
"""
function Base.put!(ch::PipeChannel{T}, value::T) where {T}
    while true
        if ch.closed[]
            check_closed_and_throw(ch)
        end

        head = ch.head[]
        next_head = head == ch.capacity ? 1 : head + 1

        # Check if buffer is full - spin-wait
        if next_head == ch.tail[]
            yield()
            continue
        end

        # Write the value
        @inbounds ch.buffer[head] = value

        # Publish the write by advancing head
        ch.head[] = next_head

        return value
    end
end

"""
    take!(ch::PipeChannel{T}) -> T

Remove and return an element from the buffer. Blocks if the buffer is empty.
Throws `InvalidStateException` (or the bound task's exception) if the channel is
closed and empty. Single consumer thread only.
"""
function Base.take!(ch::PipeChannel{T}) where {T}
    while true
        tail = ch.tail[]
        head = ch.head[]

        # Check if buffer is empty
        if tail == head
            if ch.closed[]
                check_closed_and_throw(ch)
            end
            # Spin-wait
            yield()
            continue
        end

        # Read the value
        @inbounds value = ch.buffer[tail]

        # Advance tail
        next_tail = tail == ch.capacity ? 1 : tail + 1
        ch.tail[] = next_tail

        return value
    end
end

# The @inline annotation is critical for avoiding heap allocation of the
# returned `(value, nothing)` tuple when `T` is not an isbits type.
@inline function Base.iterate(ch::PipeChannel{T}, state = nothing) where {T}
    try
        value = take!(ch)
        return (value, nothing)
    catch e
        if e isa InvalidStateException
            return nothing
        end
        rethrow()
    end
end

Base.IteratorSize(::Type{<:PipeChannel}) = Base.SizeUnknown()
Base.eltype(::Type{PipeChannel{T}}) where {T} = T

# ============================================================================
# Batch Operations
# ============================================================================

"""
    put!(ch::PipeChannel{T}, values::AbstractVector{T}) -> AbstractVector{T}

Add multiple elements to the buffer in a single batch operation, writing as many
as fit before blocking for space. More efficient than repeated `put!` (fewer
atomic writes). Single producer thread only.
"""
function Base.put!(ch::PipeChannel{T}, values::AbstractVector{T}) where {T}
    isempty(values) && return values

    offset = 0
    total = length(values)

    while offset < total
        if ch.closed[]
            check_closed_and_throw(ch)
        end

        head = ch.head[]
        tail = ch.tail[]

        # Calculate available space
        if head >= tail
            # Available space is split: from head to capacity, and from 1 to tail-1
            space_to_end = ch.capacity - head + 1
            space_at_start = tail - 1
            total_space = space_to_end + space_at_start - 1  # -1 because we can't fill completely
        else
            # Available space is contiguous: from head to tail-1
            total_space = tail - head - 1
        end

        # No space available - spin-wait
        if total_space <= 0
            yield()
            continue
        end

        # Write as many items as we can
        n_to_write = min(total - offset, total_space)

        pos = head
        @inbounds for i = 1:n_to_write
            ch.buffer[pos] = values[offset+i]
            pos = pos == ch.capacity ? 1 : pos + 1
        end

        # Single atomic update to publish all writes
        ch.head[] = pos
        offset += n_to_write
    end

    return values
end

"""
    take!(ch::PipeChannel{T}, n::Integer) -> Vector{T}

Remove and return exactly `n` elements from the buffer in a single batch
operation, blocking until all `n` are available. Single consumer thread only.
"""
function Base.take!(ch::PipeChannel{T}, n::Integer) where {T}
    n <= 0 && return T[]
    result = Vector{T}(undef, n)
    take!(ch, result)
    return result
end

"""
    take!(ch::PipeChannel{T}, output::AbstractVector{T}) -> Int

Remove elements from the buffer into a pre-allocated `output` vector, blocking
until it is filled. Returns `length(output)`. Single consumer thread only.
"""
function Base.take!(ch::PipeChannel{T}, output::AbstractVector{T}) where {T}
    isempty(output) && return 0

    total = length(output)
    offset = 0

    while offset < total
        tail = ch.tail[]
        head = ch.head[]

        # Check if buffer is empty
        if tail == head
            if ch.closed[]
                check_closed_and_throw(ch)
            end
            # Spin-wait for data
            yield()
            continue
        end

        # Calculate available items
        if head >= tail
            available = head - tail
        else
            available = ch.capacity - tail + head
        end

        # Read as many items as we can
        n_to_read = min(total - offset, available)

        pos = tail
        @inbounds for i = 1:n_to_read
            output[offset+i] = ch.buffer[pos]
            pos = pos == ch.capacity ? 1 : pos + 1
        end

        # Single atomic update to release all read slots
        ch.tail[] = pos
        offset += n_to_read
    end

    return total
end
