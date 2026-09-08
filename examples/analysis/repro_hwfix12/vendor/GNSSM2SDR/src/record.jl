# The DMA1 correlator-dump record: wire format and parsing.
#
# Mirrors gnss-m2sdr's `gnss_m2sdr/record_format.py`. The record is 128 bytes,
# not the 80 the two-antenna payload needs, because 8192 / 128 = 64 exactly:
# every DMA buffer then starts on a record boundary, so a dropped buffer costs
# whole records instead of shifting every subsequent one by 32 bytes forever.

const RECORD_WORDS = 16
const RECORD_BYTES = RECORD_WORDS * 8

# "GNSS" as it reads in a little-endian hexdump.
const RECORD_MAGIC = 0x53534E47
const MAGIC_WORD = 5
const MAGIC_SHIFT = 32
const MAGIC_OFFSET = MAGIC_WORD * 8 + MAGIC_SHIFT ÷ 8   # byte offset within a record

const N_ANTS_MAX = 2
const ANT_PROMPT_WORD = (2, 6)   # 0-based word index of each antenna's E/P/L block
const NANTS_WORD = 9

const FLAG_OVERFLOW = 0x01
const FLAG_EPOCH_STROBE = 0x02

# Reserved channel id for the timebase marker; the round-robin serializer only
# ever reaches n_channels, so 0xFF cannot collide with a real channel.
const STROBE_CHANNEL = 0xFF

"""
    M2SDRRecord

One decoded 128-byte record. Accumulators are per antenna, in the gateware's
`(prompt, early, late)` word order — the reordering into Tracking's
`[late, prompt, early]` happens where the `CorrelatorDump` is built.
"""
struct M2SDRRecord{N}
    sample_index::Int64
    integrated_samples::Int32
    channel::UInt8
    prn::UInt8
    flags::UInt8
    seq::UInt8
    code_phase::UInt32
    num_ants::Int
    prompt::NTuple{N,ComplexF64}
    early::NTuple{N,ComplexF64}
    late::NTuple{N,ComplexF64}
end

is_strobe(r::M2SDRRecord) = r.channel == STROBE_CHANNEL || (r.flags & FLAG_EPOCH_STROBE) != 0
has_overflow(r::M2SDRRecord) = (r.flags & FLAG_OVERFLOW) != 0

_s32(x::UInt64) = (v = UInt32(x & 0xFFFFFFFF); v & 0x80000000 != 0 ? Int64(v) - (Int64(1) << 32) : Int64(v))

@inline _word(data::AbstractVector{UInt8}, offset::Int, i::Int) =
    GC.@preserve data unsafe_load(Ptr{UInt64}(pointer(data, offset + i * 8 + 1)))

"""
    is_record_start(data, offset) -> Bool

Whether a record begins at `offset` (0-based) — i.e. the magic sits where it
should. The stream endpoint's `first`/`last` are set by the recorder but
litepcie's DMA writer ignores them, so the magic is the only in-band anchor.
"""
function is_record_start(data::AbstractVector{UInt8}, offset::Integer)
    offset + RECORD_BYTES > length(data) && return false
    GC.@preserve data begin
        unsafe_load(Ptr{UInt32}(pointer(data, offset + MAGIC_OFFSET + 1))) == RECORD_MAGIC
    end
end

"""
    find_record_offset(data) -> Union{Int,Nothing}

The offset (0-based) of the first whole record in `data`, or `nothing` if none
is visible. Lets the host attach mid-stream, or resynchronise after a torn or
dropped DMA buffer, instead of misparsing everything that follows.
"""
function find_record_offset(data::AbstractVector{UInt8})
    for offset = 0:(length(data)-RECORD_BYTES)
        is_record_start(data, offset) && return offset
    end
    nothing
end

"""
    parse_record(data, offset, ::Val{N}) -> M2SDRRecord{N}

Decode the record starting at `offset` (0-based), reading `N` antenna blocks.
"""
function parse_record(data::AbstractVector{UInt8}, offset::Integer, ::Val{N}) where {N}
    o = Int(offset)
    w0 = _word(data, o, 0)
    w1 = _word(data, o, 1)
    w5 = _word(data, o, MAGIC_WORD)
    # Clamp: a record from a garbled or future build must not index past the
    # reserved blocks. Every record carries at least antenna 0's words, even a
    # strobe (which zeroes them and reports num_ants = 0).
    reported = Int(_word(data, o, NANTS_WORD) & 0xFF)
    num_ants = clamp(reported, 0, N_ANTS_MAX)

    prompt = ntuple(Val(N)) do n
        base = ANT_PROMPT_WORD[n]
        w = _word(data, o, base)
        ComplexF64(_s32(w), _s32(w >> 32))
    end
    early = ntuple(Val(N)) do n
        w = _word(data, o, ANT_PROMPT_WORD[n] + 1)
        ComplexF64(_s32(w), _s32(w >> 32))
    end
    late = ntuple(Val(N)) do n
        w = _word(data, o, ANT_PROMPT_WORD[n] + 2)
        ComplexF64(_s32(w), _s32(w >> 32))
    end

    M2SDRRecord{N}(
        Int64(w0),
        Int32((w1 >> 32) & 0xFFFFFFFF),
        UInt8((w1 >> 24) & 0xFF),
        UInt8((w1 >> 16) & 0xFF),
        UInt8((w1 >> 8) & 0xFF),
        UInt8(w1 & 0xFF),
        UInt32(w5 & 0xFFFFFFFF),
        num_ants,
        prompt,
        early,
        late,
    )
end

"""
    parse_records!(sink, data, ::Val{N}) -> Int

Decode every whole record in `data`, pushing each into `sink`, and return the
number of bytes consumed. Resynchronises on the magic rather than misparsing
when the stream is attached mid-record or a buffer was dropped.
"""
function parse_records!(sink, data::AbstractVector{UInt8}, ::Val{N}) where {N}
    offset = 0
    consumed = 0
    while offset + RECORD_BYTES <= length(data)
        if is_record_start(data, offset)
            push!(sink, parse_record(data, offset, Val(N)))
            offset += RECORD_BYTES
            consumed = offset
        else
            # No anchor here: this is a mid-record attach or a torn/dropped
            # buffer. Step one byte and keep hunting rather than trusting the
            # stride through bytes that are not a record.
            offset += 1
        end
    end
    consumed
end
