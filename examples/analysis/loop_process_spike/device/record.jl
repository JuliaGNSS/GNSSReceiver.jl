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
# ... and of each antenna's very-early/very-late pair, in the tail words record
# format v2 left reserved. Two antennas x two extra taps is exactly the four
# words that were left, which is why a five-tap record still fits the 128-byte
# stride the DMA framing depends on and the magic does not move.
const ANT_VERY_WORD = (12, 14)

# Correlator tap layouts. The count is per *record*, not per build: a five-tap
# gateware runs GPS L1 C/A on three taps next to Galileo E1 on five in the same
# bank and the same record stream, and word 9's `num_taps` is what says which
# one a record is (gnss-m2sdr#32).
const TAPS_EPL = 3
const TAPS_VEPL = 5
const TAP_LAYOUTS = (TAPS_EPL, TAPS_VEPL)

# Word 9: num_ants [7:0] | version [15:8] | num_taps [23:16].
const NANTS_WORD = 9
const VERSION_SHIFT = 8
const NUM_TAPS_SHIFT = 16
# Word 10: code_phase_chip [31:0] | code_length [63:32].
const CODE_WORD = 10
const CODE_LENGTH_SHIFT = 32
# Word 11: code_step [31:0].
const CODE_STEP_WORD = 11

"""
    RECORD_FORMAT_VERSION

The DMA1 wire contract this package parses (word 9, bits [15:8]).

Version 1 is the GPS-L1-C/A-only layout, which reads `0` there because it left
the byte reserved. Version 2 adds `num_taps` and the three signal fields
(`code_phase_chip`, `code_length`, `code_step`) — all in words version 1 left
reserved, so [`RECORD_MAGIC`](@ref) is deliberately *unchanged* and a version-1
host keeps parsing a version-2 record correctly. The magic anchors the framing,
and a host that cannot frame the stream cannot read the version byte that would
tell it why; bump it only for a layout change that moves or resizes a field.
"""
const RECORD_FORMAT_VERSION = 2

"""
    CSR_LAYOUT_VERSION

The bank's CSR-layout revision this package addresses by name
(`gnss_version.csr`). Bumped with any change to the register set, and matched
*exactly*: v3 dropped `spacing` and added `tap_offset_*`, `replica`,
`subcarrier_load`, `dump_num_taps` and the `ive/qve/ivl/qvl` readbacks, so a v2
build has no register this driver can place a tap through, and a v4 one has
registers it does not know. Either way, addressing the wrong set by name reports
whatever the fields happen to line up with instead of failing.
"""
const CSR_LAYOUT_VERSION = 3

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

`version` is the wire contract the record was emitted under
([`RECORD_FORMAT_VERSION`](@ref)); it is carried on epoch strobes too, since it
describes the wire rather than the payload, and a host that has only ever seen
strobes is exactly a host with nothing locked. The four fields it gates —
`num_taps`, `code_phase_chip`, `code_length`, `code_step` — read `0` on a
version-1 record, which is why none of them may be believed without checking
`version` first.

  - `code_phase_chip` is the replica's *integer* chip index on the last
    integrated sample; together with the fractional `code_phase` it is the
    complete code phase ([`code_phase_chips`](@ref)). Inferring the chip as
    `code_length - 1` — which is what a version-1 host had to do — is right only
    for a 1023-chip code dumping exactly on its wrap.
  - `code_length` is the primary-code length the dump was integrated at and
    `code_step` the code NCO's per-sample increment, in `code_frac_bits`
    fixed point. The step is what lets the host propagate the phase across a
    scheduled rate change instead of assuming the nominal chip rate.
  - `num_taps` is how many taps the *channel* was configured for, not how many
    the build has: a five-tap bank runs GPS L1 C/A on three next to Galileo E1
    on five in one record stream. `very_early` / `very_late` are the tail words
    a `num_taps == 5` record adds; on a three-tap record the gateware zeroes
    them, and they must not be read as accumulators — a zero accumulator is a
    value a correlator can legitimately produce, and "this record has no such
    tap" is not.
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
    version::UInt8
    num_taps::UInt8
    code_phase_chip::UInt32
    code_length::UInt32
    code_step::UInt32
    prompt::NTuple{N,ComplexF64}
    early::NTuple{N,ComplexF64}
    late::NTuple{N,ComplexF64}
    very_early::NTuple{N,ComplexF64}
    very_late::NTuple{N,ComplexF64}
end

"""
    code_phase_chips(record, frac_bits) -> Float64

The complete code phase of a dump, in chips: the integer chip index the record
reports plus the fractional chip phase, with no assumption about where in the
code the integration ended.

Throws on a version-1 record rather than reading its reserved zero as "chip 0" —
a plausible-looking answer is the worst possible one here, and the chip index
genuinely cannot be reconstructed (`code_length - 1` holds only for a dump that
ends exactly on a code wrap, of a code whose length the record does not carry).
"""
function code_phase_chips(record::M2SDRRecord, frac_bits::Integer)
    record.version >= RECORD_FORMAT_VERSION || throw(
        ArgumentError(
            "record format version $(Int(record.version)) carries no code_phase_chip; " *
            "the integer chip index cannot be reconstructed (assuming code_length - 1 " *
            "is only valid for a dump ending exactly on a code wrap of a 1023-chip " *
            "code). Flash gateware streaming record format v$RECORD_FORMAT_VERSION.",
        ),
    )
    Int(record.code_phase_chip) + record.code_phase / (1 << frac_bits)
end

"""
    code_chip_rate(record, frac_bits, fs) -> Float64

The chip rate (Hz) the dump was integrated at, from the code NCO step the
record carries. Throws on a version-1 record, which does not carry it.
"""
function code_chip_rate(record::M2SDRRecord, frac_bits::Integer, fs::Real)
    record.version >= RECORD_FORMAT_VERSION || throw(
        ArgumentError("record format version $(Int(record.version)) carries no code_step"),
    )
    Int(record.code_step) / (1 << frac_bits) * Float64(fs)
end

is_strobe(r::M2SDRRecord) =
    r.channel == STROBE_CHANNEL || (r.flags & FLAG_EPOCH_STROBE) != 0
has_overflow(r::M2SDRRecord) = (r.flags & FLAG_OVERFLOW) != 0

_s32(x::UInt64) = (
    v = UInt32(x & 0xFFFFFFFF);
    v & 0x80000000 != 0 ? Int64(v) - (Int64(1) << 32) : Int64(v)
)

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
    w9 = _word(data, o, NANTS_WORD)     # num_ants | version | num_taps
    w10 = _word(data, o, CODE_WORD)     # code_phase_chip | code_length
    # Clamp: a record from a garbled or future build must not index past the
    # reserved blocks. Every record carries at least antenna 0's words, even a
    # strobe (which zeroes them and reports num_ants = 0).
    reported = Int(w9 & 0xFF)
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
    # The five-tap tail. Read unconditionally — the words exist in every
    # 128-byte record and a three-tap one zeroes them — and gated on `num_taps`
    # where the correlator is built, which is the one place that can tell a
    # zeroed reserved word from an accumulator that happened to come out zero.
    very_early = ntuple(Val(N)) do n
        w = _word(data, o, ANT_VERY_WORD[n])
        ComplexF64(_s32(w), _s32(w >> 32))
    end
    very_late = ntuple(Val(N)) do n
        w = _word(data, o, ANT_VERY_WORD[n] + 1)
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
        UInt8((w9 >> VERSION_SHIFT) & 0xFF),
        UInt8((w9 >> NUM_TAPS_SHIFT) & 0xFF),
        UInt32(w10 & 0xFFFFFFFF),
        UInt32((w10 >> CODE_LENGTH_SHIFT) & 0xFFFFFFFF),
        UInt32(_word(data, o, CODE_STEP_WORD) & 0xFFFFFFFF),
        prompt,
        early,
        late,
        very_early,
        very_late,
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
