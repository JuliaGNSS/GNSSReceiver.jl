# Control of the gnss-m2sdr tracking bank over CSRs.
#
# A port of gnss-m2sdr's `software/gnss_tracking.py` (GNSSChannel / GNSSBank).
# Every fixed-point convention here has to match the gateware exactly, so the
# reasoning behind each word is kept with it.

const GPS_L1_HZ = 1_575_420_000.0
const GPS_CA_CHIP_RATE = 1_023_000.0
const CA_CODE_LENGTH = 1023
# Fractional bits of the code NCO (and thus of `dump_code_phase`); must match
# the gateware's `code_frac_bits`.
const CODE_FRAC_BITS = 24

"""
    GNSSBankChannel(csr, index; fs, carrier_phase_bits = 32, code_frac_bits = 24)

One hardware tracking channel. `index` is 0-based, matching the gateware's
`gnss_ch<N>_` CSR prefix.
"""
struct GNSSBankChannel
    csr::LiteXCSR
    fs::Float64
    index::Int
    carrier_phase_bits::Int
    code_frac_bits::Int
    prefix::String
end

# Accept a plain number (Hz) as well as a Unitful frequency.
_hz(f::Real) = Float64(f)
_hz(f) = Float64(ustrip(uconvert(Hz, f)))

GNSSBankChannel(
    csr::LiteXCSR,
    index::Integer;
    fs,
    carrier_phase_bits::Integer = 32,
    code_frac_bits::Integer = 24,
) = GNSSBankChannel(
    csr,
    _hz(fs),
    Int(index),
    Int(carrier_phase_bits),
    Int(code_frac_bits),
    "gnss_ch$(index)_",
)

# ── Fixed-point conversions ──────────────────────────────────────────────────
#
# Kept as plain functions of (value, fs, width) rather than methods on the
# channel: they are the part that has to agree with the gateware bit for bit,
# and this way they can be tested without a device to open.

# Carrier NCO phase increment for `hz`, as a `bits`-wide word.
carrier_word(hz, fs, bits::Integer = 32) =
    round(Int64, hz / fs * (Int64(1) << bits)) & ((Int64(1) << bits) - 1)

# Code NCO step. The chip rate scales with carrier Doppler: fc = R_c(1 + fd/L1).
# NOTE: the input is the *carrier* Doppler (Hz at L1), not the code Doppler —
# use [`code_word_from_code_doppler`](@ref) when you have the latter.
function code_word(doppler_hz, fs, frac_bits::Integer = 24)
    fc = GPS_CA_CHIP_RATE * (1.0 + doppler_hz / GPS_L1_HZ)
    round(Int64, fc / fs * (Int64(1) << frac_bits)) & ((Int64(1) << frac_bits) - 1)
end

# Code NCO step from the *code* Doppler (Hz of chip rate): fc = R_c + fd_code.
# This is the unit `Tracking` reports and `NCOUpdate.code_doppler` carries —
# 1/1540 of the carrier Doppler for GPS L1 C/A. Feeding that value into
# `code_word` (which expects the carrier Doppler) silently programs a ~zero
# code-rate offset: a 1540× loop-gain error on the code NCO.
function code_word_from_code_doppler(code_doppler_hz, fs, frac_bits::Integer = 24)
    fc = GPS_CA_CHIP_RATE + code_doppler_hz
    round(Int64, fc / fs * (Int64(1) << frac_bits)) & ((Int64(1) << frac_bits) - 1)
end

carrier_phase_word(cycles, bits::Integer = 32) =
    round(Int64, mod(cycles, 1.0) * (Int64(1) << bits)) & ((Int64(1) << bits) - 1)

# Code phase (fractional chips) → the chip|frac word the gateware wants.
function code_phase_word(chips, frac_bits::Integer = 24)
    phase = mod(chips, CA_CODE_LENGTH)
    chip = floor(Int, phase)
    frac = round(Int64, (phase - chip) * (Int64(1) << frac_bits))
    if frac == (Int64(1) << frac_bits)   # rounding carried a whole chip
        chip = mod(chip + 1, CA_CODE_LENGTH)
        frac = 0
    end
    (Int64(chip) << frac_bits) | frac
end

"""
    spacing_word(sample_shift, code_doppler_hz, fs, frac_bits)

The E/L half-spacing CSR word for `sample_shift` whole NCO samples.

`sample_shift * code_step` places the Early tap exactly that many samples ahead
of the prompt (and Late the same behind) with no rounding drift between the two
fixed-point words. Programming the raw preferred chip shift instead would leave
the accumulators at a spacing `dll_disc` does not assume — a ~2.3 % DLL
loop-gain error at fs = 4 MHz and 0.5 chips. The taps only reach chip index ±1,
so the word must stay below one chip.
"""
function spacing_word(sample_shift::Integer, code_doppler_hz, fs, frac_bits::Integer = 24)
    step = code_word(code_doppler_hz, fs, frac_bits)
    word = Int64(sample_shift) * step
    if word >= (Int64(1) << frac_bits)
        throw(
            ArgumentError(
                "E/L half-spacing $(word / (1 << frac_bits)) chips ≥ 1 chip: the E/L " *
                "taps only reach chip index ±1 (sample_shift=$sample_shift at fs=$fs Hz)",
            ),
        )
    end
    word
end

# Channel-flavoured forwarders.
carrier_word(ch::GNSSBankChannel, hz) = carrier_word(hz, ch.fs, ch.carrier_phase_bits)
code_word(ch::GNSSBankChannel, doppler_hz = 0.0) =
    code_word(doppler_hz, ch.fs, ch.code_frac_bits)
code_word_from_code_doppler(ch::GNSSBankChannel, code_doppler_hz = 0.0) =
    code_word_from_code_doppler(code_doppler_hz, ch.fs, ch.code_frac_bits)
carrier_phase_word(ch::GNSSBankChannel, cycles) =
    carrier_phase_word(cycles, ch.carrier_phase_bits)
code_phase_word(ch::GNSSBankChannel, chips) = code_phase_word(chips, ch.code_frac_bits)
spacing_word(ch::GNSSBankChannel, sample_shift::Integer, code_doppler_hz = 0.0) =
    spacing_word(sample_shift, code_doppler_hz, ch.fs, ch.code_frac_bits)

"""
    load_code!(ch, prn)

Write PRN `prn`'s 1023 chips into the channel's code RAM and set its PRN field.
1023 CSR writes, so this is a configuration-time operation, not a hot path.
"""
function load_code!(ch::GNSSBankChannel, prn::Integer, code::AbstractVector)
    write(ch.csr, ch.prefix * "code_load", 0b100)              # reset address
    for chip in code
        write(ch.csr, ch.prefix * "code_load", 0b010 | (Int(chip) & 1))  # we | data
    end
    write(ch.csr, ch.prefix * "prn", prn)
    ch
end

"""
    restart!(ch)

Pulse the channel's restart + carrier-set strobe (edge-triggered 0 → 1 → 0).
"""
function restart!(ch::GNSSBankChannel)
    write(ch.csr, ch.prefix * "control", 0)
    write(ch.csr, ch.prefix * "control", 0b11)
    write(ch.csr, ch.prefix * "control", 0)
    ch
end

"""
    schedule!(ch, sample_index; carrier_hz, code_doppler_hz, carrier_phase_cycles,
              code_phase_chips)

Commit the supplied values atomically on global sample `sample_index`.

`sample_index` is on the bank's free-running counter — the same axis records are
timestamped on — and names the first input sample processed with the new values.
This is the hardware meaning of `GNSSReceiver.NCOUpdate.apply_at_sample`, and it
is what buys a fixed feedback delay instead of PCIe jitter. Only the values
passed are committed; supplying `code_phase_chips` (an acquisition handover)
also restarts the integration on that sample.

Check [`apply_status`](@ref) afterwards: `late` means the CSR writes did not
reach the board in time and the commit slipped to a later sample.

!!! warning "One outstanding commit per channel"
    The gateware has a single staging slot and a single `armed` bit, so this is
    **not** a queue: calling `schedule!` again while `apply_status(ch).armed` is
    still set does not enqueue a second commit, it *replaces* the pending one —
    the values from the first call are then never applied at the sample it
    picked. Wait for `armed` to clear (as [`assign_channel!`](@ref) does) before
    scheduling the next commit on the same channel.

    That makes the scheduled path unsuitable for *streaming* NCO corrections: at
    a 1 kHz loop rate the next correction arrives ~1 ms after the last, while
    `sample_index` is one or two epochs ahead, so each commit would be cancelled
    ~1 ms before it was due and the channel would keep free-running on its
    handover words while the host believed it was steering it. Rate-only updates
    therefore go through the immediate `carrier_freq` / `code_freq` CSRs
    (`_drain_ncos!`); `schedule!` is for sample-exact handovers.
"""
function schedule!(
    ch::GNSSBankChannel,
    sample_index::Integer;
    carrier_hz = nothing,
    code_doppler_hz = nothing,
    carrier_phase_cycles = nothing,
    code_phase_chips = nothing,
)
    flags = 0b1                                       # arm
    if carrier_hz !== nothing
        write(ch.csr, ch.prefix * "carrier_freq_next", carrier_word(ch, carrier_hz))
        flags |= 1 << 3
    end
    if code_doppler_hz !== nothing
        # `code_doppler_hz` is the *code* Doppler (chip-rate offset in Hz), the
        # unit Tracking and `NCOUpdate` carry — not the carrier Doppler.
        write(
            ch.csr,
            ch.prefix * "code_freq_next",
            code_word_from_code_doppler(ch, code_doppler_hz),
        )
        flags |= 1 << 4
    end
    if carrier_phase_cycles !== nothing
        write(
            ch.csr,
            ch.prefix * "carrier_phase",
            carrier_phase_word(ch, carrier_phase_cycles),
        )
        flags |= 1 << 2
    end
    if code_phase_chips !== nothing
        write(ch.csr, ch.prefix * "code_phase", code_phase_word(ch, code_phase_chips))
        flags |= 1 << 1
    end
    write(ch.csr, ch.prefix * "apply_at", sample_index)
    write(ch.csr, ch.prefix * "apply", 0)             # arm is 0 → 1 edge-triggered
    write(ch.csr, ch.prefix * "apply", flags)
    ch
end

"""
    apply_status(ch) -> (armed, late)
"""
function apply_status(ch::GNSSBankChannel)
    s = read(ch.csr, ch.prefix * "apply_status")
    (armed = (s & 0b01) != 0, late = (s & 0b10) != 0)
end

applied_at(ch::GNSSBankChannel) = read(ch.csr, ch.prefix * "applied_at")

"""
    detect_num_channels(csr) -> Int

How many `gnss_ch<i>_` tracking channels the flashed gateware exposes, counted
off the CSR map (channels are numbered consecutively from 0). This is the
authoritative channel count — passing a larger `n_channels` to [`GNSSBank`](@ref)
would build channels whose CSR reads throw `KeyError`.
"""
detect_num_channels(csr::LiteXCSR) = detect_num_channels(csr.regs)

# Core on the register map itself, so it can be tested without a device.
function detect_num_channels(regs::AbstractDict)
    n = 0
    while haskey(regs, "gnss_ch$(n)_control")
        n += 1
    end
    n
end

"""
    GNSSBank(csr; fs, n_channels = detect_num_channels(csr))

The tracking bank: the channels plus the bank-wide controls (enable, epoch
strobe period, overflow status, the free-running sample counter).
"""
struct GNSSBank
    csr::LiteXCSR
    channels::Vector{GNSSBankChannel}
end

function GNSSBank(
    csr::LiteXCSR;
    fs,
    n_channels::Integer = detect_num_channels(csr),
    code_frac_bits::Integer = 24,
)
    GNSSBank(
        csr,
        [
            GNSSBankChannel(csr, i - 1; fs, code_frac_bits) for i = 1:n_channels
        ],
    )
end

enable!(bank::GNSSBank, on::Bool = true) = (write(bank.csr, "gnss_control", on ? 1 : 0); bank)

"""
    set_epoch_period!(bank, samples)

Emit a timebase-marker record every `samples` input samples. Without it the
host's epoch clock stalls whenever no channel is dumping.
"""
set_epoch_period!(bank::GNSSBank, samples::Integer) =
    (write(bank.csr, "gnss_epoch_period", samples); bank)

num_ants(bank::GNSSBank) = Int(read(bank.csr, "gnss_num_ants"))

"""
    overflow(bank) -> UInt64

The sticky per-channel overflow bitmap. Clear it with [`clear_overflow!`](@ref);
it is write-1-to-clear in the gateware, so reading alone does not reset it.
"""
overflow(bank::GNSSBank) = read(bank.csr, "gnss_overflow")
clear_overflow!(bank::GNSSBank, mask::Integer = 0xFFFFFFFF) =
    (write(bank.csr, "gnss_overflow_clear", mask); bank)

"""
    sample_count(bank; tries = 3) -> Int64

The bank's free-running sample counter. Read twice and retried because the
64-bit value is assembled from two 32-bit CSR reads that the counter can tick
between; a pair whose high word is stable is consistent.
"""
function sample_count(bank::GNSSBank; tries::Integer = 3)
    local value
    for _ = 1:tries
        first_read = read(bank.csr, "gnss_sample_count")
        second_read = read(bank.csr, "gnss_sample_count")
        value = second_read
        (second_read >> 32) == (first_read >> 32) && return Int64(second_read)
    end
    Int64(value)
end
