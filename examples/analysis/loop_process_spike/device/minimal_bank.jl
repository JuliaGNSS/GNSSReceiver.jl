# The few bank operations the spike needs, lifted from GNSSM2SDR's bank.jl
# without its GNSSSignals/Unitful dependencies: the sample counter, enable, the
# epoch strobe period, and the immediate carrier/code words of one channel.
const SPIKE_CODE_FRAC_BITS = 24

function sample_count(csr::LiteXCSR; tries::Integer = 3)
    local value
    for _ = 1:tries
        first_read = read(csr, "gnss_sample_count")
        second_read = read(csr, "gnss_sample_count")
        value = second_read
        (second_read >> 32) == (first_read >> 32) && return Int64(second_read)
    end
    Int64(value)
end

enable!(csr::LiteXCSR, on::Bool) = (write(csr, "gnss_control", on ? 1 : 0); nothing)
set_epoch_period!(csr::LiteXCSR, samples::Integer) =
    (write(csr, "gnss_epoch_period", samples); nothing)

carrier_word(hz, fs, bits::Integer = 32) =
    round(Int64, hz / fs * (Int64(1) << bits)) & ((Int64(1) << bits) - 1)

function code_step_word(chip_rate, fs, frac_bits::Integer = SPIKE_CODE_FRAC_BITS)
    word = round(Int64, chip_rate / fs * (Int64(1) << frac_bits))
    0 < word < (Int64(1) << frac_bits) || return Int64(-1)
    word
end

function write_channel_words!(csr::LiteXCSR, channel::Integer, carrier_hz, code_rate_hz, fs)
    prefix = "gnss_ch$(channel)_"
    write(csr, prefix * "carrier_freq", carrier_word(carrier_hz, fs))
    step = code_step_word(code_rate_hz, fs)
    step < 0 && return false
    write(csr, prefix * "code_freq", step)
    true
end
