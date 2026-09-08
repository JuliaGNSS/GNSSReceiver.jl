# Closed-loop tracking of SEVERAL live satellites at once, one FPGA correlator
# channel each, with Tracking.jl running every loop filter.
#
# The multi-channel counterpart of closed_loop.jl (see there for the history of
# every hard-won detail: handover verification, rate-only feedback, the armed
# guard, DMA0 drain hygiene). What is new here:
#
#   * the channel count comes from the CSR map, not from a constant — this is
#    the validation harness for the >2-channel gateware images;
#   * one TrackState holds all satellites; each service cycle appends whatever
#     dumps arrived and runs the estimator once for everyone;
#   * satellite k's code-phase sweep is interleaved with servicing the already
#     locked channels: a sweep trial occupies only its own channel for ~15 ms,
#     and the satellites locked before it keep their feedback running between
#     trials, so they do not drift while later satellites are still searching.
#
# Usage: julia --project=. closed_loop_multi.jl CSR_CSV [MAX_SATS] [SECONDS]

using Printf
using StaticArrays: SVector
using Statistics: mean, median
using Unitful: Hz, ustrip

using Acquisition: acquire
using GNSSSignals: GPSL1CA, get_code
using Tracking:
    TrackState,
    TrackedSat,
    ConventionalAssistedPLLAndDLL,
    CorrelatorOutput,
    EarlyPromptLateCorrelator,
    append_correlator_output!,
    estimate_dopplers_and_filter_prompt!,
    get_sat_state,
    get_carrier_doppler,
    get_code_doppler,
    reset_start_sample_and_bit_buffer!

# On the Orin this runs with csr.jl/bank.jl pulled into a standalone M2Bank
# module (see closed_loop.jl); via the package it is just:
using GNSSM2SDR:
    LiteXCSR, GNSSBank, GNSSBankChannel, read_signed, detect_num_channels,
    carrier_word, code_word, load_code!, schedule!, apply_status, applied_at,
    sample_count, overflow, spacing_word,
    GPS_L1_HZ, GPS_CA_CHIP_RATE, CA_CODE_LENGTH

const FS_NOMINAL = 4e6Hz
const FS_NOMINAL_HZ = 4e6
const FRAC = 24
const MARGIN = 24_000
const SKIP = 2
const PREFERRED_CHIP_SHIFT = 0.5
const EPOCH_SAMPLES = 4000
const FEEDBACK_EPOCHS = 2
const DOPPLER_COVERAGE = 30_000.0Hz
const CN0_FLOOR_DBHZ = 38.0

csv = ARGS[1]
max_sats = length(ARGS) > 1 ? parse(Int, ARGS[2]) : 4
seconds = length(ARGS) > 2 ? parse(Float64, ARGS[3]) : 30.0

gpsl1 = GPSL1CA()
csr = LiteXCSR(csv)
bank = GNSSBank(csr; fs = FS_NOMINAL)   # channel count from the CSR map
@printf("gateware exposes %d tracking channels\n", length(bank.channels))

# ---- capture + acquisition (identical to closed_loop.jl) ---------------------
const CAPTURE_FILE = "/tmp/hwloop_capture.bin"
need = 800_000 * 8
run(ignorestatus(`pkill -x m2sdr_record`))
sleep(0.2)
recorder = run(pipeline(`m2sdr_record $CAPTURE_FILE $(need * 2)`,
                        stdout = devnull, stderr = devnull), wait = false)
while !(isfile(CAPTURE_FILE) && filesize(CAPTURE_FILE) >= need) && process_running(recorder)
    sleep(0.02)
end
raw = reinterpret(Int16, read(CAPTURE_FILE)[1:min(need, filesize(CAPTURE_FILE))])
process_running(recorder) && kill(recorder)
drain = run(pipeline(`m2sdr_record /dev/null $(Int(FS_NOMINAL_HZ * 600) * 8)`,
                     stdout = devnull, stderr = devnull), wait = false)
atexit(() -> process_running(drain) && kill(drain))
rm(CAPTURE_FILE; force = true)
nsmp = length(raw) ÷ 4
words = reshape(view(raw, 1:4nsmp), 4, nsmp)
signal = ComplexF32.(Float32.(view(words, 1, :)), Float32.(view(words, 2, :)))
results = acquire(gpsl1, signal, FS_NOMINAL, collect(1:32);
                  min_doppler_coverage = DOPPLER_COVERAGE,
                  num_coherently_integrated_code_periods = 10,
                  num_noncoherent_accumulations = 10)
strong = sort(filter(r -> r.CN0 > CN0_FLOOR_DBHZ, results); by = r -> -r.CN0)
n_track = min(length(strong), max_sats, length(bank.channels))
@printf("acquired %d satellite(s) above %.0f dBHz; tracking the top %d:\n",
        length(strong), CN0_FLOOR_DBHZ, n_track)
for r in strong[1:n_track]
    @printf("  PRN %2d  CN0 %.1f dBHz  doppler %+7.0f Hz\n",
            r.prn, r.CN0, Float64(ustrip(Hz, r.carrier_doppler)))
end
if n_track == 0
    println("nothing to track"); exit(0)
end

# ---- per-channel setup --------------------------------------------------------
mutable struct Chan
    ch::GNSSBankChannel
    regs::Any
    prn::Int
    doppler0::Float64
    r::Float64                 # chips per sample at doppler0
    S0::Int64                  # sweep frame anchor (set at sweep time)
    phi_peak::Float64
    floor_power::Float64
    peak_power::Float64
    locked::Bool
    prev_count::Int
    carrier_hz::Float64
    code_dopp_hz::Float64
    powers::Vector{Float64}
    commits::Int
    skipped::Int
    last_si::Int
end

struct DumpRegs
    count::String
    ie::String
    qe::String
    ip::String
    qp::String
    il::String
    ql::String
    n::String
    si::String
end
DumpRegs(ch) = DumpRegs(
    ch.prefix * "dump_count",
    ch.prefix * "ie", ch.prefix * "qe",
    ch.prefix * "ip", ch.prefix * "qp",
    ch.prefix * "il", ch.prefix * "ql",
    ch.prefix * "integrated_samples",
    ch.prefix * "sample_index",
)

function coherent_dump(ch, regs::DumpRegs)
    for _ = 1:10
        c0 = read(ch.csr, regs.count)
        ie = read_signed(ch.csr, regs.ie, 32)
        qe = read_signed(ch.csr, regs.qe, 32)
        ip = read_signed(ch.csr, regs.ip, 32)
        qp = read_signed(ch.csr, regs.qp, 32)
        il = read_signed(ch.csr, regs.il, 32)
        ql = read_signed(ch.csr, regs.ql, 32)
        n = read(ch.csr, regs.n)
        si = read(ch.csr, regs.si)
        if read(ch.csr, regs.count) == c0
            return (count = Int(c0), n = Int(n), sample_index = Int(si),
                    ie = ie, qe = qe, ip = ip, qp = qp, il = il, ql = ql)
        end
    end
    nothing
end

prompt_power(d) = float(d.ip)^2 + float(d.qp)^2

write(csr, "gnss_control", 1)
chans = Chan[]
for (k, acq) in enumerate(strong[1:n_track])
    ch = bank.channels[k]
    doppler0 = Float64(ustrip(Hz, acq.carrier_doppler))
    code_step = code_word(ch, doppler0)
    sample_shift = max(1, round(Int, 0.5 * (1 << FRAC) / code_step))
    load_code!(ch, acq.prn, [get_code(gpsl1, i, acq.prn) > 0 ? 1 : 0 for i = 0:1022])
    write(csr, ch.prefix * "spacing", spacing_word(ch, sample_shift, doppler0))
    write(csr, ch.prefix * "carrier_freq", carrier_word(ch, doppler0))
    write(csr, ch.prefix * "code_freq", code_step)
    push!(chans, Chan(ch, DumpRegs(ch), acq.prn, doppler0,
                      code_step / (1 << FRAC), 0, NaN, NaN, NaN,
                      false, -1, doppler0, doppler0, Float64[], 0, 0, 0))
end

# One TrackState for everyone. The code phase seed is a placeholder: the loop
# is rate-only, so Tracking's absolute code phase is bookkeeping the device
# never sees — the discriminators work on the accumulators alone.
doppler_estimator = ConventionalAssistedPLLAndDLL()
track_state = TrackState(
    gpsl1,
    [TrackedSat(gpsl1, c.prn, 0.0, c.doppler0 * Hz; doppler_estimator) for c in chans];
    doppler_estimator,
)
sampling_frequencies = (L1 = FS_NOMINAL,)

# ---- servicing: the per-cycle heartbeat for every locked channel --------------
function service!(track_state, chans)
    fresh = Chan[]
    for c in chans
        c.locked || continue
        # One ioctl to see whether there is anything new before twelve more.
        Int(read(c.ch.csr, c.regs.count)) == c.prev_count && continue
        d = coherent_dump(c.ch, c.regs)
        (d === nothing || d.count == c.prev_count || d.n == 0) && continue
        c.prev_count = d.count
        accs = SVector{3,ComplexF64}(
            ComplexF64(d.il, d.ql),
            ComplexF64(d.ip, d.qp),
            ComplexF64(d.ie, d.qe),
        )
        append_correlator_output!(
            track_state,
            CorrelatorOutput(EarlyPromptLateCorrelator(accs, PREFERRED_CHIP_SHIFT),
                             d.n, d.sample_index),
            c.prn,
        )
        push!(c.powers, prompt_power(d))
        c.last_si = d.sample_index
        push!(fresh, c)
    end
    isempty(fresh) && return track_state
    track_state = estimate_dopplers_and_filter_prompt!(track_state, sampling_frequencies)
    for c in fresh
        sat = get_sat_state(track_state, c.prn)
        c.carrier_hz = Float64(ustrip(Hz, get_carrier_doppler(sat)))
        c.code_dopp_hz = Float64(ustrip(Hz, get_code_doppler(sat))) *
                         (GPS_L1_HZ / GPS_CA_CHIP_RATE)
        st = apply_status(c.ch)
        if !st.armed
            schedule!(c.ch, c.last_si + FEEDBACK_EPOCHS * EPOCH_SAMPLES;
                      carrier_hz = c.carrier_hz, code_doppler_hz = c.code_dopp_hz)
            c.commits += 1
        else
            c.skipped += 1
        end
    end
    track_state
end

# ---- sweep for one satellite, servicing the others between trials -------------
function measure!(track_state, chans, c::Chan, phases, dumps_each)
    out = Tuple{Float64,Float64}[]
    for phi_base in phases
        target = sample_count(bank) + MARGIN
        phi = mod(phi_base + (target - c.S0) * c.r, 1023)
        schedule!(c.ch, target; carrier_hz = c.doppler0, code_doppler_hz = c.doppler0,
                  code_phase_chips = phi)
        while apply_status(c.ch).armed
        end
        powers = Float64[]
        seen = 0
        while length(powers) < dumps_each
            d = coherent_dump(c.ch, c.regs)
            d === nothing && break
            seen += 1
            seen > SKIP && push!(powers, prompt_power(d))
        end
        !isempty(powers) && push!(out, (Float64(phi_base), mean(powers)))
        track_state = service!(track_state, chans)
    end
    out, track_state
end

function handover!(track_state, chans, c::Chan; tries = 10)
    for attempt = 1:tries
        GC.gc()
        target = sample_count(bank) + MARGIN
        schedule!(c.ch, target; carrier_hz = c.doppler0, code_doppler_hz = c.doppler0,
                  code_phase_chips = mod(c.phi_peak + (target - c.S0) * c.r, 1023))
        while apply_status(c.ch).armed
        end
        st = apply_status(c.ch)
        aa = Int(applied_at(c.ch))
        if !st.late && aa == target
            return true
        end
        @printf("  PRN %d handover attempt %d slipped %+d samples, retrying\n",
                c.prn, attempt, aa - target)
        track_state = service!(track_state, chans)
    end
    false
end

for c in chans
    global track_state
    @printf("\n-- PRN %d on %s --\n", c.prn, c.ch.prefix)
    c.S0 = sample_count(bank)
    coarse, track_state = measure!(track_state, chans, c, 0:1022, 10)
    best_coarse = coarse[argmax(last.(coarse))][1]
    fine, track_state = measure!(track_state, chans, c,
                                 [mod(best_coarse - 3 + 0.25i, 1023) for i = 0:24], 40)
    phi_fine, peak_power = fine[argmax(last.(fine))]
    c.floor_power = median(last.(fine))
    recheck, track_state = measure!(track_state, chans, c,
                                    [mod(phi_fine - 2 + 0.25i, 1023) for i = 0:16], 20)
    rc_best = recheck[argmax(last.(recheck))]
    c.phi_peak = rc_best[1]
    c.peak_power = rc_best[2]
    ratio = c.peak_power / c.floor_power
    @printf("  peak %.2fx floor at %.2f chips (fine sweep said %.2f at %.2fx)\n",
            ratio, c.phi_peak, phi_fine, peak_power / c.floor_power)
    if ratio < 2
        @printf("  no usable peak, skipping PRN %d\n", c.prn)
        continue
    end
    if handover!(track_state, chans, c)
        c.locked = true
        c.prev_count = -1
        @printf("  handover committed; PRN %d is now closed-loop\n", c.prn)
    else
        @printf("  handover kept slipping, skipping PRN %d\n", c.prn)
    end
end

locked = [c for c in chans if c.locked]
if isempty(locked)
    println("\nno satellite made it to closed loop"); exit(1)
end

# ---- the joint loop -----------------------------------------------------------
@printf("\nclosing the loop on %d satellite(s) for %.0f s\n\n", length(locked), seconds)
print("   t(s)")
for c in locked
    @printf("   PRN%02d |P|^2/fl  carr(Hz)", c.prn)
end
println()

t0 = time()
last_print = 0.0
while time() - t0 < seconds
    global track_state = service!(track_state, chans)
    t = time() - t0
    if t - last_print >= 1.0
        global last_print = t
        @printf("  %5.1f", t)
        for c in locked
            win = @view c.powers[max(1, end - 199):end]
            @printf("   %14.2f  %8.1f", isempty(win) ? 0.0 : mean(win) / c.floor_power,
                    c.carrier_hz)
        end
        println()
        reset_start_sample_and_bit_buffer!(track_state)
    end
end

println()
for c in locked
    win = @view c.powers[max(1, end - 399):end]
    @printf("PRN %2d: final %.2fx floor (open-loop peak %.2fx), carrier %+.1f Hz, ",
            c.prn, isempty(win) ? 0.0 : mean(win) / c.floor_power,
            c.peak_power / c.floor_power, c.carrier_hz)
    @printf("%d dumps folded, %d commits (%d skipped)\n",
            length(c.powers), c.commits, c.skipped)
end
@printf("saturation=0x%x overflow=0x%x\n", read(csr, "gnss_saturation"), overflow(bank))
close(csr)
