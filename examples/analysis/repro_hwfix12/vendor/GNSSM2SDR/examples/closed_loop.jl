# Closed-loop tracking of a live satellite on the on-FPGA correlators, with
# Tracking.jl doing the loop filters.
#
# This is the seam GNSSReceiver.jl#107 is about: the FPGA downconverts and
# correlates, its Early/Prompt/Late dumps are appended to a satellite's
# `correlator_outputs` buffer through Tracking.jl's external-producer path, the
# estimator folds them into one NCO update per epoch, and the update is written
# back over CSR. No correlation on the CPU, and no reimplementation of the
# discriminators or loop filters -- those are Tracking.jl's, already optimised and
# tested.
#
# Device control is GNSSM2SDR.jl's csr.jl/bank.jl, included as M2Bank so this can
# run on the board without GNSSReceiver's full dependency tree.
#
# Usage: julia --project=. closed_loop.jl CSR_CSV PRN DOPPLER_HZ [SECONDS]

using Printf
using StaticArrays: SVector
using Statistics: mean, median
using Unitful: Hz, ustrip

using Acquisition: acquire
using GNSSSignals: GPSL1CA, get_code, get_code_frequency, get_code_center_frequency_ratio
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
    get_code_phase,
    get_prompt,
    get_correlator_outputs,
    reset_start_sample_and_bit_buffer!

# Device control is GNSSM2SDR's own csr.jl / bank.jl. On the Orin this was run with
# those two files pulled into a standalone `M2Bank` module instead, to avoid
# precompiling GNSSReceiver's dependency tree on the board -- swap this for
# `include("M2Bank.jl"); using .M2Bank` to do the same.
using GNSSM2SDR:
    LiteXCSR, GNSSBank, GNSSBankChannel, read_signed,
    carrier_word, code_word, load_code!, schedule!, apply_status, applied_at,
    sample_count, overflow, spacing_word,
    GPS_L1_HZ, GPS_CA_CHIP_RATE, CA_CODE_LENGTH

const FS_NOMINAL = 4e6Hz         # the rate every NCO word is referred to
const FS_NOMINAL_HZ = 4e6
const FRAC = 24
const MARGIN = 24_000
const SKIP = 2
const PREFERRED_CHIP_SHIFT = 0.5  # chips, the value the gateware is programmed with
# Set GNSS_NO_FEEDBACK=1 to run everything except the NCO write-back. If the prompt
# stays strong then the handover, the dump reading and the CorrelatorOutput
# construction are all fine and only the feedback is at fault.
const NO_FEEDBACK = get(ENV, "GNSS_NO_FEEDBACK", "0") == "1"
const EPOCH_SAMPLES = 4000       # one GPS L1 C/A code period at 4 MHz
const FEEDBACK_EPOCHS = 2        # loop delay, in epochs, made deterministic

csv     = ARGS[1]
# PRN 0 means "acquire them all and track the strongest" -- satellites here rise
# and set within the hour, so hand-picking one goes stale fast.
prn_arg = parse(Int, ARGS[2])
seconds = length(ARGS) > 2 ? parse(Float64, ARGS[3]) : 15.0
# Doppler coverage well past +-7 kHz: the reference offset shows up as an apparent
# Doppler on top of the satellite's own, and observed offsets past 9 kHz are normal
# on this board. A Doppler clipped to a narrow grid is ~1 kHz out, which is exactly
# the first null of a 1 ms coherent integration.
const DOPPLER_COVERAGE = 30_000.0Hz

gpsl1 = GPSL1CA()
csr  = LiteXCSR(csv)
bank = GNSSBank(csr; fs = FS_NOMINAL, n_channels = 2)
ch   = bank.channels[1]

# ---- acquire in THIS process, so the Doppler is fresh -----------------------
# A Doppler even a couple of minutes old is a few hundred Hz out, and 100 Hz is
# ~0.065 chips/s of code-rate error -- two or three chips across the phase sweep
# below, which smears the peak over as many bins and makes the search unreliable.
# There must also be exactly one DMA0 reader, and the bank only sees samples while
# DMA0 drains, so one long capture serves as both the acquisition source and the
# drain for the whole run.
const CAPTURE_FILE = "/tmp/hwloop_capture.bin"
const ACQ_INSTANTS = 800_000                     # 200 ms: 10 ms coherent x 10
need = ACQ_INSTANTS * 8
# A drain leaked by a crashed earlier run is fatal in the least visible way:
# two DMA0 readers split the buffers, the capture comes out fragmented, and the
# only symptom is 5-10 dB less CN0 and an unreliable phase sweep. Kill any
# leftover reader first, and clean up via atexit so a crash cannot leak ours.
run(ignorestatus(`pkill -x m2sdr_record`))
sleep(0.2)
recorder = run(pipeline(`m2sdr_record $CAPTURE_FILE $(need * 2)`,
                        stdout = devnull, stderr = devnull), wait = false)
while !(isfile(CAPTURE_FILE) && filesize(CAPTURE_FILE) >= need) && process_running(recorder)
    sleep(0.02)
end
raw = reinterpret(Int16, read(CAPTURE_FILE)[1:min(need, filesize(CAPTURE_FILE))])
# Hand the drain over to /dev/null: DMA0 must keep draining for the bank to see
# samples at all, but there is no need to keep writing hundreds of MB. Sample
# continuity across the switch does not matter -- the sweep re-derives the phase.
process_running(recorder) && kill(recorder)
drain = run(pipeline(`m2sdr_record /dev/null $(Int(FS_NOMINAL_HZ * 600) * 8)`,
                     stdout = devnull, stderr = devnull), wait = false)
atexit(() -> process_running(drain) && kill(drain))
rm(CAPTURE_FILE; force = true)
nsmp = length(raw) ÷ 4
words = reshape(view(raw, 1:4nsmp), 4, nsmp)
signal = ComplexF32.(Float32.(view(words, 1, :)), Float32.(view(words, 2, :)))
# 10 ms coherent -> 1/(10 ms) = 100 Hz Doppler bins, so a worst-case residual of
# ~50 Hz. That is already well inside the pull-in: Tracking.jl's own test has
# ConventionalPLLAndDLL converging at a 90 Hz initial error and the assisted
# estimator at 240 Hz. Going to 20 ms would halve the bins but costs acquisition
# time and needs a bit-edge search (a nav-bit transition inside a 20 ms window
# cancels the coherent gain), which is not worth it for resolution that is already
# sufficient.
prns = prn_arg == 0 ? collect(1:32) : [prn_arg]
results = acquire(gpsl1, signal, FS_NOMINAL, prns;
                  min_doppler_coverage = DOPPLER_COVERAGE,
                  num_coherently_integrated_code_periods = 10,
                  num_noncoherent_accumulations = 10)
acq = results[argmax([r.CN0 for r in results])]
prn = acq.prn
doppler0 = Float64(ustrip(Hz, acq.carrier_doppler))
if prn_arg == 0
    strong = sort(filter(r -> r.CN0 > 38, results); by = r -> -r.CN0)
    @printf("acquired %d satellite(s) above 38 dBHz from %d instants:\n",
            length(strong), nsmp)
    for r in strong
        @printf("  PRN %2d  CN0 %.1f dBHz  peak/noise %5.1f  doppler %+7.0f Hz  code phase %7.2f\n",
                r.prn, r.CN0, r.peak_to_noise_ratio,
                Float64(ustrip(Hz, r.carrier_doppler)), r.code_phase)
    end
end
@printf("tracking PRN %d: CN0 %.1f dBHz, peak/noise %.1f, doppler %+.0f Hz, code phase %.2f chips\n",
        prn, acq.CN0, acq.peak_to_noise_ratio, doppler0, acq.code_phase)
if acq.CN0 < 38
    println("CN0 too low to bother tracking")
    close(csr); process_running(drain) && kill(drain); exit(0)
end

code_step = code_word(ch, doppler0)
r = code_step / (1 << FRAC)                      # chips per sample
# Tracking.jl quantises the E/L shift to whole samples and `dll_disc` normalises
# with that quantised spacing, so the gateware has to be programmed with the same
# integer shift.
sample_shift = max(1, round(Int, 0.5 * (1 << FRAC) / code_step))

@printf("PRN %d at %+.0f Hz, code_step=%d, E/L shift %d samples\n",
        prn, doppler0, code_step, sample_shift)

write(csr, "gnss_control", 1)
# The loader wants the raw 0/1 chips; get_code returns +-1.
load_code!(ch, prn, [get_code(gpsl1, i, prn) > 0 ? 1 : 0 for i = 0:1022])
write(csr, ch.prefix * "spacing", spacing_word(ch, sample_shift, doppler0))
write(csr, ch.prefix * "carrier_freq", carrier_word(ch, doppler0))
write(csr, ch.prefix * "code_freq", code_step)

# ---- read a dump the FPGA did not overwrite mid-read -------------------------
# Register names are precomputed: the hot loop reads ~10 CSRs per millisecond and
# building the names each time is avoidable GC pressure, and GC pauses are what
# make scheduled commits miss their apply sample.
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

const DUMP_REGS = DumpRegs(ch)

prompt_power(d) = float(d.ip)^2 + float(d.qp)^2

# ---- drift-compensated code-phase search ------------------------------------
# The satellite's code phase moves a few chips/s, and each trial phase is
# restarted at a different sample, so the trial is expressed in the satellite's
# frame: phi_j = phi_base + (S_j - S0) * r.
function measure(bank, ch, doppler, r, S0, phases, dumps_each)
    out = Tuple{Float64,Float64}[]
    for phi_base in phases
        target = sample_count(bank) + MARGIN
        phi = mod(phi_base + (target - S0) * r, 1023)
        schedule!(ch, target; carrier_hz = doppler, code_doppler_hz = doppler,
                  code_phase_chips = phi)
        while apply_status(ch).armed
        end
        powers = Float64[]
        seen = 0
        while length(powers) < dumps_each
            d = coherent_dump(ch, DUMP_REGS)
            d === nothing && break
            seen += 1
            seen > SKIP && push!(powers, prompt_power(d))
        end
        !isempty(powers) && push!(out, (Float64(phi_base), mean(powers)))
    end
    out
end

S0 = sample_count(bank)
coarse = measure(bank, ch, doppler0, r, S0, 0:1022, 10)
best_coarse = coarse[argmax(last.(coarse))][1]
fine = measure(bank, ch, doppler0, r, S0,
               [mod(best_coarse - 3 + 0.25i, 1023) for i = 0:24], 40)
phi_peak, peak_power = fine[argmax(last.(fine))]
floor_power = median(last.(fine))
@printf("\ncode phase %.2f chips, peak %.2fx floor\n",
        phi_peak, peak_power / floor_power)
if peak_power / floor_power < 3
    println("no usable peak -- not closing the loop")
    close(csr)
process_running(drain) && kill(drain)
    exit(0)
end

# ---- hand the acquisition to Tracking.jl ------------------------------------
# `TrackState` is seeded with the code phase and Doppler the search just
# confirmed; from here Tracking.jl owns the loop.
# The FLL-assisted estimator, for its much wider pull-in: Tracking.jl's own test
# has ConventionalPLLAndDLL converging at a 90 Hz initial Doppler error and failing
# at 100 Hz, while ConventionalAssistedPLLAndDLL converges at 240 Hz. A 10 ms
# coherent acquisition leaves at most ~50 Hz (half of a 100 Hz bin), so either
# would do, but the assisted one leaves real margin for a stale Doppler.
doppler_estimator = ConventionalAssistedPLLAndDLL()
track_state = TrackState(
    gpsl1,
    [TrackedSat(gpsl1, prn, phi_peak, doppler0 * Hz; doppler_estimator)];
    doppler_estimator,
)
# Keyed by RF band (Tracking.jl looks up sampling_frequencies[band]), not by group.
sampling_frequencies = (L1 = FS_NOMINAL,)

# Re-measure around the peak with the SAME function that found it, so any
# disagreement isolates the handover rather than the search.
recheck = measure(bank, ch, doppler0, r, S0,
                  [mod(phi_peak - 2 + 0.25i, 1023) for i = 0:16], 20)
rc_floor = median(last.(recheck))
rc_best = recheck[argmax(last.(recheck))]
println("\nre-check around the peak (same `measure` path):")
for (ph, pw) in recheck
    off = mod(ph - phi_peak + 511.5, 1023) - 511.5
    @printf("  offset %+6.2f chips: %8.2fx\n", off, pw / rc_floor)
end
@printf("  best at offset %+.2f chips, %.2fx\n",
        mod(rc_best[1] - phi_peak + 511.5, 1023) - 511.5, rc_best[2] / rc_floor)

# Hand over on the re-check's own maximum: it is the freshest measurement and,
# on a marginal satellite, the fine sweep's peak bin is one noisy 40-dump
# average -- a 1-chip disagreement here is exactly what run-to-run losses
# looked like (the DLL pulls in ~1 chip on a strong signal, but not more).
if rc_best[2] / rc_floor > 3 && rc_best[1] != phi_peak
    @printf("  handing over on the re-check maximum (%.2f chips) instead\n", rc_best[1])
    phi_peak = rc_best[1]
end

# The handover commit MUST land on its exact sample: a code-phase commit applied
# N samples late puts the replica (N * r mod 1023) chips off -- measured on this
# board as a 15 ms slip = 207 chips = total, permanent signal loss, which is what
# every earlier "the loop instantly loses the signal" run turned out to be. The
# 15 ms stall is host-side (a GC pause fits), so collect garbage first, then
# verify `late`/`applied_at` and retry until one lands exactly. The sweep frame
# (phi_peak, S0, r) stays valid across retries -- that is the whole point of the
# drift-compensated parameterisation.
function handover!(bank, ch, doppler, r, S0, phi_peak; tries = 10)
    for attempt = 1:tries
        GC.gc()
        target = sample_count(bank) + MARGIN
        schedule!(ch, target; carrier_hz = doppler, code_doppler_hz = doppler,
                  code_phase_chips = mod(phi_peak + (target - S0) * r, 1023))
        while apply_status(ch).armed
        end
        st = apply_status(ch)
        aa = Int(applied_at(ch))
        if !st.late && aa == target
            @printf("handover committed at sample %d (attempt %d)\n", target, attempt)
            return target
        end
        @printf("handover attempt %d slipped %+d samples (late=%s), retrying\n",
                attempt, aa - target, st.late)
    end
    error("handover kept missing its apply sample after $tries attempts")
end

handover!(bank, ch, doppler0, r, S0, phi_peak)

# ---- the loop, in a function so the hot path is not in global scope ---------
function run_loop(csr, ch, bank, gpsl1, prn, track_state, sampling_frequencies,
                  sample_shift, floor_power, peak_power, seconds)
    @printf("\nclosing the loop for %.0f s -- Tracking.jl folds the dumps and the\n",
            seconds)
    @printf("NCO updates go back over CSR\n\n")
    println("   t(s)  epochs   |P|^2/floor   carrier(Hz)   code_dopp(Hz)   |prompt|")

    t0 = time()
    epochs = 0
    prev_count = -1
    powers = Float64[]
    sizehint!(powers, round(Int, seconds * 1100))
    last_print = 0.0
    carrier_hz = 0.0
    code_dopp_hz = 0.0
    commits = 0
    commits_skipped = 0
    commits_late = 0
    while time() - t0 < seconds
        d = coherent_dump(ch, DUMP_REGS)
        (d === nothing || d.count == prev_count) && continue
        prev_count = d.count
        d.n == 0 && continue

        # The dump *is* Tracking.jl's CorrelatorOutput: accumulators in its
        # [late, prompt, early] order, the integrated sample count, and the
        # free-running sample index.
        accs = SVector{3,ComplexF64}(
            ComplexF64(d.il, d.ql),
            ComplexF64(d.ip, d.qp),
            ComplexF64(d.ie, d.qe),
        )
        # The second argument is the preferred Early/Late-to-prompt shift in
        # CHIPS, not the sample shift. Passing the sample shift (2) makes
        # get_early_late_sample_spacing report 16 samples = 4.09 chips, and
        # dll_disc's normalisation (2 - d)/2 then goes NEGATIVE (-1.046) -- the DLL
        # inverts, drives the code phase off the peak within a fraction of a
        # second, and the third-order carrier filter winds up on the noise that
        # follows. 0.5 chips is what the gateware is actually programmed with
        # (a 2-sample shift at 4 MHz is 0.5115 chips), giving +0.4885.
        correlator = EarlyPromptLateCorrelator(accs, PREFERRED_CHIP_SHIFT)
        # append_correlator_output!(track_state, output, prn): the output comes
        # second, then the same addressing forms as the per-signal accessors.
        append_correlator_output!(track_state,
                                  CorrelatorOutput(correlator, d.n, d.sample_index),
                                  prn)

        track_state = estimate_dopplers_and_filter_prompt!(track_state,
                                                           sampling_frequencies)
        sat = get_sat_state(track_state, prn)
        carrier_hz = Float64(ustrip(Hz, get_carrier_doppler(sat)))
        # Tracking.jl reports the code Doppler in chips/s; the CSR helper wants it
        # as the equivalent carrier Doppler.
        code_dopp_hz = Float64(ustrip(Hz, get_code_doppler(sat))) *
                       (GPS_L1_HZ / GPS_CA_CHIP_RATE)
        # Apply at a KNOWN future epoch, exactly as GNSSReceiver's
        # HardwareCorrelatorLink does (apply_at_sample = boundary +
        # feedback_delay_epochs * epoch_length). Writing the words immediately
        # instead leaves the loop delay unmodelled and variable -- a 1 ms loop with
        # default bandwidths does not tolerate that.
        if !NO_FEEDBACK
            # RATE-ONLY feedback, and never replace a pending commit. Two earlier
            # architectures both failed for reasons that looked like signal decay:
            #
            #  * Re-arming `schedule!` every epoch: arming REPLACES the pending
            #    commit, and with the loop re-arming ~1 ms after every dump while
            #    each commit still had ~1 epoch to wait, commits were replaced
            #    before they ever fired -- the NCO words never reached the
            #    hardware at all. So only schedule when the previous commit has
            #    actually applied (armed has cleared).
            #  * Commanding the code phase per epoch: a code-phase commit also
            #    restarts the integration at that sample, so the loop perturbed
            #    the very dumps it was folding. The DLL corrects phase through
            #    the rate -- that is what a DLL is -- so phase actuation belongs
            #    in the acquisition handover only.
            #
            # A commit that applies late is harmless here: a frequency word a few
            # epochs behind schedule is still the right frequency to first order,
            # unlike a phase word, which becomes garbage. Count them anyway.
            st = apply_status(ch)
            if !st.armed
                st.late && (commits_late += 1)
                schedule!(ch, d.sample_index + FEEDBACK_EPOCHS * EPOCH_SAMPLES;
                          carrier_hz = carrier_hz, code_doppler_hz = code_dopp_hz)
                commits += 1
            else
                commits_skipped += 1
            end
        end

        epochs += 1
        push!(powers, prompt_power(d))
        t = time() - t0
        if t - last_print >= 1.0
            last_print = t
            win = @view powers[max(1, end - 199):end]
            @printf("  %5.1f  %6d   %11.2f   %11.1f   %13.1f   %8.0f\n",
                    t, epochs, mean(win) / floor_power, carrier_hz, code_dopp_hz,
                    abs(get_prompt(correlator)))
            # Nobody reads the nav bits here, and the hard-bit buffer throws once
            # 128 bits pile up (2.56 s after bit sync) -- normally `track!` resets
            # it at the start of every call. Resetting keeps the bit-edge sync
            # and the partial-bit accumulator; only the decoded bits are dropped.
            reset_start_sample_and_bit_buffer!(track_state)
        end
    end

    win = @view powers[max(1, end - 399):end]
    @printf("\n%d epochs in %.0f s (%.0f Hz update rate)\n",
            epochs, seconds, epochs / seconds)
    @printf("NCO commits: %d fired, %d skipped (previous still pending), %d applied late\n",
            commits, commits_skipped, commits_late)
    @printf("final mean |P|^2/floor = %.2fx (open-loop peak was %.2fx)\n",
            isempty(powers) ? 0.0 : mean(win) / floor_power,
            peak_power / floor_power)
    @printf("carrier ended at %+.1f Hz, code Doppler %+.1f Hz\n",
            carrier_hz, code_dopp_hz)
    @printf("saturation=0x%x overflow=0x%x\n",
            read(csr, "gnss_saturation"), overflow(bank))
end

run_loop(csr, ch, bank, gpsl1, prn, track_state, sampling_frequencies,
         sample_shift, floor_power, peak_power, seconds)
close(csr)
process_running(drain) && kill(drain)

# ---------------------------------------------------------------------------
# State of the investigation — RESOLVED: the loop locks on hardware
#
# Verified on the Orin (2026-07-29): 60 s at 60-84x the noise floor, then 180 s
# without losing lock — ~180k epochs at a 999 Hz update rate, ~90k NCO commits
# applied, zero late, carrier tracking the satellite's physical Doppler ramp
# (-0.77 Hz/s) throughout.
#
# The earlier "the loop instantly loses the signal / free-running dumps decay"
# mystery was never signal decay. A decay probe (no feedback at all, watching
# every dump after a handover, then re-sweeping) showed the satellite exactly
# where the drift-compensated frame predicted, dump cadence exact, and ONE
# anomaly: the handover commit had applied 15 ms (60797 samples) LATE. A
# code-phase commit applied N samples after the sample its phase word was
# computed for puts the replica (N*r mod 1023) chips off -- ~207 chips here, so
# the correlators saw pure noise from t=0 and no DLL could ever recover it.
# `late` was never checked; the sweep's `measure` masked the failure mode
# because it re-schedules per trial and a late trial just reads as noise.
#
# What it took to lock, each individually fatal:
#
#  * Handover: retry until `!late && applied_at == target` (see `handover!`),
#    with a GC.gc() first -- the 15 ms stall is host-side, and a Julia GC pause
#    right after the allocation-heavy sweep fits the measurements.
#  * Feedback: RATE-ONLY, and never replace a pending commit. Re-arming
#    `schedule!` replaces the armed commit, and at a ~1 ms loop period every
#    commit was replaced ~1.5 ms before it would have fired -- the NCO words
#    never reached the hardware. Re-arm only once `armed` clears (~500 Hz
#    effective update rate). A late RATE commit is harmless (it is still the
#    right frequency); a late PHASE commit is garbage, which is why phase
#    actuation belongs in the handover only. Per-epoch phase commands also
#    restart the integration window every time they fire, perturbing the very
#    dumps being folded.
#  * Read the nav bits out (or reset the bit buffer): Tracking.jl's hard-bit
#    buffer throws once 128 bits accumulate after bit sync -- 2.56 s into what
#    looks like a perfect lock. `track!` resets it each call; a dump-driven
#    loop has to do it itself (`reset_start_sample_and_bit_buffer!`).
#  * Kill leaked DMA0 readers at startup and clean up on every exit path: a
#    drain left behind by a crashed run splits the buffers with the new one,
#    and the only symptom is 5-10 dB less CN0 and an unreliable sweep.
#  * Hand over on the re-check's own maximum: on a marginal satellite the fine
#    sweep's peak bin is one noisy 40-dump average, and a 1-chip disagreement
#    is more than the DLL can pull in at this integration length.
#
# Still open, deliberately: multi-channel scale-up, the CorrelatorDump DMA ring
# (readout here is CSR polling at ~1 kHz, which one channel tolerates), and
# wiring this same sequence through GNSSReceiver's HardwareCorrelatorLink,
# whose rate-only NCOUpdate architecture the measurements above vindicate.
