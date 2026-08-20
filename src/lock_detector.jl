"""
    AbstractLockDetector

Supertype for the receiver's per-satellite lock detectors. A concrete detector is
advanced with `update` and queried with [`is_in_lock`](@ref); a satellite is treated
as locked only while both its code and carrier detectors report lock.

The detectors track elapsed signal time rather than a number of `update` calls, so
their behaviour is independent of how the incoming signal is chunked: each `update`
is handed the `signal_duration` it represents and accumulates that duration into its
timers.

## Why the timings are in primary code periods, not seconds

`Tracking` sizes its default loop bandwidths from the primary code period `T`, at
`B_L·T ≈ 0.018`: `B_L = 0.018/T` for the carrier loop and `B_L/18` for the code loop. So
every loop time constant scales with `T` — `1/B_L` is `55.6·T` for the carrier and
`1000·T` for the code loop:

| signal | `T` | carrier `B_L` | code `B_L` | `1/B_L` carrier | `1/B_L` code |
|---|---|---|---|---|---|
| GPS L1 C/A | 1 ms | 18 Hz | 1 Hz | 56 ms | 1.0 s |
| Galileo E1B | 4 ms | 4.5 Hz | 0.25 Hz | 222 ms | 4.0 s |
| GPS L1C-D | 10 ms | 1.8 Hz | 0.1 Hz | 556 ms | 10.0 s |

A dwell fixed in *seconds* therefore means something different on every signal. The 200 ms
this receiver used to allow is 200 code periods on GPS L1 C/A — enough — but only 50 on
Galileo E1B and 20 on GPS L1C-D, against loops that are 4× and 10× slower. That is why lock
detection failed on the slower signals while GPS L1 C/A looked fine. Counted in code
periods the same number means the same thing everywhere, and the GPS L1 C/A defaults are
reproduced exactly (`T = 1 ms`, so 200 code periods *is* 200 ms).

So every detector takes a `reference_integration_time` — one primary code period of the
signal it runs on, which the receiver passes from `primary_code_period` — and takes its
timings as counts of that period. The fields themselves stay in seconds.
"""
abstract type AbstractLockDetector end

"""
    LockDwell

The out-of-lock dwell shared by the receiver's lock detectors: a timer that accumulates
while the detector's statistic reports "no usable signal" and declares lock lost once it
reaches a threshold. Both [`CodeLockDetector`](@ref) and [`CarrierLockDetector`](@ref) hold
one, so the handover staging below is defined in exactly one place. Its timings are counts of
one primary code period — see [`AbstractLockDetector`](@ref) for why.

## Why there are two stages

The acquisition → tracking handover is deliberately coarse: the acquisition Doppler bin is
sized to half the loop's pull-in range (`receive`'s `pull_in_margin`), and the code phase is
only resolved to the sample grid, halved by `subsample_interpolation`. The loops need time to
pull those errors in, and until they have, the prompt correlator sits off the correlation peak
and carries less power than the satellite really has — so a *healthy* satellite reads below
its true C/N0, and its carrier phase is still spinning at the residual Doppler, so the
phase-lock indicator reads near zero.

Measured on synthetic handovers at this receiver's own worst-case residual (see the commit
that introduced this type), the peak accumulated out-of-lock time a satellite the receiver
should keep demands is **292 code periods** — GPS L1 C/A at 32 dBHz, a 0.25-chip and 125 Hz
handover at 2.048 MHz. That is more than the 200-code-period steady-state dwell tolerates, so
a single un-staged dwell would drop it. The carrier detector's demand is larger again, 460 to
590 code periods at a healthy 34 dBHz, because the FLL has to pull in the residual Doppler
before `cos(2φ)` stops averaging to zero.

So the dwell has two stages:

  * **pull-in**, from construction: tolerates `pull_in_out_of_lock_time_threshold`, sized to
    survive the whole handover transient with margin over the measured worst case;
  * **steady state**, latched permanently once the handover is over: tolerates the tighter
    `out_of_lock_time_threshold`.

Two stages rather than one long dwell, because they answer different questions. A single dwell
would have to be ≳600 code periods to be safe, and would then take three times as long to
notice a satellite that genuinely disappeared. Staging buys handover tolerance without paying
for it in steady-state detection latency.

The handover counts as over on *either* of two conditions:

  * `confirm_time_threshold` of uninterrupted favourable evidence with nothing left on the
    out-of-lock clock — the satellite has demonstrably converged, so there is no reason to
    keep extending it handover credit;
  * `pull_in_time_threshold` has elapsed and the accumulated out-of-lock time is under the
    steady-state threshold — the backstop for a satellite that never quite settles.

Both are needed. Without the evidence-based one, a satellite that converged immediately would
still be treated as pulling in until the timer expired, and a fade during that window would
take the *pull-in* dwell to notice. Without the timer, a satellite that never strings together
a clean run never leaves the stage.

Requiring the accumulated time to be under the steady-state threshold before latching matters
too: otherwise a satellite sitting comfortably inside the generous pull-in threshold would be
declared lost the instant the tighter one took over.

One residual case is left open deliberately. Both latches require the out-of-lock clock to be
under the steady-state threshold, so a satellite whose clock parks *between* the two
allowances — 200 to 600 code periods — never leaves the pull-in stage, and so never becomes
ranging-ready either. It takes a burst of failures followed by a sustained near-50% duty
cycle; stochastically the clock random-walks out of that band, either down to zero (whereupon
it latches) or past the pull-in allowance (whereupon lock is lost and the satellite is
reacquired). Closing it would mean letting the pull-in stage latch with its clock still above
the steady-state threshold, which is exactly the discontinuity the latch guard prevents.

## Why ranging readiness is a separate, later latch

`pulled_in` answers "may the dwell tighten?" and wants to fire *early*, so a fade after a
successful handover is caught on the steady-state timescale. A pseudorange consumer needs
something different and slower: the **code loop** to have settled. Those two conflict, and
measurement shows one threshold cannot serve both.

C/N0 recovers well before the code phase does — on BPSK the prompt only loses `1 − |Δτ|`, so a
GPS L1 C/A satellite reads healthy almost immediately while its DLL (`1/B_L` = 1000 code
periods) is still walking the handover error in. Measured at the same 0.25-chip handover, the
residual code phase is 0.15 to 0.22 chip after 200 code periods — **44 to 64 m of pseudorange
error** — falling to 0.018 to 0.15 chip, 5 to 44 m, by 600. So there are two latches over the
same `settled_time`: `confirm_time_threshold` (200 code periods) tightens the dwell, and
`ranging_confirm_time_threshold` (600) admits the satellite to ranging via
[`is_ranging_ready`](@ref). It is floored at `confirm_code_periods`, since ranging can never
be ready before the handover is over.

Being conservative on the second is close to free: the earliest it can fire is the warm-up plus
one clean run, 680 code periods, and measured on real handovers it lands at 760 to 1500 on GPS
L1 C/A — the carrier loop has its own residual Doppler to pull in before the phase-lock
indicator stops resetting the run — so still inside `receive`'s own 2 s
`time_in_lock_before_calculating_pvt` gate. On slower signals the extra seconds are dwarfed by
the tens of seconds of subframe decoding that actually set time to first fix.

Like the pull-in stage, ranging readiness needs a *timer* as well as an evidence path, and for
the same reason: `settled_time` is reset by any out-of-lock verdict, so an uninterrupted run of
600 is a bar a satellite can fail to clear indefinitely — while sitting comfortably in lock,
tracked, converged, and silently absent from the PVT solve. Since PVT needs four satellites,
that trades a degraded fix for no fix.

Under Tracking's default noise-reference CN0 estimator the *code* detector rarely blocks that
run: measured steady-state per-chunk out-of-lock rates with no handover error are 0.58 at the
30 dBHz threshold itself, 0.10 one dB above it and 0.00 from two dB up — so a satellite the
code detector holds at all completes a clean run easily. The **carrier** detector is what
blocks it: its 460-to-590-code-period pull-in transient resets `settled_time` well after the
code detector is happy, and any recurring disturbance shorter than the run reopens the same
hole at any C/N0. So `ranging_pull_in_time_threshold` (2000 code periods) backstops it, under
the same guard as `pulled_in`'s timer — the out-of-lock clock must be within what steady state
tolerates, so a channel that is genuinely failing is dropped by `is_in_lock` rather than
admitted by the timer. It also takes `pulled_in` as a conjunct, so "ranging is never ready
before the handover is over" holds structurally rather than by threshold ordering.

Two code-loop time constants (`1/B_L` = 1000 code periods) rather than anything shorter,
because the backstop admits a measurement it has not seen clean evidence for, and what the
DLL's convergence depends on is *elapsed* time, not clean-run length. What it concedes is
thermal jitter on a marginal satellite's code phase, not a handover bias: for the default 1 Hz
code loop and Tracking's 1-chip early-late spacing, Kaplan & Hegarty's coherent-DLL expression
gives 1σ = 0.039 chip (11 m) at 30 dBHz and 0.027 chip (8 m) at 32, against 0.004 chip (1.2 m)
at 45 — inside the same band the evidence path already accepts. On GPS L1 C/A the timer is 2 s,
exactly `receive`'s default `time_in_lock_before_calculating_pvt`, so there it costs nothing.

Before `warm_up_time_threshold` no evidence is accumulated at all. Under the noise-reference
estimator an unmeasured channel reads `-Inf dBHz` and so *fails* the CN0 test rather than
passing it, so the warm-up is not there to hide an estimator's cold bias — it is there for the
carrier detector's own moving averages, which start empty (see [`CarrierLockDetector`](@ref)).

`resets_on_lock` selects how favourable evidence is credited. `false` pays the accumulated time
back one `signal_duration` at a time. `true` clears it outright, which is the "optimistic"
detector of Kaplan & Hegarty §5.11.2 — it "decides quickly and changes its mind slowly", so
declaring loss needs a *consecutive* run of failures. The carrier detector uses it, because a
marginal satellite's phase-lock indicator dips below threshold intermittently and a paying-back
accumulator would random-walk across any threshold.
"""
struct LockDwell
    elapsed::typeof(1.0s)
    out_of_lock_time::typeof(1.0s)
    # Uninterrupted favourable signal time; reset by any out-of-lock verdict.
    settled_time::typeof(1.0s)
    # Latches once the handover transient is over; never returns to the pull-in stage.
    pulled_in::Bool
    # Latches once the loops have settled enough to range on — a longer, separate bar, on the
    # same evidence-or-timer pattern as `pulled_in`; see `is_ranging_ready`.
    ranging_ready::Bool
    warm_up_time_threshold::typeof(1.0s)
    pull_in_time_threshold::typeof(1.0s)
    confirm_time_threshold::typeof(1.0s)
    ranging_confirm_time_threshold::typeof(1.0s)
    ranging_pull_in_time_threshold::typeof(1.0s)
    pull_in_out_of_lock_time_threshold::typeof(1.0s)
    out_of_lock_time_threshold::typeof(1.0s)
    resets_on_lock::Bool
end

"""
    LockDwell(; reference_integration_time, kwargs...)

Build a dwell whose timings are the given multiples of `reference_integration_time` — one
primary code period of the signal the detector runs on.

The defaults are sized from measured handover transients (see the type's docstring) and, for a
1 ms code period, reproduce this receiver's historical absolute timings: an 80 ms warm-up and a
200 ms steady-state dwell.

`pull_in_out_of_lock_code_periods` is floored at `out_of_lock_code_periods`, so the pull-in
stage can never be *stricter* than steady state however the steady-state dwell is set. A
detector whose steady-state dwell already exceeds the pull-in default — the carrier one, at
4000 code periods — is therefore effectively unstaged, which is correct: it clears its
accumulator on any favourable chunk, so its dwell already asks for a consecutive run of
failures far longer than any handover transient.

`ranging_pull_in_code_periods` is floored at `ranging_confirm_code_periods` for the
mirror-image reason: a backstop that expired before the evidence path could fire would replace
the criterion rather than back it up.
"""
function LockDwell(;
    reference_integration_time,
    warm_up_code_periods = 80,
    pull_in_code_periods = 800,
    confirm_code_periods = 200,
    ranging_confirm_code_periods = 600,
    ranging_pull_in_code_periods = 2000,
    out_of_lock_code_periods = 200,
    pull_in_out_of_lock_code_periods = max(600, out_of_lock_code_periods),
    resets_on_lock = false,
)
    T = code_period_reference(reference_integration_time)
    check_code_periods(:warm_up_code_periods, warm_up_code_periods)
    check_code_periods(:pull_in_code_periods, pull_in_code_periods)
    check_code_periods(:confirm_code_periods, confirm_code_periods)
    check_code_periods(:ranging_confirm_code_periods, ranging_confirm_code_periods)
    check_code_periods(:ranging_pull_in_code_periods, ranging_pull_in_code_periods)
    check_code_periods(:out_of_lock_code_periods, out_of_lock_code_periods; positive = true)
    check_code_periods(
        :pull_in_out_of_lock_code_periods,
        pull_in_out_of_lock_code_periods;
        positive = true,
    )
    LockDwell(
        0.0s,
        0.0s,
        0.0s,
        false,
        false,
        warm_up_code_periods * T,
        pull_in_code_periods * T,
        confirm_code_periods * T,
        max(ranging_confirm_code_periods, confirm_code_periods) * T,
        max(ranging_pull_in_code_periods, ranging_confirm_code_periods) * T,
        pull_in_out_of_lock_code_periods * T,
        out_of_lock_code_periods * T,
        resets_on_lock,
    )
end

# The out-of-lock time currently tolerated: generous while pulling in, tighter once the
# handover transient has latched closed.
current_out_of_lock_time_threshold(dwell::LockDwell) =
    dwell.pulled_in ? dwell.out_of_lock_time_threshold :
    dwell.pull_in_out_of_lock_time_threshold

current_out_of_lock_time_threshold(lock_detector::AbstractLockDetector) =
    current_out_of_lock_time_threshold(lock_detector.dwell)

"""
    update(dwell::LockDwell, is_out_of_lock, signal_duration)

Advance the dwell by `signal_duration` of signal, crediting `is_out_of_lock` as this chunk's
verdict from the detector's statistic.
"""
function update(dwell::LockDwell, is_out_of_lock::Bool, signal_duration)
    out_of_lock_time = dwell.out_of_lock_time
    settled_time = dwell.settled_time
    # Compared before advancing `elapsed`, so the warm-up covers exactly
    # `warm_up_time_threshold` of signal rather than one chunk less.
    if dwell.elapsed >= dwell.warm_up_time_threshold
        if is_out_of_lock
            # Capped at a small multiple of what the *current stage* tolerates, not of the
            # steady-state threshold: capping at twice the latter would silently truncate the
            # pull-in allowance to two thirds of itself. The stage transition needs no special
            # handling, because both latches require the clock to be under the steady-state
            # threshold and so under the new stage's cap as well.
            out_of_lock_time = min(
                out_of_lock_time + signal_duration,
                OUT_OF_LOCK_TIME_CAP_FACTOR * current_out_of_lock_time_threshold(dwell),
            )
            settled_time = 0.0s
        else
            settled_time += signal_duration
            # Clamped at zero rather than merely guarded above it: paying back a chunk longer
            # than what is left on the clock would otherwise leave a residue of a few times
            # `eps` — enough that the `iszero` test below never fires and the detector never
            # latches out of its pull-in stage.
            out_of_lock_time =
                dwell.resets_on_lock ? 0.0s :
                max(zero(out_of_lock_time), out_of_lock_time - signal_duration)
        end
    end
    elapsed = dwell.elapsed + signal_duration
    # The handover is over once the satellite has demonstrably converged, or — as a backstop
    # for one that never quite settles — once its pull-in window has expired. Both require the
    # out-of-lock clock to be within what steady state will tolerate, so the tighter threshold
    # taking over cannot itself declare the satellite lost.
    pulled_in =
        dwell.pulled_in || (
            out_of_lock_time < dwell.out_of_lock_time_threshold && (
                (iszero(out_of_lock_time) && settled_time >= dwell.confirm_time_threshold) ||
                elapsed >= dwell.pull_in_time_threshold
            )
        )
    # A separate, longer bar than `pulled_in`: the dwell may tighten as soon as the C/N0
    # deficit is gone, but a pseudorange is only trustworthy once the code loop has settled.
    # Two conditions, for the same reasons as `pulled_in`'s two — evidence when there is any,
    # and a timer for the satellite that never strings a clean run together. Without the timer
    # this bar is one a marginal satellite can fail forever while staying comfortably in lock,
    # which excludes it from PVT silently and permanently.
    #
    # The timer path takes the freshly computed `pulled_in` as a conjunct rather than relying
    # on the thresholds being ordered, so "ranging is never ready before the handover is over"
    # holds structurally however the two timers are configured.
    ranging_ready =
        dwell.ranging_ready ||
        (iszero(out_of_lock_time) && settled_time >= dwell.ranging_confirm_time_threshold) ||
        (
            pulled_in &&
            out_of_lock_time < dwell.out_of_lock_time_threshold &&
            elapsed >= dwell.ranging_pull_in_time_threshold
        )
    LockDwell(
        elapsed,
        out_of_lock_time,
        settled_time,
        pulled_in,
        ranging_ready,
        dwell.warm_up_time_threshold,
        dwell.pull_in_time_threshold,
        dwell.confirm_time_threshold,
        dwell.ranging_confirm_time_threshold,
        dwell.ranging_pull_in_time_threshold,
        dwell.pull_in_out_of_lock_time_threshold,
        dwell.out_of_lock_time_threshold,
        dwell.resets_on_lock,
    )
end

is_in_lock(dwell::LockDwell) =
    dwell.out_of_lock_time < current_out_of_lock_time_threshold(dwell)

"""
    has_pulled_in(lock_detector::AbstractLockDetector)

Whether the detector has left its pull-in stage — the acquisition → tracking handover is over
and the satellite is being held to steady-state standards (see [`LockDwell`](@ref)).

This is about the *dwell*, not about measurement quality: it fires as soon as the C/N0 deficit
of the handover transient has cleared, which on a BPSK signal happens well before the code loop
has settled. Use [`is_ranging_ready`](@ref) to decide whether to range on the satellite.

The receiver itself gates nothing on this; it is exposed for introspection and diagnostics —
"which stage is this channel in?" — while [`is_ranging_ready`](@ref) is what actually gates
ranging (see `collect_pvt_sat_states!`) and [`is_in_lock`](@ref) what gates tracking.
"""
has_pulled_in(dwell::LockDwell) = dwell.pulled_in
has_pulled_in(lock_detector::AbstractLockDetector) = has_pulled_in(lock_detector.dwell)

"""
    is_ranging_ready(lock_detector::AbstractLockDetector)

Whether the tracking loops have settled enough for this satellite's *measurements* to be
trustworthy — the question a pseudorange consumer has to ask.

[`is_in_lock`](@ref) answers "should this satellite still be tracked?" and is deliberately
tolerant through the handover, because the point is to keep a converging satellite rather than
drop and reacquire it. But while the loops are still walking the coarse acquisition estimate
in, the code phase — and so the pseudorange — carries a converging bias, measured at tens of
metres. This latches after `ranging_confirm_time_threshold` of uninterrupted good evidence or,
for a satellite whose evidence is never uninterrupted, once `ranging_pull_in_time_threshold` of
signal has elapsed with the out-of-lock clock inside what steady state tolerates. It never
un-latches: a satellite whose loops settled once has settled, and later disturbances are
[`is_in_lock`](@ref)'s business — which also means a vector-tracking member carried through an
outage is immediately rangeable again on return, as it should be, since the navigation filter
kept both its NCOs steered throughout.

The timer is what keeps a marginal-but-tracked satellite from being excluded from PVT forever;
see [`LockDwell`](@ref) for the measurements behind both thresholds, and for what the timer
concedes in exchange (a few metres of extra code-phase jitter, not a handover bias).
"""
is_ranging_ready(dwell::LockDwell) = dwell.ranging_ready
is_ranging_ready(lock_detector::AbstractLockDetector) = is_ranging_ready(lock_detector.dwell)

"""
    CodeLockDetector <: AbstractLockDetector

Declares code lock from the estimated carrier-to-noise density ratio. After a warm-up it
accumulates out-of-lock time whenever the CN0 is below `cn0_threshold` (and pays it back down
otherwise); lock is lost once the accumulated out-of-lock time reaches the threshold its
[`LockDwell`](@ref) stage allows — generous while the loops pull the acquisition handover in,
tighter in steady state. Every timing is a count of `reference_integration_time`; the dwell's
keywords are accepted here and forwarded to it.

The accumulated time is capped at a small multiple of what the current stage tolerates. Since
it is paid back at the rate it is spent, an uncapped dwell would make re-declaring lock take as
long as the outage that preceded it — minutes, for a satellite kept in tracking through a
tunnel by the vector-tracking loop. With the cap, one stage threshold of good signal always
brings a satellite back.

The comparison is made on the *post-integration* SNR, `CN0 · T`, rather than on CN0
alone. `T` is the integration time of the record the CN0 estimate came from
(Tracking's `get_last_fully_integrated_integration_time`), and `cn0_threshold` is the CN0
demanded over `reference_integration_time` — one primary code period. A record spanning
`N` code blocks therefore clears the detector at `10·log10(N)` dB less CN0, which is what
the longer coherent integration genuinely buys: detectability is set by the
post-integration SNR, not by CN0 on its own.

Anchoring to the primary code period rather than to the symbol period keeps
`cn0_threshold` meaning exactly what it meant for one-block records (the default
tracking cadence), and needs no data rate — so it is also defined for pilot signals,
where `get_data_frequency` is 0 Hz.

`coherence_limit` caps the credited `T`, and defaults to `Inf·s` — uncapped — because
Tracking already bounds a record at the coherent limit itself: post-sync it integrates
`clamp(preferred_num_code_blocks_to_integrate, 1, blocks_per_symbol)` blocks, where a
symbol is the data-bit period, or the secondary-code period for a pilot. That bound is
generous for a long overlay (GPS L1C-P's is 1800 blocks), so set `coherence_limit` when
crediting a full symbol is more than the loop should be trusted with.

Note that Tracking's `preferred_num_code_blocks_to_integrate` defaults to 1 and this
receiver never raises it, so every record is currently one code block and the credit is
exactly `cn0_threshold` — the `N > 1` behaviour above only engages once that is plumbed
through.

The estimate itself comes from Tracking's default CN0 estimator, which as of Tracking 7 is
the measured noise reference (`NoiseRefCN0Estimator`): each record's prompt power is divided
by a noise density Tracking measures by despreading the signal's own code at a randomised
phase, through the same correlator the satellites go through. `cn0_threshold` is unchanged
in units and meaning, but the number it is compared against is a different — and better
behaved — statistic:

  - it has no noise-only floor to clear. The moment ratio manufactured signal power out of
    noise, reporting a median of ~27.6 dB-Hz on pure noise, so a threshold below ~30 dB-Hz
    could never trip and one at 30 dB-Hz was cleared by noise on ~19 % of records. The
    noise reference's per-record terms average to about zero there — `-Inf dB-Hz` on about
    half the realizations and single digits on the rest — so the out-of-lock dwell is spent
    monotonically: a satellite that dies drops after exactly `out_of_lock_time_threshold`,
    rather than the ~1.3× the detector used to take paying time back on the noise
    realizations that cleared the threshold;
  - it reads the true CN0 rather than a biased one, because the floor is measured instead of
    inferred from the same prompts. In the synthetic model the tests drive (a perfectly
    phase-locked prompt in unit-power noise) it lands within 0.1 dB of the truth from
    25 dB-Hz up, where the moment ratio reads 2.9 dB high at 25 dB-Hz. On the ION reference
    recording it reads a median 1.0 dB above what the narrowband/wideband ratio (Tracking
    6's default) reported, and up to 2.4 dB on the least steadily tracked satellite: that
    estimator summed ~5 ms of prompts coherently and charged the loop's residual phase noise
    to the signal, and this one does not;
  - `T` is applied per record, as the record is folded, rather than when the estimate is
    read. That changes nothing for the detector — it multiplies the record's own `T` back in
    either way — but it does mean `estimate_cn0` reports the same true CN0 whatever the
    record length, instead of a number that has to be read together with one.

An unmeasured channel — a `TrackState` with no noise source for the signal, or one whose
window is still empty — reports `-Inf dB-Hz`, which needs no special handling in the
threshold test (it linearizes to `0 Hz`). The receiver provisions a noise source for every
signal it tracks (`GNSSReceiver.create_noise_estimators`), so that is the transient before
the first measurement rather than a configuration a user can land in. At four or more
antennas the same `-Inf` can extend over the first chunk or two, while the array's spatial
noise covariance is still rank-deficient in its own dimensions — and only for a receiver
fed buffers of about one code period, since a longer chunk clears it within its first call.
Either way the detector's own warm-up (`warm_up_code_periods`, 80 code periods) is orders of
magnitude longer — on every signal, since it scales with the code period too — so it never
accumulates against it.
"""
struct CodeLockDetector <: AbstractLockDetector
    cn0_threshold::typeof(1.0dBHz)
    reference_integration_time::typeof(1.0s)
    coherence_limit::typeof(1.0s)
    dwell::LockDwell
end

function CodeLockDetector(;
    cn0_threshold = 30dBHz,
    # GPS L1 C/A's code period. The receiver passes the ranging signal's own period —
    # see `primary_code_period` and the `ReceiverSatState` constructors.
    reference_integration_time = 1ms,
    coherence_limit = Inf * s,
    dwell_kwargs...,
)
    CodeLockDetector(
        cn0_threshold,
        # `reference_integration_time` may arrive as `Tracking`'s `Hz^-1`; normalise once so
        # every stored duration is in seconds.
        code_period_reference(reference_integration_time),
        coherence_limit,
        LockDwell(; reference_integration_time, dwell_kwargs...),
    )
end

# One primary code period, normalised to seconds. Every detector timing is a multiple of it.
function code_period_reference(reference_integration_time)
    T = uconvert(s, reference_integration_time)
    T > zero(T) && isfinite(ustrip(T)) || throw(
        ArgumentError(
            "reference_integration_time must be a finite, positive duration — it is one " *
            "primary code period of the signal the detector runs on — got $T",
        ),
    )
    T
end

# Reject a code-period count that cannot describe a timing before it is silently scaled into
# one: a negative count scales into a negative threshold, and `out_of_lock_time < threshold`
# would then be false from the first update, so the channel would be dead on arrival rather
# than loudly misconfigured. The dwells have to be *strictly* positive for the same reason —
# `is_in_lock` tests `<`, so a zero threshold is never satisfied. A warm-up may legitimately
# be zero. `NaN` fails both comparisons, so it is caught here too.
function check_code_periods(name::Symbol, value; positive::Bool = false)
    isfinite(value) && (positive ? value > 0 : value >= 0) || throw(
        ArgumentError(
            "$name must be a finite, $(positive ? "strictly positive" : "non-negative") " *
            "number of code periods, got $value",
        ),
    )
    value
end

# Post-integration SNR of the last fully integrated record against the SNR
# `cn0_threshold` demands over one `reference_integration_time`. Both sides are
# dimensionless (Hz · s), so `uconvert` makes the comparison unit-agnostic regardless of
# whether the caller's integration time arrives as `s` or as Tracking's `Hz^-1`.
#
# Written as a negated `>=` so that a non-finite CN0 counts as *out* of lock: the moments
# estimator degenerates to `NaN` for an all-zero prompt buffer, and `NaN < threshold` is
# `false` — which would otherwise hold a dead channel in lock indefinitely. Tracking's "no
# detectable signal" is `-Inf dB-Hz`, which needs no special handling: it linearizes to
# `0 Hz` and so fails the comparison against every finite threshold.
function is_below_cn0_threshold(lock_detector::CodeLockDetector, cn0, integration_time)
    credited = min(integration_time, lock_detector.coherence_limit)
    snr = uconvert(NoUnits, Unitful.linear(cn0) * credited)
    required = uconvert(
        NoUnits,
        Unitful.linear(lock_detector.cn0_threshold) *
        lock_detector.reference_integration_time,
    )
    !(snr >= required)
end

# How far above the current stage's threshold the accumulated out-of-lock time may run. The
# dwell is paid back one-for-one, so without a cap the time to *re*-declare lock is the full
# length of the outage that came before it: a satellite obscured for a minute would need a
# minute of clean signal to come back. That contradicts what the dwell is for — riding out
# brief fades — and it is what a vector-tracking member, which is kept in tracking through
# an outage instead of being dropped and reacquired (`remove_lost_satellites`), actually
# lives through. Capping at a small multiple of the threshold keeps the "consistently weak"
# verdict (the detector still sits well past the threshold) while bounding the recovery at
# `(cap - 1) · out_of_lock_time_threshold` of good signal.
const OUT_OF_LOCK_TIME_CAP_FACTOR = 2

function update(lock_detector::CodeLockDetector, cn0, integration_time, signal_duration)
    @set lock_detector.dwell = update(
        lock_detector.dwell,
        is_below_cn0_threshold(lock_detector, cn0, integration_time),
        signal_duration,
    )
end

"""
    is_in_lock(lock_detector::AbstractLockDetector)

Return `true` while the detector's accumulated out-of-lock time is below the threshold its
current stage allows (see [`LockDwell`](@ref)).
"""
is_in_lock(lock_detector::AbstractLockDetector) = is_in_lock(lock_detector.dwell)

"""
    set_out_of_lock(lock_detector::AbstractLockDetector)

Return a copy of the detector with its accumulated out-of-lock time raised to the threshold
its *current stage* allows, so [`is_in_lock`](@ref) immediately reports `false`. Used to force
a satellite out of lock, e.g. when vector tracking releases it for cause
(`force_out_of_lock`) — which silently no-ops unless the lock verdict flips on the spot, so
raising it to the fixed steady-state value would not do: a satellite still in its pull-in stage
tolerates three times that.

`pulled_in` is deliberately left alone. A released satellite is removed from tracking on the
next chunk and reacquired with a fresh detector, so demoting the flag buys nothing and would
muddy its meaning.
"""
set_out_of_lock(dwell::LockDwell) =
    @set dwell.out_of_lock_time = current_out_of_lock_time_threshold(dwell)
set_out_of_lock(lock_detector::AbstractLockDetector) =
    @set lock_detector.dwell = set_out_of_lock(lock_detector.dwell)

"""
    CarrierLockDetector <: AbstractLockDetector

Declares carrier lock from the prompt correlator using the Van Dierendonck
narrowband-difference / narrowband-power phase-lock indicator

```
PLI = Σ(Iₖ² − Qₖ²) / Σ(Iₖ² + Qₖ²)
```

accumulated over every prompt the tracking loops produce. For a prompt of amplitude `A`
in noise of total variance `σ²` — post-integration SNR `ρ = A²/σ² = CN0·T` — at carrier
phase error `φ`,

```
E[Iₖ² − Qₖ²] = A²·cos(2φ)      E[Iₖ² + Qₖ²] = A² + σ²
```

so `E[PLI] = cos(2φ)·ρ/(ρ+1)`. Lock is declared while `PLI ≥ phase_lock_threshold`; its
[`LockDwell`](@ref) clears outright on any favourable chunk (`resets_on_lock`), so declaring
loss needs a *consecutive* run of failures. Its default `out_of_lock_code_periods` of 4000 is
4 s on GPS L1 C/A, matching both this receiver's historical value and Ward's
`L0 = 240 × 20 ms = 4.8 s`. Because `pull_in_out_of_lock_code_periods` is floored at that,
both stages are equal and the carrier detector is deliberately unstaged — its dwell already
asks for a consecutive run of failures far longer than any handover transient.

This replaces a low-pass-filtered `|I|`-versus-`|Q|` amplitude comparison
(`lowpass(|I|)/K2 > lowpass(|Q|)` with `K1 = 0.0247`, `K2 = 1.5` — the detector of
Kaplan & Hegarty §5.11.2). Three concrete reasons:

  * **It sees every prompt.** The old detector was fed one prompt per processing chunk,
    discarding three of every four at the receiver's 4 ms chunk over a 1 ms code period.
  * **Its averaging is continuous.** The old filter state was reset every 80 ms block, so
    the value actually tested had only reached ~39% of its asymptote — `K1 = 0.0247` is
    specified for 20 ms epochs and gives a 40-epoch time constant, which the 20-update
    block truncated.
  * **Its expectation is known in closed form**, so a threshold maps to an explicit phase
    error at a given C/N0. The old comparison only approaches a phase test at high SNR;
    at low SNR it degenerates into an implicit SNR test (at perfect phase lock it flips at
    `ρ ≈ 0.54`, i.e. ~27.4 dBHz over a 1 ms record), and that crossover is a consequence of
    `K2` rather than something you can read off it.

`phase_lock_threshold` defaults to `0.25`, matching PocketSDR's `THRES_PLI`. Because the
indicator saturates at `ρ/(ρ+1)`, a threshold has to be read together with the C/N0 one: at
the default 30 dBHz over a 1 ms record `ρ = 1`, so even perfect phase lock reads only `0.5`,
and `0.25` demands `cos(2φ) ≥ 0.5` — `|φ| ≤ 30°`, comparable to the 33.7° `K2 = 1.5`
implied. A threshold above `0.5` would be unreachable at the C/N0 the code detector still
accepts. Read as a pure sensitivity floor the default is `ρ ≥ 1/3`, i.e. 25.2 dBHz at a 1 ms
record, so — as before — the code detector's 30 dBHz stays the binding limit and the carrier
detector is left to judge phase.

The two sums are tracked as exponential moving averages sharing one gain, so the indicator
is continuous. `smoothing_records` sets the averaging window; the default of 200 keeps the
noise-only indicator's median at 0.00 and its 99th percentile at 0.12 — both well clear of
the 0.25 threshold — while a 36 dBHz signal reads 0.78 against its 0.80 asymptote.

The window is counted in **records, not code periods**, unlike the two timings: the gain is
applied once per prompt, and a prompt is one record spanning
`get_last_fully_integrated_num_code_blocks` code blocks. The timings stay honest in code
periods because they are driven by `signal_duration`, which is real elapsed time; this
average has no access to that. The two coincide today only because Tracking's
`preferred_num_code_blocks_to_integrate` is 1 and this receiver never raises it (see
[`CodeLockDetector`](@ref)) — once that is plumbed through, a 200-record window will be
200·N code periods long.

Those noise-only figures are **asymptotic**, and the 80-code-period warm-up is shorter than
the 200-record window, so the first judgement is made on an average that has not converged.
Measured over 4000 noise realizations at the default window, the p99 settles at 0.12 but is
0.17 at 200 records, 0.26 at 80 (max 0.38) and 0.35 at 40 — i.e. at the moment the warm-up
ends a noise-only channel is above threshold 1.2% of the time. The warm-up is deliberately
*not* lengthened to match: clearing the dwell on any favourable chunk, together with the
4000-code-period dwell, means declaring loss needs a consecutive run of failures, so an
early excursion in either direction is absorbed. In the other direction the early window is
already tight enough: at the code-lock threshold (`ρ = 1`, perfect phase lock) the indicator
is below 0.25 for 0.1% of realizations at 80 records and none at 200.
"""
struct CarrierLockDetector <: AbstractLockDetector
    # Exponential moving averages of I² − Q² and I² + Q², sharing `smoothing_gain`. The gain
    # is applied once per *record*, so — unlike the two timings below — the window it implies
    # is a count of records rather than of code periods.
    filtered_coherent_power::Float64
    filtered_total_power::Float64
    smoothing_gain::Float64
    phase_lock_threshold::Float64
    dwell::LockDwell
end

function CarrierLockDetector(;
    reference_integration_time = 1ms,
    phase_lock_threshold = 0.25,
    smoothing_records = 200,
    out_of_lock_code_periods = 4000,
    dwell_kwargs...,
)
    # A window shorter than one record gives a gain above 1, which makes the "moving average"
    # overshoot every sample; a window of 0 gives an infinite gain, which turns both averages
    # into `NaN` on the first prompt. `is_below_phase_lock_threshold` reads a non-finite
    # average as out of lock — correctly, for a dead correlator — so the channel would then be
    # silently dead from construction rather than loudly misconfigured.
    isfinite(smoothing_records) && smoothing_records >= 1 || throw(
        ArgumentError(
            "smoothing_records is the averaging window in records and sets the moving " *
            "averages' gain to 1/smoothing_records, so it must be a finite number ≥ 1, " *
            "got $smoothing_records",
        ),
    )
    # The indicator is `cos(2φ)·ρ/(ρ+1)`, so it lives in `[-1, 1]`: a threshold at or above 1
    # is unreachable at any C/N0 and holds the channel permanently out of lock, one at or
    # below −1 is met by anything and never reports loss. In practice the ceiling is lower
    # still — the indicator saturates at `ρ/(ρ+1)`, which is 0.5 at the 30 dBHz the code
    # detector accepts over a one-block record — but that bound moves with the record length,
    # so only the mathematical one is enforced.
    isfinite(phase_lock_threshold) && -1 < phase_lock_threshold < 1 || throw(
        ArgumentError(
            "phase_lock_threshold is compared against cos(2φ)·ρ/(ρ+1) ∈ [-1, 1], so it " *
            "must be strictly inside that range (and below ρ/(ρ+1) ≈ 0.5 at the code " *
            "detector's own 30 dBHz over a one-block record to be reachable at all), got " *
            "$phase_lock_threshold",
        ),
    )
    CarrierLockDetector(
        0.0,
        0.0,
        1 / smoothing_records,
        phase_lock_threshold,
        LockDwell(;
            reference_integration_time,
            out_of_lock_code_periods,
            resets_on_lock = true,
            dwell_kwargs...,
        ),
    )
end

"""
    phase_lock_indicator(lock_detector::CarrierLockDetector)

The smoothed phase-lock indicator, or `NaN` before any prompt has been folded in.
"""
phase_lock_indicator(lock_detector::CarrierLockDetector) =
    lock_detector.filtered_total_power > 0 ?
    lock_detector.filtered_coherent_power / lock_detector.filtered_total_power : NaN

function is_below_phase_lock_threshold(lock_detector::CarrierLockDetector)
    # A non-finite moving average means a non-finite prompt reached the detector — a dead or
    # saturated correlator channel. That has to fail *closed*, exactly as a non-finite CN0
    # does in `is_below_cn0_threshold`: `NaN` poisons both averages permanently, and since
    # `NaN > 0` is `false` it would otherwise look like "nothing measured yet" forever and
    # hold the channel in lock indefinitely.
    (
        isfinite(lock_detector.filtered_total_power) &&
        isfinite(lock_detector.filtered_coherent_power)
    ) || return true
    # Genuinely nothing folded in yet: the warm-up owns that window, so not out of lock.
    lock_detector.filtered_total_power > 0 || return false
    # Negated `>=` so any remaining non-finite indicator counts as out of lock too.
    !(phase_lock_indicator(lock_detector) >= lock_detector.phase_lock_threshold)
end

"""
    update(lock_detector::CarrierLockDetector, prompts, signal_duration)

Fold every prompt of one processing chunk into the indicator, then advance the dwell once by
`signal_duration`.

`prompts` is the whole sequence of filtered prompt correlator values the chunk produced
(Tracking's `get_filtered_prompts`), not just the last one: at the receiver's default 4 ms
chunk over a 1 ms code period, reading only `get_last_fully_integrated_filtered_prompt`
would discard three of every four. A lone prompt is accepted too, for callers that have
only one. An empty sequence — a chunk too short to complete a record — advances the timers
without inventing evidence.
"""
function update(lock_detector::CarrierLockDetector, prompts, signal_duration)
    coherent = lock_detector.filtered_coherent_power
    total = lock_detector.filtered_total_power
    K = lock_detector.smoothing_gain
    for prompt in prompts
        inphase_power, quadrature_power = real(prompt)^2, imag(prompt)^2
        coherent += K * (inphase_power - quadrature_power - coherent)
        total += K * (inphase_power + quadrature_power - total)
    end
    # The verdict is read off the *updated* averages, so a chunk is judged on everything it
    # contributed rather than on the state that preceded it.
    measured = CarrierLockDetector(
        coherent,
        total,
        K,
        lock_detector.phase_lock_threshold,
        lock_detector.dwell,
    )
    @set measured.dwell =
        update(lock_detector.dwell, is_below_phase_lock_threshold(measured), signal_duration)
end

update(lock_detector::CarrierLockDetector, prompt::Complex, signal_duration) =
    update(lock_detector, (prompt,), signal_duration)
