"""
    AbstractLockDetector

Supertype for the receiver's per-satellite lock detectors. A concrete detector is
advanced with `update` and queried with [`is_in_lock`](@ref); a satellite is treated
as locked only while both its code and carrier detectors report lock.

The detectors track elapsed signal time rather than a number of `update` calls, so
their behaviour is independent of how the incoming signal is chunked: each `update`
is handed the `signal_duration` it represents and accumulates that duration into its
timers.

## Which timings are code periods, and which are seconds

The detectors' timings answer two different questions, and they scale differently.

**Evidence counts** — the warm-up, the clean run that confirms the handover is over, and the
steady-state out-of-lock dwells — measure how much of a detector's own statistic it has
seen. Both statistics are built record by record (the C/N0 of each record; the phase-lock
indicator's averages, which TrackingLoops advances once per record), and a record is one
primary code period `T`, so these are counts of `T`: the same count then means the same
amount of evidence on every signal, and the GPS L1 C/A values are reproduced exactly
(`T = 1 ms`, so 200 code periods *is* 200 ms). In seconds they would not: the 200 ms dwell
this receiver once allowed was only 50 records on Galileo E1B and 20 on GPS L1C-D, which is why
lock detection failed on the slower signals while GPS L1 C/A looked fine.

**Handover settling** — how long the pull-in stage lasts, how much out-of-lock time it
tolerates, and when a satellite may be ranged on — waits for the tracking loops to walk the
acquisition residual in. TrackingLoops sets those loops' bandwidths in hertz, capped against
the integration time, not as a multiple of `1/T`; and the code loop, which ranging waits for,
runs at 1 Hz on every signal here, so its time constant is one second whatever the code
period:

| signal | `T` | carrier `B_L`, pull-in → locked | code `B_L` | `1/B_L` code |
|---|---|---|---|---|
| GPS L1 C/A | 1 ms | 50 Hz → 18 Hz | 1 Hz | 1 s |
| Galileo E1B | 4 ms | 22.5 Hz → 18 Hz | 1 Hz | 1 s |
| GPS L1C-D | 10 ms | 9 Hz → 9 Hz | 1 Hz | 1 s |

So these timings are durations. Counted in code periods, as they were while TrackingLoops
sized every bandwidth at `B_L·T ≈ 0.018`, they made a Galileo E1B satellite wait 2.7 s and a
GPS L1C-D one 6.8 s before ranging, long after its code loop had settled.

Every detector takes a `reference_integration_time` — one primary code period of the signal it
runs on, which the receiver passes from `primary_code_period` — for its evidence counts. The
fields themselves are all in seconds.
"""
abstract type AbstractLockDetector end

"""
    LockDwell

The out-of-lock dwell shared by the receiver's lock detectors: a timer that accumulates
while the detector's statistic reports "no usable signal" and declares lock lost once it
reaches a threshold. Both [`CodeLockDetector`](@ref) and [`CarrierLockDetector`](@ref) hold
one, so the handover staging below is defined in exactly one place. Its evidence counts are
counts of one primary code period and its handover timings are durations — see
[`AbstractLockDetector`](@ref) for why.

## Why there are two stages

The acquisition → tracking handover is deliberately coarse: the acquisition Doppler bin is
sized to half the loop's pull-in range (`receive`'s `pull_in_margin`), and the code phase is
only resolved to the sample grid, halved by `subsample_interpolation`. The loops need time to
pull those errors in, and until they have, the prompt correlator sits off the correlation peak
and carries less power than the satellite really has — so a *healthy* satellite reads below
its true C/N0, and its carrier phase is still spinning at the residual Doppler, so the
phase-lock indicator reads near zero.

Measured on synthetic handovers at this receiver's own worst-case residual — half an
acquisition bin of Doppler (125 Hz on GPS L1 C/A, 31 Hz on Galileo E1B, 12.5 Hz on GPS L1C-D)
and half a sample of code phase (0.25, 0.042 and 0.062 chip at 2.048, 12.288 and 8.192 MHz) —
under TrackingLoops 5's staged carrier loop, the code detector's peak accumulated out-of-lock
time is small for a satellite with margin: at most 28 ms on GPS L1 C/A from 34 dBHz up, none
on Galileo E1B from 31 dBHz up, and none on GPS L1C-D from 32 dBHz up. Nearer the 30 dBHz
threshold it grows — 0.51 s on GPS L1C-D at 31 dBHz, and up to 320 ms on GPS L1 C/A at 32 dBHz
for the satellites the loop holds at all (its wide, FLL-assisted pull-in loses about half of
those handovers outright) — which is more than the 200 ms steady-state dwell GPS L1 C/A gets,
so a single un-staged dwell would drop them. The carrier detector's demand is larger, up to
1.6 s on GPS L1 C/A at 34 dBHz and 0.23 s at 38, because the FLL has to pull in the residual
Doppler before `cos(2φ)` stops averaging to zero; on the longer codes it stays under 25 ms.

So the dwell has two stages:

  * **pull-in**, from construction: tolerates `pull_in_out_of_lock_time_threshold`, sized to
    survive the whole handover transient with margin over the measured worst case;
  * **steady state**, latched permanently once the handover is over: tolerates the tighter
    `out_of_lock_time_threshold`.

Two stages rather than one long dwell, because they answer different questions. A single dwell
would have to be ≳600 ms on GPS L1 C/A to be safe, and would then take three times as long to
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
allowances — 200 to 600 ms on GPS L1 C/A — never leaves the pull-in stage, and so never becomes
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
GPS L1 C/A satellite reads healthy almost immediately while its DLL (`1/B_L` = 1 s) is still
walking the handover error in. Measured at the same 0.25-chip handover, the residual code
phase at 34 to 38 dBHz is 0.16 to 0.20 chip after 200 ms — **47 to 59 m of pseudorange
error** — falling to 0.02 to 0.11 chip, 6 to 32 m, by 600 ms. So there are two latches over
the same `settled_time`: `confirm_time_threshold` (200 code periods) tightens the dwell, and
`ranging_confirm_time_threshold` (600 ms) admits the satellite to ranging via
[`is_ranging_ready`](@ref). It is floored at the confirm run, since ranging can never be ready
before the handover is over — and on GPS L1C-D, whose 200 code periods are 2 s, that floor is
what binds.

The ranging bar is a duration because the DLL's time constant is one: its 1 Hz loop needs the
same second to walk a handover error in on every signal, and the error it has to walk is
smaller on the longer codes, a sample there being a smaller fraction of a chip. On Galileo E1B
the residual is under 0.008 chip from 0.5 s on, and on GPS L1C-D under 0.03 chip, so the
600 ms bar costs those signals nothing in accuracy — counted in code periods it held them back
to 2.7 s and 6.8 s.

The residual does not decay monotonically on GPS L1 C/A. While the carrier loop pulls its
Doppler in, the error reaches the code loop through carrier aiding, so the code phase
overshoots once the carrier has settled — 0.07 to 0.11 chip (20 to 32 m) at 2 s at 34 dBHz,
0.02 to 0.03 chip at 45 dBHz — and stays within 0.02 chip only after 2 to 5 s. The bar is
not tuned against that swing, which the loops leave behind whatever the gate.

Being conservative is close to free: the earliest the bar can fire is the warm-up plus one
clean run, 680 ms on GPS L1 C/A, and measured on synthetic handovers it lands at 0.8 to 2 s
there — the carrier loop has its own residual Doppler to pull in before the phase-lock
indicator stops resetting the run — so still inside `receive`'s own 2 s
`time_in_lock_before_calculating_pvt` gate. On slower signals the extra time is dwarfed by
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
blocks it: its pull-in transient — up to 1.6 s on GPS L1 C/A at 34 dBHz — resets
`settled_time` well after the code detector is happy, and any recurring disturbance shorter
than the run reopens the same hole at any C/N0. So `ranging_pull_in_time_threshold` (2 s)
backstops it; on GPS L1 C/A at 34 dBHz it is what admits two satellites in ten. It
also takes `pulled_in` as a conjunct, so "ranging is never ready before the handover is over"
holds structurally rather than by threshold ordering.

What the backstop relaxes is the *length* of the clean run, not the requirement that the
channel be clean at all: it fires only with the out-of-lock clock paid down to **zero**. A
weaker guard — "the clock is within what steady state tolerates" — reads like the one
`pulled_in`'s timer uses, but the two are not the same test. `pulled_in`'s guard exists to
stop the *threshold switch itself* from declaring loss, so it is only ever consulted where the
pull-in allowance exceeds the steady-state one. A health guard has to be reachable instead,
and the tolerance form is not: the clock can never exceed `elapsed - warm_up_time_threshold`,
so wherever the dwell outlasts the backstop it cannot fail before the timer fires. The carrier
detector is exactly that case — a 4000-code-period dwell, 4 s even on GPS L1 C/A, against a
2 s backstop — and under the tolerance form a channel whose phase-lock indicator had never once
passed would be admitted to ranging at 2 s, then held there by `resets_on_lock` on one
favourable chunk in a thousand. Requiring zero is reachable for both detectors (a single
favourable chunk clears it under `resets_on_lock`, a repaid debt otherwise), cannot be made
vacuous by any configuration, and keeps the deterministic case the backstop exists for: a
disturbance recurring faster than the clean run still leaves the clock at zero between
disturbances, whereas a channel failing steadily enough never to pay its debt off is dropped
by `is_in_lock` rather than admitted here.

Two code-loop time constants (`1/B_L` = 1 s) rather than anything shorter,
because the backstop admits a measurement it has not seen clean evidence for, and what the
DLL's convergence depends on is *elapsed* time, not clean-run length. What it concedes is
thermal jitter on a marginal satellite's code phase, not a handover bias: for the default 1 Hz
code loop and Tracking's 1-chip early-late spacing, Kaplan & Hegarty's coherent-DLL expression
gives 1σ = 0.039 chip (11 m) at 30 dBHz and 0.027 chip (8 m) at 32, against 0.004 chip (1.2 m)
at 45 — inside the same band the evidence path already accepts. The timer is 2 s on every
signal, exactly `receive`'s default `time_in_lock_before_calculating_pvt`, so it costs nothing
there — except on a code whose warm-up and confirm run are longer, GPS L1C-D's 0.8 s and 2 s,
where it is floored at their 2.8 s.

Before `warm_up_time_threshold` no evidence is accumulated at all. Under the noise-reference
estimator an unmeasured channel reads `-Inf dBHz` and so *fails* the CN0 test rather than
passing it, so the warm-up is not there to hide an estimator's cold bias — it is there for the
carrier detector, whose indicator TrackingLoops only reports once its averages span 0.1 s (see
[`CarrierLockDetector`](@ref)).

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

Build a dwell whose evidence counts — `warm_up_code_periods`, `confirm_code_periods` and
`out_of_lock_code_periods` — are multiples of `reference_integration_time`, one primary code
period of the signal the detector runs on, and whose handover timings — `pull_in_time`,
`pull_in_out_of_lock_time`, `ranging_confirm_time` and `ranging_pull_in_time` — are durations
(see [`AbstractLockDetector`](@ref) for why the two differ).

The defaults are sized from measured handover transients (see the type's docstring) and, for a
1 ms code period, reproduce this receiver's historical absolute timings: an 80 ms warm-up, a
200 ms steady-state dwell, a 600 ms pull-in allowance over an 800 ms window, and ranging after
600 ms of clean evidence or a 2 s backstop.

Three floors are applied, in the constructor body rather than in the keyword defaults, so
they hold for every caller rather than only for one that leaves the keyword alone:

  * `pull_in_out_of_lock_time` is floored at the steady-state dwell, so the pull-in stage can
    never be *stricter* than steady state however either is set — otherwise `pulled_in`
    latching would *loosen* the detector and a freshly acquired satellite would be held to a
    tighter dwell than a converged one. A detector whose steady-state dwell already exceeds the
    pull-in allowance — the carrier one, at 4000 code periods, and the code one on a code of
    3 ms or more — is therefore effectively unstaged, which is correct: its dwell is already
    longer than the handover transient it would otherwise have to tolerate;
  * `ranging_confirm_time` is floored at the confirm run, since ranging can never be ready
    before the handover is declared over;
  * `ranging_pull_in_time` is floored at the warm-up plus that *effective* ranging-confirm
    time — the already-floored one, not the raw keyword — for the mirror-image reason: a
    backstop that expired before the evidence path could fire would replace the criterion
    rather than back it up. The evidence path cannot fire before the warm-up is over, so the
    floor includes it, and flooring against the raw value would miss exactly the case where
    `confirm_code_periods` is what raised the evidence bar. On GPS L1C-D it is this floor,
    2.8 s, rather than the 2 s default that sets the backstop.
"""
function LockDwell(;
    reference_integration_time,
    warm_up_code_periods = 80,
    confirm_code_periods = 200,
    out_of_lock_code_periods = 200,
    pull_in_time = 800ms,
    pull_in_out_of_lock_time = 600ms,
    ranging_confirm_time = 600ms,
    ranging_pull_in_time = 2s,
    resets_on_lock = false,
)
    T = code_period_reference(reference_integration_time)
    check_code_periods(:warm_up_code_periods, warm_up_code_periods)
    check_code_periods(:confirm_code_periods, confirm_code_periods)
    check_code_periods(:out_of_lock_code_periods, out_of_lock_code_periods; positive = true)
    pull_in_time = check_duration(:pull_in_time, pull_in_time)
    pull_in_out_of_lock_time =
        check_duration(:pull_in_out_of_lock_time, pull_in_out_of_lock_time; positive = true)
    ranging_confirm_time = check_duration(:ranging_confirm_time, ranging_confirm_time)
    ranging_pull_in_time = check_duration(:ranging_pull_in_time, ranging_pull_in_time)
    out_of_lock_time_threshold = out_of_lock_code_periods * T
    confirm_time_threshold = confirm_code_periods * T
    # The documented floors, applied here so they bind for every caller — a keyword default
    # only ever protects the caller who does not pass the keyword. Each one is floored against
    # the *effective* value of what it is ordered against, so the ordering is transitive: the
    # backstop follows the ranging-confirm bar that `confirm_code_periods` may itself have
    # raised, not the raw keyword. They compare a duration in seconds with a count of code
    # periods, so which side binds depends on the signal: on a 10 ms code the 200-code-period
    # dwells outlast the handover allowances.
    # The backstop is floored at the earliest the evidence path can fire, which is after the
    # warm-up as well as the clean run.
    warm_up_time_threshold = warm_up_code_periods * T
    staged_out_of_lock_time = max(pull_in_out_of_lock_time, out_of_lock_time_threshold)
    effective_ranging_confirm_time = max(ranging_confirm_time, confirm_time_threshold)
    effective_ranging_pull_in_time =
        max(ranging_pull_in_time, warm_up_time_threshold + effective_ranging_confirm_time)
    LockDwell(
        0.0s,
        0.0s,
        0.0s,
        false,
        false,
        warm_up_time_threshold,
        pull_in_time,
        confirm_time_threshold,
        effective_ranging_confirm_time,
        effective_ranging_pull_in_time,
        staged_out_of_lock_time,
        out_of_lock_time_threshold,
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
    #
    # It also demands a *zero* out-of-lock clock, not merely one inside what steady state
    # tolerates: the clock can never exceed `elapsed - warm_up_time_threshold`, so the
    # tolerance form cannot fail before the timer fires wherever the dwell outlasts the
    # backstop — the carrier detector's 4000-code-period dwell against its 2 s backstop is
    # exactly that, and a channel whose indicator never once passed would be
    # admitted. What the backstop relaxes is the *length* of the clean run, not the
    # requirement that the channel be clean; see the type's docstring.
    ranging_ready =
        dwell.ranging_ready ||
        (iszero(out_of_lock_time) && settled_time >= dwell.ranging_confirm_time_threshold) ||
        (
            pulled_in &&
            iszero(out_of_lock_time) &&
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
signal has elapsed with nothing left on the out-of-lock clock — the backstop shortens the clean
run it asks for, it does not waive it. It never un-latches: a satellite whose loops settled
once has settled, and later disturbances are [`is_in_lock`](@ref)'s business — which also means a vector-tracking member carried through an
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
tighter in steady state. Its evidence counts are counts of `reference_integration_time` and
its handover timings are durations (see [`AbstractLockDetector`](@ref)); the dwell's keywords
are accepted here and forwarded to it.

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

# One primary code period, normalised to seconds. Every evidence count is a multiple of it.
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

# The handover timings' counterpart of `check_code_periods`, for the ones given as durations:
# normalised to seconds, and rejected when they cannot describe a timing for the same reasons.
function check_duration(name::Symbol, value; positive::Bool = false)
    t = uconvert(s, value)
    isfinite(ustrip(t)) && (positive ? t > zero(t) : t >= zero(t)) || throw(
        ArgumentError(
            "$name must be a finite, $(positive ? "strictly positive" : "non-negative") " *
            "duration, got $value",
        ),
    )
    t
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

Declares carrier lock when the satellite's carrier loop reports itself phase-locked
(`TrackingLoops.phase_locked`), read off its Doppler-estimator state once per processing
chunk. A scalar loop is phase-locked while the phase-lock indicator its carrier-loop staging
keeps (`TrackingLoops.phase_lock_indicator`) reads at least 0.5:

```
PLI = ⟨I² − Q²⟩ / A²,    A² = √(2·M₂² − M₄),    Mₖ = ⟨|P|ᵏ⟩
```

averaged exponentially over every prompt the loop produced, with a time constant of 0.1 s
and at least 25 records. `⟨I² − Q²⟩` is Van Dierendonck's narrowband difference, and for a
prompt of amplitude `A` at carrier phase error `φ` its expectation is `A²·cos(2φ)`. The signal
power it is normalised with comes from the prompt's second and fourth moments, from which the
noise cancels, so `E[PLI] ≈ cos(2φ)` whatever the C/N0: 1 in phase lock, 0 for a spinning
phase. Lock is declared while the loop is phase-locked; its [`LockDwell`](@ref) clears
outright on any favourable chunk (`resets_on_lock`), so declaring loss needs a *consecutive*
run of failures. Its default `out_of_lock_code_periods` of 4000 is 4 s on GPS L1 C/A,
matching both this receiver's historical value and Ward's `L0 = 240 × 20 ms = 4.8 s`. Because
`pull_in_out_of_lock_time` is floored at that, both stages are equal and the carrier detector
is deliberately unstaged — its dwell already asks for a consecutive run of failures far longer
than any handover transient.

This replaces the receiver's own `Σ(I² − Q²) / Σ(I² + Q²)` over a 200-record moving average of
Tracking's filtered prompts. The tracking loops already compute the indicator for their
staging, so reading theirs saves recomputing it from every prompt, and it is the better
statistic: normalised with the total power `A² + σ²`, the old one saturated at
`ρ/(ρ+1)` (`ρ = CN0·T`), 0.5 at 30 dBHz over a 1 ms record. Its threshold therefore had to sit
at 0.25 and mixed an SNR test into the phase test. Reading the loops' own indicator also means
the receiver's carrier lock and the staging's — which drops the FLL and narrows the loop on
the same indicator crossing the same 0.5 — answer the same question. A satellite in the
vector loop is phase-locked by the residual PLL's own decision instead, which also asks that
the C/N0 its rotated prompts read be no more than 6 dB below the vector loop's lock
threshold, since the indicator alone reads lock on bare noise now and then.

The threshold of 0.5 is an RMS phase error of about 30°. Measured on synthetic prompts at
one reading per 4 ms chunk, a phase-locked GPS L1 C/A signal reads below it 1% of the time
at 30 dBHz, 0.03% at 32 dBHz and never from 35 dBHz up; Galileo E1B and GPS L1C-D, averaging
over longer records, do better still. A phase spinning at 2 Hz reads above it 0.1% of the
time, and from 10 Hz up never.

The indicator is *not* a noise detector. On noise alone `A²` estimates zero, so the ratio is
heavy-tailed, and a noise-only channel reads above 0.5 on 3% of GPS L1 C/A chunks and 6% of
Galileo E1B records — enough that, under `resets_on_lock`, the carrier detector alone would
never declare it lost. It does not have to: the [`CodeLockDetector`](@ref)'s C/N0 test drops a
noise-only channel within its 200-code-period dwell, and this detector is left to judge phase
on a satellite whose power is there. The same holds for a carrier loop that has run away: on
synthetic GPS L1 C/A handovers at 30 and 32 dBHz whose Doppler ran off by kHz, this indicator
passed on 1 to 9% of chunks and so never completed the 4 s run, where the old one did from
4.1 s — but the code detector had dropped every one of them first, from 0.7 s.

The indicator is `NaN` until its averages span one time constant — 100 ms on GPS L1 C/A and
Galileo E1B, 250 ms on GPS L1C-D — and again for as long after a release from the vector
loop, which restarts the staging. TrackingLoops reports a `NaN` reading as not phase-locked,
as a non-finite indicator always must count (a dead correlator poisons the averages
permanently): GPS L1 C/A's 80 ms warm-up ends 20 ms before the first reading, which those 20
ms of the 4 s dwell absorb, and the longer codes' warm-ups outlast it. A chunk too short to
complete a record leaves the reading where it was, so the dwell repeats the previous verdict
rather than inventing one.
"""
struct CarrierLockDetector <: AbstractLockDetector
    # TrackingLoops' phase-lock indicator as of the latest chunk, `NaN` before the first.
    phase_lock_indicator::Float64
    # Whether the carrier loop reported itself phase-locked as of the latest chunk.
    phase_locked::Bool
    dwell::LockDwell
end

function CarrierLockDetector(;
    reference_integration_time = 1ms,
    out_of_lock_code_periods = 4000,
    dwell_kwargs...,
)
    CarrierLockDetector(
        NaN,
        false,
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

TrackingLoops' phase-lock indicator as of the latest chunk, or `NaN` before the first reading
(and while TrackingLoops' own averages are still filling).
"""
phase_lock_indicator(lock_detector::CarrierLockDetector) = lock_detector.phase_lock_indicator

is_out_of_phase_lock(lock_detector::CarrierLockDetector) = !lock_detector.phase_locked

"""
    update(
        lock_detector::CarrierLockDetector,
        phase_lock_indicator,
        phase_locked,
        signal_duration,
    )

Judge one processing chunk on TrackingLoops' `phase_locked` of the satellite's
Doppler-estimator state after it, keep its `phase_lock_indicator` for inspection, and
advance the dwell by `signal_duration`.
"""
function update(
    lock_detector::CarrierLockDetector,
    phase_lock_indicator::Real,
    phase_locked::Bool,
    signal_duration,
)
    CarrierLockDetector(
        Float64(phase_lock_indicator),
        phase_locked,
        update(lock_detector.dwell, !phase_locked, signal_duration),
    )
end
