"""
    AbstractLockDetector

Supertype for the receiver's per-satellite lock detectors. A concrete detector is
advanced with `update` and queried with [`is_in_lock`](@ref); a satellite is treated
as locked only while both its code and carrier detectors report lock.

The detectors track elapsed signal time rather than a number of `update` calls, so
their behaviour is independent of how the incoming signal is chunked: each `update`
is handed the `signal_duration` it represents and accumulates that duration into its
timers.
"""
abstract type AbstractLockDetector end

"""
    CodeLockDetector <: AbstractLockDetector

Declares code lock from the estimated carrier-to-noise density ratio. After a
`wait_time_threshold` warm-up it accumulates out-of-lock time whenever the CN0 is below
`cn0_threshold` (and pays it back down otherwise); lock is lost once the accumulated
out-of-lock time reaches `out_of_lock_time_threshold`.

The accumulated time is capped at twice that threshold. Since it is paid back at the rate
it is spent, an uncapped dwell would make re-declaring lock take as long as the outage that
preceded it — minutes, for a satellite kept in tracking through a tunnel by the
vector-tracking loop. With the cap, one `out_of_lock_time_threshold` of good signal always
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
Either way the detector's own warm-up (`wait_time_threshold`, 80 ms) is orders of magnitude
longer, so it never accumulates against it.
"""
struct CodeLockDetector <: AbstractLockDetector
    cn0_threshold::typeof(1.0dBHz)
    reference_integration_time::typeof(1.0s)
    coherence_limit::typeof(1.0s)
    out_of_lock_time::typeof(1.0s)
    out_of_lock_time_threshold::typeof(1.0s)
    wait_time::typeof(1.0s)
    wait_time_threshold::typeof(1.0s)
end

function CodeLockDetector(;
    cn0_threshold = 30dBHz,
    # GPS L1 C/A's code period. The receiver passes the ranging signal's own period —
    # see `primary_code_period` and the `ReceiverSatState` constructors.
    reference_integration_time = 1ms,
    coherence_limit = Inf * s,
    out_of_lock_time_threshold = 200ms,
    wait_time_threshold = 80ms,
)
    CodeLockDetector(
        cn0_threshold,
        reference_integration_time,
        coherence_limit,
        0.0s,
        out_of_lock_time_threshold,
        0.0s,
        wait_time_threshold,
    )
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

# How far above `out_of_lock_time_threshold` the accumulated out-of-lock time may run. The
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
    out_of_lock_time = lock_detector.out_of_lock_time
    if lock_detector.wait_time >= lock_detector.wait_time_threshold
        if is_below_cn0_threshold(lock_detector, cn0, integration_time)
            out_of_lock_time = min(
                out_of_lock_time + signal_duration,
                OUT_OF_LOCK_TIME_CAP_FACTOR * lock_detector.out_of_lock_time_threshold,
            )
        elseif out_of_lock_time > 0.0s
            out_of_lock_time -= signal_duration
        end
    end
    CodeLockDetector(
        lock_detector.cn0_threshold,
        lock_detector.reference_integration_time,
        lock_detector.coherence_limit,
        out_of_lock_time,
        lock_detector.out_of_lock_time_threshold,
        min(lock_detector.wait_time + signal_duration, lock_detector.wait_time_threshold),
        lock_detector.wait_time_threshold,
    )
end

"""
    is_in_lock(lock_detector::AbstractLockDetector)

Return `true` while the detector's accumulated out-of-lock time is below its threshold.
"""
function is_in_lock(lock_detector::AbstractLockDetector)
    lock_detector.out_of_lock_time < lock_detector.out_of_lock_time_threshold
end

"""
    set_out_of_lock(lock_detector::AbstractLockDetector)

Return a copy of the detector with its accumulated out-of-lock time raised to the
threshold, so [`is_in_lock`](@ref) immediately reports `false`. Used to force a
satellite out of lock, e.g. when vector tracking releases it for cause.
"""
function set_out_of_lock(lock_detector::AbstractLockDetector)
    @set lock_detector.out_of_lock_time = lock_detector.out_of_lock_time_threshold
end

"""
    CarrierLockDetector <: AbstractLockDetector

Declares carrier lock from the prompt correlator using the standard low-pass filtered
in-phase/quadrature amplitude test. Over each `integration_time_threshold` block it
compares the filtered in-phase amplitude against the filtered quadrature amplitude; too
little in-phase dominance accumulates out-of-lock time, and lock is lost once it reaches
`out_of_lock_time_threshold`.
"""
struct CarrierLockDetector <: AbstractLockDetector
    prev_filtered_inphase::Float64
    prev_filtered_quadrature::Float64
    integration_time::typeof(1.0s)
    integration_time_threshold::typeof(1.0s)
    out_of_lock_time::typeof(1.0s)
    out_of_lock_time_threshold::typeof(1.0s)
    wait_time::typeof(1.0s)
    wait_time_threshold::typeof(1.0s)
end

function CarrierLockDetector(;
    out_of_lock_time_threshold = 4s,
    wait_time_threshold = 80ms,
    integration_time_threshold = 80ms,
)
    CarrierLockDetector(
        0.0,
        0.0,
        0.0s,
        integration_time_threshold,
        0.0s,
        out_of_lock_time_threshold,
        0.0s,
        wait_time_threshold,
    )
end

function update(lock_detector::CarrierLockDetector, prompt, signal_duration)
    K1 = 0.0247
    K2 = 1.5
    next_filtered_inphase =
        (abs(real(prompt)) - lock_detector.prev_filtered_inphase) * K1 +
        lock_detector.prev_filtered_inphase
    next_filtered_quadrature =
        (abs(imag(prompt)) - lock_detector.prev_filtered_quadrature) * K1 +
        lock_detector.prev_filtered_quadrature

    out_of_lock_time = lock_detector.out_of_lock_time
    next_integration_time = lock_detector.integration_time + signal_duration
    if next_integration_time >= lock_detector.integration_time_threshold
        if lock_detector.wait_time >= lock_detector.wait_time_threshold
            if next_filtered_inphase / K2 < next_filtered_quadrature
                out_of_lock_time += next_integration_time
            else
                out_of_lock_time = 0.0s
            end
        end
        next_filtered_inphase = 0.0
        next_filtered_quadrature = 0.0
        next_integration_time = 0.0s
    end
    CarrierLockDetector(
        next_filtered_inphase,
        next_filtered_quadrature,
        next_integration_time,
        lock_detector.integration_time_threshold,
        out_of_lock_time,
        lock_detector.out_of_lock_time_threshold,
        min(lock_detector.wait_time + signal_duration, lock_detector.wait_time_threshold),
        lock_detector.wait_time_threshold,
    )
end
