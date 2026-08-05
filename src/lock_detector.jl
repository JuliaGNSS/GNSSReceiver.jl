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

The estimate itself comes from Tracking's default CN0 estimator, which as of Tracking 6 is
the narrowband/wideband power ratio (`NWPRCN0Estimator`) rather than the moment ratio.
`cn0_threshold` is unchanged in units and meaning, but the number it is compared against
is a different — and far better behaved — statistic:

  - the moment ratio manufactured signal power out of noise, reporting a median of
    ~27.6 dB-Hz on pure noise, so a threshold below ~30 dB-Hz could never trip and one at
    30 dB-Hz was cleared by noise on ~19 % of records. NWPR reports `-Inf dB-Hz` on noise
    and clears no finite threshold, so the out-of-lock dwell is now spent monotonically: a
    satellite that dies drops after exactly `out_of_lock_time_threshold`, rather than the
    ~1.3× that the detector used to take paying time back on the noise realizations that
    cleared the threshold;
  - NWPR measures *coherent* CN0, so residual loop phase noise counts against it. With the
    conventional PLL at 1 ms records and the default ~5 ms coherent window, Tracking
    measures a median 44.4 dB-Hz at a true 45 dB-Hz and 23.6 dB-Hz at a true 25 dB-Hz: a
    fraction of a dB on a strong satellite, one to two dB near threshold. The detector is
    therefore that much stricter than the nominal number suggests; lower `cn0_threshold`
    by the same if the pre-Tracking-6 sensitivity is wanted back.
"""
struct CodeLockDetector <: AbstractLockDetector
    cn0_threshold::typeof(1.0u"dBHz")
    reference_integration_time::typeof(1.0u"s")
    coherence_limit::typeof(1.0u"s")
    out_of_lock_time::typeof(1.0u"s")
    out_of_lock_time_threshold::typeof(1.0u"s")
    wait_time::typeof(1.0u"s")
    wait_time_threshold::typeof(1.0u"s")
end

function CodeLockDetector(;
    cn0_threshold = 30u"dBHz",
    # GPS L1 C/A's code period. The receiver passes the ranging signal's own period —
    # see `primary_code_period` and the `ReceiverSatState` constructors.
    reference_integration_time = 1u"ms",
    coherence_limit = Inf * u"s",
    out_of_lock_time_threshold = 200u"ms",
    wait_time_threshold = 80u"ms",
)
    CodeLockDetector(
        cn0_threshold,
        reference_integration_time,
        coherence_limit,
        0.0u"s",
        out_of_lock_time_threshold,
        0.0u"s",
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
# `false` — which would otherwise hold a dead channel in lock indefinitely. NWPR's "no
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

function update(lock_detector::CodeLockDetector, cn0, integration_time, signal_duration)
    out_of_lock_time = lock_detector.out_of_lock_time
    if lock_detector.wait_time >= lock_detector.wait_time_threshold
        if is_below_cn0_threshold(lock_detector, cn0, integration_time)
            out_of_lock_time += signal_duration
        elseif out_of_lock_time > 0.0u"s"
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
    integration_time::typeof(1.0u"s")
    integration_time_threshold::typeof(1.0u"s")
    out_of_lock_time::typeof(1.0u"s")
    out_of_lock_time_threshold::typeof(1.0u"s")
    wait_time::typeof(1.0u"s")
    wait_time_threshold::typeof(1.0u"s")
end

function CarrierLockDetector(;
    out_of_lock_time_threshold = 4u"s",
    wait_time_threshold = 80u"ms",
    integration_time_threshold = 80u"ms",
)
    CarrierLockDetector(
        0.0,
        0.0,
        0.0u"s",
        integration_time_threshold,
        0.0u"s",
        out_of_lock_time_threshold,
        0.0u"s",
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
                out_of_lock_time = 0.0u"s"
            end
        end
        next_filtered_inphase = 0.0
        next_filtered_quadrature = 0.0
        next_integration_time = 0.0u"s"
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
