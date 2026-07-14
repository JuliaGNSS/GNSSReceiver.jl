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
"""
struct CodeLockDetector <: AbstractLockDetector
    cn0_threshold::typeof(1.0u"dBHz")
    out_of_lock_time::typeof(1.0u"s")
    out_of_lock_time_threshold::typeof(1.0u"s")
    wait_time::typeof(1.0u"s")
    wait_time_threshold::typeof(1.0u"s")
end

function CodeLockDetector(;
    cn0_threshold = 30u"dBHz",
    out_of_lock_time_threshold = 200u"ms",
    wait_time_threshold = 80u"ms",
)
    CodeLockDetector(
        cn0_threshold,
        0.0u"s",
        out_of_lock_time_threshold,
        0.0u"s",
        wait_time_threshold,
    )
end

function update(lock_detector::CodeLockDetector, cn0, signal_duration)
    out_of_lock_time = lock_detector.out_of_lock_time
    if lock_detector.wait_time >= lock_detector.wait_time_threshold
        if cn0 < lock_detector.cn0_threshold
            out_of_lock_time += signal_duration
        elseif out_of_lock_time > 0.0u"s"
            out_of_lock_time -= signal_duration
        end
    end
    CodeLockDetector(
        lock_detector.cn0_threshold,
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
