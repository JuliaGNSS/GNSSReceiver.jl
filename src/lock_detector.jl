"""
    AbstractLockDetector

Supertype for the receiver's per-satellite lock detectors. A concrete detector is
advanced with `update` and queried with [`is_in_lock`](@ref); a satellite is treated
as locked only while both its code and carrier detectors report lock.
"""
abstract type AbstractLockDetector end

"""
    CodeLockDetector <: AbstractLockDetector

Declares code lock from the estimated carrier-to-noise density ratio. After a
`wait_counter_threshold`-update warm-up it increments an out-of-lock counter each time
the CN0 is below `cn0_threshold` (and decrements it otherwise); lock is lost once that
counter reaches `num_out_of_lock_threshold`.
"""
struct CodeLockDetector <: AbstractLockDetector
    cn0_threshold::typeof(1.0u"dBHz")
    num_out_of_lock::Int
    num_out_of_lock_threshold::Int
    wait_counter_threshold::Int
    wait_counter::Int
end

function CodeLockDetector(;
    cn0_threshold = 30u"dBHz",
    num_out_of_lock_threshold = 50,
    wait_counter_threshold = 20,
)
    CodeLockDetector(cn0_threshold, 0, num_out_of_lock_threshold, wait_counter_threshold, 0)
end

function update(lock_detector::CodeLockDetector, cn0)
    num_out_of_lock = lock_detector.num_out_of_lock
    if lock_detector.wait_counter + 1 > lock_detector.wait_counter_threshold
        if cn0 < lock_detector.cn0_threshold
            num_out_of_lock += 1
        elseif num_out_of_lock > 0
            num_out_of_lock -= 1
        end
    end
    CodeLockDetector(
        lock_detector.cn0_threshold,
        num_out_of_lock,
        lock_detector.num_out_of_lock_threshold,
        lock_detector.wait_counter_threshold,
        min(lock_detector.wait_counter + 1, lock_detector.wait_counter_threshold),
    )
end

"""
    is_in_lock(lock_detector::AbstractLockDetector)

Return `true` while the detector's out-of-lock counter is below its threshold.
"""
function is_in_lock(lock_detector::AbstractLockDetector)
    lock_detector.num_out_of_lock < lock_detector.num_out_of_lock_threshold
end

"""
    CarrierLockDetector <: AbstractLockDetector

Declares carrier lock from the prompt correlator using the standard low-pass filtered
in-phase/quadrature amplitude test. Over each 20-integration block it compares the
filtered in-phase amplitude against the filtered quadrature amplitude; too little
in-phase dominance increments the out-of-lock counter, and lock is lost once it reaches
`num_out_of_lock_threshold`.
"""
struct CarrierLockDetector <: AbstractLockDetector
    prev_filtered_inphase::Float64
    prev_filtered_quadrature::Float64
    integration_counter::Int
    num_out_of_lock::Int
    num_out_of_lock_threshold::Int
    wait_counter_threshold::Int
    wait_counter::Int
end

function CarrierLockDetector(num_out_of_lock_threshold = 50, wait_counter_threshold = 20)
    CarrierLockDetector(
        0.0,
        0.0,
        0,
        0,
        num_out_of_lock_threshold,
        wait_counter_threshold,
        0,
    )
end

function update(lock_detector::CarrierLockDetector, prompt)
    K1 = 0.0247
    K2 = 1.5
    integration_counter_threshold = 20
    next_filtered_inphase =
        (abs(real(prompt)) - lock_detector.prev_filtered_inphase) * K1 +
        lock_detector.prev_filtered_inphase
    next_filtered_quadrature =
        (abs(imag(prompt)) - lock_detector.prev_filtered_quadrature) * K1 +
        lock_detector.prev_filtered_quadrature

    num_out_of_lock = lock_detector.num_out_of_lock
    if lock_detector.integration_counter + 1 == integration_counter_threshold
        if lock_detector.wait_counter + 1 > lock_detector.wait_counter_threshold
            if next_filtered_inphase / K2 < next_filtered_quadrature
                num_out_of_lock += 1
            else
                num_out_of_lock = 0
            end
        end
        next_filtered_inphase = 0.0
        next_filtered_quadrature = 0.0
    end
    CarrierLockDetector(
        next_filtered_inphase,
        next_filtered_quadrature,
        mod(lock_detector.integration_counter + 1, integration_counter_threshold),
        num_out_of_lock,
        lock_detector.num_out_of_lock_threshold,
        lock_detector.wait_counter_threshold,
        min(lock_detector.wait_counter + 1, lock_detector.wait_counter_threshold),
    )
end
