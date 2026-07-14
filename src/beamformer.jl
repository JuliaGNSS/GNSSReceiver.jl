"""
    EigenBeamformer{N} <: AbstractPostCorrFilter

Post-correlation filter for an `N`-antenna array that steers towards a satellite by
eigen-beamforming. It accumulates the prompt correlator's spatial covariance and,
every `calc_new_every` updates, recomputes the beamforming weights from the dominant
eigenvector (the estimated signal subspace) before resetting the accumulator.

Construct one with [`EigenBeamformer(num_ants)`](@ref); apply it by calling the
instance on a per-antenna sample vector, and evolve it with `Tracking.update`.
"""
struct EigenBeamformer{N} <: AbstractPostCorrFilter
    covariance::SMatrix{N,N,ComplexF64}
    counter::Int
    calc_new_every::Int
    beamformer::SVector{N,ComplexF64}
end

"""
    EigenBeamformer(num_ants, calc_new_every = 20)

Create an [`EigenBeamformer`](@ref) for `num_ants` antennas whose weights are
recomputed every `calc_new_every` updates. The weights start as a unit response on the
last antenna and the covariance accumulator starts empty.
"""
function EigenBeamformer(num_ants, calc_new_every = 20)
    EigenBeamformer(
        zeros(SMatrix{num_ants,num_ants,ComplexF64}),
        0,
        calc_new_every,
        SVector{num_ants,ComplexF64}([zeros(num_ants - 1); 1]),
    )
end

function Tracking.update(filter::EigenBeamformer{N}, prompt) where {N}
    covariance = filter.covariance + prompt * prompt'
    counter = filter.counter + 1
    beamformer = filter.beamformer
    if counter % filter.calc_new_every == 0
        U = eigvecs(covariance)
        signal_space = U[:, N]
        beamformer = signal_space / signal_space[N]
        covariance = zero(covariance)
    end
    EigenBeamformer(covariance, counter, filter.calc_new_every, beamformer)
end

function (filter::EigenBeamformer)(x::AbstractVector)
    filter.beamformer' * x
end
