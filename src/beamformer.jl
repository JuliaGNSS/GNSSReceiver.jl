"""
    EigenBeamformer{N} <: AbstractPostCorrFilter

Post-correlation filter for an `N`-antenna array that steers towards a satellite by
eigen-beamforming. It accumulates the prompt correlator's spatial covariance and,
every `calc_new_every` updates, recomputes the beamforming weights from the dominant
eigenvector (the estimated signal subspace) before resetting the accumulator.

Construct one with [`EigenBeamformer(num_ants)`](@ref); `Tracking` reads its current
weights through `Tracking.get_weights` and combines the correlator's per-antenna taps
itself, and `Tracking.update` evolves it.
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

# Tracking combines the antennas itself (`wᴴ·b`) rather than calling the filter, so
# that the C/N₀ path can reduce the signal's measured noise covariance through the same
# weights (`N₀ = wᴴR̂w`). The weights are the eigen-beamformer's own state, so declaring
# them is the whole of the contract — and it is scale-free: `R̂` carries the antennas'
# relative noise scale, so the reported C/N₀ no longer depends on the last-element
# normalisation `Tracking.update` applies.
Tracking.get_weights(filter::EigenBeamformer{N}, ::NumAnts{N}) where {N} =
    filter.beamformer
