struct EigenBeamformer{N} <: AbstractPostCorrFilter
    covariance::SMatrix{N,N,ComplexF64}
    counter::Int
    calc_new_every::Int
    beamformer::SVector{N,ComplexF64}
end

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