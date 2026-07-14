# Seed a local RNG so the noise — and hence any acquisition false alarms — is
# deterministic and this test isn't flaky. An explicit Xoshiro avoids the
# task-local RNG nondeterminism of the producer running in a spawned task.
function make_noise_channel(type, num_samples, num_ants)
    GNSSReceiver.SignalChannel{type,num_ants}(num_samples) do ch
        rng = Random.Xoshiro(1234)
        if type <: Complex{Int16}
            foreach(
                i -> put!(
                    ch,
                    type.(round.(randn(rng, ComplexF32, num_samples, num_ants) * 512)),
                ),
                1:20,
            )
        else
            foreach(i -> put!(ch, randn(rng, type, num_samples, num_ants) * 512), 1:20)
        end
    end
end

@testset "Receive signal matrix of type $(type)" for type in [
    ComplexF64,
    ComplexF32,
    Complex{Int16},
]
    sampling_freq = 5e6Hz
    system = GPSL1CA()
    num_samples = 20000
    num_ants = 4

    measurement_channel = make_noise_channel(type, num_samples, num_ants)

    # Samples are `randn * 512`, so |real|/|imag| stays well under 2^12; declare
    # that as the Int16 full-scale. `max_meas` is ignored for float element types.
    max_meas = 2^12
    data_channel = receive(
        measurement_channel,
        system,
        sampling_freq;
        num_ants = NumAnts(num_ants),
        max_meas,
    )

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 0
        @test isnothing(data.pvt.time)
    end

    receiver_sat_states = (Dictionary([1], [GNSSReceiver.ReceiverSatState(system, 1)]),)

    track_state =
        TrackState(system, [TrackedSat(system, 1, 0.0, 20u"Hz"; num_ants = NumAnts(4))])

    acquisition_buffer = GNSSReceiver.SampleBuffer(ComplexF64, 20000)

    pvt = PositionVelocityTime.PVTSolution()

    decoder = GNSSDecoderState(system, 1)
    pvt_sat_state_buffer = SatelliteState{Float64,typeof(decoder),typeof(system)}[]

    receiver_state = ReceiverState(
        track_state,
        receiver_sat_states,
        acquisition_buffer,
        pvt,
        pvt_sat_state_buffer,
        0.0u"s",
        -Inf * 1.0u"s",
        -Inf * 1.0u"s",
        0,
    )

    measurement_channel = make_noise_channel(type, num_samples, num_ants)

    data_channel = receive(
        measurement_channel,
        system,
        sampling_freq;
        num_ants = NumAnts(num_ants),
        receiver_state,
        max_meas,
    )

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 1
        @test isnothing(data.pvt.time)
    end
end

@testset "Receive with a custom extract hook" begin
    sampling_freq = 5e6Hz
    system = GPSL1CA()
    num_samples = 20000
    num_ants = 4
    max_meas = 2^12

    measurement_channel = make_noise_channel(Complex{Int16}, num_samples, num_ants)

    # A custom payload instead of the default ReceiverDataOfInterest: the runtime and
    # the number of currently tracked satellites.
    my_extract(rs) =
        (runtime = rs.runtime, num_sats = length(Tracking.get_sat_states(rs.track_state)))

    data_channel = receive(
        measurement_channel,
        system,
        sampling_freq;
        num_ants = NumAnts(num_ants),
        max_meas,
        extract = my_extract,
    )

    # The channel element type is inferred from `extract`, concretely, and is no longer
    # a ReceiverDataOfInterest.
    @test isconcretetype(eltype(data_channel))
    @test eltype(data_channel) <: NamedTuple
    @test !(eltype(data_channel) <: GNSSReceiver.ReceiverDataOfInterest)

    # collect_data works on the custom-payload channel too.
    data = collect_data(data_channel)
    @test eltype(data) == eltype(data_channel)
    @test !isempty(data)
    @test all(d -> d.num_sats == 0, data)                # pure noise ⇒ nothing tracked
    @test issorted([d.runtime for d in data])            # runtime advances monotonically
end

@testset "Receive falls back to a runtime call for a non-inferrable extract" begin
    sampling_freq = 5e6Hz
    system = GPSL1CA()
    num_samples = 20000
    num_ants = 4
    max_meas = 2^12

    measurement_channel = make_noise_channel(Complex{Int16}, num_samples, num_ants)

    # An extract whose return type inference can't pin down concretely (the
    # inference barrier hides it as `Any`), even though every call returns an `Int`.
    # `Base.promote_op` yields a non-concrete type, so `receive` falls back to
    # calling `extract` on the initial state to learn the concrete payload type.
    opaque_extract(rs) = Base.inferencebarrier(rs.num_samples_processed)::Any

    data_channel = receive(
        measurement_channel,
        system,
        sampling_freq;
        num_ants = NumAnts(num_ants),
        max_meas,
        extract = opaque_extract,
    )

    # The fallback pins the concrete element type from the initial state's payload.
    @test eltype(data_channel) == Int
    data = collect_data(data_channel)
    @test !isempty(data)
    @test eltype(data) == Int
end
