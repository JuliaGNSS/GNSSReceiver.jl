@testset "Receive signal matrix of type $(type)" for type in [
    ComplexF64,
    ComplexF32,
    Complex{Int16},
]
    sampling_freq = 5e6Hz
    system = GPSL1CA()
    key = get_signal_id(system)
    num_samples = 20000
    num_ants = 4

    measurement_channel = GNSSReceiver.spawn_signal_channel_thread(;
        T = type,
        num_samples,
        num_antenna_channels = num_ants,
    ) do ch
        # Seed a local RNG so the noise — and hence any acquisition false alarms — is
        # deterministic and this test isn't flaky. An explicit Xoshiro avoids the
        # task-local RNG nondeterminism of the producer running in a spawned task.
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

    # The `Complex{Int16}` variant auto-selects Tracking's integer backend, which needs
    # `max_meas`; the noise is `round.(randn) * 512`, so `2^12` covers its full-scale.
    # `max_meas` is ignored for the float element types.
    data_channel = receive(
        measurement_channel,
        system,
        sampling_freq;
        num_ants = NumAnts(num_ants),
        max_meas = 2^12,
    )

    GNSSReceiver.consume_channel(data_channel) do data
        @test length(data.sat_data) == 0
        @test isnothing(data.pvt.time)
    end
end

@testset "Rejects a pilot-only (non-decodable) system" begin
    # A bare pilot carries no navigation data, so it has no decoder and must be
    # rejected up front rather than failing deep in decoder construction.
    @test_throws ArgumentError GNSSReceiver.ReceiverState(
        ComplexF64,
        GPSL5Q();
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(1),
    )
    # A CombinedSignal that pairs the pilot with its data component is accepted.
    @test GNSSReceiver.ReceiverState(
        ComplexF64,
        GNSSReceiver.CombinedSignal(GPSL5Q(), GPSL5I());
        num_samples_for_acquisition = 20000,
        num_ants = NumAnts(1),
    ) isa GNSSReceiver.ReceiverState
    # Guard predicate directly: pilot not decodable, data / combined are.
    @test !GNSSReceiver.is_decodable(GPSL5Q())
    @test GNSSReceiver.is_decodable(GPSL5I())
    @test GNSSReceiver.is_decodable(GNSSReceiver.CombinedSignal(GPSL5Q(), GPSL5I()))
end

# Deterministic multi-antenna noise channel for the extract-hook tests below: the
# noise (and hence any acquisition false alarm) is reproducible, and nothing is ever
# actually acquired, so the tests exercise the payload plumbing, not tracking.
function make_noise_channel(type, num_samples, num_ants)
    GNSSReceiver.spawn_signal_channel_thread(;
        T = type,
        num_samples,
        num_antenna_channels = num_ants,
    ) do ch
        rng = Random.Xoshiro(1234)
        foreach(1:20) do _
            put!(ch, type.(round.(randn(rng, ComplexF32, num_samples, num_ants) * 512)))
        end
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

    # `pvt_update_interval = 4u"ms"` (one chunk) makes every chunk emit a payload, so
    # the 80 ms noise run yields a real sequence rather than a single snapshot.
    data_channel = receive(
        measurement_channel,
        system,
        sampling_freq;
        num_ants = NumAnts(num_ants),
        max_meas,
        pvt_update_interval = 4u"ms",
        extract = my_extract,
    )

    # The channel element type is inferred from `extract` and is the concrete
    # payload type `extract` returns, not `ReceiverDataOfInterest`.
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
    opaque_extract(rs) =
        Base.inferencebarrier(length(Tracking.get_sat_states(rs.track_state)))::Any

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
