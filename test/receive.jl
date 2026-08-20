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
    # The message names the offending component the way an ICD does
    # (`get_signal_name`), not by its type name.
    @test occursin(
        "GPS L5-Q",
        try
            GNSSReceiver.assert_decodable((GPSL5Q(),))
        catch e
            e.msg
        end,
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

@testset "Rejects systems spanning more than one RF band" begin
    # One sample stream carries one carrier, so a genuinely un-tunable mix must be
    # rejected — while signals of different constellations sharing a carrier are fine.
    @test isnothing(GNSSReceiver.assert_single_band((GPSL1CA(), GalileoE1B())))
    @test_throws ArgumentError GNSSReceiver.assert_single_band((GPSL1CA(), GPSL5I()))
    # The message names the bands (`get_band_name`) rather than showing the `Band`
    # instances, which would read "(L1(), L5())".
    msg = try
        GNSSReceiver.assert_single_band((GPSL1CA(), GPSL5I()))
    catch e
        e.msg
    end
    @test occursin("L1, L5", msg)
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

# The wiring that puts the lock stage into the emitted payload. `SatelliteDataOfInterest`
# reports `is_ranging_ready` so that a satellite kept through the acquisition handover — in
# lock, deliberately, but not yet trustworthy to range on — is distinguishable from one the
# PVT solve is ignoring for no visible reason.
@testset "Emitted satellite data reports the ranging stage" begin
    # Detectors driven through `update`, never by writing fields, so this stays independent
    # of the dwell's layout and of how ranging readiness is decided.
    function ready_code_detector()
        detector = GNSSReceiver.CodeLockDetector(;
            cn0_threshold = 30.0u"dBHz",
            reference_integration_time = 1u"ms",
        )
        updates = 0
        while !GNSSReceiver.is_ranging_ready(detector)
            detector = GNSSReceiver.update(detector, 45.0u"dBHz", 1u"ms", 4u"ms")
            updates += 1
            @assert updates <= 10_000 "detector never reported ranging readiness"
        end
        detector
    end

    sat_state(code_detector) = GNSSReceiver.ReceiverSatState(
        1,
        GNSSDecoderState(GPSL1CA(), 1),
        code_detector,
        GNSSReceiver.CarrierLockDetector(; reference_integration_time = 1u"ms"),
        5.0u"s",
        0.0u"s",
        0,
        false,
    )

    # A fresh detector is in lock but not ranging-ready; a driven one is both.
    fresh = GNSSReceiver.CodeLockDetector(;
        cn0_threshold = 30.0u"dBHz",
        reference_integration_time = 1u"ms",
    )
    settling = sat_state(fresh)
    @test GNSSReceiver.is_in_lock(settling)
    @test !GNSSReceiver.is_ranging_ready(settling)

    states = Dictionary([1], [settling])
    @test !GNSSReceiver.is_sat_ranging_ready_at(states, 1)
    # The carrier detector gates too, so a ready *code* detector alone is not enough.
    @test !GNSSReceiver.is_sat_ranging_ready_at(
        Dictionary([1], [sat_state(ready_code_detector())]),
        1,
    )
    # Absent state must never read as ready — the same fail-safe direction as
    # `is_sat_healthy_at`, which a satellite the receiver holds no state for also fails.
    @test !GNSSReceiver.is_sat_ranging_ready_at(states, 99)
    @test !GNSSReceiver.is_sat_healthy_at(states, 99)

    # The lock verdict is reported alongside it, and is NOT constant-true in the payload: a
    # vector-loop member that has lost lock is deliberately kept in tracking
    # (`remove_lost_satellites`), and its ranging readiness stays latched from before the
    # outage. So readiness alone does not answer "would the receiver range on this satellite
    # right now?" — `collect_pvt_sat_states!` requires both, and a consumer needs both.
    @test GNSSReceiver.is_sat_in_lock_at(states, 1)
    @test !GNSSReceiver.is_sat_in_lock_at(states, 99)
    coasting = sat_state(GNSSReceiver.set_out_of_lock(ready_code_detector()))
    coasting = @set coasting.in_vt_loop = true
    coasted_states = Dictionary([1], [coasting])
    @test GNSSReceiver.is_sat_ranging_ready_at(coasted_states, 1)
    @test !GNSSReceiver.is_sat_in_lock_at(coasted_states, 1)
end
