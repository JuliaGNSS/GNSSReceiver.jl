# Note: this file is included from runtests.jl which provides all `using` statements.
# Scratch must be loaded before this file is included.

# Per-type conversion of an offset-binary (I, Q) byte pair into a baseband sample.
# The recording is 8-bit unsigned with 128 ≈ zero. The float path recentres on the
# exact 127.5 midscale; the Int16 path recentres on the integer 128 (a 0.5-LSB DC
# offset the carrier loop removes) and so stays within ±128, making `max_meas = 2^7`
# exact and on Tracking's fast integer path. The samples are integers either way, so
# the Int16 element type adds no sample quantisation over the float path — the only
# difference is the correlator's own carrier-replica quantisation.
_ion_sample(::Type{ComplexF32}, i::UInt8, q::UInt8) =
    ComplexF32(Float32(i) - 127.5f0, Float32(q) - 127.5f0)
_ion_sample(::Type{Complex{Int16}}, i::UInt8, q::UInt8) =
    complex(Int16(i) - Int16(128), Int16(q) - Int16(128))

# Producer: read the recording in `num_samples`-sample chunks, convert each to `T`,
# and push onto `ch`. Parametric on `T` so the per-sample conversion dispatches
# statically — a function barrier off the element type, which is only a captured
# (non-constant) variable in the channel's `do` block.
function _ion_produce!(ch, ::Type{T}, dat_file, num_samples, num_ants) where {T}
    io = open(dat_file)
    raw_buf = Vector{UInt8}(undef, 2 * num_samples)
    try
        while !eof(io)
            n = readbytes!(io, raw_buf)
            n < 2 * num_samples && break
            # Fresh buffer per chunk: the channel buffers many chunks in flight
            # and `put!` stores them by reference (zero-copy), so a recycled
            # buffer would be overwritten while the consumer still holds it.
            chunk = Matrix{T}(undef, num_samples, num_ants)
            @inbounds for i = 1:num_samples
                chunk[i, 1] = _ion_sample(T, raw_buf[2i-1], raw_buf[2i])
            end
            put!(ch, chunk)
        end
    finally
        close(io)
    end
end

# Like `_ion_produce!`, but drops `slip_samples` samples right after `slip_after_chunk`
# chunks (shifting the rest of the stream by that many samples) and stops after
# `stop_after_chunk` chunks. Used to force a mid-stream timing slip so the tracking
# loops lose lock and the receiver has to reacquire.
function _ion_produce_with_slip!(
    ch,
    ::Type{T},
    dat_file,
    num_samples,
    num_ants;
    slip_after_chunk,
    slip_samples,
    stop_after_chunk,
) where {T}
    io = open(dat_file)
    raw_buf = Vector{UInt8}(undef, 2 * num_samples)
    slip_buf = Vector{UInt8}(undef, 2 * slip_samples)
    try
        chunk_i = 0
        while !eof(io)
            n = readbytes!(io, raw_buf)
            n < 2 * num_samples && break
            chunk = Matrix{T}(undef, num_samples, num_ants)
            @inbounds for i = 1:num_samples
                chunk[i, 1] = _ion_sample(T, raw_buf[2i-1], raw_buf[2i])
            end
            put!(ch, chunk)
            chunk_i += 1
            chunk_i == slip_after_chunk && readbytes!(io, slip_buf)
            chunk_i >= stop_after_chunk && break
        end
    finally
        close(io)
    end
end

# ION SDR sample data: 2.048 MS/s, 8-bit unsigned offset-binary I/Q, 60 s, GPS L1.
# Source:  https://sdr.ion.org/api-sample-data.html
# Ground truth (publisher's PRN list): {5, 13, 15, 20, 21, 28, 30}.
# Downloaded/cached once via Scratch.jl and shared across all testsets below.
let
    url = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
    scratch_dir = @get_scratch!("rtl_sdr_test_data")
    dat_file = joinpath(scratch_dir, "RTLSDR_Bands-L1.uint8")
    if !isfile(dat_file)
        @info "Downloading ION RTL-SDR test signal (~246 MB)..."
        cmd = `curl -sfL -o $dat_file $url`
        run(cmd)
        @info "Download complete: $(filesize(dat_file)) bytes"
    else
        @info "Using cached signal file: $dat_file ($(filesize(dat_file)) bytes)"
    end

    # Run the full acquire→track→decode→PVT pipeline for both sample element types
    # and assert against the SAME baseline. `receive` auto-selects the float CPU
    # backend for `ComplexF32` and Tracking's fast integer backend for
    # `Complex{Int16}`; both must reach the same fix on the same recording.
    @testset "ION RTL-SDR signal integration test ($type)" for type in
                                                               [ComplexF32, Complex{Int16}]
        sampling_freq = 2.048e6u"Hz"
        system = GPSL1CA()
        signal_id = get_signal_id(system)  # :GPSL1CA — the key half of the (signal, prn) tuple
        # 4 ms chunks = 8192 samples at 2.048 MHz
        num_samples = Int(upreferred(sampling_freq * 4u"ms"))
        num_ants = 1
        expected_prns = Set([5, 13, 15, 20, 21, 28, 30])

        measurement_channel = GNSSReceiver.spawn_signal_channel_thread(;
            T = type,
            num_samples,
            num_antenna_channels = num_ants,
        ) do ch
            _ion_produce!(ch, type, dat_file, num_samples, num_ants)
        end

        # `pvt_approximate_year = 2017` resolves the GPS L1 1024-week rollover for this
        # 2017-09-10 recording (without it, the default `year(now(UTC))` picks the wrong
        # cycle and reports a date offset by ~19.6 years).
        # `max_meas = 2^7`: the 8-bit offset-binary samples recentred on 128 have
        # |real|/|imag| ≤ 128. Required by (and only used for) the `Complex{Int16}`
        # integer backend; ignored for the `ComplexF32` float backend.
        data_channel = receive(
            measurement_channel,
            system,
            sampling_freq;
            num_ants = NumAnts(num_ants),
            interm_freq = 0.0u"Hz",
            max_meas = 2^7,
            pvt_approximate_year = 2017,
        )

        max_sats_seen = 0
        got_pvt = false
        last_pvt = nothing
        last_sat_data = nothing
        # sat_data / pvt.sats are keyed by (get_signal_id, prn); reduce to bare PRNs here.
        acquired_prns = Set{Int}()
        healthy_prns = Set{Int}()

        GNSSReceiver.consume_channel(data_channel) do data
            num_sats = length(data.sat_data)
            max_sats_seen = max(max_sats_seen, num_sats)
            for ((_, prn), sd) in pairs(data.sat_data)
                push!(acquired_prns, prn)
                if sd.is_healthy
                    push!(healthy_prns, prn)
                end
            end
            if !isnothing(data.pvt.time)
                got_pvt = true
                last_pvt = data.pvt
                last_sat_data = data.sat_data
            end
        end

        @info "Results ($type)" max_sats_seen got_pvt
        @info "Acquired PRNs ($(length(acquired_prns))): $(sort(collect(acquired_prns)))"
        @info "Healthy PRNs ($(length(healthy_prns))): $(sort(collect(healthy_prns)))"
        @info "Extras beyond ground truth: $(sort(collect(setdiff(acquired_prns, expected_prns))))"
        if got_pvt
            @info "PVT fix" time = last_pvt.time position = last_pvt.position
        end

        # The receiver locks the full healthy set of 11 sats on this 60 s signal with its
        # default acquisition. Asserted exactly so a regression that drops or adds a sat
        # is caught.
        healthy_ground_truth = Set([5, 7, 8, 13, 15, 18, 20, 21, 24, 28, 30])
        @test healthy_prns == healthy_ground_truth
        @test got_pvt

        # Tightened end-to-end check: assert the final PVT against captured baseline values
        # so any regression in tracking, decoding, or PVT shows up. The run is deterministic
        # to sub-mm / sub-ppm per element type; the tolerances absorb float-arithmetic
        # drift and the float-vs-integer correlator's carrier-replica quantisation.
        # Recording: Sep 10, 2017, Oegstgeest, NL (52.177°N 4.490°E, ~74 m).
        # Baseline captured under PositionVelocityTime 5.0.3, whose Niell tropospheric
        # mapping moved the fix (+0.35, +0.29, +1.14) m against the 5.0.0 capture — the
        # mostly-vertical shift a mapping-function change produces.
        expected_position = [3.9074087926e6, 3.0683836901e5, 5.0149608655e6]   # ECEF metres
        expected_velocity = [0.610, 0.209, 3.577]                              # m/s
        expected_time = TAIEpoch(2017, 9, 10, 22, 57, 20.697)                  # final-fix epoch (TAI)
        expected_time_correction = -2.0226547144e7                             # receiver clock bias (metres)
        expected_relative_clock_drift = 7.40e-7                                # dimensionless
        expected_gdop = 1.62
        expected_pdop = 1.45
        expected_hdop = 0.91
        expected_vdop = 1.13
        expected_tdop = 0.72
        # Baselined on Tracking 6's default NWPR estimator, midway between the two
        # backends. Within ~1 dB of the moment ratio's numbers these replaced: every
        # satellite here is 40 dB-Hz or better, where a coherent estimator loses little to
        # loop phase noise. The exception is PRN 13, 2 dB lower — also the least steadily
        # tracked satellite of the eleven, its per-record estimate dipping to ~27 dB-Hz
        # during the run, which is exactly what a coherent estimate charges for and the
        # moment ratio's noise floor hid.
        expected_cn0_dbhz = Dict(
            5 => 50.6, 7 => 41.4, 8 => 41.3, 13 => 45.0, 15 => 47.6,
            18 => 40.1, 20 => 43.6, 21 => 41.4, 24 => 40.3, 28 => 49.6, 30 => 47.9,
        )

        # Position: 1 m tolerance. Each backend is deterministic (bit-identical run to run),
        # so this only absorbs the float-vs-integer correlator difference (~0.1 m here) —
        # both element types are asserted against the same baseline.
        @test isapprox(last_pvt.position[1], expected_position[1], atol = 1.0)
        @test isapprox(last_pvt.position[2], expected_position[2], atol = 1.0)
        @test isapprox(last_pvt.position[3], expected_position[3], atol = 1.0)

        # Velocity: 1 m/s tolerance. The receiver is stationary, so this is a noise-level
        # observable (the tracking-loop steady-state residual). The integer backend's
        # threaded correlator reduces in a scheduling-dependent order, so the Int16 velocity
        # drifts by a few tenths of a m/s between multi-threaded runs; 1 m/s absorbs that
        # while still catching real regressions (a Doppler sign flip is ~±4 m/s,
        # reader-conjugation bugs ~10³ m/s).
        @test isapprox(last_pvt.velocity[1], expected_velocity[1], atol = 1.0)
        @test isapprox(last_pvt.velocity[2], expected_velocity[2], atol = 1.0)
        @test isapprox(last_pvt.velocity[3], expected_velocity[3], atol = 1.0)

        # Final-fix epoch: exact recording date (validates 1024-week rollover resolution
        # via pvt_approximate_year=2017). 1 ms tolerance covers chunk-boundary jitter in
        # when the last PVT happens to be reported.
        @test last_pvt.time isa TAIEpoch
        @test abs(AstroTime.value(last_pvt.time - expected_time)) < 0.001

        # Receiver clock bias (now in metres) and drift
        @test isapprox(ustrip(u"m", last_pvt.time_correction), expected_time_correction, rtol = 1e-4)
        @test isapprox(last_pvt.relative_clock_drift, expected_relative_clock_drift, rtol = 0.05)

        # DOPs depend only on satellite geometry; reproducible to ~1%.
        @test isapprox(last_pvt.dop.GDOP, expected_gdop, rtol = 0.05)
        @test isapprox(last_pvt.dop.PDOP, expected_pdop, rtol = 0.05)
        @test isapprox(last_pvt.dop.HDOP, expected_hdop, rtol = 0.05)
        @test isapprox(last_pvt.dop.VDOP, expected_vdop, rtol = 0.05)
        @test isapprox(last_pvt.dop.TDOP, expected_tdop, rtol = 0.05)

        # All 11 healthy sats should be in the fix.
        @test length(last_pvt.sats) == 11

        # Per-satellite CN0 at the time of the final PVT fix: ±2 dB-Hz catches sensitivity
        # regressions in the correlator/post-corr filter. Each backend is deterministic, so
        # the tolerance covers the float-vs-integer correlator difference (both element types
        # are asserted against this one baseline), not run-to-run noise.
        for (prn, expected_dbhz) in expected_cn0_dbhz
            key = (signal_id, prn)
            @test haskey(last_sat_data, key)
            cn0_dbhz = ustrip(last_sat_data[key].cn0)
            @test isapprox(cn0_dbhz, expected_dbhz, atol = 2.0)
        end
    end

    # Drive the reacquisition path with the real recording: once the full set is
    # locked, drop a few samples so the code phase slips out of the correlator's
    # pull-in range. Satellites lose lock and the receiver reacquires them (a single
    # dropped sample is only a ~0.5-chip step the DLL absorbs, and a full 2048-sample
    # code period would realign exactly — a ~1.5-chip, 3-sample slip reliably knocks
    # them out).
    @testset "Reacquisition after a mid-stream sample slip" begin
        sampling_freq = 2.048e6u"Hz"
        system = GPSL1CA()
        num_samples = Int(upreferred(sampling_freq * 4u"ms"))
        chunk_time = 4e-3  # seconds of signal per chunk
        # The derived cold-start acquisition (4 ms coherent integration here) locks the
        # 7 strongest sats on its first pass; the weaker ones clear the CFAR detection
        # threshold only on later periodic passes — PRNs 7/18/24 at ~10 s and PRN 8
        # (the weakest, ~41 dB-Hz) at ~20 s. Slip ~21.2 s in — once the full healthy set of 11 is locked — and
        # stop ~24.4 s in, before the next periodic acquisition at 30 s, so any
        # recovery is via reacquisition of the lost satellites rather than the
        # periodic full search.
        slip_after_chunk = 5300
        stop_after_chunk = 6100

        measurement_channel = GNSSReceiver.spawn_signal_channel_thread(;
            T = ComplexF32,
            num_samples,
            num_antenna_channels = 1,
        ) do ch
            _ion_produce_with_slip!(
                ch,
                ComplexF32,
                dat_file,
                num_samples,
                1;
                slip_after_chunk,
                slip_samples = 3,
                stop_after_chunk,
            )
        end

        data_channel = receive(
            measurement_channel,
            system,
            sampling_freq;
            num_ants = NumAnts(1),
            interm_freq = 0.0u"Hz",
            pvt_approximate_year = 2017,
        )

        # `receive` emits one snapshot per `pvt_update_interval` (100 ms), not per
        # chunk, so record (runtime, tracked-sat count) pairs and split the series at
        # the signal time of the slip. `sat_data` is built from the track state, so
        # satellites removed on lock loss drop out of the count.
        records = Tuple{Float64,Int}[]
        GNSSReceiver.consume_channel(data_channel) do data
            push!(records, (ustrip(u"s", data.runtime), length(data.sat_data)))
        end

        slip_time = slip_after_chunk * chunk_time
        before_slip = [n for (t, n) in records if t <= slip_time]
        after_slip = [n for (t, n) in records if t > slip_time]
        @info "Reacquisition slip" locked_before = maximum(before_slip) min_after =
            minimum(after_slip) final = records[end][2]

        # (Nearly) the full healthy set of 11 is locked before the slip: the four
        # weakest extras (~41 dB-Hz) bounce in and out of lock, so the tracked count
        # at the 100 ms snapshot granularity sits at 10-11.
        @test maximum(before_slip) >= 10
        # The slip knocks satellites out of lock (they are removed from the track state).
        @test minimum(after_slip) < maximum(before_slip)
        # Reacquisition brings satellites back before the run ends.
        @test records[end][2] > minimum(after_slip)
    end

    # Run the same recording with vector tracking: the receiver starts on the vector
    # estimator's scalar fallback loops, switches to the navigation filter with the
    # first scalar fix, and from then on every emitted PVT solution comes from the
    # filter with the loops closed centrally. Assert against the same ground truth as
    # the scalar baseline, for both measurement sets — pseudoranges + pseudorange
    # rates (VDFLL) and pseudoranges only (VDLL).
    @testset "ION RTL-SDR vector tracking integration test" begin
        # Deliberately in MHz (not Hz) while the IF below is in Hz: a real front end mixes
        # units this way, and `Tracking.BandMeasurement`'s `promote` then collapses both to
        # the SI base `s^-1`. Vector tracking's `Hz`-typed discriminator accumulator rejects
        # that, so this unit choice guards the `uconvert(Hz, …)` normalisation in `process`
        # (regression for the VT first-fix crash). Do not "tidy" this back to Hz.
        sampling_freq = 2.048u"MHz"
        system = GPSL1CA()
        num_samples = Int(upreferred(sampling_freq * 4u"ms"))

        measurement_channel = GNSSReceiver.spawn_signal_channel_thread(;
            T = ComplexF32,
            num_samples,
            num_antenna_channels = 1,
        ) do ch
            _ion_produce!(ch, ComplexF32, dat_file, num_samples, 1)
        end

        # Capture the vector-tracking engagement alongside the default payload: how
        # many satellites the navigation filter controls and whether it is running.
        extract = function (receiver_state)
            (
                data = GNSSReceiver.default_data_of_interest(receiver_state),
                vt_running = receiver_state.vt.running,
                num_vt_sats = count(
                    GNSSReceiver.in_vt_loop,
                    get_sat_states(receiver_state.track_state, :GPSL1CA),
                ),
            )
        end

        data_channel = receive(
            measurement_channel,
            system,
            sampling_freq;
            num_ants = NumAnts(1),
            interm_freq = 0.0u"Hz",
            pvt_approximate_year = 2017,
            vector_tracking = true,
            extract,
        )

        num_vt_outputs = 0
        max_vt_sats = 0
        last = nothing
        GNSSReceiver.consume_channel(data_channel) do out
            if out.vt_running
                num_vt_outputs += 1
                max_vt_sats = max(max_vt_sats, out.num_vt_sats)
                last = out
            end
        end

        @info "Vector tracking results" num_vt_outputs max_vt_sats
        # The vector loop engages and stays engaged for most of the run (the first
        # scalar fix needs the ~30 s navigation-message decode plus lock time; ~25 s
        # of vector tracking remain on this 60 s recording).
        @test num_vt_outputs > 100
        @test max_vt_sats >= 7
        @test !isnothing(last)

        pvt = last.data.pvt
        @info "Vector tracking final fix" pvt.time pvt.position num_sats = length(pvt.sats)

        # The same receiver location as the scalar baseline (Sep 10, 2017,
        # Oegstgeest, NL). The navigation filter's pseudoranges are atmosphere-
        # corrected like the scalar solve's; the final fix lands within a few
        # metres of the baseline (the filter smooths through its own promotion
        # transients rather than solving each epoch independently).
        expected_position = [3.9074087926e6, 3.0683836901e5, 5.0149608655e6]
        @test isapprox(pvt.position[1], expected_position[1], atol = 10.0)
        @test isapprox(pvt.position[2], expected_position[2], atol = 10.0)
        @test isapprox(pvt.position[3], expected_position[3], atol = 10.0)

        # Stationary receiver: the filter's velocity is noise-level.
        @test norm([pvt.velocity...]) < 2.0

        # The final epoch matches the recording date to within the run length.
        @test pvt.time isa TAIEpoch
        expected_final_time = TAIEpoch(2017, 9, 10, 22, 57, 20.697)
        @test abs(AstroTime.value(pvt.time - expected_final_time)) < 1.0

        # The filter reports a full measurement geometry at the end of the run.
        @test length(pvt.sats) >= 7
        @test !isnothing(pvt.dop)

        # `SatInfo.residual` is the POST-fit pseudorange residual — the same
        # quantity the scalar `calc_pvt` reports, in the same observed − computed
        # orientation — so a vector-tracking solution is directly comparable with a
        # scalar one. Reporting the filter's pre-fit innovation here instead would
        # carry the prediction error that the measurement update removes.
        #
        # The *scatter* across satellites is the part that reflects measurement
        # quality, and it is comparable to the scalar solve's on this recording
        # (~7 m vs ~4 m). The common mode is not: the vector loop settles with a
        # steady-state code-phase lag against the receiver clock drift, which on
        # this RTL-SDR capture (cheap TCXO, large drift) reaches ~48 m ≈ 0.16
        # L1 C/A chips within 10 s of engaging and then holds. Being common mode
        # it is absorbed by the clock-bias state, so the position above is
        # unaffected — but it does inflate the reported residual RMS relative to
        # a scalar solve. Tracked as an open issue; asserted loosely here so a
        # regression in the per-satellite part is still caught.
        residuals = [ustrip(u"m", info.residual) for info in pvt.sats]
        @test all(isfinite, residuals)
        common_mode = sum(residuals) / length(residuals)
        scatter = sqrt(sum(abs2, residuals .- common_mode) / length(residuals))
        @test scatter < 15.0
        @test abs(common_mode) < 80.0

        # `SatInfo.rate_residual` is the range-rate counterpart, formed from the
        # carrier-Doppler (FLL) measurements the VDFLL fuses alongside the
        # pseudoranges. It is the independent half of the diagnostic: a satellite
        # can carry a clean pseudorange residual and still disagree with the
        # velocity/clock-drift solution.
        #
        # Unlike the pseudorange residual above, this one does *not* inherit a large
        # common mode from the code-phase lag — the lag is a phase offset, not a rate
        # one. Both the scatter (~1.7 m/s) and the common mode (~0.6 m/s) land in the
        # same range as the scalar solve's rate residuals on this recording (~1.3 m/s
        # RMS about a mean of zero): on an RTL-SDR front end this is the Doppler
        # measurement quality itself, not a vector-loop artefact. Bounded with
        # headroom so a real regression — a sign flip, a wavelength/Hz mix-up, or a
        # missing FLL contribution — still shows up.
        rate_residuals = [ustrip(u"m/s", info.rate_residual) for info in pvt.sats]
        @test all(isfinite, rate_residuals)
        rate_common_mode = sum(rate_residuals) / length(rate_residuals)
        rate_scatter =
            sqrt(sum(abs2, rate_residuals .- rate_common_mode) / length(rate_residuals))
        @test rate_scatter < 4.0
        @test abs(rate_common_mode) < 2.0
    end
end
