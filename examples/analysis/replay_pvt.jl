# Replay the B/P/D/A/T records from hardware_correlator_position_fix.jl.
# This re-decodes the recorded soft bits and compares the reported phase with
# a diagnostic integer-period correction inferred at each initial bit sync.
# It does not re-run the tracking loops. The inference assumes the newest
# satellite record is less than one primary period behind the fold boundary;
# live anchor_bit_phases! additionally uses the actual per-channel timestamp.
# Reported errors are 3D distances from the supplied reference. calc_pvt may
# retain its previous solution when a solve fails, just as in the live receiver.
# This parser targets the 4 MSPS GPS L1 C/A recordings from the 2026 campaign.
# Run in the same Julia environment that produced the recording.
using GNSSReceiver,
    GNSSDecoder, GNSSSignals, PositionVelocityTime, Unitful, Geodesy, Statistics
using Unitful: Hz
function replay(path, reference)
    system = GPSL1CA()
    decoders = Dict{Int,typeof(GNSSDecoderState(system, 1))}()
    phases = Dict{Int,Tuple{Float64,Float64,Int64}}()
    blocks = Dict{Int,Int}()
    offsets = Dict{Int,Int}()
    old = PVTSolution()
    fixed = PVTSolution()
    epoch = Int64(0)
    firstepoch = Int64(0)
    lastsolve = Int64(0)
    ref = ECEF(reference, wgs84)
    errs = Float64[]
    olderrs = Float64[]
    pdops = Float64[]
    fresh_errors = Float64[]
    fresh_epochs = Float64[]
    function solve(epoch)
        states = SatelliteState[]
        corrected = SatelliteState[]
        for (prn, (phase, doppler, t)) in phases
            t == epoch || continue
            d = get(decoders, prn, nothing)
            isnothing(d) && continue
            isnothing(d.num_bits_after_valid_syncro_sequence) && continue
            push!(states, SatelliteState(d, system, phase, doppler*Hz, 0.0))
            cp = mod(phase-get(offsets, prn, 0)*1023, 20460)
            push!(corrected, SatelliteState(d, system, cp, doppler*Hz, 0.0))
        end
        length(states) < 4 && return
        old = calc_pvt(states, old; approximate_year = 2026)
        previous_time = fixed.time
        fixed = calc_pvt(corrected, fixed; approximate_year = 2026)
        if !isnothing(fixed.time)
            err = sqrt(sum(abs2, fixed.position - ref))
            push!(errs, err)
            if !isequal(fixed.time, previous_time)
                push!(fresh_errors, err)
                push!(fresh_epochs, (epoch - firstepoch) / 4e6)
            end
            isnothing(fixed.dop) || push!(pdops, fixed.dop.PDOP)
            oe = sqrt(sum(abs2, old.position - ref))
            push!(olderrs, oe)
            if length(errs)%20 == 1
                println(
                    "t=",
                    (epoch-firstepoch)/4e6,
                    " original_m=",
                    round(oe),
                    " corrected_m=",
                    round(err),
                    " offsets=",
                    offsets,
                )
            end
        end
    end
    for line in eachline(path)
        s=split(line)
        isempty(s) && continue
        if s[1]=="T"
            if epoch-lastsolve>=2_000_000
                solve(epoch)
                lastsolve=epoch
            end
            epoch=parse(Int64, s[3])
            firstepoch==0 && (firstepoch=epoch)
        elseif s[1]=="A"
            prev=parse(Int, s[3])
            prn=parse(Int, s[4])
            if prev>0
                delete!(phases, prev)
                delete!(decoders, prev)
                delete!(offsets, prev)
                delete!(blocks, prev)
            end
            if prn>0
                decoders[prn]=GNSSDecoderState(system, prn)
                delete!(blocks, prn)
                delete!(offsets, prn)
            end
        elseif s[1]=="B"
            prn=parse(Int, s[2])
            bits=parse.(Float32, s[4:end])
            d=get!(decoders, prn) do
                GNSSDecoderState(system, prn)
            end
            decoders[prn]=decode(d, bits, length(bits))
        elseif s[1]=="P" && s[5]=="true"
            blocks[parse(Int, s[2])]=parse(Int, s[6])
        elseif s[1]=="D"
            prn=parse(Int, s[2])
            phase=parse(Float64, s[11])
            doppler=parse(Float64, s[7])
            t=parse(Int64, s[3])
            phases[prn]=(phase, doppler, t)
            if haskey(blocks, prn) && !haskey(offsets, prn)
                offsets[prn]=mod(floor(Int, phase/1023)-blocks[prn]+10, 20)-10
            end
        end
    end
    println("solutions=", length(errs))
    isempty(fresh_errors) || println(
        "fresh_timestamp_updates=", length(fresh_errors),
        " first_last_s=", extrema(fresh_epochs),
        " fresh_median_m=", median(fresh_errors),
        " fresh_range_m=", extrema(fresh_errors),
    )
    isempty(errs) || println(
        "original median_m=",
        median(olderrs),
        " corrected median_m=",
        median(errs),
        " corrected range_m=",
        extrema(errs),
        " median_PDOP=",
        isempty(pdops) ? NaN : median(pdops),
    )
end
length(ARGS) == 4 ||
    error("usage: replay_pvt.jl <bits.log> <latitude_deg> <longitude_deg> <height_m>")
replay(ARGS[1], LLA(parse.(Float64, ARGS[2:4])...))
