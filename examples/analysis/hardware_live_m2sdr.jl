# Live run of the hardware-correlator receiver on the LiteX-M2SDR with a
# five-tap gnss-m2sdr gateware (four or six channels; HW_BUILD picks the build). This is the
# script behind the `live_m2sdr_l1_20260918` field record of the signal-support
# matrix (see hardware_live_m2sdr.md next to it).
#
# Usage: julia -t 6,4 --project=<env with GNSSReceiver + GNSSM2SDR + Geodesy> \
#            hardware_live_m2sdr.jl MODE [MAX_SECONDS] [SECONDS_AFTER_FIX]
#   MODE  l1ca    -- GPS L1 C/A only (the regression baseline, three-tap channels)
#         e1      -- GPS L1 C/A + Galileo E1 pilot/data pair (BOC(1,1) replicas, five taps)
#         e1only  -- Galileo E1 pilot/data pair alone (a pair takes two channels, so on
#                    a four-channel build GPS would otherwise leave none for it)
#         e1bonly -- Galileo E1B data component alone, one channel per satellite
#         mixb    -- GPS L1 C/A (three taps) next to Galileo E1B (five taps), one
#                    channel per satellite: the mixed-layout case in one bank
#         e1cboc  -- GPS L1 C/A + Galileo E1 CBOC (needs fs >= 12.276 MHz)
#   HW_FS     sampling rate in Hz (default 4e6); the RF front end must match, e.g.
#             m2sdr_rf --sample-rate 4000000 --rx-freq 1575420000 --rx-gain 60 --bandwidth 4000000
#   HW_DELAY  the link's feedback_delay_epochs (default 2)
#   HW_TRACE  1 prints the link's epoch grid every 5 s
#   HW_BUILD  gateware build directory under ~/gnss-m2sdr/build (default: the six-channel build)
#   HW_ACQ_EVERY  seconds between acquisition scans (default 60)
#   HW_GPS_PRNS  comma-separated GPS PRNs to search (default: all), e.g. 14,5 in a
#             mixed run so a four-channel bank keeps channels for Galileo
#
# `noise_source = :samples` meters the C/N₀ noise density off the raw stream, so
# no hardware channel is spent on the open-loop reference and all four can hold
# satellites -- the minimum for a fix.

using Printf
using Unitful
using Unitful: Hz, ms, s, dBHz, ustrip
using Geodesy: LLAfromECEF, wgs84
using GNSSSignals
using GNSSReceiver
using GNSSM2SDR

const MODE = length(ARGS) >= 1 ? ARGS[1] : "l1ca"
const MAX_SECONDS = length(ARGS) >= 2 ? parse(Float64, ARGS[2]) : 240.0
const SECONDS_AFTER_FIX = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 60.0
const FS = get(ENV, "HW_FS", "4e6") |> x -> parse(Float64, x) * Hz
const CHUNK = round(Int, ustrip(Hz, FS) * 2e-3)   # 2 ms of samples per processing chunk
# HW_BUILD names the gateware build directory under ~/gnss-m2sdr/build; the
# channel count is read from its capability CSRs.
const HW_BUILD = get(ENV, "HW_BUILD", "gnss_m2sdr_m2_x1_ch6_ant1_code4092_tap5_sub12_synthPerfSpread")
const CSR_CSV = expanduser("~/gnss-m2sdr/build/$HW_BUILD/csr.csv")
const gpsl1 = GPSL1CA()

systems_for(mode) =
    mode == "l1ca" ? gpsl1 :
    mode == "e1" ? (gpsl1, CombinedSignal(GalileoE1C_BOC11(), GalileoE1B_BOC11())) :
    mode == "e1only" ? CombinedSignal(GalileoE1C_BOC11(), GalileoE1B_BOC11()) :
    mode == "e1bonly" ? GalileoE1B_BOC11() :
    mode == "mixb" ? (gpsl1, GalileoE1B_BOC11()) :
    mode == "e1cboc" ? (gpsl1, CombinedSignal(GalileoE1C(), GalileoE1B())) :
    error("unknown MODE $mode")

cn0_db(cn0) = 10 * log10(Unitful.linear(cn0) / Hz)
function sat_summary(sat_data)
    join(
        (
            # suffix: "?" not in lock, "r" ranging-ready (bit clock + code phase anchored), "h" decoded and healthy
            @sprintf("%s%d:%.0f%s%s%s", string(sys)[1], prn, cn0_db(sat.cn0), sat.is_in_lock ? "" : "?",
                     sat.is_ranging_ready ? "r" : "", sat.is_healthy ? "h" : "")
            for ((sys, prn), sat) in pairs(sat_data) if sat.is_in_lock || cn0_db(sat.cn0) > 30
        ),
        " ",
    )
end
function position_summary(pvt)
    lla = LLAfromECEF(wgs84)(pvt.position)
    @sprintf("%.6f° %.6f° %.0f m (%d sats)", lla.lat, lla.lon, lla.alt, length(pvt.sats))
end

function main()
    systems = systems_for(MODE)
    origin = let csr = LiteXCSR(CSR_CSV)
        count = sample_count(GNSSBank(csr; fs = ustrip(Hz, FS)))
        close(csr)
        count
    end
    run(ignorestatus(`pkill -x m2sdr_record`))
    sleep(0.3)
    stream = start_raw_stream(; chunk = CHUNK)
    sdr = M2SDRCorrelator(CSR_CSV, stream.channel; fs = FS)
    @info "gateware exposes $(num_hardware_channels(sdr)) channels" HW_BUILD MODE FS
    @info "capabilities" hardware_capabilities(sdr)
    # `noise_source = :samples`: the C/N₀ noise density is metered off the raw
    # stream, so no hardware channel is spent on the open-loop reference and all
    # every channel can hold a satellite.
    link = HardwareCorrelatorLink(
        sdr;
        sampling_freq = FS,
        reference_signal = MODE == "e1only" ? GalileoE1C_BOC11() :
                           MODE == "e1bonly" ? GalileoE1B_BOC11() : gpsl1,
        noise_source = :samples,
        feedback_delay_epochs = parse(Int, get(ENV, "HW_DELAY", "2")),
    )
    data = receive(
        sdr,
        systems,
        FS;
        link,
        max_meas = 2^11,
        # HW_GPS_PRNS="14,5" restricts the GPS search so a four-channel bank has
        # channels left for the other constellation in a mixed run.
        prns = haskey(ENV, "HW_GPS_PRNS") ?
               (GPS = parse.(Int, split(ENV["HW_GPS_PRNS"], ",")), Galileo = collect(1:36)) : nothing,
        acq_min_doppler_coverage = 25_000.0Hz,
        acq_coherent_integration_time = 10ms,
        acq_noncoherent_rounds = 5,
        acquire_every = parse(Float64, get(ENV, "HW_ACQ_EVERY", "60"))s,
        code_lock_cn0_threshold = 24.0dBHz,
    )
    start!(sdr; dump_source = :dma, epoch_period = 25)
    sdr.device_origin = origin
    t0 = time()
    first_fix_at = nothing
    last_fix_time = nothing
    fresh_fixes = 0
    last_print = -Inf
    best = Dict{Any,Float64}()
    for d in data
        t = time() - t0
        for (key, sat) in pairs(d.sat_data)
            sat.is_in_lock && (best[key] = max(get(best, key, -Inf), cn0_db(sat.cn0)))
        end
        if !isnothing(d.pvt.time) && !isequal(d.pvt.time, last_fix_time)
            last_fix_time = d.pvt.time
            fresh_fixes += 1
            if isnothing(first_fix_at)
                first_fix_at = t
                @info @sprintf("first fix after %.1f s: %s", t, position_summary(d.pvt))
            end
        end
        if t - last_print >= 5
            last_print = t
            @info @sprintf("t=%6.1f s  cn0[%s]  %s", t, sat_summary(d.sat_data),
                           isnothing(d.pvt.time) ? "no fix" : position_summary(d.pvt))
            if get(ENV, "HW_TRACE", "0") == "1"
                dev = sample_count(sdr.bank)
                @info @sprintf("   grid: epoch_length=%d latest_record=%d boundary=%d device_now=%d (latest lags device by %.1f ms, boundary-latest=%.1f ms) skipped=%d implausible=%d ncos=%d",
                               link.epoch_length, link.latest_sample_index, link.next_epoch_boundary, dev,
                               (dev - sdr.device_origin - link.latest_sample_index) / ustrip(Hz, FS) * 1e3,
                               (link.next_epoch_boundary - link.latest_sample_index) / ustrip(Hz, FS) * 1e3,
                               link.skipped_epochs, link.implausible_dumps, sdr.nco_commits)
            end
        end
        (!isnothing(first_fix_at) && t - first_fix_at > SECONDS_AFTER_FIX) && break
        t > MAX_SECONDS && break
    end
    close(data)
    close(stream)
    stop!(sdr)
    isnothing(sdr.reader) || wait(sdr.reader)
    isnothing(sdr.writer) || wait(sdr.writer)
    @info "dump stream: lost-record gaps $(link.lost_record_gaps), re-arm gaps $(link.rearm_gaps), " *
          "device-reported drops $(link.dropped_dumps), skipped epochs $(link.skipped_epochs), " *
          "implausible indices $(link.implausible_dumps), dropped NCO updates $(link.dropped_nco_updates), " *
          "tap-layout mismatches $(link.tap_layout_mismatches), unsupported signals $(link.unsupported_signals)"
    @info "best C/N0 per satellite while in lock" sort(collect(best); by = last, rev = true)
    if isnothing(first_fix_at)
        @warn "no position fix in $MAX_SECONDS s"
    else
        @info @sprintf("first fix after %.1f s, %d fresh solutions afterwards", first_fix_at, fresh_fixes)
    end
end
main()
