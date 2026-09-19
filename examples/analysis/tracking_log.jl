# Shared by the analysis scripts in this directory: one CSV row per tracked
# satellite per processing chunk, taken from the receiver state through
# `receive`'s `extract` hook.
#
# Columns: runtime [s], prn, carrier_doppler [Hz], code_doppler [Hz],
# code_phase [chips], prompt_re, prompt_im (the last fully integrated filtered
# prompt), n_records (records folded this chunk), cn0 [dBHz], bit_found,
# in_lock, time_in_lock [s], then the link's counters at the time of the row
# (zero for a software run): stale dumps, lost-record gaps, skipped epochs,
# dropped NCO updates, the device's newest sample index and the host's consumed
# sample count.
#
# `snapshot` runs inside the processing task and must copy everything out: the
# `ReceiverState` it sees is mutated in place by the next chunk.

using Printf
using Unitful
using Unitful: Hz, s, ustrip
using Tracking
using GNSSReceiver

cn0_db(cn0) = 10 * log10(Unitful.linear(cn0) / Hz)

const CSV_HEADER = "runtime,prn,carrier_doppler,code_doppler,code_phase,prompt_re,prompt_im," *
                   "n_records,cn0,bit_found,in_lock,time_in_lock," *
                   "stale,lost_gaps,skipped,dropped_nco,latest_idx,samples_consumed"

# One row per tracked satellite. Pass this as `extract` to `receive`.
function snapshot(rs)
    rows = NamedTuple[]
    for (group_key, dict) in pairs(rs.receiver_sat_states)
        for sat in Tracking.get_sat_states(rs.track_state, group_key)
            prn = get_prn(sat)
            ts = Tracking.get_signals(sat)[1]
            p = get_last_fully_integrated_filtered_prompt(ts)
            rss = haskey(dict, prn) ? dict[prn] : nothing
            push!(rows, (;
                runtime = ustrip(s, rs.runtime),
                prn,
                carrier_doppler = ustrip(Hz, get_carrier_doppler(sat)),
                code_doppler = ustrip(Hz, get_code_doppler(sat)),
                code_phase = get_code_phase(sat),
                prompt_re = real(p),
                prompt_im = imag(p),
                n_records = length(get_filtered_prompts(rs.track_state, group_key, prn, 1)),
                cn0 = cn0_db(estimate_cn0(sat, 1)),
                bit_found = Tracking.get_bit_buffer(ts).found,
                in_lock = isnothing(rss) ? false : GNSSReceiver.is_in_lock(rss),
                time_in_lock = isnothing(rss) ? NaN : ustrip(s, rss.time_in_lock),
            ))
        end
    end
    rows
end

# Write one chunk's rows. `link` is the `HardwareCorrelatorLink` of a hardware
# run, or `nothing`.
function write_rows(io, rows, link)
    stale, lost, skipped, dropped, latest, consumed =
        isnothing(link) ? (0, 0, 0, 0, 0, 0) :
        (link.stale_dumps, link.lost_record_gaps, link.skipped_epochs,
         link.dropped_nco_updates, link.latest_sample_index, link.samples_consumed)
    for r in rows
        @printf(io, "%.4f,%d,%.3f,%.5f,%.4f,%.1f,%.1f,%d,%.2f,%d,%d,%.3f,%d,%d,%d,%d,%d,%d\n",
            r.runtime, r.prn, r.carrier_doppler, r.code_doppler, r.code_phase,
            r.prompt_re, r.prompt_im, r.n_records, r.cn0, r.bit_found, r.in_lock,
            r.time_in_lock, stale, lost, skipped, dropped, latest, consumed)
    end
end

locked_summary(rows) = join(
    (@sprintf("%d:%.0f%s", r.prn, r.cn0, r.bit_found ? "b" : "") for r in rows if r.in_lock),
    " ",
)

# The tracking-loop estimator for a `MODE` argument: `std` is Tracking's
# conventional FLL-assisted loop, `nco` the delay-aware `NCOReferencedPLLAndDLL`
# (the hardware receiver's default), `nconp` its negative control (re-based on
# the applied word, no landing prediction). `pll_hz` is the carrier bandwidth,
# referenced to one code period.
function estimator_for(mode, pll_hz)
    bw = pll_hz * Hz
    mode == "std" ? Tracking.ConventionalAssistedPLLAndDLL(; carrier_loop_filter_bandwidth = bw) :
    mode == "nco" ? NCOReferencedPLLAndDLL(; carrier_loop_filter_bandwidth = bw) :
    mode == "nconp" ? NCOReferencedPLLAndDLL(; carrier_loop_filter_bandwidth = bw, predict_landing = false) :
    error("unknown estimator mode $mode (std | nco | nconp)")
end
