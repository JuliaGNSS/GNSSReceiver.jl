# ─────────────────────────────────────────────────────────────────────────────
# The per-signal support/evidence matrix.
#
# Issue #130's completion criteria ask for "a per-signal support/evidence matrix
# [that] distinguishes software, simulated FPGA, hardware replay and live RF
# validation", and for no blanket support claim that "rests on metadata alone".
# This module is that matrix, and it is a *test* artifact rather than a document
# on purpose: every `supported` cell has to name the evidence behind it, and
# test/signal_validation.jl fails if a cell claims evidence that no check
# produced — or if a check produced evidence that no cell claims.
#
# The honest state of most cells today is `untested` or `blocked`. Steps 1-8 of
# the roadmap have not landed, so there is nothing to have tested; recording that
# is the point of the artifact, and filling a cell in is what each of those steps
# does when it lands.
#
# Depends only on GNSSSignals (for the signal scope) — everything receiver-side
# lives in the checks, not in the table.
# ─────────────────────────────────────────────────────────────────────────────

module SignalSupport

using GNSSSignals

export ROLES,
    EVIDENCE_LEVELS,
    STATUSES,
    SupportEntry,
    SIGNAL_SCOPE,
    MATRIX,
    SOURCES,
    reason_text,
    reason_keys,
    scope_signals,
    entry,
    supported,
    untested,
    blocked,
    not_applicable,
    record_evidence!,
    recorded_evidence,
    render_markdown,
    normalize_markdown

# ─────────────────────────────────────────────────────────────────────────────
# Vocabulary
# ─────────────────────────────────────────────────────────────────────────────

"""
The roles a signal can fulfil, exactly as issue #135 names them. A signal is not
"supported" or "unsupported" as a whole: a pilot never decodes navigation data and
a data component is rarely what a receiver ranges on, so support is a statement
about a (signal, role) pair.
"""
const ROLES =
    (:replica, :acquisition_handover, :tracking, :secondary_sync, :data_decode, :pvt)

const ROLE_DESCRIPTIONS = Dict(
    :replica =>
        "The code replica is generated correctly: unit-power, balanced, correlating " *
        "to a single dominant peak at the stated code phase, isolated from other PRNs, " *
        "and (where there is one) carrying the right secondary code.",
    :acquisition_handover =>
        "A cold acquisition detects the satellite and hands over a code phase and " *
        "Doppler accurate enough for the tracking loops to pull in.",
    :tracking => "The code and carrier loops hold lock and keep the replica aligned.",
    :secondary_sync =>
        "The receiver synchronises to the secondary (overlay) code, so integration " *
        "can extend past one primary code period.",
    :data_decode => "The navigation message is demodulated and decoded into a usable ephemeris.",
    :pvt => "The satellite contributes valid measurements to a position/velocity/time fix.",
)

"""
The validation paths, weakest to strongest. `:software` covers the software
receive path — over synthetic samples from the reference harness or over a
recording; the `source` says which. The other three are the paths issue #130 asks
to be distinguished, and none of them can produce evidence before the gateware
steps land.
"""
const EVIDENCE_LEVELS = (:none, :software, :simulated_fpga, :hardware_replay, :live_rf)

const EVIDENCE_DESCRIPTIONS = Dict(
    :none => "No evidence.",
    :software => "The software receive path, over harness samples or a recording.",
    :simulated_fpga => "The simulated hardware correlator of test/simulated_fpga.jl, fed the same samples.",
    :hardware_replay => "Recorded samples replayed through real gateware.",
    :live_rf => "A live antenna through the hardware-correlator path.",
)

"""
`:supported` — evidence exists and is named. `:untested` — the capability may well
be there, but nothing has demonstrated it. `:blocked` — the capability does not
exist yet, and the reason names what it waits on. `:not_applicable` — the role is
meaningless for this signal (navigation decoding on a pilot, secondary
synchronisation on a signal with no secondary code).
"""
const STATUSES = (:supported, :untested, :blocked, :not_applicable)

"""
    SupportEntry(status, evidence, source, reason)

One cell. `source` names the evidence (a key into [`SOURCES`](@ref)) and is empty
unless `status` is `:supported`; `reason` names why the cell is not supported (a
key into the reason legend, see [`reason_text`](@ref)) and is empty when it is.
"""
struct SupportEntry
    status::Symbol
    evidence::Symbol
    source::Symbol
    reason::Symbol
end

"""
    supported(evidence, source) -> SupportEntry

A cell backed by evidence at level `evidence` from `source`.
"""
supported(evidence::Symbol, source::Symbol) =
    SupportEntry(:supported, evidence, source, Symbol(""))

"""
    untested(reason) -> SupportEntry

A cell with no evidence yet; `reason` says what is missing.
"""
untested(reason::Symbol) = SupportEntry(:untested, :none, Symbol(""), reason)

"""
    blocked(reason) -> SupportEntry

A cell whose capability does not exist yet; `reason` names the step it waits on.
"""
blocked(reason::Symbol) = SupportEntry(:blocked, :none, Symbol(""), reason)

"""
    not_applicable(reason) -> SupportEntry

A cell whose role is meaningless for this signal.
"""
not_applicable(reason::Symbol) = SupportEntry(:not_applicable, :none, Symbol(""), reason)

# ─────────────────────────────────────────────────────────────────────────────
# Evidence sources and reasons
#
# Both are keyed, so a cell is a short token and the explanation is written once.
# test/signal_validation.jl checks that every key used exists and that every key
# defined is used, which is what stops the legend from rotting as cells move.
# ─────────────────────────────────────────────────────────────────────────────

"""
Where a `:supported` cell's evidence comes from. A key starting with `harness_` is
produced *at run time* by the checks in test/signal_validation.jl and is verified
against [`recorded_evidence`](@ref); a key starting with `live_` is a **field
record** — a live-RF run on real hardware, which no test suite can reproduce, so it
names the script that was run and the file that records the run (both checked to
exist, neither wired into test/runtests.jl); any other key names a test file, which
is checked to exist and to be wired into test/runtests.jl.
"""
const SOURCES = Dict(
    :harness_replica => (
        file = "test/signal_validation.jl",
        text = "The reference harness's per-signal replica check: unit code power, " *
               "code balance, peak alignment against a fractional code phase and a " *
               "Doppler, main-peak dominance, cross-PRN isolation and secondary-code " *
               "agreement, all against `ReferenceHarness`' noise-free reference.",
    ),
    :harness_acquisition => (
        file = "test/signal_validation.jl",
        text = "The reference harness's per-signal acquisition check: a cold " *
               "`acquire!` over harness samples has to detect the satellite and hand " *
               "over a code phase and Doppler within the stated tolerances.",
    ),
    :harness_receive => (
        file = "test/signal_validation.jl",
        text = "A full `receive` run over harness samples: the satellite has to be " *
               "acquired, tracked and held in lock with its C/N₀ within tolerance of " *
               "the case's.",
    ),
    :harness_hardware_overlay => (
        file = "test/secondary_code_removal.jl",
        text = "GPS L5I through the simulated hardware correlator of " *
               "`test/simulated_fpga.jl` over reference-harness samples: the device " *
               "replicates the primary code only, `Tracking`'s own detector finds the " *
               "NH10 overlay, the link then removes it from every dump, and the " *
               "decoded symbols are the ones the harness transmitted — at close to the " *
               "full ten blocks of energy per symbol rather than the overlay's own " *
               "sum of two.",
    ),
    :live_m2sdr_l1_20260918 => (
        file = "examples/analysis/hardware_live_m2sdr.md",
        text = "Live sky on orin2 through the LiteX-M2SDR hardware correlator, " *
               "2026-09-18, gateware `gnss_m2sdr_m2_x1_ch4_ant1_code4092_tap5_sub12_" *
               "placeSpread` (four channels, five taps, 4092-chip code memory, sub-chip " *
               "replicas), fs = 4 MS/s, one antenna seeing about a quarter of the " *
               "hemisphere, run by `examples/analysis/hardware_live_m2sdr.jl`. GPS L1 " *
               "C/A on three-tap channels held PRN 14 at 50–52 dBHz for 300 s and, on the " *
               "six-channel build with the link's record sizing fixed, decoded four LNAV " *
               "ephemerides and produced a GPS fix after 205 s; Galileo " *
               "E1B (BOC(1,1) replica on five-tap channels) held four satellites at " *
               "36–47 dBHz, decoded their I/NAV ephemerides and produced a Galileo-only " *
               "position fix after 40 s. The log excerpts and the counters are in the " *
               "record file; the run is not reproducible without the board.",
    ),
    :ion_recording => (
        file = "test/ion_rtlsdr_integration.jl",
        text = "The 60 s ION RTL-SDR live-sky GPS L1 recording through the software " *
               "receive path, asserted against a captured baseline: eleven healthy " *
               "satellites, decoded ephemerides, and a PVT fix repeatable to one metre " *
               "per ECEF component.",
    ),
)

# Why a cell is not `:supported`. Kept short and specific: a reason that does not
# name what would change it is not a reason. Read through `reason_text` /
# `reason_keys`, which is also how the integrity testset checks that every key used
# is defined and every key defined is used.
const _REASON_TEXT = Dict(
    :pilot_no_data =>
        "A pilot component carries no navigation data (`get_data_frequency` is 0 Hz), " *
        "so there is no message to decode. It contributes to a fix through its " *
        "`CombinedSignal` pairing with the data component, not on its own.",
    :no_secondary_code =>
        "The signal has no secondary code (`get_secondary_code_length` is 1), so there " *
        "is nothing to synchronise to.",
    :tracking_sweep_pending =>
        "No per-signal tracking sweep exists yet. The harness can generate the case; " *
        "what is missing is the run and its baseline — part of step 9 itself, and only " *
        "meaningful once the loops it exercises are the ones the roadmap settles on.",
    :secondary_sync_sweep_pending =>
        "The receiver's own secondary-code synchronisation is not swept per signal yet. " *
        "The harness does verify that the secondary code is recoverable from the " *
        "reference (it is part of the replica check), which is the prerequisite, not " *
        "the capability.",
    :decode_sweep_pending =>
        "A decoder exists for this signal (`GNSSDecoderState` has a method), but nothing " *
        "has demonstrated a decode. Synthetic samples cannot: the harness modulates a " *
        "reproducible bit stream, not a navigation message with a preamble, parity and " *
        "an ephemeris. Evidence has to come from a recording or live sky.",
    :pvt_sweep_pending =>
        "No fix has been computed with this signal contributing. Follows the decode " *
        "sweep for a data component, and the tracking sweep for a pilot, which " *
        "contributes pseudoranges through its `CombinedSignal` pairing.",
    :l2cl_tracking_sweep_pending =>
        "The hardware path's *timing and accounting* for this signal are validated and " *
        "the code loop does pull in: test/partial_primary_records.jl runs GPS L2CL " *
        "through the simulated correlator of test/simulated_fpga.jl dumping inside its " *
        "1.5 s primary code period, the summed dumps reproduce the harness's " *
        "`reference_correlation` over the same span, the loops are handed a record " *
        "every `max_integration_time` instead of once per 1.5 s code wrap, and no short " *
        "record is counted as a completed code period. That is not a tracking sweep. " *
        "Nothing has yet shown the *carrier* loop holding lock on L2CL — in the same " *
        "simulated run its Doppler estimate barely moves against a deliberate offset, " *
        "while GPS L1 C/A through the identical harness converges — and whether that is " *
        "a property of the signal, of the 20 ms coherent window a 1.5 s code forces, or " *
        "of `Tracking`'s per-signal support is the subject of the software-support " *
        "audit (JuliaGNSS/Tracking.jl#236) and the per-signal tracking sweep of step 9, " *
        "not of the record accounting.",
    :acquisition_window_too_long =>
        "One coherent acquisition window is a whole primary code period, and this " *
        "signal's is 1.5 s — 3 million samples at four samples per chip. So L2CL is " *
        "never the signal a receiver acquires: `acquisition_signal` falls back to the " *
        "pairing's L2CM data component above ~0.67 Hz Doppler resolution, and L2CL is " *
        "handed over from L2CM's code phase rather than searched for. A property of the " *
        "signal, not a gap in the evidence.",
)

# ─────────────────────────────────────────────────────────────────────────────
# The signal scope
# ─────────────────────────────────────────────────────────────────────────────

"""
The signal families of issue #130's scope table, in its order. The matrix has to
cover exactly the concrete signals GNSSSignals exports (test/signal_validation.jl
checks it), so a GNSSSignals release that adds a signal fails the suite until the
matrix is extended — which is what "extend this matrix when the supported
GNSSSignals release adds signals" means in practice.
"""
const SIGNAL_SCOPE = [
    "GPS L1 C/A" => [GPSL1CA()],
    "GPS L1C" => [GPSL1C_D(), GPSL1C_P()],
    "GPS L2C" => [GPSL2CM(), GPSL2CL()],
    "GPS L5" => [GPSL5I(), GPSL5Q()],
    "Galileo E1" =>
        [GalileoE1B(), GalileoE1C(), GalileoE1B_BOC11(), GalileoE1C_BOC11()],
    "Galileo E5" =>
        [GalileoE5aI(), GalileoE5aQ(), GalileoE5aQP(), GalileoE5bI(), GalileoE5bQ()],
    "Galileo E6" => [GalileoE6B(), GalileoE6C()],
    "BeiDou legacy/B2" =>
        [BeiDouB1I(), BeiDouB3I(), BeiDouB2bI(), BeiDouB2aI(), BeiDouB2aQ()],
    "BeiDou B1C" => [BeiDouB1C_D(), BeiDouB1C_P()],
]

"""
    scope_signals() -> Vector{AbstractGNSSSignal}

Every signal of [`SIGNAL_SCOPE`](@ref), flattened, in table order.
"""
scope_signals() = reduce(vcat, last.(SIGNAL_SCOPE))

# ─────────────────────────────────────────────────────────────────────────────
# The runtime evidence registry
# ─────────────────────────────────────────────────────────────────────────────

const _RECORDED = Set{Tuple{Symbol,Symbol,Symbol}}()

"""
    record_evidence!(signal_id, role, source)

Record that the check identified by `source` ran for `(signal_id, role)` and
passed. Called by test/signal_validation.jl *after* its assertions, so a failing
check records nothing and the matrix's claim is then left unbacked — which the
integrity testset reports.
"""
function record_evidence!(signal_id::Symbol, role::Symbol, source::Symbol)
    role in ROLES || throw(ArgumentError("unknown role $role"))
    haskey(SOURCES, source) || throw(ArgumentError("unknown evidence source $source"))
    push!(_RECORDED, (signal_id, role, source))
    nothing
end

"""
    recorded_evidence() -> Set{Tuple{Symbol,Symbol,Symbol}}

Every `(signal_id, role, source)` recorded so far in this run.
"""
recorded_evidence() = _RECORDED

# ─────────────────────────────────────────────────────────────────────────────
# The matrix
# ─────────────────────────────────────────────────────────────────────────────

# Shorthands for the cells that repeat across most rows, so a row reads as the
# handful of decisions it actually makes.
const _REPLICA_OK = supported(:software, :harness_replica)
const _ACQ_OK = supported(:software, :harness_acquisition)
const _TRACK_PENDING = untested(:tracking_sweep_pending)
const _SEC_NA = not_applicable(:no_secondary_code)
const _SEC_PENDING = untested(:secondary_sync_sweep_pending)
const _DECODE_NA = not_applicable(:pilot_no_data)
const _DECODE_PENDING = untested(:decode_sweep_pending)
const _PVT_PENDING = untested(:pvt_sweep_pending)

# One row per signal, keyed by `get_signal_id`. Written out in full rather than
# derived from the signal's properties: a table that computed its own statuses
# would only ever restate the metadata, which is exactly the claim issue #135
# refuses to accept.
const MATRIX = Dict{Symbol,NamedTuple{ROLES,NTuple{6,SupportEntry}}}(
    # GPS L1 C/A — the regression baseline. The only row with evidence beyond the
    # harness: the ION recording carries a real navigation message, so it is the
    # only place a decode and a fix have actually happened.
    :GPSL1CA => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        # Also tracked live through the hardware correlator on 2026-09-18 (see
        # `live_m2sdr_l1_20260918`); the cell keeps the claim CI can reproduce.
        tracking = supported(:software, :harness_receive),
        secondary_sync = _SEC_NA,
        data_decode = supported(:software, :ion_recording),
        pvt = supported(:software, :ion_recording),
    ),
    :GPSL1C_D => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_NA,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :GPSL1C_P => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    :GPSL2CM => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_NA,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    # GPS L2CL is where the long-code timing work of issue #133 is demonstrated —
    # its 1.5 s primary code period is the one in the scope that a
    # per-code-period dump contract cannot serve at all. The timing and
    # accounting are validated; the tracking cell is not, and says why.
    :GPSL2CL => (
        replica = _REPLICA_OK,
        acquisition_handover = not_applicable(:acquisition_window_too_long),
        tracking = untested(:l2cl_tracking_sweep_pending),
        secondary_sync = _SEC_NA,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    # GPS L5I is where the hardware path's overlay removal is demonstrated end to
    # end (issue #132): the only secondary-code row so far whose synchronisation
    # has actually been run, and the only cell with simulated-FPGA evidence.
    :GPSL5I => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = supported(:simulated_fpga, :harness_hardware_overlay),
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :GPSL5Q => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    :GalileoE1B => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_NA,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :GalileoE1C => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    # Galileo E1B with the BOC(1,1) replica is the first non-GPS signal through
    # the hardware correlator on sky: five-tap channels, a 4092-chip code, an
    # I/NAV decode and a Galileo-only fix (2026-09-18, see the source).
    :GalileoE1B_BOC11 => (
        replica = _REPLICA_OK,
        acquisition_handover = supported(:live_rf, :live_m2sdr_l1_20260918),
        tracking = supported(:live_rf, :live_m2sdr_l1_20260918),
        secondary_sync = _SEC_NA,
        data_decode = supported(:live_rf, :live_m2sdr_l1_20260918),
        pvt = supported(:live_rf, :live_m2sdr_l1_20260918),
    ),
    :GalileoE1C_BOC11 => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    :GalileoE5aI => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :GalileoE5aQ => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    :GalileoE5aQP => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_NA,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    :GalileoE5bI => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :GalileoE5bQ => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    :GalileoE6B => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_NA,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :GalileoE6C => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    :BeiDouB1I => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :BeiDouB3I => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :BeiDouB2bI => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_NA,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :BeiDouB2aI => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :BeiDouB2aQ => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
    :BeiDouB1C_D => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_NA,
        data_decode = _DECODE_PENDING,
        pvt = _PVT_PENDING,
    ),
    :BeiDouB1C_P => (
        replica = _REPLICA_OK,
        acquisition_handover = _ACQ_OK,
        tracking = _TRACK_PENDING,
        secondary_sync = _SEC_PENDING,
        data_decode = _DECODE_NA,
        pvt = _PVT_PENDING,
    ),
)

"""
    entry(signal_or_id, role) -> SupportEntry

The matrix cell for a signal (or its `get_signal_id`) and a role.
"""
entry(signal_id::Symbol, role::Symbol) = getfield(MATRIX[signal_id], role)
entry(signal::AbstractGNSSSignal, role::Symbol) = entry(get_signal_id(signal), role)

# ─────────────────────────────────────────────────────────────────────────────
# Rendering
# ─────────────────────────────────────────────────────────────────────────────

# The token a cell prints as: what is known, then the key that explains it. A
# supported cell shows its evidence *level* (`software` for every cell today, and
# not once the gateware steps land) and the source behind it; every other cell
# shows its status and the reason key.
function cell_text(e::SupportEntry)
    e.status === :supported && return "$(e.evidence) ($(e.source))"
    e.status === :not_applicable && return "n/a ($(e.reason))"
    "$(e.status) ($(e.reason))"
end

_role_header(role) = replace(String(role), '_' => ' ')

"""
    render_markdown() -> String

The matrix as a Markdown fragment: the per-signal table, the legend of evidence
sources, and the legend of reasons. test/signal_validation.jl regenerates this and
compares it against the block in docs/src/signal_support.md, so the documented
matrix cannot drift from the tested one.
"""
function render_markdown()
    io = IOBuffer()
    println(io, "| Family | Signal | ", join(_role_header.(ROLES), " | "), " |")
    println(io, "|", repeat("---|", 2 + length(ROLES)))
    for (family, signals) in SIGNAL_SCOPE
        for (index, signal) in enumerate(signals)
            row = MATRIX[get_signal_id(signal)]
            println(
                io,
                "| ",
                index == 1 ? family : "",
                " | `",
                get_signal_id(signal),
                "` | ",
                join((cell_text(getfield(row, role)) for role in ROLES), " | "),
                " |",
            )
        end
    end
    println(io)
    println(io, "### Roles")
    println(io)
    for role in ROLES
        println(io, "- **", _role_header(role), "** — ", ROLE_DESCRIPTIONS[role])
    end
    println(io)
    println(io, "### Evidence sources")
    println(io)
    for key in sort(collect(keys(SOURCES)))
        source = SOURCES[key]
        println(io, "- **`", key, "`** (`", source.file, "`) — ", source.text)
    end
    println(io)
    println(io, "### Why a cell is not supported")
    println(io)
    for key in sort(collect(keys(_REASON_TEXT)))
        println(io, "- **`", key, "`** — ", _REASON_TEXT[key])
    end
    String(take!(io))
end

"""
    normalize_markdown(text) -> String

`text` reduced to what it *says*, so a comparison against the rendered matrix is not
also a comparison against a formatter's taste.

`JuliaFormatter` formats Markdown too (`format_markdown = true` in this repo's
`.JuliaFormatter.toml`), and running it over the documentation pads a table's columns
into alignment, writes the separator row as `|:--- |` and re-marks bullets. None of that
changes a single cell, so none of it should fail the sync test — but a raw string
comparison would, and the next contributor to run the formatter would be left staring
at a diff of spaces.

So: blank lines go, leading and trailing whitespace goes, runs of whitespace collapse,
`*` and `-` bullets become one marker, table cells are stripped, and a table's alignment
row is dropped entirely. What survives is the content.
"""
function normalize_markdown(text::AbstractString)
    lines = String[]
    for raw in eachsplit(text, '\n')
        line = strip(raw)
        isempty(line) && continue
        if startswith(line, "|")
            cells = strip.(split(strip(line, '|'), '|'))
            all(cell -> occursin(r"^:?-+:?$", cell), cells) && continue
            push!(lines, join(cells, " | "))
        else
            line = replace(line, r"\s+" => " ")
            startswith(line, "* ") && (line = "- " * line[3:end])
            push!(lines, line)
        end
    end
    join(lines, "\n")
end

"""
    reason_text(key) -> String

The explanation behind a reason key.
"""
reason_text(key::Symbol) = _REASON_TEXT[key]

"""
    reason_keys() -> Vector{Symbol}

Every reason key the legend defines.
"""
reason_keys() = collect(keys(_REASON_TEXT))

end # module SignalSupport
