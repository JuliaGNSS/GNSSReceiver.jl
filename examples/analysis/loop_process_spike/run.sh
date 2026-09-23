#!/usr/bin/env bash
# Milestone 0 trim spike (docs/plans/2026-09-22-loop-process.md). Runs on the
# target (aarch64 Orin) with Julia 1.13 and the JuliaC app.
#   ./run.sh                 build every rung
#   ./run.sh device 5        build + run the device rung for 5 s against the board
set -uo pipefail
cd "$(dirname "$0")"
export PATH="$HOME/.juliaup/bin:$HOME/.julia/bin:$PATH"
JULIA="${JULIA:-julia +1.13}"
JULIAC="${JULIAC:-juliac}"
mkdir -p build results
echo "julia:  $($JULIA --version)" | tee results/environment.txt
echo "juliac: $($JULIAC --version 2>&1 | head -1)" | tee -a results/environment.txt
echo "uname:  $(uname -m) $(uname -r)" | tee -a results/environment.txt
echo "date:   $(date -Is)" | tee -a results/environment.txt

EXPERIMENTAL=""
$JULIAC --help 2>&1 | grep -q -- "--experimental" && EXPERIMENTAL="--experimental"

build_one() {
    local src="$1" name="$2"
    local t0=$(date +%s)
    rm -f "build/$name"
    ( cd build && $JULIAC --output-exe "$name" --project .. --trim=safe $EXPERIMENTAL "../$src" ) > "results/$name.safe.log" 2>&1
    local t1=$(date +%s)
    local errors=$(grep -c "^Verifier error" "results/$name.safe.log" | tr -dc '0-9')
    if [ -x "build/$name" ]; then
        echo "$name: built in $((t1-t0)) s, $(stat -c%s build/$name) bytes, verifier errors: ${errors:-0}" | tee -a results/summary.txt
    else
        echo "$name: FAILED in $((t1-t0)) s, verifier errors: ${errors:-0}" | tee -a results/summary.txt
        grep -m 12 -A 3 "^Verifier error" "results/$name.safe.log" | head -60
    fi
}

: > results/summary.txt
if [ "${1:-all}" = all ]; then
    $JULIA --project=. -e 'using Pkg; Pkg.instantiate(); Pkg.precompile()' > results/instantiate.log 2>&1 || { tail -20 results/instantiate.log; exit 1; }
    build_one rungs/rung_fileread.jl rung_fileread
    build_one rungs/rung_device.jl rung_device
    build_one rungs/rung_tracking_fold.jl rung_tracking_fold
    # Reference output of the estimator rung under the JIT, for the equivalence check.
    $JULIA --project=. rungs/rung_tracking_fold.jl > results/rung_tracking_fold.jit.out 2> results/rung_tracking_fold.jit.log
    if [ -x build/rung_tracking_fold ]; then
        ./build/rung_tracking_fold > results/rung_tracking_fold.trim.out 2>&1
        if diff -q results/rung_tracking_fold.jit.out results/rung_tracking_fold.trim.out > /dev/null; then
            echo "rung_tracking_fold: trimmed output matches JIT" | tee -a results/summary.txt
        else
            echo "rung_tracking_fold: OUTPUT DIFFERS" | tee -a results/summary.txt
            diff results/rung_tracking_fold.jit.out results/rung_tracking_fold.trim.out
        fi
    fi
elif [ "$1" = device ]; then
    CSR="${CSR:?set CSR=path/to/csr.csv}"
    SECONDS_="${2:-5}"
    if pgrep -x m2sdr_record > /dev/null; then echo "m2sdr_record already running; refusing" >&2; exit 1; fi
    m2sdr_record -c 0 -q - 0 > /dev/null 2> results/recorder.log &
    REC=$!
    sleep 1
    ./build/rung_device "$CSR" "$SECONDS_" 4e6 /dev/m2sdr1 50 2>&1 | tee results/rung_device.run.txt
    kill $REC 2>/dev/null; wait $REC 2>/dev/null
fi
