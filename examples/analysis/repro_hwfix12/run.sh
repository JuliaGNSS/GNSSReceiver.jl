#!/usr/bin/env bash
set -euo pipefail
repro_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
receiver_dir=$(cd "$repro_dir/../../.." && pwd)
mkdir -p "$repro_dir/runs"
run_dir=$(mktemp -d "$repro_dir/runs/run-XXXXXX")
cp "$receiver_dir/examples/hardware_correlator_position_fix.jl" "$run_dir/receiver.jl"
ln -s "$receiver_dir/examples/analysis" "$run_dir/analysis"
echo "Run directory: $run_dir"
cd "$run_dir"
export HWFIX_PROC_POOL=${HWFIX_PROC_POOL:-interactive}
export HWFIX_ACQ_EVERY=${HWFIX_ACQ_EVERY:-60}
export HWFIX_BITLOG=${HWFIX_BITLOG:-bits.log}
exec "${JULIA:-julia}" -t 6,3 --project="$repro_dir" receiver.jl "$@"
