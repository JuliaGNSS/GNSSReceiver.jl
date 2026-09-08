#!/usr/bin/env bash
set -euo pipefail
repro_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
mkdir -p "$repro_dir/deps"
checkout_dependency() {
    local name=$1 url=$2 revision=$3
    local target="$repro_dir/deps/$name"
    if [[ ! -d "$target/.git" ]]; then
        git clone --no-checkout "$url" "$target"
    elif [[ -n $(git -C "$target" status --porcelain) ]]; then
        echo "Refusing to overwrite changes in $target" >&2
        exit 1
    fi
    git -C "$target" fetch origin "$revision"
    git -C "$target" checkout --detach "$revision"
}
checkout_dependency Tracking https://github.com/JuliaGNSS/Tracking.jl.git a2ff103b1c7657e6db8e177c59f4fe7f8d84c5ac
checkout_dependency GNSSDecoder https://github.com/JuliaGNSS/GNSSDecoder.jl.git 28725906711265c341b59542bb4a5dabc99a9e87
"${JULIA:-julia}" --project="$repro_dir" -e '
    VERSION == v"1.12.6" || error("This measured environment requires Julia 1.12.6")
    using Pkg
    Pkg.instantiate()
'
