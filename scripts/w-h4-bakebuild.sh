#!/bin/bash
# yack: builds yack-bench with the bake-off feature, copies it out as yack-ref/bin/yack-bench-bakeoff-<tag>, then rebuilds the default yack-bench.
# usage (a builder request): bash scripts/w-h4-bakebuild.sh TAG
set -euo pipefail
tag=${1:?tag}
cd /home/brenn/github/yack
cargo build --release -p yack-bench --features bakeoff
mkdir -p /home/brenn/github/yack-ref/bin
cp bin/release/yack-bench /home/brenn/github/yack-ref/bin/yack-bench-bakeoff-$tag
cargo build --release -p yack-bench
echo "BAKEBUILD OK: yack-ref/bin/yack-bench-bakeoff-$tag"
