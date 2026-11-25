#!/bin/bash
# yack: M4.8: `xtask bench --cases chamfer/` at this tree, its result kept as swarm/res/m48-bench-TAG.json (the harness overwrites bin/bench/yack.json).
# usage (a builder request): bash scripts/m48-bench.sh TAG
set -uo pipefail
tag=${1:?tag}
cd /home/brenn/github/yack
echo "M48 BENCH $tag tree $(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"
echo "M48 BENCH $tag chamfer_angle rungs, in order: $(grep -n 'glue::chamfer_angle\|engine_blend::chamfer_angle' crates/yack/src/blend.rs | sed 's/^\([0-9]*\):.*name: "\([a-z]*\)".*/\2@\1/' | tr '\n' ' ')"
cargo xtask bench --cases 'chamfer/*'; rc=$?
cp bin/bench/yack.json swarm/res/m48-bench-$tag.json
echo "M48 BENCH $tag rc=$rc kept swarm/res/m48-bench-$tag.json"
exit $rc
