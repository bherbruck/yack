#!/bin/bash
# yack: M4.8: brep-bench `chamfer/` graded with a cadrs-first and a native-first bake-off yack-bench (built_by per case), then `xtask bench` on this tree; results kept in swarm/res/.
# usage (a builder request): bash scripts/m48-bench2.sh
# pre: yack-ref/bin/yack-bench-bakeoff-m48, built at fe208d4+75d8024b (chamfer_angle rungs cadrs, native);
# post: a bake-off build of this tree, and xtask bench's default build of it. No engine is forced.
set -uo pipefail
cd /home/brenn/github/yack
H="python3 crates/yack-bench/harness.py --case chamfer/"
echo "M48 tree $(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"
echo "M48 rungs now: $(grep -n 'glue::chamfer_angle\|engine_blend::chamfer_angle' crates/yack/src/blend.rs | sed 's/^\([0-9]*\):.*name: "\([a-z]*\)".*/\2@\1/' | tr '\n' ' ')"
$H --out swarm/res/m48-bench-pre.json --runner /home/brenn/github/yack-ref/bin/yack-bench-bakeoff-m48; echo "M48 pre rc=$?"
bash scripts/w-h4-bakebuild.sh m48post || { echo "M48 bakebuild failed"; exit 1; }
$H --out swarm/res/m48-bench-post-bakeoff.json --runner /home/brenn/github/yack-ref/bin/yack-bench-bakeoff-m48post; echo "M48 post-bakeoff rc=$?"
cargo xtask bench --cases 'chamfer/*'; rc=$?; cp bin/bench/yack.json swarm/res/m48-bench-post.json; echo "M48 post xtask rc=$rc"
exit $rc
