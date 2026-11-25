#!/bin/bash
# yack: w-m11 perf #2 step 1 (Seeds::nearest bound pass), inside the builder: yack-bench and
# yack-conformance built with yack-geom's seeds.rs as it stands (post) and as committed (pre),
# copied out, seeds.rs restored exactly. Then, outside the builder and under swarm/DIGEST.lock:
#   bash scripts/w-m11-seeds-compare.sh
set -u
Y=/home/brenn/github/yack
cd $Y
if [ -e swarm/FREEZE ]; then echo "w-m11-seeds ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "w-m11-seeds window since $(date '+%F %T')" > swarm/FREEZE
S=crates/yack-geom/src/nurbs/seeds.rs; BS=$(mktemp); cp -a $S $BS
O=/home/brenn/github/yack-ref/bin/w-m11-seeds; mkdir -p $O/pre $O/post
trap 'cp -a $BS $Y/$S; touch $Y/$S; rm -f $Y/swarm/FREEZE' EXIT; trap 'exit 130' INT TERM HUP
cargo build --release -p yack-bench -p yack-conformance || exit 1
cp bin/release/yack-bench bin/release/yack-conformance $O/post/
git show HEAD:$S > $S.tmp && mv $S.tmp $S; touch $S
cargo build --release -p yack-bench -p yack-conformance || exit 1
cp bin/release/yack-bench bin/release/yack-conformance $O/pre/
cp -a $BS $S; touch $S
cmp -s $BS $S && echo "SEEDS restored identical"
