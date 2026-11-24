#!/bin/bash
# yack: w-m11 names-only proof, inside the builder: builds yack-bench with springrep's edit.rs as
# it stands (post) and as committed (pre), copies both out, and restores edit.rs exactly.
# Then, outside the builder: bash scripts/w-m11-names-compare.sh
set -u
Y=/home/brenn/github/yack
cd $Y
# swarm/FREEZE: held for as long as edit.rs is swapped in the shared tree, released on every exit
# path; another window's freeze is left alone and this one refuses to start.
if [ -e swarm/FREEZE ]; then echo "w-m11-names ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "w-m11-names window since $(date '+%F %T')" > swarm/FREEZE
F=crates/yack-springrep/src/edit.rs; BK=$(mktemp); cp -a $F $BK
O=/home/brenn/github/yack-ref/bin/w-m11-names; mkdir -p $O
trap 'cp -a $BK $Y/$F; touch $Y/$F; rm -f $Y/swarm/FREEZE' EXIT; trap 'exit 130' INT TERM HUP
cargo build --release -p yack-bench && cp bin/release/yack-bench $O/yack-bench-post || exit 1
git show HEAD:$F > $F.tmp && mv $F.tmp $F; touch $F
cargo build --release -p yack-bench && cp bin/release/yack-bench $O/yack-bench-pre || exit 1
cp -a $BK $F; touch $F
cmp -s $BK $F && echo "EDIT restored identical"
