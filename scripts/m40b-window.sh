#!/bin/bash
# yack: the M4.0b window (swarm/done/m4-rehost-plan.md) in one builder request, m22-window style:
# pre build + copy, apply scripts/m40b.py to crates/yack-cadrs (delete the native modules yack
# can't reach), post build + yack-cadrs tests + copy, then a digest pair (pre records into a
# private dir, post checks against it), with a hash guard on every other frozen crate. On any
# failure after the apply it restores the files the apply touched, so the tree builds.
# usage: bash scripts/m40b-window.sh            (staged; the lead schedules it between rounds)
set -u
TAG=M40B; tag=m40b
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; C=$Y/crates/yack-cadrs
cd $Y
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench"
others() { { git diff -- $FC ':!crates/yack-cadrs'; git status --short --untracked-files=all $FC | grep -v yack-cadrs; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

if [ -n "$(git status --short -- crates/yack-cadrs)" ]; then echo "$TAG ABORT: crates/yack-cadrs has uncommitted changes"; git status --short -- crates/yack-cadrs; exit 1; fi
O1=$(others); PFP=$(fp)
build || { echo "$TAG pre build failed"; exit 1; }
snap $tag-pre-$PFP; echo "$TAG PRE=$PFP others=$O1"

# The apply deletes files and edits others: back up the crate's src, tests and IMPORT.md.
BK=$(mktemp -d); cp -a $C/src $C/tests $C/IMPORT.md $BK/
restore() {
  rm -rf $C/src $C/tests; cp -a $BK/src $BK/tests $C/; cp -a $BK/IMPORT.md $C/IMPORT.md
  echo "$TAG restored crates/yack-cadrs (src, tests, IMPORT.md)"
}
M40B_ROOT=$C python3 scripts/m40b.py || { restore; exit 1; }
QFP=$(fp)
build || { echo "$TAG post build failed"; restore; exit 1; }
cargo test --release -p yack-cadrs --no-fail-fast || { echo "$TAG yack-cadrs tests fail"; restore; exit 1; }
snap $tag-post-$QFP; O2=$(others)
echo "$TAG POST=$QFP others=$O2"
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi

# The digest pair: pure deletion of code yack never reaches, so every digest is the same.
D=/home/brenn/github/yack-ref/tmp/$tag-digests-$PFP; rm -rf $D; mkdir -p $D
nice -n 10 $R/$tag-pre-$PFP/xtask digest --threads 6 --bin-dir $R/$tag-pre-$PFP --tree $PFP --dir $D || { echo "$TAG pre digest failed"; exit 4; }
nice -n 10 $R/$tag-post-$QFP/xtask digest --check --threads 6 --bin-dir $R/$tag-post-$QFP --tree $QFP --dir $D || { echo "$TAG DIGESTS DIFFER (or the check failed): do not commit"; exit 5; }
echo "$TAG WINDOW OK pre=$PFP post=$QFP others=$O1 digests identical"
