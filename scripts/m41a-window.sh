#!/bin/bash
# yack: the M4.1a window (swarm/done/m4-rehost-plan.md: cadrs's tolerance values into
# yack_tol::cadrs, a pure move) in one builder request, build part only: hash the other frozen
# crates, pre build + copy, apply scripts/m41a.py from the stage dir, check the result equals the
# dry run byte for byte, post build + yack-tol and yack-cadrs tests + copy, re-hash. On any failure
# after the apply it restores what it touched. The digest pair runs OUTSIDE the builder after:
# scripts/m41a-digest.sh (it prints the line).
# usage: bash scripts/m41a-window.sh            (the lead schedules it between rounds)
set -u
TAG=M41A; tag=m41a
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin
STAGE=/home/brenn/github/yack-ref/cadrs-on-shim/m41a-stage
DRY=/home/brenn/github/yack-ref/cadrs-on-shim/m41a-dry
TOUCHED="crates/yack-tol crates/yack-cadrs"
cd $Y
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench crates/yack-tol crates/yack-base"
others() { { git diff -- $FC ':!crates/yack-cadrs' ':!crates/yack-tol'; git status --short --untracked-files=all $FC | grep -v "yack-cadrs\|yack-tol"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

if [ -n "$(git status --short -- $TOUCHED)" ]; then echo "$TAG ABORT: $TOUCHED has uncommitted changes"; git status --short -- $TOUCHED; exit 1; fi
O1=$(others); PFP=$(fp)
build || { echo "$TAG pre build failed"; exit 1; }
snap $tag-pre-$PFP; echo "$TAG PRE=$PFP others=$O1"

BK=$(mktemp -d); for c in $TOUCHED; do mkdir -p $BK/$c; cp -a $c/. $BK/$c/; done
restore() { for c in $TOUCHED; do rm -rf $c; mkdir -p $c; cp -a $BK/$c/. $c/; done; echo "$TAG restored $TOUCHED"; }
python3 scripts/m41a.py $STAGE || { echo "$TAG m41a.py failed"; restore; exit 1; }
for c in $TOUCHED; do
  if ! diff -r $c $DRY/$c; then echo "$TAG ABORT: the in-tree $c differs from the dry run"; restore; exit 1; fi
done
echo "$TAG in-tree result identical to $DRY"
QFP=$(fp)
build || { echo "$TAG post build failed"; restore; exit 1; }
cargo test --release -p yack-tol -p yack-cadrs --no-fail-fast || { echo "$TAG tests fail"; restore; exit 1; }
cargo xtask gate || { echo "$TAG gate fails"; restore; exit 1; }
snap $tag-post-$QFP; O2=$(others)
echo "$TAG POST=$QFP others=$O2"
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
echo "$TAG BUILT; digests: bash scripts/m41a-digest.sh $PFP $QFP"
