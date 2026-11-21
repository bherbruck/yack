#!/bin/bash
# yack: the D1 window (dead yack-cadrs code out: what nothing reaches, not even a test) in one
# builder request, build part only: check the tree still holds the dry run's base, hash the other
# frozen crates, pre build + copy, put the dry run's src in place, post build + yack-cadrs tests +
# gate + copy, re-hash. On any failure after the apply it restores yack-cadrs. The digest pair
# runs OUTSIDE the builder after: scripts/d1-digest.sh (it prints the line).
# usage: bash scripts/d1-window.sh            (the lead schedules it between rounds)
set -u
TAG=D1; tag=d1
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin
DRY=/home/brenn/github/yack-ref/cadrs-on-shim/d1

TOUCHED="crates/yack-cadrs"
cd $Y
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench crates/yack-tol crates/yack-base crates/yack-linalg crates/yack-step"
others() { { git diff -- $FC ':!crates/yack-cadrs' ':!**/__pycache__/**'; git status --short --untracked-files=all $FC | grep -v "yack-cadrs\|__pycache__"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

if [ -n "$(git status --short -- $TOUCHED)" ]; then echo "$TAG ABORT: $TOUCHED has uncommitted changes"; git status --short -- $TOUCHED; exit 1; fi
# The dry run was made from this src: the tree must still hold it.
if ! diff -rq crates/yack-cadrs/src /home/brenn/github/yack-ref/cadrs-on-shim/d1-base/src >/dev/null; then echo "$TAG ABORT: crates/yack-cadrs/src is no longer the dry run's base"; exit 1; fi
O1=$(others); PFP=$(fp)
build || { echo "$TAG pre build failed"; exit 1; }
snap $tag-pre-$PFP; echo "$TAG PRE=$PFP others=$O1"

BK=$(mktemp -d); for c in $TOUCHED; do mkdir -p $BK/$c; cp -a $c/. $BK/$c/; done
restore() { for c in $TOUCHED; do rm -rf $c; mkdir -p $c; cp -a $BK/$c/. $c/; done; for c in $TOUCHED; do [ -d $c ] && find $c -type f -exec touch {} +; done; echo "$TAG restored $TOUCHED (new mtimes: cargo rebuilds)"; }
rm -rf crates/yack-cadrs/src && cp -a $DRY/src crates/yack-cadrs/src && find crates/yack-cadrs/src -type f -exec touch {} + || { echo "$TAG apply failed"; restore; exit 1; }
for c in $TOUCHED; do
  if ! diff -r $c/src $DRY/src; then echo "$TAG ABORT: the in-tree $c differs from the dry run"; restore; exit 1; fi
done
echo "$TAG in-tree result identical to $DRY"
QFP=$(fp)
build || { echo "$TAG post build failed"; restore; exit 1; }
cargo check --workspace --all-targets || { echo "$TAG workspace check fails"; restore; exit 1; }
cargo test --release -p yack-cadrs --no-fail-fast || { echo "$TAG tests fail"; restore; exit 1; }
cargo xtask gate || { echo "$TAG gate fails"; restore; exit 1; }
snap $tag-post-$QFP; O2=$(others)
echo "$TAG POST=$QFP others=$O2"
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
echo "$TAG BUILT; digests: bash scripts/d1-digest.sh $PFP $QFP"
