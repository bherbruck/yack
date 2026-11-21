#!/bin/bash
# yack: the M4.0a window (plan/m4-cadrs-map.md, "foundations down": pure moves inside
# crates/yack-cadrs) in one builder request, build part only: hash the other frozen crates, pre
# build + copy, apply scripts/m40a.py, check the result equals the dry-run copy byte for byte, post
# build + yack-cadrs tests + copy, re-hash. On any failure after the apply it restores yack-cadrs.
# The digest pair runs OUTSIDE the builder after: scripts/m40a-digest.sh (it prints the line).
# usage: bash scripts/m40a-window.sh            (the lead schedules it between rounds)
set -u
TAG=M40A; tag=m40a
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; C=$Y/crates/yack-cadrs
POSTCOPY=/home/brenn/github/yack-ref/cadrs-on-shim/m40a-copy
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

BK=$(mktemp -d); cp -a $C/src $C/import.py $BK/
restore() { rm -rf $C/src; cp -a $BK/src $C/src; cp -a $BK/import.py $C/import.py; find $C/src $C/import.py -type f -exec touch {} +; echo "$TAG restored crates/yack-cadrs (src, import.py)"; }
M40A_ROOT=$C python3 scripts/m40a.py || { echo "$TAG m40a.py failed"; restore; exit 1; }
if ! diff -r $C/src $POSTCOPY/src || ! diff $C/import.py $POSTCOPY/import.py; then echo "$TAG ABORT: the in-tree result differs from the dry-run copy"; restore; exit 1; fi
echo "$TAG in-tree result identical to $POSTCOPY"
QFP=$(fp)
build || { echo "$TAG post build failed"; restore; exit 1; }
cargo test --release -p yack-cadrs --no-fail-fast || { echo "$TAG yack-cadrs tests fail"; restore; exit 1; }
snap $tag-post-$QFP; O2=$(others)
echo "$TAG POST=$QFP others=$O2"
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
echo "$TAG BUILT; digests: bash scripts/m40a-digest.sh $PFP $QFP"
