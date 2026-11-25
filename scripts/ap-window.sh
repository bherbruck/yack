#!/bin/bash
# yack: w-h1's window for an apply script over one crate (no pre build: the lead's whole-tree digest
# check proves pure moves): back the crate up, run the apply, then the given checks, gate, post build +
# copy. Any failure puts the crate back exactly (files the apply added are removed).
# usage: bash scripts/ap-window.sh TAG CRATE APPLY.py -- CHECK_CMD [-- CHECK_CMD ...]
#   e.g. ... W9C0 yack-step scripts/w9c0-apply.py -- "cargo check -p yack-step --features wip --all-targets"
set -u
TAG=$1; KR=$2; APPLY=$3; shift 3; tag=$(echo $TAG | tr A-Z a-z)
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; C=crates/$KR
cd $Y
if [ -e swarm/FREEZE ]; then echo "$TAG ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "w-h1 $TAG ap-window since $(date '+%F %T')" > swarm/FREEZE
trap 'rm -f $Y/swarm/FREEZE' EXIT; trap 'exit 130' INT TERM HUP
[ -n "$(git status --short -- $C)" ] && { echo "$TAG ABORT: $C has uncommitted changes"; git status --short -- $C; exit 1; }
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }
BK=$(mktemp -d); cp -a $C $BK/crate
restore() { rm -rf $C && cp -a $BK/crate $C && find $C -type f -exec touch {} + && echo "$TAG restored $C (git: $(git status --short -- $C | wc -l) changed)"; }
echo "$TAG before: $(fp)"
python3 $APPLY $C || { echo "$TAG apply failed"; restore; exit 1; }
QFP=$(fp); echo "$TAG applied: $(git status --short -- $C | tr '\n' ' ')"
while [ $# -gt 0 ]; do
  [ "$1" = "--" ] && { shift; continue; }
  echo "$TAG \$ $1"; bash -c "$1" || { echo "$TAG check fails: $1"; restore; exit 1; }
  shift
done
cargo xtask gate || { echo "$TAG gate fails"; restore; exit 1; }
build || { echo "$TAG post build failed"; restore; exit 1; }
snap h1-post-$QFP
echo "$TAG BUILT POST=$QFP (the change stays in the tree)"
