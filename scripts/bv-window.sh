#!/bin/bash
# yack: w-h1's window for Bvh::new -> Result across its callers (yack-math, yack-locate, yack-mesh,
# yack-springrep), in one builder request: hash the other files,
# build pre + copy, put the staged files in, clippy + tests + gate, build post + copy, re-hash. Any
# failure after the swap puts the old files back. The digest pair runs OUTSIDE the builder after:
# scripts/h1-digest.sh (it prints the line).
# usage: bash scripts/bv-window.sh STEP STAGE_DIR   (STAGE_DIR mirrors the tree: crates/...)
set -u
STEP=$1; STAGE=$2; tag=h1
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin
cd $Y
if [ -e swarm/FREEZE ]; then echo "$STEP ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "w-h1 $STEP window since $(date '+%F %T')" > swarm/FREEZE
trap 'rm -f $Y/swarm/FREEZE' EXIT; trap 'exit 130' INT TERM HUP
FILES=$(cd $STAGE && find crates -type f | sort)
[ -n "$FILES" ] || { echo "$STEP ABORT: nothing staged in $STAGE"; exit 1; }
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
others() { { git diff -- crates $(for f in $FILES; do echo ":!$f"; done) ':!**/__pycache__/**'; git status --short --untracked-files=all crates | grep -v "__pycache__" | grep -vF "$FILES"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }
BK=$(mktemp -d)
for f in $FILES; do mkdir -p $BK/$(dirname $f); cp $f $BK/$f; done
restore() { for f in $FILES; do cp $BK/$f $f && touch $f; done; echo "$STEP restored $FILES"; }
O1=$(others); PFP=$(fp)
build || { echo "$STEP pre build failed"; exit 1; }
snap $tag-pre-$PFP; echo "$STEP PRE=$PFP others=$O1"
for f in $FILES; do cp $STAGE/$f $f && touch $f && cmp -s $STAGE/$f $f || { echo "$STEP ABORT: swap failed on $f"; restore; exit 1; }; done
QFP=$(fp)
cargo clippy -p yack-math --all-targets -- -D warnings || { echo "$STEP clippy fails"; restore; exit 1; }
cargo clippy -p yack-locate -p yack-mesh --all-targets || { echo "$STEP caller clippy fails"; restore; exit 1; }
cargo test -p yack-math || { echo "$STEP yack-math tests fail"; restore; exit 1; }
cargo check --workspace --all-targets || { echo "$STEP workspace check fails"; restore; exit 1; }
cargo test --release -p yack-locate -p yack-mesh || { echo "$STEP caller tests fail"; restore; exit 1; }
cargo test --release -p yack-springrep --lib -- overlap:: probe:: || { echo "$STEP springrep overlap/probe tests fail"; restore; exit 1; }
cargo xtask gate || { echo "$STEP gate fails"; restore; exit 1; }
build || { echo "$STEP post build failed"; restore; exit 1; }
snap $tag-post-$QFP; O2=$(others)
echo "$STEP POST=$QFP others=$O2"
if [ "$O1" != "$O2" ]; then echo "$STEP OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
echo "$STEP BUILT (the change stays in the tree); digests: bash scripts/h1-digest.sh $PFP $QFP"
