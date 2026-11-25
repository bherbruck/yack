#!/bin/bash
# yack: w-h1's hardening window for crates/yack-numeric/src/lib.rs, in one builder request: hash the
# other crates, build pre + copy, put the staged lib.rs in, clippy + tests + gate, build post + copy,
# re-hash. Any failure after the swap puts the old lib.rs back. The digest pair runs OUTSIDE the
# builder after: scripts/h1-digest.sh (it prints the line).
# usage: bash scripts/h1-window.sh STEP STAGED_LIB_RS
set -u
STEP=$1; NEW=$2; tag=h1
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; F=crates/yack-numeric/src/lib.rs
cd $Y
if [ -e swarm/FREEZE ]; then echo "$STEP ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "w-h1 $STEP window since $(date '+%F %T')" > swarm/FREEZE
trap 'rm -f $Y/swarm/FREEZE' EXIT; trap 'exit 130' INT TERM HUP
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
others() { { git diff -- crates ":!$F" ':!**/__pycache__/**'; git status --short --untracked-files=all crates | grep -v "yack-numeric/src/lib.rs\|__pycache__"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }
[ -f "$NEW" ] || { echo "$STEP ABORT: no staged file $NEW"; exit 1; }
BK=$(mktemp); cp $F $BK
restore() { cp $BK $F && touch $F && echo "$STEP restored $F"; }
O1=$(others); PFP=$(fp)
build || { echo "$STEP pre build failed"; exit 1; }
snap $tag-pre-$PFP; echo "$STEP PRE=$PFP others=$O1"
cp "$NEW" $F && touch $F
cmp -s "$NEW" $F || { echo "$STEP ABORT: swap failed"; restore; exit 1; }
QFP=$(fp)
cargo clippy -p yack-numeric --all-targets -- -D warnings || { echo "$STEP clippy fails"; restore; exit 1; }
cargo test -p yack-numeric || { echo "$STEP yack-numeric tests fail"; restore; exit 1; }
cargo check --workspace --all-targets || { echo "$STEP workspace check fails"; restore; exit 1; }
cargo test --release -p yack-geom -p yack-measure || { echo "$STEP caller tests fail"; restore; exit 1; }
cargo xtask gate || { echo "$STEP gate fails"; restore; exit 1; }
build || { echo "$STEP post build failed"; restore; exit 1; }
snap $tag-post-$QFP; O2=$(others)
echo "$STEP POST=$QFP others=$O2"
if [ "$O1" != "$O2" ]; then echo "$STEP OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
echo "$STEP BUILT (the change stays in the tree); digests: bash scripts/h1-digest.sh $PFP $QFP"
