#!/bin/bash
# M2.2 freeze window (any step), run by the builder as one request: pre build + copy, apply the change,
# post build + copy. On any failure after the apply it restores yack-springrep, so the tree builds.
set -u
# usage: bash scripts/m22-window.sh TAG APPLY.py TEST_FILTER   (e.g. T4A scripts/t4a-apply.py codec::)
TAG=$1; APPLY=$2; FILTER=$3; tag=$(echo $TAG | tr A-Z a-z)
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; SR=$Y/crates/yack-springrep
cd $Y
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench"
others() { { git diff -- $FC ':!crates/yack-springrep'; git status --short --untracked-files=all $FC | grep -v yack-springrep; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

if [ -n "$(git status --short -- crates/yack-springrep)" ]; then echo "$TAG ABORT: crates/yack-springrep has uncommitted changes"; git status --short -- crates/yack-springrep; exit 1; fi
O1=$(others); PFP=$(fp)
build || { echo "$TAG pre build failed"; exit 1; }
snap $tag-pre-$PFP; echo "$TAG PRE=$PFP others=$O1"

BK=$(mktemp -d); cp -a $SR/src $BK/src
# Restore only the files this window's apply touched (never the whole crate: others edit it too).
touched() { ( cd $SR/src && find . -type f; cd $BK/src && find . -type f ) | sort -u | while read -r f; do cmp -s "$SR/src/$f" "$BK/src/$f" || echo "$f"; done; }
restore() { for f in $TOUCHED; do if [ -f "$BK/src/$f" ]; then cp "$BK/src/$f" "$SR/src/$f" && touch "$SR/src/$f"; else rm -f "$SR/src/$f"; fi; done; touch "$SR/src/lib.rs"; echo "$TAG restored its own files:" $TOUCHED; }
python3 $APPLY $SR; ARC=$?; TOUCHED=$(touched); echo "$TAG touched:" $TOUCHED
[ $ARC -eq 0 ] || { restore; exit 1; }
QFP=$(fp)
build || { echo "$TAG post build failed"; restore; exit 1; }
cargo test -p yack-springrep --no-run || { echo "$TAG springrep tests do not compile"; restore; exit 1; }
cargo test -p yack-springrep --lib $FILTER || { echo "$TAG codec tests fail"; restore; exit 1; }
snap $tag-post-$QFP; O2=$(others)
echo "$TAG POST=$QFP others=$O2"
if [ "$O1" = "$O2" ]; then echo "$TAG others unchanged"; else echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
