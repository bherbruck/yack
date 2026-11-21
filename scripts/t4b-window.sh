#!/bin/bash
# M2.2 T4b freeze window, run by the builder as one request: pre build + copy, apply the change,
# post build + copy. On any failure after the apply it restores yack-springrep, so the tree builds.
set -u
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; SR=$Y/crates/yack-springrep
cd $Y
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench"
others() { { git diff -- $FC ':!crates/yack-springrep'; git status --short --untracked-files=all $FC | grep -v yack-springrep; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

if [ -n "$(git status --short -- crates/yack-springrep)" ]; then echo "T4B ABORT: crates/yack-springrep has uncommitted changes"; git status --short -- crates/yack-springrep; exit 1; fi
O1=$(others); PFP=$(fp)
build || { echo "T4B pre build failed"; exit 1; }
snap t4b-pre-$PFP; echo "T4B PRE=$PFP others=$O1"

BK=$(mktemp -d); cp -a $SR/src $BK/src
restore() { rm -rf $SR/src; cp -a $BK/src $SR/src; find $SR/src -type f -exec touch {} +; echo "T4B restored yack-springrep"; }
python3 scripts/t4b-apply.py $SR || { restore; exit 1; }
QFP=$(fp)
build || { echo "T4B post build failed"; restore; exit 1; }
cargo test -p yack-springrep --no-run || { echo "T4B springrep tests do not compile"; restore; exit 1; }
cargo test -p yack-springrep --lib codec:: || { echo "T4B codec tests fail"; restore; exit 1; }
snap t4b-post-$QFP; O2=$(others)
echo "T4B POST=$QFP others=$O2"
if [ "$O1" = "$O2" ]; then echo "T4B others unchanged"; else echo "T4B OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
