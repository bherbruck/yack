#!/bin/bash
# yack: w-h1's test-only check: put the staged files in, run clippy and the crate's tests, and
# always put the old files back (tests only: nothing stays in the tree, no binaries, no digests).
# usage: bash scripts/lt-window.sh STEP STAGE_DIR CRATE [CLIPPY [TEST_ARGS...]]   (STAGE_DIR mirrors the
# tree: crates/...; CLIPPY 0 skips clippy, for temporary crates; TEST_ARGS narrow `cargo test`)
set -u
STEP=$1; STAGE=$2; CRATE=$3; CLIPPY=${4:-1}; shift 4 2>/dev/null || shift $#
Y=/home/brenn/github/yack
cd $Y
if [ -e swarm/FREEZE ]; then echo "$STEP ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "w-h1 $STEP test check since $(date '+%F %T')" > swarm/FREEZE
FILES=$(cd $STAGE && find crates -type f | sort)
BK=$(mktemp -d)
for f in $FILES; do mkdir -p $BK/$(dirname $f); cp $f $BK/$f; done
restore() { for f in $FILES; do cp $BK/$f $f && touch $f; done; echo "$STEP restored"; rm -f $Y/swarm/FREEZE; }
trap restore EXIT; trap 'exit 130' INT TERM HUP
for f in $FILES; do cp $STAGE/$f $f && touch $f; done
rc=0
if [ "$CLIPPY" = 1 ]; then cargo clippy -p $CRATE --all-targets -- -D warnings || { echo "$STEP clippy fails"; rc=1; }; fi
cargo test -p $CRATE "$@" || { echo "$STEP tests fail"; rc=1; }
exit $rc
