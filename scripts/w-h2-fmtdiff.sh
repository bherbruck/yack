#!/bin/bash
# yack: w-h2's rustfmt diff of its own yack-geom files, in full (the builder log keeps only a tail).
# usage: bash scripts/w-h2-fmtdiff.sh OUT FILE...
set -u
OUT=$1; shift
cd /home/brenn/github/yack
rustfmt --edition 2024 --check "$@" > "$OUT" 2>&1
echo "rustfmt rc=$? ($(grep -c '^Diff in' "$OUT") diffs) -> $OUT"
