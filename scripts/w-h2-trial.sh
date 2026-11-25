#!/bin/bash
# yack: w-h2's trial of a staged multi-file yack-geom change, in one builder request: put
# STAGE/post/<path> in (new files too), run clippy -D warnings and the crate's tests, and on any
# failure put back exactly what was there (removing files the stage added), so the tree never stays
# broken.
# usage: bash scripts/w-h2-trial.sh STAGE   (STAGE/fmt: leaf files to rustfmt, and copy back to
# STAGE/post, once the trial passes)
set -u
STAGE=$1
CRATE=$(cat $STAGE/crate 2>/dev/null || echo yack-geom)
cd /home/brenn/github/yack
FILES=$(cd $STAGE/post && find . -type f | sed 's|^\./||' | sort)
BK=$(mktemp -d)
for f in $FILES; do if [ -f $f ]; then mkdir -p $BK/$(dirname $f); cp -p $f $BK/$f; fi; done
restore() { for f in $FILES; do if [ -f $BK/$f ]; then cp -p $BK/$f $f; else rm -f $f; fi; done; echo "TRIAL restored the previous files"; }
for f in $FILES; do mkdir -p $(dirname $f); cp $STAGE/post/$f $f.w-h2tmp && mv $f.w-h2tmp $f; done
cargo clippy -p $CRATE --all-targets -- -D warnings || { echo "TRIAL clippy failed"; restore; exit 1; }
cargo test -p $CRATE || { echo "TRIAL tests failed"; restore; exit 1; }
if [ -f $STAGE/checks ]; then
  while read -r cmd; do [ -n "$cmd" ] && { $cmd || { echo "TRIAL check failed: $cmd"; restore; exit 1; }; }; done < $STAGE/checks
fi
if [ -f $STAGE/fmt ]; then
  rustfmt --edition 2024 $(cat $STAGE/fmt) && for f in $(cat $STAGE/fmt); do cp $f $STAGE/post/$f; done && echo "TRIAL formatted (and restaged): $(cat $STAGE/fmt)"
fi
echo "TRIAL OK: the staged files stay in the tree"
