#!/bin/bash
# yack: w-h4's check of a staged hardening batch on one crate, in one builder request (digests are the
# lead's, BRIEF rule 4): run rustfmt over STAGE/post, require the tree to hold STAGE/pre, put
# STAGE/post in, run clippy -D warnings and the tests on CRATE, check `yack` (so every dependent
# compiles) and the gate; on any failure put STAGE/pre back, so the tree never stays broken.
# usage: bash scripts/w-h4-trial.sh STAGE CRATE   (STAGE/{pre,post}/<path>; <path>.absent where the
# file does not exist on that side)
set -u
STAGE=$1; CRATE=$2
cd /home/brenn/github/yack
bash scripts/w-h4-fmt.sh $STAGE/post
FILES=$( { (cd $STAGE/pre && find . -type f); (cd $STAGE/post && find . -type f); } | sed -e 's|^\./||' -e 's|\.absent$||' | sort -u)
same() { if [ -f $STAGE/$1/$2.absent ]; then [ ! -e $2 ]; else cmp -s $STAGE/$1/$2 $2; fi; }
for f in $FILES; do
  for s in pre post; do
    [ -f $STAGE/$s/$f ] || [ -f $STAGE/$s/$f.absent ] || { echo "TRIAL ABORT: no $s copy of $f"; exit 1; }
  done
  same pre $f || { echo "TRIAL ABORT: in-tree $f differs from $STAGE/pre"; exit 1; }
done
put() {
  for f in $FILES; do
    if [ -f $STAGE/$1/$f.absent ]; then rm -f $f; else mkdir -p $(dirname $f); cp $STAGE/$1/$f $f.w-h4tmp && mv $f.w-h4tmp $f; fi
  done
  find crates/$CRATE -type d -empty -delete
}
put post
cargo clippy -p $CRATE --all-targets -- -D warnings || { put pre; echo "TRIAL clippy failed; pre restored"; exit 2; }
cargo test -p $CRATE || { put pre; echo "TRIAL tests failed; pre restored"; exit 2; }
cargo check -p yack || { put pre; echo "TRIAL dependents fail to compile; pre restored"; exit 2; }
cargo xtask gate || { put pre; echo "TRIAL gate failed; pre restored"; exit 2; }
echo "TRIAL OK: $CRATE post files stay in the tree"
