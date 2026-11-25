#!/bin/bash
# yack: w-h4's proof of one yack-intersect hardening batch, in one builder request. The tree must
# hold the batch's pre files. Hash the rest of the tree, build the pre binaries and copy them out,
# put the post files in, run clippy -D warnings, the crate's tests and the gate (on failure the pre
# files go back), build the post binaries and copy them out, then re-hash. On success the tree holds
# the post files. The digest pair runs OUTSIDE the builder after: scripts/w-h4-digest.sh.
# usage: bash scripts/w-h4-window.sh TAG STAGE
#   STAGE/{pre,post}/<path> per file; <path>.absent in place of a copy where the file does not exist
#   on that side (created or removed by the batch).
set -u
TAG=$1; STAGE=$2
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin
cd $Y
FILES=$( { (cd $STAGE/pre && find . -type f); (cd $STAGE/post && find . -type f); } | sed -e 's|^\./||' -e 's|\.absent$||' | sort -u)
same() { if [ -f $STAGE/$1/$2.absent ]; then [ ! -e $2 ]; else cmp -s $STAGE/$1/$2 $2; fi; }
for f in $FILES; do
  for side in pre post; do
    [ -f $STAGE/$side/$f ] || [ -f $STAGE/$side/$f.absent ] || { echo "$TAG ABORT: no $side copy of $f"; exit 1; }
  done
  same pre $f || { echo "$TAG ABORT: in-tree $f differs from $STAGE/pre"; exit 1; }
done
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
others() { { git diff -- . $(for f in $FILES; do echo ":!$f"; done); git status --short --untracked-files=all | grep -v -F "$FILES"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }
put() {
  for f in $FILES; do
    if [ -f $STAGE/$1/$f.absent ]; then rm -f $f; else mkdir -p $(dirname $f); cp $STAGE/$1/$f $f.w-h4tmp && mv $f.w-h4tmp $f; fi
  done
  find crates/yack-intersect -type d -empty -delete
}
O1=$(others)
PFP=$(fp)
build || { echo "$TAG pre build failed"; exit 1; }
snap w-h4-$TAG-pre-$PFP
put post; QFP=$(fp)
cargo clippy -p yack-intersect --all-targets -- -D warnings || { put pre; echo "$TAG CHECK clippy failed; pre files restored"; exit 2; }
cargo test -p yack-intersect || { put pre; echo "$TAG CHECK tests failed; pre files restored"; exit 2; }
cargo xtask gate || { put pre; echo "$TAG CHECK gate failed; pre files restored"; exit 2; }
build || { put pre; echo "$TAG post build failed; pre files restored"; exit 1; }
snap w-h4-$TAG-post-$QFP
for f in $FILES; do case $f in *.rs) [ -f $f ] && { rustfmt --edition 2024 --check $f >/dev/null 2>&1 || echo "$TAG FMT differs: $f"; };; esac; done
O2=$(others)
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated, resubmit"; exit 3; fi
echo "$TAG BUILT; digests: bash scripts/w-h4-digest.sh $TAG $PFP $QFP"
