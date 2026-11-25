#!/bin/bash
# yack: w-h4's three-way build for the yack-intersect hardening, in one builder request: the tree holds
# STAGE/post. Hash the rest of the tree; put in STAGE/base (HEAD's files), build and copy the binaries
# out; put in STAGE/post, run clippy -D warnings, the crate's tests and the gate, build, copy out; put
# in STAGE/mid (the same rewrite with its contract panics kept), check, build, copy out; re-hash. The
# tree ends with STAGE/mid, for the first commit; post goes in after it. On a failed check the tree
# goes back to post. Digests and timing run OUTSIDE the builder: scripts/w-h4-proof3.sh.
# usage: bash scripts/w-h4-window3.sh TAG STAGE   (STAGE/{base,mid,post}/<path>, <path>.absent where
# the file does not exist on that side)
set -u
TAG=$1; STAGE=$2
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin
cd $Y
FILES=$( for s in base mid post; do (cd $STAGE/$s && find . -type f); done | sed -e 's|^\./||' -e 's|\.absent$||' | sort -u)
same() { if [ -f $STAGE/$1/$2.absent ]; then [ ! -e $2 ]; else cmp -s $STAGE/$1/$2 $2; fi; }
for f in $FILES; do
  for s in base mid post; do
    [ -f $STAGE/$s/$f ] || [ -f $STAGE/$s/$f.absent ] || { echo "$TAG ABORT: no $s copy of $f"; exit 1; }
  done
  same post $f || { echo "$TAG ABORT: in-tree $f differs from $STAGE/post"; exit 1; }
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
check() {
  cargo clippy -p yack-intersect --all-targets -- -D warnings || { put post; echo "$TAG CHECK $1 clippy failed; post restored"; exit 2; }
  cargo test -p yack-intersect || { put post; echo "$TAG CHECK $1 tests failed; post restored"; exit 2; }
  cargo xtask gate || { put post; echo "$TAG CHECK $1 gate failed; post restored"; exit 2; }
}
O1=$(others)
put base; BFP=$(fp)
build || { put post; echo "$TAG base build failed; post restored"; exit 1; }
snap w-h4-$TAG-base-$BFP
put post; PFP=$(fp)
check post
build || { echo "$TAG post build failed"; exit 1; }
snap w-h4-$TAG-post-$PFP
put mid; MFP=$(fp)
check mid
build || { put post; echo "$TAG mid build failed; post restored"; exit 1; }
snap w-h4-$TAG-mid-$MFP
O2=$(others)
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): contaminated, resubmit"; exit 3; fi
echo "$TAG BUILT (tree holds mid); proof: bash scripts/w-h4-proof3.sh $TAG $BFP $MFP $PFP"
