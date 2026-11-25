#!/bin/bash
# yack: w-h3's A/B build for one yack-topo hardening batch, in one builder request: hash the rest of
# the tree, put the step's pre files in, build + copy the binaries out, put the post files in, build +
# copy out, re-hash. The tree ends with the post files (the in-tree state it started with, which the
# script checks). The digest pair runs OUTSIDE the builder after: scripts/w-h3-digest.sh.
# usage: bash scripts/w-h3-window.sh TAG STAGE   (STAGE/pre/<path> and STAGE/post/<path> per file;
# STAGE/pre/<path>.absent instead of a pre copy for a file the step creates)
set -u
TAG=$1; STAGE=$2
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin
cd $Y
FILES=$(cd $STAGE/post && find . -type f | sed 's|^\./||' | sort)
for f in $FILES; do
  cmp -s $STAGE/post/$f $f || { echo "$TAG ABORT: in-tree $f differs from $STAGE/post"; exit 1; }
  [ -f $STAGE/pre/$f ] || [ -f $STAGE/pre/$f.absent ] || { echo "$TAG ABORT: no pre copy of $f"; exit 1; }
done
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
others() { { git diff -- . $(for f in $FILES; do echo ":!$f"; done); git status --short --untracked-files=all | grep -v -F "$FILES"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }
put() { for f in $FILES; do if [ -f $STAGE/$1/$f.absent ]; then rm -f $f; else cp $STAGE/$1/$f $f.w-h3tmp && mv $f.w-h3tmp $f; fi; done; }
O1=$(others)
put pre; PFP=$(fp)
build || { put post; echo "$TAG pre build failed"; exit 1; }
snap w-h3-$TAG-pre-$PFP
put post; QFP=$(fp)
build || { echo "$TAG post build failed"; exit 1; }
snap w-h3-$TAG-post-$QFP
O2=$(others)
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated, resubmit"; exit 3; fi
echo "$TAG BUILT; digests: bash scripts/w-h3-digest.sh $TAG $PFP $QFP"
