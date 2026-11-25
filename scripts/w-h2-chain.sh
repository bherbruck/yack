#!/bin/bash
# yack: w-h2's A/B/C... build for a run of yack-geom hardening steps, in one builder request, so
# every state is built against the same rest of the tree: hash the rest, put the base in (each
# file's pre copy from the first stage that names it), build + copy out, then apply each stage's
# post files in turn, building + copying out after each, and re-hash. The tree ends with the last
# stage's files, which the script checks it started with. The digests run OUTSIDE the builder
# after: scripts/w-h2-chain-digest.sh records the base once and checks every stage against it.
# usage: bash scripts/w-h2-chain.sh TAG STAGE1 STAGE2 ...   (each STAGE as for w-h2-window.sh)
set -u
TAG=$1; shift
STAGES="$@"
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin
cd $Y
ALL=$(for s in $STAGES; do (cd $s/post && find . -type f | sed 's|^\./||'); done | sort -u)
# The in-tree file is the last stage's post copy that names it.
for f in $ALL; do
  last=""; for s in $STAGES; do [ -f $s/post/$f ] && last=$s; done
  cmp -s $last/post/$f $f || { echo "$TAG ABORT: in-tree $f differs from $last/post"; exit 1; }
done
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
others() { { git diff -- . $(for f in $ALL; do echo ":!$f"; done); git status --short --untracked-files=all | grep -v -F "$ALL"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }
place() { if [ -f $1.absent ]; then rm -f $2; else mkdir -p $(dirname $2); cp $1 $2.w-h2tmp && mv $2.w-h2tmp $2; fi; }
final() { for f in $ALL; do last=""; for s in $STAGES; do [ -f $s/post/$f ] && last=$s; done; place $last/post/$f $f; done; }
O1=$(others)
for f in $ALL; do
  first=""; for s in $STAGES; do if [ -z "$first" ] && [ -f $s/post/$f ]; then first=$s; fi; done
  [ -f $first/pre/$f ] || [ -f $first/pre/$f.absent ] || { echo "$TAG ABORT: no pre copy of $f in $first"; final; exit 1; }
  place $first/pre/$f $f
done
BFP=$(fp)
build || { final; echo "$TAG base build failed"; exit 1; }
snap w-h2-$TAG-0-$BFP
LIST="$BFP"; k=0
for s in $STAGES; do
  k=$((k+1))
  for f in $(cd $s/post && find . -type f | sed 's|^\./||'); do place $s/post/$f $f; done
  SFP=$(fp)
  build || { final; echo "$TAG stage $k build failed ($s)"; exit 1; }
  snap w-h2-$TAG-$k-$SFP
  LIST="$LIST $SFP"
done
O2=$(others)
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): chain contaminated, resubmit"; exit 3; fi
echo "$TAG BUILT; digests: bash scripts/w-h2-chain-digest.sh $TAG $LIST"
