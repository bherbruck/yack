#!/bin/bash
# yack: w-h2's check-only digest proof (BRIEF "DIGEST SPEED-UP"): the post binaries of a
# digest-neutral batch check straight against the committed baseline, scores/digests, run OUTSIDE
# the builder under one of the two slots, DIGEST.lock or DIGEST.lock2 (QUEUE rule honoured).
# usage: bash scripts/w-h2-check.sh TAG BIN_DIR TREE_FP
set -u
TAG=$1; BIN=$2; FP=$3; Y=/home/brenn/github/yack
SLOT=""
until [ -n "$SLOT" ]; do
  if ! [ -e $Y/swarm/DIGEST.next ] || grep -q w-h2 $Y/swarm/DIGEST.next; then
    for s in DIGEST.lock DIGEST.lock2; do mkdir $Y/swarm/$s 2>/dev/null && { SLOT=$s; break; }; done
  fi
  [ -n "$SLOT" ] || sleep 60
done
echo "w-h2-$TAG since $(date '+%F %T')" > $Y/swarm/$SLOT/owner
trap 'rm -f $Y/swarm/$SLOT/owner; rmdir $Y/swarm/$SLOT' EXIT
nice -n 10 $BIN/xtask digest --check --threads 6 --bin-dir $BIN --tree $FP || { echo "$TAG CHECK FAILED against scores/digests ($FP): bisect or report"; exit 5; }
echo "$TAG digests identical to scores/digests ($FP)"
