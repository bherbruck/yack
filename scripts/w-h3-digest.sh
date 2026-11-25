#!/bin/bash
# yack: w-h3's digest check for one yack-topo hardening batch, run OUTSIDE the builder after
# scripts/w-h3-window.sh. The batches are meant to be digest-identical, so (BRIEF "DIGEST SPEED-UP")
# the post binaries check straight against scores/digests, under either DIGEST slot.
# usage: bash scripts/w-h3-digest.sh TAG POST_FP
set -u
TAG=$1; QFP=$2; R=/home/brenn/github/yack-ref/bin; Y=/home/brenn/github/yack
# QUEUE rule: a swarm/DIGEST.next that does not name w-h3 reserves the slots for someone else.
take() {
  [ ! -e $Y/swarm/DIGEST.next ] || grep -q w-h3 $Y/swarm/DIGEST.next || return 1
  for L in $Y/swarm/DIGEST.lock $Y/swarm/DIGEST.lock2; do
    mkdir $L 2>/dev/null && { LOCK=$L; return 0; }
  done
  return 1
}
until take; do sleep 20; done
trap 'rmdir $LOCK' EXIT
echo "$TAG check holds $LOCK since $(date '+%F %T')"
nice -n 10 $R/w-h3-$TAG-post-$QFP/xtask digest --check --threads 6 --bin-dir $R/w-h3-$TAG-post-$QFP --tree $QFP || { echo "$TAG DIGESTS DIFFER (or the check failed): do not commit"; exit 5; }
echo "$TAG digests identical to scores/digests post=$QFP"
