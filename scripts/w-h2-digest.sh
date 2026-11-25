#!/bin/bash
# yack: w-h2's digest pair for one yack-geom hardening step, run OUTSIDE the builder after
# scripts/w-h2-window.sh, under swarm/DIGEST.lock: the pre binaries record into a private dir, the post
# binaries check against it. Behaviour-identical steps: every digest the same.
# usage: bash scripts/w-h2-digest.sh TAG PRE_FP POST_FP
set -u
TAG=$1; PFP=$2; QFP=$3; R=/home/brenn/github/yack-ref/bin; Y=/home/brenn/github/yack
# QUEUE rule (BRIEF): no lock while swarm/DIGEST.next exists and does not name w-h2; retry slowly so
# a waiting run of someone else's is not out-raced.
until { ! [ -e $Y/swarm/DIGEST.next ] || grep -q w-h2 $Y/swarm/DIGEST.next; } && mkdir $Y/swarm/DIGEST.lock 2>/dev/null; do sleep 60; done
echo "w-h2-$TAG since $(date '+%F %T')" > $Y/swarm/DIGEST.lock/owner
trap 'rm -f $Y/swarm/DIGEST.lock/owner; rmdir $Y/swarm/DIGEST.lock' EXIT
D=/home/brenn/github/yack-ref/tmp/w-h2-$TAG-digests-$PFP; rm -rf $D; mkdir -p $D
nice -n 10 $R/w-h2-$TAG-pre-$PFP/xtask digest --threads 6 --bin-dir $R/w-h2-$TAG-pre-$PFP --tree $PFP --dir $D || { echo "$TAG pre digest failed"; exit 4; }
nice -n 10 $R/w-h2-$TAG-post-$QFP/xtask digest --check --threads 6 --bin-dir $R/w-h2-$TAG-post-$QFP --tree $QFP --dir $D || { echo "$TAG DIGESTS DIFFER (or the check failed): do not commit"; exit 5; }
echo "$TAG digests identical pre=$PFP post=$QFP"
