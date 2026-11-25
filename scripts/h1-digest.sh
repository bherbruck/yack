#!/bin/bash
# yack: w-h1 (yack-numeric hardening) digest pair, run OUTSIDE the builder after scripts/h1-window.sh (its two suite
# runs pass the builder's 3600 s limit): the pre binaries record into a private dir, the post
# binaries check against it. Behaviour-identical rewrites: every digest the same.
# usage: bash scripts/h1-digest.sh PRE_FP POST_FP
set -u
PFP=$1; QFP=$2; tag=h1; R=/home/brenn/github/yack-ref/bin
# swarm/DIGEST.lock is held through both halves (BRIEF); wait for it.
L=/home/brenn/github/yack/swarm/DIGEST.lock
until mkdir $L 2>/dev/null; do sleep 30; done
echo "h1 digest pair (w-h1) since $(date '+%F %T')" > $L/owner
trap 'rm -rf $L' EXIT; trap 'exit 130' INT TERM HUP
D=/home/brenn/github/yack-ref/tmp/$tag-digests-$PFP; rm -rf $D; mkdir -p $D
nice -n 10 $R/$tag-pre-$PFP/xtask digest --threads 6 --bin-dir $R/$tag-pre-$PFP --tree $PFP --dir $D || { echo "h1 pre digest failed"; exit 4; }
nice -n 10 $R/$tag-post-$QFP/xtask digest --check --threads 6 --bin-dir $R/$tag-post-$QFP --tree $QFP --dir $D || { echo "h1 DIGESTS DIFFER (or the check failed): do not commit"; exit 5; }
echo "h1 digests identical pre=$PFP post=$QFP"
