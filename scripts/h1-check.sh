#!/bin/bash
# yack: w-h1's further digest checks: each later step's post binaries checked against the record
# scripts/h1-digest.sh made from the first step's pre binaries. A step whose digests all equal the
# base's equals the step before it too, so one record covers the chain.
# usage: bash scripts/h1-check.sh BASE_FP POST_FP [POST_FP...]
set -u
PFP=$1; shift; tag=h1; R=/home/brenn/github/yack-ref/bin
D=/home/brenn/github/yack-ref/tmp/$tag-digests-$PFP
[ -d $D ] || { echo "h1 no record at $D"; exit 1; }
L=/home/brenn/github/yack/swarm/DIGEST.lock
until mkdir $L 2>/dev/null; do sleep 30; done
echo "h1 digest checks (w-h1) since $(date '+%F %T')" > $L/owner
trap 'rm -rf $L' EXIT; trap 'exit 130' INT TERM HUP
for QFP in "$@"; do
  nice -n 10 $R/$tag-post-$QFP/xtask digest --check --threads 6 --bin-dir $R/$tag-post-$QFP --tree $QFP --dir $D || { echo "h1 DIGESTS DIFFER for $QFP (or the check failed): do not commit"; exit 5; }
  echo "h1 digests identical base=$PFP post=$QFP"
done
