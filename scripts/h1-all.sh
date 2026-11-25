#!/bin/bash
# yack: w-h1's digest run under ONE swarm/DIGEST.lock hold: record from BASE's pre binaries, then
# --check each POST's binaries against that record, stopping at the first difference (the lead
# bisects a failure; the diffing ids are in that check's output).
# usage: bash scripts/h1-all.sh BASE_FP POST_FP [POST_FP...]
set -u
PFP=$1; shift; tag=h1; R=/home/brenn/github/yack-ref/bin
L=/home/brenn/github/yack/swarm/DIGEST.lock
until mkdir $L 2>/dev/null; do sleep 10; done
echo "w-h1 record + $# checks since $(date '+%F %T')" > $L/owner
trap 'rm -rf $L' EXIT; trap 'exit 130' INT TERM HUP
D=/home/brenn/github/yack-ref/tmp/$tag-digests-$PFP; rm -rf $D; mkdir -p $D
nice -n 10 $R/$tag-pre-$PFP/xtask digest --threads 6 --bin-dir $R/$tag-pre-$PFP --tree $PFP --dir $D || { echo "h1 RECORD FAILED base=$PFP"; exit 4; }
echo "h1 recorded base=$PFP"
for QFP in "$@"; do
  nice -n 10 $R/$tag-post-$QFP/xtask digest --check --threads 6 --bin-dir $R/$tag-post-$QFP --tree $QFP --dir $D || { echo "h1 DIGESTS DIFFER base=$PFP post=$QFP: stop, report to the lead"; exit 5; }
  echo "h1 digests identical base=$PFP post=$QFP"
done
echo "h1 ALL IDENTICAL base=$PFP posts=$*"
