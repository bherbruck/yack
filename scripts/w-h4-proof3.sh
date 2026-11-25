#!/bin/bash
# yack: w-h4's proof of the yack-intersect hardening, run OUTSIDE the builder after
# scripts/w-h4-window3.sh. It takes one digest slot (swarm/DIGEST.lock or DIGEST.lock2), and only
# while swarm/DIGEST.next is absent or names w-h4 (QUEUE rule). Both steps are meant to be
# digest-identical, so each is checked straight against scores/digests (DIGEST SPEED-UP): first the
# mid binaries (the rewrite with its contract panics kept), then the post binaries (panics removed),
# so a difference is pinned on one step. Then the interleaved CPU-time comparison of the hot cases,
# base (HEAD's files) against post.
# usage: bash scripts/w-h4-proof3.sh TAG BASE_FP MID_FP POST_FP [ROUNDS]
set -u
TAG=$1; BFP=$2; MFP=$3; PFP=$4; ROUNDS=${5:-5}; R=/home/brenn/github/yack-ref/bin; Y=/home/brenn/github/yack
LOCK=
take() {
  while true; do
    if [ -e $Y/swarm/DIGEST.next ] && ! grep -q 'w-h4' $Y/swarm/DIGEST.next; then sleep 20; continue; fi
    for l in DIGEST.lock DIGEST.lock2; do
      if mkdir $Y/swarm/$l 2>/dev/null; then LOCK=$l; return; fi
    done
    sleep 20
  done
}
take
trap 'rmdir $Y/swarm/$LOCK' EXIT
echo "$TAG holds swarm/$LOCK"
cd $Y
BASE=$R/w-h4-$TAG-base-$BFP; MID=$R/w-h4-$TAG-mid-$MFP; POST=$R/w-h4-$TAG-post-$PFP
nice -n 10 $MID/xtask digest --check --threads 6 --bin-dir $MID --tree $MFP || { echo "$TAG MID DIGESTS DIFFER from scores/digests (the splits and renames)"; exit 5; }
echo "$TAG mid digests identical to scores/digests"
nice -n 10 $POST/xtask digest --check --threads 6 --bin-dir $POST --tree $PFP || { echo "$TAG POST DIGESTS DIFFER from scores/digests (the panics removed)"; exit 6; }
echo "$TAG post digests identical to scores/digests"
python3 $Y/scripts/w-h4-pairtime.py $BASE/yack-bench $POST/yack-bench $ROUNDS
