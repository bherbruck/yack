#!/bin/bash
# yack: w-h2's digest chain after scripts/w-h2-chain.sh, run OUTSIDE the builder under
# swarm/DIGEST.lock: the base binaries record once into a private dir, and each stage's binaries,
# in order, check against that record. Behaviour-identical steps: every digest the same as the base,
# so each stage is identical to the one before it.
# usage: bash scripts/w-h2-chain-digest.sh TAG BASE_FP STAGE1_FP STAGE2_FP ...
set -u
TAG=$1; BFP=$2; shift 2
R=/home/brenn/github/yack-ref/bin; Y=/home/brenn/github/yack
# QUEUE rule (BRIEF): no lock while swarm/DIGEST.next exists and does not name w-h2; retry slowly so
# a waiting run of someone else's is not out-raced.
until { ! [ -e $Y/swarm/DIGEST.next ] || grep -q w-h2 $Y/swarm/DIGEST.next; } && mkdir $Y/swarm/DIGEST.lock 2>/dev/null; do sleep 60; done
echo "w-h2-$TAG since $(date '+%F %T')" > $Y/swarm/DIGEST.lock/owner
trap 'rm -f $Y/swarm/DIGEST.lock/owner; rmdir $Y/swarm/DIGEST.lock' EXIT
D=/home/brenn/github/yack-ref/tmp/w-h2-$TAG-digests-$BFP; rm -rf $D; mkdir -p $D
nice -n 10 $R/w-h2-$TAG-0-$BFP/xtask digest --threads 6 --bin-dir $R/w-h2-$TAG-0-$BFP --tree $BFP --dir $D || { echo "$TAG base digest failed"; exit 4; }
k=0
for fp in "$@"; do
  k=$((k+1))
  nice -n 10 $R/w-h2-$TAG-$k-$fp/xtask digest --check --threads 6 --bin-dir $R/w-h2-$TAG-$k-$fp --tree $fp --dir $D || { echo "$TAG stage $k DIGESTS DIFFER (or the check failed): stages $k and later unproven"; exit 5; }
  echo "$TAG stage $k digests identical to the base ($BFP -> $fp)"
done
echo "$TAG chain identical: $BFP $*"
