#!/bin/bash
# yack: w-h2's pending digest proofs, one at a time (never two w-h2 waiters racing for the lock),
# with a pause between them so others' waiting runs get the lock first.
# usage: bash scripts/w-h2-queue.sh OUTDIR 'tag script args...' ...
set -u
OUT=$1; shift
cd /home/brenn/github/yack
for job in "$@"; do
  set -- $job; tag=$1; shift
  bash "$@" > $OUT/digest-$tag.log 2>&1; echo "$tag rc=$?" >> $OUT/queue.log
  sleep 120
done
