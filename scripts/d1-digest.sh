#!/bin/bash
# yack: D1 (dead code)'s digest pair, run OUTSIDE the builder after scripts/d1-window.sh (its two suite
# runs pass the builder's 3600 s limit): the pre binaries record into a private dir, the post
# binaries check against it. Pure moves: every digest the same.
# usage: bash scripts/d1-digest.sh PRE_FP POST_FP
set -u
PFP=$1; QFP=$2; tag=d1; R=/home/brenn/github/yack-ref/bin
if pgrep -f "xtask digest" >/dev/null; then echo "another xtask digest is running: wait"; exit 1; fi
D=/home/brenn/github/yack-ref/tmp/$tag-digests-$PFP; rm -rf $D; mkdir -p $D
nice -n 10 $R/$tag-pre-$PFP/xtask digest --threads 6 --bin-dir $R/$tag-pre-$PFP --tree $PFP --dir $D || { echo "D1 pre digest failed"; exit 4; }
nice -n 10 $R/$tag-post-$QFP/xtask digest --check --threads 6 --bin-dir $R/$tag-post-$QFP --tree $QFP --dir $D || { echo "D1 DIGESTS DIFFER (or the check failed): do not commit"; exit 5; }
echo "D1 digests identical pre=$PFP post=$QFP"
