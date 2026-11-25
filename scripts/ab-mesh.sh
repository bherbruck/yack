#!/bin/bash
# yack: w-h1's interleaved CPU-time A/B of two yack-bench builds on mesh-heavy cases (the M3+M4 layout
# change): each round runs every case on both builds, in alternating order, timing user + system CPU.
# Waits until at most one `xtask digest` runs. Prints per-case medians and the post/pre ratio.
# usage: bash scripts/ab-mesh.sh PRE_DIR POST_DIR ROUNDS CASE...
set -u
PRE=$1; POST=$2; N=$3; shift 3
OUT=$(mktemp -d)
digests() { ps -eo args | grep -E "^\S*xtask digest" | grep -vc proxy; }
until [ "$(digests)" -le 1 ]; do sleep 30; done
cpu() { /usr/bin/time -f "%U %S" "$1/yack-bench" "$2" "$OUT/x.stl" 2>&1 >/dev/null | tail -1 | awk '{print $1 + $2}'; }
for c in "$@"; do : > "$OUT/$(echo $c | tr / _).pre"; : > "$OUT/$(echo $c | tr / _).post"; done
for r in $(seq 1 $N); do
  for c in "$@"; do
    f=$OUT/$(echo $c | tr / _)
    if [ $((r % 2)) = 1 ]; then cpu $PRE $c >> $f.pre; cpu $POST $c >> $f.post
    else cpu $POST $c >> $f.post; cpu $PRE $c >> $f.pre; fi
  done
done
med() { sort -n "$1" | awk '{a[NR]=$1} END {print a[int((NR+1)/2)]}'; }
for c in "$@"; do
  f=$OUT/$(echo $c | tr / _)
  a=$(med $f.pre); b=$(med $f.post)
  echo "$c pre=$a post=$b ratio=$(awk -v a=$a -v b=$b 'BEGIN {printf "%.3f", b / a}') runs: pre[$(tr '\n' ' ' < $f.pre)] post[$(tr '\n' ' ' < $f.post)]"
done
rm -rf $OUT
