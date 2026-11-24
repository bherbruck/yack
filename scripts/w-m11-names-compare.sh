#!/bin/bash
# yack: w-m11 names-only proof, outside the builder: every bench case's digest from the pre and post
# yack-bench, then for each case whose digest moved, its volume, area, centroid, inertia, face
# kinds, validity and mesh from both, which must be identical.
set -u
O=/home/brenn/github/yack-ref/bin/w-m11-names; W=$O/work; mkdir -p $W
nice -n 10 $O/yack-bench-pre --digests > $W/pre.txt
nice -n 10 $O/yack-bench-post --digests > $W/post.txt
echo "cases: $(wc -l < $W/pre.txt) pre, $(wc -l < $W/post.txt) post"
moved=$(diff <(sort $W/pre.txt) <(sort $W/post.txt) | grep '^>' | awk '{print $2}')
echo "moved: $(echo "$moved" | grep -c .)"
for c in $moved; do
  a=$(nice -n 10 $O/yack-bench-pre "$c" $W/pre.stl); b=$(nice -n 10 $O/yack-bench-post "$c" $W/post.stl)
  same=$([ "$a" = "$b" ] && echo same || echo DIFFER); mesh=$(cmp -s $W/pre.stl $W/post.stl && echo same || echo DIFFER)
  echo "$c report $same mesh $mesh :: $a"
done
