#!/bin/bash
# yack: w-m11 perf #2 step 1, outside the builder, under swarm/DIGEST.lock: bench and conformance
# seed-0 digests from the pre and post binaries, each recorded in full and diffed (every digest
# must be identical), then ring-carried-along-a-spline timed with each binary, interleaved.
set -u
Y=/home/brenn/github/yack; cd $Y
LOCK=$Y/swarm/DIGEST.lock
until mkdir $LOCK 2>/dev/null; do sleep 15; done
echo "w-m11-seeds since $(date '+%F %T')" > $LOCK/owner; trap 'rm -rf $LOCK' EXIT
X=${XTASK:-/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/lead/xtask-frozen}
O=/home/brenn/github/yack-ref/bin/w-m11-seeds; W=$O/work; mkdir -p $W/pre $W/post
for side in pre post; do
  nice -n 10 $X digest --bin-dir $O/$side --tree seeds-$side --dir $W/$side > $W/$side.log 2>&1
done
for f in bench.txt conformance-seed0.txt; do
  n=$(diff <(grep -v '^#' $W/pre/$f | sort) <(grep -v '^#' $W/post/$f | sort) | grep -c '^>')
  echo "$f: $(grep -vc '^#' $W/pre/$f) lines, $n moved"
done
diff <(grep -v '^#' $W/pre/bench.txt | sort) <(grep -v '^#' $W/post/bench.txt | sort) | head -20
diff <(grep -v '^#' $W/pre/conformance-seed0.txt | sort) <(grep -v '^#' $W/post/conformance-seed0.txt | sort) | head -20
C=swept/ring-carried-along-a-spline; ta=0; tb=0
for i in 1 2 3 4 5 6; do
  for b in $( [ $((i % 2)) = 1 ] && echo "pre post" || echo "post pre" ); do
    s=$(date +%s.%N); nice -n 10 $O/$b/yack-bench $C $W/r.stl > /dev/null; e=$(date +%s.%N)
    d=$(echo "$e - $s" | bc); if [ $b = pre ]; then ta=$(echo "$ta + $d" | bc); else tb=$(echo "$tb + $d" | bc); fi
  done
done
echo "$C six runs each, interleaved: pre $ta s, post $tb s, ratio $(echo "scale=3; $tb / $ta" | bc)"
