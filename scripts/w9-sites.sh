#!/bin/bash
# yack: lists the `.expect`/`.unwrap` sites clippy finds in springrep files a W9 wave moves, with the
# type they are called on (Option or Result), into swarm/res/w9-sites-<tag>.txt (full, untrimmed).
# usage: bash scripts/w9-sites.sh TAG PATH_PREFIX...   (prefixes under crates/yack-springrep/src/)
set -u
TAG=$1; shift
cd /home/brenn/github/yack
OUT=swarm/res/w9-sites-$TAG.txt
cargo clippy -p yack-springrep --lib --message-format short -- -W clippy::expect_used -W clippy::unwrap_used 2>&1 \
  | grep -E "expect_used|unwrap_used|used \`(expect|unwrap)\(\)\`" > /tmp/w9-sites-all.$$ || true
: > $OUT
for p in "$@"; do grep "crates/yack-springrep/src/$p" /tmp/w9-sites-all.$$ >> $OUT; done
rm -f /tmp/w9-sites-all.$$
sort -u -o $OUT $OUT
echo "w9-sites $TAG: $(wc -l < $OUT) sites -> $OUT"
