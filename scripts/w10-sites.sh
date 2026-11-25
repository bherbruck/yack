#!/bin/bash
# yack: lists every clippy finding (warning or error, short format) in springrep's facade/ files, which
# W10 moves into crates gated with -D warnings, into swarm/res/w10-facade-clippy.txt.
set -u
cd /home/brenn/github/yack
OUT=swarm/res/w10-facade-clippy.txt
cargo clippy -p yack-springrep --all-targets --no-deps --message-format short -- -W clippy::indexing_slicing -W clippy::unwrap_used -W clippy::expect_used -W clippy::panic -W clippy::disallowed_types -W clippy::disallowed_methods 2>&1 | grep "crates/yack-springrep/src/facade/" | sort -u > $OUT || true
echo "w10-sites: $(wc -l < $OUT) findings -> $OUT"
