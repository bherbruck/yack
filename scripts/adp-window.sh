#!/bin/bash
# The adapter's protocol checks as a pre/post pair on the tree as it stands: build the adapter, copy it,
# apply the patch, build again, copy. Other workers' uncommitted work is in both sides alike, so a
# seed-0 comparison of the two copies shows only what the patch changes. Restores main.rs on failure.
# usage: bash scripts/adp-window.sh PATCH
set -u
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; M=crates/yack-conformance/src/main.rs; cd $Y
[ -z "$(git status --short crates/yack-conformance)" ] || { echo "ADP ABORT: crates/yack-conformance is dirty"; exit 1; }
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
back() { git show HEAD:$M > $M; touch $M; echo "ADP restored $M"; }
# clippy lints the workspace path deps too, and those carry their own warnings: count only the
# warnings located in the adapter crate, before and after; the count may not rise
lints() { cargo clippy -p yack-conformance --all-targets 2>&1 | grep -c -- '--> crates/yack-conformance/'; }
L1=$(lints); echo "ADP adapter clippy warnings before: $L1"
P=$(fp); cargo build --release -p yack-conformance || { echo "ADP pre build failed"; exit 1; }
mkdir -p $R/adp-pre-$P && cp bin/release/yack-conformance $R/adp-pre-$P/
git apply "$1" || { echo "ADP patch does not apply"; exit 1; }
cargo build --release -p yack-conformance || { echo "ADP post build failed"; back; exit 1; }
L2=$(lints); echo "ADP adapter clippy warnings after: $L2"; [ "$L2" -le "$L1" ] || { echo "ADP clippy warnings rose $L1 -> $L2"; back; exit 1; }
cargo test -p yack-conformance || { echo "ADP tests fail"; back; exit 1; }
Q=$(fp); mkdir -p $R/adp-post-$Q && cp bin/release/yack-conformance $R/adp-post-$Q/
echo "ADP PRE=adp-pre-$P POST=adp-post-$Q"
