#!/bin/bash
# yack: w-h4's W8 pre-flight and window, in one builder request. The dry run's new crate is
# self-contained (it does not depend on springrep), so it goes in as crates/yack-boolean alone for
# clippy -D warnings and its tests, and comes out again with Cargo.lock as it was. Only if both pass
# does the W8 window itself run (scripts/m2-window.sh, from the dry run's request line).
# usage: bash scripts/w-h4-w8-trial.sh SCRATCH_DIR   (SCRATCH_DIR from scripts/w-h4-w8-dryrun.sh)
set -u
S=$1; Y=/home/brenn/github/yack; cd $Y
[ -e crates/yack-boolean ] && { echo "W8 TRIAL ABORT: crates/yack-boolean exists"; exit 1; }
# the trial puts a crate into the shared tree and takes it out: it holds swarm/FREEZE throughout, and
# hands over to m2-window (which refuses to start under another freeze, then takes its own) at the end
if [ -e swarm/FREEZE ]; then echo "W8 TRIAL ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "W8 trial (w-h4) since $(date '+%F %T')" > swarm/FREEZE
trap 'rm -f $Y/swarm/FREEZE' EXIT
cp Cargo.lock $S/Cargo.lock.w8trial
rm -rf crates/.yack-boolean-w8trial; cp -r $S/dryreal/crates/yack-boolean crates/.yack-boolean-w8trial
mv crates/.yack-boolean-w8trial crates/yack-boolean
ok=1
cargo clippy -p yack-boolean --all-targets -- -D warnings || ok=0
[ $ok = 1 ] && { cargo test -p yack-boolean --lib || ok=0; }
rm -rf crates/yack-boolean
cp $S/Cargo.lock.w8trial Cargo.lock
[ $ok = 1 ] || { echo "W8 TRIAL FAILED: the new crate does not pass clippy or its tests; tree restored"; exit 2; }
echo "W8 TRIAL OK: clippy -D warnings and the crate's tests pass on the dry run's yack-boolean"
rm -f swarm/FREEZE; trap - EXIT
eval "$(cat $S/w8-req.txt)"
