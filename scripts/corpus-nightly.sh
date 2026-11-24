#!/usr/bin/env bash
# yack: the SMLib corpus nightly (PLAN §6.7, M6): build, fetch, then grade at low priority, ratcheted.
#
#   bash scripts/corpus-nightly.sh [--jobs N] [--timeout S]
#
# Builds yack-corpus, yack-corpus-fetch and xtask, and runs `cargo xtask corpus smlib --fetch`.
# In a swarm (swarm/ exists) the build goes through the builder as one request; otherwise it
# runs here. Then it copies the binaries to ../yack-ref/bin/corpus-<HEAD>/ and grades every case
# from those copies under `nice -n 10`, never holding the builder. It exits nonzero if any case
# is worse than scores/corpus/smlib.json (the ratchet; `--bless` is the lead's, by hand). There's
# no scheduler here: run it by hand or from the user's own cron. Before a run in a busy swarm, ask
# the lead or w-m04 whether a digest pair is running (its two halves have a gap between them).
set -euo pipefail
cd "$(dirname "$0")/.."
JOBS=4
TIMEOUT=120
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) JOBS="$2"; shift 2 ;;
        --timeout) TIMEOUT="$2"; shift 2 ;;
        *) echo "usage: $0 [--jobs N] [--timeout S]" >&2; exit 2 ;;
    esac
done
STEPS=('cargo build --release -p yack-corpus -p yack-corpus-fetch' 'cargo build -p xtask' 'cargo xtask corpus smlib --fetch')
if [ -d swarm/req ]; then
    test -e swarm/FREEZE && { echo "corpus-nightly: swarm/FREEZE is set; try after the window" >&2; exit 1; }
    n=0
    while [ -e "swarm/req/w-m03-api-corpus-nightly-$n.txt" ] || [ -e "swarm/res/w-m03-api-corpus-nightly-$n.log" ]; do n=$((n + 1)); done
    req="w-m03-api-corpus-nightly-$n"
    printf '%s\n' "${STEPS[@]}" > "swarm/req/$req.txt"
    echo "corpus-nightly: queued $req; waiting for the builder"
    until grep -q "$req " swarm/DONE.log 2>/dev/null; do sleep 15; done
    grep -q "$req OK" swarm/DONE.log || { echo "corpus-nightly: $req failed (swarm/res/$req.log)" >&2; exit 1; }
else
    for s in "${STEPS[@]}"; do $s; done
fi
D="../yack-ref/bin/corpus-$(git rev-parse --short HEAD)"
mkdir -p "$D"
cp bin/release/yack-corpus bin/debug/xtask "$D/"
# one digest-class run at a time in a swarm (BRIEF): hold swarm/DIGEST.lock through the grading
if [ -d swarm ]; then
    until mkdir swarm/DIGEST.lock 2>/dev/null; do
        echo "corpus-nightly: swarm/DIGEST.lock is held; waiting" >&2
        sleep 60
    done
    trap 'rmdir swarm/DIGEST.lock' EXIT
fi
echo "corpus-nightly: grading from $D (nice 10, --jobs $JOBS, --timeout $TIMEOUT)"
nice -n 10 "$D/xtask" corpus smlib --bin "$D" --jobs "$JOBS" --timeout "$TIMEOUT"
