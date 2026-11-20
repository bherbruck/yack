#!/usr/bin/env bash
# yack: the M4.0a freeze window in one builder request (plan/m4-cadrs-map.md): hash the frozen
# crates, build pre, apply scripts/m40a.py to crates/yack-cadrs, check it equals the dry-run copy,
# build post, re-hash. Aborts (and restores yack-cadrs) on any mismatch.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
REF=/home/brenn/github/yack-ref
POSTCOPY=$REF/cadrs-on-shim/m40a-copy
fp() { echo "$(git rev-parse --short HEAD)+$(git diff HEAD | sha1sum | cut -c1-8)"; }
frozen() { { git diff -- crates ':!crates/yack-cadrs'; git status --short --untracked-files=all -- crates ':!crates/yack-cadrs'; } | md5sum | cut -d' ' -f1; }
build() {
  cargo build -p xtask
  cargo build --release -p yack-conformance -p yack-bench
}
keep() { # $1 tag
  local d=$REF/bin/m40a-$1-$2
  mkdir -p "$d"
  cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask "$d/"
  echo "copied to $d"
}

if [ -n "$(git status --short -- crates/yack-cadrs)" ]; then
  echo "ABORT: crates/yack-cadrs is not clean"; git status --short -- crates/yack-cadrs; exit 1
fi
H1=$(frozen)
echo "frozen-hash-before=$H1"

build
PRE=$(fp)
echo "pre-fp=$PRE"
keep pre "$PRE"

BACKUP=$(mktemp -d)
cp -a crates/yack-cadrs/src crates/yack-cadrs/import.py "$BACKUP/"
restore() {
  echo "restoring crates/yack-cadrs"
  rm -rf crates/yack-cadrs/src
  cp -a "$BACKUP/src" crates/yack-cadrs/src
  cp -a "$BACKUP/import.py" crates/yack-cadrs/import.py
}
if ! M40A_ROOT="$ROOT/crates/yack-cadrs" python3 scripts/m40a.py; then
  echo "ABORT: m40a.py failed"; restore; exit 1
fi
if ! diff -r crates/yack-cadrs/src "$POSTCOPY/src" || ! diff crates/yack-cadrs/import.py "$POSTCOPY/import.py"; then
  echo "ABORT: in-tree result differs from the dry-run copy"; restore; exit 1
fi
echo "in-tree result identical to $POSTCOPY"

if ! build || ! cargo test -p yack-cadrs --no-run; then
  echo "ABORT: post build failed"; restore; exit 1
fi
POST=$(fp)
echo "post-fp=$POST"
keep post "$POST"

H2=$(frozen)
echo "frozen-hash-after=$H2"
if [ "$H1" != "$H2" ]; then
  echo "WARNING: the frozen crates changed during the window (pre binaries not comparable)"; exit 2
fi
echo "M40A WINDOW OK pre=$PRE post=$POST frozen=$H1"
