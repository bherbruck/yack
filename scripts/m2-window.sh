#!/bin/bash
# M2 pure-move window, run by the builder as one request: pre build + copy, apply the move, post build,
# tests compile, clippy, copy. On any failure after the apply it restores exactly the files the apply
# touched (in yack-springrep's src/ and tests/, and any --external file), never anything else.
# usage: bash scripts/m2-window.sh TAG APPLY.py TEST_FILTER [--no-pre] [--touches FILE...] [--dirty-ok FILE...] [--external FILE...] [--extra "CMD"]...
# --extra "CMD" (lead, W9c+W9d; repeatable, anywhere after TEST_FILTER): each CMD runs (bash -c) after the
# window's own gate and tests, before the post snapshot, inside the same backup: a failing one restores
# everything like any other step.
# --no-pre (lead, for pure moves the lead's whole-tree digest check proves): no pre build and no pre
# binaries; the apply, the post build and copy, clippy, tests, the gate (before and after) and the
# restore on failure all stay. Without it the window is as before.
set -u
# a verdict line goes to stdout as before and to stderr too: the builder keeps stdout's tail only, before
# stderr, so a failed window's reason would otherwise be lost (w-h1-59, lead)
say() { echo "$*"; echo "$*" >&2; }
TAG=$1; APPLY=$2; FILTER=$3; shift 3; tag=$(echo $TAG | tr A-Z a-z)
EXTRA=(); REST=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--extra" ]; then [ $# -ge 2 ] || { say "$TAG ABORT: --extra needs a command"; exit 1; }; EXTRA+=("$2"); shift 2
  else REST+=("$1"); shift; fi
done
set -- "${REST[@]}"
NOPRE=0; if [ "${1:-}" = "--no-pre" ]; then NOPRE=1; shift; fi
# --touches FILE...: the springrep files the move edits, when other springrep files hold another
# worker's uncommitted work (lead's ruling for T7); only these must be clean, and the guard over the
# untouched springrep files still catches any save into the others mid-window
EXT=(); APPLY_ARGS=(); ONLY=()
if [ "${1:-}" = "--touches" ]; then shift; while [ $# -gt 0 ] && [ "$1" != "--external" ] && [ "$1" != "--dirty-ok" ]; do ONLY+=("$1"); shift; done; fi
# --dirty-ok FILE...: externals another worker holds uncommitted hunks in (lead's ruling, W7): they may
# be dirty at the start; the backup keeps their hunks, so a restore puts them back as they were
DIRTY_OK=()
if [ "${1:-}" = "--dirty-ok" ]; then shift; while [ $# -gt 0 ] && [ "$1" != "--external" ]; do DIRTY_OK+=("$1"); shift; done; fi
if [ "${1:-}" = "--external" ]; then shift; EXT=("$@"); APPLY_ARGS=(--with-external "$@"); fi
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; SR=$Y/crates/yack-springrep
cd $Y
# the whole run, both streams, in swarm/res/<tag>-window.txt
WLOG=$Y/swarm/res/$tag-window.txt; : > "$WLOG"
exec > >(tee -a "$WLOG") 2> >(tee -a "$WLOG" >&2)
# swarm/FREEZE: every worker checks it before each save (BRIEF), for as long as the window runs; it
# goes on every exit path. Another window's freeze is left alone and this one refuses to start.
if [ -e swarm/FREEZE ]; then say "$TAG ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "$TAG m2-window since $(date '+%F %T')" > swarm/FREEZE
trap 'rm -f $Y/swarm/FREEZE' EXIT; trap 'exit 130' INT TERM HUP
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench"
EXCL=(":!crates/yack-springrep"); for f in "${EXT[@]}"; do EXCL+=(":!$f"); done
others() { { git diff -- $FC "${EXCL[@]}"; git status --short --untracked-files=all -- $FC "${EXCL[@]}"; } | md5sum | cut -c1-8; }
# whether layers.toml's row for crate $1 says kind = "temporary"
temporary() { awk -v c="[crate.$1]" '$0 == c { on = 1; next } /^\[/ { on = 0 } on && /^kind *= *"temporary"/ { t = 1 } END { exit !t }' $Y/layers.toml; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

CHECK=(); for f in "${EXT[@]}"; do case " ${DIRTY_OK[*]} " in *" $f "*) ;; *) CHECK+=("$f");; esac; done
if [ ${#ONLY[@]} -gt 0 ]; then dirty=$(git status --short -- "${ONLY[@]}" "${CHECK[@]}"); else dirty=$(git status --short -- crates/yack-springrep "${CHECK[@]}"); fi
for f in "${DIRTY_OK[@]}"; do say "$TAG dirty-ok $f pre-hash $(md5sum < $f | cut -c1-8)"; done
if [ -n "$dirty" ]; then say "$TAG ABORT: files the move touches have uncommitted changes:"; echo "$dirty"; exit 1; fi
# yack-springrep's own files outside the move: hashed at the start, and again at the end over every file
# the apply did not touch, so another worker's save into the crate mid-window is caught too
srhash() { ( cd $SR && find src tests -type f | sort | while read -r f; do case " $SKIP " in *" $f "*) ;; *) md5sum "$f";; esac; done ) | md5sum | cut -c1-8; }
SKIP=""; S1_ALL=$( (cd $SR && find src tests -type f | sort | xargs md5sum) )
O1=$(others); PFP=$(fp)
# springrep is a temporary crate under a lints ratchet: the warning count may not rise
# (counted at springrep's own lines: a crate the move fills carries the workspace lints, whose
# warn-level ones, indexing_slicing until M5, would otherwise count against it; its denies are
# checked below by clippy's exit status)
lints() { cargo clippy -p yack-springrep --all-targets 2>&1 | grep -c -- '--> crates/yack-springrep/' ; }
if [ $NOPRE -eq 0 ]; then build || { say "$TAG pre build failed"; exit 1; }; fi
L1=$(lints); say "$TAG clippy warnings before: $L1"
cargo xtask gate >/dev/null 2>&1; G1=$?; say "$TAG gate before: rc=$G1"
if [ $NOPRE -eq 0 ]; then snap $tag-pre-$PFP; say "$TAG PRE=$PFP others=$O1"; else say "$TAG --no-pre: no pre build (tree $PFP) others=$O1"; fi

BK=$(mktemp -d); cp -a $SR/src $BK/src; cp -a $SR/tests $BK/tests
# an external file the move creates (a new crate's) has no backup: restore deletes it, and its empty dirs.
# springrep's Cargo.toml and the workspace's Cargo.lock are always backed up (w-h4, W8): an apply adds the
# new crate's dependency there, and a failed window must not leave the workspace pointing at it
# (lead, W9) likewise every --touches path outside springrep's src/ and tests/, which the src/tests
# backup below does not cover
BACKED=("${EXT[@]}")
for f in crates/yack-springrep/Cargo.toml Cargo.lock "${ONLY[@]}"; do
  case $f in crates/yack-springrep/src/*|crates/yack-springrep/tests/*) continue;; esac
  case " ${BACKED[*]} " in *" $f "*) ;; *) BACKED+=("$f");; esac
done
for f in "${BACKED[@]}"; do mkdir -p "$BK/ext/$(dirname $f)"; [ -e "$f" ] && cp -a "$f" "$BK/ext/$f"; done
touched() {
  for d in src tests; do
    ( cd $SR/$d && find . -type f; cd $BK/$d && find . -type f ) | sort -u | while read -r f; do cmp -s "$SR/$d/$f" "$BK/$d/$f" || echo "$d/$f"; done
  done
}
restore() {
  # plain cp + touch: a restored file must look newer than what the build made from the moved one,
  # or cargo keeps the stale artifacts (cp -a would restore the old mtime)
  for f in $TOUCHED; do if [ -f "$BK/$f" ]; then mkdir -p "$(dirname "$SR/$f")" && cp "$BK/$f" "$SR/$f" && touch "$SR/$f"; else rm -f "$SR/$f"; fi; done
  for f in "${BACKED[@]}"; do
    if [ -e "$BK/ext/$f" ]; then cp "$BK/ext/$f" "$f" && touch "$f"; else rm -f "$f"; rmdir -p --ignore-fail-on-non-empty "$(dirname $f)" 2>/dev/null; fi
  done
  # a removed file's module may be the only change: touch the crate root so cargo rebuilds
  touch "$SR/src/lib.rs"
  say "$TAG restored its own files:" $TOUCHED "${EXT[@]}"
}
python3 $APPLY $SR "${APPLY_ARGS[@]}"; ARC=$?; TOUCHED=$(touched); say "$TAG touched:" $TOUCHED "${EXT[@]}"
[ $ARC -eq 0 ] || { restore; exit 1; }
build || { say "$TAG post build failed"; restore; exit 1; }
QFP=$(fp)  # after the post build: the name records the tree that was built
# full output kept in swarm/res/ (the builder's log keeps only the tail): every diagnostic's site
cargo test -p yack-springrep --no-run > $Y/swarm/res/$tag-springrep-tests.txt 2>&1 || { say "$TAG springrep tests do not compile (all of it: swarm/res/$tag-springrep-tests.txt)"; grep -A3 '^error\|^warning' $Y/swarm/res/$tag-springrep-tests.txt | grep -- '-->' | head -40; restore; exit 1; }
grep -A3 '^warning' $Y/swarm/res/$tag-springrep-tests.txt | grep -- '-->' | sort -u | head -20
for pkg in $(for f in "${EXT[@]}"; do case $f in crates/*) echo $f | sed 's|crates/\([^/]*\)/.*|\1|';; esac; done | sort -u); do
  # (a workspace file such as layers.toml has no crate: the gate checks it)
  cargo test -p $pkg --no-run || { say "$TAG $pkg does not compile"; restore; exit 1; }
done
for pkg in $(for f in "${EXT[@]}"; do case $f in crates/yack-springrep/*) ;; crates/*) echo $f | sed 's|crates/\([^/]*\)/.*|\1|';; esac; done | sort -u); do
  # a shipping crate must be clippy-clean under -D warnings (the springrep ratchet above stays a count);
  # a temporary crate (layers.toml kind = "temporary": shim, cadrs, bridge, ...) is exempt (BRIEF, lead W10),
  # and --no-deps keeps a temporary dependency's lints from failing a kernel crate
  if temporary $pkg; then say "$TAG clippy: $pkg is temporary, skipped"; continue; fi
  # the full output goes to swarm/res/ (the builder's log keeps only the tail), so every site is named
  cargo clippy -p $pkg --all-targets --no-deps -- -D warnings >/dev/null 2>&1 || { say "$TAG clippy -D warnings fails in $pkg (all sites: swarm/res/$tag-clippy-$pkg.txt)"; cargo clippy -p $pkg --all-targets --no-deps -- -D warnings > $Y/swarm/res/$tag-clippy-$pkg.txt 2>&1; grep -A3 '^error' $Y/swarm/res/$tag-clippy-$pkg.txt | grep -- '-->' | head -40; restore; exit 1; }
done
L2=$(lints); say "$TAG clippy warnings after: $L2"
[ "$L2" -le "$L1" ] || { say "$TAG clippy warnings rose $L1 -> $L2"; restore; exit 1; }
cargo xtask gate; G2=$?; say "$TAG gate after: rc=$G2"
[ $G1 -ne 0 ] || [ $G2 -eq 0 ] || { say "$TAG gate was green and the move makes it fail"; restore; exit 1; }
cargo test -p yack-springrep --lib $FILTER || { say "$TAG tests fail"; restore; exit 1; }
# a crate the move fills (M3 waves) runs its own unit tests whole; the facade's are too long for a window
for pkg in $(for f in "${EXT[@]}"; do case $f in crates/yack/*|crates/yack-springrep/*) ;; crates/*) echo $f | sed 's|crates/\([^/]*\)/.*|\1|';; esac; done | sort -u); do
  cargo test -p $pkg --lib || { say "$TAG $pkg tests fail"; restore; exit 1; }
done
for cmd in "${EXTRA[@]}"; do
  say "$TAG extra: $cmd"
  bash -c "$cmd" || { say "$TAG extra fails: $cmd"; restore; exit 1; }
done
snap $tag-post-$QFP; O2=$(others)
# untouched springrep files: compare their start hashes with now
SKIP=" $(echo $TOUCHED | sed "s#/\./#/#g") "; S1=$(echo "$S1_ALL" | while read -r h f; do case "$SKIP" in *" $f "*) ;; *) echo "$h  $f";; esac; done | md5sum | cut -c1-8); S2=$(srhash)
say "$TAG POST=$QFP others=$O2 springrep-untouched=$S1->$S2"
if [ "$O1" != "$O2" ]; then say "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
if [ "$S1" != "$S2" ]; then say "$TAG yack-springrep files outside the move CHANGED during window: pair contaminated"; exit 3; fi
say "$TAG others unchanged"
