#!/bin/bash
# M2 pure-move window, run by the builder as one request: pre build + copy, apply the move, post build,
# tests compile, clippy, copy. On any failure after the apply it restores exactly the files the apply
# touched (in yack-springrep's src/ and tests/, and any --external file), never anything else.
# usage: bash scripts/m2-window.sh TAG APPLY.py TEST_FILTER [--touches FILE...] [--external FILE...]
set -u
TAG=$1; APPLY=$2; FILTER=$3; shift 3; tag=$(echo $TAG | tr A-Z a-z)
# --touches FILE...: the springrep files the move edits, when other springrep files hold another
# worker's uncommitted work (lead's ruling for T7); only these must be clean, and the guard over the
# untouched springrep files still catches any save into the others mid-window
EXT=(); APPLY_ARGS=(); ONLY=()
if [ "${1:-}" = "--touches" ]; then shift; while [ $# -gt 0 ] && [ "$1" != "--external" ]; do ONLY+=("$1"); shift; done; fi
if [ "${1:-}" = "--external" ]; then shift; EXT=("$@"); APPLY_ARGS=(--with-external "$@"); fi
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; SR=$Y/crates/yack-springrep
cd $Y
# swarm/FREEZE: every worker checks it before each save (BRIEF), for as long as the window runs; it
# goes on every exit path. Another window's freeze is left alone and this one refuses to start.
if [ -e swarm/FREEZE ]; then echo "$TAG ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "$TAG m2-window since $(date '+%F %T')" > swarm/FREEZE
trap 'rm -f $Y/swarm/FREEZE' EXIT; trap 'exit 130' INT TERM HUP
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench"
EXCL=(":!crates/yack-springrep"); for f in "${EXT[@]}"; do EXCL+=(":!$f"); done
others() { { git diff -- $FC "${EXCL[@]}"; git status --short --untracked-files=all -- $FC "${EXCL[@]}"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

if [ ${#ONLY[@]} -gt 0 ]; then dirty=$(git status --short -- "${ONLY[@]}" "${EXT[@]}"); else dirty=$(git status --short -- crates/yack-springrep "${EXT[@]}"); fi
if [ -n "$dirty" ]; then echo "$TAG ABORT: files the move touches have uncommitted changes:"; echo "$dirty"; exit 1; fi
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
build || { echo "$TAG pre build failed"; exit 1; }
L1=$(lints); echo "$TAG clippy warnings before: $L1"
cargo xtask gate >/dev/null 2>&1; G1=$?; echo "$TAG gate before: rc=$G1"
snap $tag-pre-$PFP; echo "$TAG PRE=$PFP others=$O1"

BK=$(mktemp -d); cp -a $SR/src $BK/src; cp -a $SR/tests $BK/tests
# an external file the move creates (a new crate's) has no backup: restore deletes it, and its empty dirs
for f in "${EXT[@]}"; do mkdir -p "$BK/ext/$(dirname $f)"; [ -e "$f" ] && cp -a "$f" "$BK/ext/$f"; done
touched() {
  for d in src tests; do
    ( cd $SR/$d && find . -type f; cd $BK/$d && find . -type f ) | sort -u | while read -r f; do cmp -s "$SR/$d/$f" "$BK/$d/$f" || echo "$d/$f"; done
  done
}
restore() {
  # plain cp + touch: a restored file must look newer than what the build made from the moved one,
  # or cargo keeps the stale artifacts (cp -a would restore the old mtime)
  for f in $TOUCHED; do if [ -f "$BK/$f" ]; then mkdir -p "$(dirname "$SR/$f")" && cp "$BK/$f" "$SR/$f" && touch "$SR/$f"; else rm -f "$SR/$f"; fi; done
  for f in "${EXT[@]}"; do
    if [ -e "$BK/ext/$f" ]; then cp "$BK/ext/$f" "$f" && touch "$f"; else rm -f "$f"; rmdir -p --ignore-fail-on-non-empty "$(dirname $f)" 2>/dev/null; fi
  done
  # a removed file's module may be the only change: touch the crate root so cargo rebuilds
  touch "$SR/src/lib.rs"
  echo "$TAG restored its own files:" $TOUCHED "${EXT[@]}"
}
python3 $APPLY $SR "${APPLY_ARGS[@]}"; ARC=$?; TOUCHED=$(touched); echo "$TAG touched:" $TOUCHED "${EXT[@]}"
[ $ARC -eq 0 ] || { restore; exit 1; }
build || { echo "$TAG post build failed"; restore; exit 1; }
QFP=$(fp)  # after the post build: the name records the tree that was built
cargo test -p yack-springrep --no-run || { echo "$TAG springrep tests do not compile"; restore; exit 1; }
for pkg in $(for f in "${EXT[@]}"; do case $f in crates/*) echo $f | sed 's|crates/\([^/]*\)/.*|\1|';; esac; done | sort -u); do
  # (a workspace file such as layers.toml has no crate: the gate checks it)
  cargo test -p $pkg --no-run || { echo "$TAG $pkg does not compile"; restore; exit 1; }
done
for pkg in $(for f in "${EXT[@]}"; do case $f in crates/yack-springrep/*) ;; crates/*) echo $f | sed 's|crates/\([^/]*\)/.*|\1|';; esac; done | sort -u); do
  # a shipping crate must be clippy-clean under -D warnings (the springrep ratchet above stays a count)
  # the full output goes to swarm/res/ (the builder's log keeps only the tail), so every site is named
  cargo clippy -p $pkg --all-targets -- -D warnings >/dev/null 2>&1 || { echo "$TAG clippy -D warnings fails in $pkg (all sites: swarm/res/$tag-clippy-$pkg.txt)"; cargo clippy -p $pkg --all-targets -- -D warnings > $Y/swarm/res/$tag-clippy-$pkg.txt 2>&1; grep -A3 '^error' $Y/swarm/res/$tag-clippy-$pkg.txt | grep -- '-->' | head -40; restore; exit 1; }
done
L2=$(lints); echo "$TAG clippy warnings after: $L2"
[ "$L2" -le "$L1" ] || { echo "$TAG clippy warnings rose $L1 -> $L2"; restore; exit 1; }
cargo xtask gate; G2=$?; echo "$TAG gate after: rc=$G2"
[ $G1 -ne 0 ] || [ $G2 -eq 0 ] || { echo "$TAG gate was green and the move makes it fail"; restore; exit 1; }
cargo test -p yack-springrep --lib $FILTER || { echo "$TAG tests fail"; restore; exit 1; }
# a crate the move fills (M3 waves) runs its own unit tests whole; the facade's are too long for a window
for pkg in $(for f in "${EXT[@]}"; do case $f in crates/yack/*|crates/yack-springrep/*) ;; crates/*) echo $f | sed 's|crates/\([^/]*\)/.*|\1|';; esac; done | sort -u); do
  cargo test -p $pkg --lib || { echo "$TAG $pkg tests fail"; restore; exit 1; }
done
snap $tag-post-$QFP; O2=$(others)
# untouched springrep files: compare their start hashes with now
SKIP=" $(echo $TOUCHED | sed "s#/\./#/#g") "; S1=$(echo "$S1_ALL" | while read -r h f; do case "$SKIP" in *" $f "*) ;; *) echo "$h  $f";; esac; done | md5sum | cut -c1-8); S2=$(srhash)
echo "$TAG POST=$QFP others=$O2 springrep-untouched=$S1->$S2"
if [ "$O1" != "$O2" ]; then echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
if [ "$S1" != "$S2" ]; then echo "$TAG yack-springrep files outside the move CHANGED during window: pair contaminated"; exit 3; fi
echo "$TAG others unchanged"
