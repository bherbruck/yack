#!/bin/bash
# yack: the W11 window (M3, lead): the move that deletes yack-springrep, run by the builder as one request, restored whole on any failure.
# m2-window.sh lints and tests springrep itself, so it cannot run the move that deletes it; this is its
# variant. The apply deletes crates/yack-springrep (and fills the crates it moves into); the window then
# builds, checks and tests what is left, and on any failure brings springrep back whole from HEAD
# (`git checkout` of the directory, after removing whatever the apply left there) and restores every
# --external file, Cargo.lock and the workspace Cargo.toml from its backup.
# usage: bash scripts/w11-window.sh TAG APPLY.py [--no-pre] [--dirty-ok FILE...] [--external FILE...] [--extra "CMD"]...
#   APPLY.py is run as `python3 APPLY.py <yack-springrep dir> [--with-external FILE...]` (as m2-window's).
#   --no-pre: no pre build and no pre binaries (the gate before still runs).
#   --dirty-ok FILE...: externals another worker holds uncommitted hunks in; the backup keeps them.
#   --external FILE...: every file outside crates/yack-springrep the apply writes, creates or deletes.
#   --extra "CMD" (repeatable, anywhere): run with `bash -c` after the window's own checks, before the
#   post snapshot, inside the same backup.
# Checks after the apply, each fatal (restore, exit 1):
#   the post build (xtask, release yack-conformance and yack-bench), the tests of `yack`, `yack-bridge`
#   and every --external crate compiling, clippy -D warnings (--no-deps) on each of those that is not
#   kind = "temporary" in layers.toml, `cargo xtask api`, the gate (fatal when it was green before),
#   `cargo test -p yack` whole (its moved tests in crates/yack/tests included), the --lib tests of
#   yack-bridge and every --external crate, then the extras.
set -u
# a verdict line goes to stdout as before and to stderr too: the builder keeps stdout's tail only, before
# stderr, so a failed window's reason would otherwise be lost (w-h1-59, lead)
say() { echo "$*"; echo "$*" >&2; }
TAG=$1; APPLY=$2; shift 2; tag=$(echo $TAG | tr A-Z a-z)
EXTRA=(); REST=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--extra" ]; then [ $# -ge 2 ] || { say "$TAG ABORT: --extra needs a command"; exit 1; }; EXTRA+=("$2"); shift 2
  else REST+=("$1"); shift; fi
done
set -- "${REST[@]}"
NOPRE=0; if [ "${1:-}" = "--no-pre" ]; then NOPRE=1; shift; fi
DIRTY_OK=(); EXT=(); APPLY_ARGS=()
if [ "${1:-}" = "--dirty-ok" ]; then shift; while [ $# -gt 0 ] && [ "$1" != "--external" ]; do DIRTY_OK+=("$1"); shift; done; fi
if [ "${1:-}" = "--external" ]; then shift; EXT=("$@"); APPLY_ARGS=(--with-external "$@"); set --; fi
[ $# -eq 0 ] || { say "$TAG ABORT: unread arguments: $*"; exit 1; }
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; SR=$Y/crates/yack-springrep; SRREL=crates/yack-springrep
cd $Y
# the whole run, both streams, in swarm/res/<tag>-window.txt
WLOG=$Y/swarm/res/$tag-window.txt; : > "$WLOG"
exec > >(tee -a "$WLOG") 2> >(tee -a "$WLOG" >&2)
if [ -e swarm/FREEZE ]; then say "$TAG ABORT: swarm/FREEZE is held: $(cat swarm/FREEZE)"; exit 1; fi
echo "$TAG w11-window since $(date '+%F %T')" > swarm/FREEZE
trap 'rm -f $Y/swarm/FREEZE' EXIT; trap 'exit 130' INT TERM HUP
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench"
EXCL=(":!$SRREL"); for f in "${EXT[@]}"; do EXCL+=(":!$f"); done
others() { { git diff -- $FC "${EXCL[@]}"; git status --short --untracked-files=all -- $FC "${EXCL[@]}"; } | md5sum | cut -c1-8; }
temporary() { awk -v c="[crate.$1]" '$0 == c { on = 1; next } /^\[/ { on = 0 } on && /^kind *= *"temporary"/ { t = 1 } END { exit !t }' $Y/layers.toml; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

# springrep must be exactly HEAD's (tracked and untracked), or a restore from HEAD would lose work
dirty=$(git status --short --untracked-files=all -- $SRREL)
[ -z "$dirty" ] || { say "$TAG ABORT: $SRREL differs from HEAD, so a restore from HEAD would lose it:"; echo "$dirty"; exit 1; }
CHECK=(); for f in "${EXT[@]}"; do case " ${DIRTY_OK[*]} " in *" $f "*) ;; *) CHECK+=("$f");; esac; done
if [ ${#CHECK[@]} -gt 0 ]; then dirty=$(git status --short -- "${CHECK[@]}"); else dirty=""; fi
for f in "${DIRTY_OK[@]}"; do say "$TAG dirty-ok $f pre-hash $(md5sum < $f | cut -c1-8)"; done
[ -z "$dirty" ] || { say "$TAG ABORT: files the move touches have uncommitted changes:"; echo "$dirty"; exit 1; }
O1=$(others); PFP=$(fp)
if [ $NOPRE -eq 0 ]; then build || { say "$TAG pre build failed"; exit 1; }; fi
cargo xtask gate >/dev/null 2>&1; G1=$?; say "$TAG gate before: rc=$G1"
if [ $NOPRE -eq 0 ]; then snap $tag-pre-$PFP; say "$TAG PRE=$PFP others=$O1"; else say "$TAG --no-pre: no pre build (tree $PFP) others=$O1"; fi

# the externals, Cargo.lock and the workspace Cargo.toml (its members lose springrep) are backed up; an
# external the move creates has no backup, so a restore deletes it (and its empty directories)
BK=$(mktemp -d); BACKED=("${EXT[@]}")
for f in Cargo.toml Cargo.lock; do case " ${BACKED[*]} " in *" $f "*) ;; *) BACKED+=("$f");; esac; done
for f in "${BACKED[@]}"; do mkdir -p "$BK/ext/$(dirname $f)"; [ -e "$f" ] && cp -a "$f" "$BK/ext/$f"; done
restore() {
  # springrep whole from HEAD: what the apply left there goes first, so no stray file survives; checkout
  # writes fresh mtimes, so cargo rebuilds from it
  rm -rf "$SR"; git checkout HEAD -- $SRREL && touch "$SR/src/lib.rs"
  for f in "${BACKED[@]}"; do
    if [ -e "$BK/ext/$f" ]; then mkdir -p "$(dirname "$f")" && cp "$BK/ext/$f" "$f" && touch "$f"; else rm -f "$f"; rmdir -p --ignore-fail-on-non-empty "$(dirname $f)" 2>/dev/null; fi
  done
  left=$(git status --short --untracked-files=all -- $SRREL)
  if [ -n "$left" ]; then say "$TAG RESTORE INCOMPLETE: $SRREL still differs from HEAD:"; echo "$left"; else say "$TAG restored $SRREL whole from HEAD"; fi
  say "$TAG restored its files:" "${BACKED[@]}"
}
python3 $APPLY $SR "${APPLY_ARGS[@]}"; ARC=$?
if [ -d "$SR" ]; then say "$TAG apply rc=$ARC; $SRREL still present"; else say "$TAG apply rc=$ARC; $SRREL deleted"; fi
[ $ARC -eq 0 ] || { restore; exit 1; }
build || { say "$TAG post build failed"; restore; exit 1; }
QFP=$(fp)
crates() { { echo yack; echo yack-bridge; for f in "${EXT[@]}"; do case $f in crates/*) echo $f | sed 's|crates/\([^/]*\)/.*|\1|';; esac; done; } | sort -u | while read -r c; do [ -f "crates/$c/Cargo.toml" ] && echo $c; done; }
for pkg in $(crates); do
  cargo test -p $pkg --no-run > $Y/swarm/res/$tag-tests-$pkg.txt 2>&1 || { say "$TAG $pkg tests do not compile (all of it: swarm/res/$tag-tests-$pkg.txt)"; grep -A3 '^error' $Y/swarm/res/$tag-tests-$pkg.txt | grep -- '-->' | head -40; restore; exit 1; }
done
for pkg in $(crates); do
  if temporary $pkg; then say "$TAG clippy: $pkg is temporary, skipped"; continue; fi
  cargo clippy -p $pkg --all-targets --no-deps -- -D warnings > $Y/swarm/res/$tag-clippy-$pkg.txt 2>&1 || { say "$TAG clippy -D warnings fails in $pkg (all sites: swarm/res/$tag-clippy-$pkg.txt)"; grep -A3 '^error' $Y/swarm/res/$tag-clippy-$pkg.txt | grep -- '-->' | head -40; restore; exit 1; }
done
cargo xtask api || { say "$TAG xtask api fails"; restore; exit 1; }
cargo xtask gate; G2=$?; say "$TAG gate after: rc=$G2"
[ $G1 -ne 0 ] || [ $G2 -eq 0 ] || { say "$TAG gate was green and the move makes it fail"; restore; exit 1; }
cargo test -p yack || { say "$TAG yack tests fail"; restore; exit 1; }
for pkg in $(crates); do
  [ "$pkg" = yack ] && continue
  cargo test -p $pkg --lib || { say "$TAG $pkg tests fail"; restore; exit 1; }
done
for cmd in "${EXTRA[@]}"; do
  say "$TAG extra: $cmd"
  bash -c "$cmd" || { say "$TAG extra fails: $cmd"; restore; exit 1; }
done
snap $tag-post-$QFP; O2=$(others)
say "$TAG POST=$QFP others=$O2"
if [ "$O1" != "$O2" ]; then say "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
say "$TAG others unchanged"
