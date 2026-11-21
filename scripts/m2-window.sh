#!/bin/bash
# M2 pure-move window, run by the builder as one request: pre build + copy, apply the move, post build,
# tests compile, clippy, copy. On any failure after the apply it restores exactly the files the apply
# touched (in yack-springrep's src/ and tests/, and any --external file), never anything else.
# usage: bash scripts/m2-window.sh TAG APPLY.py TEST_FILTER [--external FILE...]
set -u
TAG=$1; APPLY=$2; FILTER=$3; shift 3; tag=$(echo $TAG | tr A-Z a-z)
EXT=(); APPLY_ARGS=()
if [ "${1:-}" = "--external" ]; then shift; EXT=("$@"); APPLY_ARGS=(--with-external); fi
Y=/home/brenn/github/yack; R=/home/brenn/github/yack-ref/bin; SR=$Y/crates/yack-springrep
cd $Y
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FC="crates/yack crates/yack-springrep crates/yack-cadrs crates/yack-bridge crates/yack-shim crates/yack-math crates/yack-bench"
EXCL=(":!crates/yack-springrep"); for f in "${EXT[@]}"; do EXCL+=(":!$f"); done
others() { { git diff -- $FC "${EXCL[@]}"; git status --short --untracked-files=all -- $FC "${EXCL[@]}"; } | md5sum | cut -c1-8; }
build() { cargo build -p xtask && cargo build --release -p yack-conformance -p yack-bench; }
snap() { mkdir -p $R/$1 && cp bin/release/yack-conformance bin/release/yack-bench bin/debug/xtask $R/$1/; }

dirty=$(git status --short -- crates/yack-springrep "${EXT[@]}")
if [ -n "$dirty" ]; then echo "$TAG ABORT: files the move touches have uncommitted changes:"; echo "$dirty"; exit 1; fi
O1=$(others); PFP=$(fp)
# springrep is a temporary crate under a lints ratchet: the warning count may not rise
lints() { cargo clippy -p yack-springrep --all-targets 2>&1 | grep -c '^warning: ' ; }
build || { echo "$TAG pre build failed"; exit 1; }
L1=$(lints); echo "$TAG clippy warnings before: $L1"
cargo xtask gate >/dev/null 2>&1; G1=$?; echo "$TAG gate before: rc=$G1"
snap $tag-pre-$PFP; echo "$TAG PRE=$PFP others=$O1"

BK=$(mktemp -d); cp -a $SR/src $BK/src; cp -a $SR/tests $BK/tests
for f in "${EXT[@]}"; do mkdir -p "$BK/ext/$(dirname $f)"; cp -a "$f" "$BK/ext/$f"; done
touched() {
  for d in src tests; do
    ( cd $SR/$d && find . -type f; cd $BK/$d && find . -type f ) | sort -u | while read -r f; do cmp -s "$SR/$d/$f" "$BK/$d/$f" || echo "$d/$f"; done
  done
}
restore() {
  # plain cp + touch: a restored file must look newer than what the build made from the moved one,
  # or cargo keeps the stale artifacts (cp -a would restore the old mtime)
  for f in $TOUCHED; do if [ -f "$BK/$f" ]; then cp "$BK/$f" "$SR/$f" && touch "$SR/$f"; else rm -f "$SR/$f"; fi; done
  for f in "${EXT[@]}"; do cp "$BK/ext/$f" "$f" && touch "$f"; done
  # a removed file's module may be the only change: touch the crate root so cargo rebuilds
  touch "$SR/src/lib.rs"
  echo "$TAG restored its own files:" $TOUCHED "${EXT[@]}"
}
python3 $APPLY $SR "${APPLY_ARGS[@]}"; ARC=$?; TOUCHED=$(touched); echo "$TAG touched:" $TOUCHED "${EXT[@]}"
[ $ARC -eq 0 ] || { restore; exit 1; }
QFP=$(fp)
build || { echo "$TAG post build failed"; restore; exit 1; }
cargo test -p yack-springrep --no-run || { echo "$TAG springrep tests do not compile"; restore; exit 1; }
for f in "${EXT[@]}"; do
  pkg=$(echo $f | sed 's|crates/\([^/]*\)/.*|\1|')
  cargo test -p $pkg --no-run || { echo "$TAG $pkg does not compile"; restore; exit 1; }
done
L2=$(lints); echo "$TAG clippy warnings after: $L2"
[ "$L2" -le "$L1" ] || { echo "$TAG clippy warnings rose $L1 -> $L2"; restore; exit 1; }
cargo xtask gate; G2=$?; echo "$TAG gate after: rc=$G2"
[ $G1 -ne 0 ] || [ $G2 -eq 0 ] || { echo "$TAG gate was green and the move makes it fail"; restore; exit 1; }
cargo test -p yack-springrep --lib $FILTER || { echo "$TAG tests fail"; restore; exit 1; }
snap $tag-post-$QFP; O2=$(others)
echo "$TAG POST=$QFP others=$O2"
if [ "$O1" = "$O2" ]; then echo "$TAG others unchanged"; else echo "$TAG OTHERS CHANGED during window ($O1 -> $O2): pair contaminated"; exit 3; fi
