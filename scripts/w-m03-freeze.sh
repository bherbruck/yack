#!/usr/bin/env bash
# yack: copies the freshly built release yack-conformance out of bin/ into yack-ref/bin/<name>-<fp>/ (<name> the first argument),
# <fp> being the builder's tree fingerprint (HEAD + a hash of the working-tree diff), for a
# conformance run outside the builder.
set -euo pipefail
cd "$(dirname "$0")/.."
fp() { echo "$(git rev-parse --short HEAD)+$( { git diff; git status --short --untracked-files=all; } | md5sum | cut -c1-8)"; }
FP=$(fp)
OUT=../yack-ref/bin/${1:-r7a}-$FP
mkdir -p "$OUT"
cp bin/release/yack-conformance "$OUT/"
sha256sum "$OUT/yack-conformance"
echo "w-m03-freeze: $OUT"
