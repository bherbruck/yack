#!/usr/bin/env bash
# yack: builds an UNCOMMITTED variant (b) yack-bench for grading held cases (lead ruling): a held
# solid's cadrs-written STEP is returned when cadrs's reader agrees, springrep's read-back not
# required. Applies the change, builds, copies the binary out, restores step.rs, and checks it.
set -euo pipefail
cd "$(dirname "$0")/.."
F=crates/yack/src/step.rs
OUT=../yack-ref/bin/w-m03-variant-b
before=$(sha256sum "$F" | cut -d' ' -f1)
cp "$F" /tmp/w-m03-step.rs.orig
trap 'cp /tmp/w-m03-step.rs.orig "$F"' EXIT
python3 - "$F" <<'PY'
import sys
p=sys.argv[1]; s=open(p).read()
a='''            sr::check_written(&text, v, a).map_err(|e| Error::failed(op, Stage::Check, format!("springrep (step::read): {e}")))?;
            yack_bridge::check_written_cadrs(&text, v, a).map_err(|e| Error::failed(op, Stage::Check, format!("cadrs: {e}")))?;
            Ok(text)'''
b='''            yack_bridge::check_written_cadrs(&text, v, a).map_err(|e| Error::failed(op, Stage::Check, format!("cadrs: {e}")))?;
            Ok(text)'''
assert s.count(a)==1, "held branch not found"
open(p,'w').write(s.replace(a,b))
PY
cargo build -p yack-bench --release
mkdir -p "$OUT"
cp bin/release/yack-bench "$OUT/yack-bench"
cp /tmp/w-m03-step.rs.orig "$F"
trap - EXIT
after=$(sha256sum "$F" | cut -d' ' -f1)
[ "$before" = "$after" ] || { echo "step.rs NOT restored"; exit 1; }
# rebuild from the restored tree, so bin/ never keeps the variant
cargo build -p yack-bench --release
echo "variant (b) yack-bench at $OUT/yack-bench; step.rs restored ($after); bin/ rebuilt"
