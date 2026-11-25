# yack: M3 W9a-0: springrep's blend.rs loses its hand-run SPRINGREP_BLEND_VERIFY mode (the env read behind a
# global OnceLock, two clocks and `compare`), which no bench, conformance or test run sets: with it unset
# the sewn result or else the tools' is returned, as before. The edited file is staged out of the tree.
# Usage: python3 w9a0-apply.py <yack-springrep dir>
import os, shutil, sys

STAGED = "/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/w-h1/w9a0/crates/yack-springrep/src/blend.rs"
dst = os.path.join(os.path.abspath(sys.argv[1]), "src", "blend.rs")
text = open(dst).read()
for gone in ("SPRINGREP_BLEND_VERIFY\").is_some()", "fn compare(", "fn verifying()"):
    if gone not in text:
        raise SystemExit(f"blend.rs no longer has {gone!r}: the stage is stale")
shutil.copyfile(STAGED, dst + ".tmp")
os.replace(dst + ".tmp", dst)
print("blend.rs: verify mode removed")
