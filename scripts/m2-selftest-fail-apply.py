# m2-window.sh self-test (w-m04, --no-pre): `noop` changes nothing (the window must pass, leaving the
# tree as it was); `fail` appends a comment line to springrep's lib.rs and exits 1 (the window must
# restore it); `touch` appends the line and exits 0 (for --extra: a failing extra must restore it).
# Usage: python3 m2-selftest-apply.py <yack-springrep dir> (noop); the copies m2-selftest-fail-apply.py
# and m2-selftest-touch-apply.py run `fail` and `touch`
import os, sys

root = sys.argv[1]
name = os.path.basename(sys.argv[0])
mode = "fail" if name.startswith("m2-selftest-fail") else "touch" if name.startswith("m2-selftest-touch") else "noop"
if mode in ("fail", "touch"):
    p = os.path.join(root, "src", "lib.rs")
    open(p, "a").write("// m2-window self-test: this line must be restored away\n")
    sys.exit(1 if mode == "fail" else 0)
print("noop")
