#!/usr/bin/env python3
"""M4.1a: move cadrs's tolerance values to yack_tol::cadrs (a pure move; digests must not change)."""
import pathlib, shutil, sys
import os
R = pathlib.Path(os.environ.get("M41A_ROOT") or pathlib.Path(__file__).resolve().parent.parent)
S = pathlib.Path(sys.argv[1])  # the staged cadrs.rs and tol.rs

def sub(path, old, new):
    p = R / path; s = p.read_text()
    if s.count(old) != 1: sys.exit(f"{path}: expected one {old!r}")
    p.write_text(s.replace(old, new))

shutil.copy(S / "cadrs.rs", R / "crates/yack-tol/src/cadrs.rs")
sub("crates/yack-tol/src/lib.rs", "#![deny(missing_docs)]\n", "#![deny(missing_docs)]\n\npub mod cadrs;\n")
shutil.copy(S / "tol.rs", R / "crates/yack-cadrs/src/native/tol.rs")
sub("crates/yack-cadrs/src/native/kernel.rs", "        tol.validate()?;\n", "        super::tol::validate(&tol)?;\n")
sub("crates/yack-cadrs/Cargo.toml", 'nalgebra = { path = "../yack-shim", package = "yack-shim" }\n',
    'nalgebra = { path = "../yack-shim", package = "yack-shim" }\nyack-tol = { path = "../yack-tol" }\n')
p = R / "crates/yack-cadrs/IMPORT.md"
p.write_text(p.read_text().rstrip("\n") + """

## M4.1a: tolerance values in yack-tol

cadrs's tolerance levels and `Tolerances` now live in `yack_tol::cadrs` (values unchanged), and
`native/tol.rs` re-exports them by name; `Tolerances::validate` is the free fn
`native::tol::validate`. `import.py`'s patch of `src/native/tol.rs` (`exact_only`) is obsolete:
`import.py` is frozen since M4.0a and is not re-run; the field is in `yack_tol::cadrs`.
""")
print("m41a applied")
