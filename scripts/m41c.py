#!/usr/bin/env python3
"""M4.1c: move yack-shim's nalgebra-shaped linalg into the new kernel crate yack-linalg (a pure
move; digests must not change). usage: python3 scripts/m41c.py STAGE   (M41C_ROOT overrides the tree)"""
import os, pathlib, shutil, sys
R = pathlib.Path(os.environ.get("M41C_ROOT") or pathlib.Path(__file__).resolve().parent.parent)
S = pathlib.Path(sys.argv[1])  # holds crates/yack-linalg/{Cargo.toml,src/lib.rs}

def sub(path, old, new):
    p = R / path; s = p.read_text()
    if s.count(old) != 1: sys.exit(f"{path}: expected one {old!r}")
    p.write_text(s.replace(old, new))

shim, lin = R / "crates/yack-shim", R / "crates/yack-linalg"
if lin.exists(): sys.exit("crates/yack-linalg exists already")
shutil.copytree(S / "crates/yack-linalg", lin)
mods = sorted(p.name for p in (shim / "src/linalg").glob("*.rs") if p.name != "mod.rs")
assert mods == ["decomp.rs", "dmatrix.rs", "geometry.rs", "matrix.rs", "ops.rs", "point.rs", "storage.rs", "unit.rs"], mods
for m in mods:
    shutil.copy(shim / "src/linalg" / m, lin / "src" / m)
shutil.rmtree(shim / "src/linalg")
sub("crates/yack-linalg/src/storage.rs", "fn out_of_bounds(i: usize, n: usize) -> ! {\n",
    '#[expect(clippy::panic, reason = "an index contract, as nalgebra\'s: counted in scores/lints.txt until M5")]\nfn out_of_bounds(i: usize, n: usize) -> ! {\n')
sub("crates/yack-shim/src/lib.rs", "pub mod linalg;\n", "pub use yack_linalg as linalg;\n")
sub("crates/yack-shim/Cargo.toml", "[dependencies]\n", '[dependencies]\nyack-linalg = { path = "../yack-linalg" }\n')
print("m41c applied")
