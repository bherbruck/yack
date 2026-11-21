# M2 (lead's ruling on profile -> topo::FaceName): the naming types are plain data and move down to a
# new src/names.rs (map row `names = []`; yack-base in M3). topo re-exports them, so topo::FaceName and
# topo::EdgeName keep working; profile names them from `names`. Run after T8 (EdgeName is then in
# topo/names.rs). Usage: python3 tnames-apply.py <yack-springrep dir>
import os, sys

src = os.path.join(sys.argv[1], "src")
p = lambda *a: os.path.join(src, *a)


def rd(f):
    return open(p(f)).read()


def wr(f, s):
    open(p(f), "w").write(s)


def edit(f, pairs):
    s = rd(f)
    for a, b in pairs:
        if s.count(a) != 1:
            raise SystemExit(f"{f}: expected one {a[:70]!r}, found {s.count(a)}")
        s = s.replace(a, b)
    wr(f, s)


s = rd("topo/names.rs")
if "pub struct EdgeName" not in s:
    raise SystemExit("topo/names.rs: no EdgeName: run T8 first")
s = s.replace("// yack: springrep module src/topo/names.rs", "// yack: the names features give faces, and the face-name pairs that pick edges: plain data.", 1)
import re
s = re.sub(r"// Changes: .*\n(//.*\n)?", "// Changes: moved from topo/names.rs to names.rs (M2), being plain data topo and profile both use;\n// EdgeName joined it from blend.rs (M2.5, T8).\n", s, count=1)
wr("names.rs", s)
os.remove(p("topo/names.rs"))

edit("topo/mod.rs", [("mod names;\n", ""), ("pub use names::{EdgeName, FaceName};", "pub use crate::names::{EdgeName, FaceName};")])
edit("profile.rs", [("crate::topo::FaceName::SIDE", "crate::names::FaceName::SIDE")])
edit("lib.rs", [("pub mod mesh;\n", "pub mod mesh;\npub mod names;\n")])
print("ok")
