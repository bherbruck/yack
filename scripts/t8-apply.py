# M2.5 (T8): `pick` is dissolved. faces_named and edges_named go to blend (their one user and the owner
# of what they wrap), convex_edges to edit (its one user), and EdgeName, plain data, to topo::names, so
# blend <-> pick <-> edit no longer loops. Paths only. Usage: python3 t8-apply.py <yack-springrep dir>
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


def item(s, start):
    """The doc comment and body of the item beginning at `start`, up to its closing brace line."""
    i = s.index(start)
    j = s.rfind("\n\n", 0, i) + 2
    k = s.index("\n}\n", i) + 3
    return s[j:k]


pick = rd("pick.rs")
faces_named = item(pick, "pub fn faces_named(")
convex = item(pick, "pub fn convex_edges(")
edges_named = item(pick, "pub fn edges_named(").replace("blend::picked_edges(", "picked_edges(")
os.remove(p("pick.rs"))
edit("lib.rs", [("pub mod pick;\n", "")])

# EdgeName -> topo/names.rs
bl = rd("blend.rs")
en = item(bl, "pub struct EdgeName<'a> {")
bl = bl.replace(en, "")
en_body = en  # doc + derive + struct
names = rd("topo/names.rs")
names = names.replace("// Changes: none (verbatim); this header added.",
                      "// Changes: EdgeName moved here from blend.rs (M2.5, T8), so pick's dissolving leaves no loop.", 1)
names = names.rstrip("\n") + "\n\n" + en_body
wr("topo/names.rs", names)
edit("topo/mod.rs", [("pub use names::FaceName;", "pub use names::{EdgeName, FaceName};")])

# faces_named and edges_named -> a new blend/named.rs (blend.rs is at its file-size allowance)
bl = bl.replace("use crate::pick::faces_named;\n", "")
bl = bl.replace("// Changes: none (verbatim); this header added.",
                "// Changes: faces_named and edges_named moved in from pick.rs (to blend/named.rs), EdgeName out to\n// topo::names (M2.5, T8).", 1)
anchor = "mod rolling;\n"
if bl.count(anchor) != 1:
    raise SystemExit("blend.rs: no `mod rolling;` line")
bl = bl.replace(anchor, "mod named;\n" + anchor, 1)
first_use = bl.index("\nuse crate::{")
bl = bl[:first_use + 1] + "pub use crate::topo::EdgeName;\npub use named::{edges_named, faces_named};\n" + bl[first_use + 1:]
wr("blend.rs", bl)
wr("blend/named.rs", """// yack: a blend's faces and edges picked by the names faces carry.
// Adapted from springrep@a03e8a1 src/pick.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from pick.rs to blend/named.rs (M2.5, T8), pick being dissolved; paths only.
//! A blend's faces and edges picked by the names faces carry.

use std::collections::BTreeSet;

use super::{has_name, picked_edges};
use crate::{
    Result,
    intersect::ssi::same_domain,
    topo::{Brep, EdgeId, EdgeName, FaceId},
};

""" + faces_named.rstrip("\n") + "\n\n" + edges_named)

# convex_edges -> edit.rs
edit("edit.rs", [("    pick::convex_edges,\n", "")])
s = rd("edit.rs")
k = s.index("\nfn named(segment: Segment, name: &str) -> NamedSegment {")
s = s[:k + 1] + convex + "\n" + s[k + 1:]
s = s.replace("// Changes: none (verbatim); this header added.",
              "// Changes: convex_edges moved in from pick.rs (M2.5, T8).", 1)
if "math::{Aabb, Frame, Point3, Vec2, Vec3}" not in s:
    raise SystemExit("edit.rs: expected Point3 already imported")
wr("edit.rs", s)
# integration tests named pick's functions through the crate: they are blend's now
tdir = os.path.join(sys.argv[1], "tests")
for fn in sorted(os.listdir(tdir)):
    if fn.endswith(".rs"):
        f = os.path.join(tdir, fn)
        t0 = t = open(f).read()
        t = t.replace("pick::edges_named", "blend::edges_named").replace("pick::faces_named", "blend::faces_named")
        if "pick::" in t:
            raise SystemExit(f"tests/{fn}: still names pick::")
        if t != t0:
            open(f, "w").write(t)
            print("tests/" + fn)
print("ok")
