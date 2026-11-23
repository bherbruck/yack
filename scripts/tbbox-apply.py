# M2 (lead's ruling: face_box down; the loose box): the face-box family moves from measure.rs to a new
# topo/bbox.rs (topo now has numeric), so distance and mesh reach it in topo, below them. measure
# re-exports what other modules still name through it. Paths only.
# Usage: python3 tbbox-apply.py <yack-springrep dir>
import os, re, sys

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


# (signature start, name), in their order in measure.rs
ITEMS = [
    ("pub(crate) fn uv_polygons(", "uv_polygons"),
    ("fn uv_polygons_at(", "uv_polygons_at"),
    ("fn winding(", "winding"),
    ("pub(crate) fn in_face(", "in_face"),
    ("fn stationary(", "stationary"),
    ("fn nurbs_starts(", "nurbs_starts"),
    ("fn uv_bounds(", "uv_bounds"),
    ("fn reached_patches<'a>(", "reached_patches"),
    ("fn patches_in<'a>(", "patches_in"),
    ("pub fn loose_bbox(", "loose_bbox"),
    ("pub fn face_bbox(", "face_bbox"),
    ("pub(crate) fn face_box(", "face_box"),
    ("fn face_loose_box(", "face_loose_box"),
    ("pub(crate) fn face_hull_box(", "face_hull_box"),
    ("pub fn bbox(", "bbox"),
    ("fn tight_or_loose_bbox(", "tight_or_loose_bbox"),
]

m = rd("measure.rs")
moved, vis = [], {}
for start, name in ITEMS:
    if m.count("\n" + start) != 1:
        raise SystemExit(f"measure.rs: {start!r} not found once")
    i = m.index("\n" + start) + 1
    j = m.rfind("\n\n", 0, i) + 2           # its doc comment
    k = m.index("\n}\n", i) + 3
    text = m[j:k]
    vis[name] = "pub" if start.startswith("pub fn") else "pub(crate)"
    # in topo every moved fn is reachable from measure
    text = re.sub(r"(?m)^fn " + re.escape(name), "pub(crate) fn " + name, text, count=1)
    moved.append(text)
    m = m[:j] + m[k:]

body = "\n".join(t.rstrip("\n") + "\n" for t in moved)
uses = []
for name in ["Aabb", "Point3", "Vec2", "Vec3"]:
    if re.search(r"\b" + name + r"\b", body):
        uses.append(name)
geom = [n for n in ["NurbsSurface", "Patch", "Surface"] if re.search(r"\b" + n + r"\b", body)]
num = [n for n in ["solve2"] if re.search(r"\b" + n + r"\b", body)]
wr("topo/bbox.rs", f"""// yack: boxes round a face and an arena, and the parameter-plane outlines and stationary points they use.
// Adapted from springrep@a03e8a1 src/measure.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from measure.rs to topo/bbox.rs (M2), so distance and mesh reach a face's box below
// them; private helpers made pub(crate) where measure still calls them. Paths only.
//! Boxes round a face (its loose box, kept on its memo, and its hull box) and round an arena, with the
//! parameter-plane outlines, winding test and stationary-point search they are found by.

use super::{{Brep, FaceId, coedge_params}};
use crate::{{
    geom::{{{", ".join(geom)}}},
    math::{{{", ".join(uses)}}},
{"    numeric::" + num[0] + "," + chr(10) if num else ""}}};

{body}""")

# what remaining measure code, and other files through `measure::`, still name
names = [n for _, n in ITEMS]
code = "\n".join(l for l in m.splitlines() if not l.lstrip().startswith("//"))
local = [n for n in names if re.search(r"(?<![.\w])" + n + r"\b", code)]
outside = set()
for dp, _, fs in os.walk(src):
    for f in fs:
        if f.endswith(".rs") and os.path.join(dp, f) != p("measure.rs") and "/topo/bbox.rs" not in os.path.join(dp, f):
            s = open(os.path.join(dp, f)).read()
            s = "\n".join(l for l in s.splitlines() if not l.lstrip().startswith("//"))
            for n in names:
                if re.search(r"measure::" + n + r"\b", s) or (dp.endswith("measure") and re.search(r"(?<![.\w])" + n + r"\b", s)):
                    outside.add(n)
pub_re = sorted(n for n in names if vis[n] == "pub")            # public API stays on measure
crate_re = sorted(n for n in names if vis[n] != "pub" and (n in local or n in outside))
reexp = ""
if pub_re:
    reexp += "pub use crate::topo::bbox::{" + ", ".join(pub_re) + "};\n"
if crate_re:
    reexp += "pub(crate) use crate::topo::bbox::{" + ", ".join(crate_re) + "};\n"
mt = re.search(r"(?m)^pub use tolerant::[^;]*;\n", m)
if not mt:
    raise SystemExit("measure.rs: no `pub use tolerant::..;` line to place the re-exports after")
m = m[:mt.end()] + reexp + m[mt.end():]
m = m.replace("// Changes: none (verbatim); this header added.",
              "// Changes: the face-box family moved to topo/bbox.rs (M2), re-exported here.", 1)
# imports only the moved code used
for n in ["NurbsSurface", "Patch"]:
    if len(re.findall(r"\b" + n + r"\b", code)) <= 1:
        m = m.replace(n + ", ", "", 1)
wr("measure.rs", m)

edit("topo/mod.rs", [("mod facts;\n", "pub(crate) mod bbox;\nmod facts;\n")])
# the two edges this clears: name the boxes in topo
edit("distance.rs", [("crate::measure::face_box(", "crate::topo::bbox::face_box(")])
edit("mesh/mod.rs", [("crate::measure::loose_bbox(", "crate::topo::bbox::loose_bbox(")])
print("ok: moved", len(moved), "items; measure re-exports", pub_re, crate_re)
