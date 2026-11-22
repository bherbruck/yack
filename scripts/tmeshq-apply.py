# M2 (lead's ruling (c) on rays/distance -> mesh, a cycle locate -> rays -> mesh -> locate): rays and
# distance take a tessellation as input and mesh nothing; the meshing they did (the same MESHES table,
# finest first, the same fallback loop) moves up to mesh/query.rs, which callers now call. MeshOptions
# and edge_params (chord sampling of a curve) move to geom/chord.rs, re-exported by mesh. Same mesh,
# same seeds, same results. Run after tbvh-apply.py. Usage: python3 tmeshq-apply.py <yack-springrep dir>
import os, re, sys

root = sys.argv[1]
src = os.path.join(root, "src")
p = lambda *a: os.path.join(src, *a)


def rd(f):
    return open(p(f)).read()


def wr(f, s):
    open(p(f), "w").write(s)


def sub1(s, a, b, f):
    if s.count(a) != 1:
        raise SystemExit(f"{f}: expected one {a[:80]!r}, found {s.count(a)}")
    return s.replace(a, b)


def item(s, start):
    i = s.index(start)
    j = s.rfind("\n\n", 0, i) + 2
    k = s.index("\n}\n", i) + 3
    return j, k


# ---- 1. MeshOptions + edge_params cluster: mesh/mod.rs -> geom/chord.rs ----
m = rd("mesh/mod.rs")
cut = []
for start in ["#[derive(Clone, Copy, Debug, PartialEq)]\npub struct MeshOptions {", "impl MeshOptions {",
              "pub fn edge_params(", "fn coarsen(", "fn segment_dist(", "fn subdivide("]:
    if m.count(start) != 1:
        raise SystemExit(f"mesh/mod.rs: {start[:50]!r} not found once")
    if start.startswith("#[derive"):
        i = m.index(start)
        j = m.rfind("\n\n", 0, i) + 2
        k = m.index("\n}\n", i) + 3
    else:
        j, k = item(m, start)
    cut.append(m[j:k])
    m = m[:j] + m[k:]
chord = "\n".join(c.rstrip("\n") + "\n" for c in cut)
chord = chord.replace("\nfn coarsen(", "\npub(crate) fn coarsen(").replace("\nfn segment_dist(", "\nfn segment_dist(")
consts = [c for c in ["FRAC_PI_2", "PI", "TAU"] if re.search(r"\b" + c + r"\b", chord)]
wr("geom/chord.rs", "// yack: how finely a curve is sampled for a mesh: the chord limits, and an edge's sample parameters.\n"
   "// Adapted from springrep@a03e8a1 src/mesh/mod.rs (MIT, (c) 2026 Springcraft, Inc.).\n"
   "// Changes: moved from mesh/mod.rs to geom/chord.rs (M2), so rays and distances, below the mesher, can\n"
   "// sample curves alike; mesh re-exports them. Paths only.\n"
   "//! How finely a curve is sampled for a mesh: [`MeshOptions`], the chord limits, and the parameters\n"
   "//! [`edge_params`] samples an edge's curve at.\n\n"
   + (("use std::f64::consts::" + consts[0] if len(consts) == 1 else "use std::f64::consts::{" + ", ".join(consts) + "}") + ";\n\n" if consts else "")
   + "use super::Curve;\nuse crate::math::{Point3, Vec3};\n\n" + chord)
# std consts only the moved chord code used
for c in ["FRAC_PI_2", "PI", "TAU"]:
    if len(re.findall(r"\b" + c + r"\b", m)) == 1:
        m = re.sub(r"(?m)^use std::f64::consts::" + c + r";\n", "", m)
        if re.search(r"\b" + c + r"\b", m):
            raise SystemExit(f"mesh/mod.rs: {c} is left imported but unused")
m = sub1(m, "\nmod predicates;\n", "\nmod predicates;\nmod query;\n", "mesh/mod.rs")
m = sub1(m, "pub(crate) use solid::from_indexed;\n", "pub use crate::geom::{MeshOptions, edge_params};\npub(crate) use solid::from_indexed;\n", "mesh/mod.rs")
m = sub1(m, "pub use solid::solid;\n", "pub use query::{closest, nearest_boundary, nearest_of, nearest_solid, rays};\npub use solid::solid;\n", "mesh/mod.rs")
wr("mesh/mod.rs", m)
g = rd("geom/mod.rs")
g = sub1(g, "mod bspline;\n", "mod bspline;\nmod chord;\n", "geom/mod.rs")
g = sub1(g, "pub use bspline::BSpline;\n", "pub use bspline::BSpline;\npub use chord::{MeshOptions, edge_params};\n", "geom/mod.rs")
wr("geom/mod.rs", g)

# ---- 2. rays: Rays::new(b) -> Rays::from_tessellation(b, positions, triangles, faces, deflection) ----
r = rd("rays.rs")
r = sub1(r, "    mesh::{\n        self, MeshOptions,\n    },\n", "", "rays.rs")
j, k = item(r, "const MESHES: [MeshOptions; 3] = [")
meshes = r[j:k] if False else None
i = r.index("/// The meshes tried, finest first")
k = r.index("];\n", i) + 3
meshes_text = r[i:k]
r = r[:i] + r[k:].lstrip("\n")
old_new = r[r.index("    pub fn new(b: &'a Brep) -> Result<Rays<'a>> {"):r.index("        let tree = Bvh::new(")]
r = r.replace(old_new, "    /// The caster over `b`'s faces, from a tessellation of them within `deflection`: the triangles\n"
                       "    /// `positions` and `triangles` give, `faces` naming each face's run of them.\n"
                       "    pub fn from_tessellation(\n        b: &'a Brep,\n        positions: &[Point3],\n"
                       "        triangles: &[[u32; 3]],\n        faces: &[(FaceId, std::ops::Range<usize>)],\n"
                       "        deflection: f64,\n    ) -> Rays<'a> {\n", 1)
body_end = r.index("            reach,\n        })\n    }\n", r.index("pub fn from_tessellation("))
body = r[r.index("pub fn from_tessellation("):body_end]
nb = (body.replace("Bvh::new(&m.positions, &m.triangles)", "Bvh::new(positions, triangles)")
      .replace("vec![None; m.triangles.len()]", "vec![None; triangles.len()]")
      .replace("for (f, range) in &m.faces {", "for (f, range) in faces {")
      .replace("let tris = m\n            .triangles\n", "let tris = triangles\n")
      .replace("(f, t.map(|i| m.positions[i as usize]))", "(f, t.map(|i| positions[i as usize]))")
      .replace("for &q in &m.positions {", "for &q in positions {")
      .replace("deflection: opt.linear,", "deflection,"))
if re.search(r"\bm\.|\bopt\b", nb):
    raise SystemExit("rays.rs: from_tessellation still names the mesh: " + str(re.findall(r".*\b(?:m\.|opt)\b.*", nb)[:3]))
r = r.replace(body, nb, 1)
r = sub1(r, "            reach,\n        })\n    }\n", "            reach,\n        }\n    }\n", "rays.rs")
r = r.replace("        Ok(Rays {\n            b,\n            tris,", "        Rays {\n            b,\n            tris,", 1)
r = r.replace("// Changes: none (verbatim); this header added.",
              "// Changes: meshes nothing (M2): `Rays::from_tessellation` takes the mesh; mesh::rays makes it.", 1)
wr("rays.rs", r)

# ---- 3. distance: Side::new(b, sel) -> Side::from_tessellation; the entry points go to mesh/query.rs ----
d = rd("distance.rs")
d = sub1(d, "    mesh::{\n        self, MeshOptions,\n    },\n", "    geom::MeshOptions,\n", "distance.rs")
i = d.index("/// The meshes tried, finest first")
k = d.index("];\n", i) + 3
if d[i:k] != meshes_text:
    raise SystemExit("distance.rs: its MESHES differ from rays.rs's; not one table")
d = d[:i] + d[k:].lstrip("\n")
d = sub1(d, "struct Side<'a> {", "pub(crate) struct Side<'a> {", "distance.rs")
s0 = d.index("    fn new(b: &'a Brep, sel: &Selection) -> Result<Side<'a>> {")
mesh_block = d[d.index("        let mut err = None;\n        let mut found = None;\n", s0):d.index("        let tri = |t: &[u32; 3]|", s0)]
d = d.replace(mesh_block, "", 1)
d = d.replace("    fn new(b: &'a Brep, sel: &Selection) -> Result<Side<'a>> {",
              "    /// The side `sel` names, from a tessellation of `b` made within `opt`: the triangles\n"
              "    /// `positions` and `triangles` give, `faces` naming each face's run of them.\n"
              "    pub(crate) fn from_tessellation(\n        b: &'a Brep,\n        sel: &Selection,\n"
              "        positions: &[Point3],\n        triangles: &[[u32; 3]],\n"
              "        faces_of: &[(FaceId, std::ops::Range<usize>)],\n        opt: &MeshOptions,\n    ) -> Side<'a> {", 1)
s0 = d.index("pub(crate) fn from_tessellation(")
e0 = d.index("            deflection: opt.linear,\n        })\n    }\n", s0)
sb = d[s0:e0]
nsb = (sb.replace("let tri = |t: &[u32; 3]| t.map(|i| m.positions[i as usize]);", "let tri = |t: &[u32; 3]| t.map(|i| positions[i as usize]);")
       .replace("for (f, range) in &m.faces {", "for (f, range) in faces_of {")
       .replace("for t in &m.triangles[range.clone()] {", "for t in &triangles[range.clone()] {")
       .replace("let ts = mesh::edge_params(c, t0, t1, &opt);", "let ts = crate::geom::edge_params(c, t0, t1, opt);"))
if re.search(r"\bm\.|mesh::", nsb):
    raise SystemExit("distance.rs: from_tessellation still names the mesh")
d = d.replace(sb, nsb, 1)
d = sub1(d, "            deflection: opt.linear,\n        })\n    }\n", "            deflection: opt.linear,\n        }\n    }\n", "distance.rs")
d = d.replace("        Ok(Side {\n            b,\n            vertices,\n            pieces,", "        Side {\n            b,\n            vertices,\n            pieces,", 1)
# entry points: closest and Nearest's constructors go to mesh/query.rs; Nearest keeps `to`
j, k = item(d, "pub fn closest(a: &Brep, sa: &Selection, b: &Brep, sb: &Selection) -> Result<Closest> {")
closest_doc = d[j:k]
d = d[:j] + d[k:]
ni = d.index("impl<'a> Nearest<'a> {")
nk = d.index("    /// The nearest point to `p`, as `on_b`, and its distance; `on_a` is `p`.", ni)
ctors = d[ni:nk]
d = d[:ni] + "impl<'a> Nearest<'a> {\n" + d[nk:]
d = sub1(d, "pub struct Nearest<'a> {\n    side: Side<'a>,\n}",
         "pub struct Nearest<'a> {\n    side: Side<'a>,\n    /// How a point to measure from is made a side: the mesher's, as the arena's own was.\n"
         "    point: for<'p> fn(&'p Brep, &Selection) -> Result<Side<'p>>,\n}\n\nimpl<'a> Nearest<'a> {\n"
         "    /// The nearest-point search over `side`, measuring from points made sides by `point`.\n"
         "    pub(crate) fn from_side(side: Side<'a>, point: for<'p> fn(&'p Brep, &Selection) -> Result<Side<'p>>) -> Nearest<'a> {\n"
         "        Nearest { side, point }\n    }\n}", "distance.rs")
d = sub1(d, "        between(&Side::new(&point, &Selection::vertex(v))?, &self.side)",
         "        between(&(self.point)(&point, &Selection::vertex(v))?, &self.side)", "distance.rs")
d = sub1(d, "/// [`closest`] between two sides already expanded.\nfn between(", "/// The closest distance between two sides already expanded.\npub(crate) fn between(", "distance.rs")
d = d.replace("// Changes: none (verbatim); this header added.",
              "// Changes: meshes nothing (M2): sides are made from a tessellation mesh::query makes; `closest`\n// and Nearest's constructors moved there.", 1)
wr("distance.rs", d)

# ---- 4. mesh/query.rs ----
ctor_doc = {}
for name in ["boundary", "of", "solid"]:
    mm = re.search(r"((?:    ///.*\n)+)    pub fn " + name + r"\(", ctors)
    ctor_doc[name] = "".join(l[4:] + "\n" for l in mm.group(1).splitlines()) if mm else ""
wr("mesh/query.rs", "// yack: rays and distances over a solid, meshed here and handed to them (M2: they mesh nothing).\n"
   "// Adapted from springrep@a03e8a1 src/rays.rs and src/distance.rs (MIT, (c) 2026 Springcraft, Inc.).\n"
   "// Changes: the meshing `Rays::new`, `distance::closest` and `Nearest`'s constructors did, moved up\n"
   "// here unchanged (one MESHES table, finest first), so rays and distance sit below the mesher.\n"
   "//! A solid's ray caster and nearest-point searches, each over a mesh made here: the finest of\n"
   "//! [`MESHES`] the mesher manages, seeded to the solid's size.\n\n"
   "use super::{Mesh, MeshOptions, seeding, tessellate};\n"
   "use crate::{\n    distance::{self, Closest, Nearest, Selection, Side},\n    error::Result,\n    rays::Rays,\n    topo::Brep,\n};\n\n"
   + meshes_text + "\n"
   "/// The finest of [`MESHES`] the mesher manages for `b`, with the options it was made with.\n"
   "fn mesh_for(b: &Brep) -> Result<(Mesh, MeshOptions)> {\n"
   "    let mut err = None;\n    for opt in MESHES.iter().map(|o| seeding(o, b)) {\n        match tessellate(b, &opt) {\n"
   "            Ok(m) => return Ok((m, opt)),\n            Err(e) => err = Some(e),\n        }\n    }\n"
   "    Err(err.expect(\"a mesh was tried\"))\n}\n\n"
   "/// The ray caster over `b`'s faces.\npub fn rays(b: &Brep) -> Result<Rays<'_>> {\n    let (m, opt) = mesh_for(b)?;\n"
   "    Ok(Rays::from_tessellation(b, &m.positions, &m.triangles, &m.faces, opt.linear))\n}\n\n"
   "/// The side of `b` that `sel` names, over `b`'s mesh.\nfn side<'b>(b: &'b Brep, sel: &Selection) -> Result<Side<'b>> {\n"
   "    let (m, opt) = mesh_for(b)?;\n    Ok(Side::from_tessellation(b, sel, &m.positions, &m.triangles, &m.faces, &opt))\n}\n\n"
   + closest_doc.replace("pub fn closest(a: &Brep, sa: &Selection, b: &Brep, sb: &Selection) -> Result<Closest> {\n    between(&Side::new(a, sa)?, &Side::new(b, sb)?)",
                         "pub fn closest(a: &Brep, sa: &Selection, b: &Brep, sb: &Selection) -> Result<Closest> {\n    distance::between(&side(a, sa)?, &side(b, sb)?)")
   + "\n" + ctor_doc["boundary"]
   + "pub fn nearest_boundary(b: &Brep) -> Result<Nearest<'_>> {\n    let faces = Selection {\n        faces: b.face_ids().collect(),\n"
   "        ..Selection::default()\n    };\n    Ok(Nearest::from_side(side(b, &faces)?, side))\n}\n\n"
   + ctor_doc["of"] + "pub fn nearest_of<'b>(b: &'b Brep, selection: &Selection) -> Result<Nearest<'b>> {\n"
   "    Ok(Nearest::from_side(side(b, selection)?, side))\n}\n\n"
   + ctor_doc["solid"] + "pub fn nearest_solid(b: &Brep) -> Result<Nearest<'_>> {\n"
   "    Ok(Nearest::from_side(side(b, &Selection::body(b))?, side))\n}\n")
if "distance::between(&side(a, sa)?, &side(b, sb)?)" not in rd("mesh/query.rs"):
    raise SystemExit("closest body not as expected")

# ---- 5. callers: paths ----
REWRITES = [
    (r"\brays::Rays::new\(", "crate::mesh::rays("), (r"(?<![\w:])Rays::new\(", "crate::mesh::rays("),
    (r"\bdistance::closest\(", "crate::mesh::closest("),
    (r"\b(?:distance::)?Nearest::boundary\(", "crate::mesh::nearest_boundary("),
    (r"\b(?:distance::)?Nearest::of\(", "crate::mesh::nearest_of("),
    (r"\b(?:distance::)?Nearest::solid\(", "crate::mesh::nearest_solid("),
]
changed = []
for dp, _, fs in os.walk(src):
    for fn in fs:
        f = os.path.join(dp, fn)
        if not fn.endswith(".rs") or f.endswith("mesh/query.rs"):
            continue
        s0 = s = open(f).read()
        for a, b in REWRITES:
            s = re.sub(a, b, s)
        s = s.replace("crate::crate::", "crate::")
        if s != s0:
            open(f, "w").write(s)
            changed.append(os.path.relpath(f, src))
# the crate's own integration tests name these through the crate
TREWRITES = [(a, b.replace("crate::mesh::", "yack_springrep::mesh::")) for a, b in REWRITES]
tdir = os.path.join(root, "tests")
for dp, _, fs in os.walk(tdir):
    for fn in fs:
        f = os.path.join(dp, fn)
        if not fn.endswith(".rs"):
            continue
        s0 = s = open(f).read()
        for a, b in TREWRITES:
            s = re.sub(a, b, s)
        if s != s0:
            open(f, "w").write(s)
            changed.append("tests/" + os.path.relpath(f, tdir))
# ---- 6. the same renames outside the crate (path-only edits; owners agreed) ----
ws = os.path.dirname(os.path.dirname(os.path.abspath(root)))
# outside the crate: the files named after --with-external (found by grep at queue time); in each,
# the same renames, through yack_springrep:: (query.rs already imports `mesh`, `rays` and `distance`)
EXT_SUBS = [(r"\brays::Rays::new\(", "mesh::rays("), (r"\bdistance::Nearest::solid\(", "mesh::nearest_solid("),
            (r"\bdistance::Nearest::of\(", "mesh::nearest_of("), (r"\bdistance::Nearest::boundary\(", "mesh::nearest_boundary("),
            (r"\bdistance::closest\(", "mesh::closest("),
            (r"(?<![\w:])Rays::new\(", "yack_springrep::mesh::rays("),
            (r"(?<![\w:])Nearest::(boundary|of|solid)\(", r"yack_springrep::mesh::nearest_\1(")]
external = []
if len(sys.argv) > 2 and sys.argv[2] == "--with-external":
    for rel in sys.argv[3:]:
        f = os.path.join(ws, rel)
        s0 = s = open(f).read()
        for a, b in EXT_SUBS:
            s = re.sub(a, b, s)
        # `mesh::` alone is only in scope where the file imports yack_springrep::mesh
        if "mesh::" in s and not re.search(r"yack_springrep::\{[^}]*\bmesh\b|use yack_springrep::mesh", s0):
            s = re.sub(r"(?<![\w:])mesh::(rays|closest|nearest_\w+)\(", r"yack_springrep::mesh::\1(", s)
        if s == s0:
            raise SystemExit(f"{rel}: nothing to rename")
        open(f, "w").write(s)
        external.append(f)


# ---- 7. imports the renames left unused (clippy -D warnings) ----
def code_count(s, name):
    return len(re.findall(r"(?<![\w])" + name + r"\b", "\n".join(l for l in s.splitlines() if not l.lstrip().startswith("//"))))


def drop_unused(f):
    s0 = s = open(f).read()
    for name in ["Rays", "Nearest", "closest"]:
        uses = [m.group(0) for m in re.finditer(r"(?ms)^\s*use [^;]*;", s)]
        in_use = sum(len(re.findall(r"(?<![\w])" + name + r"\b", u)) for u in uses)
        if in_use == 0 or code_count(s, name) > in_use:
            continue
        s = re.sub(r"(\w+)::\{" + name + r", (\w+)\}", r"\1::\2", s)
        s = re.sub(r"(\w+)::\{(\w+), " + name + r"\}", r"\1::\2", s)
        s = re.sub(r"(?m)^[ \t]*\w+::" + name + r",\n", "", s)
        s = re.sub(r"use crate::\{\w+::" + name + r", (\w+::\w+)\};", r"use crate::\1;", s)
        if code_count(s, name) != 0:
            raise SystemExit(f"{f}: could not drop the unused import of {name}")
    # a module imported as `self` that nothing names any more
    for mod in ["distance", "rays"]:
        if re.search(mod + r"::\{self, ", s) and len(re.findall(r"(?<![\w])" + mod + r"::", s)) == 1:
            s = s.replace(mod + "::{self, ", mod + "::{", 1)
    if s != s0:
        open(f, "w").write(s)


for rel in changed:
    drop_unused(os.path.join(src, rel) if not rel.startswith("tests/") else os.path.join(tdir, rel[len("tests/"):]))
for f in external:
    drop_unused(f)
print("ok; callers rewritten in", sorted(changed), "and outside:", [os.path.relpath(f, ws) for f in external])
