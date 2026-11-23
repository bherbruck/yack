# M2 (lead's ruling: the BVH is generic geometry): mesh/bvh.rs moves to math/bvh.rs. Its one tie to the
# mesh, `Bvh::new(&Mesh)`, becomes `Bvh::new(positions, triangles)`; callers pass the mesh's two arrays.
# Same boxes, same order, same tree. Usage: python3 tbvh-apply.py <yack-springrep dir>
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


s = rd("mesh/bvh.rs")
s = s.replace("// Changes: none (verbatim); this header added.",
              "// Changes: moved from mesh/bvh.rs to math/bvh.rs (M2); `Bvh::new` takes the triangles' positions and\n// indices rather than a mesh.", 1)
for a, b in [
    ("\nuse super::Mesh;\n", "\n"),
    ("    pub(crate) fn new(mesh: &Mesh) -> Bvh {\n        let boxes: Vec<Aabb> = mesh\n            .triangles\n",
     "    pub(crate) fn new(positions: &[Point3], triangles: &[[u32; 3]]) -> Bvh {\n        let boxes: Vec<Aabb> = triangles\n"),
    ("                    b.add(mesh.positions[i as usize]);", "                    b.add(positions[i as usize]);"),
]:
    if s.count(a) != 1:
        raise SystemExit(f"bvh.rs: expected one {a[:60]!r}")
    s = s.replace(a, b)
os.makedirs(p("math"), exist_ok=True)
wr("math/bvh.rs", s)
os.remove(p("mesh/bvh.rs"))
edit("mesh/mod.rs", [("pub(crate) mod bvh;\n", "")])
m = rd("math.rs")
m = m.replace("use std::ops::{", "pub(crate) mod bvh;\n\nuse std::ops::{", 1)
wr("math.rs", m)

# callers: a `bvh::{..}` line inside a `mesh::{..}` group becomes `math::bvh::{..}` in the same use
for f in ["overlap.rs", "distance.rs", "probe.rs", "rays.rs", "boolean/near.rs"]:
    s = rd(f)
    mm = re.search(r"\n    mesh::\{\n(?:        .*\n)*?        (bvh::\{[^}]*\}),\n", s)
    if not mm:
        raise SystemExit(f"{f}: no bvh line in a mesh group")
    line = "        " + mm.group(1) + ",\n"
    s = s.replace(line, "", 1)
    s = s.replace("\n    mesh::{\n", "\n    math::" + mm.group(1) + ",\n    mesh::{\n", 1)
    wr(f, s)
edit("boolean/mod.rs", [("    mesh::bvh::Bvh,\n", "    math::bvh::Bvh,\n")])
for dp, _, fs in os.walk(src):
    for fn in fs:
        if fn.endswith(".rs"):
            path = os.path.join(dp, fn)
            s0 = s = open(path).read()
            s = s.replace("crate::mesh::bvh::", "crate::math::bvh::")
            s = re.sub(r"Bvh::new\(&(\w+)\)", r"Bvh::new(&\1.positions, &\1.triangles)", s)
            if s != s0:
                open(path, "w").write(s)
print("ok")
