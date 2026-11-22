# M2 T5 (lead's ruling: src/build/, mapped to yack-build): the raw-to-arena builder the STEP reader and
# the blends share leaves step/ for a new src/build/: step/{heal,adjust,assemble}.rs whole, and from
# step/topology.rs the raw types and everything rebuilt/solid_of/parts_of/build need (its call graph,
# mapped by scratchpad t5/graph.py); and mesh/solid.rs as build/mesh_solid.rs. step keeps the file
# side and names build. Paths and visibility only. Usage: python3 t5-apply.py <yack-springrep dir>
import os, re, sys

root = sys.argv[1]
src = os.path.join(root, "src")
p = lambda *a: os.path.join(src, *a)
ws = os.path.dirname(os.path.dirname(os.path.abspath(root)))


def rd(f):
    return open(p(f)).read()


def wr(f, s):
    os.makedirs(os.path.dirname(p(f)), exist_ok=True)
    open(p(f), "w").write(s)


def sub1(s, a, b, f):
    if s.count(a) != 1:
        raise SystemExit(f"{f}: expected one {a[:80]!r}, found {s.count(a)}")
    return s.replace(a, b)


def edit(f, pairs):
    s = rd(f)
    for a, b in pairs:
        s = sub1(s, a, b, f)
    wr(f, s)


def header(s, old, new, why):
    s = re.sub(r"// Changes: none \(verbatim\); this header added\.",
               f"// Changes: moved from {old} to {new} ({why}); paths and visibility only.", s, count=1)
    return s


def crate_vis(s):
    return re.sub(r"\bpub\(super\)", "pub(crate)", s)


def prune(s, f):
    """Drops names from the file's `use crate::{...};` block that its code (comments aside) no longer
    names, so a move leaves no unused import behind."""
    m = re.search(r"(?ms)^use crate::\{\n(.*?)^\};\n", s)
    if not m:
        raise SystemExit(f"{f}: no use crate block")
    code = "\n".join(l.split("//")[0] for l in (s[:m.start()] + s[m.end():]).split("\n") if not l.lstrip().startswith("use "))
    used = lambda n: re.search(r"(?<![\w:])" + re.escape(n) + r"\b", code)
    out = []
    for e in re.findall(r"\s*([\w]+(?:::\{[^}]*\})?|[\w:]+),", m.group(1)):
        if "::{" in e:
            head, names = e.split("::{")
            keep = [n.strip() for n in names.rstrip("}").split(",") if n.strip() and used(n.strip())]
            if len(keep) == 1:
                out.append(f"{head}::{keep[0]}")
            elif keep:
                out.append(f"{head}::{{{', '.join(keep)}}}")
        elif used(e.split("::")[-1]):
            out.append(e)
    bare = [e for e in out if "::" not in e and e[0].isupper()]
    rest_ = [e for e in out if e not in bare]
    lines = ([", ".join(bare) + ","] if bare else []) + [e + "," for e in rest_]
    return s[:m.start()] + "use crate::{\n" + "".join(f"    {l}\n" for l in lines) + "};\n" + s[m.end():]


# ---- 1. whole files: step/{heal,adjust,assemble}.rs -> build/ ----
for f in ["heal", "adjust", "assemble"]:
    s = header(rd(f"step/{f}.rs"), f"step/{f}.rs", f"build/{f}.rs", "M2 T5")
    s = s.replace("use super::topology::", "use super::raw::")
    wr(f"build/{f}.rs", crate_vis(s))
    os.remove(p(f"step/{f}.rs"))

# ---- 2. step/topology.rs: the builder's items -> build/raw.rs ----
MOVE = ["Use", "RawEdge", "RawFace", "Raw", "param_of", "edge_range", "Track", "Builder", "impl Builder",
        "coincides_with_twin", "pole_free", "closes", "gap_is_pole", "area2", "shift_into", "uv_box",
        "build", "rebuilt", "Parts", "PartFace", "parts_of", "solid_of"]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
HEAD = re.compile(r"^(?:pub(?:\([^)]*\))?\s+)?(fn|struct|enum|const|static|type|impl|trait|mod|use|macro_rules!)\b\s*(?:<[^>]*>\s*)?([A-Za-z_][A-Za-z0-9_]*)?")
t = rd("step/topology.rs").split("\n")
starts = [k for k, l in enumerate(t) if HEAD.match(l) and not l.startswith(" ")]
spans = []
for idx, k in enumerate(starts):
    m = HEAD.match(t[k]); kind, name = m.group(1), m.group(2)
    if kind == "impl":
        mm = re.match(r"^impl(?:<[^>]*>)?\s+(?:[\w:]+\s+for\s+)?([A-Za-z_]\w*)", t[k]); name = "impl " + mm.group(1)
    j = k
    while j > 0 and (t[j - 1].startswith("///") or t[j - 1].startswith("#[")):
        j -= 1
    e = starts[idx + 1] if idx + 1 < len(starts) else len(t)
    while e > k and (t[e - 1].startswith("///") or t[e - 1].startswith("#[") or t[e - 1].strip() == ""):
        e -= 1
    spans.append((name, j, e))
found = {n for n, _, _ in spans}
missing = [n for n in MOVE if n not in found]
if missing:
    raise SystemExit(f"step/topology.rs: items not found: {missing}")
moved_text, keep = [], [True] * len(t)
for name, j, e in spans:
    if name in MOVE:
        moved_text.append("\n".join(t[j:e]))
        for q in range(j, e):
            keep[q] = False
rest = "\n".join(l for q, l in enumerate(t) if keep[q])
rest = re.sub(r"\n{3,}", "\n\n", rest)
body = crate_vis("\n\n".join(moved_text))
# what the moved text names
std = [n for n in ["HashMap"] if re.search(r"\b" + n + r"\b", body)]
geom = [n for n in ["Circle", "Curve", "Line", "Surface"] if re.search(r"\b" + n + r"\b", body)]
math = [n for n in ["Frame", "Point3", "Vec2"] if re.search(r"\b" + n + r"\b", body)]
topo = [n for n in ["Brep", "CoedgeSpec", "EdgeId", "FaceId", "PCurve", "VertexId", "project_on"] if re.search(r"\b" + n + r"\b", body)]
err = [n for n in ["Error", "Result"] if re.search(r"\b" + n + r"\b", body)]
if re.search(r"\b(Geometry|Param|Reader|Bound)\b", body):
    raise SystemExit("build/raw.rs would name step's file side: " + str(re.findall(r"\b(Geometry|Param|Reader|Bound)\b", body)))
locals_ = [n for n in ["bad", "unsupported", "about", "use_start", "join_contacts", "without_split", "closes_at",
                       "tessellated", "strips_and_fans", "solid", "sew"] if re.search(r"(?<![\w.])" + n + r"\s*\(", body)]
if locals_:
    raise SystemExit(f"build/raw.rs would call items that stay in step: {locals_}")
uses = ""
if std:
    uses += "use std::collections::{" + ", ".join(std) + "};\n\n"
if re.search(r"(?<![\w.])assemble\s*\(", body):
    uses += "use super::assemble::assemble;\n"
if re.search(r"\bheal::", body):
    uses += "use super::heal;\n"
uses += "use crate::{\n" + (f"    {', '.join(err)},\n" if err else "") + (f"    geom::{{{', '.join(geom)}}},\n" if geom else "") \
    + (f"    math::{{{', '.join(math)}}},\n" if math else "") + ("    tol,\n" if re.search(r"\btol::", body) else "") \
    + (f"    topo::{{{', '.join(topo)}}},\n" if topo else "") + "};\n"
wr("build/raw.rs", "// yack: a solid's raw topology (points, edges, faces and their uses) built into an arena.\n"
   "// Adapted from springrep@a03e8a1 src/step/topology.rs (MIT, (c) 2026 Springcraft, Inc.).\n"
   "// Changes: moved from step/topology.rs to build/raw.rs (M2 T5): the builder the STEP reader, the\n"
   "// blends and edit::affine share; paths and visibility only.\n"
   "//! A solid's raw topology, as a file or a rebuild gives it, built into an arena: its points, its\n"
   "//! edges and their uses, its faces and their loops, then each edge's range and pcurves.\n\n"
   + uses + "\n" + body.strip("\n") + "\n")
wr("build/raw.rs", prune(rd("build/raw.rs"), "build/raw.rs"))

# step/topology.rs keeps the file side and names the builder in build
need = [n for n in ["Use", "RawEdge", "RawFace", "Raw", "param_of", "edge_range", "build"]
        if re.search(r"(?<![\w.])" + n + (r"\s*\(" if n[0].islower() else r"\b"), "\n".join(l for l in rest.split("\n") if not l.lstrip().startswith("//") and not l.startswith("use ")))]
rest = header(rest, "", "", "")  # no-op unless verbatim
rest = rest.replace("// Changes: moved from  to  (); paths and visibility only.", "// Changes: none (verbatim); this header added.")
rest = re.sub(r"// Changes: none \(verbatim\); this header added\.",
              "// Changes: the raw builder moved to build/raw.rs (M2 T5); the file side stays here.", rest, count=1)
rest = sub1(rest, "use super::assemble::assemble;\n", "", "step/topology.rs")
rest = sub1(rest, "use super::heal;\n", "", "step/topology.rs")
rest = sub1(rest, "use crate::{\n    Error, Result,\n", "use crate::{\n    Error, Result,\n    build::{\n        Raw, RawEdge, RawFace, Use, assemble, edge_range, heal, param_of,\n    },\n", "step/topology.rs")
if sorted(need) != sorted(["Raw", "RawEdge", "RawFace", "Use", "edge_range", "param_of"]):
    raise SystemExit(f"step/topology.rs needs {need}, not the six the import names")
rest = sub1(rest, "[`super::heal`]", "[`crate::build::heal`]", "step/topology.rs")
rest = rest.replace("crate::mesh::from_indexed(", "crate::build::from_indexed(")
wr("step/topology.rs", prune(rest, "step/topology.rs"))

# ---- 3. mesh/solid.rs -> build/mesh_solid.rs ----
ms = header(rd("mesh/solid.rs"), "mesh/solid.rs", "build/mesh_solid.rs", "M2 T5; mesh no longer calls the checker")
wr("build/mesh_solid.rs", ms)
os.remove(p("mesh/solid.rs"))
edit("mesh/mod.rs", [("mod solid;\n", ""), ("pub(crate) use solid::from_indexed;\n", ""), ("pub use solid::solid;\n", "")])

# ---- 4. build/mod.rs ----
wr("build/mod.rs", "// yack: building a solid's arena from raw topology: the assembly, healing and checks a reader or a rebuild ends in.\n"
   "//! Building a solid's arena from its raw topology (`raw`), healing it onto its surfaces (`heal`,\n"
   "//! `adjust`) and checking it (`assemble`), and a solid from a closed triangle mesh (`mesh_solid`). Moved\n"
   "//! out of `step/` and `mesh/` in M2 (T5) so the blends and edit::affine reach them below step, and the\n"
   "//! mesher no longer calls the checker.\n\n"
   "mod adjust;\nmod assemble;\npub(crate) mod heal;\nmod mesh_solid;\nmod raw;\n\n"
   "pub(crate) use assemble::assemble;\n"
   "pub use heal::{Healed, MAX_HEAL};\n"
   "pub(crate) use mesh_solid::from_indexed;\n"
   "pub use mesh_solid::solid as solid_from_mesh;\n"
   "pub(crate) use raw::{\n    PartFace, Parts, Raw, RawEdge, RawFace, Use, edge_range, param_of, parts_of, rebuilt, solid_of,\n};\n")

# ---- 5. step/mod.rs, step/read.rs ----
edit("step/mod.rs", [("mod adjust;\nmod assemble;\n", ""), ("mod heal;\n", ""),
                     ("pub use heal::{Healed, MAX_HEAL};", "pub use crate::build::{Healed, MAX_HEAL};"),
                     ("pub(crate) use topology::{PartFace, Parts, parts_of, rebuilt, solid_of};\n", "")])
edit("step/read.rs", [("    heal::Healed,\n", "")])
s = rd("step/read.rs")
s = sub1(s, "use crate::{\n    Error, Result, check,", "use crate::{\n    Error, Result,\n    build::Healed,\n    check,", "step/read.rs")
wr("step/read.rs", s)

# ---- 6. callers ----
for f in ["blend/shaped/uneven.rs", "blend/shaped/corner.rs"]:
    edit(f, [("    step::{PartFace, Parts, parts_of, solid_of},\n", "    build::{PartFace, Parts, parts_of, solid_of},\n")])
edit("edit/affine.rs", [("    step,\n", "    build,\n"), ("step::rebuilt(", "build::rebuilt(")])
edit("lib.rs", [("pub mod boolean;\n", "pub mod boolean;\npub mod build;\n")])
tr = os.path.join(root, "tests", "transforms.rs")
s = open(tr).read()
s = sub1(s, "mesh::solid(", "build::solid_from_mesh(", tr)
if not re.search(r"(?<![\w:])mesh::", s):
    s = sub1(s, "    measure, mesh, primitive,", "    build, measure, primitive,", tr)
open(tr, "w").write(s)

# ---- 7. the module map (lead-authorised for T5) ----
lt = os.path.join(ws, "layers.toml")
s = open(lt).read()
step_row = re.search(r'(?m)^step = \[([^\]]*)\]', s)
if not step_row:
    raise SystemExit("layers.toml: no step module row")
if '"build"' not in step_row.group(1):
    s = s.replace(step_row.group(0), step_row.group(0)[:-1] + ', "build"]', 1)
if not re.search(r'(?m)^build = \[', s):
    s = s.replace(step_row.group(0) if '"build"' in step_row.group(1) else step_row.group(0)[:-1] + ', "build"]',
                  'build = ["error", "tol", "numeric", "math", "geom", "topo", "check"]\n'
                  + (step_row.group(0) if '"build"' in step_row.group(1) else step_row.group(0)[:-1] + ', "build"]'), 1)
open(lt, "w").write(s)
print("ok; moved", len(moved_text), "items to build/raw.rs; step/topology.rs imports", need)
