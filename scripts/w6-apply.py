# M3 W6: springrep's check.rs and mesh/ become yack-check and yack-mesh, re-exported by springrep
# under their old names. As in W4/W5: precise pub (rule 4), counted panics, per-item allows, the
# eps/lints rows, and tests that build their operands above the new crate move verbatim into
# springrep's loft/tests.rs (mesh/tests.rs is a file of its own: split the same way). New in W6: a
# guard that stops if a springrep test reaches a crate-private item of the moved crates.
# Usage: python3 w6-apply.py <yack-springrep dir> [--with-external ...]
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m3_move import (allow_items, deps_guard, path_guard, cargo, count, counted_panics, crate_names, live_text, precise_pub, rd,
                     rs_files, set_row, split_test_file, split_tests, springrep_deps, strip_module_paths, sub1,
                     test_reach_guard, wr)

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)


def oex(msg):
    return f'.unwrap_or_else(|| panic!("{msg}"))'


# loft/tests.rs (W4/W5's home for moved tests) is near 1,500 lines: W6's go to a file of their own beside it
HUB = os.path.join(src, "loft", "moved_tests.rs")
lm = os.path.join(src, "loft", "mod.rs")
wr(lm, sub1(rd(lm), "#[cfg(test)]\nmod tests;\n", "#[cfg(test)]\nmod moved_tests;\n#[cfg(test)]\nmod tests;\n", lm))
wr(HUB, "// yack: tests moved out of the crates M3's W6 extracted, which build their operands with springrep's\n"
        "// modules above those crates (M3 W6).\n")
FORBID = {"primitive", "sweep", "loft", "boolean", "blend", "facade", "block", "cylinder", "sphere", "extrude",
          "revolve", "union", "difference", "intersection", "Section", "cone", "torus", "Profile", "fillet"}
moved_tests = []

# ---- 0. tests that build above their new crate move to the hub ----
for rel, mp, at, crate_ in [("check.rs", ["crate", "check"], None, "yack-check")] + [
        # a private module's items are reached where mesh re-exports them; display is `pub mod`
        (f"mesh/{f}", ["crate", "mesh", f[:-3]], None if f == "display.rs" else ["crate", "mesh"], "yack-mesh")
        for f in ["mod.rs", "cdt.rs", "display.rs", "face.rs", "predicates.rs", "query.rs"]]:
    t = rd(os.path.join(src, rel))
    k = t.find("#[cfg(test)]\nmod tests {\n")
    if k < 0:
        continue
    reach = set(re.findall(r"\bcrate::(\w+)", t[k:]))
    for u in re.findall(r"\buse crate::\{([^;]*)\};", t[k:]):
        reach |= set(re.findall(r"(?:^|[{,])\s*(\w+)", u))
    if reach & {"primitive", "sweep", "loft", "boolean", "blend", "facade"}:
        name = rel.replace("/", "_").replace(".rs", "") + "_moved"
        if rel == "mesh/mod.rs":
            mp, at = ["crate", "mesh"], None
        mod, tests_, helpers = split_tests(os.path.join(src, rel), mp, FORBID, name,
                                           f"Moved from {rel}'s tests (M3 W6): they build above {crate_}.", own_at=at)
        wr(HUB, rd(HUB).rstrip("\n") + "\n\n" + mod)
        moved_tests.append((rel, tests_, helpers))
mod, tests_, helpers = split_test_file(os.path.join(src, "mesh", "tests.rs"), os.path.join(src, "mesh", "mod.rs"),
                                       ["crate", "mesh"], FORBID, "mesh_moved",
                                       "Moved from mesh/tests.rs (M3 W6): they mesh solids built with sweep, boolean, "
                                       "loft and blend, above yack-mesh.")
wr(HUB, rd(HUB).rstrip("\n") + "\n\n" + mod)
moved_tests.append(("mesh/tests.rs", tests_, helpers))

# boolean/tests.rs's two-ball contact test counted fans with check's crate-private vertex_fans; it
# asserts through the public contacts() instead (lead's ruling (b), tightened): no radial edges, and
# exactly one contact vertex, at the touching point. contacts' vertices are those vertex_fans counts
# more than one fan at, plus radial edges' ends, so with no radial edges that is the same set. Lost:
# "exactly 2 fans" (known weakening, back when the test can live in yack-check).
bp = os.path.join(src, "boolean", "tests.rs")
wr(bp, sub1(rd(bp), "    assert_eq!(crate::check::vertex_fans(&u).get(&at), Some(&2));\n",
            "    let (edges, vertices) = crate::check::contacts(&u);\n"
            "    assert!(edges.is_empty(), \"{edges:?}\");\n"
            "    assert_eq!(vertices.len(), 1, \"{vertices:?}\");\n"
            "    assert!(vertices.contains(&at));\n", bp))

# ---- 1. the modules ----
mesh_files = {"mesh/mod.rs": "lib.rs"}
for f in sorted(os.listdir(os.path.join(src, "mesh"))):
    if f != "mod.rs":
        mesh_files[f"mesh/{f}"] = f
SPECS = [
    ("check", "yack-check", {"check.rs": "lib.rs"}, {},
     ["yack-base", "yack-geom", "yack-intersect", "yack-locate", "yack-math", "yack-measure", "yack-tol",
      "yack-topo"]),
    ("mesh", "yack-mesh", mesh_files,
     {"cdt.rs": [('.expect("neighbour shares the edge")', oex("neighbour shares the edge"), 3),
                 ('.expect("vertex triangle holds it")', oex("vertex triangle holds it"), 2),
                 ('.expect("fan triangle holds it")', oex("fan triangle holds it"), 1),
                 ('.expect("neighbour has a third corner")', oex("neighbour has a third corner"), 2),
                 ('.expect("walk keeps the crossed edge")', oex("walk keeps the crossed edge"), 1)],
      "face.rs": [('.expect("one snapped point per loop point")', oex("one snapped point per loop point"), 1)],
      "lib.rs": [('.expect("one snapped point per loop point")', oex("one snapped point per loop point"), 1),
                 ('w.join().expect("mesh worker panicked")', 'w.join().unwrap_or_else(|e| panic!("mesh worker panicked: {e:?}"))', 1)],
      "query.rs": [('.expect("a mesh was tried")', oex("a mesh was tried"), 1)]},
     ["yack-base", "yack-geom", "yack-intersect", "yack-locate", "yack-math", "yack-numeric", "yack-tol", "yack-topo"]),
]
ALIASES = {
    "error": "use yack_base::kernel as error;", "Error": "use yack_base::kernel::Error;",
    "Result": "use yack_base::kernel::Result;", "geom": "use yack_geom as geom;",
    "math": "use yack_math::kernel as math;", "numeric": "use yack_numeric as numeric;",
    "tol": "use yack_tol as tol;", "topo": "use yack_topo as topo;", "intersect": "use yack_intersect as intersect;",
    "names": "use yack_topo::names;", "locate": "use yack_locate as locate;", "rays": "use yack_locate::rays;",
    "distance": "use yack_locate::distance;", "measure": "use yack_measure as measure;",
}


def drop_root_aliases(u):
    parts, depth, cur = [], 0, ""
    for c in u.group(1):
        depth += c == "{"
        depth -= c == "}"
        if c == "," and depth == 0:
            parts.append(cur)
            cur = ""
        else:
            cur += c
    parts.append(cur)
    keep = [p for p in parts if p.strip() and p.strip() not in ALIASES]
    return ("use crate::{" + ",".join(keep) + (",\n" if u.group(1).rstrip().endswith(",") else "") + "};") if keep else ""


staged = []
for mod, krate, files, panics, deps in SPECS:
    moved = {}
    for s_rel, d_rel in files.items():
        if not os.path.exists(os.path.join(src, s_rel)):
            continue  # a tests file every test left
        t = strip_module_paths(rd(os.path.join(src, s_rel)), [mod], s_rel)
        if d_rel == "lib.rs":
            # at the crate root `super` is gone: what was visible to springrep is crate-wide (precise
            # pub below then makes it pub where springrep still names it)
            t = re.sub(r"\bpub\(super\)", "pub(crate)", t)
        if re.search(r"// Changes: none \(verbatim\); this header added\.", t):
            t = re.sub(r"// Changes: none \(verbatim\); this header added\.",
                       f"// Changes: moved from springrep's src/{s_rel} to {krate} (M3 W6); paths and visibility only.", t, count=1)
        elif "// Changes:" in t[:600]:
            t = re.sub(r"(// Changes: [^\n]*(?:\n// [^\n]*)*?)\.?\n(?!// )",
                       rf"\1; then to {krate} (M3 W6), paths and visibility only.\n", t, count=1)
            t = t.replace(f".; then to {krate} (M3 W6)", f"; then to {krate} (M3 W6)", 1)
        if d_rel in panics:
            t = counted_panics(t, panics[d_rel], s_rel)
        moved[d_rel] = t
    own = set(re.findall(r"(?m)^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", moved["lib.rs"]))
    reached = crate_names([v for k2, v in moved.items() if k2 != "tests.rs"]) - own
    lines = moved["lib.rs"].split("\n")
    k = next(i for i, l in enumerate(lines) if l.strip() and not l.startswith("//"))
    head = ["", f"// springrep's {mod} indexes by position; D26's hardening pass moves it onto checked access,",
            "// as for yack-linalg until M5.", "#![allow(clippy::indexing_slicing)]", "",
            f"// springrep's foundation modules (D26), under the names {mod}'s code uses."]
    head += [ALIASES[n] for n in sorted(reached & set(ALIASES))]
    rest = re.sub(r"use crate::\{([^;]*)\};", drop_root_aliases, "\n".join(lines[k:]))
    moved["lib.rs"] = "\n".join(lines[:k] + head) + "\n" + rest
    staged.append((mod, krate, files, panics, deps, moved))

REAL = "/home/brenn/github/yack/crates"
moving = {os.path.join(src, s) for _, _, files, _, _, _ in staged for s in files}
outside, outside_tests = [], []
for p in rs_files(src) + rs_files(os.path.join(root, "tests")):
    if p in moving:
        continue
    t = rd(p)
    outside_tests.append(t)
    if os.path.basename(p) != "tests.rs" and "/tests/" not in p:
        outside.append(live_text(t))
for c in ["yack", "yack-bridge", "yack-step", "yack-cadrs", "yack-topo", "yack-intersect", "yack-profile",
          "yack-locate", "yack-measure", "yack-codec"]:
    d = os.path.join(crates, c, "src") if os.path.isdir(os.path.join(crates, c, "src")) else os.path.join(REAL, c, "src")
    outside += [live_text(rd(p)) for p in rs_files(d) if os.path.basename(p) != "tests.rs"]
texts, made_pub = precise_pub({krate: moved for _, krate, _, _, _, moved in staged}, "\n".join(outside))
hits = test_reach_guard(texts, made_pub, outside_tests, {"yack-check": ["check"], "yack-mesh": ["mesh"]})
if hits:
    raise SystemExit(f"springrep tests reach crate-private items of the moved crates: {hits}")
unresolved = path_guard(texts, outside_tests, {"yack-check": ["check"], "yack-mesh": ["mesh"]})
if unresolved:
    raise SystemExit(f"springrep tests name paths the moved crates do not have: {unresolved}")
eps_total, rows = 0, []
for mod, krate, files, panics, deps, _ in staged:
    N = os.path.join(crates, krate, "src")
    for d_rel, t in texts[krate].items():
        if d_rel in ("lib.rs", "display.rs", "face.rs") and re.search(r"\bHash(Map|Set)\b", live_text(t)):
            t = allow_items(t, r"\bHash(Map|Set)\b", "clippy::disallowed_types",
                            "D26: BTree* in hardening, proven by digests", d_rel)
        wr(os.path.join(N, d_rel), t)
    for s_rel in files:
        if os.path.exists(os.path.join(src, s_rel)):
            os.remove(os.path.join(src, s_rel))
    d = os.path.join(src, mod)
    if os.path.isdir(d):
        for a, _, _ in sorted(os.walk(d), reverse=True):
            os.rmdir(a)
    wr(os.path.join(crates, krate, "Cargo.toml"), cargo(krate, deps))
    e, pnc = count([N]), count(["--panics", N])
    want = sum(n * b.count("panic!(") for _, b, n in sum(panics.values(), []))
    if pnc != want:
        raise SystemExit(f"{krate}: {pnc} contract panics, expected {want}")
    eps_total += e
    rows.append((krate, e, pnc, sum(len(rd(p).split("\n")) - 1 for p in rs_files(N))))

# ---- 2. springrep: re-exports and dependencies ----
lp = os.path.join(src, "lib.rs")
l = rd(lp)
l = sub1(l, "pub mod check;\n", "pub use yack_check as check;\n", lp)
l = sub1(l, "pub mod mesh;\n", "pub use yack_mesh as mesh;\n", lp)
wr(lp, l)
springrep_deps(ws, root, ["yack-check", "yack-mesh"])

# yack-mesh's display.rs names intersect (ssi::same_domain): its row gains yack-intersect (layer 4)
lt = os.path.join(ws, "layers.toml")
s_ = rd(lt)
mm = re.search(r'(?ms)^\[crate\.yack-mesh\]\n.*?^deps = \[([^\]]*)\]', s_)
have = [x.strip().strip('"') for x in mm.group(1).split(",") if x.strip()]
if "yack-intersect" not in have:
    s_ = s_[:mm.start(1)] + ", ".join(f'"{d}"' for d in have[:1] + ["yack-intersect"] + have[1:]) + s_[mm.end(1):]
wr(lt, s_)

# gate 1 before the window: every new crate's Cargo deps in its row (after mesh's row gained intersect)
for _, krate, _, _, deps, _ in staged:
    deps_guard(ws, krate, deps)

# ---- 3. scores ----
sd = os.path.join(ws, "scores")
first = True
for krate, e, pnc, n in rows:
    if n > 5000:
        raise SystemExit(f"{krate} is over 5k lines: it needs a module map")
    set_row(os.path.join(sd, "eps.txt"), krate, e, ("yack-springrep", eps_total) if first else None)
    first = False
    if pnc:
        set_row(os.path.join(sd, "lints.txt"), krate, pnc)
fp = os.path.join(sd, "filesize.txt")
f = rd(fp)
f = re.sub(r"(?m)^crates/yack-springrep/src/check\.rs ", "crates/yack-check/src/lib.rs ", f)
f = re.sub(r"(?m)^crates/yack-springrep/src/mesh/(\S+) ", r"crates/yack-mesh/src/\1 ", f)
wr(fp, f)
for a, t, h in moved_tests:
    print(f"tests {a} -> loft/moved_tests.rs: {t} (helpers {h})")
print("rows:", [(k, e, p) for k, e, p, _ in rows])
for k, v in made_pub.items():
    print(f"{k}: pub(crate) -> pub for {len(v)} names: {' '.join(v)}")
