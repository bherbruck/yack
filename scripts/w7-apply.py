# M3 W7 (lead's rulings): springrep's tweak/ becomes yack-tweak; primitive, sweep, loft and T5's build/
# (raw, assemble, mesh_solid, and heal/adjust for now: yack-heal starts empty, since heal and raw use
# each other and both crates are layer 7) become yack-build, whose root holds them as `primitive`,
# `sweep`, `loft` and `build`, so their `crate::` paths stand. springrep re-exports all five names.
# The moved-test hubs (loft/tests.rs's tail and loft/moved_tests.rs) get a home of their own first,
# springrep's `#[cfg(test)] mod moved_tests;` (src/moved_tests/<origin>.rs), verbatim; tests of the
# moving modules that build above their crate join them there. Usual machinery otherwise.
# Usage: python3 w7-apply.py <yack-springrep dir> [--with-external ...]
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m3_move import (allow_items, cargo, count, counted_panics, crate_names, deps_guard, live_text, path_guard,
                     precise_pub, rd, rs_files, set_row, split_test_file, split_tests, springrep_deps,
                     strip_module_paths, sub1, test_reach_guard, unit_edges, wr)

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)


def oex(msg):
    return f'.unwrap_or_else(|| panic!("{msg}"))'


def rex(msg):
    return f'.unwrap_or_else(|e| panic!("{msg}: {{e:?}}"))'


# ---- 0a. the moved-test hubs get their own home: src/moved_tests/<origin>.rs ----
MT = os.path.join(src, "moved_tests")
hub_files = {}  # origin -> text


def to_hub(origin, text):
    hub_files[origin] = (hub_files.get(origin, "").rstrip("\n") + "\n\n" + text.strip("\n") + "\n").lstrip("\n")


lt_ = os.path.join(src, "loft", "tests.rs")
t = rd(lt_)
k = t.index("/// Moved from intersect/march.rs's tests")
own, moved = t[:k].rstrip("\n") + "\n", t[k:]
ORIGIN = {"intersect_march": "intersect", "measure_moved": "measure", "measure_sample_moved": "measure",
          "measure_tolerant_moved": "measure", "rays_moved": "rays", "codec_moved": "codec"}
blocks = re.split(r"(?m)^(?=/// Moved from )", moved)
for b in blocks:
    if not b.strip():
        continue
    name = re.search(r"(?m)^mod (\w+) \{", b).group(1)
    to_hub(ORIGIN[name], b)
wr(lt_, own)
lm_ = os.path.join(src, "loft", "moved_tests.rs")
t = rd(lm_)
t = t[t.index("\n/// ") + 1:]  # past its header comment
to_hub("mesh", t)
os.remove(lm_)
lmod = os.path.join(src, "loft", "mod.rs")
wr(lmod, sub1(rd(lmod), "#[cfg(test)]\nmod moved_tests;\n", "", lmod))

# ---- 0b. tests of the moving modules that build above their new crate join the hubs ----
ABOVE_TWEAK = {"edit", "sweep", "primitive", "loft", "boolean", "blend", "facade", "build", "step", "extrude",
               "revolve", "block", "cylinder", "sphere", "union", "difference", "fillet", "Section", "loft"}
ABOVE_BUILD = {"boolean", "blend", "edit", "facade", "step", "draw", "overlap", "probe", "union", "difference",
               "intersection", "fillet", "chamfer"}
moved_tests = []
for rel, mp, forbid, origin in [
    ("loft/tests.rs", None, ABOVE_BUILD, "loft"),
    ("sweep/curve.rs", ["crate", "sweep", "curve"], ABOVE_BUILD, "sweep"),
    ("build/mesh_solid.rs", ["crate", "build", "mesh_solid"], ABOVE_BUILD, "build"),
]:
    # a private module's items are reached where its parent re-exports them (mesh_solid: build)
    at = ["crate", "build"] if rel == "build/mesh_solid.rs" else None
    p = os.path.join(src, rel)
    if rel == "loft/tests.rs":
        tt = rd(p)
        if not (set(re.findall(r"\bcrate::(\w+)", tt)) | set(re.findall(r"(?<![\w:.])(\w+)::", tt))
                | set(re.findall(r"(?<![\w:.])(\w+)\(", tt))) & forbid:
            continue  # with the hubs gone, loft's own tests build within yack-build
        mod, tests_, helpers = split_test_file(p, os.path.join(src, "loft", "mod.rs"), ["crate", "loft"], forbid,
                                               "loft_moved", "Moved from loft/tests.rs (M3 W7): they build with "
                                               "modules above yack-build.")
    else:
        if "#[cfg(test)]\nmod tests {\n" not in rd(p):
            continue
        tt = rd(p)[rd(p).index("#[cfg(test)]\nmod tests {\n"):]
        if not (set(re.findall(r"\bcrate::(\w+)", tt)) | set(re.findall(r"(?<![\w:.])(\w+)::", tt))) & forbid:
            continue
        mod, tests_, helpers = split_tests(p, mp, forbid, rel.replace("/", "_")[:-3] + "_moved",
                                           f"Moved from {rel}'s tests (M3 W7): they build above yack-build.",
                                           own_at=at)
    if rel == "build/mesh_solid.rs":  # build re-exports mesh_solid's `solid` as `solid_from_mesh`
        mod = mod.replace("    use crate::build::solid;\n", "    use crate::build::solid_from_mesh as solid;\n")
    to_hub(origin, mod)
    moved_tests.append((rel, tests_, helpers))
def reaches(path, forbid):
    tt = rd(path)
    return (set(re.findall(r"\bcrate::(\w+)", tt)) | set(re.findall(r"(?<![\w:.])(\w+)::", tt))
            | set(re.findall(r"(?<![\w:.])(\w+)\(", tt))) & forbid


# tweak's own tests build their operands with sweep::extrude, beside yack-tweak: they read extrude's
# exact solids from fixtures instead (lead's rulings), each printed once and proven equal to a fresh
# extrude by the removed one-off printers (w-m04-fix-1: cube10; w-m04-fix-2: box20, rod10, tip), so the
# tests of move_planes stay in yack-tweak. Only the extrude line, and profile lines nothing else uses,
# change; each changed body says which fixture.
HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES = {"cube10": rd(os.path.join(HERE, "w7-cube10.hex")).strip()}
for line in rd(os.path.join(HERE, "w7-fixtures.hex")).split("\n"):
    if line.strip():
        tag, hx = line.split()
        FIXTURES[tag] = hx
if sorted(FIXTURES) != ["box20", "cube10", "rod10", "tip"]:
    raise SystemExit(f"fixtures: {sorted(FIXTURES)}")
os.makedirs(os.path.join(crates, "yack-tweak", "tests", "data"), exist_ok=True)
for tag, hx in FIXTURES.items():
    open(os.path.join(crates, "yack-tweak", "tests", "data", tag + ".yack"), "wb").write(bytes.fromhex(hx))
EXTRUDE_CUBE = """fn cube() -> Brep {
    let p = Profile::rect(Vec2::ZERO, 10.0, 10.0);
    sweep::extrude(&p, &Frame::WORLD, 10.0, "box").unwrap()
}"""
FIXTURE_CUBE = """fn cube() -> Brep {
    // cube10 fixture: sweep::extrude's 10 mm cube, proven by w-m04-fix-1.
    fixture("cube10")
}

/// A solid sweep::extrude built, read from its fixture (extrude sits beside yack-tweak, M3 W7): the
/// same solid bit for bit, as the printer that wrote each one proved.
fn fixture(name: &str) -> Brep {
    let bytes: &[u8] = match name {
        "cube10" => include_bytes!("../tests/data/cube10.yack"),
        "box20" => include_bytes!("../tests/data/box20.yack"),
        "rod10" => include_bytes!("../tests/data/rod10.yack"),
        "tip" => include_bytes!("../tests/data/tip.yack"),
        _ => panic!("no fixture {name}"),
    };
    yack_codec::from_bytes(bytes).and_then(|mut v| v.pop()).unwrap()
}"""
tp = os.path.join(src, "tweak", "tests.rs")
t = sub1(rd(tp), EXTRUDE_CUBE, FIXTURE_CUBE, tp)
t = sub1(t, """    let p = Profile::rect(Vec2::ZERO, 20.0, 20.0);
    let b = sweep::extrude(&p, &Frame::WORLD, 10.0, "box").unwrap();
""", """    // box20 fixture: sweep::extrude's 20 x 20 x 10 box, proven by w-m04-fix-2.
    let b = fixture("box20");
""", tp)
t = sub1(t, """    let p = Profile::circle(Vec2::ZERO, 10.0);
    let b = sweep::extrude(&p, &Frame::WORLD, 10.0, "box").unwrap();
""", """    // rod10 fixture: sweep::extrude's rod of radius 10, 10 long, proven by w-m04-fix-2.
    let b = fixture("rod10");
""", tp)
k0 = t.index("fn a_tip_moved_through_the_other_tip_is_refused() {\n") + len("fn a_tip_moved_through_the_other_tip_is_refused() {\n")
k1 = t.index('    let b = sweep::extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, 5.0, "box").unwrap();\n', k0)
k1 += len('    let b = sweep::extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, 5.0, "box").unwrap();\n')
if "pts" in t[k1:t.index("\n}\n", k1)] or "names" in t[k1:t.index("\n}\n", k1)]:
    raise SystemExit("tweak tests: the tip test's pts or names feed something after the extrude")
t = t[:k0] + """    // tip fixture: sweep::extrude's 14-sided prism, 5 tall, proven by w-m04-fix-2.
    let b = fixture("tip");
""" + t[k1:]
wr(tp, t)

if reaches(os.path.join(src, "sweep", "helix", "tests.rs"), ABOVE_BUILD):
  mod, tests_, helpers = split_test_file(os.path.join(src, "sweep", "helix", "tests.rs"),
                                       os.path.join(src, "sweep", "helix.rs"), ["crate", "sweep", "helix"],
                                       ABOVE_BUILD, "sweep_helix_moved",
                                       "Moved from sweep/helix/tests.rs (M3 W7): they build above yack-build.",
                                       own_at=["crate", "sweep"])
  to_hub("sweep", mod)
  moved_tests.append(("sweep/helix/tests.rs", tests_, helpers))
mod, tests_, helpers = split_test_file(os.path.join(src, "tweak", "tests.rs"), os.path.join(src, "tweak", "mod.rs"),
                                       ["crate", "tweak"], ABOVE_TWEAK, dest_name="tweak_moved",
                                       # the `frame` helper read a face's plane through tweak's private
                                       # plane_of; it reads the public Surface::Plane itself (same frame)
                                       transform=lambda t: t if "plane_of(" not in t else sub1(t, "plane_of(&b.face(f).surface).unwrap()",
                                                                "match &b.face(f).surface {\n"
                                                                "        crate::geom::Surface::Plane(p) => p.frame,\n"
                                                                "        _ => panic!(\"a plane face\"),\n"
                                                                "    }", "tweak tests"),
                                       dest_doc="Moved from tweak/tests.rs (M3 W7): they build with edit, sweep and "
                                       "primitive, above yack-tweak.")
to_hub("tweak", mod)
moved_tests.append(("tweak/tests.rs", tests_, helpers))
# the tests that stay in yack-tweak measure volumes: yack-measure is their dev-dependency
tp = os.path.join(src, "tweak", "tests.rs")
t = rd(tp)
if re.search(r"\bProfile\b|\bsweep::|\bprimitive::|\bedit::", re.sub(r"//[^\n]*", "", t.split("use super::*;", 1)[1])):
    raise SystemExit("tweak tests left in yack-tweak still build with profile, sweep, primitive or edit")
wr(tp, sub1(t, "use crate::measure;\n", "use yack_measure as measure;\n", tp))
CRATE_OF = {"codec": "yack-codec", "intersect": "yack-intersect", "measure": "yack-measure", "mesh": "yack-mesh",
            "rays": "yack-locate", "tweak": "yack-tweak", "sweep": "yack-build", "build": "yack-build",
            "loft": "yack-build"}
for origin, text in hub_files.items():
    # gate 10: every file names its purpose, and these hold springrep's tests
    wr(os.path.join(MT, origin + ".rs"),
       f"// yack: {origin}'s tests that build their operands with springrep modules above {CRATE_OF[origin]}.\n"
       f"// Adapted from springrep@a03e8a1 src/{origin} (MIT, (c) 2026 Springcraft, Inc.).\n"
       f"// Changes: moved out of the crate's tests in M3 (W4-W7); paths and imports only.\n" + text)
wr(os.path.join(MT, "mod.rs"),
   "// yack: tests moved out of the crates M3 extracted, which build their operands with springrep's modules\n"
   "// above those crates; they move to crates/yack/tests through the public API when springrep goes.\n"
   + "".join(f"mod {o};\n" for o in sorted(hub_files)))
lp = os.path.join(src, "lib.rs")
l = rd(lp)
l = sub1(l, "pub mod loft;\n", "pub mod loft;\n#[cfg(test)]\nmod moved_tests;\n", lp)
wr(lp, l)

# ---- 1. the crates ----
BUILD_FILES = {"primitive.rs": "primitive.rs", "primitive/slab.rs": "primitive/slab.rs", "sweep.rs": "sweep.rs"}
for sub in ("sweep", "loft", "build"):
    for p in rs_files(os.path.join(src, sub)):
        r = os.path.relpath(p, src)
        BUILD_FILES[r] = r
TWEAK_FILES = {"tweak/mod.rs": "lib.rs"}
for p in rs_files(os.path.join(src, "tweak")):
    r = os.path.relpath(p, src)
    if r != "tweak/mod.rs":
        TWEAK_FILES[r] = os.path.relpath(p, os.path.join(src, "tweak"))
PANICS = {
    "yack-tweak": {"lib.rs": [('.expect("drafted planes are planes")', oex("drafted planes are planes"), 1)],
                   "through.rs": [('mids[i].expect("curved")', "mids[i]" + oex("curved"), 1),
                                  ('.as_ref().expect("curved")', ".as_ref()" + oex("curved"), 1)]},
    "yack-build": {"build/raw.rs": [('.expect("STEP edges have curves")', oex("STEP edges have curves"), 1)],
                   "sweep.rs": [('.expect("curved sweep has edges")', oex("curved sweep has edges"), 2),
                                ('.expect("partial sweep has edges")', oex("partial sweep has edges"), 4)],
                   "sweep/curve.rs": [('.expect("a sweep has stations")', oex("a sweep has stations"), 1)],
                   "sweep/helix.rs": [('.expect("a rail is a valid curve")', rex("a rail is a valid curve"), 1)],
                   "loft/mod.rs": [('.expect("a normalized profile has a loop")', oex("a normalized profile has a loop"), 1),
                                   ('.expect("band rows have edges")', oex("band rows have edges"), 2),
                                   ('.expect("cap rows have edges")', oex("cap rows have edges"), 1)]},
}
DEPS = {"yack-tweak": ["yack-base", "yack-check", "yack-geom", "yack-intersect", "yack-locate", "yack-math",
                       "yack-numeric", "yack-tol", "yack-topo"],
        "yack-build": ["yack-base", "yack-check", "yack-geom", "yack-intersect", "yack-locate", "yack-math",
                       "yack-measure", "yack-numeric", "yack-profile", "yack-tol", "yack-topo"]}
DEV = {"yack-tweak": ["yack-codec", "yack-measure"]}  # the cube fixture's reader; gate 1 checks dev-deps against the row too
ALIASES = {
    "error": "use yack_base::kernel as error;", "Error": "use yack_base::kernel::Error;",
    "Result": "use yack_base::kernel::Result;", "geom": "use yack_geom as geom;",
    "math": "use yack_math::kernel as math;", "numeric": "use yack_numeric as numeric;",
    "tol": "use yack_tol as tol;", "topo": "use yack_topo as topo;", "intersect": "use yack_intersect as intersect;",
    "names": "use yack_topo::names;", "locate": "use yack_locate as locate;", "rays": "use yack_locate::rays;",
    "distance": "use yack_locate::distance;", "measure": "use yack_measure as measure;",
    "check": "use yack_check as check;", "profile": "use yack_profile as profile;", "mesh": "use yack_mesh as mesh;",
}


def header_note(t, s_rel, krate):
    if re.search(r"// Changes: none \(verbatim\); this header added\.", t):
        return re.sub(r"// Changes: none \(verbatim\); this header added\.",
                      f"// Changes: moved from springrep's src/{s_rel} to {krate} (M3 W7); paths and visibility only.", t, count=1)
    if "// Changes:" in t[:600]:
        t = re.sub(r"(// Changes: [^\n]*(?:\n// [^\n]*)*?)\.?\n(?!// )",
                   rf"\1; then to {krate} (M3 W7), paths and visibility only.\n", t, count=1)
        return t.replace(f".; then to {krate} (M3 W7)", f"; then to {krate} (M3 W7)", 1)
    return t


def drop_root_aliases(text):
    def one(u):
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
    return re.sub(r"use crate::\{([^;]*)\};", one, text)


staged = {}
# yack-tweak: tweak/mod.rs is its root
mv = {}
for s_rel, d_rel in TWEAK_FILES.items():
    if not os.path.exists(os.path.join(src, s_rel)):
        continue
    t = header_note(strip_module_paths(rd(os.path.join(src, s_rel)), ["tweak"], s_rel), s_rel, "yack-tweak")
    if d_rel == "lib.rs":
        t = re.sub(r"\bpub\(super\)", "pub(crate)", t)
    if d_rel in PANICS["yack-tweak"]:
        t = counted_panics(t, PANICS["yack-tweak"][d_rel], s_rel)
    mv[d_rel] = t
staged["yack-tweak"] = mv
# yack-build: a fresh root holds the four modules at their springrep names
mv = {}
for s_rel, d_rel in BUILD_FILES.items():
    t = header_note(rd(os.path.join(src, s_rel)), s_rel, "yack-build")
    if d_rel in PANICS["yack-build"]:
        t = counted_panics(t, PANICS["yack-build"][d_rel], s_rel)
    mv[d_rel] = t
mv["lib.rs"] = ("// yack: building solids: primitives, sweeps and lofts, and the raw-to-arena builder with its healing\n"
                "// (M3 W7; heal and adjust stay here until yack-heal takes them, D26).\n"
                "//! Building solids: `primitive`, `sweep` and `loft` (PLAN §3's build), and `build`, the raw-to-arena\n"
                "//! builder the STEP reader, the blends and the affine edit share, with its healing for now.\n\n"
                "pub mod build;\npub mod loft;\npub mod primitive;\npub mod sweep;\n")
staged["yack-build"] = mv
for krate, mv in staged.items():
    mod = "tweak" if krate == "yack-tweak" else "build"
    own = set(re.findall(r"(?m)^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", mv["lib.rs"]))
    reached_all = crate_names(list(mv.values())) - own  # test files too: their names need cfg(test) aliases
    reached = crate_names([live_text(v) for k2, v in mv.items() if not k2.endswith("tests.rs")]) - own
    test_only = reached_all - reached  # named only in cfg(test) code: its alias is cfg(test) too
    lines = mv["lib.rs"].split("\n")
    k = next(i for i, l in enumerate(lines) if l.strip() and not l.startswith("//"))
    head = ["", f"// springrep's {mod} code indexes by position; D26's hardening pass moves it onto checked access,",
            "// as for yack-linalg until M5.", "#![allow(clippy::indexing_slicing)]", "",
            "// springrep's foundation modules (D26), under the names the moved code uses."]
    head += [ALIASES[n] for n in sorted(reached & set(ALIASES))]
    head += ["#[cfg(test)]\n" + ALIASES[n] for n in sorted(test_only & set(ALIASES))]
    mv["lib.rs"] = "\n".join(lines[:k] + head) + "\n" + drop_root_aliases("\n".join(lines[k:]))
    unknown = reached_all - set(ALIASES) - own
    if unknown:
        raise SystemExit(f"{krate} reaches crate-root names {sorted(unknown)} it does not have")

REAL = "/home/brenn/github/yack/crates"
moving = {os.path.join(src, s) for s in list(TWEAK_FILES) + list(BUILD_FILES)}
outside, outside_tests = [], []
for p in rs_files(src) + rs_files(os.path.join(root, "tests")):
    if p in moving:
        continue
    t = rd(p)
    outside_tests.append(t)
    if os.path.basename(p) != "tests.rs" and "/tests/" not in p and "/moved_tests/" not in p:
        outside.append(live_text(t))
for c in ["yack", "yack-bridge", "yack-step", "yack-cadrs"]:
    d = os.path.join(crates, c, "src") if os.path.isdir(os.path.join(crates, c, "src")) else os.path.join(REAL, c, "src")
    outside += [live_text(rd(p)) for p in rs_files(d) if os.path.basename(p) != "tests.rs"]
texts, made_pub = precise_pub(staged, "\n".join(outside))
ALIAS_OF = {"yack-tweak": ["tweak"], "yack-build": ["primitive", "sweep", "loft", "build"]}
hits = test_reach_guard(texts, made_pub, outside_tests, ALIAS_OF)
if hits:
    raise SystemExit(f"springrep tests reach crate-private items of the moved crates: {hits}")
# every path springrep's tests name into the moved crates resolves to a public item there
PATH_ALIASES = {"yack-tweak": ["tweak"],
                "yack-build": ["primitive:primitive", "sweep:sweep", "loft:loft", "build:build"]}
unresolved = path_guard(texts, outside_tests, PATH_ALIASES)
if unresolved:
    raise SystemExit(f"springrep tests name paths the moved crates do not have: {unresolved}")
lt0 = os.path.join(ws, "layers.toml")
s0 = rd(lt0)
m0 = re.search(r'(?ms)^\[crate\.yack-tweak\]\n.*?^deps = \[([^\]]*)\]', s0)
have0 = [x.strip().strip('"') for x in m0.group(1).split(",") if x.strip()]
add0 = [d for d in ("yack-codec", "yack-measure") if d not in have0]  # dev-only (gate 1 checks them too)
if add0:
    s0 = s0[:m0.start(1)] + ", ".join(f'"{d}"' for d in have0[:3] + add0 + have0[3:]) + s0[m0.end(1):]
    wr(lt0, s0)
eps_total, rows = 0, []
for krate, fs in texts.items():
    N = os.path.join(crates, krate, "src")
    for d_rel, t in fs.items():
        if re.search(r"\bHash(Map|Set)\b", live_text(t)):
            t = allow_items(t, r"\bHash(Map|Set)\b", "clippy::disallowed_types",
                            "D26: BTree* in hardening, proven by digests", d_rel)
        wr(os.path.join(N, d_rel), t)
    deps_guard(ws, krate, DEPS[krate] + DEV.get(krate, []))
    c = cargo(krate, DEPS[krate])
    if krate in DEV:
        c = c.replace("\n[lints]", "\n[dev-dependencies]\n" + "".join(f'{d} = {{ path = "../{d}" }}\n' for d in DEV[krate]) + "\n[lints]")
    wr(os.path.join(crates, krate, "Cargo.toml"), c)
    e, pnc = count([N]), count(["--panics", N])
    want = sum(n * b.count("panic!(") for _, b, n in sum(PANICS[krate].values(), []))
    if pnc != want:
        raise SystemExit(f"{krate}: {pnc} contract panics, expected {want}")
    eps_total += e
    rows.append((krate, e, pnc, sum(len(rd(p).split("\n")) - 1 for p in rs_files(N))))
for s_rel in list(TWEAK_FILES) + list(BUILD_FILES):
    p = os.path.join(src, s_rel)
    if os.path.exists(p):
        os.remove(p)
for d in ("tweak", "sweep", "loft", "build", "primitive"):
    dd = os.path.join(src, d)
    if os.path.isdir(dd):
        for a, _, _ in sorted(os.walk(dd), reverse=True):
            os.rmdir(a)

# ---- 2. springrep: re-exports and dependencies ----
l = rd(lp)
for name in ("build", "loft", "primitive", "sweep"):
    l = sub1(l, f"pub mod {name};\n", f"pub use yack_build::{name};\n", lp)
l = sub1(l, "pub(crate) mod tweak;\n", "pub(crate) use yack_tweak as tweak;\n", lp)
wr(lp, l)
springrep_deps(ws, root, ["yack-build", "yack-tweak"])

# ---- 3. layers.toml map, scores ----
lt = os.path.join(ws, "layers.toml")
s_ = rd(lt)
for krate, _, _, n in rows:
    if n > 5000 and f"[modules.{krate}]" not in s_:
        mp = unit_edges(os.path.join(crates, krate, "src"))
        block = f"[modules.{krate}]\n" + "".join(f"{u} = [{', '.join(chr(34) + v + chr(34) for v in vs)}]\n" for u, vs in mp) + "\n"
        s_ = sub1(s_, "[modules.yack-springrep]\n", block + "[modules.yack-springrep]\n", lt)
wr(lt, s_)
sd = os.path.join(ws, "scores")
first = True
for krate, e, pnc, _ in rows:
    # rows for the new crates only (lead: eps/lints hold another worker's uncommitted hunks; springrep's
    # row only falls, so it is left)
    set_row(os.path.join(sd, "eps.txt"), krate, e)
    first = False
    if pnc:
        set_row(os.path.join(sd, "lints.txt"), krate, pnc)
fp = os.path.join(sd, "filesize.txt")
f = rd(fp)
f = re.sub(r"(?m)^crates/yack-springrep/src/tweak/mod\.rs ", "crates/yack-tweak/src/lib.rs ", f)
f = re.sub(r"(?m)^crates/yack-springrep/src/tweak/(\S+) ", r"crates/yack-tweak/src/\1 ", f)
f = re.sub(r"(?m)^crates/yack-springrep/src/((?:primitive|sweep|loft|build)\S*) ", r"crates/yack-build/src/\1 ", f)
wr(fp, f)
# gate 10 before the window: every .rs file this wave writes starts `// yack:`
for d in [MT, os.path.join(crates, "yack-tweak", "src"), os.path.join(crates, "yack-build", "src")]:
    for p in rs_files(d):
        if not rd(p).startswith("// yack:"):
            raise SystemExit(f"{p}: first line is not `// yack: <purpose>` (gate 10)")
for a, t_, h in moved_tests:
    print(f"tests {a} -> moved_tests: {t_} (helpers {h})")
print("hubs:", sorted(hub_files))
print("rows:", [(k, e, p, n) for k, e, p, n in rows])
for k, v in made_pub.items():
    print(f"{k}: pub(crate) -> pub for {len(v)} names: {' '.join(v)}")
