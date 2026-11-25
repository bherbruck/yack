# M3 W8 (lead's rulings): springrep's boolean/ becomes yack-boolean, whose root is boolean/mod.rs; plane.rs
# goes with it (its extrude is yack-build's since W7). springrep re-exports it as `boolean`. boolean's
# tests stay with it: the three solids they build above yack-boolean (a fillet, a chamfer, an affine
# stretch) are read from codec fixtures, each proven equal to a fresh build by the one-off printer
# (crates/yack-springrep/tests/w8_fixtures_print.rs, request w-h4-9), which this script deletes. Only
# the build lines change, each saying which fixture. Usual machinery otherwise (W7's shape).
# Usage: python3 w8-apply.py <yack-springrep dir> [--with-external ...]
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m3_move import (allow_items, cargo, count, counted_panics, crate_names, deps_guard, live_text, path_guard,
                     precise_pub, rd, rs_files, set_row, springrep_deps, strip_module_paths, sub1,
                     test_reach_guard, unit_edges, wr)

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)
HERE = os.path.dirname(os.path.abspath(__file__))
KRATE = "yack-boolean"
N = os.path.join(crates, KRATE, "src")
B = os.path.join(src, "boolean")
PROOF = "w-h4-9"


def oex(msg):
    return f'.unwrap_or_else(|| panic!("{msg}"))'


# ---- 0. the fixtures, and the tests' build lines that read them ----
FIXTURES = {}
for line in rd(os.path.join(HERE, "w8-fixtures.hex")).split("\n"):
    if line.strip():
        tag, hx = line.split()
        FIXTURES[tag] = hx
if sorted(FIXTURES) != ["affine-ball", "key-chamfered", "rounded"]:
    raise SystemExit(f"fixtures: {sorted(FIXTURES)}")
os.makedirs(os.path.join(crates, KRATE, "tests", "data"), exist_ok=True)
for tag, hx in FIXTURES.items():
    open(os.path.join(crates, KRATE, "tests", "data", tag + ".yack"), "wb").write(bytes.fromhex(hx))
printer = os.path.join(root, "tests", "w8_fixtures_print.rs")
if os.path.exists(printer):
    os.remove(printer)

tp = os.path.join(B, "tests.rs")
t = rd(tp)
FIXTURE_FN = f"""
/// A solid built above yack-boolean (with blend or edit), read from its fixture: the same solid bit for
/// bit, as the printer that wrote each one proved ({PROOF}, M3 W8).
fn fixture(name: &str) -> Brep {{
    let bytes: &[u8] = match name {{
        "rounded" => include_bytes!("../tests/data/rounded.yack"),
        "key-chamfered" => include_bytes!("../tests/data/key-chamfered.yack"),
        "affine-ball" => include_bytes!("../tests/data/affine-ball.yack"),
        _ => panic!("no fixture {{name}}"),
    }};
    yack_codec::from_bytes(bytes).and_then(|mut v| v.pop()).unwrap()
}}
"""
t = sub1(t, "\nfn block(lo: [f64; 3], hi: [f64; 3]) -> Brep {\n", FIXTURE_FN + "\nfn block(lo: [f64; 3], hi: [f64; 3]) -> Brep {\n", tp)
t = sub1(t, """    let r = rod([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 20.0, 20.0);
    let rounded = crate::blend::fillet(&r, &[("h.end", "h.side")], 3.0, "f").unwrap();
""", f"""    // rounded fixture: blend::fillet's rod of radius 20, 20 long, rounded 3 at its top, proven by {PROOF}.
    let rounded = fixture("rounded");
""", tp)
t = sub1(t, """    let rounded = {
        let r = rod([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 20.0, 20.0);
        crate::blend::fillet(&r, &[("h.end", "h.side")], 3.0, "f").unwrap()
    };
""", f"""    // rounded fixture: blend::fillet's rod of radius 20, 20 long, rounded 3 at its top, proven by {PROOF}.
    let rounded = fixture("rounded");
""", tp)
t = sub1(t, """    let end = format!("{id}.end");
    let sides = ["v_min", "u_max", "v_max", "u_min"].map(|s| format!("{id}.{s}"));
    let pairs: Vec<(&str, &str)> = sides.iter().map(|s| (end.as_str(), s.as_str())).collect();
    crate::blend::chamfer(&key, &pairs, chamfer, "ch").unwrap()
""", f"""    // key-chamfered fixture: blend::chamfer's key "key" turned 30°, z -3 to 3, its top chamfered 0.5,
    // proven by {PROOF}: the one chamfered key the tests ask for.
    assert!(id == "key" && turn == 30f64.to_radians() && (z0, z1) == (-3.0, 3.0) && chamfer == 0.5);
    fixture("key-chamfered")
""", tp)
if re.search(r"\bblend::|\bedit::|\bfacade::|\bstep::|\boverlap::", re.sub(r"//[^\n]*", "", t)):
    raise SystemExit("boolean tests still build above yack-boolean")
wr(tp, t)
mp = os.path.join(B, "tests", "more.rs")
t = rd(mp)
t = sub1(t, """    use crate::{
        edit::{Affine, affine},
        primitive,
        profile::PointsInterp,
    };
""", """    use crate::{primitive, profile::PointsInterp};
""", mp)
t = sub1(t, """    let ball = primitive::sphere(&tilted, 2.0, "s").unwrap();
    let map = Affine {
        m: [[1.1, -0.9, 0.5], [-1.0, 0.6, 0.5], [1.3, 0.6, 0.4]],
        t: Vec3::new(2.0, -3.0, 1.0),
    };
""", """    let ball = primitive::sphere(&tilted, 2.0, "s").unwrap();
""", mp)
t = sub1(t, """        affine(&ball, &map).unwrap(),
""", f"""        // affine-ball fixture: edit::affine's stretch of the ball above, proven by {PROOF}.
        fixture("affine-ball"),
""", mp)
if re.search(r"\bedit::|\bAffine\b|\baffine\(", re.sub(r"//[^\n]*", "", t)):
    raise SystemExit("boolean/tests/more.rs still builds with edit")
wr(mp, t)

# ---- 1. the crate ----
FILES = {"boolean/mod.rs": "lib.rs"}
for p in rs_files(B):
    r = os.path.relpath(p, src)
    if r != "boolean/mod.rs":
        FILES[r] = os.path.relpath(p, B)
PANICS = {
    "lib.rs": [('.expect("pierced edges have curves")', oex("pierced edges have curves"), 1),
               ('.expect("cuts have curves")', oex("cuts have curves"), 1),
               ('.expect("cut has a curve")', oex("cut has a curve"), 2),
               ('.expect("has a curve")', oex("has a curve"), 1)],
    "sew.rs": [('.expect("filtered")', oex("filtered"), 1),
               ('.expect("queued members are placed")', oex("queued members are placed"), 1),
               ('.expect("loop has a trim")', oex("loop has a trim"), 1),
               ('.expect("checked")', oex("checked"), 1)],
}
DEPS = ["yack-base", "yack-build", "yack-check", "yack-geom", "yack-intersect", "yack-locate", "yack-math",
        "yack-measure", "yack-mesh", "yack-numeric", "yack-profile", "yack-tol", "yack-topo", "yack-tweak"]
DEV = ["yack-codec"]  # the fixtures' reader; gate 1 checks dev-deps against the row too
ALIASES = {
    "error": "use yack_base::kernel as error;", "Error": "use yack_base::kernel::Error;",
    "Result": "use yack_base::kernel::Result;", "geom": "use yack_geom as geom;",
    "math": "use yack_math::kernel as math;", "numeric": "use yack_numeric as numeric;",
    "tol": "use yack_tol as tol;", "topo": "use yack_topo as topo;", "intersect": "use yack_intersect as intersect;",
    "names": "use yack_topo::names;", "locate": "use yack_locate as locate;", "rays": "use yack_locate::rays;",
    "distance": "use yack_locate::distance;", "measure": "use yack_measure as measure;",
    "check": "use yack_check as check;", "profile": "use yack_profile as profile;", "mesh": "use yack_mesh as mesh;",
    "tweak": "use yack_tweak as tweak;", "sweep": "use yack_build::sweep;", "loft": "use yack_build::loft;",
    "primitive": "use yack_build::primitive;", "build": "use yack_build::build;",
    "codec": "use yack_codec as codec;",
}


def header_note(t, s_rel):
    if re.search(r"// Changes: none \(verbatim\); this header added\.", t):
        return re.sub(r"// Changes: none \(verbatim\); this header added\.",
                      f"// Changes: moved from springrep's src/{s_rel} to {KRATE} (M3 W8); paths and visibility only.", t, count=1)
    if "// Changes:" in t[:800]:
        t = re.sub(r"(// Changes: [^\n]*(?:\n// [^\n]*)*?)\.?\n(?!// )",
                   rf"\1; then to {KRATE} (M3 W8), paths and visibility only.\n", t, count=1)
        return t.replace(f".; then to {KRATE} (M3 W8)", f"; then to {KRATE} (M3 W8)", 1)
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


# plane.rs and section.rs name the boolean module itself (`boolean::intersection`): its items are the
# crate root's now
SELF = {"boolean/plane.rs": ["intersection", "difference"], "boolean/section.rs": ["intersection"]}
mv = {}
for s_rel, d_rel in FILES.items():
    t = rd(os.path.join(src, s_rel))
    if s_rel in SELF:
        t = sub1(t, "    boolean,\n", "", s_rel)
        for f in SELF[s_rel]:
            t = re.sub(r"(?<![\w:])boolean::" + f + r"\(", "crate::" + f + "(", t)
    t = header_note(strip_module_paths(t, ["boolean"], s_rel), s_rel)
    if d_rel == "lib.rs":
        t = re.sub(r"\bpub\(super\)", "pub(crate)", t)
    if d_rel in PANICS:
        t = counted_panics(t, PANICS[d_rel], s_rel)
    if d_rel == "lib.rs":
        # counted_panics hangs a site's allow on the nearest enclosing `fn` line above it, which inside
        # `dedup` is its nested helper `fn find`: the allow belongs to `dedup`, where the panic is
        # (fixed in m3_move since: then the allow is on `dedup` already and nothing moves)
        if "        #[allow(clippy::panic)] fn find(p: &mut [usize], i: usize) -> usize {\n" in t:
            t = sub1(t, "        #[allow(clippy::panic)] fn find(p: &mut [usize], i: usize) -> usize {\n",
                     "        fn find(p: &mut [usize], i: usize) -> usize {\n", s_rel)
            t = sub1(t, "    fn dedup(&mut self) {\n", "    #[allow(clippy::panic)] fn dedup(&mut self) {\n", s_rel)
        if "    #[allow(clippy::panic)] fn dedup(&mut self) {\n" not in t:
            raise SystemExit("lib.rs: dedup's panic has no allow")
    if d_rel == "arrange.rs":
        # springrep was lint-exempt as a temporary crate; a pure move keeps the code, so its one
        # clippy::match_like_matches_macro gets an item allow until the hardening pass
        t = allow_items(t, r"let curved = match \(&tr\.map, &tr\.curve\)", "clippy::match_like_matches_macro",
                        "M3 W8: moved verbatim; hardening rewrites it as matches!", d_rel)
    mv[d_rel] = t
own = set(re.findall(r"(?m)^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", mv["lib.rs"]))
reached_all = crate_names(list(mv.values())) - own
reached = crate_names([live_text(v) for k2, v in mv.items() if not (k2.endswith("tests.rs") or k2.startswith("tests/"))]) - own
test_only = reached_all - reached
# the crate root's own items (`crate::Op`, `crate::union`) are no aliases
root_items = set(re.findall(r"(?m)^(?:pub(?:\([^)]*\))?\s+)?(?:fn|struct|enum|type|const|static|trait)\s+(\w+)", mv["lib.rs"]))
for g in re.findall(r"(?m)^pub use \w+::\{([^}]*)\};", mv["lib.rs"]):
    root_items |= {x.strip() for x in g.split(",") if x.strip()}
root_items |= set(re.findall(r"(?m)^pub use \w+::(\w+);", mv["lib.rs"]))
lines = mv["lib.rs"].split("\n")
k = next(i for i, l in enumerate(lines) if l.strip() and not l.startswith("//"))
head = ["", "// springrep's boolean code indexes by position; D26's hardening pass moves it onto checked access,",
        "// as for yack-linalg until M5.", "#![allow(clippy::indexing_slicing)]", "",
        "// springrep's foundation modules (D26), under the names the moved code uses."]
head += [ALIASES[n] for n in sorted(reached & set(ALIASES))]
head += ["#[cfg(test)]\n" + ALIASES[n] for n in sorted(test_only & set(ALIASES))]
mv["lib.rs"] = "\n".join(lines[:k] + head) + "\n" + drop_root_aliases("\n".join(lines[k:]))
unknown = reached_all - set(ALIASES) - own - root_items
if unknown:
    raise SystemExit(f"{KRATE} reaches crate-root names {sorted(unknown)} it does not have")
staged = {KRATE: mv}

REAL = "/home/brenn/github/yack/crates"
moving = {os.path.join(src, s) for s in FILES}
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
# lead's ruling: only boolean_with goes pub. precise_pub reads names textually, and these nine match
# unrelated items outside (other crates' `new`, `edges`, `faces`, `split`, blend's `assemble`, ...), not
# uses of boolean's: they stay pub(crate). m3_move.py is left as it is.
KEEP_PRIVATE = ["OutFace", "PieceGeom", "assemble", "edges", "face_bbox", "faces", "merge_edges", "new", "split"]
masked = "\n".join(outside)
for n in KEEP_PRIVATE:
    masked = re.sub(r"\b" + n + r"\b", n + "_w8_unrelated", masked)
texts, made_pub = precise_pub(staged, masked)
if made_pub.get(KRATE) != ["boolean_with"]:
    raise SystemExit(f"{KRATE}: pub(crate) -> pub for {made_pub.get(KRATE)}, expected only boolean_with")
hits = test_reach_guard(texts, made_pub, outside_tests, {KRATE: ["boolean"]})
if hits:
    raise SystemExit(f"springrep tests reach crate-private items of yack-boolean: {hits}")
unresolved = path_guard(texts, outside_tests, {KRATE: ["boolean"]})
if unresolved:
    raise SystemExit(f"springrep tests name paths yack-boolean does not have: {unresolved}")
lt0 = os.path.join(ws, "layers.toml")
s0 = rd(lt0)
m0 = re.search(r'(?ms)^\[crate\.yack-boolean\]\n.*?^deps = \[([^\]]*)\]', s0)
have0 = [x.strip().strip('"') for x in m0.group(1).split(",") if x.strip()]
add0 = [d for d in DEV if d not in have0]  # dev-only (gate 1 checks them too)
if add0:
    s0 = s0[:m0.start(1)] + ", ".join(f'"{d}"' for d in have0 + add0) + s0[m0.end(1):]
    wr(lt0, s0)
for d_rel, t in texts[KRATE].items():
    if re.search(r"\bHash(Map|Set)\b", live_text(t)):
        t = allow_items(t, r"\bHash(Map|Set)\b", "clippy::disallowed_types",
                        "D26: BTree* in hardening, proven by digests", d_rel)
    wr(os.path.join(N, d_rel), t)
deps_guard(ws, KRATE, DEPS + DEV)
c = cargo(KRATE, DEPS)
c = c.replace("\n[lints]", "\n[dev-dependencies]\n" + "".join(f'{d} = {{ path = "../{d}" }}\n' for d in DEV) + "\n[lints]")
wr(os.path.join(crates, KRATE, "Cargo.toml"), c)
e, pnc = count([N]), count(["--panics", N])
want = sum(n * b.count("panic!(") for _, b, n in sum(PANICS.values(), []))
if pnc != want:
    raise SystemExit(f"{KRATE}: {pnc} contract panics, expected {want}")
size = sum(len(rd(p).split("\n")) - 1 for p in rs_files(N))
for s_rel in FILES:
    os.remove(os.path.join(src, s_rel))
for a, _, _ in sorted(os.walk(B), reverse=True):
    os.rmdir(a)

# ---- 2. springrep: re-export and dependency ----
lp = os.path.join(src, "lib.rs")
l = sub1(rd(lp), "pub mod boolean;\n", "pub use yack_boolean as boolean;\n", lp)
# boolean was the last of springrep's live code to name `crate::locate`: the alias goes, or clippy warns
rest = "\n".join(live_text(rd(p)) for p in rs_files(src) if p != lp and os.path.basename(p) != "tests.rs"
                 and "/moved_tests/" not in p)
if not re.search(r"(?m)\bcrate::locate\b|^\s+locate::|[{,\s]locate::\{", rest):
    l = sub1(l, "pub(crate) use yack_locate as locate;\n", "", lp)
wr(lp, l)
springrep_deps(ws, root, [KRATE])

# ---- 3. layers.toml map, scores ----
lt = os.path.join(ws, "layers.toml")
s_ = rd(lt)
if size > 5000 and f"[modules.{KRATE}]" not in s_:
    mp_ = unit_edges(N)
    block = f"[modules.{KRATE}]\n" + "".join(f"{u} = [{', '.join(chr(34) + v + chr(34) for v in vs)}]\n" for u, vs in mp_) + "\n"
    s_ = sub1(s_, "[modules.yack-springrep]\n", block + "[modules.yack-springrep]\n", lt)
# springrep's map keeps its boolean row: `boolean` stays a name there, re-exported (as W7 kept its modules')
wr(lt, s_)
sd = os.path.join(ws, "scores")
set_row(os.path.join(sd, "eps.txt"), KRATE, e)
if pnc:
    set_row(os.path.join(sd, "lints.txt"), KRATE, pnc)
fp = os.path.join(sd, "filesize.txt")
f = rd(fp)
# a row for a new path is the file's size at move time (lead): the moved files carry the D26 header and
# per-item allows, so each row is the post-apply line count, and may only fall afterwards
def moved_row(m):
    rel = "lib.rs" if m.group(1) == "mod.rs" else m.group(1)
    return f"crates/{KRATE}/src/{rel} {len(rd(os.path.join(N, rel)).split(chr(10))) - 1}"
f = re.sub(r"(?m)^crates/yack-springrep/src/boolean/(\S+) \d+$", moved_row, f)
wr(fp, f)
print("files:", sorted(FILES.values()))
print("made pub:", made_pub[KRATE])
print("rows:", (KRATE, e, pnc, size))
