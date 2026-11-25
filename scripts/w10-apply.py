# yack: M3 W10 (lead's plan and rulings): springrep's facade/ split into the crates it wraps. api.rs's items go
# to yack-mesh `pick`, yack-topo `local`, yack-measure `guard`, yack-geom (bspline.rs) and yack-edit
# `requirement`; edit.rs, exchange.rs, loops.rs and bridge.rs move whole to yack-edit `glue`, yack-step
# `exchange`, yack-profile `loops` and yack-bridge `builder`. springrep's facade re-exports each by its old
# path until W11. Bodies move verbatim; only paths, the files' heads and clippy allows change.
# Usage: python3 w10-apply.py <yack-springrep dir> [--with-external ...]
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(1, "/home/brenn/github/yack/scripts")
from m3_move import allow_items, count, live_text, rd, rs_files, set_row, sub1, wr

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)
FAC = os.path.join(src, "facade")
WAVE = "M3 W10"
C = lambda k, *p: os.path.join(crates, k, "src", *p)
KS = ["yack-mesh", "yack-topo", "yack-measure", "yack-geom", "yack-edit", "yack-step", "yack-profile", "yack-springrep"]
BEFORE = {k: (count([C(k, "")]), count(["--panics", C(k, "")])) for k in KS}


def rewrite(t, pairs, f):
    """Each (regex, replacement) must match at least once, so a stale pattern fails loudly."""
    for a, b in pairs:
        t, n = re.subn(a, b, t)
        if not n:
            raise SystemExit(f"{f}: no match for {a!r}")
    return t


def first_line(t, new, f):
    if not t.startswith("// yack: "):
        raise SystemExit(f"{f}: no // yack: first line")
    return new + t[t.index("\n"):]


# ---- 1. facade/api.rs: cut into its items ----
api_p = os.path.join(FAC, "api.rs")
api = rd(api_p)
cut = api.index("#[cfg(test)]\nmod tests {")
live, tests = api[:cut], api[cut:]
lines = live.split("\n")
start_re = re.compile(r"^(?:pub )?(?:fn|enum) (\w+)\b")
items, used = {}, set()
for k, l in enumerate(lines):
    m = start_re.match(l)
    if not m:
        continue
    s = k
    while s > 0 and (lines[s - 1].startswith("///") or lines[s - 1].startswith("#[")):
        s -= 1
    e = next(j for j in range(k, len(lines)) if lines[j] == "}")
    items[m.group(1)] = "\n".join(lines[s:e + 1])
    used |= set(range(s, e + 1))
# everything else of api.rs's live part is its head (the `use` block), blank lines and the guards' banner
use_end = live.index("};\n", live.index("use crate::{")) + 3
head_lines = live[:use_end].count("\n")
rest = [lines[k] for k in range(head_lines, len(lines)) if k not in used and lines[k].strip()]
if rest != ["// ---- guards: shapes springrep would build wrong, which its rung must decline (M1.10) ----"]:
    raise SystemExit(f"api.rs: lines no item claims: {rest}")
PLAN = {
    "pick": ["near", "Pick", "pick", "face_within", "is_seam", "edge_within", "face_closest", "nearest_face", "blend_strays"],
    "local": ["convexity", "face_normal_at", "edge_points", "edge_point", "radius_collapses", "rims_square", "open_at_concave_edge"],
    "guard": ["face_grid", "same_shape", "walls_cross", "plane_cuts", "boxes_overlap"],
    "geom": ["periodic_through"],
    "requirement": ["breaks_requirement"],
}
planned = [n for v in PLAN.values() for n in v]
if sorted(planned) != sorted(items) or len(set(planned)) != len(planned):
    raise SystemExit(f"api.rs items {sorted(items)} != plan {sorted(planned)}")
PUB = {g: [n for n in v if re.search(r"^(?:///[^\n]*\n|#\[[^\n]*\n)*pub ", items[n])] for g, v in PLAN.items()}


def body(group, pairs):
    t = "\n\n".join(items[n] for n in PLAN[group])
    return rewrite(t, pairs, f"api.rs -> {group}") if pairs else t


def imports(text, cands):
    """The `use` lines whose names the code (not its comments) names."""
    code = re.sub(r"//[^\n]*", "", text)
    return [u for n, u in cands if re.search(r"(?<![\w:])" + n + r"\b", code)]


def merged(uses):
    by = {}
    for u in uses:
        p, n = u.rsplit("::", 1)
        by.setdefault(p, []).append(n)
    return "".join(f"use {p}::{{{', '.join(sorted(ns, key=str.lower))}}};\n" if len(ns) > 1 else f"use {p}::{ns[0]};\n"
                   for p, ns in sorted(by.items()))


MATH = [("Aabb", "crate::math::Aabb"), ("Point3", "crate::math::Point3"), ("Vec3", "crate::math::Vec3")]

# yack-mesh `pick`: measure::face_bbox is yack-topo's bbox::face_bbox, which yack-measure only re-exports
pk = body("pick", [(r"(?<![\w:])measure::face_bbox\(", "crate::topo::bbox::face_bbox("),
                   (r"crate::mesh::nearest_of\(", "crate::nearest_of(")])
pk_use = imports(pk, [("Result", "crate::Result"), ("Selection", "crate::distance::Selection")] + MATH +
                 [("Brep", "crate::topo::Brep"), ("EdgeId", "crate::topo::EdgeId"), ("FaceId", "crate::topo::FaceId")])
PICK = ("// yack: picks within a tolerance, and the nearest face (moved from springrep's facade/api.rs, M3 W10).\n"
        "//! What a pick within a tolerance finds, the face nearest a point, and blend faces that stray from where\n"
        "//! they were asked for: the facade's picks, over [`crate::nearest_of`].\n\n"
        + merged(pk_use) + "\n" + pk + "\n")

lc = body("local", None)
lc_use = imports(lc, MATH + [("Brep", "crate::Brep"), ("EdgeId", "crate::EdgeId"), ("FaceId", "crate::FaceId")])
LOCAL = ("// yack: a solid's shape at one edge or face (moved from springrep's facade/api.rs, M3 W10).\n"
         "//! A solid's shape at one place: an edge's convexity and points, a face's outward normal, and the\n"
         "//! local checks the facade's edits make before they run.\n\n" + merged(lc_use) + "\n" + lc + "\n")

gd = body("guard", [(r"(?<![\w:])measure::", "crate::")])
gd_use = imports(gd, MATH + [("Brep", "crate::topo::Brep"), ("FaceId", "crate::topo::FaceId")])
GUARD = ("// yack: measured guards on a solid's shape, and a face's grid (moved from springrep's facade/api.rs, M3 W10).\n"
         "//! Guards the facade runs before an operation, from a solid's measures: shapes springrep would build\n"
         "//! wrong, which its rung must decline (M1.10), and a face's grid of points.\n\n"
         + merged(gd_use) + "\n" + gd + "\n")

REQ_TEST = "a_blend_with_no_edges_breaks_a_requirement"
m_ = re.search(r"\n(    #\[test\]\n    fn " + REQ_TEST + r"\(\) \{\n.*?\n    \}\n)", tests, re.S)
if not m_:
    raise SystemExit(f"api.rs: no test {REQ_TEST}")
REQ = ("// yack: which refusals break a stated requirement (moved from springrep's facade/api.rs, M3 W10).\n"
       "//! Which of an operation's refusals say the caller broke a stated requirement (a size that must be\n"
       "//! positive, a blend that must fit), rather than that the kernel could not do it.\n\n"
       + body("requirement", None) + "\n\n#[cfg(test)]\nmod tests {\n    use super::breaks_requirement;\n\n"
       + m_.group(1) + "}\n")
tests_left = tests.replace(m_.group(0), "\n", 1)
# the remaining tests name it only as `super::breaks_requirement` (w-h1-60: a bare import of it was unused
# and raised springrep's clippy count 0 -> 1), so they import only what they name bare
keep_req = re.search(r"(?<![\w:])breaks_requirement\(", tests_left) is not None
tests_left = sub1(tests_left, "    use super::{Point3, Vec3, breaks_requirement};\n",
                  ("    use super::breaks_requirement;\n" if keep_req else "") + "    use crate::math::{Point3, Vec3};\n", api_p)
tests_left = re.sub(r"\n\n\n+", "\n\n", tests_left).replace("\n\n}\n", "\n}\n")

# yack-geom: periodic_through goes into bspline.rs, before its tests
pt = body("geom", None)
bs_p = C("yack-geom", "bspline.rs")
bs = rd(bs_p)
if "fn periodic_through" not in bs:
    bs = sub1(bs, "\n#[cfg(test)]\nmod tests {", "\n" + pt + "\n\n#[cfg(test)]\nmod tests {", bs_p)

# ---- 2. the whole files ----
ed = rd(os.path.join(FAC, "edit.rs"))
ed = first_line(ed, "// yack: glue for yack's edits: offsets, face moves, drafts and transforms over this crate's edits (moved from springrep's facade/edit.rs, M3 W10).", "edit.rs")
ed = rewrite(ed, [(r"use crate::\{\n    Result,\n    edit,\n", "use crate::{\n    Result,\n"),
                  (r"(?<![\w:])edit::", "crate::")], "edit.rs")
ed = allow_items(ed, r"\bHash(Map|Set)\b", "clippy::disallowed_types", "D26: BTree* in hardening, proven by digests", "glue.rs")

ex = rd(os.path.join(FAC, "exchange.rs"))
ex = first_line(ex, "// yack: glue for yack's STEP and codec exchange over this crate and yack-codec (moved from springrep's facade/exchange.rs, M3 W10).", "exchange.rs")
ex = rewrite(ex, [(r"use crate::\{Result, codec, step, topo::Brep\};", "use crate::{Result, topo::Brep};"),
                  (r"crate::measure::", "yack_measure::"),
                  (r"(?<![\w:])codec::", "yack_codec::"),
                  (r"(?<![\w:])step::", "crate::")], "exchange.rs")

lo = rd(os.path.join(FAC, "loops.rs"))
lo = first_line(lo, "// yack: one closed planar loop rounded at every corner, or offset, over this crate's regions (moved from springrep's facade/loops.rs, M3 W10).", "loops.rs")
lo = rewrite(lo, [(r"    profile::\{\n        Names, Segment,\n        region::\{Geom, Loop, Region, Seg\},\n    \},\n",
                   "    Names, Segment,\n    region::{Geom, Loop, Region, Seg},\n")], "loops.rs")
# clippy's one site (swarm/res/w10-facade-clippy.txt, loops.rs:78): the first and last segment of a loop
# already known to be non-empty
lo = allow_items(lo, r"out\[0\]\.geom\.start\(\)", "clippy::indexing_slicing", "D26: checked access in hardening", "loops.rs")

br = rd(os.path.join(FAC, "bridge.rs"))
br = first_line(br, "// yack: a checked builder of springrep `Brep`s for yack-bridge (moved from springrep's facade/bridge.rs, M3 W10).", "bridge.rs")
br = rewrite(br, [(r"use crate::\{\n    Error, Result, check,\n    geom::\{Curve, Surface\},\n    math::Point3,\n"
                   r"    topo::\{Brep, CoedgeSpec, EdgeId, FaceId, FaceName, LoopId, ShellId, SolidId, VertexId\},\n\};\n",
                   "use yack_base::kernel::{Error, Result};\nuse yack_check as check;\nuse yack_geom::{Curve, Surface};\n"
                   "use yack_math::kernel::Point3;\n"
                   "use yack_topo::{Brep, CoedgeSpec, EdgeId, FaceId, FaceName, LoopId, ShellId, SolidId, VertexId};\n")], "bridge.rs")

# ---- 3. write the new homes and their `mod` lines ----
def add_line(p, after, line):
    t = rd(p)
    if line not in t.split("\n"):
        t = sub1(t, after + "\n", after + "\n" + line + "\n", p)
    wr(p, t)


wr(C("yack-mesh", "pick.rs"), PICK)
add_line(C("yack-mesh", "lib.rs"), "mod face;", "pub mod pick;")
wr(C("yack-topo", "local.rs"), LOCAL)
add_line(C("yack-topo", "lib.rs"), "pub mod history;", "pub mod local;")
wr(C("yack-measure", "guard.rs"), GUARD)
add_line(C("yack-measure", "lib.rs"), "mod sample;", "pub mod guard;")
wr(bs_p, bs)
gl = C("yack-geom", "lib.rs")
g_ = rd(gl)
if "periodic_through" not in g_:
    g_ = sub1(g_, "pub use bspline::BSpline;\n", "pub use bspline::{BSpline, periodic_through};\n", gl)
wr(gl, g_)
wr(C("yack-edit", "requirement.rs"), REQ)
wr(C("yack-edit", "glue.rs"), ed)
add_line(C("yack-edit", "lib.rs"), "mod affine;", "pub mod glue;")
add_line(C("yack-edit", "lib.rs"), "pub mod glue;", "pub mod requirement;")
wr(C("yack-step", "exchange.rs"), ex)
add_line(C("yack-step", "lib.rs"), "mod write;", "pub mod exchange;")
wr(C("yack-profile", "loops.rs"), lo)
add_line(C("yack-profile", "lib.rs"), "pub mod region;", "pub mod loops;")
wr(C("yack-bridge", "builder.rs"), br)
bl = C("yack-bridge", "lib.rs")
b_ = rd(bl)
if "mod builder;" not in b_:
    b_ = sub1(b_, "\nmod report;\n", "\nmod builder;\nmod report;\n", bl)
wr(bl, b_)
ts = C("yack-bridge", "to_springrep.rs")
t_ = rd(ts)
if "crate::builder::Builder" not in t_:
    t_ = sub1(t_, "    facade::bridge::Builder,\n", "", ts)
    t_ = sub1(t_, "\nuse yack_springrep::{\n", "\nuse crate::builder::Builder;\nuse yack_springrep::{\n", ts)
wr(ts, t_)
bc = os.path.join(crates, "yack-bridge", "Cargo.toml")
c_ = rd(bc)
for d in ["yack-topo", "yack-math", "yack-geom", "yack-check", "yack-base"]:  # each lands right after yack-cadrs
    if f"\n{d} = " not in c_:
        c_ = sub1(c_, 'yack-cadrs = { path = "../yack-cadrs" }\n',
                  f'yack-cadrs = {{ path = "../yack-cadrs" }}\n{d} = {{ path = "../{d}" }}\n', bc)
wr(bc, c_)

# ---- 3b. each crate's Cargo.toml carries every crate its new code names (w-h1-58: yack-step had yack-codec
# and yack-measure only as dev-dependencies) ----
NEW = {"yack-mesh": ["pick.rs"], "yack-topo": ["local.rs"], "yack-measure": ["guard.rs"], "yack-geom": ["bspline.rs"],
       "yack-edit": ["glue.rs", "requirement.rs"], "yack-step": ["exchange.rs"], "yack-profile": ["loops.rs"],
       "yack-bridge": ["builder.rs"]}
for k, fs in NEW.items():
    code = "\n".join(live_text(rd(C(k, f))) for f in fs)
    names = sorted({c.replace("_", "-") for c in re.findall(r"(?<![\w:])(yack_\w+?)(?:::|\b(?=\s+as\b)|;)", code)} - {k})
    cp = os.path.join(crates, k, "Cargo.toml")
    c = rd(cp)
    sec = lambda name: re.search(r"(?ms)^\[" + re.escape(name) + r"\]\n(.*?)(?=^\[|\Z)", c)
    for d in names:
        dep, dev = sec("dependencies"), sec("dev-dependencies")
        if re.search(r"(?m)^" + re.escape(d) + r" = ", dep.group(1)):
            continue
        line = f'{d} = {{ path = "../{d}" }}\n'
        if dev and line in dev.group(1):  # a dev-dependency the live code now needs: move it up
            c = c[:dev.start(1)] + dev.group(1).replace(line, "", 1) + c[dev.end(1):]
            if not sec("dev-dependencies").group(1).strip():
                c = re.sub(r"(?m)^\[dev-dependencies\]\n\n?", "", c)
            dep = sec("dependencies")
        c = c[:dep.end(1)].rstrip("\n") + "\n" + line + "\n" + c[dep.end(1):].lstrip("\n")
        print(f"cargo: {k} gains {d}")
    wr(cp, c)

# ---- 4. springrep: the facade re-exports ----
def pub_use(path, names):
    names = sorted(names, key=lambda n: (n[0].islower(), n))
    return f"pub use {path}::{{{', '.join(names)}}};\n" if len(names) > 1 else f"pub use {path}::{names[0]};\n"


API = ("// yack: the facade's glue items, re-exported from the crates they moved to (M3 W10); dies with this crate (M3).\n"
       "//! Glue for `yack::make`, selection and blends, now in the crates it wraps: picks in `yack_mesh::pick`,\n"
       "//! local shape in `yack_topo::local`, measured guards in `yack_measure::guard`, and so on.\n\n"
       + pub_use("yack_edit::requirement", PUB["requirement"]) + pub_use("yack_geom", PUB["geom"])
       + pub_use("yack_measure::guard", PUB["guard"]) + pub_use("yack_mesh::pick", PUB["pick"])
       + pub_use("yack_topo::local", PUB["local"]) + "\n" + tests_left.lstrip("\n"))
wr(api_p, API)
mp = os.path.join(FAC, "mod.rs")
m = rd(mp)
m = sub1(m, "pub mod api;\npub mod bridge;\npub mod edit;\npub mod exchange;\npub mod loops;\n",
         "pub mod api;\npub use yack_edit::glue as edit;\npub use yack_profile::loops;\npub use yack_step::exchange;\n", mp)
wr(mp, m)
for f in ["bridge.rs", "edit.rs", "exchange.rs", "loops.rs"]:
    os.remove(os.path.join(FAC, f))

# springrep's private crate-root aliases the facade was the last to use (w-h4's W8 lesson): test-only ones
# become #[cfg(test)], unused ones go
lp = os.path.join(src, "lib.rs")
lt_ = rd(lp)
rest_live = "\n".join(live_text(rd(p_)) for p_ in rs_files(src) if p_ != lp and os.path.basename(p_) != "tests.rs"
                      and "/moved_tests/" not in p_)
rest_all = "\n".join(rd(p_) for p_ in rs_files(src) + rs_files(os.path.join(root, "tests")) if p_ != lp)
named = lambda n, t: re.search(r"\bcrate::" + n + r"\b|use crate::\{[^;]*(?<![\w:])" + n + r"\b", t)
for a in list(re.finditer(r"(?m)^(?:pub\(crate\) )?use [\w:]+ as (\w+);\n", lt_)):
    n_ = a.group(1)
    root_rest = live_text(lt_.replace(a.group(0), ""))
    if named(n_, rest_live) or named(n_, root_rest) or re.search(r"(?<![\w:])" + n_ + r"::", root_rest):
        continue
    if named(n_, rest_all):
        if f"#[cfg(test)]\n{a.group(0)}" not in lt_:
            lt_ = lt_.replace(a.group(0), "#[cfg(test)]\n" + a.group(0), 1)
            print("springrep alias test-only:", n_)
    else:
        lt_ = lt_.replace(a.group(0), "", 1)
        print("springrep alias dropped:", n_)
wr(lp, lt_)

# ---- 5. layers.toml maps (idempotent), scores ----
def heads(text, prefix):
    out = set(re.findall(r"(?<![\w:])" + prefix + r"(\w+)", text))
    for g in re.findall(r"(?<![\w:])use\s+" + prefix + r"\{([^;]*)\};", text):
        depth, cur = 0, ""
        for ch in g + ",":
            if ch == "{": depth += 1
            if ch == "}": depth -= 1
            if ch == "," and depth == 0:
                h = re.match(r"\s*(\w+)", cur)
                out |= {h.group(1)} if h else set()
                cur = ""
            else:
                cur += ch
    return out


def edges(krate, unit):
    s_ = C(krate, "")
    mods = set(re.findall(r"(?m)^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", rd(C(krate, "lib.rs"))))
    refs = set()
    files = [(os.path.join(s_, unit + ".rs"), 0)] + [(p, 1) for p in rs_files(os.path.join(s_, unit)) if os.path.basename(p) != "tests.rs"]
    for p, depth in files:
        if os.path.exists(p):
            t = live_text(rd(p))
            refs |= heads(t, "crate::") | heads(t, "super::" * (depth + 1))
    return sorted((refs & mods) - {unit})


lt = os.path.join(ws, "layers.toml")
s = rd(lt)
row = lambda u, vs: f"{u} = [{', '.join(chr(34) + v + chr(34) for v in vs)}]\n"
blk = re.search(r"(?ms)^\[modules\.yack-step\]\n(.*?)\n(?=\[|\Z)", s)
if not re.search(r"(?m)^exchange = ", blk.group(1)):
    s = s[:blk.start(1)] + row("exchange", edges("yack-step", "exchange")) + blk.group(1) + s[blk.end(1):]
blk = re.search(r"(?ms)^\[modules\.yack-springrep\]\n(.*?)\n(?=\[|\Z)", s)
fr = re.search(r"(?m)^facade = \[[^\]]*\]\n", blk.group(1) + "\n")
new_fr = row("facade", edges("yack-springrep", "facade"))
s = s[:blk.start(1)] + (blk.group(1) + "\n").replace(fr.group(0), new_fr, 1).rstrip("\n") + s[blk.end(1):]
wr(lt, s)
print("modules:", row("exchange", edges("yack-step", "exchange")).strip(), "|", new_fr.strip())
sd = os.path.join(ws, "scores")
# gate 7's eps rows follow the counts where the move changes them (a row may only fall, so a crate the move
# fills gets its new count and springrep's stands where it already lies below its row); gate 9's contract
# panics must not change at all (clippy listed no expect/unwrap/panic site in the moved live code)
for k in KS:
    e, pn = count([C(k, "")]), count(["--panics", C(k, "")])
    if pn != BEFORE[k][1]:
        raise SystemExit(f"{k}: contract panics {BEFORE[k][1]} -> {pn}")
    if e != BEFORE[k][0]:
        rows = dict(r.split() for r in rd(os.path.join(sd, "eps.txt")).split("\n") if r.strip() and not r.startswith("#"))
        if k == "yack-springrep" and e <= int(rows.get(k, 0)):
            continue
        set_row(os.path.join(sd, "eps.txt"), k, e)
        print("eps:", k, BEFORE[k][0], "->", e)
print("pub:", PUB)
