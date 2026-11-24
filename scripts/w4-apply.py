# M3 W4: springrep's topo/ (with names.rs, PLAN §3: names belong to topo), profile and intersect/
# become yack-topo, yack-profile and yack-intersect, re-exported by springrep under their old names.
# Lead's rulings: yack-profile moves to layer 5 above yack-topo (it names FaceName); memo.rs's static
# id counter moves as is under a scores/hidden.txt allowance (a per-Brep counter follows); HashMap/
# HashSet stay under site allows (BTree* follows); `.expect`/`.unwrap` become counted panics as in W3;
# indexing allows at each crate root as in W2'/W3; tests that build their operands with a module
# above the new crate move, verbatim, into springrep beside that module's tests.
# Usage: python3 w4-apply.py <yack-springrep dir> [--with-external ...]
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m3_move import (OPTION_UNWRAP, allow_items, items, live_text, precise_pub, cargo, count, counted_panics, crate_names, rd, rs_files, set_row,
                     split_tests, springrep_deps, strip_module_paths, sub1, trim_numeric, unit_edges, wr)

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)
OU = OPTION_UNWRAP


def rex(msg):
    return f'.unwrap_or_else(|e| panic!("{msg}: {{e:?}}"))'


def oex(msg):
    return f'.unwrap_or_else(|| panic!("{msg}"))'


# ---- 0. tests that reach above their new crate move first, while their files are in place ----
moved_tests = []
# march <-> ssi was a module cycle (march named ssi's SsiCurve, ssi calls march::trace): the curve
# record goes down into march, and ssi imports it (lead's ruling), so ssi = ["march"], march = []
mp, sp = os.path.join(src, "intersect", "march.rs"), os.path.join(src, "intersect", "ssi.rs")
ms, ss = rd(mp), rd(sp)
m = re.search(r"\n#\[derive\(Clone, Debug\)\]\npub\(crate\) struct SsiCurve \{\n(?:    .*\n)*?\}\n", ss)
if not m or "impl SsiCurve" in ss:
    raise SystemExit("ssi.rs: SsiCurve is not the plain struct expected")
ss = ss.replace(m.group(0), "\n", 1)
ss = sub1(ss, "use super::march;\n", "use super::march::{self, SsiCurve};\n", sp)
ms = sub1(ms, "use super::ssi::SsiCurve;\n", "", mp)
k = ms.index("\n};\n") + len("\n};\n")  # after march's `use crate::{..};`
ms = ms[:k] + m.group(0) + ms[k:]
# boolean/tests.rs's tests of march use only geom surfaces: they move into march's own tests
bp = os.path.join(src, "boolean", "tests.rs")
bt = rd(bp).split("\n")
spans = {n: (j, e) for n, j, e in items(bt)}
take = ["a_branch_is_traced_both_ways_from_a_seed_in_its_middle", "saddle_pair",
        "two_touching_points_on_a_shared_line_are_two_crossings",
        "branches_through_two_touching_points_a_step_apart_reach_both"]
if any(n not in spans for n in take):
    raise SystemExit(f"boolean/tests.rs: missing {[n for n in take if n not in spans]}")
texts = ["\n".join(bt[spans[n][0]:spans[n][1]]) for n in take]
rest = "\n".join(l for k, l in enumerate(bt) if not any(spans[n][0] <= k < spans[n][1] for n in take))
if re.search(r"\bsaddle_pair\b|\bmarch::", rest):
    raise SystemExit("boolean/tests.rs: something left still uses saddle_pair or march")
rest = re.sub(r"\n{3,}", "\n\n", rest)
rest = sub1(rest, "    intersect::march,\n", "", bp)
wr(bp, rest)
moved = "\n\n".join("    " + t.replace("\n", "\n    ").replace("\n    \n", "\n\n") for t in texts)
moved = moved.replace("march::trace(", "trace(").replace("march::same_crossing(", "same_crossing(")
ms = ms.rstrip("\n")
if not ms.endswith("}"):
    raise SystemExit("march.rs: its tests module does not end the file")
ms = ms[:-1].rstrip("\n") + "\n\n    // Moved from boolean/tests.rs (M3 W4): march's own tests on geom surfaces.\n" + moved + "\n}\n"
# the names they need beyond march's tests' scope
ms = sub1(ms, "        geom::{Cylinder, Plane, Torus},\n", "        geom::{Cylinder, Plane, Torus},\n        math::Transform,\n", mp) if "Transform" in moved and "Transform" not in ms.split("mod tests")[1].split("fn ")[0] else ms
wr(mp, ms)
wr(sp, ss)
moved_tests.append(("boolean/tests.rs", "intersect/march.rs", take, []))
# march's loft tests need loft, above yack-intersect: they move to loft's tests and reach the
# crossing through ssi::intersect, the public path, by a local macro (ssi falls back to march::trace
# for these free-form pairs); their bodies are verbatim but for `trace(` -> `ssi_trace!(`
SSI_TRACE = """    /// `march::trace` through the public path: `ssi::intersect` traces a free-form pair from the seeds.
    macro_rules! ssi_trace {
        ($a:expr, $b:expr, $region:expr, $seeds:expr) => {
            match crate::intersect::ssi::intersect($a, $b, $region, &mut || $seeds.to_vec()) {
                Ok(crate::intersect::ssi::Ssi::Curves { curves, points }) => Ok::<_, crate::Error>((curves, points)),
                Ok(_) => panic!("a free-form pair has a closed form"),
                Err(e) => Err(e),
            }
        };
    }

"""
mod, tests, helpers = split_tests(
    mp, ["crate", "intersect", "march"], {"loft", "Section", "Profile", "PointsInterp"}, "intersect_march",
    "Moved from intersect/march.rs's tests (M3 W4): they trace across lofts, and yack-intersect sits below "
    "loft; they reach the trace through ssi::intersect.",
    transform=lambda t: re.sub(r"(?<![\w:])trace\(", "ssi_trace!(", t), preamble=SSI_TRACE)
lt_ = os.path.join(src, "loft", "tests.rs")
wr(lt_, rd(lt_).rstrip("\n") + "\n\n" + mod)
moved_tests.append(("intersect/march.rs", "loft/tests.rs", tests, helpers))
# pcurve.rs's pole test stretched a primitive sphere with edit::affine, both above yack-topo, and its
# asserts call pcurve's private leaving_u, so it stays and builds the same skin itself: the rational
# revolution of a half circle edit::affine makes of a sphere, its control points mapped (lead's ruling
# (b); the asserts and tolerances are verbatim)
pc = os.path.join(src, "topo", "pcurve.rs")
t = rd(pc)
t = sub1(t, "        edit::{Affine, affine},\n", "", pc)
t = sub1(t, "        primitive::sphere,\n", "", pc)
t = sub1(t, """        let map = Affine {
            m: [
                [0.1028, -0.9280, 0.5227],
                [-0.6650, 0.2041, 0.6977],
                [1.2920, 0.5654, 0.3662],
            ],
            t: Vec3::new(20.147, -35.624, 7.546),
        };
        let f = Frame::new(Vec3::new(-2.9, -1.3, -1.2), Vec3::Z, Vec3::X).unwrap();
        let ball = affine(&sphere(&f, 1.8, "sph").unwrap(), &map).unwrap();
        let s = &ball.face(ball.face_ids().next().unwrap()).surface;
""", """        // The skin of a ball of radius 1.8 on `f` under the affine map `m`, `t`, as edit::affine
        // builds it: the rational revolution of a half circle, its control points mapped (M3 W4:
        // built here, since yack-topo sits below primitive and edit).
        let m = [
            [0.1028, -0.9280, 0.5227],
            [-0.6650, 0.2041, 0.6977],
            [1.2920, 0.5654, 0.3662],
        ];
        let t = Vec3::new(20.147, -35.624, 7.546);
        let map = |p: Vec3| {
            let r = m.map(|row| row[0] * p.x + row[1] * p.y + row[2] * p.z);
            Vec3::new(r[0], r[1], r[2]) + t
        };
        let f = Frame::new(Vec3::new(-2.9, -1.3, -1.2), Vec3::Z, Vec3::X).unwrap();
        let (r, h) = (1.8, std::f64::consts::FRAC_1_SQRT_2);
        let half = [[0.0, -r], [r, -r], [r, 0.0], [r, r], [0.0, r]];
        let half_w = [1.0, h, 1.0, h, 1.0];
        let ring = [
            [1.0, 0.0, 1.0],
            [1.0, 1.0, h],
            [0.0, 1.0, 1.0],
            [-1.0, 1.0, h],
            [-1.0, 0.0, 1.0],
            [-1.0, -1.0, h],
            [0.0, -1.0, 1.0],
            [1.0, -1.0, h],
            [1.0, 0.0, 1.0],
        ];
        let (mut ctrl, mut weights) = (Vec::new(), Vec::new());
        for c in ring {
            for (p, w) in half.iter().zip(&half_w) {
                let local = f.x * (p[0] * c[0]) + f.y * (p[0] * c[1]) + f.z * p[1];
                ctrl.push(map(f.origin + local));
                weights.push(c[2] * w);
            }
        }
        let u_knots = vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
        let skin = crate::geom::NurbsSurface::new((2, 2), (u_knots, v_knots), (9, 5), ctrl, Some(weights)).unwrap();
        let s = &Surface::Nurbs(Box::new(skin));
""", pc)
if re.search(r"\b(affine|sphere|Affine)\(|Affine \{", t.split("#[cfg(test)]")[-1]):
    raise SystemExit("pcurve.rs: the test still names edit or primitive")
wr(pc, t)

hp = os.path.join(src, "topo", "history.rs")
if "crate::primitive" in rd(hp).split("#[cfg(test)]")[-1]:
    mod, tests, helpers = split_tests(hp, ["crate", "topo", "history"], {"block"}, "topo_history",
                                      "Moved from topo/history.rs's tests (M3 W4): they build blocks, and yack-topo sits below primitive.")
    pp = os.path.join(src, "primitive.rs")
    wr(pp, rd(pp).rstrip("\n") + "\n\n" + mod)
    moved_tests.append(("topo/history.rs", "primitive.rs", tests, helpers))

# ---- 1. the modules ----
SPECS = [
    # (springrep module, crate, {src rel: dest rel}, panics {dest rel: [(old, new, n)]}, deps)
    ("topo", "yack-topo",
     dict([("topo/mod.rs", "lib.rs"), ("names.rs", "names.rs")]
          + [(f"topo/{f}", f) for f in sorted(os.listdir(os.path.join(src, "topo"))) if f != "mod.rs"]),
     {"similar.rs": [('.last().expect("samples")', ".last()" + oex("samples"), 1)],
      "lib.rs": [('u32::try_from(i).expect("arena index fits in u32")', 'u32::try_from(i)' + rex("arena index fits in u32"), 1)]},
     ["yack-base", "yack-geom", "yack-math", "yack-numeric", "yack-tol"]),
    ("profile", "yack-profile",
     {"profile.rs": "lib.rs", "profile/region.rs": "region.rs"},
     {"lib.rs": [('.expect("a Bézier\'s knots and weights fit its points")', rex("a Bézier's knots and weights fit its points"), 1)],
      "region.rs": [("lp.last().unwrap()", "lp.last()" + OU, 2)]},
     ["yack-base", "yack-geom", "yack-math", "yack-numeric", "yack-tol", "yack-topo"]),
    ("intersect", "yack-intersect",
     dict([("intersect/mod.rs", "lib.rs")]
          + [(os.path.relpath(p, src), os.path.relpath(p, os.path.join(src, "intersect")))
             for p in rs_files(os.path.join(src, "intersect")) if not p.endswith("intersect/mod.rs")]),
     {"csi.rs": [('.expect("conic has a plane")', oex("conic has a plane"), 1),
                 ('.expect("an index has spans")', oex("an index has spans"), 1),
                 ('.expect("a wanted sample\'s neighbours are sampled")', oex("a wanted sample's neighbours are sampled"), 1)],
      "ssi.rs": [("axis_of(a).unwrap(), axis_of(b).unwrap()", "axis_of(a)" + OU + ", axis_of(b)" + OU, 2),
                 ('.expect("axis is a unit vector")', rex("axis is a unit vector"), 1),
                 ('.expect("planes are not parallel")', oex("planes are not parallel"), 1),
                 ('.expect("axis lies along the plane")', oex("axis lies along the plane"), 1),
                 ('.expect("axis is oblique to the plane")', oex("axis is oblique to the plane"), 1),
                 ('.expect("axes are not parallel")', oex("axes are not parallel"), 2)]},
     ["yack-base", "yack-geom", "yack-math", "yack-numeric", "yack-tol"]),
]
ALIASES = {  # crate-root name -> the line that brings it, D26's foundation modules
    "error": "use yack_base::kernel as error;", "Error": "use yack_base::kernel::Error;",
    "Result": "use yack_base::kernel::Result;", "geom": "use yack_geom as geom;",
    "math": "use yack_math::kernel as math;", "numeric": "use yack_numeric as numeric;",
    "tol": "use yack_tol as tol;", "names": "use yack_topo::names;", "topo": "use yack_topo as topo;",
}
eps_total, rows, staged = 0, [], []
for mod, krate, files, panics, deps in SPECS:
    N = os.path.join(crates, krate, "src")
    moved = {}
    for s_rel, d_rel in files.items():
        t = rd(os.path.join(src, s_rel))
        t = strip_module_paths(t, [mod], s_rel)
        if re.search(r"// Changes: none \(verbatim\); this header added\.", t):
            t = re.sub(r"// Changes: none \(verbatim\); this header added\.",
                       f"// Changes: moved from springrep's src/{s_rel} to {krate} (M3 W4); paths and visibility only.", t, count=1)
        else:
            # the Changes block may run over several `// ` lines: note the move at its end
            t = re.sub(r"(// Changes: [^\n]*(?:\n// [^\n]*)*?)\.?\n(?!// )",
                       rf"\1; then to {krate} (M3 W4), paths and visibility only.\n", t, count=1)
            t = t.replace(f".; then to {krate} (M3 W4)", f"; then to {krate} (M3 W4)", 1)
        if d_rel in panics:
            t = counted_panics(t, panics[d_rel], s_rel)
            t = re.sub(r"(// Changes: [^\n]*?)(; paths and visibility only\.|\.)\n",
                       r"\1; `.expect`/`.unwrap` as counted `panic!`s (gate 9).\n", t, count=1)
        moved[d_rel] = t
    # names the moved code reaches at the crate root that are not its own modules come from D26's crates
    own = set(re.findall(r"(?m)^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", moved["lib.rs"]))
    reached = crate_names(moved.values()) - own - {"names"} if mod == "topo" else crate_names(moved.values()) - own
    unknown = reached - set(ALIASES) - set(re.findall(r"(?m)^pub use \w+::\{?([\w, ]+)", moved["lib.rs"]))
    lib = moved["lib.rs"]
    lines = lib.split("\n")
    # inner attributes go before the first item: after the header comments and the module doc
    k = next(i for i, l in enumerate(lines) if l.strip() and not l.startswith("//"))
    head = ["", f"// springrep's {mod} indexes by position; D26's hardening pass moves it onto checked access,",
            "// as for yack-linalg until M5.", "#![allow(clippy::indexing_slicing)]", "",
            f"// springrep's foundation modules (D26), under the names {mod}'s code uses."]
    head += [ALIASES[n] for n in sorted(reached & set(ALIASES))]
    # the crate root's own `use crate::{..}` named these when they were springrep's modules; at the
    # root they are the aliases now, and importing them again would define them twice
    def drop_root_aliases(u):
        body = u.group(1)
        parts, depth, cur = [], 0, ""
        for c in body:
            depth += c == "{"
            depth -= c == "}"
            if c == "," and depth == 0:
                parts.append(cur)
                cur = ""
            else:
                cur += c
        parts.append(cur)
        keep = [p for p in parts if p.strip() and p.strip() not in ALIASES]
        return "use crate::{" + ",".join(keep) + (",\n" if body.rstrip().endswith(",") else "") + "};" if keep else ""
    lines = [l for l in "\n".join(lines).split("\n")]
    lib_rest = re.sub(r"use crate::\{([^;]*)\};", drop_root_aliases, "\n".join(lines[k:]))
    lines = lines[:k] + lib_rest.split("\n")
    if mod == "topo":
        head += ["", "pub mod names;"]
    moved["lib.rs"] = "\n".join(lines[:k] + head + lines[k:])
    staged.append((mod, krate, files, panics, deps, moved, unknown))

# `pub(crate)` becomes `pub` only where code outside the crate names the item (rule 4): springrep's
# live code that stays, the facade and bridge crates, and the other new crates
REAL = "/home/brenn/github/yack/crates"
moving = {os.path.join(src, s_rel) for _, _, files, _, _, _, _ in staged for s_rel in files}
outside = [live_text(rd(p)) for p in rs_files(src) if p not in moving
           and os.path.basename(p) != "tests.rs" and "/tests/" not in p]
for c in ["yack", "yack-bridge", "yack-step", "yack-cadrs"]:
    d = os.path.join(crates, c, "src") if os.path.isdir(os.path.join(crates, c, "src")) else os.path.join(REAL, c, "src")
    outside += [live_text(rd(p)) for p in rs_files(d) if os.path.basename(p) != "tests.rs"]
texts, made_pub = precise_pub({krate: moved for _, krate, _, _, _, moved, _ in staged}, "\n".join(outside))
for mod, krate, files, panics, deps, moved, unknown in staged:
    N = os.path.join(crates, krate, "src")
    moved = texts[krate]
    for d_rel, t in moved.items():
        if d_rel in ("memo.rs", "walks.rs") and mod == "topo":
            t = allow_items(t, r"\bHash(Map|Set)\b", "clippy::disallowed_types",
                            "D26: BTree* in hardening, proven by digests", d_rel)
        wr(os.path.join(N, d_rel), t)
    for s_rel in files:
        os.remove(os.path.join(src, s_rel))
    for d in [os.path.join(src, mod)]:
        if os.path.isdir(d):
            for a, ds, _ in sorted(os.walk(d), reverse=True):
                os.rmdir(a)
    wr(os.path.join(crates, krate, "Cargo.toml"), cargo(krate, deps))
    e, pnc = count([N]), count(["--panics", N])
    want = sum(n * b.count("panic!(") for _, b, n in sum(panics.values(), []))
    if pnc != want:
        raise SystemExit(f"{krate}: {pnc} contract panics, expected {want}")
    eps_total += e
    rows.append((krate, e, pnc, sum(len(rd(p).split("\n")) - 1 for p in rs_files(N))))
    if unknown:
        print(f"note: {krate} reaches crate-root names {sorted(unknown)} (its own re-exports?)")

# ---- 2. springrep: re-exports, numeric's re-export list, dependencies ----
lp = os.path.join(src, "lib.rs")
l = rd(lp)
l = sub1(l, "pub mod names;\n", "pub use yack_topo::names;\n", lp)
l = sub1(l, "pub mod topo;\n", "pub use yack_topo as topo;\n", lp)
l = sub1(l, "pub mod profile;\n", "pub use yack_profile as profile;\n", lp)
l = sub1(l, "pub(crate) mod intersect;\n", "pub(crate) use yack_intersect as intersect;\n", lp)
wr(lp, l)
kept = trim_numeric(src)
springrep_deps(ws, root, ["yack-topo", "yack-profile", "yack-intersect"])

# ---- 3. layers.toml: yack-profile above yack-topo; module maps for the crates over 5k lines ----
lt = os.path.join(ws, "layers.toml")
s = rd(lt)
m = re.search(r'(?ms)^\[crate\.yack-profile\]\nlayer = 4\n(.*?)^deps = \[([^\]]*)\]', s)
if not m:
    raise SystemExit("layers.toml: yack-profile is not at layer 4")
pd = [x.strip().strip('"') for x in m.group(2).split(",") if x.strip()]
s = s[:m.start()] + "[crate.yack-profile]\nlayer = 5\n" + m.group(1) + "deps = [" + ", ".join(
    f'"{d}"' for d in ["yack-topo"] + pd) + "]" + s[m.end():]
for krate, _, _, lines_ in rows:
    if lines_ > 5000 and f"[modules.{krate}]" not in s:
        mp = unit_edges(os.path.join(crates, krate, "src"))
        block = f"[modules.{krate}]\n" + "".join(f"{u} = [{', '.join(chr(34) + v + chr(34) for v in vs)}]\n" for u, vs in mp) + "\n"
        s = sub1(s, "[modules.yack-springrep]\n", block + "[modules.yack-springrep]\n", lt)
wr(lt, s)

# ---- 4. scores ----
sd = os.path.join(ws, "scores")
first = True
for krate, e, pnc, _ in rows:
    set_row(os.path.join(sd, "eps.txt"), krate, e, ("yack-springrep", eps_total) if first else None)
    first = False
    if pnc:
        set_row(os.path.join(sd, "lints.txt"), krate, pnc)
set_row(os.path.join(sd, "hidden.txt"), "yack-topo", 1, ("yack-springrep", 1))
fp = os.path.join(sd, "filesize.txt")
f = rd(fp)
f = re.sub(r"(?m)^crates/yack-springrep/src/intersect/(\S+) ", r"crates/yack-intersect/src/\1 ", f)
f = re.sub(r"(?m)^crates/yack-springrep/src/topo/(\S+) ", r"crates/yack-topo/src/\1 ", f)
f = re.sub(r"(?m)^crates/yack-springrep/src/profile\.rs ", r"crates/yack-profile/src/lib.rs ", f)
wr(fp, f)
for a, b, t, h in moved_tests:
    print(f"tests {a} -> {b}: {t} (helpers {h})")
print("rows:", [(k, e, p) for k, e, p, _ in rows], "springrep numeric re-exports:", kept)
for k, v in made_pub.items():
    print(f"{k}: pub(crate) -> pub for {len(v)} names: {' '.join(v)}")
