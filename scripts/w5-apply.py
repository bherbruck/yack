# M3 W5: springrep's locate/ (with rays.rs and distance.rs, PLAN §3), measure and codec become
# yack-locate, yack-measure and yack-codec, re-exported by springrep under their old names. As in W4:
# precise pub (rule 4), counted panics, per-item allows, the eps/lints/hidden/filesize rows, and
# tests that build their operands above the new crate move verbatim into springrep (loft/tests.rs,
# beside W4's). Lead's rulings for W5: the Gauss rules (legendre_rule, gauss, gauss_fixed, integrate,
# adapt) join yack-numeric verbatim, their OnceLock cache under a hidden.txt row (D26 note), since
# measure calls them; codec's moved tests reach the codec through its public to_bytes/from_bytes,
# their `bytes`/`back` helpers rewritten over those, the test bodies verbatim.
# Usage: python3 w5-apply.py <yack-springrep dir> [--with-external ...]
import os, re, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from m3_move import (OPTION_UNWRAP, allow_items, cargo, count, counted_panics, crate_names, items, live_text,
                     precise_pub, rd, rs_files, set_row, split_tests, springrep_deps, strip_module_paths, sub1,
                     wr)

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)
OU = OPTION_UNWRAP
RU = '.unwrap_or_else(|e| panic!("called `Result::unwrap()` on an `Err` value: {e:?}"))'


def rex(msg):
    return f'.unwrap_or_else(|e| panic!("{msg}: {{e:?}}"))'


def oex(msg):
    return f'.unwrap_or_else(|| panic!("{msg}"))'


HUB = os.path.join(src, "loft", "tests.rs")  # W4's home for moved tests
moved_tests = []

# ---- 0a. the Gauss rules join yack-numeric (measure calls them) ----
np_ = os.path.join(src, "numeric.rs")
nt = rd(np_).split("\n")
cut = nt.index("#[cfg(test)]")
body, tests = nt[:cut], nt[cut:]
spans = {n: (j, e) for n, j, e in items(body)}
GAUSS = ["legendre_rule", "gauss", "gauss_fixed", "integrate", "adapt"]
if any(n not in spans for n in GAUSS):
    raise SystemExit(f"numeric.rs: missing {[n for n in GAUSS if n not in spans]}")
gauss_text = "\n\n".join("\n".join(body[spans[n][0]:spans[n][1]]) for n in GAUSS)
# measure (another crate now) calls gauss and integrate: those two are pub; the rest stay crate-private
gauss_text = re.sub(r"pub\(crate\) fn (gauss|integrate)\(", r"pub fn \1(", gauss_text)
gauss_text = gauss_text.replace("    static RULES:", "    // springrep's rule cache (D26): hardening computes the rules per call, measured first.\n    static RULES:", 1)
while tests and not tests[-1].strip():
    tests.pop()
if tests[:2] != ["#[cfg(test)]", "mod tests {"] or tests[-1] != "}":
    raise SystemExit("numeric.rs: its tests module is not the shape expected")
tinner = tests[2:-1]
tspans = {n: (j, e) for n, j, e in items(tinner, "    ")}
gtests = "\n\n".join("\n".join(tinner[j:e]) for n, (j, e) in tspans.items()
                     if n in ("gauss_rules_integrate_polynomials_exactly", "adaptive_quadrature_handles_a_kink"))
if gtests.count("#[test]") != 2:
    raise SystemExit("numeric.rs: the two Gauss tests not found")
yn = os.path.join(crates, "yack-numeric", "src", "lib.rs")
y = rd(yn)
y = sub1(y, "\nuse yack_base::kernel::", "\nuse std::sync::OnceLock;\n\nuse yack_base::kernel::", yn)
y = sub1(y, "\n#[cfg(test)]\nmod tests {\n    use super::*;\n",
         "\n// The Gauss-Legendre rules and their quadrature, from springrep's numeric.rs (M3 W5).\n"
         + gauss_text + "\n\n#[cfg(test)]\nmod tests {\n    use super::*;\n\n" + gtests + "\n", yn)
y = re.sub(r"(// Changes: [^\n]*(?:\n// [^\n]*)*?)\.?\n(?!// )", r"\1; the Gauss rules joined in M3 W5.\n", y, count=1)
wr(yn, y)
os.remove(np_)
lp = os.path.join(src, "lib.rs")
wr(lp, sub1(rd(lp), "mod numeric;\n", "use yack_numeric as numeric;\n", lp))

# ---- 0b. tests that build above their new crate move to the hub ----
FORBID = {"primitive", "sweep", "loft", "boolean", "blend", "mesh", "facade", "block", "cylinder", "sphere",
          "extrude", "revolve", "union", "difference", "intersection", "Section", "cone", "torus", "Profile",
          "fillet", "measure"}
# every moving file with an inline tests module that reaches above its new crate (found, not listed)
SPLIT = []
for rel, mp, at, crate_ in [
    ("measure.rs", ["crate", "measure"], None, "yack-measure"),
    ("measure/sample.rs", ["crate", "measure", "sample"], ["crate", "measure"], "yack-measure"),
    ("measure/tables.rs", ["crate", "measure", "tables"], ["crate", "measure"], "yack-measure"),
    ("measure/tolerant.rs", ["crate", "measure", "tolerant"], ["crate", "measure"], "yack-measure"),
    ("locate/mod.rs", ["crate", "locate"], None, "yack-locate"),
    ("locate/classify.rs", ["crate", "locate", "classify"], None, "yack-locate"),
    ("rays.rs", ["crate", "rays"], None, "yack-locate"),
    ("distance.rs", ["crate", "distance"], None, "yack-locate"),
    ("codec/topo.rs", ["crate", "codec", "topo"], ["crate", "codec"], "yack-codec"),
]:
    t = rd(os.path.join(src, rel))
    k = t.find("#[cfg(test)]\nmod tests {\n")
    if k < 0:
        continue
    reach = set(re.findall(r"\bcrate::(\w+)", t[k:])) | set(re.findall(r"(?<![\w:])(\w+)::", t[k:]))
    for u in re.findall(r"\buse crate::\{([^;]*)\};", t[k:]):
        reach |= set(re.findall(r"(?:^|[{,])\s*(\w+)", u))
    forbid_mods = FORBID - ({"measure"} if rel.startswith("measure") else set())
    if reach & forbid_mods & {"primitive", "sweep", "loft", "boolean", "blend", "mesh", "facade", "measure"}:
        name = rel.replace("/", "_").replace(".rs", "").replace("_mod", "") + "_moved"
        SPLIT.append((rel, mp, name, f"Moved from {rel}'s tests (M3 W5): they build with modules above {crate_}.", at))
for rel, mp, name, doc, at in SPLIT:
    at = [at] if at else []
    forbid = FORBID - ({"measure"} if rel.startswith("measure") else set())
    mod, tests_, helpers = split_tests(os.path.join(src, rel), mp, forbid, name, doc, own_at=at[0] if at else None)
    wr(HUB, rd(HUB).rstrip("\n") + "\n\n" + mod)
    moved_tests.append((rel, tests_, helpers))

# codec/tests.rs is a file of its own: wrap it as codec.rs's `mod tests { .. }` to split, then unwrap
cp, ctp = os.path.join(src, "codec.rs"), os.path.join(src, "codec", "tests.rs")
codec_src, ctests = rd(cp), rd(ctp)
hdr_end = ctests.index("\nuse ")
chead, cbody = ctests[:hdr_end + 1], ctests[hdr_end + 1:]
tmp = cp + ".w5"
wr(tmp, codec_src.replace("#[cfg(test)]\nmod tests;\n", "") + "\n#[cfg(test)]\nmod tests {\n"
   + "\n".join(("    " + l) if l.strip() else "" for l in cbody.rstrip("\n").split("\n")) + "\n}\n")
SOLIDS = '''    /// The solids' bytes through the codec's public path (yack-codec keeps `Codec` crate-private).
    fn bytes<T: Solids>(x: &T) -> Vec<u8> {
        crate::codec::to_bytes(&x.solids())
    }

    fn back<T: Solids>(b: &[u8]) -> Option<T> {
        crate::codec::from_bytes(b).and_then(T::from_solids)
    }

    /// A solid, or a list of solids, as `to_bytes`/`from_bytes` carry them.
    trait Solids: Sized {
        fn solids(&self) -> Vec<Brep>;
        fn from_solids(v: Vec<Brep>) -> Option<Self>;
    }

    impl Solids for Brep {
        fn solids(&self) -> Vec<Brep> {
            vec![self.clone()]
        }
        fn from_solids(mut v: Vec<Brep>) -> Option<Brep> {
            (v.len() == 1).then(|| v.remove(0))
        }
    }

    impl Solids for Vec<Brep> {
        fn solids(&self) -> Vec<Brep> {
            self.clone()
        }
        fn from_solids(v: Vec<Brep>) -> Option<Vec<Brep>> {
            Some(v)
        }
    }

'''


def codec_public(t):
    for name in ("bytes", "back"):
        t, n = re.subn(r"(?ms)^(\s*)fn " + name + r"<T: Codec>.*?^\1}\n\n?", "", t)
        if n != 1:
            raise SystemExit(f"codec tests: the Codec-based `{name}` helper not found")
    return t.replace("T: Codec +", "T: Solids +")


mod, tests_, helpers = split_tests(tmp, ["crate", "codec"], FORBID - {"measure"}, "codec_moved",
                                   "Moved from codec/tests.rs (M3 W5): they encode built solids (sweep, boolean, "
                                   "blend), above yack-codec; they reach the codec through to_bytes/from_bytes.",
                                   transform=codec_public, preamble=SOLIDS)
mod = mod.replace("    use crate::codec::Codec;\n", "")
wr(HUB, rd(HUB).rstrip("\n") + "\n\n" + mod)
moved_tests.append(("codec/tests.rs", tests_, helpers))
left = rd(tmp)
os.remove(tmp)
k = left.index("#[cfg(test)]\nmod tests {\n")
inner = left[k + len("#[cfg(test)]\nmod tests {\n"):].rstrip("\n")
inner = inner[:inner.rstrip().rfind("}")]
wr(ctp, chead + "\n".join(l[4:] if l.startswith("    ") else l for l in inner.rstrip("\n").split("\n")) + "\n")

# boolean/tests.rs's point-contact round trip wrote and read the arena through the codec's internals
# (`Codec`, `Writer`, `Reader`), which stay crate-private in yack-codec: it takes the public path
bp = os.path.join(src, "boolean", "tests.rs")
b_ = rd(bp)
b_ = sub1(b_, "    use crate::codec::{Codec, Reader, Writer};\n", "", bp)
b_ = sub1(b_, """    let mut w = Writer::default();
    u.put(&mut w);
    let back = Brep::get(&mut Reader::new(&w.0)).unwrap();
""", """    // Through the codec's public path (M3 W5: yack-codec keeps `Codec` crate-private).
    let back = crate::codec::from_bytes(&crate::codec::to_bytes(std::slice::from_ref(&u)))
        .and_then(|mut v| v.pop())
        .unwrap();
""", bp)
wr(bp, b_)

# ---- 1. the modules ----
SPECS = [
    ("locate", "yack-locate",
     {"locate/mod.rs": "lib.rs", "locate/classify.rs": "classify.rs", "rays.rs": "rays.rs", "distance.rs": "distance.rs"},
     {"rays.rs": [('f.expect("every mesh triangle lies in its face\'s range")', "f" + oex("every mesh triangle lies in its face's range"), 1)]},
     ["yack-base", "yack-geom", "yack-intersect", "yack-math", "yack-numeric", "yack-tol", "yack-topo"],
     ["pub mod distance;", "pub mod rays;"]),
    ("measure", "yack-measure",
     {"measure.rs": "lib.rs", "measure/sample.rs": "sample.rs", "measure/tables.rs": "tables.rs",
      "measure/tolerant.rs": "tolerant.rs"},
     {"lib.rs": [(".lock().unwrap()", ".lock()" + RU, 4),
                 ('.expect("projected pcurves have curves")', oex("projected pcurves have curves"), 1)]},
     ["yack-base", "yack-geom", "yack-intersect", "yack-math", "yack-numeric", "yack-tol", "yack-topo"], []),
    ("codec", "yack-codec",
     {"codec.rs": "lib.rs", "codec/topo.rs": "topo.rs", "codec/tests.rs": "tests.rs"},
     {}, ["yack-base", "yack-geom", "yack-math", "yack-topo"], []),
]
ALIASES = {
    "error": "use yack_base::kernel as error;", "Error": "use yack_base::kernel::Error;",
    "Result": "use yack_base::kernel::Result;", "geom": "use yack_geom as geom;",
    "math": "use yack_math::kernel as math;", "numeric": "use yack_numeric as numeric;",
    "tol": "use yack_tol as tol;", "topo": "use yack_topo as topo;", "intersect": "use yack_intersect as intersect;",
    "names": "use yack_topo::names;",
}
staged = []
for mod, krate, files, panics, deps, extra in SPECS:
    moved = {}
    for s_rel, d_rel in files.items():
        t = rd(os.path.join(src, s_rel))
        if mod == "codec":
            # codec has a module of its own named `topo` (the arena's codec): springrep's topo is
            # yack_topo by its crate name there
            t = re.sub(r"\buse crate::topo::", "use yack_topo::", t)
            t = re.sub(r"(?m)^(\s+)topo::(\w+),\n", r"", t) if s_rel == "codec.rs" and "    topo::Brep,\n" in t else t
            if s_rel == "codec.rs":
                t = sub1(t, "\nuse crate::{", "\nuse yack_topo::Brep;\nuse crate::{", s_rel)
        t = strip_module_paths(t, [mod], s_rel)
        if re.search(r"// Changes: none \(verbatim\); this header added\.", t):
            t = re.sub(r"// Changes: none \(verbatim\); this header added\.",
                       f"// Changes: moved from springrep's src/{s_rel} to {krate} (M3 W5); paths and visibility only.", t, count=1)
        elif "// Changes:" in t[:600]:
            t = re.sub(r"(// Changes: [^\n]*(?:\n// [^\n]*)*?)\.?\n(?!// )",
                       rf"\1; then to {krate} (M3 W5), paths and visibility only.\n", t, count=1)
            t = t.replace(f".; then to {krate} (M3 W5)", f"; then to {krate} (M3 W5)", 1)
        if d_rel in panics:
            t = counted_panics(t, panics[d_rel], s_rel)
        moved[d_rel] = t
    own = set(re.findall(r"(?m)^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+(\w+)\s*;", moved["lib.rs"]))
    own |= {"rays", "distance"} if mod == "locate" else set()
    reached = crate_names([v for k2, v in moved.items() if k2 != "tests.rs"]) - own
    lines = moved["lib.rs"].split("\n")
    k = next(i for i, l in enumerate(lines) if l.strip() and not l.startswith("//"))
    head = ["", f"// springrep's {mod} indexes by position; D26's hardening pass moves it onto checked access,",
            "// as for yack-linalg until M5.", "#![allow(clippy::indexing_slicing)]", "",
            f"// springrep's foundation modules (D26), under the names {mod}'s code uses."]
    head += [ALIASES[n] for n in sorted(reached & set(ALIASES))] + ([""] + extra if extra else [])

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
    rest = re.sub(r"use crate::\{([^;]*)\};", drop_root_aliases, "\n".join(lines[k:]))
    moved["lib.rs"] = "\n".join(lines[:k] + head) + "\n" + rest
    staged.append((mod, krate, files, panics, deps, moved))

REAL = "/home/brenn/github/yack/crates"
moving = {os.path.join(src, s) for _, _, files, _, _, _ in staged for s in files}
outside = [live_text(rd(p)) for p in rs_files(src) if p not in moving and os.path.basename(p) != "tests.rs"
           and "/tests/" not in p]
for c in ["yack", "yack-bridge", "yack-step", "yack-cadrs", "yack-topo", "yack-intersect", "yack-profile"]:
    d = os.path.join(crates, c, "src") if os.path.isdir(os.path.join(crates, c, "src")) else os.path.join(REAL, c, "src")
    outside += [live_text(rd(p)) for p in rs_files(d) if os.path.basename(p) != "tests.rs"]
texts, made_pub = precise_pub({krate: moved for _, krate, _, _, _, moved in staged}, "\n".join(outside))
eps_total, rows = 0, []
for mod, krate, files, panics, deps, _ in staged:
    N = os.path.join(crates, krate, "src")
    for d_rel, t in texts[krate].items():
        if mod == "measure" and d_rel == "lib.rs":
            t = allow_items(t, r"\bHash(Map|Set)\b", "clippy::disallowed_types",
                            "D26: BTree* in hardening, proven by digests", d_rel)
        wr(os.path.join(N, d_rel), t)
    for s_rel in files:
        os.remove(os.path.join(src, s_rel))
    for d in [os.path.join(src, mod)]:
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
l = rd(lp)
l = sub1(l, "pub mod codec;\n", "pub use yack_codec as codec;\n", lp)
l = sub1(l, "pub mod distance;\n", "pub use yack_locate::distance;\n", lp)
l = sub1(l, "pub(crate) mod locate;\n", "pub(crate) use yack_locate as locate;\n", lp)
l = sub1(l, "pub mod measure;\n", "pub use yack_measure as measure;\n", lp)
l = sub1(l, "pub mod rays;\n", "pub use yack_locate::rays;\n", lp)
wr(lp, l)
springrep_deps(ws, root, ["yack-locate", "yack-measure", "yack-codec"])

# ---- 3. scores ----
sd = os.path.join(ws, "scores")
first = True
for krate, e, pnc, _ in rows:
    set_row(os.path.join(sd, "eps.txt"), krate, e, ("yack-springrep", eps_total) if first else None)
    first = False
    if pnc:
        set_row(os.path.join(sd, "lints.txt"), krate, pnc)
gauss_eps = count([yn]) * 0  # yack-numeric is an eps home: its literals carry no row
set_row(os.path.join(sd, "hidden.txt"), "yack-numeric", 1, ("yack-springrep", 1))
fp = os.path.join(sd, "filesize.txt")
f = rd(fp)
f = re.sub(r"(?m)^crates/yack-springrep/src/measure\.rs ", "crates/yack-measure/src/lib.rs ", f)
f = re.sub(r"(?m)^crates/yack-springrep/src/(codec|locate)/(\S+) ", r"crates/yack-\1/src/\2 ", f)
wr(fp, f)
lt = os.path.join(ws, "layers.toml")
for krate, _, _, n in rows:
    if n > 5000:
        raise SystemExit(f"{krate} is over 5k lines: it needs a module map")
for a, t, h in moved_tests:
    print(f"tests {a} -> loft/tests.rs: {t} (helpers {h})")
print("rows:", [(k, e, p) for k, e, p, _ in rows])
for k, v in made_pub.items():
    print(f"{k}: pub(crate) -> pub for {len(v)} names: {' '.join(v)}")
