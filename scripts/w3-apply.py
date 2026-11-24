# M3 W3: springrep's geom/ becomes the crate yack-geom (git-mv-shaped: same files, same text), with
# springrep re-exporting it as `geom`. Changes beyond paths and visibility, all lead-ruled:
#   - geom's foundation modules come from their crates (D26) under the names its code uses, through
#     private aliases at the crate root (`error`, `math`, `numeric`, `tol`, `Error`, `Result`);
#   - its seven `.expect(msg)` become `.unwrap_or_else(|| panic!(msg))`, which panics the same way,
#     under a fn-level `#[allow(clippy::panic)]`: counted contract panics (scores/lints.txt row);
#   - the cfg(test) helper `octant`, which springrep tests call, is copied into each of them;
#   - scores: eps.txt and lints.txt rows for yack-geom, springrep's eps row lowered by the same
#     amount, and nurbs.rs's filesize.txt row renamed.
# Usage: python3 w3-apply.py <yack-springrep dir> [--with-external ...]
import os, re, subprocess, sys

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)
here = os.path.dirname(os.path.abspath(__file__))
G = os.path.join(src, "geom")
N = os.path.join(crates, "yack-geom")


def rd(p):
    return open(p).read()


def wr(p, s):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p, "w").write(s)


def sub1(s, a, b, f):
    if s.count(a) != 1:
        raise SystemExit(f"{f}: expected one {a[:80]!r}, found {s.count(a)}")
    return s.replace(a, b)


def count(args):
    return int(subprocess.check_output([sys.executable, os.path.join(here, "eps_count.py")] + args).decode())


files = sorted(os.path.relpath(os.path.join(d, f), G) for d, _, fl in os.walk(G) for f in fl if f.endswith(".rs"))
if files != ["bspline.rs", "chord.rs", "curve.rs", "intersection.rs", "mod.rs", "nurbs.rs", "nurbs/seeds.rs", "surface.rs"]:
    raise SystemExit(f"geom/ holds {files}")
moved_eps = count([G])

# the seven expects: (file, exact text, replacement)
EXPECTS = [
    ("intersection.rs", '*out.last_mut().expect("checked") = n;', '*out.last_mut().unwrap_or_else(|| panic!("checked")) = n;'),
    ("nurbs.rs", '.expect("a row is left");', '.unwrap_or_else(|| panic!("a row is left"));'),
    ("nurbs.rs", 'self.held.expect("a polynomial iso line");', 'self.held.unwrap_or_else(|| panic!("a polynomial iso line"));'),
    ("nurbs.rs", '.expect("a row of a valid net is a valid curve")', '.unwrap_or_else(|_| panic!("a row of a valid net is a valid curve"))'),
    ("bspline.rs", '.expect("a span holds its parameter")[end];', '.unwrap_or_else(|| panic!("a span holds its parameter"))[end];'),
    ("curve.rs", 'self.spline().expect("checked above").extremes(dir, t0, t1)', 'self.spline().unwrap_or_else(|| panic!("checked above")).extremes(dir, t0, t1)'),
    ("surface.rs", 'let spline = s.curve.spline().expect("checked above");', 'let spline = s.curve.spline().unwrap_or_else(|| panic!("checked above"));'),
]


def allow_panic_at(lines, k):
    """Puts `#[allow(clippy::panic)]` on the fn enclosing line k (a counted contract panic), unless it has one."""
    ind = len(lines[k]) - len(lines[k].lstrip())
    for j in range(k, -1, -1):
        l = lines[j]
        li = len(l) - len(l.lstrip())
        if li < ind and re.match(r"\s*(pub(\([^)]*\))?\s+)?(const\s+)?fn\s", l):
            # on the fn's own line, so that no file grows (nurbs.rs sits at its filesize.txt allowance)
            if "#[allow(clippy::panic)]" not in l:
                lines[j] = " " * li + "#[allow(clippy::panic)] " + l.lstrip()
            return
    raise SystemExit(f"no enclosing fn for line {k}")


texts = {f: rd(os.path.join(G, f)) for f in files}
for f, a, b in EXPECTS:
    texts[f] = sub1(texts[f], a, b, f)
for f in set(f for f, _, _ in EXPECTS):
    lines = texts[f].split("\n")
    sites = [k for k, l in enumerate(lines) if "unwrap_or_else(|| panic!(" in l or "unwrap_or_else(|_| panic!(" in l]
    for k in sorted(sites, reverse=True):
        allow_panic_at(lines, k)
    texts[f] = "\n".join(lines)
if any(".expect(" in re.split(r"#\[cfg\(test\)\]\nmod tests", t)[0] for t in texts.values()):
    raise SystemExit("geom: an .expect( is left in live code")

for f in files:
    t = texts[f]
    t = re.sub(r"\bcrate::geom::", "crate::", t)
    # a `geom::{..}` entry inside a `use crate::{..}` group (a test module's) names the crate root now
    t = re.sub(r"(?<!::)\bgeom::\{([^}]*)\},", r"\1,", t)
    if re.search(r"\bgeom::|\bcrate::geom\b", t):
        raise SystemExit(f"{f}: still names geom:: after the rewrite")
    t = re.sub(r"\bpub\(crate\)", "pub", t)
    old = "src/geom/" + f
    t = re.sub(r"// Changes: none \(verbatim\); this header added\.",
               f"// Changes: moved from springrep's {old} to yack-geom (M3 W3); paths and visibility only.", t, count=1)
    if f in {x for x, _, _ in EXPECTS}:
        t = re.sub(r"(// Changes: [^\n]*?)( paths and visibility only\.|\.)\n",
                   r"\1; `.expect` as a counted `panic!` (gate 9).\n", t, count=1)
    if f == "mod.rs":
        t = sub1(t, "\nmod bspline;\n",
                 "\n// springrep's geometry indexes by position; D26's hardening pass moves it onto checked\n"
                 "// access, as for yack-linalg until M5.\n#![allow(clippy::indexing_slicing)]\n\n"
                 "// springrep's foundation modules (D26), under the names geom's code uses.\n"
                 "use yack_base::kernel::{self as error, Error, Result};\n"
                 "use yack_math::kernel as math;\n"
                 "use yack_numeric as numeric;\n"
                 "use yack_tol as tol;\n\nmod bspline;\n", f)
        wr(os.path.join(N, "src", "lib.rs"), t)
    else:
        wr(os.path.join(N, "src", f), t)
for f in files:
    os.remove(os.path.join(G, f))
os.rmdir(os.path.join(G, "nurbs"))
os.rmdir(G)
wr(os.path.join(N, "Cargo.toml"),
   '[package]\nname = "yack-geom"\ndescription = "yack internal crate (not published; use the `yack` crate)"\n'
   'version.workspace = true\nedition.workspace = true\nrust-version.workspace = true\nlicense.workspace = true\n'
   'repository.workspace = true\npublish = false\n\n[dependencies]\n'
   'yack-base = { path = "../yack-base" }\nyack-math = { path = "../yack-math" }\n'
   'yack-numeric = { path = "../yack-numeric" }\nyack-tol = { path = "../yack-tol" }\n\n[lints]\nworkspace = true\n')
panics = count(["--panics", os.path.join(N, "src")])
if panics != 7:
    raise SystemExit(f"yack-geom has {panics} contract panics, expected 7")

# the cfg(test) helper `octant`, which another crate's tests cannot see: each springrep test that
# calls it gets its own copy (lead's ruling), placed after an anchor in its test code
def octant_copy(ind):
    body = [
        "/// The octant of the sphere of radius `r` about the origin, as the rational biquadratic of",
        "/// Piegl and Tiller. A copy of yack-geom's test helper, whose cfg(test) items another crate's",
        "/// tests cannot see (M3 W3).",
        "fn octant(r: f64) -> crate::geom::Surface {",
        "    let w = std::f64::consts::FRAC_1_SQRT_2;",
        "    let equator = [(r, 0.0, 1.0), (r, r, w), (0.0, r, 1.0)];",
        "    let mut ctrl = Vec::new();",
        "    let mut weights = Vec::new();",
        "    for &(x, y, wu) in &equator {",
        "        for (j, wv) in [1.0, w, 1.0].into_iter().enumerate() {",
        "            ctrl.push(match j {",
        "                0 => crate::math::Vec3::new(x, y, 0.0),",
        "                1 => crate::math::Vec3::new(x, y, r),",
        "                _ => crate::math::Vec3::new(0.0, 0.0, r),",
        "            });",
        "            weights.push(wu * wv);",
        "        }",
        "    }",
        "    let k = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];",
        "    let n = crate::geom::NurbsSurface::new((2, 2), (k.clone(), k), (3, 3), ctrl, Some(weights)).unwrap();",
        "    crate::geom::Surface::Nurbs(Box::new(n))",
        "}",
    ]
    return "".join(ind + l + "\n" for l in body)


OCTANT_USERS = {
    os.path.join("intersect", "contact.rs"): ("    use crate::math::Frame;\n", "    "),
    os.path.join("tweak", "tests.rs"): ("use crate::{measure, profile::Profile, sweep, topo::FaceName};\n", ""),
}
callers = sorted(os.path.relpath(os.path.join(d, f), src) for d, _, fl in os.walk(src) for f in fl
                 if f.endswith(".rs") and "crate::geom::octant(" in rd(os.path.join(d, f)))
if callers != sorted(OCTANT_USERS):
    raise SystemExit(f"geom::octant callers are {callers}, the script copies into {sorted(OCTANT_USERS)}")
for rel, (anchor, ind) in OCTANT_USERS.items():
    p = os.path.join(src, rel)
    c = rd(p)
    c = sub1(c, anchor, anchor + "\n" + octant_copy(ind), p)
    c = c.replace("crate::geom::octant(", "octant(")
    wr(p, c)
for d, _, fl in list(os.walk(src)) + list(os.walk(os.path.join(root, "tests"))):
    for f in fl:
        if f.endswith(".rs") and re.search(r"\bgeom::octant\b", rd(os.path.join(d, f))):
            raise SystemExit(f"{os.path.join(d, f)} still names geom::octant")

# springrep's numeric.rs re-exports (W1') name what its callers used; with geom gone, keep only the
# names springrep still calls, or the unused ones warn (and raise the clippy ratchet)
called = set()
for d, _, fl in os.walk(src):
    for f in fl:
        p = os.path.join(d, f)
        if f.endswith(".rs") and p != os.path.join(src, "numeric.rs"):
            for m in re.finditer(r"numeric::(\{[^}]*\}|\w+)", rd(p)):
                called |= {x.strip() for x in m.group(1).strip("{}").split(",") if x.strip()}
np_ = os.path.join(src, "numeric.rs")
nt = rd(np_)
m = re.search(r"pub\(crate\) use yack_numeric::\{([^}]*)\};", nt)
if not m:
    raise SystemExit("numeric.rs: no yack_numeric re-export")
names = [x.strip() for x in m.group(1).split(",") if x.strip()]
kept = [n for n in names if n in called]
if not kept:
    raise SystemExit("numeric.rs: springrep would call nothing of yack_numeric")
nt = nt.replace(m.group(0), "pub(crate) use yack_numeric::{" + ", ".join(kept) + "};")
wr(np_, nt)
print("numeric re-exports kept:", kept, "dropped:", [n for n in names if n not in kept])

# springrep: the re-export and the dependency
lp = os.path.join(src, "lib.rs")
wr(lp, sub1(rd(lp), "pub mod geom;\n", "pub use yack_geom as geom;\n", lp))
sc = os.path.join(root, "Cargo.toml")
c = rd(sc)
if "yack-geom = " not in c:
    c = sub1(c, "[dependencies]\n", '[dependencies]\nyack-geom = { path = "../yack-geom" }\n', sc)
wr(sc, c)
lt = os.path.join(ws, "layers.toml")
s = rd(lt)
mm = re.search(r'(?ms)^\[crate\.yack-springrep\]\n.*?^deps = \[([^\]]*)\]', s)
have = [x.strip().strip('"') for x in mm.group(1).split(",") if x.strip()]
if "yack-geom" not in have:
    s = s[:mm.start(1)] + ", ".join(f'"{d}"' for d in have + ["yack-geom"]) + s[mm.end(1):]
# gate 5: yack-geom is over 5k lines, so it needs a module map: its top-level modules, lowest first,
# with their (non-test) edges, transitively closed as springrep's rows are
if "[modules.yack-geom]" not in s:
    s = sub1(s, "[modules.yack-springrep]\n",
             "[modules.yack-geom]\nchord = []\ncurve = []\nnurbs = []\nsurface = []\n"
             'bspline = ["nurbs"]\nintersection = ["nurbs", "bspline"]\n\n[modules.yack-springrep]\n', lt)
wr(lt, s)


def set_row(path, crate, value, lower=None):
    e = rd(path)
    rows = e.rstrip("\n").split("\n")
    head = [r for r in rows if r.startswith("#")]
    body = [r for r in rows if not r.startswith("#") and r.strip() and not r.startswith(crate + " ")]
    if lower:
        name, by = lower
        m = [r for r in body if r.startswith(name + " ")]
        if len(m) != 1:
            raise SystemExit(f"{path}: no {name} row")
        body = [r for r in body if r != m[0]] + [f"{name} {int(m[0].split()[1]) - by}"]
    body.append(f"{crate} {value}")
    wr(path, "\n".join(head + sorted(body)) + "\n")


set_row(os.path.join(ws, "scores", "eps.txt"), "yack-geom", moved_eps, ("yack-springrep", moved_eps))
set_row(os.path.join(ws, "scores", "lints.txt"), "yack-geom", panics)
fp = os.path.join(ws, "scores", "filesize.txt")
f = rd(fp)
f = sub1(f, "crates/yack-springrep/src/geom/nurbs.rs ", "crates/yack-geom/src/nurbs.rs ", fp)
wr(fp, f)
print(f"ok; yack-geom: {moved_eps} unmarked literals, {panics} contract panics")
