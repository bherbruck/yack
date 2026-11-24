# M3 W2' (D26): springrep's math.rs and math/bvh.rs move verbatim into yack-math as the temporary
# module `yack_math::kernel` (with `kernel::bvh`), re-exported by springrep as `math`. Paths and
# visibility only; the eps ratchet's unmarked literals move with them (scores/eps.txt, lead-approved).
# Usage: python3 w2-apply.py <yack-springrep dir> [--with-external ...]
import os, re, subprocess, sys

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")
crates = os.path.dirname(root)
ws = os.path.dirname(crates)
here = os.path.dirname(os.path.abspath(__file__))


def rd(p):
    return open(p).read()


def wr(p, s):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    open(p, "w").write(s)


def sub1(s, a, b, f):
    if s.count(a) != 1:
        raise SystemExit(f"{f}: expected one {a[:80]!r}, found {s.count(a)}")
    return s.replace(a, b)


def edit(p, pairs):
    s = rd(p)
    for a, b in pairs:
        s = sub1(s, a, b, p)
    wr(p, s)


def crate_vis(s):
    return re.sub(r"\bpub\(crate\)", "pub", s)


def eps(paths):
    return int(subprocess.check_output([sys.executable, os.path.join(here, "eps_count.py")] + paths).decode())


km = os.path.join(crates, "yack-math", "src", "kernel.rs")
kb = os.path.join(crates, "yack-math", "src", "kernel", "bvh.rs")
moved_eps = eps([os.path.join(src, "math.rs"), os.path.join(src, "math")])

m = rd(os.path.join(src, "math.rs"))
m = sub1(m, "// Changes: `within` moved in from tol.rs (M2), being the one tolerance compare that takes a point.",
         "// Changes: `within` moved in from tol.rs (M2), being the one tolerance compare that takes a point;\n"
         "// moved to yack_math::kernel (M3 W2', D26); paths and visibility only.", "math.rs")
m = sub1(m, "//! name at each use says which it is.\n",
         "//! name at each use says which it is.\n//!\n"
         "//! springrep's own maths, beside yack-math's checked types. Temporary (D26): each crate's M3\n"
         "//! hardening moves onto those types, and this module goes when the last user has.\n", "math.rs")
m = sub1(m, "use crate::{Result, error::invalid, tol};\n",
         "use yack_base::kernel::{Result, invalid};\nuse yack_tol as tol;\n", "math.rs")
wr(km, crate_vis(m))

b = rd(os.path.join(src, "math", "bvh.rs"))
b = sub1(b, "indices rather than a mesh.",
         "indices rather than a mesh; moved to yack_math::kernel::bvh (M3 W2', D26); paths and visibility only.", "bvh.rs")
b = sub1(b, "use crate::math::{Aabb, Point3, Transform, Vec3};", "use super::{Aabb, Point3, Transform, Vec3};", "bvh.rs")
if "crate::" in re.sub(r"//[^\n]*", "", b):
    raise SystemExit("bvh.rs still names crate::")
wr(kb, crate_vis(b))
if "crate::" in re.sub(r"//[^\n]*", "", rd(km)):
    raise SystemExit("math.rs still names crate::")
os.remove(os.path.join(src, "math.rs"))
os.remove(os.path.join(src, "math", "bvh.rs"))
os.rmdir(os.path.join(src, "math"))

ml = os.path.join(crates, "yack-math", "src", "lib.rs")
s = rd(ml)
s = sub1(s, "\nmod aabb;\n", "\nmod aabb;\n// springrep's own maths, beside this crate's checked types until each crate converts (D26).\n"
         "#[allow(missing_docs)]\n"
         "// springrep indexes by position; D26's hardening pass moves it onto checked access, as for\n"
         "// yack-linalg until M5.\n#[allow(clippy::indexing_slicing)]\npub mod kernel;\n", "yack-math lib.rs")
wr(ml, s)
mc = os.path.join(crates, "yack-math", "Cargo.toml")
c = rd(mc)
for d in ["yack-base", "yack-tol"]:
    if f"{d} = " not in c:
        c = c.replace("[dependencies]\n", f'[dependencies]\n{d} = {{ path = "../{d}" }}\n', 1)
wr(mc, c)

edit(os.path.join(src, "lib.rs"), [("pub mod math;\n", "pub use yack_math::kernel as math;\n")])
sc = os.path.join(root, "Cargo.toml")
c = rd(sc)
if "yack-math = " not in c:
    c = sub1(c, "[dependencies]\n", '[dependencies]\nyack-math = { path = "../yack-math" }\n', sc)
wr(sc, c)
lt = os.path.join(ws, "layers.toml")
s = rd(lt)
mm = re.search(r'(?ms)^\[crate\.yack-springrep\]\n.*?^deps = \[([^\]]*)\]', s)
have = [x.strip().strip('"') for x in mm.group(1).split(",") if x.strip()]
if "yack-math" not in have:
    s = s[:mm.start(1)] + ", ".join(f'"{d}"' for d in have + ["yack-math"]) + s[mm.end(1):]
wr(lt, s)

# gate 7: the moved literals' row moves with them
ep = os.path.join(ws, "scores", "eps.txt")
e = rd(ep)
sr = re.search(r"(?m)^yack-springrep (\d+)$", e)
e = e.replace(sr.group(0), f"yack-springrep {int(sr.group(1)) - moved_eps}")
if moved_eps:
    rows = [l for l in e.rstrip("\n").split("\n")]
    rows = [r for r in rows if not r.startswith("yack-math ")]
    head = [r for r in rows if r.startswith("#")]
    body = sorted([r for r in rows if not r.startswith("#")] + [f"yack-math {moved_eps}"])
    e = "\n".join(head + body) + "\n"
wr(ep, e)
print(f"ok; {moved_eps} unmarked literals move to yack-math")
