# yack: M3 W9c-0 (lead's ruling): yack-step's file measurer, still behind `wip`, moves under src/measure/
# (bspline, geom, read become measure's own modules) so springrep's step can come in at the crate root.
# Rename only: paths change, nothing else.
# Usage: python3 w9c0-apply.py <yack-step dir>
import os, re, sys

root = os.path.abspath(sys.argv[1])
src = os.path.join(root, "src")


def rd(p):
    return open(p).read()


def wr(p, s):
    tmp = p + ".tmp"
    open(tmp, "w").write(s)
    os.replace(tmp, p)


def sub1(s, a, b, f):
    if s.count(a) != 1:
        raise SystemExit(f"{f}: expected one {a!r}, found {s.count(a)}")
    return s.replace(a, b)


SUBS = ["bspline", "geom", "read"]
os.makedirs(os.path.join(src, "measure"), exist_ok=True)
for m in SUBS:
    t = rd(os.path.join(src, m + ".rs"))
    for n in SUBS:
        t = re.sub(r"\bcrate::" + n + r"\b", "crate::measure::" + n, t)
    wr(os.path.join(src, "measure", m + ".rs"), t)
    os.remove(os.path.join(src, m + ".rs"))
mp = os.path.join(src, "measure.rs")
t = rd(mp)
for n in SUBS:
    t = re.sub(r"\bcrate::" + n + r"\b", "crate::measure::" + n, t)
# measure.rs declares the three itself now, so a `{self, ..}` entry would import them a second time
t = re.sub(r"(use crate::measure::(?:bspline|geom|read)::\{)self,\s*", r"\1", t)
# the three are measure's own modules now, declared ahead of its first `use`
first_use = re.search(r"(?m)^use ", t)
t = t[:first_use.start()] + "mod bspline;\nmod geom;\nmod read;\n\n" + t[first_use.start():]
wr(mp, t)
lp = os.path.join(src, "lib.rs")
t = rd(lp)
for n in SUBS:
    t = sub1(t, f'#[cfg(feature = "wip")]\nmod {n};\n', "", lp)
wr(lp, t)
left = [f for f in os.listdir(src) if f.endswith(".rs")]
if sorted(left) != ["lib.rs", "measure.rs"]:
    raise SystemExit(f"src/ holds {sorted(left)}")
if re.search(r"\bcrate::(bspline|geom|read)\b", "\n".join(rd(os.path.join(dp, f)) for dp, _, fs in os.walk(src) for f in fs)):
    raise SystemExit("a path still names a moved module at the crate root")
print("moved:", [f"src/measure/{m}.rs" for m in SUBS])
