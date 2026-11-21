# M2 (lead's ruling on tol -> math::Point3): the one tol function that takes a point, `within`, moves to
# math as `math::within`; tol keeps its constants and scalar compares. Same body. Paths only.
# Usage: python3 twithin-apply.py <yack-springrep dir>
import os, sys

src = os.path.join(sys.argv[1], "src")
p = lambda *a: os.path.join(src, *a)


def rd(f):
    return open(p(f)).read()


def wr(f, s):
    open(p(f), "w").write(s)


t = rd("tol.rs")
i = t.index("/// `d <= t`, read with a few ulps")
k = t.index("\n}\n", i) + 3
fn = t[i:k]
t = t[:i].rstrip("\n") + "\n" + t[k:]
t = t.replace("// Changes: none (verbatim); this header added.",
              "// Changes: `within`, which takes a point, moved to math (M2): tol keeps constants and scalar\n// compares.", 1)
wr("tol.rs", t)

fn = fn.replace("pub fn within(d: f64, t: f64, at: crate::math::Point3) -> bool {", "pub fn within(d: f64, t: f64, at: Point3) -> bool {")
if "pub fn within(d: f64, t: f64, at: Point3) -> bool {" not in fn:
    raise SystemExit("tol.rs: within() not as expected")
fn = fn.replace("[`LINEAR`]", "[`tol::LINEAR`]")
m = rd("math.rs")
m = m.replace("// Changes: none (verbatim); this header added.",
              "// Changes: `within` moved in from tol.rs (M2), being the one tolerance compare that takes a point.", 1)
anchor = "#[derive(Clone, Copy, Debug, Default, PartialEq)]\npub struct Vec3 {"
if m.count(anchor) != 1:
    raise SystemExit("math.rs: Vec3 anchor not found once")
m = m.replace(anchor, fn + "\n" + anchor)
wr("math.rs", m)

n = 0
for dp, _, fs in os.walk(src):
    for f in fs:
        if not f.endswith(".rs"):
            continue
        path = os.path.join(dp, f)
        s0 = s = open(path).read()
        s = s.replace("tol::within(", "crate::math::within(").replace("crate::crate::", "crate::")
        if s != s0:
            n += s0.count("tol::within(")
            open(path, "w").write(s)
print(f"ok: {n} calls")
