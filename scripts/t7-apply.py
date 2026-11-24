# M2 T7: sweep::along no longer calls boolean. The caller hands it the join (boolean::union), so the
# pieces are built and joined in exactly the order they were, and sweep stays below boolean.
# Paths and one parameter only. Usage: python3 t7-apply.py <yack-springrep dir> [--with-external ...]
import os, sys

root = sys.argv[1]
ws = os.path.dirname(os.path.dirname(os.path.abspath(root)))


def edit(path, pairs):
    s = open(path).read()
    for a, b, n in pairs:
        if s.count(a) != n:
            raise SystemExit(f"{path}: expected {n} of {a[:80]!r}, found {s.count(a)}")
        s = s.replace(a, b)
    open(path, "w").write(s)


sw = os.path.join(root, "src", "sweep.rs")
edit(sw, [
    ("// Changes: none (verbatim); this header added.",
     "// Changes: `along` takes the join of its pieces from the caller (M2 T7), so sweep names no boolean.", 1),
    ("    Error, Result, boolean, check,\n", "    Error, Result, check,\n", 1),
    ("/// for each bend and the fill of each sharp turn, joined into one solid.\n"
     "pub fn along(profile: &Profile, frame: &Frame, legs: &[Leg], feature: &str) -> Result<Brep> {",
     "/// for each bend and the fill of each sharp turn, joined into one solid by `join` (a union, which\n"
     "/// the caller supplies so that sweeps stay below the booleans), each piece onto the pieces before.\n"
     "pub fn along(\n    profile: &Profile,\n    frame: &Frame,\n    legs: &[Leg],\n    feature: &str,\n"
     "    join: impl Fn(&Brep, &Brep) -> Result<Brep>,\n) -> Result<Brep> {", 1),
    ("                Some(w) => boolean::union(&w, &piece)?,", "                Some(w) => join(&w, &piece)?,", 1),
])

oc = os.path.join(root, "tests", "one_call.rs")
s = open(oc).read()
n = s.count('sweep::along(&p, &Frame::WORLD, &')
edit(oc, [
    ('sweep::along(&p, &Frame::WORLD, &legs, "swp")', 'sweep::along(&p, &Frame::WORLD, &legs, "swp", boolean::union)', 3),
    ('sweep::along(&p, &Frame::WORLD, &askew, "swp")', 'sweep::along(&p, &Frame::WORLD, &askew, "swp", boolean::union)', 1),
    ('sweep::along(&p, &Frame::WORLD, &[], "swp")', 'sweep::along(&p, &Frame::WORLD, &[], "swp", boolean::union)', 1),
])
if n != 5:
    raise SystemExit(f"{oc}: {n} along calls, expected 5")

mk = os.path.join(ws, "crates", "yack", "src", "make.rs")
edit(mk, [
    ("use yack_springrep::{loft as sr_loft, primitive, sweep};",
     "use yack_springrep::{boolean as sr_boolean, loft as sr_loft, primitive, sweep};", 1),
    ("sweep::along(&engine::profile(op, profile, None)?, &engine::frame(frame), &legs, &feature))",
     "sweep::along(&engine::profile(op, profile, None)?, &engine::frame(frame), &legs, &feature, sr_boolean::union))", 1),
])
print("ok")
