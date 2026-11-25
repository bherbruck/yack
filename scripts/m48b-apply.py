# yack: M4.8 window apply: the revolved chamfer's bevel face is `<feature>.bevel_0` (the call's edge 0, as the facade's history and roles read it).
# Usage: python3 m48b-apply.py <yack-springrep dir> [--with-external ...] (springrep is untouched)
import os, sys

Y = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
B = os.path.join(Y, "crates/yack-blend/src")


def edit(rel, pairs):
    p = os.path.join(B, rel)
    s = open(p).read()
    for old, new in pairs:
        if s.count(old) != 1:
            sys.exit(f"m48b-apply: {rel}: {old!r} not found once")
        s = s.replace(old, new)
    open(p + ".tmp", "w").write(s)
    os.replace(p + ".tmp", p)


edit("chamfer_angle/revolved.rs", [
    ('            Some(FaceName::new(feature, "bevel"))', '            Some(FaceName::new(feature, "bevel_0"))'),
    ("/// (radians) to it; the bevel face is `<feature>.bevel`.", "/// (radians) to it; the bevel face is `<feature>.bevel_0`, the call's one edge."),
])
edit("chamfer_angle.rs", [
    ('FaceName::new("k", "bevel")', 'FaceName::new("k", "bevel_0")'),
    ("The bevel face is\n/// `<feature>.bevel`;", "The bevel face is\n/// `<feature>.bevel_0`;"),
])
