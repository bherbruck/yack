# M2 (lead's ruling on measure -> locate): a face's trimmed domain moves DOWN into topo, so measure and
# locate (L5 siblings) both reach it there. locate/domain.rs -> topo/domain.rs; face_domain,
# coedge_uv_at, coedge_trim_with -> topo/trims.rs. same_domain_faces and analytic stay in locate (they
# need intersect). Paths only. Usage: python3 tdom-apply.py <yack-springrep dir>
import os, re, sys

src = os.path.join(sys.argv[1], "src")
p = lambda *a: os.path.join(src, *a)


def rd(f):
    return open(p(f)).read()


def wr(f, s):
    open(p(f), "w").write(s)


def edit(f, pairs):
    s = rd(f)
    for a, b in pairs:
        if s.count(a) != 1:
            raise SystemExit(f"{f}: expected one {a[:70]!r}, found {s.count(a)}")
        s = s.replace(a, b)
    wr(f, s)


def cut(s, start, end):
    i = s.index(start)
    j = s.rfind("\n\n", 0, i) + 2
    k = s.index(end, i)
    return s[:j] + s[k:], s[j:k]


# ---- locate/domain.rs -> topo/domain.rs ----
s = rd("locate/domain.rs")
s = re.sub(r"// Changes: .*", "// Changes: moved from boolean/domain.rs to locate/domain.rs (M2.1, T1), then to topo/domain.rs\n// (M2, measure and locate both use it); paths only.", s, count=1)
wr("topo/domain.rs", s)
os.remove(p("locate/domain.rs"))

# ---- face_domain + coedge helpers out of locate/mod.rs ----
loc = rd("locate/mod.rs")
loc, uv_at = cut(loc, "pub(crate) fn coedge_uv_at(", "/// A trim for coedge `c`")
loc, trim_with = cut(loc, "pub(crate) fn coedge_trim_with(", "/// Face `f`'s trimmed domain outside any boolean")
i = loc.index("/// Face `f`'s trimmed domain outside any boolean")
k = loc.index("\n}\n", i) + 3
face_dom = loc[i:k]
loc = loc[:i] + loc[k:].lstrip("\n")

wr("topo/trims.rs", f"""// yack: a face's trimmed domain outside any boolean, and the trims of its coedges.
// Adapted from springrep@a03e8a1 src/boolean/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved to locate/mod.rs (M2.1, T1), then to topo/trims.rs (M2, measure and locate both use
// it); paths only.
//! A face's trimmed domain outside any boolean, its trims numbered by the arena's vertices, and the
//! trim of one coedge.

use std::rc::Rc;

use super::domain::{{Domain, Trim, UvMap}};
use super::{{Brep, CoedgeId, FaceId, PCurve, coedge_params}};
use crate::{{geom::Curve, math::Vec2}};

{uv_at.rstrip()}

{trim_with.rstrip()}

{face_dom.rstrip()}
""")

loc = loc.replace("pub(crate) mod classify;\npub(crate) mod domain;\n", "pub(crate) mod classify;\n")
loc = loc.replace("""use std::rc::Rc;

use self::domain::{Domain, Trim, UvMap};
use crate::{
    geom::{Curve, Surface},
    intersect::ssi::same_domain,
    math::Vec2,
    tol,
    topo::{Brep, CoedgeId, FaceId, PCurve, coedge_params},
};""", """use crate::{geom::Surface, intersect::ssi::same_domain, tol, topo::domain::Domain};""")
if "use crate::{geom::Surface, intersect::ssi::same_domain, tol, topo::domain::Domain};" not in loc:
    raise SystemExit("locate/mod.rs: import block not as expected")
loc = loc.replace("//! Point location: a face's trimmed domain (`domain`) and point-in-solid classification\n//! (`classify`), with the trimmed domain of a face outside any boolean.",
                  "//! Point location: point-in-solid classification (`classify`), and whether two faces lie on one\n//! surface. A face's trimmed domain is topo's (`topo::domain`, `topo::face_domain`).")
wr("locate/mod.rs", loc)

edit("topo/mod.rs", [
    ("mod codec;\n" if "mod codec;\n" in rd("topo/mod.rs") else "mod facts;\n",
     ("mod codec;\n" if "mod codec;\n" in rd("topo/mod.rs") else "mod facts;\n") + "pub(crate) mod domain;\n"),
    ("mod similar;\n", "mod similar;\nmod trims;\n"),
    ("pub use pcurve::PCurve;\n", "pub use pcurve::PCurve;\npub(crate) use trims::{coedge_trim_with, coedge_uv_at, face_domain};\n"),
])

# ---- callers: brace groups first ----
edit("locate/classify.rs", [("use super::domain::{Domain, OnBoundary, Where};", "use crate::topo::domain::{Domain, OnBoundary, Where};")])
edit("check.rs", [("    locate::{classify, domain::Domain, face_domain},\n", "    locate::classify,\n    topo::{domain::Domain, face_domain},\n")])
edit("blend/rolling.rs", [("    locate::{\n        domain::{Domain, Where},\n        face_domain,\n    },\n",
                           "    topo::{\n        domain::{Domain, Where},\n        face_domain,\n    },\n")])
edit("distance.rs", [("    locate::{\n        classify,\n        domain::{Domain, Where},\n        face_domain,\n    },\n",
                      "    locate::classify,\n    topo::{\n        domain::{Domain, Where},\n        face_domain,\n    },\n")])
edit("rays.rs", [("    locate::{\n        classify::DIRECTIONS,\n        domain::{Domain, Where},\n        face_domain,\n    },\n",
                  "    locate::classify::DIRECTIONS,\n    topo::{\n        domain::{Domain, Where},\n        face_domain,\n    },\n")])
edit("boolean/same.rs", [("    locate::{domain::Domain, face_domain},\n", "    topo::{domain::Domain, face_domain},\n")])
edit("boolean/tests.rs", [("    locate::{classify, face_domain},\n", "    locate::classify, topo::face_domain,\n")])
edit("boolean/unify.rs", [("use crate::locate::{\n    coedge_uv_at,\n    domain::{Trim, UvMap},\n};", "use crate::topo::{\n    coedge_uv_at,\n    domain::{Trim, UvMap},\n};")])
edit("boolean/sew.rs", [("    locate::{\n        classify,\n        domain::{Domain, Trim, UvMap, polygon_area},\n        same_domain_faces,\n    },\n",
                         "    locate::{classify, same_domain_faces},\n    topo::domain::{Domain, Trim, UvMap, polygon_area},\n")])
edit("boolean/mod.rs", [("    locate::{\n        classify, coedge_trim_with, coedge_uv_at,\n        domain::{Domain, Trim, UvMap, Where},\n        same_domain_faces,\n    },\n",
                         "    locate::{classify, same_domain_faces},\n    topo::{\n        coedge_trim_with, coedge_uv_at,\n        domain::{Domain, Trim, UvMap, Where},\n    },\n")])
edit("blend/saddle.rs", [("    locate::domain::Where,\n", "    topo::domain::Where,\n")])
edit("boolean/arrange.rs", [("use crate::locate::domain::{", "use crate::topo::domain::{")])
edit("measure/sample.rs", [("    locate::face_domain,\n", "    topo::face_domain,\n")])
edit("measure.rs", [("    use crate::locate::{domain::Where, face_domain};", "    use crate::topo::{domain::Where, face_domain};")])

# ---- callers: qualified paths ----
for dp, _, fs in os.walk(src):
    for fn in fs:
        if not fn.endswith(".rs"):
            continue
        f = os.path.join(dp, fn)
        s0 = s = open(f).read()
        s = s.replace("crate::locate::domain::", "crate::topo::domain::").replace("crate::locate::face_domain", "crate::topo::face_domain")
        if s != s0:
            open(f, "w").write(s)
left = []
for dp, _, fs in os.walk(src):
    for fn in fs:
        if fn.endswith(".rs"):
            s = open(os.path.join(dp, fn)).read()
            if re.search(r"locate::(domain|face_domain|coedge_)", s) or ("super::domain" in s and "/topo/" not in dp + "/"):
                left.append(os.path.join(dp, fn))
if left:
    raise SystemExit(f"stale paths left in {left}")
print("ok")
