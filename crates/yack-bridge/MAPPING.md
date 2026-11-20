# yack-bridge: springrep `Brep` ↔ cadrs `Body` mapping (M1.9)

This crate is temporary and is deleted by M4. It converts between yack-springrep's `topo::Brep` and
yack-cadrs's `native::Body` so that the M1.10 ladder can run either engine on either engine's
result. A conversion is **exact** where both sides have the same representation. It is **bounded**
(approximated within a stated tolerance, then verified) where they don't, and it **refuses**
(`Unsupported`, naming the entity) where no bounded form exists. It never returns a body that
fails its target's own checker.

Sources read: springrep `geom/{surface,curve,nurbs,bspline,intersection}.rs`, `topo/{mod,pcurve}.rs`,
`step/write.rs` (`ruled_rails`); cadrs `native/{surface,curve,nurbs,topo,pcurve,math}.rs`.

## 1. Surfaces

Both engines use the same `(u, v)` formulas for the analytic five. The PLAN's "cone slant /
sphere latitude reparametrisation" note doesn't apply: both use slant length for the cone and
latitude for the sphere. So pcurves of analytic faces carry over unchanged.

| springrep `geom::Surface` | cadrs `native::Surface` | Formula (both) | Kind |
|---|---|---|---|
| `Plane { frame }` | `Plane { f }` | `o + u·x + v·y` | exact |
| `Cylinder { frame, radius }` | `Cylinder { f, r }` | `o + r·radial(u) + v·z` | exact |
| `Cone { frame, radius, half_angle }` | `Cone { f, r, a }` | `o + (r + v sin a)·radial(u) + v cos a·z`, `a` signed, `v` slant | exact |
| `Sphere { frame, radius }` | `Sphere { f, r }` | `o + r(cos v·radial(u) + sin v·z)`, `v` latitude | exact |
| `Torus { frame, major, minor }` | `Torus { f, rmaj, rmin }` | `o + (R + r cos v)·radial(u) + r sin v·z` | exact |
| `Extrusion { curve, dir }` (`dir` unit) | `Extrusion { c, d }` | `C(u) + v·d` | exact if the curve maps exactly (§2) |
| `Revolution { curve, origin, axis }` | `Revolution { f, m }` | `Rot_axis(u)·m(v)`; `u = 0` is the curve itself on both | exact. For S→C, `f.o = origin`, `f.z = axis`, `f.x` = the unit radial direction of `m(v_mid)` (cadrs's revolution projection measures `u` from `f.x`) |
| `Ruled { a, a_range, b, b_range }` | `Nurbs` of degree `(q, 1)` | `(1−v)·a(u) + v·b(u)`, `u ∈ [0,1]` | exact via springrep's own `step::write::ruled_rails` (two polynomial rails, or two conics over equal spans). Anything else: **refused** |
| `Nurbs(NurbsSurface)` | `Nurbs(Arc<NurbsSurface>)` | same B-spline. springrep stores Euclidean control points + optional weights in a flat u-major net; cadrs stores homogeneous `[wx, wy, wz, w]` in rows along u | exact (multiply/divide by w) |
| – | `Nurbs` → springrep `Nurbs` | as above | exact |

Frames map field by field (`origin/x/y/z` ↔ `o/x/y/z`).

## 2. Curves (3-D edge curves)

| springrep `geom::Curve` | cadrs `native::Curve` | Parameter | Kind |
|---|---|---|---|
| `Line { origin, dir }` (unit dir) | `Line { o, d }` | `o + t·d`, `t` arc length | exact |
| `Circle { frame, radius }` | `Circle { c, x, y, r }` | `c + r(cos t·x + sin t·y)` | exact |
| `Ellipse { frame, major, minor }` | `Ellipse { c, x, y, a, b }` | `c + a cos t·x + b sin t·y` | exact |
| `BSpline` (non-rational) | `Nurbs` with all `w = 1` | same knots | exact |
| `Nurbs` (Euclidean + optional weights) | `Nurbs` (homogeneous) | same knots | exact |
| `Intersection { approx, surfaces }` | `Nurbs` of `approx` | `approx`'s own parameter | **bounded**: the same B-spline springrep's STEP writer exports. Its distance from both surfaces is checked at sample points against the edge tolerance; past it, the edge gets cadrs `tol` = the measured gap |
| – | `Nurbs` → springrep `BSpline` if every `w == 1`, else `Nurbs` | same knots | exact |

Edge ranges carry over unchanged: springrep `range: (t0, t1)` ↔ cadrs `t: [t0, t1]`, with `t0 < t1` on both.

**Degenerate edges** (poles, apexes): springrep has `curve: None`, where `range` is the pcurve parameter. cadrs has
`degenerate: true` with a zero-direction `Line` at the pole, and `t` is the pcurve parameter. They map
through `BodyBuilder::degenerate_edge(at = vertex point, t0, t1, v)` and, coming back, `add_degenerate_edge(v, range)`.

## 3. Pcurves (one per coedge)

| springrep `topo::PCurve` | cadrs `Coedge::pc` (2-D `Curve`, z = 0, parametrised by the **edge's t**) |
|---|---|
| `Affine { origin, dir }`: `uv = origin + t·dir` | a degree-1 `Nurbs` from `uv(t0)` to `uv(t1)` on knots `[t0, t0, t1, t1]`. That is exact and keeps the edge parameter. It isn't a `Line`, because cadrs's `Curve::line` normalises `d` and the bridge doesn't rely on cadrs accepting a non-unit one |
| `Projected { seed }` | `pcurve::pcurve(surf, curve, t0, t1, near)`, with `near` = springrep's own `coedge_uvs(c, [t_start])`. Then shifted by whole periods (`shift_into`) onto springrep's branch, and verified against `coedge_uvs` at 9 parameters (uv distance ≤ 1e-9 scaled). A mismatch is **refused** |
| degenerate edge's pcurve | a degree-1 `Nurbs` through springrep's `coedge_uvs` at `t0` and `t1` (a pole's pcurve is a straight line) |

| cadrs `pc` | springrep `PCurve` |
|---|---|
| `Line { o, d }` | `Affine { origin: o, dir: d }` (exact) |
| degree-1 `Nurbs`, 2 control points, knots `[t0, t0, t1, t1]`, weights 1 | `Affine { origin: p0 − dir·t0, dir: (p1 − p0)/(t1 − t0) }` (exact) |
| any other | `Projected { seed: pc(t_start) }`, `t_start = t0` if `fwd` else `t1`. springrep re-projects the 3-D curve; the result is then checked (§5) |

## 4. Topology

| | springrep | cadrs |
|---|---|---|
| Vertex | `point`; local tolerance in `Brep::tolerances` | `p`, `tol` (≥ `TOL_MIN`) |
| Edge | `curve: Option`, `range`, `start`, `end`; local tol | `curve`, `t`, `v: [start, end]`, `degenerate`, `tol` |
| Coedge | arena entry: `edge`, `reversed`, `pcurve`, `lp` | inline in the face loop: `edge`, `fwd = !reversed`, `pc` |
| Loop | material on the left seen from outside; first loop outer | CCW about the outward normal; first loop outer (same convention) |
| Face | `surface`, `reversed` (outward = −natural) | `surf`, `sense = !reversed` |
| Shell | `faces` | `faces`, `closed = true` (solids only) |
| Solid | `shells`, first `outer` of them outer | `shells`: outer first, then voids |
| Seam | one edge used twice by one face | same |

Ids are preserved by order: vertex, edge and face *i* on one side is entity *i* on the other.
Lists are built in index order, so a face id selected on one engine names the same face on the
other. springrep coedge order inside each loop is kept.

**Refused for now** (`Unsupported`, naming it):
- a springrep solid with `outer > 1` (touching pieces sharing an edge or vertex; cadrs's `Solid` has one outer shell);
- an edge used by more than two coedges (springrep's `partners`/`beside` wedges);
- a cadrs body with wires, acorn points or open shells (sheets).

**Names.** springrep `Face::names` (`FaceName` sets) have no slot in cadrs's `Body`. They are kept
in a side table by face index and re-attached on the way back. A cadrs op's `History` can then be
composed onto them (M1.10). Edge and vertex tolerances carry over both ways.

## 5. Checks (the exit test)

Every conversion ends with the target's checker: cadrs `BodyBuilder::build_with(CheckLevel::Full)`,
springrep `check::check`. A non-empty result is an error, never a body. The round-trip test
(`tests/round_trip.rs`) runs S→C→S and C→S→C and requires, on both sides:
- volume and area equal within `1e-9` relative;
- `classify` (springrep `rays::Rays::side`, cadrs `classify`) equal on a fixed grid of samples
  over the bounding box;
- check clean.

The bench-wide and conformance-wide runs (all 506 bench results, every conformance result) go
through an xtask or adapter hook once M1.10's ladder exists. Until then the crate's own tests cover
one result per surface/curve kind.
