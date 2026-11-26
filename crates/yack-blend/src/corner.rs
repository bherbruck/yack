// yack: A fillet running round a corner round smaller than itself and bent the same way: the corner of a
// Adapted from springrep@a03e8a1 src/blend/corner.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/blend/corner.rs to yack-blend (M3 W9a); paths and visibility only;
// hardened (M3 L2): checked access in place of indexing, its contract panics gone, its small literals named.
//! A fillet running round a corner round smaller than itself and bent the same way: the corner of a
//! plate whose walls meet in a round of radius ρ, its rim filleted with r > ρ, or a pocket's floor
//! filleted past its corner rounds. The ball cannot follow the round: it rolls along each wall, both
//! pieces sharing one corner centre `c`, r from the plate and from both walls, inside the material
//! for a convex rim and outside it for a concave one. The corner axis is the line through `c`
//! square to the plate, and `T` the point where it meets the plate.
//!
//! The corner is a family of quarter ellipses, one in each half plane through the axis, each from a
//! point `b` of a base curve r below the plate (on the walls, square to the plate) up to `T`: a
//! rational biquadratic whose rows are the base curve, the base curve lifted by r and `T` three
//! times, weighted by the base curve's weights times the quarter ellipse's (1, √½, 1). It leaves the
//! base square to the plate, tangent to the wall there, and reaches `T` level with the plate.
//!
//! A straight piece along a flat wall is the same family over the wall's line, since a plane
//! through a point of its axis cuts it in such an ellipse: the corner continues it exactly from any
//! half plane, and takes over from the line where the wall meets the round. A piece along a
//! cylindrical wall is a torus, of the family only in the one half plane through the cylinder's
//! axis, where its section is the circle about `c` and its distance from `c` is least, so the two
//! share their tangent plane all along it. There the torus ends, and the corner's base runs on from
//! the round along the cylinder to that half plane. The base is the round's arc between its walls'
//! tangents with an arc of each cylindrical wall, all circles: the corner is exact.
//!
//! The material between the corner, the walls and the plate is a solid, the corner tool, cut (or,
//! for a concave rim, added) with the piece before it as one tool.

use std::collections::BTreeSet;

use super::{Clip, Picked, outward, run};
use crate::{
    Error, Result, check,
    geom::{Curve, Line, NurbsCurve, NurbsSurface, Surface},
    math::{Point3, Vec2, Vec3},
    tol,
    topo::{Brep, CoedgeSpec, EdgeId, FaceId, FaceName, PCurve, VertexId},
};

/// The marker feature of the corner tool's faces until the blend names them.
pub(super) const MARK: &str = "#corner";

/// A corner the fillet's ball cannot round: the round's arc it leaves out, how each piece beside
/// it ends, and the tool that cuts the corner.
pub(super) struct Swallowed {
    /// The picked arc, by index; none at a sharp concave corner, which has no round.
    pub arc: Option<usize>,
    /// Each piece's edge, its vertex at the corner, and the plane it ends on there.
    pub ends: [(EdgeId, VertexId, Clip); 2],
    /// The face of the plate, which both pieces and the corner tool lie against.
    pub plate: FaceId,
    /// The corner tool: its faces marked [`MARK`] `patch`, `plate`, and `face_<index>` for each
    /// body face it lies on, by its index in the body.
    pub tool: Brep,
}

fn unsupported<T>(why: &str) -> Result<T> {
    Err(Error::Unsupported(why.into()))
}

/// `uv` moved by whole periods to the image nearest `to`.
fn near(s: &Surface, uv: Vec2, to: Option<Vec2>) -> Vec2 {
    let Some(to) = to else { return uv };
    let shift = |x: f64, t: f64, per: Option<f64>| per.map_or(x, |p| x + ((t - x) / p).round() * p);
    Vec2::new(
        shift(uv.x, to.x, s.u_period()),
        shift(uv.y, to.y, s.v_period()),
    )
}

/// The least weight of a quadratic arc's middle control point: below it the arc turns by nearly a
/// half turn, whose middle point runs off towards infinity.
const LEAST_WEIGHT: f64 = 1e-6; // tol: a weight, not a length

/// How near zero a determinant or distance is that leaves two walls with no single crossing.
const DEGENERATE: f64 = 1e-12; // tol: guards a division, not a length that decides anything

/// The rational quadratic arc of a circle about `centre` from `a` to `b` the short way: its middle
/// control point and weight.
fn arc_middle(centre: Point3, a: Point3, b: Point3) -> Option<(Point3, f64)> {
    let (u, v) = (a - centre, b - centre);
    let half = 0.5 * u.cross(v).norm().atan2(u.dot(v));
    let w = half.cos();
    let mid = (u + v).unit()?;
    (w > LEAST_WEIGHT).then(|| (centre + mid * (u.norm() / w), w))
}

/// A wall beside the round, in the plate's plane: a line through `p` with outward normal `n`, or a
/// circle about `centre` whose outward normal is `out` times the radial direction.
#[derive(Clone, Copy)]
enum Wall {
    Line { p: Vec2, n: Vec2 },
    Circle { centre: Vec2, radius: f64, out: f64 },
}

impl Wall {
    /// The wall moved `d` behind itself, against its outward normal.
    fn offset(self, d: f64) -> Wall {
        match self {
            Wall::Line { p, n } => Wall::Line { p: p - n * d, n },
            Wall::Circle {
                centre,
                radius,
                out,
            } => Wall::Circle {
                centre,
                radius: radius - out * d,
                out,
            },
        }
    }
}

/// Where two walls cross in their plane, the crossing nearest `near`.
fn crossing(a: Wall, b: Wall, near: Vec2) -> Option<Vec2> {
    let line_circle = |p: Vec2, n: Vec2, centre: Vec2, radius: f64| -> Vec<Vec2> {
        let along = Vec2::new(-n.y, n.x);
        let foot = p + along * (centre - p).dot(along);
        let h2 = radius * radius - (centre - foot).dot(centre - foot);
        if h2 < 0.0 {
            return Vec::new();
        }
        let h = h2.sqrt();
        vec![foot + along * h, foot - along * h]
    };
    let found: Vec<Vec2> = match (a, b) {
        (Wall::Line { p: p1, n: n1 }, Wall::Line { p: p2, n: n2 }) => {
            let det = n1.x * n2.y - n1.y * n2.x;
            if det.abs() <= DEGENERATE {
                return None;
            }
            let (c1, c2) = (n1.dot(p1), n2.dot(p2));
            vec![Vec2::new(
                (c1 * n2.y - c2 * n1.y) / det,
                (n1.x * c2 - n2.x * c1) / det,
            )]
        }
        (Wall::Line { p, n }, Wall::Circle { centre, radius, .. })
        | (Wall::Circle { centre, radius, .. }, Wall::Line { p, n }) => {
            line_circle(p, n, centre, radius)
        }
        (
            Wall::Circle {
                centre: c1,
                radius: r1,
                ..
            },
            Wall::Circle {
                centre: c2,
                radius: r2,
                ..
            },
        ) => {
            let d = (c2 - c1).norm();
            if d <= DEGENERATE {
                return None;
            }
            let x = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
            let h2 = r1 * r1 - x * x;
            if h2 < 0.0 {
                return None;
            }
            let e = (c2 - c1) / d;
            let f = Vec2::new(-e.y, e.x);
            let m = c1 + e * x;
            vec![m + f * h2.sqrt(), m - f * h2.sqrt()]
        }
    };
    found
        .into_iter()
        .min_by(|x, y| (*x - near).norm().total_cmp(&(*y - near).norm()))
}

/// One arc of the corner's base: its centre (r below the plate), its ends, and the body face it
/// lies on.
struct BaseArc {
    centre: Point3,
    from: Point3,
    to: Point3,
    face: FaceId,
}

/// The corners where two picked edges of one flat face, the plate, meet over a sharp concave edge
/// square to it, as along the top of an L where its arms meet. The ball does not stop where the two
/// pieces would mitre: it swings round the concave edge, touching it, its centre on the arc of
/// radius r about the edge, r beneath the plate. Each piece ends on the plane through the concave
/// edge square to its own edge, and the corner tool is what the swing takes: the region within
/// `δ(s) = r − √(r² − s²)` of the concave edge at height `s` above the centres' plane, between those
/// planes. Its faces there are the pieces' end sections exactly, and its patch is the horn torus of
/// radii r and r about the edge, tangent to both pieces along them.
pub(super) fn sharp(body: &Brep, picked: &[Picked], radius: f64) -> Result<Vec<Swallowed>> {
    let r = radius;
    let mut out = Vec::new();
    let flat_normal = |f: FaceId, p: Point3| match &body.face(f).surface {
        Surface::Plane(_) => Some(outward(body, f, p)),
        _ => None,
    };
    for v in body.vertex_ids() {
        let at: Vec<EdgeId> = body
            .edge_ids()
            .filter(|&e| {
                let x = body.edge(e);
                !x.is_degenerate() && (x.start == v || x.end == v)
            })
            .collect();
        if at.len() != 3 {
            continue;
        }
        let here: Vec<&Picked> = picked.iter().filter(|p| at.contains(&p.edge)).collect();
        let &[pi, pj] = here.as_slice() else { continue };
        let Some(&c) = at.iter().find(|&&e| e != pi.edge && e != pj.edge) else {
            continue;
        };
        let Some(&plate) = pi.faces.iter().find(|f| pj.faces.contains(f)) else {
            continue;
        };
        let wall = |pk: &Picked| pk.faces.iter().copied().find(|&f| f != plate);
        let (Some(w1), Some(w2)) = (wall(pi), wall(pj)) else {
            continue;
        };
        let p = body.vertex(v).point;
        let (Some(n), Some(o1), Some(o2)) = (
            flat_normal(plate, p),
            flat_normal(w1, p),
            flat_normal(w2, p),
        ) else {
            continue;
        };
        let Some(Curve::Line(line)) = &body.edge(c).curve else {
            continue;
        };
        if line.dir.cross(n).norm() > tol::ANGULAR {
            continue;
        }
        let Some(cc) = body.coedge_ids().find(|&k| body.coedge(k).edge == c) else {
            continue;
        };
        if run::convex(body, cc) != Some(false)
            || run::convex(body, pi.coedges[0]) != Some(true)
            || run::convex(body, pj.coedges[0]) != Some(true)
        {
            continue;
        }
        let (m1, m2) = (-o1, -o2);
        // Each piece ends on the plane through the concave edge square to its own edge, keeping
        // the side its edge runs to.
        // `m` the wall's inward normal on the piece's own side
        let clip = |pk: &Picked, m: Vec3| {
            let x = body.edge(pk.edge);
            let far = body
                .vertex(if x.start == v { x.end } else { x.start })
                .point;
            let along = n.cross(m);
            let keep = if (far - p).dot(along) >= 0.0 {
                along
            } else {
                -along
            };
            Clip::Plane { origin: p, keep }
        };
        out.push(Swallowed {
            arc: None,
            ends: [(pi.edge, v, clip(pi, m1)), (pj.edge, v, clip(pj, m2))],
            plate,
            tool: swing_tool(p, n, m1, m2, r)?,
        });
    }
    Ok(out)
}

/// The region the ball swinging round a concave edge through `p` takes, the plate's outward
/// normal `n`, the walls facing into the material along `m1` and `m2`: the meridian between the
/// edge, the plate and the arc of radius r about the centres' circle, turned from `m1` to `m2`.
fn swing_tool(p: Point3, n: Vec3, m1: Vec3, m2: Vec3, r: f64) -> Result<Brep> {
    use crate::profile::{NamedSegment, Profile, ProfileLoop, Segment};
    let o = p - n * r;
    let seg = |segment, name: &str| NamedSegment::new(segment, name);
    let meridian = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                seg(
                    Segment::Arc {
                        center: Vec2::new(r, 0.0),
                        radius: r,
                        start: std::f64::consts::PI,
                        sweep: -std::f64::consts::FRAC_PI_2,
                    },
                    "patch",
                ),
                seg(
                    Segment::Line {
                        a: Vec2::new(r, r),
                        b: Vec2::new(0.0, r),
                    },
                    "plate",
                ),
                seg(
                    Segment::Line {
                        a: Vec2::new(0.0, r),
                        b: Vec2::new(0.0, 0.0),
                    },
                    "axis",
                ),
            ],
        }],
    };
    let sketch = crate::math::Frame::new(o, m1.cross(n), m1)?;
    let turn = m1.dot(m2).clamp(-1.0, 1.0).acos();
    let axis = if m1.cross(m2).dot(n) > 0.0 { n } else { -n };
    crate::sweep::revolve(&meridian, &sketch, o, axis, turn, MARK)
}

/// The corners in `picked` where a fillet of `radius` runs round a smaller round bent the same way,
/// each end of the round meeting a piece along a flat wall or a cylindrical one tangent to it.
/// `Err` names a corner of that kind that cannot be built.
pub(super) fn swallowed(body: &Brep, picked: &[Picked], radius: f64) -> Result<Vec<Swallowed>> {
    let mut out = Vec::new();
    for (ai, pk) in picked.iter().enumerate() {
        let e = body.edge(pk.edge);
        let Some(Curve::Circle(circle)) = &e.curve else {
            continue;
        };
        if circle.radius >= radius - tol::LINEAR || e.start == e.end {
            continue;
        }
        // One face a plane square to the circle's axis, the other a cylinder about it: the round of
        // a plate's corner.
        let axis = circle.frame.z;
        let (mut plate, mut round) = (None, None);
        for &f in &pk.faces {
            match &body.face(f).surface {
                Surface::Plane(p) if p.frame.z.cross(axis).norm() <= tol::ANGULAR => {
                    plate = Some(f)
                }
                Surface::Cylinder(c)
                    if c.frame.z.cross(axis).norm() <= tol::ANGULAR
                        && (c.radius - circle.radius).abs() <= tol::LINEAR =>
                {
                    round = Some(f)
                }
                _ => {}
            }
        }
        let (Some(plate), Some(round)) = (plate, round) else {
            continue;
        };
        let Some(convex) = run::convex(body, pk.coedges[0]) else {
            continue;
        };
        // A round bent the other way from the edge (a pocket's corner under its rim, a boss's
        // corner above its base) leaves the ball room to roll round it.
        let at = body.vertex(e.start).point;
        let radial_at =
            |p: Point3| (p - circle.frame.origin) - axis * (p - circle.frame.origin).dot(axis);
        if (outward(body, round, at).dot(radial_at(at)) > 0.0) != convex {
            continue;
        }
        let side = if convex { 1.0 } else { -1.0 };
        let n_plate = outward(body, plate, at);
        // The plate's plane, in which the walls are lines and circles.
        let frame = crate::math::Frame::from_axis(at, n_plate)?;
        let flat = |p: Point3| {
            let l = frame.to_local(p);
            Vec2::new(l.x, l.y)
        };
        let flat_dir = |d: Vec3| Vec2::new(d.dot(frame.x), d.dot(frame.y));
        // The piece on the plate at each end, and its wall, tangent to the round there.
        let mut ends = Vec::with_capacity(2);
        let mut walls = Vec::with_capacity(2);
        for v in [e.start, e.end] {
            let beside: Vec<&Picked> = picked
                .iter()
                .filter(|q| q.edge != pk.edge)
                .filter(|q| {
                    let x = body.edge(q.edge);
                    x.start == v || x.end == v
                })
                .collect();
            let [next] = beside[..] else {
                return unsupported(
                    "a fillet runs round a round smaller than itself where it does not meet one \
                     edge on the plate at each end",
                );
            };
            let Some(&wall) = next.faces.iter().find(|&&f| f != plate) else {
                return unsupported("a fillet's edge lies on the plate twice");
            };
            if !next.faces.contains(&plate) {
                return unsupported(
                    "a fillet runs round a round smaller than itself onto an edge off the plate",
                );
            }
            let p = body.vertex(v).point;
            let n = outward(body, wall, p);
            let shape = match (&body.edge(next.edge).curve, &body.face(wall).surface) {
                (Some(Curve::Line(_)), Surface::Plane(_)) => Wall::Line {
                    p: flat(p),
                    n: flat_dir(n),
                },
                (Some(Curve::Circle(_)), Surface::Cylinder(c))
                    if c.frame.z.cross(n_plate).norm() <= tol::ANGULAR =>
                {
                    let centre = flat(c.frame.origin);
                    let radial = flat(p) - centre;
                    Wall::Circle {
                        centre,
                        radius: c.radius,
                        out: if flat_dir(n).dot(radial) > 0.0 {
                            1.0
                        } else {
                            -1.0
                        },
                    }
                }
                _ => {
                    return unsupported(
                        "a fillet runs round a round smaller than itself next to a wall that is \
                         neither flat nor a cylinder square to the plate",
                    );
                }
            };
            // The wall is tangent to the round where they meet.
            let radial = radial_at(p) * side;
            if n.cross(radial).norm() > tol::ANGULAR * radial.norm().max(1.0)
                || n.dot(radial) <= 0.0
            {
                return unsupported(
                    "a fillet runs round a round smaller than itself whose walls are not tangent to \
                     it",
                );
            }
            ends.push((next.edge, v, wall));
            walls.push(shape);
        }
        // one end and one wall at each end of the round
        let (&[end_a, end_b], [wall_a, wall_b]) = (ends.as_slice(), walls.as_slice()) else {
            return unsupported("a fillet's corner without a wall at each end");
        };
        // From the corner centre to the plate.
        let lift = n_plate * (side * radius);
        let round_centre = flat(circle.frame.origin);
        let Some(c2) = crossing(
            wall_a.offset(side * radius),
            wall_b.offset(side * radius),
            round_centre,
        ) else {
            return unsupported("a fillet's corner whose walls never come within its radius");
        };
        let c = frame.to_world(Vec3::new(c2.x, c2.y, 0.0)) - lift;
        let t = c + lift;
        // The base: round the round between its walls' tangents, and along each cylindrical wall
        // to where it is nearest the corner centre.
        let low = |p: Point3| p - lift;
        let (a1, a2) = (
            low(body.vertex(e.start).point),
            low(body.vertex(e.end).point),
        );
        let on_axis = |o: Point3, d: Vec3, p: Point3| o + d * (p - o).dot(d);
        let round_cyl = match &body.face(round).surface {
            Surface::Cylinder(cyl) => cyl.clone(),
            _ => unreachable!("checked above"),
        };
        let mut arcs = Vec::new();
        let mut base_ends = [a1, a2];
        for (k, ((_, _, wall), a)) in [end_a, end_b].into_iter().zip([a1, a2]).enumerate() {
            let Surface::Cylinder(cyl) = &body.face(wall).surface else {
                continue;
            };
            let o = on_axis(cyl.frame.origin, cyl.frame.z, c);
            let Some(dir) = (c - o).unit() else {
                return unsupported("a fillet's corner centred on its wall's axis");
            };
            let foot = o + dir * cyl.radius;
            if let Some(end) = base_ends.get_mut(k) {
                *end = foot;
            }
            let arc = if k == 0 {
                BaseArc {
                    centre: o,
                    from: foot,
                    to: a,
                    face: wall,
                }
            } else {
                BaseArc {
                    centre: o,
                    from: a,
                    to: foot,
                    face: wall,
                }
            };
            arcs.push((k, arc));
        }
        let mut base = Vec::new();
        if let Some(pos) = arcs.iter().position(|(k, _)| *k == 0) {
            base.push(arcs.remove(pos).1);
        }
        base.push(BaseArc {
            centre: on_axis(round_cyl.frame.origin, round_cyl.frame.z, a1),
            from: a1,
            to: a2,
            face: round,
        });
        if let Some(pos) = arcs.iter().position(|(k, _)| *k == 1) {
            base.push(arcs.remove(pos).1);
        }
        let mut clips = Vec::with_capacity(2);
        for ((edge, v, _), &base_end) in [end_a, end_b].iter().zip(&base_ends) {
            let Some(m) = n_plate.cross(base_end - c).unit() else {
                return unsupported("a fillet's corner whose base touches its axis");
            };
            let x = body.edge(*edge);
            let mid = x.curve.as_ref().map_or(body.vertex(x.end).point, |cv| {
                cv.eval(0.5 * (x.range.0 + x.range.1))
            });
            let far = if (mid - body.vertex(*v).point).norm() > tol::LINEAR {
                mid
            } else {
                body.vertex(if x.start == *v { x.end } else { x.start })
                    .point
            };
            let keep = if (far - c).dot(m) >= 0.0 { m } else { -m };
            clips.push(Clip::Plane { origin: c, keep });
        }
        let tool = corner_tool(body, plate, &base, c, t, lift, side)?;
        let &[clip_a, clip_b] = clips.as_slice() else {
            return unsupported("a fillet's corner without a clip at each end");
        };
        out.push(Swallowed {
            arc: Some(ai),
            ends: [(end_a.0, end_a.1, clip_a), (end_b.0, end_b.1, clip_b)],
            plate,
            tool,
        });
    }
    Ok(out)
}

/// One face of the corner tool: its surface, which way is out of the tool, its name, and its loop as
/// edges walked one way (`true` along the edge's parameter), each with its pcurve.
pub(super) struct ToolFace {
    pub surface: Surface,
    /// Out of the solid, at `at`.
    pub out: Vec3,
    pub at: Point3,
    pub names: BTreeSet<FaceName>,
    /// The loop's edges, each with whether it runs along the loop and its pcurve if not projected.
    pub walk: Vec<(EdgeId, bool, Option<PCurve>)>,
}

fn mark(name: &str) -> BTreeSet<FaceName> {
    BTreeSet::from([FaceName::new(MARK, name)])
}

fn corner_tool(
    body: &Brep,
    plate: FaceId,
    base: &[BaseArc],
    c: Point3,
    t: Point3,
    lift: Vec3,
    side: f64,
) -> Result<Brep> {
    let n_plate = outward(body, plate, t);
    let n = base.len();
    let Some(first) = base.first() else {
        return unsupported("a fillet's corner with no base");
    };
    // The base as one rational quadratic, one span to each arc.
    let mut row: Vec<(Point3, f64)> = vec![(first.from, 1.0)];
    // each arc's middle and weight, which the row's arcs were checked to have
    let mut middles = Vec::with_capacity(n);
    for arc in base {
        let Some((mid, w)) = arc_middle(arc.centre, arc.from, arc.to) else {
            return unsupported("a fillet's corner base that turns by a half turn");
        };
        row.push((mid, w));
        row.push((arc.to, 1.0));
        middles.push((mid, w));
    }
    let mut knots_u = vec![0.0; 3];
    for k in 1..n {
        knots_u.extend([k as f64, k as f64]);
    }
    knots_u.extend([n as f64; 3]);
    let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let mut ctrl = Vec::with_capacity(3 * row.len());
    let mut weights = Vec::with_capacity(3 * row.len());
    for &(q, wu) in &row {
        for (j, p) in [q, q + lift, t].into_iter().enumerate() {
            ctrl.push(p);
            weights.push(wu * if j == 1 { s } else { 1.0 });
        }
    }
    let patch = NurbsSurface::new(
        (2, 2),
        (knots_u, knots.clone()),
        (row.len(), 3),
        ctrl,
        Some(weights),
    )?;
    let quad = |pts: [Point3; 3], wm: f64| {
        NurbsCurve::new(2, knots.clone(), pts.to_vec(), Some(vec![1.0, wm, 1.0]))
            .map(|c| Curve::Nurbs(Box::new(c)))
    };

    let mut tool = Brep::new();
    let base_pts: Vec<Point3> = std::iter::once(first.from)
        .chain(base.iter().map(|a| a.to))
        .collect();
    let vb: Vec<VertexId> = base_pts.iter().map(|&p| tool.add_vertex(p)).collect();
    let vtop: Vec<VertexId> = base_pts
        .iter()
        .map(|&p| tool.add_vertex(p + lift))
        .collect();
    let vt = tool.add_vertex(t);
    let mut lows = Vec::with_capacity(n);
    let mut tops = Vec::with_capacity(n);
    // each arc between its two base vertices and its two top ones
    for (((arc, &(mid, w)), lo), hi) in base.iter().zip(&middles).zip(vb.windows(2)).zip(vtop.windows(2)) {
        let (&[l0, l1], &[h0, h1]) = (lo, hi) else {
            continue;
        };
        lows.push(tool.add_edge(quad([arc.from, mid, arc.to], w)?, (0.0, 1.0), l0, l1));
        tops.push(tool.add_edge(
            quad([arc.from + lift, mid + lift, arc.to + lift], w)?,
            (0.0, 1.0),
            h0,
            h1,
        ));
    }
    let ends_of = |vs: &[VertexId]| (vs.first().copied(), vs.last().copied());
    let (
        (Some(&b0), Some(&bn)),
        (Some(vb0), Some(vbn)),
        (Some(vtop0), Some(vtopn)),
    ) = ((base_pts.first(), base_pts.last()), ends_of(&vb), ends_of(&vtop))
    else {
        return unsupported("a fillet's corner with no base");
    };
    let side1 = tool.add_edge(quad([b0, b0 + lift, t], s)?, (0.0, 1.0), vb0, vt);
    let side2 = tool.add_edge(quad([bn, bn + lift, t], s)?, (0.0, 1.0), vbn, vt);
    let pole = tool.add_degenerate_edge(vt, (0.0, 1.0));
    let line =
        |tool: &mut Brep, p: Point3, q: Point3, vp: VertexId, vq: VertexId| -> Result<EdgeId> {
            let Some((l, len)) = Line::through(p, q) else {
                return unsupported("a fillet's corner with no room between its contacts");
            };
            Ok(tool.add_edge(Curve::Line(l), (0.0, len), vp, vq))
        };
    let mut ups = Vec::with_capacity(n + 1);
    for ((&p, &vp), &vq) in base_pts.iter().zip(&vb).zip(&vtop) {
        ups.push(line(&mut tool, p, p + lift, vp, vq)?);
    }
    let flat1 = line(&mut tool, t, b0 + lift, vt, vtop0)?;
    let flat2 = line(&mut tool, t, bn + lift, vt, vtopn)?;
    let (Some(&up0), Some(&upn)) = (ups.first(), ups.last()) else {
        return unsupported("a fillet's corner with no base");
    };

    let affine = |o: (f64, f64), d: (f64, f64)| {
        Some(PCurve::Affine {
            origin: Vec2::new(o.0, o.1),
            dir: Vec2::new(d.0, d.1),
        })
    };
    let plane_through = |p: Point3, nrm: Vec3| -> Result<Surface> {
        let frame = crate::math::Frame::from_axis(p, nrm)?;
        Ok(Surface::Plane(crate::geom::Plane { frame }))
    };
    // the trim plane at base end `end`, `other` the base's other end
    let trim_out = |end: Point3, other: Point3| -> Result<Vec3> {
        let m = n_plate.cross(end - c).unit_or_zero();
        // Out of the tool is towards the piece beside it, away from the other end.
        Ok(if (other - c).dot(m) <= 0.0 {
            m
        } else {
            -m
        })
    };
    let toward_c = |p: Point3| (c - p).unit_or_zero();
    let mid_patch = patch.eval(0.5 * n as f64, 0.5);
    let index = |f: FaceId| -> Result<usize> {
        body.face_ids()
            .position(|g| g == f)
            .ok_or_else(|| Error::Numerical("a fillet's corner on a face not of the body".into()))
    };
    let mut walk = Vec::with_capacity(n + 3);
    for (k, &e) in lows.iter().enumerate() {
        walk.push((e, true, affine((k as f64, 0.0), (1.0, 0.0))));
    }
    walk.push((side2, true, affine((n as f64, 0.0), (0.0, 1.0))));
    walk.push((pole, true, affine((n as f64, 1.0), (-(n as f64), 0.0))));
    walk.push((side1, false, affine((0.0, 0.0), (0.0, 1.0))));
    let mut faces = vec![ToolFace {
        surface: Surface::Nurbs(Box::new(patch)),
        out: toward_c(mid_patch),
        at: mid_patch,
        names: mark("patch"),
        walk,
    }];
    // each arc's face, between its low and top edges and the ups at its ends
    for ((((arc, &(mid, _)), &low), &top), up) in base.iter().zip(&middles).zip(&lows).zip(&tops).zip(ups.windows(2)) {
        let &[up_from, up_to] = up else {
            continue;
        };
        let mid_arc =
            arc.centre + (mid - arc.centre).unit_or_zero() * (arc.from - arc.centre).norm();
        faces.push(ToolFace {
            surface: body.face(arc.face).surface.clone(),
            out: outward(body, arc.face, mid_arc) * side,
            at: mid_arc + lift * 0.5,
            names: mark(&format!("face_{}", index(arc.face)?)),
            walk: vec![
                (low, true, None),
                (up_to, true, None),
                (top, false, None),
                (up_from, false, None),
            ],
        });
    }
    let mut top_walk: Vec<(EdgeId, bool, Option<PCurve>)> =
        tops.iter().map(|&e| (e, true, None)).collect();
    top_walk.push((flat2, false, None));
    top_walk.push((flat1, true, None));
    let centroid = base_pts
        .iter()
        .fold(t, |acc, &p| acc + (p + lift - t) / (n as f64 + 2.0));
    faces.push(ToolFace {
        surface: body.face(plate).surface.clone(),
        out: n_plate * side,
        at: centroid,
        names: mark("plate"),
        walk: top_walk,
    });
    faces.push(ToolFace {
        surface: plane_through(c, trim_out(b0, bn)?)?,
        out: trim_out(b0, bn)?,
        at: (b0 + (b0 + lift) + t) / 3.0,
        names: BTreeSet::new(),
        walk: vec![
            (side1, false, None),
            (up0, true, None),
            (flat1, false, None),
        ],
    });
    faces.push(ToolFace {
        surface: plane_through(c, trim_out(bn, b0)?)?,
        out: trim_out(bn, b0)?,
        at: (bn + (bn + lift) + t) / 3.0,
        names: BTreeSet::new(),
        walk: vec![
            (side2, true, None),
            (flat2, true, None),
            (upn, false, None),
        ],
    });
    assemble(tool, faces, "a fillet's corner tool")
}

/// `tool`'s vertices and edges closed into one solid by `faces`, each loop turned to run
/// anticlockwise about its face's outward normal; `what` names it in an error.
pub(super) fn assemble(mut tool: Brep, faces: Vec<ToolFace>, what: &str) -> Result<Brep> {
    let mut ids = Vec::with_capacity(faces.len());
    for f in faces {
        let uv = f.surface.project(f.at);
        let natural = f.surface.normal(uv.x, uv.y);
        let reversed = natural.dot(f.out) < 0.0;
        // The loop runs with the tool on its left seen from outside: anticlockwise about `out`.
        let mut walk = f.walk;
        if newell(&tool, &walk).dot(f.out) < 0.0 {
            walk.reverse();
            for w in &mut walk {
                w.1 = !w.1;
            }
        }
        let id = tool.add_face(f.surface.clone(), reversed, f.names);
        let mut specs = Vec::with_capacity(walk.len());
        // Where the last coedge ended: each seed continues from it, so a loop across a periodic
        // surface's seam keeps one image of its parameters all the way round.
        let mut last: Option<Vec2> = None;
        for (e, along, pcurve) in walk {
            let x = tool.edge(e);
            let pcurve = match pcurve {
                Some(p) => p,
                None => {
                    let (t0, t1) = if along {
                        x.range
                    } else {
                        (x.range.1, x.range.0)
                    };
                    let at = |tt: f64| {
                        x.curve
                            .as_ref()
                            .map_or(tool.vertex(x.start).point, |cv| cv.eval(tt))
                    };
                    let seed = near(&f.surface, f.surface.project(at(t0)), last);
                    let mut uv = seed;
                    for k in 1..=16 {
                        let p = at(t0 + (t1 - t0) * k as f64 / 16.0);
                        uv = near(&f.surface, f.surface.project(p), Some(uv));
                    }
                    last = Some(uv);
                    PCurve::Projected { seed }
                }
            };
            specs.push(CoedgeSpec {
                edge: e,
                reversed: !along,
                pcurve,
            });
        }
        tool.add_loop(id, specs);
        ids.push(id);
    }
    let shell = tool.add_shell(ids);
    tool.add_solid(vec![shell]);
    let problems = check::check(&tool);
    if let Some(p) = problems.first() {
        return Err(Error::Numerical(format!("{what} does not check: {p}")));
    }
    Ok(tool)
}

/// The Newell normal of a loop walked through its edges' sample points: anticlockwise about it.
fn newell(tool: &Brep, walk: &[(EdgeId, bool, Option<PCurve>)]) -> Vec3 {
    let mut pts = Vec::new();
    for &(e, along, _) in walk {
        let x = tool.edge(e);
        let Some(c) = &x.curve else {
            pts.push(tool.vertex(x.start).point);
            continue;
        };
        for i in 0..8 {
            let s = i as f64 / 8.0;
            let s = if along { s } else { 1.0 - s };
            pts.push(c.eval(x.range.0 + (x.range.1 - x.range.0) * s));
        }
    }
    let mut n = Vec3::ZERO;
    // each point with the next, the last with the first
    for (p, q) in pts.iter().zip(pts.iter().cycle().skip(1)) {
        n += p.cross(*q);
    }
    n
}
