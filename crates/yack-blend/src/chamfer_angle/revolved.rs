// yack: chamfer_angle on a whole circular edge between two faces that turn about its axis, worked in the meridian half-plane (M4.8).
//! The bevel of a rim. Where the edge is a whole circle and both of its faces are surfaces of
//! revolution about the circle's own axis whose meridians are lines or circles (a plane square to
//! the axis, a cylinder, a cone, a sphere or a torus about it), the chamfer is the same in every
//! half-plane through the axis, so it is worked once in that half-plane's `(r, z)` coordinates.
//!
//! From the edge's point `E` the setback runs `distance` along face A's meridian to `p1`. The
//! bevel leaves `p1` at `angle` to A, into the material: back toward `E` along A's tangent `w₁` and
//! across A's outward normal `n₁`, `−w₁·cos a + s·n₁·sin a`, where `s` is the sign of B's direction
//! from `E` against A's normal (negative at a convex rim, positive at a concave one). It ends at
//! `q`, its nearest crossing with B's meridian. The region `E → p1` along A, `p1 → q`, `q → E` along
//! B, turned once about the axis, is cut from the body at a convex rim and added at a concave one.
//!
//! Every reading comes from the faces themselves: the normals from each surface and its
//! orientation, the direction into each face from its loop. The strategy declines wherever it is
//! unsure: an edge that is not a whole circle, a face that is anything but a band about the axis
//! (an edge on it neither a circle about the axis nor its seam), faces tangent at the rim, a
//! setback or bevel that runs off its face, and a result whose volume is not the body's less (or
//! plus) the ring's, which is what a ring reaching past the two faces into the rest of the body
//! would give.

use std::f64::consts::TAU;

use yack_base::kernel::{Error, Result};
use yack_boolean as boolean;
use yack_build::sweep::revolve;
use yack_geom::{Curve, Surface};
use yack_math::kernel::{Frame, Point3, Vec2, Vec3};
use yack_profile::{NamedSegment, Profile};
use yack_tol as tol;
use yack_topo::{Brep, CoedgeId, EdgeId, FaceId, FaceName};

/// The feature the ring's faces are named under until the boolean has placed them.
const TOOL: &str = "\u{1}ach";

/// How far a reading may sit off the meridian it should lie on: a direction's component across
/// the half-plane, a boundary point's distance from the meridian.
const ON: f64 = 1e-6; // tol: a reading's slack against the surface it was read from, not a feature size

/// How nearly the result's volume must be the body's less (or plus) the ring's.
const AGREE: f64 = 1e-9; // tol: a share of the volume, not a length

fn decline<T>(why: impl Into<String>) -> Result<T> {
    Err(Error::Unsupported(format!("revolved chamfer: {}", why.into())))
}

/// The axis of the edge's circle, with the half-plane through `E`: `x` toward `E`, `a` along the
/// axis, `y` the circle's own direction at `E`.
struct Axis {
    c: Point3,
    x: Vec3,
    y: Vec3,
    a: Vec3,
}

impl Axis {
    /// `p` in the half-plane's `(r, z)`.
    fn rz(&self, p: Point3) -> Vec2 {
        let d = p - self.c;
        let z = d.dot(self.a);
        Vec2::new((d - self.a * z).norm(), z)
    }

    fn along(&self, z: Vec3) -> bool {
        z.cross(self.a).norm() <= tol::ANGULAR
    }

    fn on(&self, p: Point3) -> bool {
        self.rz(p).x <= tol::LINEAR
    }

    /// The sketch plane the ring's section is drawn on: `x` radial, `y` along the axis.
    fn sketch(&self) -> Frame {
        Frame { origin: self.c, x: self.x, y: self.a, z: self.x.cross(self.a) }
    }
}

/// A face's meridian through `E`, with arc length measured from `E` into the face.
#[derive(Clone, Copy, Debug)]
enum Meridian {
    /// The line from `e` along the unit `u`.
    Line { e: Vec2, u: Vec2 },
    /// The circle about `c` of radius `r`, from the angle `a0` (at `E`) the way `sense` (±1) turns.
    Circle { c: Vec2, r: f64, a0: f64, sense: f64 },
}

impl Meridian {
    fn angle(&self, s: f64) -> f64 {
        match *self {
            Meridian::Line { .. } => 0.0,
            Meridian::Circle { r, a0, sense, .. } => a0 + sense * s / r,
        }
    }

    fn point(&self, s: f64) -> Vec2 {
        match *self {
            Meridian::Line { e, u } => e + u * s,
            Meridian::Circle { c, r, .. } => {
                let t = self.angle(s);
                c + Vec2::new(t.cos(), t.sin()) * r
            }
        }
    }

    fn tangent(&self, s: f64) -> Vec2 {
        match *self {
            Meridian::Line { u, .. } => u,
            Meridian::Circle { sense, .. } => {
                let t = self.angle(s);
                Vec2::new(-t.sin(), t.cos()) * sense
            }
        }
    }

    /// The face's outward normal at `s`, given it is `n0` at `E`.
    fn normal(&self, n0: Vec2, s: f64) -> Vec2 {
        match *self {
            Meridian::Line { .. } => n0,
            Meridian::Circle { c, r, .. } => {
                let e = self.point(0.0);
                let outward = if n0.dot(e - c) >= 0.0 { 1.0 } else { -1.0 };
                (self.point(s) - c) * (outward / r)
            }
        }
    }

    /// How far `p` lies off the whole meridian.
    fn off(&self, p: Vec2) -> f64 {
        match *self {
            Meridian::Line { e, u } => (p - e).cross(u).abs(),
            Meridian::Circle { c, r, .. } => ((p - c).norm() - r).abs(),
        }
    }

    /// The arc length from `E` to `p` (on the meridian): signed along a line, in `[0, 2πr)` round a
    /// circle, a point a tolerance short of a whole turn read as `E` itself.
    fn param(&self, p: Vec2) -> f64 {
        match *self {
            Meridian::Line { e, u } => (p - e).dot(u),
            Meridian::Circle { c, r, a0, sense } => {
                let d = p - c;
                let turn = (sense * (d.y.atan2(d.x) - a0)).rem_euclid(TAU) * r;
                if turn >= TAU * r - tol::LINEAR { 0.0 } else { turn }
            }
        }
    }

    /// Each `t > 0` where the ray `p + t·d` (unit `d`) crosses the whole meridian.
    fn hits(&self, p: Vec2, d: Vec2) -> Vec<f64> {
        let all = match *self {
            Meridian::Line { e, u } => {
                let det = d.cross(u);
                if det.abs() <= tol::ANGULAR {
                    return Vec::new();
                }
                vec![(e - p).cross(u) / det]
            }
            Meridian::Circle { c, r, .. } => {
                let f = p - c;
                let b = f.dot(d);
                let disc = b * b - (f.dot(f) - r * r);
                if disc < 0.0 {
                    return Vec::new();
                }
                let root = disc.sqrt();
                vec![-b - root, -b + root]
            }
        };
        all.into_iter().filter(|&t| t > tol::LINEAR).collect()
    }

    /// The piece of the meridian from `s0` to `s1`, as a section segment named `name`.
    fn segment(&self, s0: f64, s1: f64, name: &str) -> NamedSegment {
        match *self {
            Meridian::Line { .. } => NamedSegment::line(self.point(s0), self.point(s1), name),
            Meridian::Circle { c, r, sense, .. } => {
                NamedSegment::arc(c, r, self.angle(s0), sense * (s1 - s0) / r, name)
            }
        }
    }
}

/// One of the edge's two faces, read in the half-plane: its meridian, its outward normal at `E`,
/// and how far along the meridian from `E` it reaches.
struct Side {
    meridian: Meridian,
    n0: Vec2,
    reach: f64,
}

/// The face that `coedge` (a use of the rim) bounds, read in the half-plane of `ax`.
fn side(body: &Brep, ax: &Axis, e: Point3, coedge: CoedgeId) -> Result<Side> {
    let face_id = body.coedge_face(coedge);
    let face = body.face(face_id);
    let surface = &face.surface;
    let uv = surface.project(e);
    let flip = if face.reversed { -1.0 } else { 1.0 };
    let n = surface.normal(uv.x, uv.y) * flip;
    // The loop has the face's material on its left seen from outside: `n × T` points into it.
    let t = if body.coedge(coedge).reversed { -ax.y } else { ax.y };
    let w3 = n.cross(t);
    if n.dot(ax.y).abs() > ON || w3.dot(ax.y).abs() > ON {
        return decline("a face's normal at the rim leaves the half-plane through the axis");
    }
    let (n0, w) = (Vec2::new(n.dot(ax.x), n.dot(ax.a)), Vec2::new(w3.dot(ax.x), w3.dot(ax.a)));
    let e2 = ax.rz(e);
    let circle = |c: Vec2, r: f64| -> Result<Meridian> {
        if ((e2 - c).norm() - r).abs() > ON {
            return decline("the rim does not lie on a face's meridian circle");
        }
        let d = e2 - c;
        Ok(Meridian::Circle {
            c,
            r,
            a0: d.y.atan2(d.x),
            sense: if d.cross(w) >= 0.0 { 1.0 } else { -1.0 },
        })
    };
    let meridian = match surface {
        Surface::Plane(p) if ax.along(p.frame.z) => Meridian::Line { e: e2, u: w },
        Surface::Cylinder(k) if ax.along(k.frame.z) && ax.on(k.frame.origin) => {
            Meridian::Line { e: e2, u: w }
        }
        Surface::Cone(k) if ax.along(k.frame.z) && ax.on(k.frame.origin) => {
            Meridian::Line { e: e2, u: w }
        }
        Surface::Sphere(k) if ax.on(k.frame.origin) => {
            circle(Vec2::new(0.0, ax.rz(k.frame.origin).y), k.radius)?
        }
        Surface::Torus(k)
            if ax.along(k.frame.z) && ax.on(k.frame.origin) && k.major > k.minor =>
        {
            circle(Vec2::new(k.major, ax.rz(k.frame.origin).y), k.minor)?
        }
        other => {
            return decline(format!(
                "a face at the rim is a {} that does not turn about the rim's axis",
                other.kind()
            ));
        }
    };
    if (meridian.tangent(0.0) - w).norm() > ON || n0.dot(w).abs() > ON {
        return decline("a face's direction from the rim is not along its meridian");
    }
    let mut reach = reach(body, ax, face_id, &meridian)?;
    if reach <= tol::LINEAR {
        // A face bounded by the rim alone: a plane's disc, reaching in to the axis.
        match (surface, meridian) {
            (Surface::Plane(_), Meridian::Line { e, u }) if u.x < -1.0 + ON => reach = e.x,
            _ => return decline("a face at the rim reaches nowhere from it"),
        }
    }
    Ok(Side { meridian, n0, reach })
}

/// How far along `meridian` from `E` the face `f` reaches, where it is a band about the axis:
/// every edge on it a circle about the axis, a pole, or its seam.
fn reach(body: &Brep, ax: &Axis, f: FaceId, meridian: &Meridian) -> Result<f64> {
    let mut far = 0.0_f64;
    for &l in &body.face(f).loops {
        for &c in &body.lp(l).coedges {
            let id = body.coedge(c).edge;
            let edge = body.edge(id);
            let (t0, t1) = edge.range;
            let points: Vec<Point3> = match &edge.curve {
                None => vec![body.vertex(edge.start).point],
                Some(g @ Curve::Circle(k)) if ax.along(k.frame.z) && ax.on(k.frame.origin) => {
                    vec![g.eval(t0)]
                }
                Some(g) if seam(body, id, f) => {
                    (0..=8).map(|i| g.eval(t0 + (t1 - t0) * f64::from(i) / 8.0)).collect()
                }
                Some(_) => {
                    return decline("a face at the rim has an edge that is not a circle about the axis");
                }
            };
            for p in points {
                let p2 = ax.rz(p);
                if meridian.off(p2) > ON {
                    return decline("a face at the rim is not a band about the axis");
                }
                let s = meridian.param(p2);
                if s < -tol::LINEAR {
                    return decline("a face at the rim reaches back past the rim");
                }
                far = far.max(s);
            }
        }
    }
    Ok(far)
}

/// Whether `e` is a seam of `f`: an edge `f` alone uses, twice.
fn seam(body: &Brep, e: EdgeId, f: FaceId) -> bool {
    let uses = body.edge_uses(e);
    uses.len() == 2 && uses.iter().all(|&c| body.coedge_face(c) == f)
}

/// `body` with the rim `edge` bevelled: set back `distance` along `face`, the bevel at `angle`
/// (radians) to it; the bevel face is `<feature>.bevel_0`, the call's one edge.
pub(super) fn chamfer(
    body: &Brep,
    edge: EdgeId,
    face: FaceId,
    distance: f64,
    angle: f64,
    feature: &str,
) -> Result<Brep> {
    let rim = body.edge(edge);
    let Some(Curve::Circle(k)) = &rim.curve else {
        return decline("the edge is not a circle");
    };
    if rim.range.1 - rim.range.0 < TAU - tol::ANGULAR {
        return decline("the edge is not a whole circle");
    }
    if k.radius <= tol::LINEAR {
        return decline("the edge's circle has no radius");
    }
    let ax = Axis { c: k.frame.origin, x: k.frame.x, y: k.frame.y, a: k.frame.z };
    let e = k.frame.origin + k.frame.x * k.radius;
    let uses = body.edge_uses(edge);
    let [one, two] = uses else {
        return decline("the edge does not lie between exactly two faces");
    };
    let (ca, cb) = match (body.coedge_face(*one) == face, body.coedge_face(*two) == face) {
        (true, false) => (*one, *two),
        (false, true) => (*two, *one),
        _ => return Err(Error::InvalidInput("the face is not one of the edge's two".into())),
    };
    let (a, b) = (side(body, &ax, e, ca)?, side(body, &ax, e, cb)?);

    // B's way from the rim against A's normal: into A's air at a concave rim, under A at a convex.
    let turn = b.meridian.tangent(0.0).dot(a.n0);
    if turn.abs() <= ON {
        return decline("the faces are tangent at the rim");
    }
    let s = turn.signum();
    if distance >= a.reach - tol::LINEAR {
        return decline("the setback runs off the face");
    }
    let p1 = a.meridian.point(distance);
    let (w1, n1) = (a.meridian.tangent(distance), a.meridian.normal(a.n0, distance));
    let (sin, cos) = angle.sin_cos();
    let dir = w1 * -cos + n1 * (s * sin);
    let e2 = a.meridian.point(0.0);
    let Some(q) = b
        .meridian
        .hits(p1, dir)
        .into_iter()
        .map(|t| p1 + dir * t)
        .min_by(|x, y| x.dist(e2).total_cmp(&y.dist(e2)))
    else {
        return decline("the bevel never meets the other face");
    };
    let tq = b.meridian.param(q);
    if !(tq > tol::LINEAR && tq < b.reach - tol::LINEAR) {
        return decline("the bevel meets the other face's surface beyond the face");
    }
    let q = b.meridian.point(tq);

    let section = Profile::outline(vec![
        a.meridian.segment(0.0, distance, "a"),
        NamedSegment::line(p1, q, "bevel"),
        b.meridian.segment(tq, 0.0, "b"),
    ]);
    let ring = revolve(&section, &ax.sketch(), ax.c, ax.a, TAU, TOOL)?;
    let mut out = if s < 0.0 { boolean::difference(body, &ring)? } else { boolean::union(body, &ring)? };

    // The ring lies wholly in the body (or wholly outside it) exactly when the volumes add up.
    let (whole, part, got) =
        (yack_measure::volume(body), yack_measure::volume(&ring), yack_measure::volume(&out));
    let want = whole + s * part;
    if (got - want).abs() > AGREE * whole.abs().max(part) {
        return decline(format!(
            "the bevel's ring reaches past the rim's faces (volume {got}, not {want})"
        ));
    }
    if out.face_ids().any(|f| {
        let names = &out.face(f).names;
        !names.is_empty() && names.iter().all(|n| n.feature == TOOL && n.face != "bevel")
    }) {
        return decline("a side of the bevel's ring is left as a face of its own");
    }
    out.rename_faces(|n| {
        if n.feature != TOOL {
            Some(n.clone())
        } else if n.face == "bevel" {
            Some(FaceName::new(feature, "bevel_0"))
        } else {
            None
        }
    });
    Ok(out)
}
