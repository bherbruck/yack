// yack: the swept solid as the start united with what each of its leading faces sweeps (M4.1 step 1).
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/sweep_faces.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: rewritten onto yack's crates, not ported: each leading face's sweep is a solid built from
//   its parts (yack-build's `solid_of`), united with the start by yack-boolean, and checked; for a
//   slide or a turn, a face the motion crosses split along its contact (a plane, or a torus's
//   coaxial cylinder) by yack-boolean first. Screws and everything else decline.
//! What any solid passes through as it slides, turns or screws: the solid where it starts, with
//! what each of its *leading* faces sweeps.
//!
//! A point the moving solid covers at some instant is inside it where it starts, or enters it
//! later across the part of its boundary moving towards it: where the face's outward normal leads
//! the motion. A face that leads all over sweeps a solid without folding: the face where it
//! starts, the face where it ends, and the sides its edges sweep. The solid swept is the start
//! united with those solids.
//!
//! A turn short of a whole one goes the same way (each point moves on a circle about the axis):
//! see [`turn`] for which faces it builds.
//!
//! Built so far for a slide: a plane leads, trails or runs along it all over, as does a cylinder
//! slid along its axis. A cylinder the slide crosses touches it along the plane through its axis
//! square to the slide's part across the axis, a sphere along the plane through its centre square
//! to the slide, a cone along the plane of its two contact generators, and a torus slid along or
//! square to its axis along a plane and its major radius's cylinder: the solid is split there
//! first, and each side's faces then lead or trail all over. The sweep of the whole is the union
//! of its pieces' sweeps, so the pieces' prisms are united with the start at the end, all at once.
//! A torus slid aslant, and splines, decline.

use std::collections::{BTreeMap, BTreeSet};
use yack_base::kernel::{Error, Result};
use yack_build::build::{PartFace, Parts, param_of, parts_of, solid_of};

use yack_geom::{Circle, Curve, Cylinder, Ellipse, Extrusion, Line, Plane, Surface};
use yack_math::kernel::{Frame, Point3, Transform, Vec3};
use yack_topo::{Brep, FaceName};

use crate::Motion;

mod turn;

/// How a face stands to a slide.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Stand {
    Leads,
    Trails,
    Along,
    /// Its contact lies on one cut, or on two: on each side of them it leads all over or trails
    /// all over. Which, the face's own normals say once splits have left it on one side of each.
    Contact(Cut, Option<Cut>),
}

/// A surface a solid is split along before its faces are swept, and that a piece then lies wholly
/// on one side of.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Cut {
    /// The plane through `origin` square to the unit `normal`.
    Plane { origin: Point3, normal: Vec3 },
    /// The cylinder of `radius` about the line through `origin` along the unit `axis`.
    Tube { origin: Point3, axis: Vec3, radius: f64 },
}

impl Cut {
    /// Whether `self` and `other` are one surface.
    fn same(&self, other: &Cut) -> bool {
        let parallel = |a: Vec3, b: Vec3| a.cross(b).norm() <= yack_tol::ANGULAR;
        match (self, other) {
            (Cut::Plane { origin: o, normal: n }, Cut::Plane { origin: p, normal: m }) => {
                parallel(*n, *m) && (*p - *o).dot(*n).abs() <= yack_tol::LINEAR
            }
            (
                Cut::Tube { origin: o, axis: a, radius: r },
                Cut::Tube { origin: p, axis: b, radius: s },
            ) => {
                parallel(*a, *b)
                    && (*p - *o).cross(*a).norm() <= yack_tol::LINEAR
                    && (r - s).abs() <= yack_tol::LINEAR
            }
            _ => false,
        }
    }

    /// Whether all of `piece` lies on one side of `self` already (touching it at most), so that a
    /// split would leave one side empty: its bounding box's corners say, for a plane.
    fn misses(&self, piece: &Brep) -> bool {
        let Cut::Plane { origin, normal } = *self else {
            return false;
        };
        let b = yack_topo::bbox::bbox(piece);
        let (mut above, mut below) = (false, false);
        for i in 0..8 {
            let pick = |bit: usize, lo: f64, hi: f64| if i & bit == 0 { lo } else { hi };
            let corner = Point3::new(
                pick(1, b.min.x, b.max.x),
                pick(2, b.min.y, b.max.y),
                pick(4, b.min.z, b.max.z),
            );
            let h = (corner - origin).dot(normal);
            above |= h > yack_tol::LINEAR;
            below |= h < -yack_tol::LINEAR;
        }
        !(above && below)
    }

    /// `piece` cut along `self`: the two sides, either of which may be empty.
    fn split(&self, piece: &Brep) -> Result<(Brep, Brep)> {
        match *self {
            Cut::Plane { origin, normal } => yack_boolean::split(piece, origin, normal),
            Cut::Tube { origin, axis, radius } => {
                // A cylinder reaching past the piece both ways along the axis: inside it, outside.
                let b = yack_topo::bbox::bbox(piece);
                let reach = 2.0 * (b.diagonal() + b.center().dist(origin)) + 1.0;
                let frame = Frame::new(origin - axis * reach, axis, axis.any_perpendicular())?;
                let tool = yack_build::primitive::cylinder(&frame, radius, 2.0 * reach, "cut")?;
                Ok((
                    yack_boolean::intersection(piece, &tool)?,
                    yack_boolean::difference(piece, &tool)?,
                ))
            }
        }
    }
}

/// How many cuts a slide may split a solid along before it declines.
const MAX_SPLITS: usize = 24;

/// How far a face's outward normal must lean towards or away from the slide (its cosine) for a
/// sample of it to count as leading or trailing: a sample nearer square than this is on the
/// contact, and a face whose samples lean both ways past it declines.
const CLEAR: f64 = 1e-3; // tol: a cosine, not a length; samples sit a quarter of an edge from its ends

/// How the solid moves from where its sweep starts.
#[derive(Clone, Copy, Debug)]
enum How {
    /// A slide by `by`, along the unit `dir`.
    Slide { by: Vec3, dir: Vec3 },
    /// A turn by `angle` (more than 0, at most a whole turn, which is exactly `TAU`),
    /// counter-clockwise looking down the unit `axis`, about the line through `origin`.
    Turn { origin: Point3, axis: Vec3, angle: f64 },
}

/// A sweep: how the solid moves, and the feature its new faces are named under.
struct Sweep<'a> {
    how: How,
    feature: &'a str,
}

impl Sweep<'_> {
    /// Whether this is a whole turn: every point goes all the way round, and back where it began.
    fn whole(&self) -> bool {
        matches!(self.how, How::Turn { angle, .. } if angle >= std::f64::consts::TAU)
    }

    /// How fast `p` moves (for a turn, per radian).
    fn velocity(&self, p: Point3) -> Vec3 {
        match self.how {
            How::Slide { dir, .. } => dir,
            How::Turn { origin, axis, .. } => axis.cross(p - origin),
        }
    }

    /// Where the motion takes the solid at its end.
    fn moved(&self) -> Transform {
        match self.how {
            How::Slide { by, .. } => Transform::translation(by),
            How::Turn { .. } if self.whole() => Transform::translation(Vec3::ZERO),
            How::Turn { origin, axis, angle } => Transform::rotation(origin, axis, angle),
        }
    }

    /// The path `p` takes, from where it starts to where it ends: none for a point that stays.
    fn rail(&self, p: Point3) -> Result<Option<Curve>> {
        match self.how {
            How::Slide { dir, .. } => Ok(Some(Curve::Line(Line { origin: p, dir }))),
            How::Turn { origin, axis, .. } => {
                let foot = origin + axis * axis.dot(p - origin);
                let out = p - foot;
                if out.norm() <= yack_tol::LINEAR {
                    return Ok(None);
                }
                let frame = Frame::new(foot, axis, out)?;
                Ok(Some(Curve::Circle(Circle { frame, radius: out.norm() })))
            }
        }
    }

    /// How `face` of `parts` stands to the motion.
    fn stand(&self, face: &PartFace, parts: &Parts) -> Result<Stand> {
        match self.how {
            How::Slide { dir, .. } => stand(face, parts, dir),
            How::Turn { origin, axis, .. } => turn::stand(face, origin, axis),
        }
    }

    /// The surface a piece of boundary sweeps, and whether the side faces against it.
    fn side(
        &self,
        curve: &Curve,
        ends: (Point3, Point3),
        reversed: bool,
    ) -> Result<(Surface, bool)> {
        match self.how {
            How::Slide { dir, .. } => side_surface(curve, ends, reversed, dir),
            How::Turn { origin, axis, angle } => {
                turn::side(&seam_safe(curve, ends)?, ends, reversed, origin, axis, angle)
            }
        }
    }
}

/// [`crate::swept`] by faces: a slide, or a turn.
pub(crate) fn swept(body: &Brep, motion: &Motion, feature: &str) -> Result<Brep> {
    match motion {
        Motion::Slide { by } => {
            let length = by.norm();
            if length <= yack_tol::LINEAR {
                return Err(Error::InvalidInput(format!("a slide of {length:e} mm moves nothing")));
            }
            let how = How::Slide { by: *by, dir: *by * (1.0 / length) };
            built(body, &Sweep { how, feature })
        }
        Motion::Turn { axis_point, axis, from, to } => {
            let axis =
                axis.unit().ok_or_else(|| Error::InvalidInput("a turn needs an axis".into()))?;
            let (a0, a1) = (from.min(*to), from.max(*to));
            let angle = a1 - a0;
            if angle <= yack_tol::ANGULAR {
                return Err(Error::InvalidInput("a turn through no angle moves nothing".into()));
            }
            // A whole turn or more covers what a whole turn does.
            let angle = if angle >= std::f64::consts::TAU - yack_tol::ANGULAR {
                std::f64::consts::TAU
            } else {
                angle
            };
            // The solid as given is at angle 0: the sweep starts where it is at the lesser angle.
            let mut start = body.clone();
            start.transform(&Transform::rotation(*axis_point, axis, a0));
            let how = How::Turn { origin: *axis_point, axis, angle };
            built(&start, &Sweep { how, feature })
        }
        Motion::Screw { .. } => {
            Err(Error::Unsupported("by faces: screws are not built yet".into()))
        }
    }
}

/// Everything `body` passes through on `sweep`.
fn built(body: &Brep, sweep: &Sweep) -> Result<Brep> {
    let mut prisms = Vec::new();
    let built = prisms_of(body, sweep, &[], &mut prisms).and_then(|()| {
        let mut all: Vec<&Brep> = vec![body];
        all.extend(prisms.iter());
        yack_boolean::union_all(&all)
    });
    // The motion is sound by now: whatever the pieces this builds say against themselves is this
    // strategy's failure, a decline, never the caller's input.
    built.map_err(|e| match e {
        Error::InvalidInput(why) => Error::Unsupported(format!("by faces: {why}")),
        other => other,
    })
}

/// Adds to `prisms` what each leading face of each solid of `body` sweeps on `slide`, splitting a
/// solid along a curved face's contact first; `sides` are the cuts earlier splits left `body` on
/// one side of.
fn prisms_of(body: &Brep, sweep: &Sweep, sides: &[Cut], prisms: &mut Vec<Brep>) -> Result<()> {
    let solids: Vec<_> = body.solid_ids().collect();
    for &solid in &solids {
        let piece =
            if solids.len() == 1 { body.clone() } else { yack_boolean::solids_of(body, &[solid]) };
        let parts = parts_of(&piece)?;
        let mut leading = Vec::new();
        let mut cut = None;
        for (k, face) in parts.faces.iter().enumerate() {
            match settled(face, &parts, sweep.stand(face, &parts)?, sides, sweep)? {
                Stand::Leads => leading.push(k),
                Stand::Trails | Stand::Along => {}
                Stand::Contact(next, _) => {
                    cut = Some(next);
                    break;
                }
            }
        }
        if let Some(cut) = cut {
            if sides.len() >= MAX_SPLITS {
                return Err(Error::Unsupported(format!(
                    "by faces: a solid with more than {MAX_SPLITS} contacts on a slide"
                )));
            }
            if cut.misses(&piece) {
                let mut more = sides.to_vec();
                more.push(cut);
                prisms_of(&piece, sweep, &more, prisms)?;
                continue;
            }
            let (above, below) = cut.split(&piece)?;
            for half in [above, below] {
                let mut more = sides.to_vec();
                more.push(cut);
                prisms_of(&half, sweep, &more, prisms)?;
            }
            continue;
        }
        // A convex piece's leading faces, slid, sweep one prism together: a line along the
        // slide leaves a convex solid once, so no two of their sweeps overlap, and the sides
        // between two of them cancel. Elsewhere each leading face sweeps its own.
        if matches!(sweep.how, How::Slide { .. }) && !leading.is_empty() && convex(&parts)? {
            prisms.push(prism(&parts, &leading, sweep)?);
        } else {
            for k in leading {
                prisms.push(prism(&parts, &[k], sweep)?);
            }
        }
    }
    Ok(())
}

/// `stand` for `face` of `parts`, a contact settled where earlier splits left the piece on one
/// side of each of its cuts: there the face leads all over or trails all over, and samples of its
/// normal along its boundary say which (a decline where they disagree). Where a cut is still to
/// come, the contact names it first.
fn settled(
    face: &PartFace,
    parts: &Parts,
    stand: Stand,
    sides: &[Cut],
    sweep: &Sweep,
) -> Result<Stand> {
    let Stand::Contact(first, second) = stand else {
        return Ok(stand);
    };
    let done = |c: &Cut| sides.iter().any(|side| side.same(c));
    match [Some(first), second].into_iter().flatten().find(|c| !done(c)) {
        Some(next) => Ok(Stand::Contact(next, None)),
        None => leans(face, parts, sweep),
    }
}

/// Whether `face`, known to lead all over or trail all over, leads or trails: from its outward
/// normal a little inside it from a quarter, a half and three quarters along each piece of its
/// boundary. Inside, not on it: a face's whole boundary can lie on its contacts (a quarter of a
/// torus cut along both of its contacts), where every normal is square to the slide.
fn leans(face: &PartFace, parts: &Parts, sweep: &Sweep) -> Result<Stand> {
    let outward = |p: Point3| {
        let uv = face.surface.project(p);
        let n = face.surface.normal(uv.x, uv.y);
        if face.reversed { -n } else { n }
    };
    // Each sample on the boundary, and the way the face's loop runs there.
    let mut samples = Vec::new();
    for &(e, reversed) in face.loops.iter().flatten() {
        let (curve, a, b) = parts.edges.get(e).ok_or_else(|| missing("edge"))?;
        let pa = *parts.points.get(*a).ok_or_else(|| missing("point"))?;
        let pb = *parts.points.get(*b).ok_or_else(|| missing("point"))?;
        let t0 = param_of(curve, pa);
        let t1 = if a == b {
            t0 + curve.period().ok_or_else(|| missing("period"))?
        } else {
            middle_param(curve, (pa, pb)) * 2.0 - t0
        };
        for f in [0.25, 0.5, 0.75] {
            let (p, d) = curve.d1(t0 + f * (t1 - t0));
            samples.push((p, if reversed { -d } else { d }));
        }
    }
    let mut span = yack_math::kernel::Aabb::EMPTY;
    for &(p, _) in &samples {
        span.add(p);
    }
    // A step in from the boundary: a fiftieth of the face's reach, along the face (its material
    // lies on the left of the loop seen from outside, `n × travel`).
    let step = span.diagonal() / 50.0;
    let (mut most, mut least) = (f64::NEG_INFINITY, f64::INFINITY);
    for (p, travel) in samples {
        let Some(inward) = outward(p).cross(travel).unit() else {
            continue;
        };
        let q = p + inward * step;
        let Some(v) = sweep.velocity(q).unit() else {
            continue;
        };
        let s = outward(q).dot(v);
        most = most.max(s);
        least = least.min(s);
    }
    if most > CLEAR && least > -CLEAR {
        Ok(Stand::Leads)
    } else if least < -CLEAR && most < CLEAR {
        Ok(Stand::Trails)
    } else {
        Err(Error::Unsupported(format!(
            "by faces: a {} face that leans both ways after its split",
            face.surface.kind()
        )))
    }
}

/// [`leans`] for a slide along the unit `dir`.
fn leans_along(face: &PartFace, parts: &Parts, dir: Vec3) -> Result<Stand> {
    let how = How::Slide { by: dir, dir };
    leans(face, parts, &Sweep { how, feature: "" })
}

/// How `face` stands to a slide along the unit `dir`: whole, or a decline where it would have to
/// be split along its contact first.
fn stand(face: &PartFace, parts: &Parts, dir: Vec3) -> Result<Stand> {
    let out = |n: Vec3| if face.reversed { -n } else { n };
    let along = |axis: Vec3| axis.cross(dir).norm() <= yack_tol::ANGULAR;
    match &face.surface {
        Surface::Plane(p) => {
            let s = out(p.frame.z).dot(dir);
            Ok(if s > yack_tol::ANGULAR {
                Stand::Leads
            } else if s < -yack_tol::ANGULAR {
                Stand::Trails
            } else {
                Stand::Along
            })
        }
        Surface::Cylinder(c) if along(c.frame.z) => Ok(Stand::Along),
        Surface::Extrusion(e) if along(e.dir) => Ok(Stand::Along),
        // The outward normal of a cylinder's face is ±radial: square to the slide on the plane
        // through the axis square to the slide's part across the axis.
        Surface::Cylinder(c) => {
            let across = dir - c.frame.z * c.frame.z.dot(dir);
            let normal = across
                .unit()
                .ok_or_else(|| Error::Unsupported("by faces: a degenerate cylinder".into()))?;
            Ok(Stand::Contact(Cut::Plane { origin: c.frame.origin, normal }, None))
        }
        Surface::Sphere(sp) => {
            Ok(Stand::Contact(Cut::Plane { origin: sp.frame.origin, normal: dir }, None))
        }
        Surface::Torus(t) => torus_stand(t, dir),
        Surface::Cone(c) => cone_stand(face, parts, c, dir),
        other => Err(Error::Unsupported(format!(
            "by faces: a {} face the slide crosses is not split along its contact yet",
            other.kind()
        ))),
    }
}

/// How a cone's face stands to a slide along the unit `dir`. Its outward normal is (up to sign)
/// `cos a·radial(u) − sin a·z`, `a` the half angle: square to the slide where
/// `radial(u)·d⊥ = tan a·(dir·z)`, `d⊥` the slide's part across the axis. Where that has two roots
/// the contact is the two generators through the apex there, in one plane; where it has none (or
/// the slide runs along the axis) the face leans one way all over, and its samples say which.
fn cone_stand(face: &PartFace, parts: &Parts, c: &yack_geom::Cone, dir: Vec3) -> Result<Stand> {
    let z = c.frame.z;
    let along = dir.dot(z);
    let across = dir - z * along;
    let (sin, cos) = c.half_angle.sin_cos();
    let Some(e) = across.unit() else {
        return leans_along(face, parts, dir);
    };
    if sin.abs() <= yack_tol::ANGULAR {
        return Err(Error::Unsupported("by faces: a cone with no half angle".into()));
    }
    let k = sin * along / (cos * across.norm());
    if k.abs() >= 1.0 {
        return leans_along(face, parts, dir);
    }
    let w = z.cross(e);
    let r1 = e * k + w * (1.0 - k * k).sqrt();
    let r2 = e * k - w * (1.0 - k * k).sqrt();
    let (g1, g2) = (r1 * sin + z * cos, r2 * sin + z * cos);
    let normal = g1
        .cross(g2)
        .unit()
        .ok_or_else(|| Error::Unsupported("by faces: a cone touched along one generator".into()))?;
    let apex = c.frame.origin - z * (c.radius * cos / sin);
    Ok(Stand::Contact(Cut::Plane { origin: apex, normal }, None))
}

/// How a torus's face stands to a slide along the unit `dir`. Its outward normal is
/// `cos v·radial(u) + sin v·z`, so it is square to the slide where
/// `cos v·(radial(u)·d⊥) + sin v·(dir·z) = 0`. Slid along its axis that is `sin v = 0`, the outer
/// and inner equators: the plane through the centre square to the axis. Slid square to its axis
/// it is `cos v = 0` (the top and bottom circles, on the cylinder of the major radius) or
/// `radial(u)·d⊥ = 0` (the plane through the axis square to the slide). Slid aslant the contact
/// lies on no plane or cylinder: a decline.
fn torus_stand(t: &yack_geom::Torus, dir: Vec3) -> Result<Stand> {
    let z = t.frame.z;
    let along = dir.dot(z);
    let across = dir - z * along;
    if t.minor >= t.major {
        return Err(Error::Unsupported("by faces: a torus that meets its own axis".into()));
    }
    let origin = t.frame.origin;
    if across.norm() <= yack_tol::ANGULAR {
        return Ok(Stand::Contact(Cut::Plane { origin, normal: z }, None));
    }
    if along.abs() <= yack_tol::ANGULAR {
        let normal = across
            .unit()
            .ok_or_else(|| Error::Unsupported("by faces: a degenerate slide".into()))?;
        return Ok(Stand::Contact(
            Cut::Plane { origin, normal },
            Some(Cut::Tube { origin, axis: z, radius: t.major }),
        ));
    }
    Err(Error::Unsupported(
        "by faces: a torus slid aslant to its axis (its contact is not planar)".into(),
    ))
}

/// One piece of a face's boundary, along its curve from point `start` to point `end` (indices of
/// start-and-slid pairs in [`Prism::points`]).
#[derive(Clone, Debug)]
struct Piece {
    curve: Curve,
    start: usize,
    end: usize,
}

/// What a leading face sweeps, built up: each boundary point where it starts and where it ends,
/// and each piece of boundary once, closed edges cut in two so that no side face meets itself.
struct Prism<'a> {
    parts: &'a Parts,
    sweep: &'a Sweep<'a>,
    moved: Transform,
    out: Parts,
    /// For each point of `parts` the face uses, its index among the pairs.
    point_at: BTreeMap<usize, usize>,
    /// Each pair: its point's index in `out.points` where it starts and where it ends (the same
    /// for a point on a turn's axis), and its rail's in `out.edges` (none there).
    pairs: Vec<Pair>,
    /// For each edge of `parts` the face uses, its pieces in its curve's direction, and each
    /// piece's edge where it starts and where it ends in `out`.
    pieces: BTreeMap<usize, Vec<(Piece, usize, usize)>>,
}

/// A boundary point of a prism: where it starts and ends, and the rail between.
#[derive(Clone, Copy, Debug)]
struct Pair {
    start: usize,
    end: usize,
    rail: Option<usize>,
}

impl<'a> Prism<'a> {
    fn new(parts: &'a Parts, sweep: &'a Sweep<'a>) -> Prism<'a> {
        Prism {
            parts,
            sweep,
            moved: sweep.moved(),
            out: Parts::default(),
            point_at: BTreeMap::new(),
            pairs: Vec::new(),
            pieces: BTreeMap::new(),
        }
    }

    /// The pair for `p` (a new point when `of` is `None`), with its rail.
    fn pair(&mut self, of: Option<usize>, p: Point3) -> Result<usize> {
        if let Some(at) = of.and_then(|v| self.point_at.get(&v)) {
            return Ok(*at);
        }
        let start = self.out.points.len();
        self.out.points.push(p);
        let pair = match self.sweep.rail(p)? {
            // A whole turn brings the point back: its rail is a whole circle.
            Some(rail) if self.sweep.whole() => {
                self.out.edges.push((rail, start, start));
                Pair { start, end: start, rail: Some(self.out.edges.len() - 1) }
            }
            Some(rail) => {
                self.out.points.push(self.moved.point(p));
                self.out.edges.push((rail, start, start + 1));
                Pair { start, end: start + 1, rail: Some(self.out.edges.len() - 1) }
            }
            None => Pair { start, end: start, rail: None },
        };
        self.pairs.push(pair);
        let at = self.pairs.len() - 1;
        if let Some(v) = of {
            self.point_at.insert(v, at);
        }
        Ok(at)
    }

    /// Pair `k`.
    fn at(&self, k: usize) -> Result<Pair> {
        self.pairs.get(k).copied().ok_or_else(|| missing("pair"))
    }

    /// Edge `e`'s pieces, made on first use: the edge, or its two halves where it is closed.
    fn pieces(&mut self, e: usize) -> Result<Vec<(Piece, usize, usize)>> {
        if let Some(p) = self.pieces.get(&e) {
            return Ok(p.clone());
        }
        let (curve, a, b) = self.parts.edges.get(e).ok_or_else(|| missing("edge"))?;
        let pa = *self.parts.points.get(*a).ok_or_else(|| missing("point"))?;
        let pb = *self.parts.points.get(*b).ok_or_else(|| missing("point"))?;
        let (ka, kb) = (self.pair(Some(*a), pa)?, self.pair(Some(*b), pb)?);
        let mut list = vec![Piece { curve: curve.clone(), start: ka, end: kb }];
        let long = match curve.period() {
            Some(period) => {
                let t0 = param_of(curve, pa);
                middle_param(curve, (pa, pb)) - t0 > 0.25 * period
            }
            None => false,
        };
        if a == b || long {
            let period = curve.period().ok_or_else(|| {
                Error::Unsupported("by faces: a closed edge on a curve with no period".into())
            })?;
            // Closed: halves; open but more than half way round: cut at its middle.
            let middle = if a == b {
                param_of(curve, pa) + period / 2.0
            } else {
                middle_param(curve, (pa, pb))
            };
            let km = self.pair(None, curve.eval(middle))?;
            list = vec![
                Piece { curve: curve.clone(), start: ka, end: km },
                Piece { curve: curve.clone(), start: km, end: kb },
            ];
        }
        let mut made = Vec::new();
        for piece in list {
            let (s, t) = (self.at(piece.start)?, self.at(piece.end)?);
            let e0 = self.out.edges.len();
            self.out.edges.push((piece.curve.clone(), s.start, t.start));
            // A piece that stays where it is (on a turn's axis), or comes back to it (a whole turn),
            // is one edge both caps (both sides of its side face) share.
            if (s.rail.is_none() && t.rail.is_none()) || self.sweep.whole() {
                made.push((piece, e0, e0));
                continue;
            }
            self.out.edges.push((piece.curve.transformed(&self.moved), s.end, t.end));
            made.push((piece, e0, e0 + 1));
        }
        self.pieces.insert(e, made.clone());
        Ok(made)
    }
}

fn missing(what: &str) -> Error {
    Error::Unsupported(format!("by faces: no such {what}"))
}

/// Whether the solid `parts` describe is convex: every edge between two of its faces folds
/// outward or is smooth (a connected closed shell all of whose edges do is a convex solid's).
/// Each edge is judged at its middle, from its faces' outward normals `n₁`, `n₂` and the way the
/// first face's loop runs along it, `t`: outward where `(n₁ × n₂)·t ≥ 0`.
fn convex(parts: &Parts) -> Result<bool> {
    let mut seen: BTreeMap<usize, (usize, bool)> = BTreeMap::new();
    for (f, face) in parts.faces.iter().enumerate() {
        if face.shell != 0 {
            return Ok(false);
        }
        for &(e, reversed) in face.loops.iter().flatten() {
            let Some(&(first, first_reversed)) = seen.get(&e) else {
                seen.insert(e, (f, reversed));
                continue;
            };
            if first == f {
                continue; // a seam
            }
            let (curve, a, b) = parts.edges.get(e).ok_or_else(|| missing("edge"))?;
            let pa = *parts.points.get(*a).ok_or_else(|| missing("point"))?;
            let pb = *parts.points.get(*b).ok_or_else(|| missing("point"))?;
            let t = match curve.period() {
                Some(period) if a == b => param_of(curve, pa) + period / 2.0,
                _ => middle_param(curve, (pa, pb)),
            };
            let (mid, d) = curve.d1(t);
            let travel = if first_reversed { -d } else { d };
            let normal = |k: usize| -> Result<Vec3> {
                let face = parts.faces.get(k).ok_or_else(|| missing("face"))?;
                let uv = face.surface.project(mid);
                let n = face.surface.normal(uv.x, uv.y);
                Ok(if face.reversed { -n } else { n })
            };
            let fold = normal(first)?.cross(normal(f)?).dot(travel.unit().unwrap_or(travel));
            if fold < -CLEAR {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// The solid the faces `ks` of `parts` sweep on `sweep`, each leading all over and no two
/// sweeping into each other: each face where it starts (facing back) and where it ends, and the
/// side each piece of their boundary sweeps (a piece two of them share sweeps none); named
/// `start`, `end` and `side` under the sweep's feature.
fn prism(parts: &Parts, ks: &[usize], sweep: &Sweep) -> Result<Brep> {
    let faces_of: Vec<&PartFace> = ks
        .iter()
        .map(|&k| parts.faces.get(k).ok_or_else(|| missing("face")))
        .collect::<Result<_>>()?;
    let mut p = Prism::new(parts, sweep);
    // A seam a face uses both ways, or an edge two of the faces share, sweeps no side: its two
    // sides would cancel.
    let uses = |e: usize| {
        faces_of.iter().flat_map(|f| f.loops.iter().flatten()).filter(|&&(x, _)| x == e).count()
    };
    let named = |part: &str| BTreeSet::from([FaceName::new(sweep.feature, part)]);
    let (mut faces, mut sides) = (Vec::new(), Vec::new());
    for face in &faces_of {
        let (mut start_loops, mut end_loops) = (Vec::new(), Vec::new());
        for lp in &face.loops {
            let (mut start_loop, mut end_loop) = (Vec::new(), Vec::new());
            for &(e, reversed) in lp {
                let mut pieces = p.pieces(e)?;
                if reversed {
                    pieces.reverse();
                }
                for (piece, e0, e1) in pieces {
                    start_loop.push((e0, !reversed));
                    end_loop.push((e1, reversed));
                    // A seam sweeps no side, nor a piece that stays where it is.
                    let stays =
                        p.at(piece.start)?.rail.is_none() && p.at(piece.end)?.rail.is_none();
                    if uses(e) == 1 && !stays {
                        sides.push((piece, reversed, e0, e1));
                    }
                }
            }
            // The start cap runs the other way round (it faces back), so its uses go in reverse.
            start_loop.reverse();
            start_loops.push(start_loop);
            end_loops.push(end_loop);
        }
        // A whole turn ends where it starts: no caps, its sides close on themselves (each meets
        // itself along its piece, its seam).
        if !sweep.whole() {
            faces.push(PartFace {
                surface: face.surface.clone(),
                reversed: !face.reversed,
                loops: start_loops,
                names: named(FaceName::START),
                shell: 0,
            });
            faces.push(PartFace {
                surface: face.surface.transformed(&p.moved),
                reversed: face.reversed,
                loops: end_loops,
                names: named(FaceName::END),
                shell: 0,
            });
        }
    }
    for (piece, reversed, e0, e1) in sides {
        // Along the loop: the piece (as the face uses it), the rail up from its end, the slid
        // piece back, the rail down to its start; the face's material lies on the loop's left.
        let (first, last) =
            if reversed { (piece.end, piece.start) } else { (piece.start, piece.end) };
        let (ra, rb) = (p.at(first)?.rail, p.at(last)?.rail);
        let point = |k: usize| -> Result<Point3> {
            let at = p.at(k)?.start;
            p.out.points.get(at).copied().ok_or_else(|| missing("point"))
        };
        let ends = (point(piece.start)?, point(piece.end)?);
        // A point that stays (on a turn's axis) has no rail: the side closes there.
        let mut loop_ = vec![(e0, reversed)];
        loop_.extend(rb.map(|r| (r, false)));
        loop_.push((e1, !reversed));
        loop_.extend(ra.map(|r| (r, true)));
        let (surface, flip) = sweep.side(&piece.curve, ends, reversed)?;
        // A whole turn of a piece square to the axis sweeps an annulus (a disc where the piece
        // reaches the axis): bounded by its ends' rails, not by the piece, which would be a slit.
        let loops = match (&surface, sweep.whole()) {
            (Surface::Plane(_), true) => annulus(&p, sweep, (piece.start, piece.end), flip)?,
            _ => vec![loop_],
        };
        faces.push(PartFace {
            surface,
            reversed: flip,
            loops,
            names: named(FaceName::SIDE),
            shell: 0,
        });
    }
    p.out.faces = faces;
    solid_of(&p.out)
}

/// The loops of the annulus (or disc) a piece square to a whole turn's axis sweeps, its outer
/// loop first: its ends' rails, each a whole circle counter-clockwise about the axis, run so that
/// the face (its natural normal the axis, against it when `flip`) lies on their left.
fn annulus(
    p: &Prism,
    sweep: &Sweep,
    ends: (usize, usize),
    flip: bool,
) -> Result<Vec<Vec<(usize, bool)>>> {
    let How::Turn { origin, axis, .. } = sweep.how else {
        return Err(missing("turn"));
    };
    let reach = |k: usize| -> Result<(f64, Option<usize>)> {
        let pair = p.at(k)?;
        let q = *p.out.points.get(pair.start).ok_or_else(|| missing("point"))?;
        Ok(((q - origin).cross(axis).norm(), pair.rail))
    };
    let (mut outer, mut inner) = (reach(ends.0)?, reach(ends.1)?);
    if inner.0 > outer.0 {
        std::mem::swap(&mut outer, &mut inner);
    }
    // Seen down the axis the outer rail runs as it is and the inner against it; seen from the
    // other side, the other way round.
    let outer = outer.1.ok_or_else(|| missing("rail"))?;
    let mut loops = vec![vec![(outer, flip)]];
    loops.extend(inner.1.map(|r| vec![(r, !flip)]));
    Ok(loops)
}

/// The surface a piece of boundary sweeps as it slides along the unit `dir`, and whether the
/// side faces against it: a plane for a straight piece or a circle in a plane along the slide, a
/// cylinder for a circle square to the slide, an extrusion otherwise. `ends` are where the piece
/// starts and ends along its curve, `reversed` whether the leading face's loop runs against the
/// curve. Its outward normal is taken at the piece's middle, where the piece does not run along
/// the slide (its ends may: a contact's ends on a circle do). The curve is [`seam_safe`] first.
fn side_surface(
    curve: &Curve,
    ends: (Point3, Point3),
    reversed: bool,
    dir: Vec3,
) -> Result<(Surface, bool)> {
    let curve = &seam_safe(curve, ends)?;
    let (mid, d) = curve.d1(middle_param(curve, ends));
    let travel = if reversed { -d } else { d };
    // The side's outward normal is square to the edge and the slide, away from the face's
    // material: `travel × dir`, since the face (its normal `n`, its material on the left of
    // `travel`, inward `n × travel`) leads: (travel × dir)·(n × travel) = −|travel|²(dir·n) ≤ 0.
    // Testing against the face's inward direction instead would guess on a contact edge, where
    // the face's normal is square to the slide and the product is zero.
    let Some(out) = travel.cross(dir).unit() else {
        return Err(Error::Unsupported("by faces: an edge that runs along the slide".into()));
    };
    let surface = match curve {
        Curve::Line(l) => Surface::Plane(Plane { frame: Frame::new(l.origin, out, l.dir)? }),
        Curve::Circle(c) if c.frame.z.cross(dir).norm() <= yack_tol::ANGULAR => {
            Surface::Cylinder(Cylinder { frame: c.frame, radius: c.radius })
        }
        // A circle slid along its own plane sweeps a piece of that plane.
        Curve::Circle(c) if c.frame.z.dot(dir).abs() <= yack_tol::ANGULAR => {
            Surface::Plane(Plane { frame: Frame::new(c.frame.origin, out, c.frame.x)? })
        }
        _ => Surface::Extrusion(Box::new(Extrusion { curve: curve.clone(), dir })),
    };
    let uv = surface.project(mid);
    let natural = surface.normal(uv.x, uv.y);
    Ok((surface, natural.dot(out) < 0.0))
}

/// `curve` reparametrised, where it is periodic, so that its seam lies off the piece from
/// `ends.0` to `ends.1`: a surface it sweeps then has no seam across the side face. A circle's
/// seam goes opposite the piece's middle; an ellipse's to whichever end of its major axis the
/// piece misses. Other periodic curves decline.
fn seam_safe(curve: &Curve, ends: (Point3, Point3)) -> Result<Curve> {
    Ok(match curve {
        Curve::Circle(c) => {
            let middle = curve.eval(middle_param(curve, ends));
            let frame = Frame::new(c.frame.origin, c.frame.z, c.frame.origin - middle)?;
            Curve::Circle(Circle { frame, radius: c.radius })
        }
        // An ellipse's parameter starts on its major axis; turned half round (its frame's x and y
        // reversed) it is the same ellipse, its parameter starting on the other end. A piece goes
        // at most half way round, so one of the two keeps the seam off it.
        Curve::Ellipse(el) => {
            let t0 = param_of(curve, ends.0);
            let t1 = 2.0 * middle_param(curve, ends) - t0;
            let seam_inside = t1 > std::f64::consts::TAU;
            if seam_inside {
                let frame = Frame::new(el.frame.origin, el.frame.z, -el.frame.x)?;
                Curve::Ellipse(Ellipse { frame, ..el.clone() })
            } else {
                curve.clone()
            }
        }
        other if other.period().is_some() => {
            return Err(Error::Unsupported(
                "by faces: the side of a closed curve other than a circle or an ellipse".into(),
            ));
        }
        other => other.clone(),
    })
}

/// The parameter halfway along `curve` from `ends.0` to `ends.1`; on a closed curve the way
/// forward, all the way round where the two meet.
fn middle_param(curve: &Curve, ends: (Point3, Point3)) -> f64 {
    let (t0, t1) = (param_of(curve, ends.0), param_of(curve, ends.1));
    let t1 = match curve.period() {
        Some(period) if t1 <= t0 => t1 + period,
        _ => t1,
    };
    0.5 * (t0 + t1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use yack_math::kernel::Frame;

    /// Every prism a slide makes passes the checker on its own, before any union.
    fn prisms_check(body: &Brep, by: Vec3) {
        let sweep = Sweep { how: How::Slide { by, dir: by.unit().unwrap() }, feature: "f" };
        let mut prisms = Vec::new();
        prisms_of(body, &sweep, &[], &mut prisms).unwrap();
        assert!(!prisms.is_empty());
        for (i, p) in prisms.iter().enumerate() {
            let issues = yack_check::check(p);
            let kinds: Vec<_> = p.face_ids().map(|f| p.face(f).surface.kind()).collect();
            assert!(issues.is_empty(), "prism {i} {kinds:?}: {issues:?}");
        }
    }

    #[test]
    fn a_turned_boxs_prisms_are_valid() {
        let b =
            yack_build::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
        let how = How::Turn { origin: Point3::ZERO, axis: Vec3::Z, angle: 1.0 };
        let mut prisms = Vec::new();
        prisms_of(&b, &Sweep { how, feature: "f" }, &[], &mut prisms).unwrap();
        assert!(!prisms.is_empty());
        for p in &prisms {
            assert!(yack_check::check(p).is_empty(), "{:?}", yack_check::check(p));
        }
    }

    #[test]
    fn a_cylinders_prisms_are_valid() {
        let c = yack_build::primitive::cylinder(&Frame::WORLD, 2.0, 10.0, "c").unwrap();
        prisms_check(&c, Vec3::new(3.0, 0.0, 4.0));
    }

    #[test]
    fn a_balls_prisms_are_valid() {
        let ball = yack_build::primitive::sphere(&Frame::WORLD, 2.0, "ball").unwrap();
        prisms_check(&ball, Vec3::new(0.0, 0.0, 5.0));
    }
}
