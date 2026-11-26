// yack: A planar profile swept along a helix, its frame turning with the helix: the profile screwed
// Adapted from springrep@a03e8a1 src/sweep/helix.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/sweep/helix.rs to yack-build (M3 W7); paths and visibility only;
// M3 hardening (w-h3): the sweep's cuts are a `Cut` enum (start, a screw's whole turn, end) instead
// of indices with an unreachable arm; each profile corner is a record of its vertex at every cut and
// its helix pieces, walked with the next by iterators; a rail that makes no curve is an error.
//! A planar profile swept along a helix, its frame turning with the helix: the profile screwed
//! about the helix's axis. Each side face is a rational B-spline surface, `u` the turn and `v` the
//! profile segment's own rational parameter, so its edges are the surface's parameter lines.
//!
//! A screw motion has no rational form, so each column of the surface's control net follows its
//! control point's helix as degree-7 Hermite pieces through the helix's points and first three
//! derivatives at evenly spaced turns. The pieces are spaced so the column stays within
//! `SCREW_FIT` of the helix, and a rational surface is a convex mix of its columns, so the surface
//! stays as close to the exact screw surface.
#![warn(clippy::indexing_slicing)]

use std::f64::consts::TAU;

use crate::{
    Error, Result,
    intersect::csi,
    check,
    error::invalid,
    geom::{Curve, Cylinder, NurbsCurve, NurbsSurface, Plane, Surface},
    loft::rail::{Hom, Rail, hom, point},
    math::{Frame, Point3, Transform, Vec2, Vec3},
    profile::{Names, Profile},
    tol,
    topo::{Brep, CoedgeSpec, EdgeId, FaceName, PCurve, VertexId},
};

/// How far a column may stray from its helix. A helix crosses a plane square to its axis, or a
/// cone about it, at its lead angle or less, a hundredth of a radian on a fine thread, and its
/// crossing slides along it by its error over that angle: a ten-thousandth of the linear
/// tolerance keeps the crossing within a hundredth of the tolerance of the true one.
const SCREW_FIT: f64 = tol::LINEAR * 1e-4;
const DEGREE: usize = 7;

/// A screw motion: a turn of `sign·θ` about the axis through `origin` along `axis`, and a rise of
/// `rise·θ` along it.
#[derive(Clone, Copy, Debug)]
pub struct Helix {
    pub origin: Point3,
    pub axis: Vec3,
    /// Rise per radian of turn, mm.
    pub rise: f64,
    /// 1 for a right-hand screw about `axis`, −1 for a left-hand one.
    pub sign: f64,
}

impl Helix {
    /// Where `p` goes at turn `t`, and its first three derivatives along `t`.
    fn d3(&self, p: Point3, t: f64) -> [Vec3; 4] {
        let q = p - self.origin;
        let along = self.axis * q.dot(self.axis);
        let x = q - along;
        let y = self.axis.cross(x);
        let (s, c) = (self.sign * t).sin_cos();
        let at = self.origin + along + x * c + y * s + self.axis * (self.rise * t);
        let d1 = (y * c - x * s) * self.sign + self.axis * self.rise;
        let d2 = -(x * c + y * s);
        let d3 = (x * s - y * c) * self.sign;
        [at, d1, d2, d3]
    }

    fn at(&self, p: Point3, t: f64) -> Point3 {
        self.d3(p, t)[0]
    }

    /// The rigid move of turn `t`.
    pub fn transform(&self, t: f64) -> Transform {
        Transform::rotation(self.origin, self.axis, self.sign * t)
            .then(&Transform::translation(self.axis * (self.rise * t)))
    }
}

/// Knots of a curve in Hermite pieces between `nodes`, at least two of them (none otherwise).
fn knots(nodes: &[f64]) -> Vec<f64> {
    let &[first, ref inner @ .., last] = nodes else { return Vec::new() };
    let mut k = vec![first; DEGREE + 1];
    for &n in inner {
        k.extend(std::iter::repeat_n(n, DEGREE));
    }
    k.extend(std::iter::repeat_n(last, DEGREE + 1));
    k
}

/// The Bézier points of a degree-7 piece of length `h` from its end values and derivatives:
/// `f⁽ᵏ⁾(0) = 7!/(7−k)!/hᵏ · Δᵏ P₀`, and the same from the far end.
fn hermite(a: [Vec3; 4], b: [Vec3; 4], h: f64) -> [Point3; 7] {
    let (k1, k2, k3) = (h / 7.0, h * h / 42.0, h * h * h / 210.0);
    let p1 = a[0] + a[1] * k1;
    let p2 = p1 * 2.0 - a[0] + a[2] * k2;
    let p3 = (p2 - p1) * 3.0 + a[0] + a[3] * k3;
    let q1 = b[0] - b[1] * k1;
    let q2 = q1 * 2.0 - b[0] + b[2] * k2;
    let q3 = (q2 - q1) * 3.0 + b[0] - b[3] * k3;
    [p1, p2, p3, q3, q2, q1, b[0]]
}

/// The control points of `p`'s helix in Hermite pieces between `nodes`.
fn column(screw: &Helix, p: Point3, nodes: &[f64]) -> Vec<Point3> {
    let mut out: Vec<Point3> = nodes.first().map(|&t| screw.at(p, t)).into_iter().collect();
    for (&t0, &t1) in nodes.iter().zip(nodes.iter().skip(1)) {
        let (a, b) = (screw.d3(p, t0), screw.d3(p, t1));
        out.extend(hermite(a, b, t1 - t0));
    }
    out
}

/// Evenly spaced turns from 0 to `angle`, close enough that a Hermite piece follows a helix of
/// radius up to `radius` within `SCREW_FIT`: its error is at most `radius · (h/2)⁸ / 8!`.
fn nodes(angle: f64, radius: f64) -> Vec<f64> {
    let h = 2.0 * (40320.0 * SCREW_FIT / radius.max(tol::LINEAR)).powf(1.0 / 8.0);
    let n = (angle / h).ceil().max(1.0) as usize;
    (0..=n).map(|k| angle * k as f64 / n as f64).collect()
}

fn coedge(edge: EdgeId, reversed: bool, pcurve: PCurve) -> CoedgeSpec {
    CoedgeSpec {
        edge,
        reversed,
        pcurve,
    }
}

fn affine(origin: Vec2, dir: Vec2) -> PCurve {
    PCurve::Affine { origin, dir }
}

fn plane_seed(frame: &Frame, p: Point3) -> PCurve {
    let l = frame.to_local(p);
    PCurve::Projected {
        seed: Vec2::new(l.x, l.y),
    }
}

fn split(pw: &[Hom]) -> (Vec<Point3>, Option<Vec<f64>>) {
    let w: Vec<f64> = pw.iter().map(|h| h[3]).collect();
    let rational = w.iter().any(|&x| (x - 1.0).abs() > 1e-15);
    (
        pw.iter().map(|&h| point(h)).collect(),
        rational.then_some(w),
    )
}

/// One profile segment: its rail on `[0, 1]` in the loop's direction and its name.
pub struct Seg {
    pub rail: Rail,
    pub name: Names,
}

/// Where the swept profile would meet itself: the profile must keep to one side of the axis, and
/// where the sweep turns a whole turn or more, the profile must stay clear of its copy a pitch
/// along the axis, which the sweep lays in the same half-plane a turn later.
fn self_contact(profile: &Profile, frame: &Frame, screw: &Helix, angle: f64) -> Result<()> {
    let side = screw.axis.cross(frame.z);
    let curves: Vec<(Curve, (f64, f64))> = profile
        .loops
        .iter()
        .flat_map(|lp| &lp.segments)
        .map(|s| {
            let placed = s.segment.place(frame);
            (placed.curve, placed.range)
        })
        .collect();
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut z_lo, mut z_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for (c, (t0, t1)) in &curves {
        let mut ts = vec![*t0, *t1];
        ts.extend(c.extremes(side, *t0, *t1));
        ts.extend(c.extremes(screw.axis, *t0, *t1));
        for t in ts {
            let q = c.eval(t) - screw.origin;
            let (r, z) = (q.dot(side), q.dot(screw.axis));
            (lo, hi) = (lo.min(r), hi.max(r));
            (z_lo, z_hi) = (z_lo.min(z), z_hi.max(z));
        }
    }
    let near = lo.abs().min(hi.abs());
    if lo <= tol::LINEAR && hi >= -tol::LINEAR {
        return invalid("a screw sweep's profile reaches its axis");
    }
    // Half-planes a hair short of a whole turn apart hold the end cap within the tolerance of the
    // start cap where the profile is nearest the axis.
    let turns = ((angle + tol::LINEAR / near) / TAU).floor();
    let rise = screw.rise * TAU;
    let mut k = 1.0;
    while k <= turns && (k * rise).abs() <= z_hi - z_lo + tol::LINEAR {
        let shift = Transform::translation(screw.axis * (k * rise));
        let meets = curves.iter().any(|(c, r)| {
            let moved = c.transformed(&shift);
            curves
                .iter()
                .any(|(d, rd)| !csi::curve_curve(&moved, *r, d, *rd).is_empty())
        });
        if meets || (k * rise).abs() <= tol::LINEAR {
            return invalid("a screw sweep's coils meet: its pitch is too short for its profile");
        }
        k += 1.0;
    }
    Ok(())
}

/// Screws `profile`, drawn on `frame`'s xy plane, through `angle` radians of `screw`. The
/// profile's plane must hold the axis, and the profile must keep to one side of it and clear of
/// itself a turn later; or its plane is square to the axis, a twisted extrusion. Caps are `start` and `end`; each side face is named after its profile
/// segment.
pub fn helix(
    profile: &Profile,
    frame: &Frame,
    screw: &Helix,
    angle: f64,
    feature: &str,
) -> Result<Brep> {
    if !(angle.is_finite() && angle > tol::ANGULAR) {
        return invalid("a screw sweep turns through a positive angle");
    }
    let Some(axis) = screw.axis.unit() else {
        return invalid("a screw sweep needs an axis");
    };
    let screw = Helix { axis, ..*screw };
    // A profile square to the axis rises as it turns, a twisted extrusion: each level holds one
    // turned copy of it, so it can never meet itself, and its outline may go round the axis.
    let square = frame.z.cross(axis).norm() <= tol::ANGULAR;
    if !square
        && (frame.z.dot(axis).abs() > tol::ANGULAR
            || (frame.origin - screw.origin).dot(frame.z).abs() > tol::LINEAR)
    {
        return Err(Error::Unsupported(
            "a screw sweep's profile off a plane through its axis or square to it".into(),
        ));
    }
    if square && screw.rise.abs() <= tol::LINEAR {
        return invalid("a twisted extrusion must rise");
    }
    let mut profile = profile.normalized()?;
    if !square {
        self_contact(&profile, frame, &screw, angle)?;
    }
    let (mut radius, mut far) = (0.0f64, frame.origin);
    for lp in &profile.loops {
        for p in lp.polygon() {
            let q = frame.at(p.x, p.y) - screw.origin;
            let r = (q - axis * q.dot(axis)).norm();
            if r > radius {
                (radius, far) = (r, frame.at(p.x, p.y));
            }
        }
    }
    // The motion of a point of the profile: on the axis, where the frame's origin may lie, a screw
    // only rises, along the very plane it must cross.
    let motion = screw.d3(far, 0.0)[1];
    if motion.dot(frame.z).abs() <= tol::ANGULAR * motion.norm() {
        return invalid("a screw sweep's profile plane runs along its motion");
    }
    // Loops run counter-clockwise about the motion, and the start cap's frame faces along it.
    let sketch = *frame;
    let mut frame = *frame;
    if motion.dot(frame.z) < 0.0 {
        for lp in &mut profile.loops {
            lp.reverse();
        }
        frame = Frame::new(frame.origin, -frame.z, frame.x)?;
    }
    let loops: Vec<Vec<Seg>> = profile
        .loops
        .iter()
        .map(|lp| {
            lp.segments
                .iter()
                .map(|s| {
                    let placed = s.segment.place(&sketch);
                    let (t0, t1) = if placed.forward {
                        placed.range
                    } else {
                        (placed.range.1, placed.range.0)
                    };
                    Ok(Seg {
                        rail: Rail::of_curve(&placed.curve, t0, t1)?,
                        name: s.names.clone(),
                    })
                })
                .collect::<Result<Vec<_>>>()
        })
        .collect::<Result<_>>()?;
    // A rational rail's inner control points lie off the profile, up to √2 its radius away.
    let far = loops
        .iter()
        .flatten()
        .flat_map(|s| s.rail.pw.iter())
        .map(|&h| {
            let q = point(h) - screw.origin;
            (q - axis * q.dot(axis)).norm()
        })
        .fold(radius, f64::max);
    let nodes = nodes(angle, far);
    let ku = knots(&nodes);
    let end_frame = frame.transformed(&screw.transform(angle));
    let motion = Motion {
        degree: DEGREE,
        knots: ku,
        range: (0.0, angle),
        column: Box::new(|p| column(&screw, p, &nodes)),
        at_end: Box::new(|p| screw.at(p, angle)),
        velocity: Box::new(|p| screw.d3(p, 0.0)[1]),
        end_frame,
        screw: Some(screw),
    };
    assemble(&loops, frame, &motion, feature)
}

/// How a profile moves along a sweep: each point's track as a curve of `degree` on `knots` over
/// `range` (its control points), where it ends, and its velocity at the start; and the frame the
/// end cap lies in.
pub struct Motion<'a> {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub range: (f64, f64),
    pub column: Box<dyn Fn(Point3) -> Vec<Point3> + 'a>,
    pub at_end: Box<dyn Fn(Point3) -> Point3 + 'a>,
    pub velocity: Box<dyn Fn(Point3) -> Vec3 + 'a>,
    pub end_frame: Frame,
    /// The screw, where the motion is one: then a segment along its axis sweeps a cylinder.
    pub screw: Option<Helix>,
}

/// The solid `loops` sweep moving by `m` from the start cap in `frame`: its caps, and a side face
/// for each segment whose `u` runs along the motion and `v` along the segment.
///
/// A screw's segment along its axis, a thread's crest, sweeps a strip of a cylinder, which it is
/// made exactly, so that a boolean meeting a shank of that radius finds the two faces on one
/// surface. A cylinder's angle runs once round, so the strip is cut at each whole turn, and with
/// it the helices along every corner of the profile, each other face bounded by their pieces.
pub fn assemble(
    loops: &[Vec<Seg>],
    frame: Frame,
    m: &Motion,
    feature: &str,
) -> Result<Brep> {
    let helix = |p: Point3| -> Result<Curve> {
        let c = NurbsCurve::new(m.degree, m.knots.clone(), (m.column)(p), None)?;
        Ok(Curve::Nurbs(Box::new(c)))
    };
    // The turns the sweep is cut at: its ends, and each whole turn of a screw between them.
    let mut cuts = vec![Cut::Start(m.range.0)];
    if let Some(sc) = m.screw {
        let mut t = m.range.0 + TAU;
        while t < m.range.1 - tol::ANGULAR {
            cuts.push(Cut::Turn(sc, t));
            t += TAU;
        }
    }
    cuts.push(Cut::End(m.range.1));
    let spans: Vec<(Cut, Cut)> = cuts.iter().copied().zip(cuts.iter().copied().skip(1)).collect();
    // Where profile point `p` is at a cut.
    let place = |p: Point3, cut: Cut| -> Point3 {
        match cut {
            Cut::Start(_) => p,
            Cut::Turn(sc, t) => sc.at(p, t),
            Cut::End(_) => (m.at_end)(p),
        }
    };
    let line = |a: Point3, b: Point3| -> Result<Curve> {
        Ok(Curve::Nurbs(Box::new(NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![a, b],
            None,
        )?)))
    };
    let end_frame = m.end_frame;
    let mut b = Brep::new();
    let start = b.add_face(
        Surface::Plane(Plane { frame }),
        true,
        [FaceName::new(feature, FaceName::START)],
    );
    let end = b.add_face(
        Surface::Plane(Plane { frame: end_frame }),
        false,
        [FaceName::new(feature, FaceName::END)],
    );
    let mut faces = vec![start, end];
    for segs in loops {
        let starts: Vec<Point3> = segs.iter().map(|s| s.rail.start()).collect();
        // Each corner's vertex at every cut, and its helix in a piece between each two.
        let verts: Vec<Vec<VertexId>> = starts
            .iter()
            .map(|&p| cuts.iter().map(|&cut| b.add_vertex(place(p, cut))).collect())
            .collect();
        let rings: Vec<Vec<EdgeId>> = starts
            .iter()
            .zip(&verts)
            .map(|(&p, vs)| {
                let curve = helix(p)?;
                Ok(spans
                    .iter()
                    .zip(vs.iter().zip(vs.iter().skip(1)))
                    .map(|((from, to), (&v0, &v1))| {
                        b.add_edge(curve.clone(), (from.t(), to.t()), v0, v1)
                    })
                    .collect())
            })
            .collect::<Result<_>>()?;
        let corners: Vec<Corner> = starts
            .into_iter()
            .zip(verts)
            .zip(rings)
            .map(|((point, verts), rings)| Corner { point, verts, rings })
            .collect();
        let mut e0 = Vec::with_capacity(segs.len());
        let mut e1 = Vec::with_capacity(segs.len());
        for (s, (here, next)) in segs.iter().zip(round(&corners)) {
            let r = &s.rail;
            let (pts, w) = split(&r.pw);
            let at_end: Vec<Hom> =
                r.pw.iter().map(|&h| hom((m.at_end)(point(h)), h[3])).collect();
            let (end_pts, _) = split(&at_end);
            let c0 = NurbsCurve::new(r.degree, r.knots.clone(), pts.clone(), w.clone())?;
            let c1 = NurbsCurve::new(r.degree, r.knots.clone(), end_pts, w.clone())?;
            let first_edge =
                b.add_edge(Curve::Nurbs(Box::new(c0)), (0.0, 1.0), here.first()?, next.first()?);
            e0.push(first_edge);
            let last_edge =
                b.add_edge(Curve::Nurbs(Box::new(c1)), (0.0, 1.0), here.last()?, next.last()?);
            e1.push(last_edge);
            let name = FaceName::all(feature, &s.name);
            let widest = spans.iter().map(|(from, to)| to.t() - from.t()).fold(0.0, f64::max);
            let strip = m.screw.as_ref().and_then(|sc| {
                along_axis(sc, &pts, &w)
                    .map(|(h0, dh, radius)| (sc, h0, dh, radius, 0.0))
                    // An arc's strip must fit one turn of its cylinder's angle.
                    .or_else(|| {
                        about_axis(sc, r)
                            .filter(|&(_, _, sweep)| sweep.abs() + widest <= TAU - 1e-9)
                            .map(|(h0, radius, sweep)| (sc, h0, 0.0, radius, sweep * sc.sign))
                    })
            });
            if let Some((sc, h0, dh, radius, round)) = strip {
                // The cylinder of the strip, angle 0 where the turn `c` starts and z along the
                // turning, so the angle runs with the screw's turn: at turn t and segment
                // parameter s the angle is `round·s + t − t0` (for an arc, at the angle it has
                // turned by s) and the height along z `sign·(h0 + rise·t + s·dh)`.
                let z = sc.axis * sc.sign;
                let (sign, rise) = (sc.sign, sc.rise);
                let seps: Vec<EdgeId> = cuts
                    .iter()
                    .zip(here.verts.iter().zip(&next.verts))
                    .map(|(&cut, (&v_here, &v_next))| match cut {
                        Cut::Start(_) => Ok(first_edge),
                        Cut::End(_) => Ok(last_edge),
                        Cut::Turn(..) if round == 0.0 => Ok(b.add_edge(
                            line(place(here.point, cut), place(next.point, cut))?,
                            (0.0, 1.0),
                            v_here,
                            v_next,
                        )),
                        Cut::Turn(_, t) => {
                            let turned: Vec<Hom> =
                                r.pw.iter().map(|&h| hom(sc.at(point(h), t), h[3])).collect();
                            let (ctrl, wts) = split(&turned);
                            let arc = NurbsCurve::new(r.degree, r.knots.clone(), ctrl, wts)?;
                            Ok(b.add_edge(Curve::Nurbs(Box::new(arc)), (0.0, 1.0), v_here, v_next))
                        }
                    })
                    .collect::<Result<_>>()?;
                let pieces = spans
                    .iter()
                    .zip(seps.iter().zip(seps.iter().skip(1)))
                    .zip(next.rings.iter().zip(&here.rings));
                for (((from, to), (&sep0, &sep1)), (&ring_next, &ring_here)) in pieces {
                    let p = place(here.point, *from) - sc.origin;
                    let x = (p - sc.axis * p.dot(sc.axis)).unit().ok_or_else(|| {
                        Error::InvalidInput("a screw's crest lies on its axis".into())
                    })?;
                    let surface = Surface::Cylinder(Cylinder {
                        frame: Frame::new(sc.origin, z, x)?,
                        radius,
                    });
                    let (t0, t1) = (from.t(), to.t());
                    let v = |t: f64, s: f64| sign * (h0 + rise * t + s * dh);
                    // A line's end is affine in its parameter; an arc's angle is not in its
                    // rational one, so its end is placed by projection.
                    let across = |t: f64| {
                        if round == 0.0 {
                            affine(Vec2::new(t - t0, v(t, 0.0)), Vec2::new(0.0, sign * dh))
                        } else {
                            PCurve::Projected { seed: Vec2::new(t - t0, v(t, 0.0)) }
                        }
                    };
                    // Outward where the motion and the segment's direction say, as below.
                    let (_, su, sv) = surface.d1(0.5 * round, v(t0, 0.5));
                    let (pm, d) = r_mid(r)?;
                    let outward = d.cross((m.velocity)(pm));
                    let face = b.add_face(surface, su.cross(sv).dot(outward) < 0.0, name.clone());
                    let turning = Vec2::new(1.0, sign * rise);
                    b.add_loop(
                        face,
                        vec![
                            coedge(sep0, false, across(t0)),
                            coedge(
                                ring_next,
                                false,
                                affine(Vec2::new(round - t0, v(0.0, 1.0)), turning),
                            ),
                            coedge(sep1, true, across(t1)),
                            coedge(ring_here, true, affine(Vec2::new(-t0, v(0.0, 0.0)), turning)),
                        ],
                    );
                    faces.push(face);
                }
                continue;
            }
            let nv = pts.len();
            let mut net = Vec::with_capacity(nv * (m.knots.len() - m.degree - 1));
            let cols: Vec<Vec<Point3>> = pts.iter().map(|&p| (m.column)(p)).collect();
            // Every column has the same length: the net row by row along the motion.
            let nu = cols.first().map_or(0, Vec::len);
            for row in 0..nu {
                net.extend(cols.iter().filter_map(|c| c.get(row).copied()));
            }
            let weights = w.map(|w| (0..nu).flat_map(|_| w.iter().copied()).collect());
            let surface = Surface::Nurbs(Box::new(NurbsSurface::new(
                (m.degree, r.degree),
                (m.knots.clone(), r.knots.clone()),
                (nu, nv),
                net,
                weights,
            )?));
            // At the start turn, where the surface's column points are the profile's own.
            let (_, su, sv) = surface.d1(m.range.0, 0.5);
            let (p, d) = r_mid(r)?;
            let outward = d.cross((m.velocity)(p));
            let face = b.add_face(surface, su.cross(sv).dot(outward) < 0.0, name);
            let mut lp = vec![coedge(
                first_edge,
                false,
                affine(Vec2::new(m.range.0, 0.0), Vec2::new(0.0, 1.0)),
            )];
            lp.extend(
                next.rings
                    .iter()
                    .map(|&e| coedge(e, false, affine(Vec2::new(0.0, 1.0), Vec2::new(1.0, 0.0)))),
            );
            lp.push(coedge(
                last_edge,
                true,
                affine(Vec2::new(m.range.1, 0.0), Vec2::new(0.0, 1.0)),
            ));
            lp.extend(
                here.rings
                    .iter()
                    .rev()
                    .map(|&e| coedge(e, true, affine(Vec2::ZERO, Vec2::new(1.0, 0.0)))),
            );
            b.add_loop(face, lp);
            faces.push(face);
        }
        // Each segment's first edge with the corner it ends at, run backwards round the cap.
        let ends: Vec<(EdgeId, &Corner)> = e0.iter().copied().zip(following(&corners)).collect();
        let lo: Vec<CoedgeSpec> = ends
            .into_iter()
            .rev()
            .map(|(e, next)| coedge(e, true, plane_seed(&frame, next.point)))
            .collect();
        b.add_loop(start, lo);
        let hi: Vec<CoedgeSpec> = e1
            .iter()
            .zip(&corners)
            .map(|(&e, corner)| coedge(e, false, plane_seed(&end_frame, (m.at_end)(corner.point))))
            .collect();
        b.add_loop(end, hi);
    }
    let shell = b.add_shell(faces);
    b.add_solid(vec![shell]);
    let issues = check::check(&b);
    if issues.is_empty() {
        Ok(b)
    } else {
        Err(Error::InvalidResult(
            issues.into_iter().map(|i| i.to_string()).collect(),
        ))
    }
}

/// Where a rail is a straight run along the screw's axis at a radius off it: its height along
/// the axis at its start, how far it rises, and the radius.
fn along_axis(sc: &Helix, pts: &[Point3], w: &Option<Vec<f64>>) -> Option<(f64, f64, f64)> {
    let &[start, end] = pts else { return None };
    let uneven =
        |w: &Vec<f64>| w.first().is_some_and(|&w0| w.iter().any(|&x| (x - w0).abs() > 1e-12));
    if w.as_ref().is_some_and(uneven) {
        return None;
    }
    let (a, b) = (start - sc.origin, end - sc.origin);
    let d = b - a;
    if d.cross(sc.axis).norm() > tol::LINEAR * 1e-3 {
        return None;
    }
    let radius = (a - sc.axis * a.dot(sc.axis)).norm();
    (radius > tol::LINEAR).then(|| (a.dot(sc.axis), d.dot(sc.axis), radius))
}

/// Where a rail is an arc about the screw's axis in a plane square to it: its height along the
/// axis, its radius, and the angle it turns through about the axis, signed.
fn about_axis(sc: &Helix, r: &Rail) -> Option<(f64, f64, f64)> {
    const N: usize = 16;
    let (ctrl, w) = split(&r.pw);
    let c = NurbsCurve::new(r.degree, r.knots.clone(), ctrl, w).ok()?;
    let at = |k: usize| {
        let q = c.eval(k as f64 / N as f64) - sc.origin;
        let h = q.dot(sc.axis);
        (q - sc.axis * h, h)
    };
    let (first, h0) = at(0);
    let radius = first.norm();
    if radius <= tol::LINEAR {
        return None;
    }
    let (mut last, mut sweep) = (first, 0.0);
    for k in 1..=N {
        let (q, h) = at(k);
        let fit = tol::LINEAR * 1e-3;
        if (h - h0).abs() > fit || (q.norm() - radius).abs() > fit {
            return None;
        }
        sweep += last.cross(q).dot(sc.axis).atan2(last.dot(q));
        last = q;
    }
    (sweep.abs() > tol::ANGULAR).then_some((h0, radius, sweep))
}

/// The rail's middle point and its direction there.
fn r_mid(r: &Rail) -> Result<(Point3, Vec3)> {
    let (ctrl, weights) = split(&r.pw);
    let c = NurbsCurve::new(r.degree, r.knots.clone(), ctrl, weights)?;
    let (p, d, _) = c.d2(0.5);
    Ok((p, d))
}

/// Where a sweep is cut across, at its parameter: its start, each whole turn of its screw between
/// its ends, and its end.
#[derive(Clone, Copy)]
enum Cut {
    Start(f64),
    Turn(Helix, f64),
    End(f64),
}

impl Cut {
    fn t(self) -> f64 {
        match self {
            Cut::Start(t) | Cut::Turn(_, t) | Cut::End(t) => t,
        }
    }
}

/// A profile corner of a sweep: where it starts, its vertex at every cut, and its helix's piece
/// between each two.
struct Corner {
    point: Point3,
    verts: Vec<VertexId>,
    rings: Vec<EdgeId>,
}

impl Corner {
    /// Its vertex at the sweep's start.
    fn first(&self) -> Result<VertexId> {
        self.verts.first().copied().ok_or_else(no_cuts)
    }

    /// Its vertex at the sweep's end.
    fn last(&self) -> Result<VertexId> {
        self.verts.last().copied().ok_or_else(no_cuts)
    }
}

/// The error of a corner with no vertices, which a sweep's two end cuts at least rule out.
fn no_cuts() -> Error {
    Error::Numerical("a swept corner has no vertices".into())
}

/// Each item with the one after it round the loop, the last with the first.
fn round<T>(items: &[T]) -> impl Iterator<Item = (&T, &T)> {
    items.iter().zip(items.iter().cycle().skip(1))
}

/// The item after each one round the loop.
fn following<T>(items: &[T]) -> impl Iterator<Item = &T> {
    round(items).map(|(_, next)| next)
}

#[cfg(test)]
mod tests;
