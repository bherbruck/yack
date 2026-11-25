// yack: a line against each kind of surface, in closed form or over the profile's pieces.
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of csi.rs (M3 hardening): the plane, sphere and cone cases out of the
// dispatcher, named rounding limits, iterators instead of indexing; the same arithmetic in the same
// order.
//! A line `o + t·d` (`d` unit) against each kind of surface: in closed form against the quadrics
//! and the torus, and against extrusions, surfaces of revolution and ruled surfaces as a function of
//! the profile's parameter over its polynomial pieces. Splines are [`super::spline`]'s.

use std::f64::consts::TAU;

use super::{
    Hits,
    pieces::{Piece, along, piece_roots, pieces, ruled_pieces},
    roots::{in_order, roots_between, sampled_roots, stationary_points, trig_roots, with_ends},
    signed_distance,
};
use crate::{
    geom::{Cone, Curve, Cylinder, Extrusion, Plane, Revolution, Ruled, Sphere, Surface, Torus},
    math::{Point3, Vec3},
    tol,
};

/// Below this a quadratic's leading or linear coefficient is zero.
const NO_COEFFICIENT: f64 = super::roots::UNDERFLOW;

/// Two of a ruled surface's joins closer than this, as its own parameter, are one.
const JOIN_ROUNDING: f64 = 1e-12; // tol: parameter rounding

/// Samples along each piece of a profile that has no polynomial pieces.
const PROFILE_SAMPLES: usize = 24;

/// The crossing of a line with a plane, or the whole line where it lies in the plane.
pub(super) fn line_plane(o: Point3, d: Vec3, p: &Plane) -> Hits {
    let dn = d.dot(p.frame.z);
    let h = (o - p.frame.origin).dot(p.frame.z);
    if dn.abs() <= tol::ANGULAR {
        Hits { ts: Vec::new(), on_surface: h.abs() <= tol::LINEAR }
    } else {
        Hits { ts: vec![-h / dn], on_surface: false }
    }
}

/// The crossings of a line with the cylinder.
pub(super) fn line_cylinder(o: Point3, d: Vec3, cyl: &Cylinder) -> Hits {
    let lo = cyl.frame.to_local(o);
    let ld = cyl.frame.dir_to_local(d);
    let a = ld.x * ld.x + ld.y * ld.y;
    if a.sqrt() <= tol::ANGULAR {
        return Hits {
            ts: Vec::new(),
            on_surface: (lo.x.hypot(lo.y) - cyl.radius).abs() <= tol::LINEAR,
        };
    }
    let t_near = -(lo.x * ld.x + lo.y * ld.y) / a;
    let m = (lo.x + t_near * ld.x).hypot(lo.y + t_near * ld.y);
    let r = cyl.radius;
    let ts = if m > r + tol::LINEAR {
        Vec::new()
    } else if m >= r - tol::LINEAR {
        vec![t_near]
    } else {
        let h = ((r - m) * (r + m) / a).sqrt();
        vec![t_near - h, t_near + h]
    };
    Hits { ts, on_surface: false }
}

/// The crossings of a line with the sphere.
pub(super) fn line_sphere(o: Point3, d: Vec3, s: &Surface, sp: &Sphere) -> Hits {
    let oc = o - sp.frame.origin;
    let b = oc.dot(d);
    let c0 = oc.norm2() - sp.radius * sp.radius;
    line_quadric(o, d, s, [1.0, 2.0 * b, c0])
}

/// The crossings of a line with the cone, which is no plane.
pub(super) fn line_cone(o: Point3, d: Vec3, s: &Surface, cone: &Cone) -> Hits {
    let lo = cone.frame.to_local(o);
    let ld = cone.frame.dir_to_local(d);
    let k = cone.half_angle.tan();
    let r0 = cone.radius + k * lo.z;
    let qa = ld.x * ld.x + ld.y * ld.y - k * k * ld.z * ld.z;
    let qb = 2.0 * (lo.x * ld.x + lo.y * ld.y - k * r0 * ld.z);
    let qc = lo.x * lo.x + lo.y * lo.y - r0 * r0;
    line_quadric(o, d, s, [qa, qb, qc])
}

/// The crossings of a line with a quadric `s` whose implicit form along the line is
/// `qa·t² + qb·t + qc`; a pair closer than the tolerance band is one touching point.
fn line_quadric(o: Point3, d: Vec3, s: &Surface, [qa, qb, qc]: [f64; 3]) -> Hits {
    let at = |t: f64| signed_distance(s, o + d * t).abs();
    if qa.abs() < NO_COEFFICIENT {
        if qb.abs() < NO_COEFFICIENT {
            return Hits { ts: Vec::new(), on_surface: at(0.0) <= tol::LINEAR };
        }
        return Hits { ts: vec![-qc / qb], on_surface: false };
    }
    let t_near = -qb / (2.0 * qa);
    let disc = qb * qb - 4.0 * qa * qc;
    let ts = if disc <= 0.0 {
        if at(t_near) <= tol::LINEAR { vec![t_near] } else { Vec::new() }
    } else {
        let q = -0.5 * (qb + qb.signum() * disc.sqrt());
        let (mut t1, mut t2) = (q / qa, qc / q);
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        if at(t_near) <= tol::LINEAR { vec![t_near] } else { vec![t1, t2] }
    };
    Hits { ts, on_surface: false }
}

/// A line against an extrusion, whose profile is carried along `dir`: the line meets it where the
/// profile passes through the plane that holds the line and runs along `dir`. That is a function
/// of the profile's parameter alone, the profile's signed distance from that plane: a sine wave
/// on a circle or an ellipse, and on a spline a polynomial over each piece. Sampling along the
/// line instead steps over crossings closer than its step, as a gear's root has.
pub(super) fn line_extrusion(o: Point3, d: Vec3, e: &Extrusion) -> Hits {
    let across = d - e.dir * d.dot(e.dir);
    let Some(n) = e.dir.cross(across).unit() else {
        return Hits::default();
    };
    let c = &e.curve;
    let height = |u: f64| n.dot(c.eval(u) - o);
    let (lo, hi) = c.domain();
    let on_profile = match c {
        Curve::Circle(ci) => trig_roots(
            n.dot(ci.frame.origin - o),
            ci.radius * n.dot(ci.frame.x),
            ci.radius * n.dot(ci.frame.y),
            (0.0, TAU),
        ),
        Curve::Ellipse(el) => trig_roots(
            n.dot(el.frame.origin - o),
            el.major * n.dot(el.frame.x),
            el.minor * n.dot(el.frame.y),
            (0.0, TAU),
        ),
        Curve::Line(l) => {
            let slope = n.dot(l.dir);
            if slope.abs() <= tol::ANGULAR {
                Hits { ts: Vec::new(), on_surface: n.dot(l.origin - o).abs() <= tol::LINEAR }
            } else {
                Hits { ts: vec![-n.dot(l.origin - o) / slope], on_surface: false }
            }
        }
        _ => match pieces(c, (lo, hi)) {
            Some(pieces) => {
                let (ts, flat) = piece_roots(&pieces, 1, &height, &mut |u| height(u));
                Hits { ts: if flat { Vec::new() } else { ts }, on_surface: flat }
            }
            // An intersection curve is no polynomial: sampled along the profile, piece by piece.
            None => {
                let cuts = with_ends(c.breakpoints(lo, hi), (lo, hi));
                let (ts, on) = sampled_roots(&mut |u| height(u), &cuts, PROFILE_SAMPLES);
                Hits { ts, on_surface: on }
            }
        },
    };
    if on_profile.on_surface {
        return on_profile;
    }
    let ts =
        on_profile.ts.into_iter().map(|u| (c.eval(u) - o).dot(across) / across.norm2()).collect();
    Hits { ts: in_order(ts), on_surface: false }
}

/// A line against a surface of revolution. Each point of the profile is a ring about the axis, at
/// a height and of a radius; a line that climbs passes that height once, at a known distance from
/// the axis, and meets the ring where the two radii agree. Their squares' difference is a
/// polynomial over each piece of the profile. A line square to the axis keeps one height: it meets
/// the rings the profile draws at that height, each in the two roots of a quadratic. `None` where
/// the profile has no pieces.
pub(super) fn line_revolution(o: Point3, d: Vec3, r: &Revolution) -> Option<Hits> {
    let c = &r.curve;
    let pieces = pieces(c, c.domain())?;
    let rel = o - r.origin;
    let (z0, dz) = (rel.dot(r.axis), d.dot(r.axis));
    // The line's squared distance from the axis at `t`: a·t² + b·t + c0.
    let (a, b, c0) = (1.0 - dz * dz, 2.0 * (rel.dot(d) - z0 * dz), rel.norm2() - z0 * z0);
    // A point of the profile as a ring: its height and its squared radius.
    let ring = |v: f64| {
        let q = c.eval(v) - r.origin;
        let z = q.dot(r.axis);
        (z, q.norm2() - z * z)
    };
    if dz.abs() > tol::ANGULAR {
        let at = |v: f64| (ring(v).0 - z0) / dz;
        let line2 = |t: f64| (a * t + b) * t + c0;
        let raw = |v: f64| line2(at(v)) - ring(v).1;
        let mut mm = |v: f64| line2(at(v)).max(0.0).sqrt() - ring(v).1.max(0.0).sqrt();
        let (vs, flat) = piece_roots(&pieces, 2, &raw, &mut mm);
        return Some(Hits {
            ts: if flat { Vec::new() } else { in_order(vs.into_iter().map(at).collect()) },
            on_surface: flat,
        });
    }
    let level = |v: f64| ring(v).0 - z0;
    let (vs, flat) = piece_roots(&pieces, 1, &level, &mut |v| level(v));
    if flat {
        // The profile runs square to the axis at the line's height: a flat ring the line lies in.
        return Some(Hits { ts: Vec::new(), on_surface: true });
    }
    let mut ts = Vec::new();
    for v in vs {
        let radius = ring(v).1.max(0.0).sqrt();
        // The line's nearest approach to the axis, and where.
        let (near, at) = ((c0 - b * b / (4.0 * a)).max(0.0).sqrt(), -b / (2.0 * a));
        if radius < near - tol::LINEAR {
            continue;
        }
        let half = ((radius * radius - near * near).max(0.0) / a).sqrt();
        if half <= tol::LINEAR {
            ts.push(at);
        } else {
            ts.extend([at - half, at + half]);
        }
    }
    Some(Hits { ts: in_order(ts), on_surface: false })
}

/// A line against a ruled surface, whose rulings run from one curve to the other at each
/// parameter: the line meets a ruling where the two lines lie in one plane, the triple product of
/// the ruling, the line's direction and the step between them vanishing. Over each piece of both
/// curves that is a polynomial, of the sum of their degrees. `None` where either curve has no
/// pieces, or is a circle or an ellipse, whose angle is no polynomial in the other's parameter.
pub(super) fn line_ruled(o: Point3, d: Vec3, r: &Ruled) -> Option<Hits> {
    let polynomial = |c: &Curve| matches!(c, Curve::Line(_) | Curve::BSpline(_) | Curve::Nurbs(_));
    if !polynomial(&r.a) || !polynomial(&r.b) {
        return None;
    }
    let (pa, pb) = (pieces(&r.a, r.a_range)?, pieces(&r.b, r.b_range)?);
    // Both curves' joins, as the surface's own parameter.
    let back = |t: f64, (lo, hi): (f64, f64)| (t - lo) / (hi - lo);
    let joins = |ps: &[Piece], range| -> Vec<f64> {
        ps.iter().map(|p| back(p.param(0.0), range)).collect()
    };
    let mut cuts: Vec<f64> = vec![0.0, 1.0];
    cuts.extend(joins(&pa, r.a_range));
    cuts.extend(joins(&pb, r.b_range));
    cuts.retain(|u| (0.0..=1.0).contains(u));
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|x, y| (*x - *y).abs() <= JOIN_ROUNDING);
    let (Some(first_a), Some(first_b)) = (pa.first(), pb.first()) else { return None };
    let both = ruled_pieces(r, &cuts, first_a.degree + first_b.degree);
    // The ruling at `u`: its foot, and the step to its other end.
    let ruling = |u: f64| {
        let foot = r.a.eval(along(r.a_range, u));
        (foot, r.b.eval(along(r.b_range, u)) - foot)
    };
    let raw = |u: f64| {
        let (foot, step) = ruling(u);
        (foot - o).dot(step.cross(d))
    };
    // The distance between the line and the ruling's line: the triple product over the area the
    // two directions span.
    let mut mm = |u: f64| {
        let (foot, step) = ruling(u);
        let span = step.cross(d);
        let area = span.norm();
        if area > 0.0 { (foot - o).dot(span) / area } else { 0.0 }
    };
    let (us, flat) = piece_roots(&both, 1, &raw, &mut mm);
    if flat {
        return Some(Hits { ts: Vec::new(), on_surface: true });
    }
    let ts = us
        .into_iter()
        .filter_map(|u| {
            let (foot, step) = ruling(u);
            let span = d.cross(step);
            // A ruling along the line meets it nowhere in particular.
            (span.norm2() > tol::ANGULAR * tol::ANGULAR * step.norm2())
                .then(|| (foot - o).cross(step).dot(span) / span.norm2())
        })
        .collect();
    Some(Hits { ts: in_order(ts), on_surface: false })
}

/// A line against a torus: the torus is the zero set of the quartic
/// `(|p|² + R² − r²)² − 4R²ρ²` along the line, so the quartic's stationary points split the line
/// into stretches with one crossing at most and mark every place it can touch. That holds for a
/// tube that reaches its axis as for a ring: the quartic is the product of the two sheets' own.
pub(super) fn line_torus(o: Point3, d: Vec3, t: &Torus, range: (f64, f64)) -> Hits {
    let (lo, ld) = (t.frame.to_local(o), t.frame.dir_to_local(d));
    let (big, small) = (t.major, t.minor);
    // Only where the line is within the torus's bounding sphere.
    let (b, c) = (lo.dot(ld), lo.dot(lo) - (big + small).powi(2));
    let disc = b * b - c;
    if disc < 0.0 {
        return Hits::default();
    }
    let (t0, t1) = ((-b - disc.sqrt()).max(range.0), (-b + disc.sqrt()).min(range.1));
    if t0 > t1 {
        return Hits::default();
    }
    let k = big * big - small * small;
    // The quartic's coefficients: the square of `|p|² + R² − r²`, less `4R²ρ²`, term by term.
    let sq = [lo.dot(lo) + k, 2.0 * b, 1.0];
    let rho =
        [lo.x * lo.x + lo.y * lo.y, 2.0 * (lo.x * ld.x + lo.y * ld.y), ld.x * ld.x + ld.y * ld.y];
    let mut quartic = [0.0; 5];
    for (i, (&si, &ri)) in sq.iter().zip(&rho).enumerate() {
        for (q, &sj) in quartic.iter_mut().skip(i).zip(&sq) {
            *q += si * sj;
        }
        if let Some(q) = quartic.get_mut(i) {
            *q -= 4.0 * big * big * ri;
        }
    }
    let knots = with_ends(stationary_points(&quartic, t0, t1), (t0, t1));
    // The distance to the nearer of the surface's two sheets, with the quartic's sign: the tube
    // about the circle on the near side of the axis and, where the tube reaches past the axis, its
    // far side come round. A ring torus has the first alone; the second lies always outside it.
    let mut f = |s: f64| {
        let p = lo + ld * s;
        let rho = p.x.hypot(p.y);
        let (near, far) = ((rho - big).hypot(p.z) - small, (rho + big).hypot(p.z) - small);
        if near.abs() <= far.abs() { near * far.signum() } else { far * near.signum() }
    };
    Hits { ts: roots_between(&mut f, &knots), on_surface: false }
}
