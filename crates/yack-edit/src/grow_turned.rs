// yack: a turned solid grown by a ball, as the union of what its faces, edges and tips sweep (w-h1).
//! A solid turned about one axis, grown by a ball of radius `d`: every point within `d` of it has
//! its nearest point inside a face, on a convex edge or at a convex tip, never on a concave edge.
//! So the growth is the solid, each face's slab (the face swept `d` along its outward normal), a
//! tube about each convex edge and a ball at each convex tip on the axis: their union. Each piece
//! is turned from its meridian, in the half plane of the axis: a plane square to the axis or a
//! cone gives a quadrilateral, a cylinder a band, a torus or a ball an annular sector.
//!
//! This builds what the faces' own moves cannot follow: a groove that closes while its rims still
//! round, where moving the faces through would drop the rims and the notch they leave.

use std::f64::consts::{PI, TAU};

use crate::{
    Error, Result, boolean,
    geom::{Curve, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    profile::{Profile, ProfileLoop, Segment},
    sweep, tol,
    topo::{Brep, FaceId},
};

/// `s` grown by `d`, where `s` is turned about one axis (see the module). Declines a solid with a
/// face of another kind or off the axis, an edge that is no circle about it, or a concave curved
/// face the growth takes past its centre.
pub fn grow_turned(s: &Brep, d: f64, feature: &str) -> Result<Brep> {
    let no = |why: &str| Error::Unsupported(format!("a turned solid's growth: {why}"));
    if d.is_nan() || d <= tol::LINEAR {
        return Err(Error::InvalidInput("a growth must be positive".into()));
    }
    let (o, k) = axis(s).ok_or_else(|| no("no single axis the faces turn about"))?;
    let at = |p: Point3| {
        let q = p - o;
        let z = q.dot(k);
        Vec2::new((q - k * z).norm(), z)
    };
    let mut pieces: Vec<Brep> = vec![s.clone()];
    for (n, f) in s.face_ids().enumerate() {
        let region = slab(s, f, d, (o, k), &at).ok_or_else(|| no("a face it cannot sweep"))?;
        pieces.push(turn(&region, (o, k), &format!("grown_f{n}"), feature)?);
    }
    for (n, e) in s.edge_ids().enumerate() {
        let x = s.edge(e);
        match &x.curve {
            None => {
                // A tip on the axis: a ball where a cone comes to its point.
                let tip = s.vertex(x.start).point;
                let cone = s.edge_uses(e).iter().any(|&c| matches!(s.face(s.coedge_face(c)).surface, Surface::Cone(_)));
                if cone {
                    pieces.push(yack_build::primitive::sphere(&Frame::WORLD.translated(tip), d, feature)?);
                }
            }
            Some(Curve::Circle(c)) => {
                let &[cf, cg] = s.edge_uses(e) else { return Err(no("an edge not between two faces")) };
                if s.coedge_face(cf) == s.coedge_face(cg) || crate::topo::local::convexity(s, e) != Some(true) {
                    continue;
                }
                if c.frame.z.cross(k).norm() > tol::TANGENT || at(c.frame.origin).x > tol::LINEAR {
                    return Err(no("a convex edge that is no circle about the axis"));
                }
                let centre = at(c.frame.origin + c.frame.x * c.radius);
                pieces.push(turn(&tube(centre, d), (o, k), &format!("round_e{n}"), feature)?);
            }
            Some(_) => {
                // A seam along a meridian is no edge of the solid's shape.
                let &[cf, cg] = s.edge_uses(e) else { return Err(no("an edge not between two faces")) };
                if s.coedge_face(cf) != s.coedge_face(cg) {
                    return Err(no("an edge that is no circle about the axis"));
                }
            }
        }
    }
    let refs: Vec<&Brep> = pieces.iter().collect();
    boolean::union_all(&refs)
}

/// The axis every face turns about: a point on it and its direction, from the first curved face.
/// `None` if a face is off it or of a kind that does not turn.
fn axis(s: &Brep) -> Option<(Point3, Vec3)> {
    let (o, k) = s.face_ids().find_map(|f| match &s.face(f).surface {
        Surface::Cylinder(c) => Some((c.frame.origin, c.frame.z)),
        Surface::Cone(c) => Some((c.frame.origin, c.frame.z)),
        Surface::Torus(t) => Some((t.frame.origin, t.frame.z)),
        _ => None,
    })?;
    let on = |p: Point3| (p - o).cross(k).norm() <= tol::LINEAR;
    let along = |z: Vec3| z.cross(k).norm() <= tol::TANGENT;
    s.face_ids()
        .all(|f| match &s.face(f).surface {
            Surface::Plane(p) => along(p.frame.z),
            Surface::Cylinder(c) => along(c.frame.z) && on(c.frame.origin),
            Surface::Cone(c) => along(c.frame.z) && on(c.frame.origin),
            Surface::Torus(t) => along(t.frame.z) && on(t.frame.origin),
            Surface::Sphere(b) => on(b.frame.origin),
            _ => false,
        })
        .then_some((o, k))
}

/// A closed region of the meridian half plane: lines and arcs, anticlockwise.
enum Piece {
    Line(Vec2, Vec2),
    /// Centre, radius, from angle, sweep (signed).
    Arc(Vec2, f64, f64, f64),
}

/// The region a disc of radius `d` about `centre` covers, clipped to the half plane: a tube's
/// meridian.
fn tube(centre: Vec2, d: f64) -> Vec<Piece> {
    if centre.x > d + tol::LINEAR {
        return vec![Piece::Arc(centre, d, 0.0, TAU)];
    }
    // The disc reaches the axis: its part right of it, closed along the axis.
    let h = (d * d - centre.x * centre.x).max(0.0).sqrt();
    let top = h.atan2(-centre.x);
    vec![Piece::Arc(centre, d, -top, 2.0 * top), Piece::Line(Vec2::new(0.0, centre.y + h), Vec2::new(0.0, centre.y - h))]
}

/// Face `f`'s slab in the meridian half plane: the face's meridian piece swept `d` along its
/// outward normal. `None` for a face it does not know or a concave curve the sweep takes past its
/// centre.
fn slab(s: &Brep, f: FaceId, d: f64, (o, k): (Point3, Vec3), at: &dyn Fn(Point3) -> Vec2) -> Option<Vec<Piece>> {
    let face = s.face(f);
    // The face's rails (circles about the axis) and tips (on it), in the half plane.
    let mut ends: Vec<Vec2> = Vec::new();
    for &lp in &face.loops {
        for &c in &s.lp(lp).coedges {
            let x = s.edge(s.coedge(c).edge);
            match &x.curve {
                // A rail: a circle about the axis (a seam round a tube is no rail).
                Some(Curve::Circle(circle)) if circle.frame.z.cross(k).norm() <= tol::TANGENT && at(circle.frame.origin).x <= tol::LINEAR => {
                    ends.push(at(circle.frame.origin + circle.frame.x * circle.radius));
                }
                None => ends.push(at(s.vertex(x.start).point)),
                Some(_) => {}
            }
        }
    }
    ends.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    ends.dedup_by(|a, b| a.dist(*b) <= tol::LINEAR);
    // The outward normal in the half plane, at a point of the face.
    let inner = crate::topo::face_domain(s, f).interior_point()?;
    let p = face.surface.eval(inner.x, inner.y);
    let n3 = {
        let n = face.surface.normal(inner.x, inner.y);
        if face.reversed { -n } else { n }
    };
    let q = p - o;
    let radial = (q - k * q.dot(k)).unit().unwrap_or(Vec3::ZERO);
    let n = Vec2::new(n3.dot(radial), n3.dot(k));
    let quad = |a: Vec2, b: Vec2| {
        let (a2, b2) = (a + n * d, b + n * d);
        // Clipped at the axis where a hole's wall sweeps across it.
        let clip = |v: Vec2| Vec2::new(v.x.max(0.0), v.y);
        let pts = [a, b, clip(b2), clip(a2)];
        let area: f64 = pts.iter().zip(pts.iter().cycle().skip(1)).map(|(u, v)| u.x * v.y - v.x * u.y).sum();
        let pts: Vec<Vec2> = if area < 0.0 { pts.iter().rev().copied().collect() } else { pts.to_vec() };
        pts.iter()
            .zip(pts.iter().cycle().skip(1))
            .filter(|(u, v)| u.dist(**v) > tol::LINEAR)
            .map(|(&u, &v)| Piece::Line(u, v))
            .collect::<Vec<_>>()
    };
    match &face.surface {
        Surface::Plane(_) => {
            // An annulus or a disc: from the axis (a disc has one rail) out to its rim.
            let z = ends.first()?.y;
            let (r0, r1) = if ends.len() == 1 { (0.0, ends.first()?.x) } else { (ends.first()?.x, ends.last()?.x) };
            Some(quad(Vec2::new(r0, z), Vec2::new(r1, z)))
        }
        Surface::Cylinder(_) | Surface::Cone(_) => {
            let (&a, &b) = (ends.first()?, ends.last()?);
            (ends.len() == 2).then(|| quad(a, b))
        }
        Surface::Torus(_) | Surface::Sphere(_) => {
            let (centre, r) = match &face.surface {
                Surface::Torus(t) => (at(t.frame.origin + Vec3::ZERO) + Vec2::new(t.major, 0.0), t.minor),
                Surface::Sphere(b) => (at(b.frame.origin), b.radius),
                _ => return None,
            };
            let [a, b] = ends.as_slice() else { return None };
            // The arc from `a` to `b` through the face's own point.
            let ang = |v: Vec2| (v.y - centre.y).atan2(v.x - centre.x);
            let (ta, tb, tm) = (ang(*a), ang(*b), ang(at(p)));
            let wrap = |t: f64| (t - ta).rem_euclid(TAU);
            let (from, sweep) = if wrap(tm) < wrap(tb) { (ta, wrap(tb)) } else { (tb, (ta - tb).rem_euclid(TAU)) };
            let convex = !face.reversed;
            let (r0, r1) = if convex { (r, r + d) } else { (r - d, r) };
            if r0 <= tol::LINEAR || sweep <= 0.0 || sweep >= PI * 2.0 - tol::ANGULAR {
                return None;
            }
            let pt = |rad: f64, t: f64| centre + Vec2::new(t.cos(), t.sin()) * rad;
            if (0..=16).any(|i| pt(r1, from + sweep * f64::from(i) / 16.0).x < -tol::LINEAR) {
                return None;
            }
            Some(vec![
                Piece::Arc(centre, r1, from, sweep),
                Piece::Line(pt(r1, from + sweep), pt(r0, from + sweep)),
                Piece::Arc(centre, r0, from + sweep, -sweep),
                Piece::Line(pt(r0, from), pt(r1, from)),
            ])
        }
        _ => None,
    }
}

/// `region` turned once about the axis `(o, k)`, its faces named `<feature>.<name>`.
fn turn(region: &[Piece], (o, k): (Point3, Vec3), name: &str, feature: &str) -> Result<Brep> {
    let segments = region
        .iter()
        .map(|piece| {
            crate::named(
                match *piece {
                    Piece::Line(a, b) => Segment::Line { a, b },
                    Piece::Arc(center, radius, start, sweep) if (sweep.abs() - TAU).abs() <= tol::ANGULAR => {
                        let _ = start;
                        Segment::Circle { center, radius, ccw: sweep > 0.0 }
                    }
                    Piece::Arc(center, radius, start, sweep) => Segment::Arc { center, radius, start, sweep },
                },
                name,
            )
        })
        .collect();
    let profile = Profile { loops: vec![ProfileLoop { segments }] };
    let axes = Frame::from_axis(o, k)?;
    sweep::revolve(&profile, &Frame::new(o, -axes.y, axes.x)?, o, axes.z, TAU, feature)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    fn declines(r: Result<Brep>, why: &str) {
        assert!(matches!(&r, Err(Error::Unsupported(m)) if m.contains(why)), "{:?}", r.map(|_| ()));
    }

    #[test]
    fn a_solid_that_is_not_turned_declines() {
        let block = crate::primitive::block(&Frame::WORLD, Vec3::new(4.0, 3.0, 2.0), "b").unwrap();
        declines(grow_turned(&block, 1.0, "o"), "no single axis");
    }

    #[test]
    fn a_ball_off_the_axis_declines() {
        // A drum with a ball sunk into its top away from its axis: the ball turns about no axis
        // the drum shares.
        let drum = crate::primitive::cylinder(&Frame::WORLD, 5.0, 4.0, "c").unwrap();
        let ball = crate::primitive::sphere(&Frame::WORLD.translated(Vec3::new(2.0, 0.0, 4.0)), 1.5, "s").unwrap();
        let s = boolean::union(&drum, &ball).unwrap();
        declines(grow_turned(&s, 1.0, "o"), "no single axis");
    }

    #[test]
    fn a_concave_round_grown_past_its_radius_declines() {
        // A stepped puck whose root is rounded 0.5, grown by 1: the root's slab would sweep past
        // the round's centre, which this construction does not build.
        let at = |z: f64| Frame::WORLD.translated(Vec3::new(0.0, 0.0, z));
        let sharp = boolean::union(
            &crate::primitive::cylinder(&Frame::WORLD, 6.0, 4.0, "p").unwrap(),
            &crate::primitive::cylinder(&at(4.0), 2.0, 4.0, "b").unwrap(),
        )
        .unwrap();
        let root = crate::concave_edges(&sharp);
        let rooted = crate::blend::fillet_edges(&sharp, &root, 0.5, "r").unwrap();
        declines(grow_turned(&rooted, 1.0, "o"), "a face it cannot sweep");
    }
}
