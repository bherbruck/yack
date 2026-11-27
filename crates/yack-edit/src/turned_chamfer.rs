// yack: a circular edge between two faces turned about its axis, chamfered by distances along each face
//! A chamfer on a full circular edge whose two faces are surfaces of revolution about the circle's
//! axis (a plane square to it, a coaxial cylinder, a sphere centred on it). In the half plane of the
//! axis each face is a curve through the edge's point; the chamfer's contact points lie the given
//! distances along them (along the arc on a sphere), and the chamfer is the cone through the two
//! circles they turn into. What it takes off is the solid of revolution, beyond the chord of the
//! contact points, of a strip as long as the chord that reaches past the corner.

use std::f64::consts::TAU;

use crate::{
    Error, Result, boolean,
    geom::{Curve, Surface},
    math::{Frame, Point3, Vec2},
    profile::Profile,
    sweep::revolve,
    tol,
    topo::{Brep, EdgeId, FaceId, FaceName},
};

/// One face's curve in the half plane of the axis through the edge point: where it goes on from
/// `at` the distance `d` along it, away from `other`'s side, and the outward normal there.
struct Face {
    surface: Surface,
    /// Outward normal at the edge point, in (radial, axial) coordinates.
    normal: Vec2,
}

fn perp(v: Vec2) -> Vec2 {
    Vec2::new(-v.y, v.x)
}

impl Face {
    /// The point `d` along the face from `at` (a sphere's centre taken as the origin) in the
    /// tangent direction `away`, and the outward normal there.
    fn along(&self, at: Vec2, away: Vec2, d: f64) -> Option<(Vec2, Vec2)> {
        match &self.surface {
            Surface::Plane(_) | Surface::Cylinder(_) => Some((at + away * d, self.normal)),
            Surface::Sphere(sp) => {
                let r = sp.radius;
                // The distance is the chord, straight from the edge in the half plane: it turns the point
                // about the centre by twice the arcsine of half the chord over the radius.
                let angle = 2.0 * (d / (2.0 * r)).clamp(-1.0, 1.0).asin();
                let turned = |s: f64| {
                    let (sn, cs) = (angle * s).sin_cos();
                    Vec2::new(at.x * cs - at.y * sn, at.x * sn + at.y * cs)
                };
                let (a, b) = (turned(1.0), turned(-1.0));
                let p = if (a - at).dot(away) >= (b - at).dot(away) { a } else { b };
                let out = if self.normal.dot(at) >= 0.0 { 1.0 } else { -1.0 };
                Some((p, p * (out / r)))
            }
            _ => None,
        }
    }
}

/// `s` with the circular edge `e`, between the face `face` and its other face, chamfered by
/// `along_face` along `face` and `along_other` along the other. The chamfer is named
/// `<feature>.bevel_0`. Declines an edge that is not a whole circle between two faces of the kinds
/// above about its axis, or distances that leave a face.
pub fn chamfer_two(s: &Brep, e: EdgeId, face: FaceId, along_face: f64, along_other: f64, feature: &str) -> Result<Brep> {
    let edge = s.edge(e);
    let Some(Curve::Circle(circle)) = &edge.curve else {
        return Err(Error::Unsupported("a turned chamfer on an edge that is not a circle".into()));
    };
    if (edge.range.1 - edge.range.0).abs() < TAU - 1e-6 { // tol: an angle: a whole turn
        return Err(Error::Unsupported("a turned chamfer on an arc of a circle".into()));
    }
    let [cf, cg] = s.edge_uses(e) else {
        return Err(Error::InvalidInput("a chamfered edge lies between two faces".into()));
    };
    let (f0, f1) = (s.coedge_face(*cf), s.coedge_face(*cg));
    let (first, second) = if f0 == face { (f0, f1) } else if f1 == face { (f1, f0) } else {
        return Err(Error::InvalidInput("the face is not one of the edge's two faces".into()));
    };
    if crate::topo::local::convexity(s, e) != Some(true) {
        return Err(Error::Unsupported("a turned chamfer on an edge that is not convex".into()));
    }
    let (o, up) = (circle.frame.origin, circle.frame.z);
    let radial = circle.frame.x;
    let rho_k = circle.radius;
    let k3 = o + radial * rho_k;
    // (radial, axial) of a point, and the point of (radial, axial).
    let flat = |p: Point3| Vec2::new((p - o).dot(radial), (p - o).dot(up));
    let space = |v: Vec2| o + radial * v.x + up * v.y;
    let k = flat(k3);
    let build = |f: FaceId| -> Result<Face> {
        let surface = s.face(f).surface.clone();
        let about_axis = match &surface {
            Surface::Plane(p) => p.frame.z.cross(up).norm() <= 1e-9, // tol: a cosine
            Surface::Cylinder(c) => c.frame.z.cross(up).norm() <= 1e-9 && (c.frame.origin - o).cross(up).norm() <= 1e-9, // tol: a cosine, and a distance
            Surface::Sphere(sp) => (sp.frame.origin - o).cross(up).norm() <= 1e-9, // tol: a distance
            _ => false,
        };
        if !about_axis {
            return Err(Error::Unsupported("a turned chamfer between faces not turned about the edge's axis".into()));
        }
        let n3 = crate::topo::local::face_normal_at(s, f, k3).ok_or_else(|| Error::Unsupported("a face with no normal at its edge".into()))?;
        Ok(Face { surface, normal: Vec2::new(n3.dot(radial), n3.dot(up)) })
    };
    let (a, b) = (build(first)?, build(second)?);
    // The sphere's centre is on the axis: its centre in (radial, axial) is (0, zc); put it at the origin.
    let shifted = |face: &Face, p: Vec2| match &face.surface {
        Surface::Sphere(sp) => p - Vec2::new(0.0, (sp.frame.origin - o).dot(up)),
        _ => p,
    };
    let unshifted = |face: &Face, p: Vec2| match &face.surface {
        Surface::Sphere(sp) => p + Vec2::new(0.0, (sp.frame.origin - o).dot(up)),
        _ => p,
    };
    // Along each face away from the edge: square to its normal, toward the other face's inside.
    let toward = |n: Vec2, other: Vec2| {
        let t = perp(n);
        if t.dot(other) < 0.0 { t } else { -t }
    };
    let (ta, tb) = (toward(a.normal, b.normal), toward(b.normal, a.normal));
    let go = |face: &Face, t: Vec2, d: f64| -> Result<(Vec2, Vec2)> {
        let (p, n) = face.along(shifted(face, k), t, d).ok_or_else(|| Error::Unsupported("a turned chamfer along a face of this kind".into()))?;
        Ok((unshifted(face, p), n))
    };
    let (pa, _) = go(&a, ta, along_face)?;
    let (pb, _) = go(&b, tb, along_other)?;
    // Both contact points lie on their faces (one may be on the axis: the chamfer then runs to its apex).
    for (face, p) in [(first, pa), (second, pb)] {
        if p.x < -tol::LINEAR || !crate::measure::face_at(s, space(p)).is_some_and(|(g, d)| g == face && d <= 10.0 * tol::LINEAR) {
            return Err(Error::InvalidInput("a chamfer whose contact leaves its face".into()));
        }
    }
    // The strip beyond the chord, reaching past the corner.
    let mid = (pa + pb) * 0.5;
    let gap = (k - mid).dot(k - mid).sqrt();
    if gap <= tol::LINEAR {
        return Err(Error::InvalidInput("a chamfer with no width".into()));
    }
    let out = (k - mid) * (1.0 / gap);
    let reach = 2.0 * gap + along_face + along_other;
    let strip = [pa, pb, pb + out * reach, pa + out * reach];
    let profile = Profile::polygon(&strip, &["chord", "b", "far", "a"]);
    // The sketch plane holds the axis and the radial direction of the edge point.
    let sketch = Frame::new(o, radial.cross(up), radial)?;
    let ring = revolve(&profile, &sketch, o, up, TAU, "\u{1}bevel")?;
    let mut done = boolean::difference(s, &ring)?;
    done.rename_faces(|n| match n.feature.strip_prefix('\u{1}') {
        Some(_) => vec![FaceName::new(feature, "bevel_0")],
        None => vec![n.clone()],
    });
    Ok(done)
}
