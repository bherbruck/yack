// yack: solids drawn again as the same solid in the form cadrs sweeps exactly: a screwed ball as a pipe, a cone from its point.
//! A screwed ball as the pipe of its radius with ball ends, and a pointed cone drawn from its point:
//! the same solids, in the forms cadrs sweeps exactly (split out of `yack_glue`, unchanged).

use super::{p3, v3, Made};
use crate::{Axis, BodyId, DefaultKernel, FaceId, History, Kernel, Primitive, Result};
use nalgebra::Point3;

/// The half-gap below which a tube about a helix of radius `radius` rising `c` a radian
/// (`turns` turns) reaches across itself: the least of its curvature's radius and half each
/// local least distance between two of its points (its reach). Points `t` apart are
/// `sqrt(4 R² sin²(t/2) + c² t²)` apart; the minima of that past `t = 0` are where the chord
/// meets the helix square at both ends.
pub(super) fn helix_reach(radius: f64, c: f64, turns: f64) -> f64 {
    let curvature = (radius * radius + c * c) / radius;
    let span = turns.abs() * std::f64::consts::TAU;
    let chord = |t: f64| (4.0 * radius * radius * (t / 2.0).sin().powi(2) + c * c * t * t).sqrt();
    let n = (span / 1e-3).ceil().clamp(16.0, 1e6) as usize;
    let mut least = f64::INFINITY;
    for i in 1..n {
        let (a, b, d) = (chord(span * (i - 1) as f64 / n as f64), chord(span * i as f64 / n as f64), chord(span * (i + 1) as f64 / n as f64));
        if b < a && b <= d {
            least = least.min(b / 2.0);
        }
    }
    curvature.min(least)
}

/// `body`, where it is a ball (one sphere face) screwed by `angle` radians about the line
/// through `point` along `axis`, rising `rise` a turn: the pipe of its radius along its centre's
/// helix (cadrs's sweep of its circle section), a ball of its radius united at each end. Only
/// where the helix's reach is over the radius (the tube never reaches across itself, so the
/// pipe and its balls are exactly what the ball passes through) and the volume is the tube's
/// (Pappus: π r² times the helix's length, and the two half balls) to `REL_POINT`; `None`
/// otherwise, for the sweep of the solid.
pub(super) fn ball_screwed_as_pipe(k: &mut DefaultKernel, body: BodyId, point: [f64; 3], axis: [f64; 3], angle: f64, rise: f64) -> Result<Option<Made>> {
    use crate::native::surface::Surface;
    let b = k.get(body)?;
    let [f] = b.faces.as_slice() else { return Ok(None) };
    let Surface::Sphere { f: fr, r } = &f.surf else { return Ok(None) };
    let (centre, r) = (fr.o, *r);
    let Some(a) = nalgebra::Unit::try_new(v3(axis), 0.0) else { return Ok(None) };
    let foot = p3(point) + a.into_inner() * (centre - p3(point)).dot(&a);
    let radial = centre - foot;
    let radius = radial.norm();
    if !(angle > 0.0 && rise > 0.0 && radius > crate::native::tol::POINT) {
        return Ok(None);
    }
    let (turns, c) = (angle / std::f64::consts::TAU, rise / std::f64::consts::TAU);
    if helix_reach(radius, c, turns) <= r * (1.0 + crate::native::tol::REL_POINT) {
        return Ok(None);
    }
    // The section: the circle of radius r about the centre, square to the helix there.
    let x = radial / radius;
    let along = a.cross(&x) * radius + a.into_inner() * c;
    let t = along.normalize();
    let u = x.cross(&t).normalize();
    // (A whole circle, as cadrs's runner draws the section: cadrs knows the sweep as a pipe.)
    let plane = crate::Plane::new(centre, x, t);
    let region = crate::Region { outer: crate::Loop { curves: vec![crate::Curve2::Circle { center: nalgebra::Point2::origin(), radius: r, source: Some(0) }] }, holes: vec![], source: Some(0) };
    let mut helix = crate::Helix::new(Axis { origin: foot, dir: a }, radius, rise, 0.0, turns);
    helix.x = nalgebra::Unit::new_normalize(x);
    let Ok(tube) = k.sweep_with(&crate::Profile::new(plane, vec![region]), &crate::SweepSpec::solid(helix.path())).and_then(|m| m.first()) else { return Ok(None) };
    let end = {
        let turn = nalgebra::Rotation3::from_axis_angle(&a, angle);
        foot + turn * radial + a.into_inner() * (rise * turns)
    };
    let mut parts = vec![tube];
    let mut out = tube;
    for at in [centre, end] {
        let ball = k.primitive(&Primitive::Sphere { center: at, radius: r, axis: a, seam: nalgebra::Unit::new_normalize(x) }).and_then(|m| m.first());
        let united = ball.and_then(|ball| {
            parts.push(ball);
            k.boolean(crate::BoolOp::Union, out, &[ball]).and_then(|m| m.first())
        });
        match united {
            Ok(u) => {
                parts.push(u);
                out = u;
            }
            Err(_) => {
                for p in parts {
                    k.release(p);
                }
                return Ok(None);
            }
        }
    }
    let length = turns.abs() * (std::f64::consts::TAU * radius).hypot(rise);
    let pappus = std::f64::consts::PI * r * r * length + 4.0 / 3.0 * std::f64::consts::PI * r.powi(3);
    let v = k.mass_properties(out)?.volume;
    let full = k.get(out)?.check_with(crate::native::topo::CheckLevel::Full).is_empty();
    for p in parts {
        if p != out {
            k.release(p);
        }
    }
    if !(full && (v - pappus).abs() <= crate::native::tol::REL_POINT * pappus) {
        k.release(out);
        return Ok(None);
    }
    Ok(Some((out, History { modified: vec![], generated: vec![], deleted: vec![] })))
}

/// `body`, where it is a solid cone (a cone face and a flat base) drawn other than from its
/// point, drawn again from its point with the same apex, axis, base and volume; with, for each of
/// the new body's faces, the face of `body` it is. `None` for any other body, or a cone already
/// drawn from its point.
pub(super) fn from_its_point(k: &mut DefaultKernel, body: BodyId) -> Result<Option<(BodyId, Vec<FaceId>)>> {
    use crate::native::curve::Curve;
    use crate::native::surface::Surface;
    let b = k.get(body)?;
    let [f0, f1] = b.faces.as_slice() else { return Ok(None) };
    let ((cone_i, cone), plane_i) = match (&f0.surf, &f1.surf) {
        (Surface::Cone { .. }, Surface::Plane { .. }) => ((0, f0), 1),
        (Surface::Plane { .. }, Surface::Cone { .. }) => ((1, f1), 0),
        _ => return Ok(None),
    };
    let Surface::Cone { f, r, a } = &cone.surf else { return Ok(None) };
    let (sin, cos) = a.sin_cos();
    if sin.abs() <= crate::native::tol::ANGULAR || (*r == 0.0 && sin > 0.0) {
        return Ok(None);
    }
    let apex = f.o + f.z * (-r / sin * cos);
    // The base: the plane face's one circle.
    let circles: Vec<(Point3<f64>, f64)> = b.faces[plane_i]
        .loops
        .iter()
        .flatten()
        .filter_map(|c| match &b.edges[c.edge].curve {
            Curve::Circle { c, r, .. } => Some((*c, *r)),
            _ => None,
        })
        .collect();
    let [(centre, radius)] = circles.as_slice() else { return Ok(None) };
    let to_base = centre - apex;
    let height = to_base.norm();
    if !(height > crate::native::tol::POINT && *radius > crate::native::tol::POINT) {
        return Ok(None);
    }
    let was = k.mass_properties(body)?.volume;
    let cone_made = k.primitive(&Primitive::Cone {
        base: apex,
        axis: nalgebra::Unit::new_normalize(to_base),
        base_radius: 0.0,
        top_radius: *radius,
        height,
    })?;
    let id = cone_made.first()?;
    let nb = k.get(id)?;
    let faces: Option<Vec<FaceId>> = nb
        .faces
        .iter()
        .map(|g| match g.surf {
            Surface::Cone { .. } => Some(FaceId(cone_i as u64)),
            Surface::Plane { .. } => Some(FaceId(plane_i as u64)),
            _ => None,
        })
        .collect();
    let same = (k.mass_properties(id)?.volume - was).abs() <= crate::native::tol::REL_POINT * was.abs();
    match faces {
        Some(faces) if same && faces.len() == 2 => Ok(Some((id, faces))),
        _ => {
            k.release(id);
            Ok(None)
        }
    }
}
