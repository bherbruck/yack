// yack: the ball an outward offset leaves round a cone's apex
//! The points within `d` of a cone's apex are a ball of radius `d` about it, which meets the cone
//! offset by `d` along the circle where the two are tangent. Moving the cone's face out leaves its
//! tip sharp past that circle, so the tip is cut off there and the ball's cap put in its place.

use std::f64::consts::{PI, TAU};

use crate::{
    Error, Result, boolean,
    geom::Surface,
    math::{Frame, Vec2},
    named,
    profile::{Profile, ProfileLoop, Segment},
    sweep::revolve,
    tol, topo,
    topo::{Brep, FaceId},
};

/// The faces of `b` that come to a point: a cone's side, where it runs to a degenerate edge.
pub(crate) fn apex_faces(b: &Brep) -> Vec<FaceId> {
    let mut found: Vec<FaceId> = b
        .coedge_ids()
        .filter(|&c| b.edge(b.coedge(c).edge).curve.is_none())
        .map(|c| b.coedge_face(c))
        .filter(|&f| matches!(b.face(f).surface, Surface::Cone(_)))
        .collect();
    found.dedup();
    found
}

/// `b`, the solid of faces moved out by `d`, with the tip of its cone's apex rounded as the points
/// within `d` of the solid round it; `b` itself where no face comes to a point.
pub(crate) fn round_apexes(b: &Brep, d: f64, feature: &str) -> Result<Brep> {
    let apexes = apex_faces(b);
    let &[f] = apexes.as_slice() else {
        return if apexes.is_empty() {
            Ok(b.clone())
        } else {
            Err(Error::Unsupported("an outward offset of a solid with several apexes".into()))
        };
    };
    let face = b.face(f);
    let Surface::Cone(c) = &face.surface else {
        return Err(Error::Unsupported("an apex that is not a cone's".into()));
    };
    let (sa, ca) = c.half_angle.sin_cos();
    if sa.abs() <= 1e-9 { // tol: a half angle this small is a cylinder, which has no apex
        return Err(Error::Unsupported("an offset of a cone with no apex".into()));
    }
    let z = c.frame.z;
    // Convex where the solid's outward normal leans away from the axis; a hollow cone's apex is
    // reached from inside.
    let near = face.surface.eval(0.0, -0.5 * c.radius / sa);
    let radial = near - (c.frame.origin + z * (near - c.frame.origin).dot(z));
    let leans_out = topo::local::face_normal_at(b, f, near).is_some_and(|n| n.dot(radial) > 0.0);
    if !leans_out {
        return Err(Error::Unsupported("an outward offset of a hollow cone's apex".into()));
    }
    // The surface moved along its own normal, by `dn` (outward by `d` either way).
    let dn = if face.reversed { -d } else { d };
    // The cone offset by `d` keeps its parameters, its apex `d / sin a` along the axis from the old
    // one, which is the ball's centre; the ball touches the cone where the foot of the perpendicular
    // from the centre falls, at generator length `vt` from the frame's origin.
    let apex = c.frame.origin + z * (-c.radius / sa * ca);
    let centre = apex + z * (dn / sa);
    let vt = (centre - c.frame.origin).dot(z) * ca - c.radius * sa;
    let toward_apex = ((-c.radius / sa - vt) * ca).signum();
    let up = z * toward_apex;
    let ring = c.frame.origin + z * (vt * ca);
    let w = (ring - centre).dot(up);
    if w.abs() >= d {
        return Err(Error::Unsupported("an apex whose ball does not reach the cone".into()));
    }
    // The edges of every other face must lie below the plane of the ring.
    let level = ring.dot(up);
    let others = b.face_ids().filter(|&g| g != f).flat_map(|g| b.face(g).loops.iter().flat_map(|&lp| b.lp(lp).coedges.iter().map(|&co| b.coedge(co).edge)));
    for e in others {
        if topo::local::edge_points(b, e, 9).iter().any(|p| p.dot(up) > level + tol::LINEAR) {
            return Err(Error::Unsupported("an apex whose tip reaches another face".into()));
        }
    }
    let rho = (d * d - w * w).sqrt();
    let from = w.atan2(rho);
    let meridian = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                named(Segment::Line { a: Vec2::new(0.0, 0.0), b: Vec2::new(rho, 0.0) }, "cut"),
                named(Segment::Arc { center: Vec2::new(0.0, -w), radius: d, start: from, sweep: PI / 2.0 - from }, "corner"),
                named(Segment::Line { a: Vec2::new(0.0, d - w), b: Vec2::new(0.0, 0.0) }, "axis"),
            ],
        }],
    };
    let axes = Frame::from_axis(ring, up)?;
    let cap = revolve(&meridian, &Frame::new(ring, -axes.y, axes.x)?, ring, axes.z, TAU, feature)?;
    let blunt = boolean::difference(b, &boolean::half_space(b, ring, up)?)?;
    boolean::union(&blunt, &cap)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::measure;
    use std::collections::BTreeMap;

    /// A probe, not a check: the frustum shelled open at its top, its cavity cone closing below the
    /// opening, followed by shares of the move; the report is the failure message.
    #[test]
    #[ignore = "probe: reports how the cavity of a frustum closing below its top comes out"]
    fn probe_frustum_closing_below_its_top() {
        let b = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 4.0, "c").unwrap();
        let face_of = |pred: &dyn Fn(&Surface) -> bool| b.face_ids().find(|&f| pred(&b.face(f).surface)).unwrap();
        let side = face_of(&|s| matches!(s, Surface::Cone(_)));
        let top = b.face_ids().find(|&f| matches!(b.face(f).surface, Surface::Plane(_)) && measure::face_bbox(&b, f).min.z > 3.9).unwrap();
        let bottom = b.face_ids().find(|&f| matches!(b.face(f).surface, Surface::Plane(_)) && measure::face_bbox(&b, f).max.z < 0.1).unwrap();
        let mut report = format!("tol::LINEAR {:e}
", tol::LINEAR);
        let try_at = |s: f64| {
            let t = 2.7 * s;
            let by: BTreeMap<FaceId, f64> = [(side, -t), (bottom, -t), (top, t)].into_iter().collect();
            crate::tweak::offset_faces(&b, &by).map(|m| m.brep)
        };
        let (mut lo, mut hi) = (0.5, 0.6);
        while hi - lo > 1e-15 {
            let mid = 0.5 * (lo + hi);
            if try_at(mid).is_ok() { lo = mid } else { hi = mid }
        }
        let radius = |s: f64| (4.0 - 7.2 * s) * 0.75;
        report.push_str(&format!("largest s that holds {lo:.15} top radius {:e}
", radius(lo)));
        for s in [0.5, 0.55, 0.5555, 0.55555, 0.555555, 1.0 / 1.8] {
            let line = match try_at(s) {
                Ok(m) => format!("ok faces {} edges {} verts {}", m.face_ids().count(), m.edge_ids().count(), m.vertex_ids().count()),
                Err(e) => format!("err {e}"),
            };
            report.push_str(&format!("s={s} radius {:e}: {line}
", radius(s)));
        }
        panic!("{report}");
    }
}
