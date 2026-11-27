// yack: a straight convex edge between planes rounded by circular sections square to it, the radius running evenly along it
//! A variable round whose every section square to the edge is a circle tangent to both faces, its
//! radius `r(t)` growing evenly with the distance along the edge. The sections are circles of one
//! family, so the round is an oblique circular cone, ruled between the circles at the edge's ends.
//! What it takes off is the solid ruled between the two end sections of the corner left of the
//! circle: the two lines along the faces and the arc between their tangent points.

use std::f64::consts::{PI, TAU};

use crate::{
    Error, Result, blend::shaped::plane_of, boolean,
    geom::Curve,
    math::{Frame, Point3, Vec2, Vec3},
    named,
    profile::{Profile, ProfileLoop, Segment},
    tol,
    topo::{Brep, EdgeId, FaceId, FaceName},
};

/// The corner of the section at one end of the edge: the profile of what the round takes off, for
/// a circle of radius `r` tangent to both faces (feet `r / tan(half)` from the edge along each,
/// centre `r / sin(half)` out along the bisector), drawn in `frame`.
fn section(frame: &Frame, ua: Vec3, ub: Vec3, bis: Vec3, half: f64, r: f64) -> Profile {
    let local = |v: Vec3| Vec2::new(v.dot(frame.x), v.dot(frame.y));
    let d = r / half.tan();
    let (a, b) = (local(ua * d), local(ub * d));
    let k = local(bis * (r / half.sin()));
    let angle = |p: Vec2| (p.y - k.y).atan2(p.x - k.x);
    let start = angle(a);
    let mut sweep = angle(b) - start;
    while sweep > PI {
        sweep -= TAU;
    }
    while sweep <= -PI {
        sweep += TAU;
    }
    let corner = Vec2::new(0.0, 0.0);
    Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                named(Segment::Line { a: corner, b: a }, "s0"),
                named(Segment::Arc { center: k, radius: r, start, sweep }, "round"),
                named(Segment::Line { a: b, b: corner }, "s2"),
            ],
        }],
    }
}

/// The faces of `s` with a coedge starting at `v`.
fn faces_at(s: &Brep, v: crate::topo::VertexId) -> Vec<FaceId> {
    let mut faces: Vec<FaceId> = s.coedge_ids().filter(|&c| s.coedge_start(c) == v).map(|c| s.coedge_face(c)).collect();
    faces.sort();
    faces.dedup();
    faces
}

/// `s` with its straight convex edge `e` between two planes rounded by circles square to it, of
/// radius `r_from` at the end nearer `from` and `r_to` at the other, growing evenly between. The
/// round's faces are named `<feature>.round`. Declines where the edge is not straight, convex and
/// between planes, its ends are not square to it (each the third face of a corner), or a circle's
/// feet leave a face.
pub fn fillet_variable(s: &Brep, e: EdgeId, from: Point3, r_from: f64, r_to: f64, feature: &str) -> Result<Brep> {
    if !(r_from > tol::LINEAR && r_to > tol::LINEAR) {
        return Err(Error::InvalidInput("a fillet's radii must be positive".into()));
    }
    let edge = s.edge(e);
    let Some(Curve::Line(_)) = &edge.curve else {
        return Err(Error::Unsupported("a variable round on an edge that is not straight".into()));
    };
    if crate::topo::local::convexity(s, e) != Some(true) {
        return Err(Error::Unsupported("a variable round on an edge that is not convex".into()));
    }
    let [cf, cg] = s.edge_uses(e) else {
        return Err(Error::InvalidInput("a rounded edge lies between two faces".into()));
    };
    let (f, g) = (s.coedge_face(*cf), s.coedge_face(*cg));
    let ((_, na), (_, nb)) = (plane_of(s, f)?, plane_of(s, g)?);
    let (v0, v1) = if s.vertex(edge.end).point.dist(from) < s.vertex(edge.start).point.dist(from) { (edge.end, edge.start) } else { (edge.start, edge.end) };
    let (p0, p1) = (s.vertex(v0).point, s.vertex(v1).point);
    let len = p0.dist(p1);
    let dir = (p1 - p0) * (1.0 / len);
    // Along each face away from the edge, square to it, toward the other face's inside.
    let along = |n: Vec3, other: Vec3| {
        let d = n.cross(dir).unit();
        d.map(|d| if d.dot(other) < 0.0 { d } else { -d })
    };
    let (Some(ua), Some(ub)) = (along(na, nb), along(nb, na)) else {
        return Err(Error::InvalidInput("a flat edge".into()));
    };
    let alpha = ua.dot(ub).clamp(-1.0, 1.0).acos();
    let half = alpha / 2.0;
    let bis = (ua + ub).unit().ok_or_else(|| Error::InvalidInput("a flat edge".into()))?;
    // Each end is a third face square to the edge; each circle's feet land on the two faces.
    for (v, p, r) in [(v0, p0, r_from), (v1, p1, r_to)] {
        let at = faces_at(s, v);
        let end = at.iter().copied().find(|&h| h != f && h != g);
        let square = end.is_some_and(|h| at.len() == 3 && plane_of(s, h).is_ok_and(|(o, n)| (n.cross(dir)).norm() <= 1e-9 && (p - o).dot(n).abs() <= tol::LINEAR)); // tol: a cosine, and a distance
        if !square {
            return Err(Error::Unsupported("a variable round whose end is not a face square to the edge".into()));
        }
        let inward = if p == p0 { dir } else { -dir } * (1e-3 * len); // tol: a step in from the end face
        for (u, face) in [(ua, f), (ub, g)] {
            let q = p + inward + u * (r / half.tan());
            let holds = crate::measure::face_at(s, q).is_some_and(|(h, d)| h == face && d <= 10.0 * tol::LINEAR);
            if !holds {
                return Err(Error::InvalidInput("a variable fillet whose circle does not fit on its faces".into()));
            }
        }
    }
    let x_hint = ua;
    let (fr0, fr1) = (Frame::new(p0, dir, x_hint)?, Frame::new(p1, dir, x_hint)?);
    let tool = yack_build::ruled::ruled(&fr0, &section(&fr0, ua, ub, bis, half, r_from), &fr1, &section(&fr1, ua, ub, bis, half, r_to), "\u{1}round")?;
    let mut out = boolean::difference(s, &tool)?;
    out.rename_faces(|n| match n.feature.strip_prefix('\u{1}') {
        Some(_) => vec![FaceName::new(feature, "round")],
        None => vec![n.clone()],
    });
    Ok(out)
}
