// yack: a straight edge between two planes chamfered by distances along each face, ending on whatever faces end it
//! The chamfer's section square to the edge is a triangle: the corner and the two points the given
//! distances along the faces. Swept along the edge past both of its ends, it is a prism that a
//! convex solid holds only along the edge, so cutting the prism out (a convex edge) ends the chamfer
//! on the faces at the edge's ends, earlier chamfers' among them; filling it (a concave edge) adds
//! the triangle's prism the same way.

use crate::{
    Error, Result, blend::shaped::plane_of, boolean,
    geom::Curve,
    math::{Frame, Vec2, Vec3},
    measure,
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    sweep::extrude,
    tol,
    topo::{Brep, EdgeId, FaceId, FaceName},
};

/// `s` with the straight edge `e`, between the planes `face` and its other face, chamfered by
/// `along_face` along `face` and `along_other` along the other; named `<feature>.bevel_0`.
/// Declines an edge that is not straight between two planes, a convex edge of a solid with a concave
/// edge elsewhere (the prism could cut other material), and distances that leave a face.
pub fn chamfer_two(s: &Brep, e: EdgeId, face: FaceId, along_face: f64, along_other: f64, feature: &str) -> Result<Brep> {
    let (prism, convex) = bevel_prism(s, e, face, along_face, along_other)?;
    let mut done = if convex { boolean::difference(s, &prism)? } else { boolean::union(s, &prism)? };
    done.rename_faces(|n| match n.feature.strip_prefix('\u{1}') {
        Some(_) => vec![FaceName::new(feature, "bevel_0")],
        None => vec![n.clone()],
    });
    Ok(done)
}

/// The prism the chamfer of `e` cuts out (a convex edge) or adds (a concave one, the flag false):
/// the section triangle swept along the edge, its faces named `\u{1}bevel` with the chamfer's own
/// face `a`. The checks of [`chamfer_two`] are made here.
pub(crate) fn bevel_prism(s: &Brep, e: EdgeId, face: FaceId, along_face: f64, along_other: f64) -> Result<(Brep, bool)> {
    edge_prism(s, e, face, along_face, along_other, false)
}

/// [`bevel_prism`], or with `round` the prism of a round of radius `along_face` (`along_other` is
/// ignored): the section is then the corner, the two feet `r / tan(phi / 2)` along the faces, and
/// the arc of the circle of radius `r` tangent to both between them, `phi` the angle between the
/// faces' directions away from the edge; the round's face is named `a`, as the chamfer's is.
pub(crate) fn edge_prism(s: &Brep, e: EdgeId, face: FaceId, along_face: f64, along_other: f64, round: bool) -> Result<(Brep, bool)> {
    let edge = s.edge(e);
    let Some(Curve::Line(line)) = &edge.curve else {
        return Err(Error::Unsupported("a straight chamfer on an edge that is not straight".into()));
    };
    let [cf, cg] = s.edge_uses(e) else {
        return Err(Error::InvalidInput("a chamfered edge lies between two faces".into()));
    };
    let (f0, f1) = (s.coedge_face(*cf), s.coedge_face(*cg));
    let (first, second) = if f0 == face { (f0, f1) } else if f1 == face { (f1, f0) } else {
        return Err(Error::InvalidInput("the face is not one of the edge's two faces".into()));
    };
    let Some(convex) = crate::topo::local::convexity(s, e) else {
        return Err(Error::Unsupported("a straight chamfer on an edge that meets tangentially".into()));
    };
    if convex && s.edge_ids().any(|g| g != e && crate::topo::local::convexity(s, g) == Some(false)) {
        return Err(Error::Unsupported("a straight chamfer on a solid with a concave edge, where its prism could cut other material".into()));
    }
    let ((_, na), (_, nb)) = (plane_of(s, first)?, plane_of(s, second)?);
    let dir = line.dir;
    // Along each face away from the edge, square to it, toward the other face's inside (convex) or away (concave).
    let along = |n: Vec3, other: Vec3| -> Option<Vec3> {
        let d = n.cross(dir).unit()?;
        Some(if (d.dot(other) < 0.0) == convex { d } else { -d })
    };
    let (Some(ua), Some(ub)) = (along(na, nb), along(nb, na)) else {
        return Err(Error::InvalidInput("a flat edge".into()));
    };
    let phi = ua.dot(ub).clamp(-1.0, 1.0).acos();
    let radius = along_face;
    let (along_face, along_other) = if round {
        if phi <= 1e-6 || phi >= std::f64::consts::PI - 1e-6 { // tol: an angle: faces that are not a corner
            return Err(Error::InvalidInput("a round on a flat edge".into()));
        }
        let t = radius / (phi / 2.0).tan();
        (t, t)
    } else {
        (along_face, along_other)
    };
    // The edge's middle: its section square to it is where the triangle is drawn.
    let mid = Curve::Line(line.clone()).eval(0.5 * (edge.range.0 + edge.range.1));
    let (pa, pb) = (mid + ua * along_face, mid + ub * along_other);
    // Both contact points lie on their faces (checked a hair short of the far end, where a distance
    // that runs a face's whole length meets the next face).
    for (face, u, d) in [(first, ua, along_face), (second, ub, along_other)] {
        let q = mid + u * (d - 100.0 * tol::LINEAR).max(0.0); // tol: a hair, far above the tolerance
        let inside = measure::face_at(s, q).is_some_and(|(g, off)| g == face && off <= 10.0 * tol::LINEAR);
        if !inside {
            return Err(Error::InvalidInput("a chamfer whose contact leaves its face".into()));
        }
    }
    // The prism: the triangle in the plane square to the edge through `mid`. A convex edge's is swept
    // past both ends (the solid ends it); a concave edge's runs the edge's own length, each end of
    // which must lie on a plane face square to the edge.
    let (start, end) = (s.vertex(edge.start).point, s.vertex(edge.end).point);
    let (from, length) = if convex {
        let reach = 2.0 * measure::bbox(s).diagonal() + along_face + along_other;
        (mid - dir * reach, 2.0 * reach)
    } else {
        for v in [edge.start, edge.end] {
            let square = s.coedge_ids().filter(|&c| s.coedge_start(c) == v).map(|c| s.coedge_face(c)).any(|h| {
                h != first && h != second && plane_of(s, h).is_ok_and(|(_, n)| n.cross(dir).norm() <= 1e-9) // tol: a cosine
            });
            if !square {
                return Err(Error::Unsupported("a concave straight chamfer whose end is not a face square to the edge".into()));
            }
        }
        let (lo, hi) = (start.dot(dir).min(end.dot(dir)), start.dot(dir).max(end.dot(dir)));
        (mid + dir * (lo - mid.dot(dir)), hi - lo)
    };
    let x = ua;
    let frame = Frame::new(from, dir, x)?;
    let local = |p: Vec3| Vec2::new((p - from).dot(frame.x), (p - from).dot(frame.y));
    let triangle = [local(mid), local(pa), local(pb)];
    let profile = if round {
        // The ball's centre, out along the bisector of the faces, and the arc between the feet.
        let Some(bis) = (ua + ub).unit() else {
            return Err(Error::InvalidInput("a round on a flat edge".into()));
        };
        let k = local(mid + bis * (radius / (phi / 2.0).sin()));
        let angle = |p: Vec2| (p.y - k.y).atan2(p.x - k.x);
        let start = angle(triangle[1]);
        let mut sweep = angle(triangle[2]) - start;
        while sweep > std::f64::consts::PI {
            sweep -= std::f64::consts::TAU;
        }
        while sweep <= -std::f64::consts::PI {
            sweep += std::f64::consts::TAU;
        }
        let named = |segment: Segment, name: &str| NamedSegment::new(segment, name);
        Profile {
            loops: vec![ProfileLoop {
                segments: vec![
                    named(Segment::Line { a: triangle[0], b: triangle[1] }, "corner"),
                    named(Segment::Arc { center: k, radius, start, sweep }, "a"),
                    named(Segment::Line { a: triangle[2], b: triangle[0] }, "b"),
                ],
            }],
        }
    } else {
        Profile::polygon(&triangle, &["corner", "a", "b"])
    };
    let prism = extrude(&profile, &frame, length, "\u{1}bevel")?;
    Ok((prism, convex))
}
