// yack: a convex polyhedron grown by a ball: its faces, edges and vertices each swept out by the ball
//! The points within `r` of a convex polyhedron of plane faces are the solid itself, a slab over
//! each face (the face's polygon `r` out along its normal), a circular wedge along each edge (the
//! sector of radius `r` between the two faces' normals, swept along the edge) and a ball at each
//! vertex. The pieces overlap only inside one another's tangent contacts, so the result is their union.

use std::f64::consts::PI;

use crate::{
    Error, Result, boolean,
    geom::Surface,
    math::{Frame, Point3, Vec2, Vec3},
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    sweep::extrude,
    tol,
    topo::{Brep, FaceId, FaceName},
};

/// `s` grown by a ball of radius `r`. Faces keep their names (the slab's outer face); the wedge along
/// the k-th edge is `<feature>.round_e<k>` and the ball at the k-th vertex `<feature>.corner_v<k>`.
/// Declines a solid with a face that is not a plane, with a hole in a face, or an edge that is not
/// convex and straight.
pub fn grow_convex(s: &Brep, r: f64, feature: &str) -> Result<Brep> {
    if r.is_nan() || r <= tol::LINEAR {
        return Err(Error::InvalidInput("a growth must be positive".into()));
    }
    if s.edge_ids().any(|e| crate::topo::local::convexity(s, e) != Some(true)) {
        return Err(Error::Unsupported("a convex growth of a solid with an edge that is not convex".into()));
    }
    let mut out = s.clone();
    // A slab over each face.
    for f in s.face_ids() {
        let slab = slab_over(s, f, r)?;
        out = boolean::union(&out, &slab)?;
    }
    // A wedge along each edge.
    for (k, e) in s.edge_ids().enumerate() {
        let x = s.edge(e);
        let Some(crate::geom::Curve::Line(_)) = &x.curve else {
            return Err(Error::Unsupported("a convex growth along an edge that is not straight".into()));
        };
        let [cf, cg] = s.edge_uses(e) else {
            return Err(Error::InvalidInput("an edge lies between two faces".into()));
        };
        let normal = |f: FaceId| plane_normal(s, f);
        let (n1, n2) = (normal(s.coedge_face(*cf))?, normal(s.coedge_face(*cg))?);
        let (p, q) = (s.vertex(x.start).point, s.vertex(x.end).point);
        let Some(dir) = (q - p).unit() else {
            return Err(Error::InvalidInput("an edge of no length".into()));
        };
        let frame = Frame::new(p, dir, n1)?;
        let b = Vec2::new(n2.dot(frame.x), n2.dot(frame.y)) * r;
        let sweep = b.y.atan2(b.x);
        if sweep.abs() <= 1e-9 || sweep.abs() >= PI - 1e-9 { // tol: an angle: faces that are not a convex corner
            return Err(Error::Unsupported("a convex growth along an edge between parallel faces".into()));
        }
        let named = |segment: Segment, name: &str| NamedSegment::new(segment, name);
        let sector = Profile {
            loops: vec![ProfileLoop {
                segments: vec![
                    named(Segment::Line { a: Vec2::ZERO, b: Vec2::new(r, 0.0) }, "a"),
                    named(Segment::Arc { center: Vec2::ZERO, radius: r, start: 0.0, sweep }, "round"),
                    named(Segment::Line { a: b, b: Vec2::ZERO }, "b"),
                ],
            }],
        };
        let mut wedge = extrude(&sector, &frame, (q - p).norm(), "wedge")?;
        wedge.rename_faces(|n| if n.face == "round" { vec![FaceName::new(feature, format!("round_e{k}"))] } else { Vec::new() });
        out = boolean::union(&out, &wedge)?;
    }
    // A ball at each vertex.
    for (k, v) in s.vertex_ids().enumerate() {
        let centre = s.vertex(v).point;
        let mut ball = yack_build::primitive::sphere(&Frame::from_axis(centre, Vec3::Z)?, r, "ball")?;
        ball.rename_faces(|_| vec![FaceName::new(feature, format!("corner_v{k}"))]);
        out = boolean::union(&out, &ball)?;
    }
    Ok(out)
}

/// The outward unit normal of the plane face `f`.
fn plane_normal(s: &Brep, f: FaceId) -> Result<Vec3> {
    match &s.face(f).surface {
        Surface::Plane(p) => Ok(if s.face(f).reversed { -p.frame.z } else { p.frame.z }),
        _ => Err(Error::Unsupported("a convex growth of a face that is not a plane".into())),
    }
}

/// The slab `r` thick over the plane face `f`, its outer face carrying the face's names.
fn slab_over(s: &Brep, f: FaceId, r: f64) -> Result<Brep> {
    let n = plane_normal(s, f)?;
    let face = s.face(f);
    let [lp] = face.loops[..] else {
        return Err(Error::Unsupported("a convex growth of a face with a hole".into()));
    };
    let points: Vec<Point3> = s.lp(lp).coedges.iter().map(|&c| s.vertex(s.coedge_start(c)).point).collect();
    let Some(&origin) = points.first() else {
        return Err(Error::InvalidInput("a face with no boundary".into()));
    };
    let frame = Frame::from_axis(origin, n)?;
    let flat: Vec<Vec2> = points.iter().map(|&p| Vec2::new((p - origin).dot(frame.x), (p - origin).dot(frame.y))).collect();
    let names: Vec<String> = (0..flat.len()).map(|i| format!("s{i}")).collect();
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    let mut slab = extrude(&Profile::polygon(&flat, &names), &frame, r, "slab")?;
    let kept = face.names.clone();
    slab.rename_faces(|name| if name.face == FaceName::END { kept.iter().cloned().collect::<Vec<_>>() } else { Vec::new() });
    Ok(slab)
}

/// `s` with every edge rounded to radius `r`, where `s` is a convex polyhedron whose inward offset by
/// `r` keeps its topology: the points a ball of radius `r` rolling inside it reaches, which is the
/// polyhedron shrunk by `r` and grown by `r`. Declines a solid whose shrink loses a face or an edge
/// (the rolling ball would then touch fewer faces).
pub fn round_all_edges(s: &Brep, r: f64, feature: &str) -> Result<Brep> {
    let shrunk = crate::offset(s, -r, feature)?;
    if !crate::same_topology(s, &shrunk) {
        return Err(Error::Unsupported("rounding every edge of a polyhedron whose shrink loses a face or an edge".into()));
    }
    grow_convex(&shrunk, r, feature)
}
