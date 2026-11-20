// yack: A solid from a closed triangle mesh: each triangle a plane face, so the kernel's booleans,
// Adapted from springrep@a03e8a1 src/mesh/solid.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! A solid from a closed triangle mesh: each triangle a plane face, so the kernel's booleans,
//! distances and measures take a mesh another tool wrote as they take any solid.

use std::collections::HashMap;

use crate::{
    check,
    error::{Error, Result},
    geom::{Curve, Line, Plane, Surface},
    math::{Frame, Point3, Vec2},
    tol,
    topo::{Brep, CoedgeSpec, EdgeId, PCurve, VertexId},
};

/// The solid that the triangles bound, their corners the `points` they index: triangles sharing
/// two corners share the edge between them. They run counter-clockwise seen from outside. A
/// triangle with no area is left out. An error where the triangles do not close a solid.
pub(crate) fn from_indexed(points: &[Point3], triangles: &[[usize; 3]]) -> Result<Brep> {
    let mut b = Brep::new();
    let mut faces = Vec::new();
    let mut vertices: HashMap<usize, VertexId> = HashMap::new();
    let mut lines: HashMap<(VertexId, VertexId), EdgeId> = HashMap::new();
    for tri in triangles {
        if tri.iter().any(|&i| i >= points.len()) {
            return Err(Error::InvalidInput(
                "a triangle names a point the mesh does not have".into(),
            ));
        }
        let p = tri.map(|i| points[i]);
        let normal = (p[1] - p[0]).cross(p[2] - p[0]);
        let Ok(frame) = Frame::new(p[0], normal, p[1] - p[0]) else {
            continue;
        };
        let vs = tri.map(|i| *vertices.entry(i).or_insert_with(|| b.add_vertex(points[i])));
        let face = b.add_face(Surface::Plane(Plane { frame }), false, std::iter::empty());
        let mut coedges = Vec::with_capacity(3);
        for k in 0..3 {
            let (from, to) = (vs[k], vs[(k + 1) % 3]);
            let key = if from <= to { (from, to) } else { (to, from) };
            let edge = match lines.get(&key) {
                Some(&e) => e,
                None => {
                    let (a, z) = (b.vertex(key.0).point, b.vertex(key.1).point);
                    let Some((line, len)) = Line::through(a, z) else {
                        return Err(Error::InvalidInput(
                            "a triangle has two corners at one point".into(),
                        ));
                    };
                    let e = b.add_edge(Curve::Line(line), (0.0, len), key.0, key.1);
                    lines.insert(key, e);
                    e
                }
            };
            let l = frame.to_local(b.vertex(from).point);
            coedges.push(CoedgeSpec {
                edge,
                reversed: from != key.0,
                pcurve: PCurve::Projected {
                    seed: Vec2::new(l.x, l.y),
                },
            });
        }
        b.add_loop(face, coedges);
        faces.push(face);
    }
    if faces.is_empty() {
        return Err(Error::InvalidInput("a mesh with no triangles".into()));
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

/// The solid a closed triangle mesh bounds: each triangle a plane face. `triangles` index
/// `positions`, counter-clockwise seen from outside. Corners at one place are one vertex whether
/// or not they share an index, as in a mesh that lists every triangle's corners on their own.
pub fn solid(positions: &[Point3], triangles: &[[u32; 3]]) -> Result<Brep> {
    // Corners within the tolerance of each other fall in one cell or in neighbouring ones.
    let cell = |p: Point3| {
        let at = |x: f64| (x / tol::LINEAR).round() as i64;
        [at(p.x), at(p.y), at(p.z)]
    };
    let mut cells: HashMap<[i64; 3], usize> = HashMap::new();
    let mut welded: Vec<usize> = Vec::with_capacity(positions.len());
    let mut points: Vec<Point3> = Vec::new();
    for &p in positions {
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return Err(Error::InvalidInput("a mesh point is not finite".into()));
        }
        let c = cell(p);
        let mut found = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let near = cells.get(&[c[0] + dx, c[1] + dy, c[2] + dz]);
                    if let Some(&i) = near.filter(|&&i| points[i].dist(p) <= tol::LINEAR) {
                        found = Some(i);
                    }
                }
            }
        }
        let i = found.unwrap_or_else(|| {
            points.push(p);
            cells.insert(c, points.len() - 1);
            points.len() - 1
        });
        welded.push(i);
    }
    let at = |i: u32| welded.get(i as usize).copied().unwrap_or(usize::MAX);
    let triangles: Vec<[usize; 3]> = triangles.iter().map(|t| t.map(at)).collect();
    from_indexed(&points, &triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{boolean::intersection, math::Vec3, measure, primitive};

    /// The tetrahedron on the origin and the three unit points of the axes scaled by `s`, every
    /// triangle listing its own three corners, as an STL does.
    fn tetrahedron(s: f64) -> (Vec<Point3>, Vec<[u32; 3]>) {
        let (o, x, y, z) = (
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(s, 0.0, 0.0),
            Point3::new(0.0, s, 0.0),
            Point3::new(0.0, 0.0, s),
        );
        let faces = [[o, y, x], [o, x, z], [o, z, y], [x, y, z]];
        let positions: Vec<Point3> = faces.iter().flatten().copied().collect();
        let triangles = (0..4).map(|k| [3 * k, 3 * k + 1, 3 * k + 2]).collect();
        (positions, triangles)
    }

    #[test]
    fn a_closed_mesh_is_the_solid_its_triangles_bound() {
        // Twelve corners at four places: four faces, six edges, four vertices, a sixth of s³.
        let (positions, triangles) = tetrahedron(6.0);
        let b = solid(&positions, &triangles).unwrap();
        assert_eq!(
            (b.face_count(), b.edge_ids().count(), b.vertex_ids().count()),
            (4, 6, 4)
        );
        assert!((measure::volume(&b) - 36.0).abs() < 1e-9);
        // It is a solid like any other: half of a 2 × 2 × 2 block at the origin lies inside it
        // up to the plane x + y + z = 6, which the block does not reach.
        let block = primitive::block(&Frame::WORLD, Vec3::new(2.0, 2.0, 2.0), "b").unwrap();
        assert!((measure::volume(&intersection(&b, &block).unwrap()) - 8.0).abs() < 1e-9);
    }

    #[test]
    fn a_mesh_with_a_triangle_missing_or_turned_is_no_solid() {
        let (positions, mut triangles) = tetrahedron(6.0);
        assert!(solid(&positions, &triangles[..3]).is_err());
        triangles[3].swap(0, 1);
        assert!(solid(&positions, &triangles).is_err());
        assert!(solid(&positions, &[[0, 1, 99]]).is_err());
    }
}
