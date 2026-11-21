// yack: Faces and edges of a held solid picked by a point and described, for yack's FaceRef/EdgeRef.
//! Picking a held body's faces and edges by a point, and what they are: the facts yack's
//! `FaceRef` and `EdgeRef` carry. A pick never guesses: no candidate is [`Picked::None`], more
//! than one is [`Picked::Ambiguous`]. Nothing here changes the kernel.

use crate::native::curve::Curve;
use crate::native::surface::Surface;
use crate::native::tol;
use crate::native::topo::Body;
use crate::{BodyId, DefaultKernel, DistanceTarget, EdgeId, FaceId, Kernel, KernelError, Result};
use nalgebra::Point3;

/// The outcome of a pick by a point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Picked {
    /// Exactly one face or edge lies within the tolerance: its id.
    One(u64),
    /// None does.
    None,
    /// This many do: yack never chooses among them.
    Ambiguous(usize),
}

/// A face's surface, with what defines it (directions unit length; a plane's normal outward).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaceKind {
    /// A plane, its outward normal.
    Plane {
        /// The outward unit normal.
        normal: [f64; 3],
    },
    /// A cylinder: its axis (a point on it, its direction) and radius.
    Cylinder {
        /// A point on the axis and its unit direction.
        axis: ([f64; 3], [f64; 3]),
        /// The radius.
        radius: f64,
    },
    /// A cone: its apex, axis direction and half-angle (radians).
    Cone {
        /// The apex.
        apex: [f64; 3],
        /// The axis's unit direction.
        axis: [f64; 3],
        /// The half-angle (radians).
        half_angle: f64,
    },
    /// A sphere: its centre and radius.
    Sphere {
        /// The centre.
        center: [f64; 3],
        /// The radius.
        radius: f64,
    },
    /// A torus: its axis (centre, direction) and radii.
    Torus {
        /// The centre and the axis's unit direction.
        axis: ([f64; 3], [f64; 3]),
        /// The radius of the tube's centre circle.
        major: f64,
        /// The tube's radius.
        minor: f64,
    },
    /// An extrusion or a revolution of a curve.
    Swept,
    /// A B-spline surface, or anything else.
    Spline,
}

/// What a face is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceFacts {
    /// Its surface.
    pub kind: FaceKind,
    /// Its area (mm²).
    pub area: f64,
    /// Its centroid.
    pub centroid: [f64; 3],
    /// A box sure to hold it (least and greatest corners).
    pub bbox: ([f64; 3], [f64; 3]),
}

/// An edge's curve, with what defines it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeKind {
    /// A line, its unit direction from start to end.
    Line {
        /// The unit direction from start to end.
        direction: [f64; 3],
    },
    /// A circle or an arc of one: its radius and axis (centre, unit normal).
    Circle {
        /// The radius.
        radius: f64,
        /// The centre and the unit normal of the circle's plane.
        axis: ([f64; 3], [f64; 3]),
    },
    /// An ellipse or an arc of one.
    Ellipse,
    /// A B-spline curve.
    Spline,
}

/// What an edge is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeFacts {
    /// Its curve.
    pub kind: EdgeKind,
    /// Its length (mm).
    pub length: f64,
    /// Its start and end.
    pub ends: ([f64; 3], [f64; 3]),
    /// Its middle (at the middle of its parameter range).
    pub mid: [f64; 3],
    /// The faces on its sides (`None` past a free edge's one).
    pub faces: [Option<u64>; 2],
    /// How many face sides it bounds: 2 on a closed manifold edge (a seam's one face twice).
    pub uses: usize,
    /// Whether the material's corner at its middle is under 180°; `None` when its faces join
    /// smoothly there or it has no second face.
    pub convex: Option<bool>,
}

fn arr(p: &Point3<f64>) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn v_arr(v: &nalgebra::Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn bad(what: &str) -> KernelError {
    KernelError::InvalidParameter(format!("no such {what} on this body"))
}

fn face_index(b: &Body, face: u64) -> Result<usize> {
    usize::try_from(face).ok().filter(|&i| i < b.faces.len()).ok_or_else(|| bad("face"))
}

fn edge_index(b: &Body, edge: u64) -> Result<usize> {
    usize::try_from(edge).ok().filter(|&i| i < b.edges.len()).ok_or_else(|| bad("edge"))
}

/// The distance from `p` to a box (0 inside it).
fn box_distance(min: &Point3<f64>, max: &Point3<f64>, p: &Point3<f64>) -> f64 {
    let d = |lo: f64, hi: f64, x: f64| (lo - x).max(x - hi).max(0.0);
    let (dx, dy, dz) = (d(min.x, max.x, p.x), d(min.y, max.y, p.y), d(min.z, max.z, p.z));
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn picked(hits: &[u64]) -> Picked {
    match hits {
        [] => Picked::None,
        [one] => Picked::One(*one),
        many => Picked::Ambiguous(many.len()),
    }
}

/// The face of `body` within `tol` of `p`, if exactly one is.
pub fn face_pick(k: &DefaultKernel, body: BodyId, p: [f64; 3], tol: f64) -> Result<Picked> {
    let b = k.get(body)?;
    let q = Point3::new(p[0], p[1], p[2]);
    let mut hits = Vec::new();
    for fi in 0..b.faces.len() {
        let bb = b.face_bbox(fi);
        if box_distance(&bb.min, &bb.max, &q) > tol {
            continue;
        }
        let d = k.min_distance(&DistanceTarget::Point(q), &DistanceTarget::Face(body, FaceId(fi as u64)))?;
        if d.distance <= tol {
            hits.push(fi as u64);
        }
    }
    Ok(picked(&hits))
}

/// The edge of `body` within `tol` of `p`, if exactly one is; a seam is no edge to pick, so it
/// counts only when no other edge is within `tol`.
pub fn edge_pick(k: &DefaultKernel, body: BodyId, p: [f64; 3], tol: f64) -> Result<Picked> {
    let q = Point3::new(p[0], p[1], p[2]);
    let (mut hits, mut seams) = (Vec::new(), Vec::new());
    for e in k.edges(body)? {
        let d = k.min_distance(&DistanceTarget::Point(q), &DistanceTarget::Edge(body, e.id))?;
        if d.distance <= tol {
            if e.is_seam() { seams.push(e.id.0) } else { hits.push(e.id.0) }
        }
    }
    Ok(picked(if hits.is_empty() { &seams } else { &hits }))
}

/// What face `face` of `body` is.
pub fn face_info(k: &DefaultKernel, body: BodyId, face: u64) -> Result<FaceFacts> {
    let b = k.get(body)?;
    let fi = face_index(b, face)?;
    let info = k.faces(body)?.into_iter().find(|f| f.id == FaceId(face)).ok_or_else(|| bad("face"))?;
    let f = &b.faces[fi];
    let out = if f.sense { 1.0 } else { -1.0 };
    let kind = match &f.surf {
        Surface::Plane { f: fr } => FaceKind::Plane { normal: v_arr(&(fr.x.cross(&fr.y).normalize() * out)) },
        Surface::Cylinder { f: fr, r } => FaceKind::Cylinder { axis: (arr(&fr.o), v_arr(&fr.z.normalize())), radius: *r },
        Surface::Cone { f: fr, r, a } if a.sin().abs() > tol::ANGULAR => {
            let z = fr.z.normalize();
            FaceKind::Cone { apex: arr(&(fr.o - z * (r * a.cos() / a.sin()))), axis: v_arr(&z), half_angle: a.abs() }
        }
        Surface::Sphere { f: fr, r } => FaceKind::Sphere { center: arr(&fr.o), radius: *r },
        Surface::Torus { f: fr, rmaj, rmin } => FaceKind::Torus { axis: (arr(&fr.o), v_arr(&fr.z.normalize())), major: *rmaj, minor: *rmin },
        Surface::Revolution { .. } | Surface::Extrusion { .. } => FaceKind::Swept,
        _ => FaceKind::Spline,
    };
    let bb = b.face_bbox(fi);
    Ok(FaceFacts { kind, area: info.area, centroid: arr(&info.center), bbox: (arr(&bb.min), arr(&bb.max)) })
}

/// The point of face `face` nearest `p`.
pub fn face_closest(k: &DefaultKernel, body: BodyId, face: u64, p: [f64; 3]) -> Result<[f64; 3]> {
    face_index(k.get(body)?, face)?;
    let q = Point3::new(p[0], p[1], p[2]);
    let d = k.min_distance(&DistanceTarget::Point(q), &DistanceTarget::Face(body, FaceId(face)))?;
    Ok(arr(&d.b))
}

/// The outward unit normal of face `face` at its point nearest `p`; `None` where the surface
/// has no normal (a cone's apex, a sphere's pole).
pub fn face_normal_at(k: &DefaultKernel, body: BodyId, face: u64, p: [f64; 3]) -> Result<Option<[f64; 3]>> {
    let b = k.get(body)?;
    let f = &b.faces[face_index(b, face)?];
    let on = face_closest(k, body, face, p)?;
    let (u, v) = f.surf.project(&Point3::new(on[0], on[1], on[2]), None);
    let (_, su, sv) = f.surf.d1(u, v);
    let n = su.cross(&sv);
    let l = n.norm();
    if l <= tol::DEGENERATE * (su.norm() * sv.norm()).max(tol::TINY) {
        return Ok(None);
    }
    let out = if f.sense { 1.0 } else { -1.0 };
    Ok(Some(v_arr(&(n * (out / l)))))
}

/// An even `nu`×`nv` grid over face `face`'s (u, v) box, rows along u (`grid[i]` at the i-th u):
/// the box spans the face's outline, so on a trimmed face some points lie off the face. One
/// sample along a direction sits at the box's low end (as springrep's `face_grid`).
pub fn face_grid(k: &DefaultKernel, body: BodyId, face: u64, nu: usize, nv: usize) -> Result<Vec<Vec<[f64; 3]>>> {
    let b = k.get(body)?;
    let f = &b.faces[face_index(b, face)?];
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for q in crate::native::classify::face_polygons(b, f).iter().flatten() {
        lo = [lo[0].min(q.x), lo[1].min(q.y)];
        hi = [hi[0].max(q.x), hi[1].max(q.y)];
    }
    if !(lo[0] <= hi[0] && lo[1] <= hi[1]) {
        return Err(KernelError::OperationFailed("the face has no outline to sample".into()));
    }
    let at = |i: usize, n: usize, a: f64, z: f64| if n <= 1 { a } else { a + (z - a) * i as f64 / (n - 1) as f64 };
    Ok((0..nu).map(|i| (0..nv).map(|j| arr(&f.surf.eval(at(i, nu, lo[0], hi[0]), at(j, nv, lo[1], hi[1])))).collect()).collect())
}

/// What edge `edge` of `body` is.
pub fn edge_info(k: &DefaultKernel, body: BodyId, edge: u64) -> Result<EdgeFacts> {
    let b = k.get(body)?;
    let ei = edge_index(b, edge)?;
    let info = k.edges(body)?.into_iter().find(|e| e.id == EdgeId(edge)).ok_or_else(|| bad("edge"))?;
    let e = &b.edges[ei];
    let kind = match &e.curve {
        Curve::Line { d, .. } => EdgeKind::Line { direction: v_arr(&d.normalize()) },
        Curve::Circle { c, x, y, r } => EdgeKind::Circle { radius: *r, axis: (arr(c), v_arr(&x.cross(y).normalize())) },
        Curve::Ellipse { .. } => EdgeKind::Ellipse,
        Curve::Nurbs(_) => EdgeKind::Spline,
    };
    let uses = b.faces.iter().flat_map(|f| f.loops.iter().flatten()).filter(|c| c.edge == ei).count();
    let convex = if info.smooth { None } else { crate::native::blend::edge_convex(b, ei) };
    Ok(EdgeFacts {
        kind,
        length: info.length,
        ends: (arr(&info.start), arr(&info.end)),
        mid: arr(&info.mid),
        faces: info.faces.map(|f| f.map(|f| f.0)),
        uses,
        convex,
    })
}

/// Edge `edge`'s parameter range, in its curve's own parameter.
pub fn edge_range(k: &DefaultKernel, body: BodyId, edge: u64) -> Result<[f64; 2]> {
    let b = k.get(body)?;
    Ok(b.edges[edge_index(b, edge)?].t)
}

/// Edge `edge`'s point at parameter `t` (in [`edge_range`]).
pub fn edge_point(k: &DefaultKernel, body: BodyId, edge: u64, t: f64) -> Result<[f64; 3]> {
    let b = k.get(body)?;
    let e = &b.edges[edge_index(b, edge)?];
    let [t0, t1] = e.t;
    let slack = tol::REL * (t1 - t0).abs().max(1.0);
    if !(t >= t0.min(t1) - slack && t <= t0.max(t1) + slack) {
        return Err(KernelError::InvalidParameter(format!("t = {t} is off the edge's range [{t0}, {t1}]")));
    }
    Ok(arr(&e.point(t)))
}

/// `n` points of edge `edge`, even in its parameter, its ends included (one: its start).
pub fn edge_points(k: &DefaultKernel, body: BodyId, edge: u64, n: usize) -> Result<Vec<[f64; 3]>> {
    let b = k.get(body)?;
    let e = &b.edges[edge_index(b, edge)?];
    let [t0, t1] = e.t;
    Ok((0..n).map(|i| arr(&e.point(if n <= 1 { t0 } else { t0 + (t1 - t0) * i as f64 / (n - 1) as f64 }))).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::facade::profile::Place;
    use crate::yack_glue::{boolean, primitive, Prim};

    const WORLD: Place = Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };

    fn near(a: [f64; 3], b: [f64; 3]) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9)
    }

    /// A block's faces and edges are picked by a point on them, never guessed between, and
    /// described as they are; an inner corner's edge is concave.
    #[test]
    fn a_blocks_faces_and_edges_picked_and_described() {
        let mut k = DefaultKernel::new();
        let (b, _) = primitive(&mut k, &WORLD, Prim::Block { size: [4.0, 2.0, 3.0] }).unwrap();
        let Picked::One(top) = face_pick(&k, b, [2.0, 1.0, 3.0], 1e-6).unwrap() else { panic!() };
        let f = face_info(&k, b, top).unwrap();
        assert!(matches!(f.kind, FaceKind::Plane { normal } if near(normal, [0.0, 0.0, 1.0])), "{f:?}");
        assert!((f.area - 8.0).abs() < 1e-9 && near(f.centroid, [2.0, 1.0, 3.0]));
        assert_eq!(face_pick(&k, b, [4.0, 1.0, 3.0], 1e-6).unwrap(), Picked::Ambiguous(2));
        assert_eq!(face_pick(&k, b, [2.0, 1.0, 10.0], 1e-6).unwrap(), Picked::None);
        assert!(near(face_closest(&k, b, top, [2.0, 1.0, 10.0]).unwrap(), [2.0, 1.0, 3.0]));
        assert!(near(face_normal_at(&k, b, top, [1.0, 1.0, 5.0]).unwrap().unwrap(), [0.0, 0.0, 1.0]));
        let grid = face_grid(&k, b, top, 3, 2).unwrap();
        assert!(grid.len() == 3 && grid.iter().flatten().count() == 6 && grid.iter().flatten().all(|p| (p[2] - 3.0).abs() < 1e-12), "{grid:?}");
        // The L's notched side: its box spans the notch, whose corner point is off the face.
        let side = |k: &DefaultKernel, l| match face_pick(k, l, [1.0, 0.0, 1.0], 1e-6).unwrap() { Picked::One(f) => f, p => panic!("{p:?}") };
        let Picked::One(e) = edge_pick(&k, b, [2.0, 0.0, 3.0], 1e-6).unwrap() else { panic!() };
        let i = edge_info(&k, b, e).unwrap();
        assert!(matches!(i.kind, EdgeKind::Line { direction } if direction[0].abs() > 1.0 - 1e-12), "{i:?}");
        assert!((i.length - 4.0).abs() < 1e-9 && i.uses == 2 && i.convex == Some(true), "{i:?}");
        let [t0, t1] = edge_range(&k, b, e).unwrap();
        assert!(near(edge_point(&k, b, e, (t0 + t1) / 2.0).unwrap(), [2.0, 0.0, 3.0]));
        assert!(edge_point(&k, b, e, t1 + 1.0).is_err());
        let pts = edge_points(&k, b, e, 3).unwrap();
        assert!(pts.len() == 3 && near(pts[1], [2.0, 0.0, 3.0]));
        assert!(face_info(&k, b, 99).is_err() && edge_info(&k, b, 99).is_err());
        // An L: the block with its upper right quarter taken off.
        let cut = Place { origin: [2.0, -1.0, 1.5], ..WORLD };
        let (c, _) = primitive(&mut k, &cut, Prim::Block { size: [3.0, 4.0, 3.0] }).unwrap();
        let (l, _) = boolean(&mut k, crate::BoolOp::Subtract, b, &[c]).unwrap();
        let Picked::One(inner) = edge_pick(&k, l, [2.0, 1.0, 1.5], 1e-6).unwrap() else { panic!() };
        assert_eq!(edge_info(&k, l, inner).unwrap().convex, Some(false));
        let g = face_grid(&k, l, side(&k, l), 3, 3).unwrap();
        let off = g.iter().flatten().filter(|p| p[0] > 2.0 + 1e-9 && p[2] > 1.5 + 1e-9).count();
        assert!(g.len() == 3 && g.iter().all(|r| r.len() == 3) && off == 1, "{g:?}");
    }

    /// Curved faces carry their axes and radii, a cone its apex; a circle edge its axis.
    #[test]
    fn curved_faces_and_circles_described() {
        let mut k = DefaultKernel::new();
        let (cy, _) = primitive(&mut k, &WORLD, Prim::Cylinder { radius: 1.0, height: 5.0 }).unwrap();
        let Picked::One(side) = face_pick(&k, cy, [0.0, 1.0, 2.5], 1e-6).unwrap() else { panic!() };
        let f = face_info(&k, cy, side).unwrap();
        assert!(matches!(f.kind, FaceKind::Cylinder { axis: (_, d), radius } if (radius - 1.0).abs() < 1e-12 && d[2].abs() > 1.0 - 1e-12), "{f:?}");
        assert!(near(face_normal_at(&k, cy, side, [0.0, 2.0, 2.5]).unwrap().unwrap(), [0.0, 1.0, 0.0]));
        let Picked::One(rim) = edge_pick(&k, cy, [0.0, 1.0, 5.0], 1e-6).unwrap() else { panic!() };
        let i = edge_info(&k, cy, rim).unwrap();
        assert!(matches!(i.kind, EdgeKind::Circle { radius, axis: (c, n) } if (radius - 1.0).abs() < 1e-12 && near(c, [0.0, 0.0, 5.0]) && n[2].abs() > 1.0 - 1e-12), "{i:?}");
        assert_eq!(i.convex, Some(true));
        let (cone, _) = primitive(&mut k, &WORLD, Prim::Cone { bottom: 3.0, top: 0.0, height: 2.0 }).unwrap();
        let Picked::One(side) = face_pick(&k, cone, [0.0, 1.5, 1.0], 1e-6).unwrap() else { panic!() };
        let f = face_info(&k, cone, side).unwrap();
        assert!(matches!(f.kind, FaceKind::Cone { apex, half_angle, .. } if near(apex, [0.0, 0.0, 2.0]) && (half_angle - 1.5f64.atan()).abs() < 1e-12), "{f:?}");
    }
}
