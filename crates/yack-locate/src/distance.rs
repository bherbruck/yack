// yack: The closest distance between two sets of faces, edges and vertices, from the exact geometry.
// Adapted from springrep@a03e8a1 src/distance.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: meshes nothing (M2): sides are made from a tessellation mesh::query makes; `closest`
// and Nearest's constructors moved there; then to yack-locate (M3 W5), paths and visibility only;
// hardening (w-h1): a bad mesh is an error, and the Newton search and its Cholesky solve run over
// their variables with checked access rather than positions.
//! The closest distance between two sets of faces, edges and vertices, from the exact geometry.
//!
//! Both sides are meshed, and the closest pair of mesh pieces gives the distance to within the two
//! meshes' deflections. Every pair of pieces that close to it seeds a Newton search for a
//! stationary point of the exact distance between the surfaces, curves and points they come from,
//! and a result counts only where both its points lie on their face or edge. The true minimum lies
//! in the basin of one of those seeds, so the least result is exact. Where a surface's nearest
//! point lies outside its face, the face's minimum is on its boundary, which its edges' pieces
//! cover. Whether a point lies on its face, and whether one solid holds another, are asked of the
//! exact faces, as the booleans ask them: the meshes only seed the search. Solids one inside the
//! other are at distance zero.

use std::collections::BTreeMap;

use crate::{
    classify,
    topo::{
        domain::{Domain, Where},
        face_domain,
    },
    error::Result,
    geom::{Curve, Surface},
    math::{Aabb, Point3, Vec3},
    math::bvh::{Bvh, on_segment, on_triangle, segment_triangle, segments},
    geom::MeshOptions,
    tol,
    topo::{Brep, EdgeId, FaceId, SolidId, VertexId},
};

/// What one side of a distance is: faces, edges, vertices and solids of one arena. A face brings
/// its edges and vertices, an edge its vertices, and a solid its faces, and counts its inside too.
#[derive(Clone, Debug, Default)]
pub struct Selection {
    pub faces: Vec<FaceId>,
    pub edges: Vec<EdgeId>,
    pub vertices: Vec<VertexId>,
    pub solids: Vec<SolidId>,
}

impl Selection {
    /// Every solid of the arena, inside and boundary.
    pub fn body(b: &Brep) -> Selection {
        Selection {
            solids: b.solid_ids().collect(),
            ..Selection::default()
        }
    }

    pub fn face(f: FaceId) -> Selection {
        Selection {
            faces: vec![f],
            ..Selection::default()
        }
    }

    pub fn edge(e: EdgeId) -> Selection {
        Selection {
            edges: vec![e],
            ..Selection::default()
        }
    }

    pub fn vertex(v: VertexId) -> Selection {
        Selection {
            vertices: vec![v],
            ..Selection::default()
        }
    }
}

/// The closest distance between two selections, and a point of each at that distance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Closest {
    pub distance: f64,
    pub on_a: Point3,
    pub on_b: Point3,
}

/// Which entity a mesh piece or a candidate point belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Entity {
    Face(FaceId),
    Edge(EdgeId),
    Vertex(VertexId),
}

/// A piece of one side's mesh: a face's triangle, an edge's segment between two curve parameters,
/// or a vertex.
#[derive(Clone, Copy, Debug)]
enum Piece {
    Tri(FaceId, [Point3; 3]),
    Seg(EdgeId, [Point3; 2]),
    Pt(VertexId, Point3),
}

impl Piece {
    fn entity(&self) -> Entity {
        match *self {
            Piece::Tri(f, _) => Entity::Face(f),
            Piece::Seg(e, _) => Entity::Edge(e),
            Piece::Pt(v, _) => Entity::Vertex(v),
        }
    }

    fn bbox(&self) -> Aabb {
        let mut bb = Aabb::EMPTY;
        match self {
            Piece::Tri(_, p) => p.iter().for_each(|&q| bb.add(q)),
            Piece::Seg(_, p) => p.iter().for_each(|&q| bb.add(q)),
            Piece::Pt(_, p) => bb.add(*p),
        }
        bb
    }
}

/// One side, expanded: its vertices, its mesh pieces, and its solids' meshes.
pub struct Side<'a> {
    b: &'a Brep,
    vertices: Vec<VertexId>,
    pieces: Vec<Piece>,
    /// The faces of the selected solids, for whether a point lies inside one.
    solid_faces: Vec<FaceId>,
    deflection: f64,
}

impl<'a> Side<'a> {
    /// The side `sel` names, from a tessellation of `b` made within `opt`: the triangles
    /// `positions` and `triangles` give, `faces` naming each face's run of them; an error when a
    /// run passes the last triangle or a triangle names a corner past `positions`.
    pub fn from_tessellation(
        b: &'a Brep,
        sel: &Selection,
        positions: &[Point3],
        triangles: &[[u32; 3]],
        faces_of: &[(FaceId, std::ops::Range<usize>)],
        opt: &MeshOptions,
    ) -> Result<Side<'a>> {
        let mut faces = sel.faces.clone();
        for &s in &sel.solids {
            faces.extend(b.solid_faces(s));
        }
        faces.sort();
        faces.dedup();
        let mut edges = sel.edges.clone();
        for &f in &faces {
            for &lp in &b.face(f).loops {
                edges.extend(b.lp(lp).coedges.iter().map(|&co| b.coedge(co).edge));
            }
        }
        edges.sort();
        edges.dedup();
        let mut vertices = sel.vertices.clone();
        for &e in &edges {
            vertices.extend([b.edge(e).start, b.edge(e).end]);
        }
        vertices.sort();
        vertices.dedup();

        let corner = |i: u32| {
            positions.get(i as usize).copied().ok_or_else(|| {
                crate::Error::InvalidInput(format!(
                    "a triangle's corner index {i} is past the mesh's {} positions",
                    positions.len()
                ))
            })
        };
        let wanted: BTreeMap<FaceId, ()> = faces.iter().map(|&f| (f, ())).collect();
        let solid_faces: BTreeMap<FaceId, ()> = sel
            .solids
            .iter()
            .flat_map(|&s| b.solid_faces(s))
            .map(|f| (f, ()))
            .collect();
        let mut pieces = Vec::new();
        for (f, range) in faces_of {
            let run = triangles.get(range.clone()).ok_or_else(|| {
                crate::Error::InvalidInput(format!(
                    "a face's triangles {range:?} pass the mesh's {} triangles",
                    triangles.len()
                ))
            })?;
            if !wanted.contains_key(f) {
                continue;
            }
            for &[i, j, k] in run {
                pieces.push(Piece::Tri(*f, [corner(i)?, corner(j)?, corner(k)?]));
            }
        }
        for &e in &edges {
            let edge = b.edge(e);
            let Some(c) = &edge.curve else { continue };
            let (t0, t1) = edge.range;
            let ts = crate::geom::edge_params(c, t0, t1, opt);
            for pair in ts.windows(2) {
                if let &[t0, t1] = pair {
                    pieces.push(Piece::Seg(e, [c.eval(t0), c.eval(t1)]));
                }
            }
        }
        for &v in &vertices {
            pieces.push(Piece::Pt(v, b.vertex(v).point));
        }
        Ok(Side {
            b,
            vertices,
            pieces,
            solid_faces: solid_faces.into_keys().collect(),
            deflection: opt.linear,
        })
    }

    /// The exact geometry an entity lies on.
    fn geo(&self, e: Entity) -> Geo<'a> {
        match e {
            Entity::Face(f) => Geo::Surface(&self.b.face(f).surface),
            Entity::Edge(id) => {
                let edge = self.b.edge(id);
                let (t0, t1) = edge.range;
                match &edge.curve {
                    Some(c) => Geo::Curve(c, t0.min(t1), t0.max(t1)),
                    None => Geo::Point(self.b.vertex(edge.start).point),
                }
            }
            Entity::Vertex(v) => Geo::Point(self.b.vertex(v).point),
        }
    }
}

impl Side<'_> {
    /// Whether one of this side's solids holds `inner`'s, given that their boundaries do not meet:
    /// then every point of `inner`'s boundary is in or every one is out, and one of its mesh's
    /// corners, which lie on its exact boundary, says which. The solid's own classifier decides,
    /// on the exact faces: a corner within the mesh's deflection of this side's boundary falls
    /// either side of the mesh, as a dowel a hair smaller than its hole does all round. The point,
    /// in both, where it holds.
    fn holds(&self, inner: &Side) -> Option<Point3> {
        if self.solid_faces.is_empty() {
            return None;
        }
        // Only a solid whose box holds the other's can hold it.
        let boxed = |ps: &[Piece]| {
            let mut bb = Aabb::EMPTY;
            ps.iter().for_each(|p| bb.union(&p.bbox()));
            bb
        };
        let (outer, within) = (boxed(&self.pieces), boxed(&inner.pieces));
        let pad = self.deflection + inner.deflection;
        let axes = |b: &Aabb| [b.min.to_array(), b.max.to_array()];
        let ([wmin, wmax], [omin, omax]) = (axes(&within), axes(&outer));
        let sticks_out = wmin
            .iter()
            .zip(wmax)
            .zip(omin.iter().zip(omax))
            .any(|((&wl, wh), (&ol, oh))| wl < ol - pad || wh > oh + pad);
        if sticks_out {
            return None;
        }
        let pad = Vec3::new(tol::LINEAR, tol::LINEAR, tol::LINEAR);
        let faces: Vec<(Domain, Aabb)> = self
            .solid_faces
            .iter()
            .map(|&f| {
                let bb = crate::topo::bbox::face_box(self.b, f);
                let padded = Aabb {
                    min: bb.min - pad,
                    max: bb.max + pad,
                };
                (face_domain(self.b, f), padded)
            })
            .collect();
        let faces: Vec<(&Domain, Aabb)> = faces.iter().map(|(d, bb)| (d, *bb)).collect();
        let corners = inner.pieces.iter().flat_map(|p| match *p {
            Piece::Tri(_, t) => t.to_vec(),
            Piece::Seg(_, s) => s.to_vec(),
            Piece::Pt(_, q) => vec![q],
        });
        // A corner no ray decides lies on this side's boundary within rounding: the next is asked.
        corners
            .take(16)
            .find_map(|p| classify::contains(&faces, p).map(|inside| inside.then_some(p)))
            .flatten()
    }
}

/// The exact geometry under one side of a candidate, with its free parameters.
#[derive(Clone, Copy)]
enum Geo<'a> {
    Point(Point3),
    /// A curve over its edge's range.
    Curve(&'a Curve, f64, f64),
    Surface(&'a Surface),
}

/// A point, its first derivatives by each free parameter, and its second.
type Jet = (Point3, [Vec3; 2], [[Vec3; 2]; 2]);

impl Geo<'_> {
    fn dim(&self) -> usize {
        match self {
            Geo::Point(_) => 0,
            Geo::Curve(..) => 1,
            Geo::Surface(_) => 2,
        }
    }

    /// The point at parameters `[u, v]` (a curve takes `u`, a point neither), with its
    /// derivatives.
    fn eval(&self, [u, v]: [f64; 2]) -> Jet {
        let z = Vec3::ZERO;
        match *self {
            Geo::Point(p) => (p, [z; 2], [[z; 2]; 2]),
            Geo::Curve(c, ..) => {
                let (p, d, dd) = c.d2(u);
                (p, [d, z], [[dd, z], [z, z]])
            }
            // The surface gives first derivatives; the second come from their central differences,
            // which only steer the search: the gradient it stops on is exact.
            Geo::Surface(s) => {
                let (p, su, sv) = s.d1(u, v);
                let (hu, hv) = (1e-6 * (1.0 + u.abs()), 1e-6 * (1.0 + v.abs()));
                let (_, su1, sv1) = s.d1(u + hu, v);
                let (_, su0, sv0) = s.d1(u - hu, v);
                let (_, su2, sv2) = s.d1(u, v + hv);
                let (_, su3, sv3) = s.d1(u, v - hv);
                let suu = (su1 - su0) * (0.5 / hu);
                let svv = (sv2 - sv3) * (0.5 / hv);
                let suv = ((sv1 - sv0) * (0.5 / hu) + (su2 - su3) * (0.5 / hv)) * 0.5;
                (p, [su, sv], [[suu, suv], [suv, svv]])
            }
        }
    }

    /// A curve's parameter kept in its edge's range: held at an end, or, on an edge that closes
    /// on itself, carried round to the other.
    fn clamp(&self, [t, _]: &mut [f64; 2]) {
        if let Geo::Curve(_, lo, hi) = *self {
            *t = match self.closes() {
                Some(period) => lo + (*t - lo).rem_euclid(period),
                None => t.clamp(lo, hi),
            };
        }
    }

    /// The period of a curve whose range runs once round it, an edge whose two ends are one
    /// point: a foot just past that point is past neither end.
    fn closes(&self) -> Option<f64> {
        let Geo::Curve(c, lo, hi) = *self else {
            return None;
        };
        c.period().filter(|&w| hi - lo >= w * (1.0 - 1e-12))
    }

    /// The parameters of the point of the geometry nearest `p`, near it.
    fn seed(&self, p: Point3) -> [f64; 2] {
        match *self {
            Geo::Point(_) => [0.0; 2],
            Geo::Curve(c, lo, hi) => [c.project(p, lo, hi), 0.0],
            Geo::Surface(s) => {
                let uv = s.project(p);
                [uv.x, uv.y]
            }
        }
    }
}

/// A stationary point of the distance between `a` and `b` near the seeds, by Newton's method with
/// a damping that keeps each step downhill; the points and their distance.
fn descend(a: Geo, b: Geo, sa: [f64; 2], sb: [f64; 2]) -> (Point3, Point3, [f64; 4]) {
    let (da, db) = (a.dim(), b.dim());
    let n = da + db;
    let [sa0, sa1] = sa;
    let [sb0, sb1] = sb;
    let mut z = [sa0, sa1, sb0, sb1];
    let split = |&[a0, a1, b0, b1]: &[f64; 4]| ([a0, a1], [b0, b1]);
    let value = |z: &[f64; 4]| {
        let (xa, xb) = split(z);
        let (p, ..) = a.eval(xa);
        let (q, ..) = b.eval(xb);
        let r = p - q;
        0.5 * r.dot(r)
    };
    // The variables, in order: a's first `da` slots of `z`, then b's first `db` from slot 2.
    let is_variable = |slot: usize| slot < da || (2..2 + db).contains(&slot);
    let mut damp = 0.0;
    for _ in 0..200 {
        let (xa, xb) = split(&z);
        let (p, pd, pdd) = a.eval(xa);
        let (q, qd, qdd) = b.eval(xb);
        let r = p - q;
        let f = 0.5 * r.dot(r);
        // Each variable's derivative of `r`: a's, then b's negated.
        let mut columns = [Vec3::ZERO; 4];
        let derivatives = pd.iter().take(da).copied().chain(qd.iter().take(db).map(|&d| -d));
        for (column, d) in columns.iter_mut().zip(derivatives) {
            *column = d;
        }
        // `r` against each pair of one side's second derivatives; zero across the sides.
        let mut second = [[0.0; 4]; 4];
        for (row, pdd_row) in second.iter_mut().zip(&pdd).take(da) {
            for (cell, &dd) in row.iter_mut().zip(pdd_row).take(da) {
                *cell = r.dot(dd);
            }
        }
        for (row, qdd_row) in second.iter_mut().skip(da).zip(&qdd).take(db) {
            for (cell, &dd) in row.iter_mut().skip(da).zip(qdd_row).take(db) {
                *cell = -r.dot(dd);
            }
        }
        let mut g = [0.0; 4];
        let mut h = [[0.0; 4]; 4];
        let rows = g.iter_mut().zip(h.iter_mut()).zip(columns).zip(second);
        for (((gi, h_row), ci), second_row) in rows.take(n) {
            *gi = r.dot(ci);
            for ((hij, cj), sij) in h_row.iter_mut().zip(columns).zip(second_row).take(n) {
                *hij = ci.dot(cj) + sij;
            }
        }
        // A curve parameter held at an end of its range by a gradient pushing past it stays.
        let mut free = [true; 4];
        let variables = z.iter().enumerate().filter(|&(slot, _)| is_variable(slot));
        for (i, ((fi, &gi), (_, &x))) in free.iter_mut().zip(&g).zip(variables).enumerate() {
            let geo = if i < da { a } else { b };
            if let Geo::Curve(_, lo, hi) = geo
                && geo.closes().is_none()
                && ((x <= lo && gi > 0.0) || (x >= hi && gi < 0.0))
            {
                *fi = false;
            }
        }
        let diagonal = h.iter().enumerate().take(n).filter_map(|(i, row)| row.get(i));
        let scale = diagonal.map(|hii| hii.abs()).fold(1e-300, f64::max);
        let mut improved = false;
        for _ in 0..30 {
            let Some(step) = solve(&h, &g, &free, n, damp * scale) else {
                damp = (damp * 10.0).max(1e-12);
                continue;
            };
            let mut next = z;
            let slots = next.iter_mut().enumerate().filter(|(slot, _)| is_variable(*slot));
            for ((_, x), dx) in slots.zip(step) {
                *x -= dx;
            }
            let (mut na, mut nb) = split(&next);
            a.clamp(&mut na);
            b.clamp(&mut nb);
            let ([na0, na1], [nb0, nb1]) = (na, nb);
            let next = [na0, na1, nb0, nb1];
            if value(&next) < f {
                let moved = next.iter().zip(&z).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max);
                z = next;
                damp *= 0.1;
                improved = moved > 1e-15 * (1.0 + z.iter().fold(0.0f64, |m, x| m.max(x.abs())));
                break;
            }
            damp = (damp * 10.0).max(1e-12);
        }
        if !improved {
            break;
        }
    }
    let (xa, xb) = split(&z);
    (a.eval(xa).0, b.eval(xb).0, z)
}

/// The transpose of a 4 × 4 matrix.
fn transpose(m: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut t = [[0.0; 4]; 4];
    for (i, row) in m.iter().enumerate() {
        for (t_row, &v) in t.iter_mut().zip(row) {
            if let Some(cell) = t_row.get_mut(i) {
                *cell = v;
            }
        }
    }
    t
}

/// `(H + λ·I)⁻¹ g` over the free variables, by Cholesky; `None` where that is not positive
/// definite, which more damping cures.
fn solve(
    h: &[[f64; 4]; 4],
    g: &[f64; 4],
    free: &[bool; 4],
    n: usize,
    lambda: f64,
) -> Option<[f64; 4]> {
    // The system over the free variables, packed into the first `m` rows and columns.
    let is_free = |&(_, &f): &(usize, &bool)| f;
    let kept: Vec<usize> =
        free.iter().enumerate().take(n).filter(is_free).map(|(i, _)| i).collect();
    let m = kept.len();
    let mut out = [0.0; 4];
    if m == 0 {
        return Some(out);
    }
    let mut a = [[0.0; 4]; 4];
    let mut rhs = [0.0; 4];
    for ((a_row, r), &i) in a.iter_mut().zip(rhs.iter_mut()).zip(&kept) {
        let (h_row, &gi) = h.get(i).zip(g.get(i))?;
        *r = gi;
        for (cell, &j) in a_row.iter_mut().zip(&kept) {
            *cell = *h_row.get(j)?;
        }
    }
    // L Lᵀ = A + λ·I, row by row: each entry less its row's and column's earlier products.
    let mut l = [[0.0; 4]; 4];
    for (i, a_row) in a.iter().enumerate().take(m) {
        let (done, rest) = l.split_at_mut(i);
        let l_row = rest.first_mut()?;
        for (j, (l_j, &aij)) in done.iter().zip(a_row).enumerate() {
            let (before, at) = l_row.split_at_mut(j);
            let s = before.iter().zip(l_j).fold(aij + 0.0, |s, (x, y)| s - x * y);
            *at.first_mut()? = s / l_j.get(j)?;
        }
        let (before, at) = l_row.split_at_mut(i);
        let s = before.iter().fold(a_row.get(i)? + lambda, |s, x| s - x * x);
        if s <= 0.0 || !s.is_finite() {
            return None;
        }
        *at.first_mut()? = s.sqrt();
    }
    // L y = rhs, first row first.
    let mut y = [0.0; 4];
    for (i, (l_row, &ri)) in l.iter().zip(&rhs).enumerate().take(m) {
        let (before, at) = y.split_at_mut(i);
        let s = l_row.iter().zip(before.iter()).fold(ri, |s, (x, yk)| s - x * yk);
        *at.first_mut()? = s / l_row.get(i)?;
    }
    // Lᵀ x = y, last row first: row i of Lᵀ is column i of L.
    let lt = transpose(&l);
    let mut x = [0.0; 4];
    for (i, (lt_row, &yi)) in lt.iter().zip(&y).enumerate().take(m).rev() {
        let (at, after) = x.split_at_mut(i + 1);
        let s = lt_row
            .iter()
            .skip(i + 1)
            .zip(after.iter())
            .take(m - i - 1)
            .fold(yi, |s, (lki, xk)| s - lki * xk);
        *at.last_mut()? = s / lt_row.get(i)?;
    }
    for (&i, &xi) in kept.iter().zip(&x) {
        *out.get_mut(i)? = xi;
    }
    Some(out)
}

/// The closest points of two mesh pieces, and their distance.
fn piece_distance(a: &Piece, b: &Piece) -> (f64, Point3, Point3) {
    match (a, b) {
        (Piece::Pt(_, p), Piece::Pt(_, q)) => (p.dist(*q), *p, *q),
        (Piece::Pt(_, p), Piece::Seg(_, s)) => {
            let q = on_segment(*p, s[0], s[1]);
            (p.dist(q), *p, q)
        }
        (Piece::Seg(..), Piece::Pt(..)) => flip(piece_distance(b, a)),
        (Piece::Pt(_, p), Piece::Tri(_, t)) => {
            let q = on_triangle(*p, t);
            (p.dist(q), *p, q)
        }
        (Piece::Tri(..), Piece::Pt(..)) => flip(piece_distance(b, a)),
        (Piece::Seg(_, s), Piece::Seg(_, u)) => segments(s[0], s[1], u[0], u[1]),
        (Piece::Seg(_, s), Piece::Tri(_, t)) => segment_triangle(s[0], s[1], t),
        (Piece::Tri(..), Piece::Seg(..)) => flip(piece_distance(b, a)),
        (Piece::Tri(_, s), Piece::Tri(_, t)) => {
            let mut best = (f64::INFINITY, s[0], t[0]);
            for ((s0, s1), (t0, t1)) in sides(s).into_iter().zip(sides(t)) {
                let x = segment_triangle(s0, s1, t);
                if x.0 < best.0 {
                    best = x;
                }
                let y = flip(segment_triangle(t0, t1, s));
                if y.0 < best.0 {
                    best = y;
                }
            }
            best
        }
    }
}

/// The three sides of triangle `t`, each from a corner to the next.
fn sides(t: &[Point3; 3]) -> [(Point3, Point3); 3] {
    let [a, b, c] = *t;
    [(a, b), (b, c), (c, a)]
}

/// A pair of closest points the other way round.
fn flip((d, p, q): (f64, Point3, Point3)) -> (f64, Point3, Point3) {
    (d, q, p)
}

/// Keeps `c` when it is nearer than the answer so far.
fn offer(answer: &mut Option<Closest>, c: Closest) {
    if answer.is_none_or(|a| c.distance < a.distance) {
        *answer = Some(c);
    }
}

/// Seeds per pair of entities kept for the exact search, spread apart: a pair can have several
/// minima (a point on a torus's axis has a ring of them), and two large faces held parallel bring
/// thousands of pieces all equally close.
const SEEDS_PER_PAIR: usize = 32;


/// An arena prepared for asking which of its points is nearest a point: its faces, edges and
/// vertices, meshed once.
pub struct Nearest<'a> {
    side: Side<'a>,
    /// How a point to measure from is made a side: the mesher's, as the arena's own was.
    point: for<'p> fn(&'p Brep, &Selection) -> Result<Side<'p>>,
}

impl<'a> Nearest<'a> {
    /// The nearest-point search over `side`, measuring from points made sides by `point`.
    pub fn from_side(side: Side<'a>, point: for<'p> fn(&'p Brep, &Selection) -> Result<Side<'p>>) -> Nearest<'a> {
        Nearest { side, point }
    }
}

impl<'a> Nearest<'a> {
    /// The nearest point to `p`, as `on_b`, and its distance; `on_a` is `p`.
    pub fn to(&self, p: Point3) -> Result<Closest> {
        let mut point = Brep::new();
        let v = point.add_vertex(p);
        between(&(self.point)(&point, &Selection::vertex(v))?, &self.side)
    }
}

/// The closest distance between two sides already expanded.
pub fn between(side_a: &Side, side_b: &Side) -> Result<Closest> {
    let (a, b) = (side_a.b, side_b.b);
    let (pa, pb) = (&side_a.pieces, &side_b.pieces);
    let (Some(first_a), Some(first_b)) = (pa.first(), pb.first()) else {
        return Err(crate::Error::InvalidInput(
            "a distance needs something on each side".into(),
        ));
    };
    let tree = Bvh::of_boxes(pb.iter().map(Piece::bbox).collect());
    let boxes: Vec<Aabb> = pa.iter().map(Piece::bbox).collect();
    // The tree names b's pieces by their place in `pb`.
    let piece_b = |j: usize| {
        let piece = pb.get(j);
        debug_assert!(piece.is_some(), "the tree's piece {j} is past b's pieces");
        piece
    };

    // The closest pair of pieces, each query widened by the best so far.
    let mut best = piece_distance(first_a, first_b).0;
    let mut hits = Vec::new();
    for (x, bx) in pa.iter().zip(&boxes) {
        tree.within(bx, best, &mut hits);
        for y in hits.iter().filter_map(|&j| piece_b(j)) {
            best = best.min(piece_distance(x, y).0);
        }
    }
    // Every pair the exact minimum could lie between.
    let margin = 2.0 * (side_a.deflection + side_b.deflection) + tol::LINEAR;
    let reach = best + margin;
    let spread = 4.0 * (side_a.deflection + side_b.deflection);
    // Nearest first, so the closest pair of pieces always seeds its entities: on parts smaller
    // than the meshes' deflection every pair lies within reach and spread of every other.
    let mut near_pairs = Vec::new();
    for (x, bx) in pa.iter().zip(&boxes) {
        tree.within(bx, reach, &mut hits);
        for y in hits.iter().filter_map(|&j| piece_b(j)) {
            let (d, p, q) = piece_distance(x, y);
            if d <= reach {
                near_pairs.push((d, (x.entity(), y.entity()), p, q));
            }
        }
    }
    near_pairs.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.cmp(&y.1)));
    let mut seeds: BTreeMap<(Entity, Entity), Vec<(Point3, Point3)>> = BTreeMap::new();
    for (_, pair, p, q) in near_pairs {
        let kept = seeds.entry(pair).or_default();
        let near = kept
            .iter()
            .any(|&(p0, q0)| p0.dist(p) <= spread && q0.dist(q) <= spread);
        if !near && kept.len() < SEEDS_PER_PAIR {
            kept.push((p, q));
        }
    }

    let mut domains: BTreeMap<(bool, FaceId), Domain> = BTreeMap::new();
    // Whether parameters `x` of entity `e` of a side lie on it: a curve's are kept in its range, a
    // surface's must fall inside its face, by the face's exact domain. A polygon of the face's
    // outline cuts inside a curved edge, and takes a point of a hole's opening for the face.
    let mut lies_on = |side: &Side, which: bool, e: Entity, x: [f64; 2]| match e {
        Entity::Face(f) => {
            let dom = domains
                .entry((which, f))
                .or_insert_with(|| face_domain(side.b, f));
            let p = dom.surface.eval(x[0], x[1]);
            !matches!(dom.locate(p), Where::Outside)
        }
        _ => true,
    };
    let mut answer: Option<Closest> = None;
    for ((ea, eb), list) in &seeds {
        let (ga, gb) = (side_a.geo(*ea), side_b.geo(*eb));
        for &(p, q) in list {
            let (x, y, z) = descend(ga, gb, ga.seed(p), gb.seed(q));
            if lies_on(side_a, false, *ea, [z[0], z[1]]) && lies_on(side_b, true, *eb, [z[2], z[3]])
            {
                offer(
                    &mut answer,
                    Closest {
                        distance: x.dist(y),
                        on_a: x,
                        on_b: y,
                    },
                );
            }
        }
    }
    // Every vertex pair is a real pair of points, so there is always an answer.
    if answer.is_none() {
        for &va in &side_a.vertices {
            for &vb in &side_b.vertices {
                let (x, y) = (a.vertex(va).point, b.vertex(vb).point);
                offer(
                    &mut answer,
                    Closest {
                        distance: x.dist(y),
                        on_a: x,
                        on_b: y,
                    },
                );
            }
        }
    }
    let mut answer = match answer {
        Some(c) => c,
        None => {
            let (_, x, y) = piece_distance(first_a, first_b);
            Closest {
                distance: x.dist(y),
                on_a: x,
                on_b: y,
            }
        }
    };
    // Boundaries apart, however little, one solid may still hold the other.
    if answer.distance > tol::LINEAR
        && let Some(p) = side_b.holds(side_a).or_else(|| side_a.holds(side_b))
    {
        answer = Closest {
            distance: 0.0,
            on_a: p,
            on_b: p,
        };
    }
    Ok(answer)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geom::{Line, Plane},
        math::Frame,
    };

    /// A 4 × 4 matrix with `rows` in its top left corner.
    fn padded(rows: &[&[f64]]) -> [[f64; 4]; 4] {
        let mut m = [[0.0; 4]; 4];
        for (i, row) in rows.iter().enumerate() {
            for (j, &v) in row.iter().enumerate() {
                m[i][j] = v;
            }
        }
        m
    }

    #[test]
    fn solve_matches_the_closed_form_inverse() {
        // [4 1; 1 3]⁻¹ [1 2] = [1 7] / 11.
        let h = padded(&[&[4.0, 1.0], &[1.0, 3.0]]);
        let x = solve(&h, &[1.0, 2.0, 0.0, 0.0], &[true; 4], 2, 0.0).unwrap();
        assert!((x[0] - 1.0 / 11.0).abs() < 1e-14 && (x[1] - 7.0 / 11.0).abs() < 1e-14, "{x:?}");
        assert_eq!(&x[2..], &[0.0, 0.0]);
    }

    #[test]
    fn solve_leaves_a_pinned_variable_and_damps() {
        let h = padded(&[&[4.0, 1.0], &[1.0, 3.0]]);
        let x = solve(&h, &[1.0, 2.0, 0.0, 0.0], &[true, false, true, true], 2, 0.0).unwrap();
        assert_eq!(x, [0.25, 0.0, 0.0, 0.0]);
        let x = solve(&padded(&[&[1.0]]), &[3.0, 0.0, 0.0, 0.0], &[true; 4], 1, 1.0).unwrap();
        assert!((x[0] - 1.5).abs() < 1e-15, "{x:?}");
        assert_eq!(solve(&h, &[1.0; 4], &[false; 4], 2, 0.0), Some([0.0; 4]));
    }

    #[test]
    fn solve_refuses_what_is_not_positive_definite() {
        assert_eq!(solve(&padded(&[&[-1.0]]), &[1.0, 0.0, 0.0, 0.0], &[true; 4], 1, 0.0), None);
        let h = padded(&[&[1.0, 2.0], &[2.0, 1.0]]);
        assert_eq!(solve(&h, &[1.0, 1.0, 0.0, 0.0], &[true; 4], 2, 0.0), None);
    }

    #[test]
    fn descend_drops_a_point_onto_a_line() {
        let line = Curve::Line(Line { origin: Vec3::ZERO, dir: Vec3::X });
        let p = Vec3::new(3.0, 4.0, 0.0);
        let (on_line, at_p, _) = descend(Geo::Curve(&line, 0.0, 10.0), Geo::Point(p), [0.0; 2], [0.0; 2]);
        assert!(on_line.dist(Vec3::new(3.0, 0.0, 0.0)) < 1e-12, "{on_line:?}");
        assert_eq!(at_p, p);
        // Held at the end of the edge's range when the foot lies past it.
        let (end, ..) = descend(Geo::Curve(&line, 0.0, 2.0), Geo::Point(p), [0.0; 2], [0.0; 2]);
        assert!(end.dist(Vec3::new(2.0, 0.0, 0.0)) < 1e-12, "{end:?}");
    }

    #[test]
    fn descend_drops_a_point_onto_a_plane() {
        let plane = Surface::Plane(Plane { frame: Frame::WORLD });
        let p = Vec3::new(1.0, -2.0, 5.0);
        let (foot, ..) = descend(Geo::Surface(&plane), Geo::Point(p), [0.0; 2], [0.0; 2]);
        assert!(foot.dist(Vec3::new(1.0, -2.0, 0.0)) < 1e-12, "{foot:?}");
    }

    #[test]
    fn triangles_one_above_the_other_are_their_gap_apart() {
        let f = FaceId::from_raw(0);
        let low = Piece::Tri(f, [Vec3::ZERO, Vec3::X, Vec3::Y]);
        let high = Piece::Tri(f, [Vec3::new(0.2, 0.2, 1.5), Vec3::new(2.0, 0.2, 1.5), Vec3::new(0.2, 2.0, 1.5)]);
        let (d, p, q) = piece_distance(&low, &high);
        assert!((d - 1.5).abs() < 1e-12 && (p.dist(q) - 1.5).abs() < 1e-12, "{d}");
        let v = Piece::Pt(VertexId::from_raw(0), Vec3::new(0.25, 0.25, -2.0));
        assert!((piece_distance(&v, &low).0 - 2.0).abs() < 1e-12);
        assert!((piece_distance(&low, &v).0 - 2.0).abs() < 1e-12);
    }

    fn invalid_input(r: Result<Side<'_>>) -> String {
        match r {
            Err(crate::Error::InvalidInput(m)) => m,
            Err(e) => panic!("expected InvalidInput, got {e}"),
            Ok(_) => panic!("expected InvalidInput, got a side"),
        }
    }

    #[test]
    fn a_bad_mesh_is_invalid_input_for_a_side() {
        let mut b = Brep::new();
        let f = b.add_face(Surface::Plane(Plane { frame: Frame::WORLD }), false, []);
        let corners = [Vec3::ZERO, Vec3::X, Vec3::Y];
        let opt = MeshOptions::within(0.1);
        let sel = Selection::face(f);
        let m = invalid_input(Side::from_tessellation(&b, &sel, &corners, &[[0, 1, 2]], &[(f, 0..3)], &opt));
        assert!(m.contains("pass the mesh's 1 triangles"), "{m}");
        let m = invalid_input(Side::from_tessellation(&b, &sel, &corners, &[[0, 1, 7]], &[(f, 0..1)], &opt));
        assert!(m.contains("corner index 7"), "{m}");
        // A face not selected still needs a run inside the mesh, but not good corners.
        let none = Selection::default();
        assert!(Side::from_tessellation(&b, &none, &corners, &[[0, 1, 7]], &[(f, 0..1)], &opt).is_ok());
    }

    #[test]
    fn a_distance_needs_something_on_each_side() {
        let b = Brep::new();
        let opt = MeshOptions::within(0.1);
        let side = Side::from_tessellation(&b, &Selection::default(), &[], &[], &[], &opt).unwrap();
        assert!(matches!(between(&side, &side), Err(crate::Error::InvalidInput(_))));
    }
}
