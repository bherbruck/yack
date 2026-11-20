// yack: a spade-shaped constrained Delaunay triangulation (the subset cadrs's tess.rs uses).
//! Incremental Delaunay insertion with Lawson flips, constraint segments recovered by Sloan's
//! flipping, and Ruppert-style refinement (circumcentres of triangles under the angle limit,
//! constraint segments kept whole). Triangles are stored with their corners counter-clockwise;
//! a directed edge `a → b` belongs to the triangle on its left. Vertex indices are the input
//! order (a duplicate point stays out of the triangulation instead of renumbering the rest), and
//! inner-face indices are `1..=num_inner_faces()`, stable while points are only added.
//!
//! The predicates are plain floating point (spade's are exact): walks and flip passes carry
//! step budgets so a near-degenerate input ends in an error, never a hang.

use std::collections::BTreeSet;
use std::marker::PhantomData;

use crate::fx::FxHashMap;

/// A point of the triangulation's plane.
#[derive(Clone, Copy, PartialEq, PartialOrd, Debug, Default)]
pub struct Point2<S> {
    pub x: S,
    pub y: S,
}

impl<S> Point2<S> {
    /// The point `(x, y)`.
    pub const fn new(x: S, y: S) -> Self {
        Point2 { x, y }
    }
}

/// What can be a triangulation vertex: anything with a position.
pub trait HasPosition {
    type Scalar;
    /// The position.
    fn position(&self) -> Point2<Self::Scalar>;
}

impl HasPosition for Point2<f64> {
    type Scalar = f64;
    fn position(&self) -> Point2<f64> {
        *self
    }
}

/// spade's `Triangulation` trait; the methods are inherent here.
pub trait Triangulation {}
impl<V: HasPosition<Scalar = f64>> Triangulation for ConstrainedDelaunayTriangulation<V> {}

/// A point could not be inserted (not finite, or the triangulation got stuck).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InsertionError;

impl std::fmt::Display for InsertionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "the point could not be inserted into the triangulation")
    }
}
impl std::error::Error for InsertionError {}

/// The smallest angle refinement aims for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngleLimit {
    rad: f64,
}

impl AngleLimit {
    /// `deg` degrees.
    pub fn from_deg(deg: f64) -> Self {
        AngleLimit { rad: deg.to_radians() }
    }
    /// `rad` radians.
    pub fn from_rad(rad: f64) -> Self {
        AngleLimit { rad }
    }
    /// No limit.
    pub fn no_limit() -> Self {
        AngleLimit { rad: 0.0 }
    }
}

/// How to refine.
#[derive(Clone, Copy, Debug)]
pub struct RefinementParameters<S> {
    exclude_outer: bool,
    keep_constraints: bool,
    angle: AngleLimit,
    min_area: Option<S>,
    max_area: Option<S>,
    max_added: usize,
}

impl RefinementParameters<f64> {
    /// spade's defaults: 30°, no area bounds, a budget of 10 times the vertices.
    pub fn new() -> Self {
        RefinementParameters {
            exclude_outer: false,
            keep_constraints: false,
            angle: AngleLimit::from_deg(30.0),
            min_area: None,
            max_area: None,
            max_added: usize::MAX,
        }
    }
    /// Leave the faces outside the constraint loops (by parity from the hull) alone.
    pub fn exclude_outer_faces(mut self, on: bool) -> Self {
        self.exclude_outer = on;
        self
    }
    /// Never split a constraint segment.
    pub fn keep_constraint_edges(mut self) -> Self {
        self.keep_constraints = true;
        self
    }
    /// The smallest angle to aim for.
    pub fn with_angle_limit(mut self, a: AngleLimit) -> Self {
        self.angle = a;
        self
    }
    /// Triangles smaller than this are left as they are.
    pub fn with_min_required_area(mut self, a: f64) -> Self {
        self.min_area = Some(a);
        self
    }
    /// Triangles larger than this are split.
    pub fn with_max_allowed_area(mut self, a: f64) -> Self {
        self.max_area = Some(a);
        self
    }
    /// At most this many points are added.
    pub fn with_max_additional_vertices(mut self, n: usize) -> Self {
        self.max_added = n;
        self
    }
}

impl Default for RefinementParameters<f64> {
    fn default() -> Self {
        Self::new()
    }
}

/// What refinement did.
#[derive(Clone, Debug)]
pub struct RefinementResult {
    /// The faces left out (outside the loops).
    pub excluded_faces: Vec<handles::FixedFaceHandle<handles::InnerTag>>,
    /// Whether every face met the limits (else the budget ran out).
    pub refinement_complete: bool,
}

/// Where a point is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PositionInTriangulation {
    OnVertex(handles::FixedVertexHandle),
    OnEdge(handles::FixedDirectedEdgeHandle),
    OnFace(handles::FixedFaceHandle<handles::InnerTag>),
    OutsideOfConvexHull(handles::FixedDirectedEdgeHandle),
    NoTriangulation,
}

pub mod handles {
    //! Handles to vertices, edges and faces: fixed ones (indices) and borrowed ones.
    use super::{ConstrainedDelaunayTriangulation, HasPosition, NONE, Point2};
    use std::marker::PhantomData;

    /// An inner face's tag.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
    pub struct InnerTag;
    /// The tag of a face that may be the outer one.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
    pub struct PossiblyOuterTag;

    /// A vertex by index.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
    pub struct FixedVertexHandle(pub(crate) usize);

    impl FixedVertexHandle {
        /// The vertex of index `i` (input order).
        pub fn from_index(i: usize) -> Self {
            FixedVertexHandle(i)
        }
        /// The index.
        pub fn index(&self) -> usize {
            self.0
        }
    }

    /// A face by index: the outer face is 0, inner faces `1..=num_inner_faces()`.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
    pub struct FixedFaceHandle<T>(pub(crate) usize, pub(crate) PhantomData<T>);

    impl<T> FixedFaceHandle<T> {
        /// The index.
        pub fn index(&self) -> usize {
            self.0
        }
    }

    /// A directed edge, by its two vertices.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
    pub struct FixedDirectedEdgeHandle {
        pub(crate) from: usize,
        pub(crate) to: usize,
    }

    impl FixedDirectedEdgeHandle {
        /// The same edge the other way.
        pub fn rev(&self) -> Self {
            FixedDirectedEdgeHandle { from: self.to, to: self.from }
        }
    }

    /// A vertex of a triangulation.
    pub struct VertexHandle<'a, V> {
        pub(crate) cdt: &'a ConstrainedDelaunayTriangulation<V>,
        pub(crate) i: usize,
    }

    impl<V> Clone for VertexHandle<'_, V> {
        fn clone(&self) -> Self {
            *self
        }
    }
    impl<V> Copy for VertexHandle<'_, V> {}

    impl<'a, V: HasPosition<Scalar = f64>> VertexHandle<'a, V> {
        /// The fixed handle.
        pub fn fix(&self) -> FixedVertexHandle {
            FixedVertexHandle(self.i)
        }
        /// The vertex's data.
        pub fn data(&self) -> &'a V {
            &self.cdt.verts[self.i]
        }
        /// The position.
        pub fn position(&self) -> Point2<f64> {
            self.cdt.verts[self.i].position()
        }
        /// The edges out of this vertex.
        pub fn out_edges(&self) -> impl Iterator<Item = DirectedEdgeHandle<'a, V>> + 'a {
            let cdt = self.cdt;
            let i = self.i;
            let mut out: Vec<usize> = cdt
                .edges
                .keys()
                .filter(|&&(a, _)| a == i)
                .map(|&(_, b)| b)
                .chain(cdt.edges.keys().filter(|&&(_, b)| b == i).map(|&(a, _)| a))
                .collect();
            out.sort_unstable();
            out.dedup();
            out.into_iter().map(move |b| DirectedEdgeHandle { cdt, from: i, to: b })
        }
    }

    /// A face of a triangulation (`t == NONE` is the outer face).
    pub struct FaceHandle<'a, T, V> {
        pub(crate) cdt: &'a ConstrainedDelaunayTriangulation<V>,
        pub(crate) t: usize,
        pub(crate) _tag: PhantomData<T>,
    }

    impl<T, V> Clone for FaceHandle<'_, T, V> {
        fn clone(&self) -> Self {
            *self
        }
    }
    impl<T, V> Copy for FaceHandle<'_, T, V> {}

    impl<'a, T, V: HasPosition<Scalar = f64>> FaceHandle<'a, T, V> {
        /// The fixed handle.
        pub fn fix(&self) -> FixedFaceHandle<T> {
            FixedFaceHandle(if self.t == NONE { 0 } else { self.t + 1 }, PhantomData)
        }
        /// Whether this is the outer face.
        pub fn is_outer(&self) -> bool {
            self.t == NONE
        }
        /// The face as an inner one, unless it is the outer face.
        pub fn as_inner(&self) -> Option<FaceHandle<'a, InnerTag, V>> {
            (self.t != NONE).then_some(FaceHandle { cdt: self.cdt, t: self.t, _tag: PhantomData })
        }
    }

    impl<'a, V: HasPosition<Scalar = f64>> FaceHandle<'a, InnerTag, V> {
        /// The corners, counter-clockwise.
        pub fn vertices(&self) -> [VertexHandle<'a, V>; 3] {
            let v = self.cdt.tris[self.t];
            v.map(|i| VertexHandle { cdt: self.cdt, i })
        }
        /// The corners' positions.
        pub fn positions(&self) -> [Point2<f64>; 3] {
            self.cdt.tris[self.t].map(|i| self.cdt.verts[i].position())
        }
        /// The sides, counter-clockwise (corner i to corner i + 1).
        pub fn adjacent_edges(&self) -> [DirectedEdgeHandle<'a, V>; 3] {
            let v = self.cdt.tris[self.t];
            [0, 1, 2].map(|k| DirectedEdgeHandle { cdt: self.cdt, from: v[k], to: v[(k + 1) % 3] })
        }
        /// The area.
        pub fn area(&self) -> f64 {
            let [a, b, c] = self.positions();
            0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y))
        }
    }

    /// A directed edge of a triangulation.
    pub struct DirectedEdgeHandle<'a, V> {
        pub(crate) cdt: &'a ConstrainedDelaunayTriangulation<V>,
        pub(crate) from: usize,
        pub(crate) to: usize,
    }

    impl<V> Clone for DirectedEdgeHandle<'_, V> {
        fn clone(&self) -> Self {
            *self
        }
    }
    impl<V> Copy for DirectedEdgeHandle<'_, V> {}

    impl<'a, V: HasPosition<Scalar = f64>> DirectedEdgeHandle<'a, V> {
        /// The fixed handle.
        pub fn fix(&self) -> FixedDirectedEdgeHandle {
            FixedDirectedEdgeHandle { from: self.from, to: self.to }
        }
        /// The start.
        pub fn from(&self) -> VertexHandle<'a, V> {
            VertexHandle { cdt: self.cdt, i: self.from }
        }
        /// The end.
        pub fn to(&self) -> VertexHandle<'a, V> {
            VertexHandle { cdt: self.cdt, i: self.to }
        }
        /// The same edge the other way.
        pub fn rev(&self) -> Self {
            DirectedEdgeHandle { cdt: self.cdt, from: self.to, to: self.from }
        }
        /// The face on the left.
        pub fn face(&self) -> FaceHandle<'a, PossiblyOuterTag, V> {
            let t = self.cdt.edges.get(&(self.from, self.to)).map_or(NONE, |&(t, _)| t);
            FaceHandle { cdt: self.cdt, t, _tag: PhantomData }
        }
        /// Whether it is (part of) a constraint segment.
        pub fn is_constraint_edge(&self) -> bool {
            self.cdt.is_constraint(self.from, self.to)
        }
        /// The two ends' positions.
        pub fn positions(&self) -> [Point2<f64>; 2] {
            [self.cdt.verts[self.from].position(), self.cdt.verts[self.to].position()]
        }
    }
}

use handles::{DirectedEdgeHandle, FaceHandle, FixedDirectedEdgeHandle, FixedFaceHandle, FixedVertexHandle, InnerTag, VertexHandle};

/// No triangle (the outer face), as a triangle index.
pub(crate) const NONE: usize = usize::MAX;

fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// > 0 when `d` is strictly inside the circle through the counter-clockwise `a`, `b`, `c`.
fn incircle(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> f64 {
    let (adx, ady) = (a[0] - d[0], a[1] - d[1]);
    let (bdx, bdy) = (b[0] - d[0], b[1] - d[1]);
    let (cdx, cdy) = (c[0] - d[0], c[1] - d[1]);
    let ad = adx * adx + ady * ady;
    let bd = bdx * bdx + bdy * bdy;
    let cd = cdx * cdx + cdy * cdy;
    adx * (bdy * cd - bd * cdy) - ady * (bdx * cd - bd * cdx) + ad * (bdx * cdy - bdy * cdx)
}

fn ckey(a: usize, b: usize) -> (usize, usize) {
    (a.min(b), a.max(b))
}

/// A constrained Delaunay triangulation of vertices `V`.
pub struct ConstrainedDelaunayTriangulation<V> {
    pub(crate) verts: Vec<V>,
    pos: Vec<[f64; 2]>,
    /// Whether each vertex is in the triangulation (a duplicate point is not).
    used: Vec<bool>,
    /// The triangles' corners, counter-clockwise.
    pub(crate) tris: Vec<[usize; 3]>,
    /// Directed edge → (its triangle, the side's index in it).
    pub(crate) edges: FxHashMap<(usize, usize), (usize, u8)>,
    /// Hull vertex → the next hull vertex: hull edge a → b is the triangle side b → a that has
    /// no twin (the hull runs clockwise).
    hull: FxHashMap<usize, usize>,
    constraints: BTreeSet<(usize, usize)>,
    /// Where the next walk starts.
    last: usize,
    /// The triangles made or changed since this was last cleared.
    touched: Vec<usize>,
}

impl<V: HasPosition<Scalar = f64>> Default for ConstrainedDelaunayTriangulation<V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<V: HasPosition<Scalar = f64>> ConstrainedDelaunayTriangulation<V> {
    /// An empty triangulation.
    pub fn new() -> Self {
        ConstrainedDelaunayTriangulation {
            verts: Vec::new(),
            pos: Vec::new(),
            used: Vec::new(),
            tris: Vec::new(),
            edges: FxHashMap::default(),
            hull: FxHashMap::default(),
            constraints: BTreeSet::new(),
            last: 0,
            touched: Vec::new(),
        }
    }

    /// The triangulation of `vertices` with the segments `edges` (vertex index pairs) in it;
    /// a segment crossing one already in is left out and handed to `on_conflict_found`.
    pub fn try_bulk_load_cdt(
        vertices: Vec<V>,
        edges: Vec<[usize; 2]>,
        mut on_conflict_found: impl FnMut([usize; 2]),
    ) -> Result<Self, InsertionError> {
        let mut cdt = Self::new();
        cdt.pos = vertices.iter().map(|v| {
            let p = v.position();
            [p.x, p.y]
        }).collect();
        if cdt.pos.iter().any(|p| !(p[0].is_finite() && p[1].is_finite())) {
            return Err(InsertionError);
        }
        cdt.used = vec![false; vertices.len()];
        cdt.verts = vertices;
        let n = cdt.verts.len();
        // Insertion order: rows across the points, alternating direction, so walks are short.
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in &cdt.pos {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let rows = ((n as f64).sqrt().ceil() as usize).max(1);
        let h = ((hi[1] - lo[1]) / rows as f64).max(f64::MIN_POSITIVE);
        let mut order: Vec<usize> = (0..n).collect();
        let row_of = |i: usize, pos: &[[f64; 2]]| (((pos[i][1] - lo[1]) / h) as usize).min(rows);
        order.sort_by(|&a, &b| {
            let (ra, rb) = (row_of(a, &cdt.pos), row_of(b, &cdt.pos));
            ra.cmp(&rb).then_with(|| {
                let c = cdt.pos[a][0].total_cmp(&cdt.pos[b][0]);
                if ra % 2 == 0 { c } else { c.reverse() }
            }).then(a.cmp(&b))
        });
        // The first triangle: the first point, the next distinct one, the next off their line.
        let Some(&i0) = order.first() else {
            return Ok(cdt);
        };
        let Some(&i1) = order.iter().find(|&&j| cdt.pos[j] != cdt.pos[i0]) else {
            cdt.used[i0] = true;
            return Ok(cdt);
        };
        let Some(&i2) = order.iter().find(|&&j| orient(cdt.pos[i0], cdt.pos[i1], cdt.pos[j]) != 0.0) else {
            cdt.used[i0] = true;
            return Ok(cdt);
        };
        let t = if orient(cdt.pos[i0], cdt.pos[i1], cdt.pos[i2]) > 0.0 { [i0, i1, i2] } else { [i0, i2, i1] };
        cdt.push_tri(t);
        cdt.hull.insert(t[0], t[2]);
        cdt.hull.insert(t[2], t[1]);
        cdt.hull.insert(t[1], t[0]);
        for k in 0..3 {
            cdt.used[t[k]] = true;
        }
        for &i in &order {
            if !cdt.used[i] {
                cdt.insert_index(i)?;
            }
        }
        for [a, b] in edges {
            if a >= n || b >= n {
                return Err(InsertionError);
            }
            if !cdt.insert_constraint(a, b)? {
                on_conflict_found([a, b]);
            }
        }
        cdt.touched.clear();
        Ok(cdt)
    }

    /// The number of vertices (inserted or not: the input's count).
    pub fn num_vertices(&self) -> usize {
        self.verts.len()
    }
    /// The number of inner faces.
    pub fn num_inner_faces(&self) -> usize {
        self.tris.len()
    }
    /// The number of faces, the outer one included.
    pub fn num_all_faces(&self) -> usize {
        self.tris.len() + 1
    }
    /// The number of constraint edges.
    pub fn num_constraints(&self) -> usize {
        self.constraints.len()
    }
    /// The vertex `h`.
    pub fn vertex(&self, h: FixedVertexHandle) -> VertexHandle<'_, V> {
        VertexHandle { cdt: self, i: h.0 }
    }
    /// The vertex `h`'s data, mutably (its position must not change).
    pub fn vertex_data_mut(&mut self, h: FixedVertexHandle) -> &mut V {
        &mut self.verts[h.0]
    }
    /// The vertices.
    pub fn vertices(&self) -> impl Iterator<Item = VertexHandle<'_, V>> + '_ {
        (0..self.verts.len()).map(move |i| VertexHandle { cdt: self, i })
    }
    /// The inner face `h`.
    pub fn face(&self, h: FixedFaceHandle<InnerTag>) -> FaceHandle<'_, InnerTag, V> {
        FaceHandle { cdt: self, t: h.0 - 1, _tag: PhantomData }
    }
    /// The inner faces, by index.
    pub fn inner_faces(&self) -> impl Iterator<Item = FaceHandle<'_, InnerTag, V>> + '_ {
        (0..self.tris.len()).map(move |t| FaceHandle { cdt: self, t, _tag: PhantomData })
    }
    /// The directed edge `e`.
    pub fn directed_edge(&self, e: FixedDirectedEdgeHandle) -> DirectedEdgeHandle<'_, V> {
        DirectedEdgeHandle { cdt: self, from: e.from, to: e.to }
    }
    /// The edge from `a` to `b`, if there is one.
    pub fn get_edge_from_neighbors(
        &self,
        a: FixedVertexHandle,
        b: FixedVertexHandle,
    ) -> Option<DirectedEdgeHandle<'_, V>> {
        (self.edges.contains_key(&(a.0, b.0)) || self.edges.contains_key(&(b.0, a.0)))
            .then_some(DirectedEdgeHandle { cdt: self, from: a.0, to: b.0 })
    }
    /// Whether `a`–`b` is a constraint edge.
    pub fn is_constraint_edge_between(&self, a: FixedVertexHandle, b: FixedVertexHandle) -> bool {
        self.is_constraint(a.0, b.0)
    }

    pub(crate) fn is_constraint(&self, a: usize, b: usize) -> bool {
        self.constraints.contains(&ckey(a, b))
    }

    /// Where `p` is.
    pub fn locate(&self, p: Point2<f64>) -> PositionInTriangulation {
        match self.locate_xy([p.x, p.y]) {
            Loc::None => PositionInTriangulation::NoTriangulation,
            Loc::Vertex(v) => PositionInTriangulation::OnVertex(FixedVertexHandle(v)),
            Loc::Edge(t, k) => {
                let v = self.tris[t];
                PositionInTriangulation::OnEdge(FixedDirectedEdgeHandle { from: v[k], to: v[(k + 1) % 3] })
            }
            Loc::Face(t) => PositionInTriangulation::OnFace(FixedFaceHandle(t + 1, PhantomData)),
            Loc::Outside(a, b) => PositionInTriangulation::OutsideOfConvexHull(FixedDirectedEdgeHandle { from: a, to: b }),
        }
    }

    /// Inserts `v` (a point already there gives that vertex back).
    pub fn insert(&mut self, v: V) -> Result<FixedVertexHandle, InsertionError> {
        let p = v.position();
        if !(p.x.is_finite() && p.y.is_finite()) {
            return Err(InsertionError);
        }
        if let Loc::Vertex(i) = self.locate_xy([p.x, p.y]) {
            return Ok(FixedVertexHandle(i));
        }
        self.verts.push(v);
        self.pos.push([p.x, p.y]);
        self.used.push(false);
        let i = self.verts.len() - 1;
        if self.tris.is_empty() {
            // (Still collinear: kept out until a triangle can be made.)
            return Err(InsertionError);
        }
        self.insert_index(i)?;
        self.touched.clear();
        Ok(FixedVertexHandle(i))
    }

    // ---- core ----

    fn push_tri(&mut self, t: [usize; 3]) -> usize {
        let id = self.tris.len();
        self.tris.push(t);
        self.touched.push(id);
        for k in 0..3 {
            self.edges.insert((t[k], t[(k + 1) % 3]), (id, k as u8));
        }
        id
    }

    fn set_tri(&mut self, id: usize, t: [usize; 3]) {
        let old = self.tris[id];
        for k in 0..3 {
            let e = (old[k], old[(k + 1) % 3]);
            if self.edges.get(&e).is_some_and(|&(x, _)| x == id) {
                self.edges.remove(&e);
            }
        }
        self.tris[id] = t;
        self.touched.push(id);
        for k in 0..3 {
            self.edges.insert((t[k], t[(k + 1) % 3]), (id, k as u8));
        }
    }

    fn locate_xy(&self, p: [f64; 2]) -> Loc {
        if self.tris.is_empty() {
            return Loc::None;
        }
        let mut t = if self.last < self.tris.len() { self.last } else { 0 };
        let budget = 4 * self.tris.len() + 64;
        let mut steps = 0;
        let mut prev = NONE;
        'walk: loop {
            steps += 1;
            if steps > budget {
                return self.locate_brute(p);
            }
            let v = self.tris[t];
            let q = v.map(|i| self.pos[i]);
            // Cross the first side p is strictly beyond (not the one just crossed).
            let start = (t.wrapping_mul(7) + steps) % 3;
            for kk in 0..3 {
                let k = (start + kk) % 3;
                let (a, b) = (v[k], v[(k + 1) % 3]);
                if orient(q[k], q[(k + 1) % 3], p) < 0.0 {
                    match self.edges.get(&(b, a)) {
                        Some(&(n, _)) if n != prev => {
                            prev = t;
                            t = n;
                            continue 'walk;
                        }
                        Some(_) => {}
                        None => return Loc::Outside(a, b),
                    }
                }
            }
            for k in 0..3 {
                if q[k] == p {
                    return Loc::Vertex(v[k]);
                }
            }
            if (0..3).any(|k| orient(q[k], q[(k + 1) % 3], p) < 0.0) {
                // Beyond the side just crossed back: settle it by brute force.
                return self.locate_brute(p);
            }
            for k in 0..3 {
                if orient(q[k], q[(k + 1) % 3], p) == 0.0 {
                    return Loc::Edge(t, k);
                }
            }
            return Loc::Face(t);
        }
    }

    fn locate_brute(&self, p: [f64; 2]) -> Loc {
        for (t, v) in self.tris.iter().enumerate() {
            let q = v.map(|i| self.pos[i]);
            let o = [0, 1, 2].map(|k| orient(q[k], q[(k + 1) % 3], p));
            if o.iter().all(|&x| x >= 0.0) {
                for k in 0..3 {
                    if q[k] == p {
                        return Loc::Vertex(v[k]);
                    }
                }
                for k in 0..3 {
                    if o[k] == 0.0 {
                        return Loc::Edge(t, k);
                    }
                }
                return Loc::Face(t);
            }
        }
        // Outside: a hull edge that sees p.
        for (&a, &b) in &self.hull {
            if orient(self.pos[a], self.pos[b], p) < 0.0 {
                return Loc::Outside(a, b);
            }
        }
        Loc::None
    }

    fn insert_index(&mut self, i: usize) -> Result<(), InsertionError> {
        let p = self.pos[i];
        let mut stack: Vec<(usize, usize)> = Vec::new();
        match self.locate_xy(p) {
            Loc::None => return Err(InsertionError),
            Loc::Vertex(_) => return Ok(()),
            Loc::Face(t) => {
                let [a, b, c] = self.tris[t];
                self.set_tri(t, [a, b, i]);
                let t1 = self.push_tri([b, c, i]);
                let _ = self.push_tri([c, a, i]);
                self.last = t1;
                stack.extend([(a, b), (b, c), (c, a)]);
            }
            Loc::Edge(t, k) => {
                let v = self.tris[t];
                let (a, b, c) = (v[k], v[(k + 1) % 3], v[(k + 2) % 3]);
                let other = self.edges.get(&(b, a)).copied();
                self.set_tri(t, [c, a, i]);
                let _ = self.push_tri([b, c, i]);
                stack.extend([(c, a), (b, c)]);
                match other {
                    Some((u, kk)) => {
                        let w = self.tris[u];
                        let d = w[(kk as usize + 2) % 3];
                        self.set_tri(u, [a, d, i]);
                        let _ = self.push_tri([d, b, i]);
                        stack.extend([(a, d), (d, b)]);
                    }
                    None => {
                        // A hull edge a → b split: the hull runs b → i → a now (it ran b → a).
                        self.hull.insert(b, i);
                        self.hull.insert(i, a);
                    }
                }
                if self.constraints.remove(&ckey(a, b)) {
                    self.constraints.insert(ckey(a, i));
                    self.constraints.insert(ckey(i, b));
                }
                self.last = t;
            }
            Loc::Outside(a0, b0) => {
                // Hull edges u → v run with the outer face on the right; hull[v] is the next.
                // `a0 → b0` here is a triangle's side (counter-clockwise), so the hull edge is
                // b0 → a0. Gather the hull edges that see p, both ways from it.
                let sees = |s: &Self, u: usize, v: usize| orient(s.pos[v], s.pos[u], p) < 0.0;
                let (mut u, mut v) = (b0, a0);
                if self.hull.get(&u) != Some(&v) {
                    return Err(InsertionError);
                }
                // Back: find the first hull edge (u0 → u) that sees p.
                let prev_of: FxHashMap<usize, usize> = self.hull.iter().map(|(&x, &y)| (y, x)).collect();
                let mut guard = 0;
                while let Some(&w) = prev_of.get(&u) {
                    guard += 1;
                    if guard > self.hull.len() || !sees(self, w, u) || w == v {
                        break;
                    }
                    v = u;
                    u = w;
                }
                // Now u → v sees p; go forward collecting.
                let first = u;
                let mut chain = vec![u];
                let mut cur = u;
                let mut guard = 0;
                loop {
                    guard += 1;
                    let Some(&nx) = self.hull.get(&cur) else { return Err(InsertionError) };
                    if guard > self.hull.len() + 1 || !sees(self, cur, nx) {
                        break;
                    }
                    chain.push(nx);
                    cur = nx;
                    if cur == first {
                        return Err(InsertionError);
                    }
                }
                if chain.len() < 2 {
                    return Err(InsertionError);
                }
                for w in chain.windows(2) {
                    let (x, y) = (w[0], w[1]);
                    // Hull edge x → y is triangle side y → x: the new triangle is (x, y, p).
                    let t = self.push_tri([x, y, i]);
                    self.last = t;
                    stack.push((x, y));
                }
                let (s, e) = (chain[0], chain[chain.len() - 1]);
                for w in chain.windows(2) {
                    self.hull.remove(&w[0]);
                }
                self.hull.insert(s, i);
                self.hull.insert(i, e);
            }
        }
        self.used[i] = true;
        self.legalize(i, stack)
    }

    /// Lawson flips for the sides opposite new point `p`, (a, b) with p on the left.
    fn legalize(&mut self, p: usize, mut stack: Vec<(usize, usize)>) -> Result<(), InsertionError> {
        let mut budget = 64 * (self.tris.len() + 16);
        while let Some((a, b)) = stack.pop() {
            budget -= 1;
            if budget == 0 {
                return Err(InsertionError);
            }
            if self.is_constraint(a, b) {
                continue;
            }
            let (Some(&(t1, _)), Some(&(t2, k2))) = (self.edges.get(&(a, b)), self.edges.get(&(b, a))) else {
                continue;
            };
            let d = self.tris[t2][(k2 as usize + 2) % 3];
            let [pa, pb, pp, pd] = [a, b, p, d].map(|x| self.pos[x]);
            if self.tris[t1].contains(&p) && incircle(pa, pb, pp, pd) > 0.0 {
                self.set_tri(t1, [a, d, p]);
                self.set_tri(t2, [d, b, p]);
                stack.push((a, d));
                stack.push((d, b));
            }
        }
        Ok(())
    }

    /// The triangle around `a` the ray toward `b` leaves `a` through, as (t, x, y) with the
    /// triangle (a, x, y); or `Err(Some(c))` when the ray runs along the edge a → c.
    fn first_crossed(&self, a: usize, b: usize) -> Option<Result<(usize, usize, usize), usize>> {
        let (pa, pb) = (self.pos[a], self.pos[b]);
        // Every triangle with corner a.
        for (&(x, y), &(t, _)) in self.edges.iter() {
            if x != a {
                continue;
            }
            let v = self.tris[t];
            let k = v.iter().position(|&q| q == a)?;
            let (c1, c2) = (v[(k + 1) % 3], v[(k + 2) % 3]);
            debug_assert_eq!(c1, y);
            let o1 = orient(pa, pb, self.pos[c1]);
            let o2 = orient(pa, pb, self.pos[c2]);
            // Strictly between a and b along the segment.
            let len2 = (pb[0] - pa[0]).powi(2) + (pb[1] - pa[1]).powi(2);
            let ahead = |c: usize| {
                let pc = self.pos[c];
                let s = (pc[0] - pa[0]) * (pb[0] - pa[0]) + (pc[1] - pa[1]) * (pb[1] - pa[1]);
                s > 0.0 && s < len2
            };
            if o1 == 0.0 && ahead(c1) {
                return Some(Err(c1));
            }
            if o2 == 0.0 && ahead(c2) {
                return Some(Err(c2));
            }
            if o1 < 0.0 && o2 > 0.0 {
                return Some(Ok((t, c1, c2)));
            }
        }
        None
    }

    /// Inserts the constraint a–b; false when it crosses a constraint already in (left out).
    fn insert_constraint(&mut self, a: usize, b: usize) -> Result<bool, InsertionError> {
        self.insert_constraint_at(a, b, 0)
    }

    /// [`Self::insert_constraint`], `depth` pieces deep in a segment split at vertices on it.
    fn insert_constraint_at(&mut self, a: usize, b: usize, depth: usize) -> Result<bool, InsertionError> {
        if depth > 4096 {
            return Err(InsertionError);
        }
        if a == b || !self.used[a] || !self.used[b] {
            return Ok(a == b);
        }
        if self.edges.contains_key(&(a, b)) || self.edges.contains_key(&(b, a)) {
            self.constraints.insert(ckey(a, b));
            return Ok(true);
        }
        let (pa, pb) = (self.pos[a], self.pos[b]);
        // The edges a–b crosses, walking from a.
        let mut crossed: Vec<(usize, usize)> = Vec::new();
        let (mut t, mut x, mut y) = match self.first_crossed(a, b) {
            None => return Err(InsertionError),
            Some(Err(c)) => {
                // Through vertex c: a–c, then c–b.
                return Ok(self.insert_constraint_at(a, c, depth + 1)?
                    & self.insert_constraint_at(c, b, depth + 1)?);
            }
            Some(Ok(f)) => f,
        };
        let mut guard = 0;
        loop {
            guard += 1;
            if guard > self.tris.len() + 8 {
                return Err(InsertionError);
            }
            // The ray leaves triangle t through side x → y (seen from a: x right, y left).
            if self.is_constraint(x, y) {
                return Ok(false);
            }
            crossed.push((x, y));
            let Some(&(n, k)) = self.edges.get(&(y, x)) else {
                return Err(InsertionError);
            };
            let _ = t;
            t = n;
            let w = self.tris[n][(k as usize + 2) % 3];
            if w == b {
                break;
            }
            let ow = orient(pa, pb, self.pos[w]);
            if ow == 0.0 {
                // Through vertex w on the segment: a–w then w–b.
                let ok = self.recover(a, w, crossed)?;
                return Ok(ok & self.insert_constraint_at(w, b, depth + 1)?);
            }
            if ow < 0.0 {
                x = w;
            } else {
                y = w;
            }
        }
        self.recover(a, b, crossed)
    }

    /// Flips away the edges `crossed` that cross a–b (Sloan), then restores Delaunay around.
    fn recover(&mut self, a: usize, b: usize, crossed: Vec<(usize, usize)>) -> Result<bool, InsertionError> {
        let (pa, pb) = (self.pos[a], self.pos[b]);
        let crosses = |s: &Self, u: usize, v: usize| {
            if u == a || u == b || v == a || v == b {
                return false;
            }
            let (pu, pv) = (s.pos[u], s.pos[v]);
            let o1 = orient(pa, pb, pu);
            let o2 = orient(pa, pb, pv);
            let o3 = orient(pu, pv, pa);
            let o4 = orient(pu, pv, pb);
            o1 * o2 < 0.0 && o3 * o4 < 0.0
        };
        let mut queue: std::collections::VecDeque<(usize, usize)> = crossed.into_iter().collect();
        let mut fresh: Vec<(usize, usize)> = Vec::new();
        let mut budget = 64 * (queue.len() + 4) * (queue.len() + 4);
        while let Some((u, v)) = queue.pop_front() {
            budget -= 1;
            if budget == 0 {
                return Err(InsertionError);
            }
            let (Some(&(t1, k1)), Some(&(t2, k2))) = (self.edges.get(&(u, v)), self.edges.get(&(v, u))) else {
                continue;
            };
            let c = self.tris[t1][(k1 as usize + 2) % 3];
            let d = self.tris[t2][(k2 as usize + 2) % 3];
            // The quad u, d, v, c is convex when c and d are on both sides of u–v and u, v on
            // both sides of c–d.
            let (pu, pv, pc, pd) = (self.pos[u], self.pos[v], self.pos[c], self.pos[d]);
            let convex = orient(pc, pd, pu) * orient(pc, pd, pv) < 0.0
                && orient(pu, pv, pc) * orient(pu, pv, pd) < 0.0;
            if !convex {
                queue.push_back((u, v));
                continue;
            }
            // Flip u–v to c–d: triangles (u, d, c) and (d, v, c).
            self.set_tri(t1, [u, d, c]);
            self.set_tri(t2, [d, v, c]);
            if crosses(self, c, d) {
                queue.push_back((c, d));
            } else {
                fresh.push((c, d));
            }
        }
        self.constraints.insert(ckey(a, b));
        // Delaunay again across the new edges (not across a–b).
        let mut changed = true;
        let mut rounds = 0;
        while changed && rounds < 64 {
            changed = false;
            rounds += 1;
            for k in 0..fresh.len() {
                let (u, v) = fresh[k];
                if self.is_constraint(u, v) {
                    continue;
                }
                let (Some(&(t1, k1)), Some(&(t2, k2))) = (self.edges.get(&(u, v)), self.edges.get(&(v, u))) else {
                    continue;
                };
                let c = self.tris[t1][(k1 as usize + 2) % 3];
                let d = self.tris[t2][(k2 as usize + 2) % 3];
                let (pu, pv, pc, pd) = (self.pos[u], self.pos[v], self.pos[c], self.pos[d]);
                if incircle(pu, pv, pc, pd) > 0.0
                    && orient(pc, pd, pu) * orient(pc, pd, pv) < 0.0
                {
                    self.set_tri(t1, [u, d, c]);
                    self.set_tri(t2, [d, v, c]);
                    fresh[k] = (c, d);
                    changed = true;
                }
            }
        }
        Ok(true)
    }

    // ---- refinement ----

    /// Which triangles are outside the constraint loops, by parity from the hull.
    fn outside_faces(&self) -> Vec<bool> {
        let n = self.tris.len();
        let mut state: Vec<i8> = vec![-1; n];
        let mut queue = std::collections::VecDeque::new();
        for (t, v) in self.tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (v[k], v[(k + 1) % 3]);
                if !self.edges.contains_key(&(b, a)) && state[t] == -1 {
                    state[t] = i8::from(self.is_constraint(a, b));
                    queue.push_back(t);
                }
            }
        }
        while let Some(t) = queue.pop_front() {
            let v = self.tris[t];
            for k in 0..3 {
                let (a, b) = (v[k], v[(k + 1) % 3]);
                if let Some(&(o, _)) = self.edges.get(&(b, a))
                    && state[o] == -1
                {
                    state[o] = if self.is_constraint(a, b) { 1 - state[t] } else { state[t] };
                    queue.push_back(o);
                }
            }
        }
        state.into_iter().map(|s| s != 1).collect()
    }

    /// Refines the triangulation: circumcentres of the triangles under the angle limit (or over
    /// the area bound) added, as `params` allow.
    pub fn refine(&mut self, params: RefinementParameters<f64>) -> RefinementResult
    where
        V: From<Point2<f64>>,
    {
        let mut excluded = if params.exclude_outer { self.outside_faces() } else { vec![false; self.tris.len()] };
        let sin_min = params.angle.rad.sin();
        let mut added = 0usize;
        let mut complete = true;
        let mut queue: Vec<usize> = (0..self.tris.len()).rev().collect();
        let mut tried: std::collections::HashSet<[usize; 3], crate::fx::FxBuildHasher> = Default::default();
        while let Some(t) = queue.pop() {
            if t >= self.tris.len() || excluded.get(t).copied().unwrap_or(true) {
                continue;
            }
            let v = self.tris[t];
            let [a, b, c] = v.map(|i| self.pos[i]);
            let area = 0.5 * orient(a, b, c);
            if !(area > 0.0) {
                continue;
            }
            if params.min_area.is_some_and(|m| area < m) {
                continue;
            }
            let la = ((b[0] - c[0]).powi(2) + (b[1] - c[1]).powi(2)).sqrt();
            let lb = ((a[0] - c[0]).powi(2) + (a[1] - c[1]).powi(2)).sqrt();
            let lc = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
            // The smallest angle's sine: 2 area / (product of its two sides), opposite the
            // shortest side.
            let shortest = la.min(lb).min(lc);
            let others = la * lb * lc / shortest;
            let sin_small = if others > 0.0 { 2.0 * area / others } else { 0.0 };
            let too_big = params.max_area.is_some_and(|m| area > m);
            if !(sin_small < sin_min || too_big) {
                continue;
            }
            let mut key = v;
            key.sort_unstable();
            if !tried.insert(key) {
                continue;
            }
            if added >= params.max_added {
                complete = false;
                break;
            }
            // The circumcentre.
            let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
            if d == 0.0 {
                continue;
            }
            let (a2, b2, c2) = (a[0] * a[0] + a[1] * a[1], b[0] * b[0] + b[1] * b[1], c[0] * c[0] + c[1] * c[1]);
            let cc = [
                (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d,
                (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d,
            ];
            if !(cc[0].is_finite() && cc[1].is_finite()) {
                continue;
            }
            // Where it lands: an included face (or an interior side), clear of every
            // constraint's diametral circle.
            let land = match self.locate_xy(cc) {
                Loc::Face(f) => Some(f),
                Loc::Edge(f, k) => {
                    let w = self.tris[f];
                    if self.is_constraint(w[k], w[(k + 1) % 3]) { None } else { Some(f) }
                }
                _ => None,
            };
            let Some(f) = land else { continue };
            if excluded.get(f).copied().unwrap_or(true) || self.encroaches(f, cc) {
                continue;
            }
            // Insert it as a fresh vertex made from its position.
            let n0 = self.tris.len();
            let i = self.verts.len();
            self.pos.push(cc);
            self.used.push(false);
            self.verts.push(V::from(Point2::new(cc[0], cc[1])));
            self.touched.clear();
            if self.insert_index(i).is_err() {
                complete = false;
                break;
            }
            added += 1;
            // New triangles are inside (the cavity never crosses a constraint).
            excluded.resize(self.tris.len(), false);
            for nt in n0..self.tris.len() {
                excluded[nt] = false;
            }
            // Recheck the triangles made or changed.
            queue.extend(self.touched.drain(..));
        }
        let excluded_faces = excluded
            .iter()
            .enumerate()
            .filter(|&(_, &e)| e)
            .map(|(t, _)| FixedFaceHandle(t + 1, PhantomData))
            .collect();
        RefinementResult { excluded_faces, refinement_complete: complete }
    }

    /// Whether `p` lies inside the diametral circle of a constraint side near face `f`.
    fn encroaches(&self, f: usize, p: [f64; 2]) -> bool {
        // The constraint sides of the faces within two steps of f.
        let mut seen: Vec<usize> = vec![f];
        let mut ring = vec![f];
        for _ in 0..2 {
            let mut next = Vec::new();
            for &t in &ring {
                let v = self.tris[t];
                for k in 0..3 {
                    if let Some(&(o, _)) = self.edges.get(&(v[(k + 1) % 3], v[k]))
                        && !seen.contains(&o)
                    {
                        seen.push(o);
                        next.push(o);
                    }
                }
            }
            ring = next;
        }
        seen.iter().any(|&t| {
            let v = self.tris[t];
            (0..3).any(|k| {
                let (a, b) = (v[k], v[(k + 1) % 3]);
                if !self.is_constraint(a, b) {
                    return false;
                }
                let (pa, pb) = (self.pos[a], self.pos[b]);
                // Inside the circle on a–b as diameter: (p − a)·(p − b) < 0.
                (p[0] - pa[0]) * (p[0] - pb[0]) + (p[1] - pa[1]) * (p[1] - pb[1]) < 0.0
            })
        })
    }
}

/// Where a point is (internal).
enum Loc {
    None,
    Vertex(usize),
    Edge(usize, usize),
    Face(usize),
    Outside(usize, usize),
}
