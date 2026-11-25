// yack: The topology arena: solids of shells of faces bounded by loops of coedges, each the use of an
// Adapted from springrep@a03e8a1 src/topo/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/topo/mod.rs to yack-topo (M3 W4); `.expect`/`.unwrap` as counted `panic!`s (gate 9);
// M3 hardening: every look-up by id goes through `entity`/`entity_mut`, the arena's one indexing
// contract; a coedge's dense samples come from walks::densified; no other indexing; faces found
// apart are marked on their memos through the arena.
//! The topology arena: solids of shells of faces bounded by loops of coedges, each the use of an
//! edge by one face. Every entity lives in one [`Brep`] and is referred to by a typed id.

// springrep's foundation modules (D26), under the names topo's code uses.
use yack_base::kernel::Error;
use yack_base::kernel::Result;
use yack_geom as geom;
use yack_math::kernel as math;
use yack_numeric as numeric;
use yack_tol as tol;

pub mod names;
pub mod bbox;
mod facts;
pub mod domain;
pub mod history;
mod memo;
mod parts;
pub mod pcurve;
pub mod radial;
mod similar;
mod trims;
mod walks;

use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::sync::Arc;

pub use facts::{Facts, OnFace};
pub use memo::{FaceMemo, FaceVolume, IntegralKey, MemoSlot};
pub use crate::names::{EdgeName, FaceName};
pub use parts::{Entities, Parts};
pub use pcurve::PCurve;
pub use trims::{coedge_trim_with, coedge_uv_at, face_domain};
pub use walks::{coedge_params, fans_at_vertices};
pub use pcurve::project_on;

use walks::densified;

use crate::{
    geom::{Curve, Surface},
    math::{Point3, Transform, Vec2},
};

macro_rules! id {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $name(u32);

        impl $name {
            pub fn index(self) -> usize {
                self.0 as usize
            }

            #[allow(clippy::panic)] fn at(i: usize) -> $name {
                $name(u32::try_from(i).unwrap_or_else(|e| panic!("arena index fits in u32: {e:?}")))
            }

            /// The id as the encoder writes it.
            pub fn raw(self) -> u32 {
                self.0
            }

            /// The id the encoder wrote as `raw`; the arena checks it names one of its entities.
            pub fn from_raw(raw: u32) -> $name {
                $name(raw)
            }
        }
    };
}

id!(VertexId);
id!(EdgeId);
id!(CoedgeId);
id!(LoopId);
id!(FaceId);
id!(ShellId);
id!(SolidId);

/// The entity at `index` in one of an arena's lists. Every id is minted by the arena whose entity
/// it names, and [`Brep::from_parts`] checks the ids it is handed, so an index past the end is a
/// caller's bug (an id of another arena): the arena's one look-up that panics.
#[allow(clippy::indexing_slicing)]
fn entity<T>(items: &[T], index: usize) -> &T {
    &items[index]
}

/// [`entity`], mutably.
#[allow(clippy::indexing_slicing)]
fn entity_mut<T>(items: &mut [T], index: usize) -> &mut T {
    &mut items[index]
}

#[derive(Clone, Debug, PartialEq)]
pub struct Vertex {
    pub point: Point3,
}

/// A bounded piece of a curve from `curve(range.0)` at `start` to `curve(range.1)` at `end`,
/// `range.0 < range.1`. A degenerate edge has no curve: it is a pole, a single vertex that its face
/// spans a stretch of parameter plane at, and `range` is the parameter its pcurve runs over.
#[derive(Clone, Debug, PartialEq)]
pub struct Edge {
    pub curve: Option<Curve>,
    pub range: (f64, f64),
    pub start: VertexId,
    pub end: VertexId,
}

impl Edge {
    pub fn is_degenerate(&self) -> bool {
        self.curve.is_none()
    }
}

/// One face's use of an edge, along the edge's parameter or against it when `reversed`.
#[derive(Clone, Debug, PartialEq)]
pub struct Coedge {
    pub edge: EdgeId,
    pub reversed: bool,
    pub pcurve: PCurve,
    pub lp: LoopId,
}

/// A closed chain of coedges with the face's material on its left, seen from outside the solid.
#[derive(Clone, Debug, PartialEq)]
pub struct Loop {
    pub coedges: Vec<CoedgeId>,
    pub face: FaceId,
}

/// A bounded piece of a surface. Its outward normal is the surface's natural normal, or its
/// opposite when `reversed`. The first loop is the outer one.
#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    pub surface: Surface,
    pub reversed: bool,
    pub loops: Vec<LoopId>,
    pub names: BTreeSet<FaceName>,
    pub shell: Option<ShellId>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Shell {
    pub faces: Vec<FaceId>,
    pub solid: Option<SolidId>,
}

/// A solid: one connected piece of material. Its first `outer` shells bound it from outside, any
/// others bound voids. Each shell is a sheet: faces joined edge to edge through each coedge's
/// partner. A solid has several outer sheets where pieces touch along an edge or at a vertex they
/// share, which is the solid's own: an edge used by four faces, a vertex two fans meet at.
#[derive(Clone, Debug, PartialEq)]
pub struct Solid {
    pub shells: Vec<ShellId>,
    pub outer: usize,
}

/// What `Brep::add_loop` needs for each coedge.
#[derive(Clone, Debug, PartialEq)]
pub struct CoedgeSpec {
    pub edge: EdgeId,
    pub reversed: bool,
    pub pcurve: PCurve,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Brep {
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    coedges: Vec<Coedge>,
    loops: Vec<Loop>,
    faces: Vec<Face>,
    shells: Vec<Shell>,
    solids: Vec<Solid>,
    memos: memo::Memos,
    /// Local tolerances above [`tol::LINEAR`], which only an import's edges and vertices carry.
    tolerances: Box<Tolerances>,
    /// Each coedge of an edge more than two faces use, with the coedge across the wedge of
    /// material it bounds. An edge two faces use pairs its two coedges without an entry.
    partners: BTreeMap<CoedgeId, CoedgeId>,
    /// Each coedge of such an edge with the coedge across the empty wedge next to its face, round
    /// the edge the other way from its partner.
    beside: BTreeMap<CoedgeId, CoedgeId>,
    /// Each edge's coedges, in the order they were added: worked out from the coedges, never
    /// encoded.
    uses: Vec<Vec<CoedgeId>>,
}

/// The steps along `curve` from `t0` to `t1` that can never skip a turn of it round `surface`,
/// whose periodic parameters it continues from `seed`, given that `gaps` steps are asked for.
fn turn_steps(
    surface: &Surface,
    curve: &Curve,
    seed: Vec2,
    (t0, t1): (f64, f64),
    gaps: usize,
) -> usize {
    // A turn is at least the surface's circumference, so steps an eighth of that along the
    // curve can never skip one; finer samples cannot tell, as their remainders alias alike.
    let mut length = 0.0;
    let mut prev = curve.eval(t0);
    for k in 1..=256 {
        let p = curve.eval(t0 + (t1 - t0) * k as f64 / 256.0);
        length += p.dist(prev);
        prev = p;
    }
    let (_, su, sv) = surface.d1(seed.x, seed.y);
    let around = [(surface.u_period(), su), (surface.v_period(), sv)]
        .iter()
        .filter_map(|&(p, d)| p.map(|p| p * d.norm()))
        .filter(|&c| c > 1e-9)
        .fold(f64::INFINITY, f64::min);
    let steps = if around.is_finite() { (8.0 * length / around).ceil() as usize } else { 0 };
    if steps < gaps {
        return steps;
    }
    // That counts the turns a curve of this length could make. Where it asks for steps, how fast
    // the curve does turn says how many it needs: an edge along a cylinder's axis turns not at
    // all, however thin the cylinder, and one a tolerance thick would otherwise take millions.
    let n = 32;
    let mut turns = 0.0;
    for k in 0..n {
        let (p, dp) = curve.d1(t0 + (t1 - t0) * (k as f64 + 0.5) / n as f64);
        let uv = surface.project(p);
        let (_, su, sv) = surface.d1(uv.x, uv.y);
        let rate = [(surface.u_period(), su), (surface.v_period(), sv)]
            .iter()
            .filter_map(|&(per, d)| per.map(|per| (per * d.norm(), d)))
            .filter(|&(c, _)| c > 1e-9)
            .map(|(c, d)| dp.dot(d).abs() / (d.norm() * c))
            .fold(0.0, f64::max);
        turns += rate * (t1 - t0).abs() / n as f64;
    }
    // Sixteen steps a turn, twice what the bound above takes, for the rate between samples.
    let needed = (16.0 * turns).ceil();
    if needed.is_finite() { steps.min(needed as usize) } else { steps }
}

/// Each vertex and each edge with a local tolerance, with it.
pub type Tolerant = (Vec<(VertexId, f64)>, Vec<(EdgeId, f64)>);

/// The vertices and edges of imported geometry that meet only to their source's tolerance
/// (doc/design.md, "Tolerance policy"). A vertex's sphere holds the tube of every edge ending at
/// it, and two entities coincide within the sum of their tolerances.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Tolerances {
    pub vertices: BTreeMap<VertexId, f64>,
    pub edges: BTreeMap<EdgeId, f64>,
}

impl Brep {
    pub fn new() -> Brep {
        Brep::default()
    }

    pub fn add_vertex(&mut self, point: Point3) -> VertexId {
        let id = VertexId::at(self.vertices.len());
        self.vertices.push(Vertex { point });
        id
    }

    pub fn add_edge(
        &mut self,
        curve: Curve,
        range: (f64, f64),
        start: VertexId,
        end: VertexId,
    ) -> EdgeId {
        self.push_edge(Edge { curve: Some(curve), range, start, end })
    }

    pub fn add_degenerate_edge(&mut self, vertex: VertexId, range: (f64, f64)) -> EdgeId {
        self.push_edge(Edge { curve: None, range, start: vertex, end: vertex })
    }

    /// Adds `edge`, used by no coedge yet.
    fn push_edge(&mut self, edge: Edge) -> EdgeId {
        let id = EdgeId::at(self.edges.len());
        self.edges.push(edge);
        self.uses.push(Vec::new());
        id
    }

    pub fn add_face(
        &mut self,
        surface: Surface,
        reversed: bool,
        names: impl IntoIterator<Item = FaceName>,
    ) -> FaceId {
        let id = FaceId::at(self.faces.len());
        self.faces.push(Face {
            surface,
            reversed,
            loops: Vec::new(),
            names: names.into_iter().collect(),
            shell: None,
        });
        self.memos.0.push(Arc::default());
        id
    }

    pub fn add_loop(&mut self, face: FaceId, coedges: Vec<CoedgeSpec>) -> LoopId {
        let lp = LoopId::at(self.loops.len());
        let mut ids = Vec::with_capacity(coedges.len());
        for CoedgeSpec { edge, reversed, pcurve } in coedges {
            let id = CoedgeId::at(self.coedges.len());
            self.coedges.push(Coedge { edge, reversed, pcurve, lp });
            entity_mut(&mut self.uses, edge.index()).push(id);
            ids.push(id);
        }
        self.loops.push(Loop { coedges: ids, face });
        entity_mut(&mut self.faces, face.index()).loops.push(lp);
        self.forget(face);
        lp
    }

    pub fn add_shell(&mut self, faces: Vec<FaceId>) -> ShellId {
        let id = ShellId::at(self.shells.len());
        for &f in &faces {
            entity_mut(&mut self.faces, f.index()).shell = Some(id);
        }
        self.shells.push(Shell { faces, solid: None });
        id
    }

    /// A solid of one outer shell, the first, and voids.
    pub fn add_solid(&mut self, shells: Vec<ShellId>) -> SolidId {
        self.add_solid_of(shells, 1)
    }

    /// A solid whose first `outer` shells bound it from outside and the rest voids.
    pub fn add_solid_of(&mut self, shells: Vec<ShellId>, outer: usize) -> SolidId {
        let id = SolidId::at(self.solids.len());
        for &s in &shells {
            entity_mut(&mut self.shells, s.index()).solid = Some(id);
        }
        self.solids.push(Solid { shells, outer });
        id
    }

    /// Pairs coedges `a` and `b` of one edge across the wedge of material between their faces: for
    /// an edge more than two faces use.
    pub fn pair(&mut self, a: CoedgeId, b: CoedgeId) {
        self.partners.insert(a, b);
        self.partners.insert(b, a);
    }

    /// Sets coedges `a` and `b` of one edge either side of an empty wedge between their faces: for
    /// an edge more than two faces use.
    pub fn set_beside(&mut self, a: CoedgeId, b: CoedgeId) {
        self.beside.insert(a, b);
        self.beside.insert(b, a);
    }

    /// The coedge across the empty wedge next to coedge `c`'s face at its edge: on an edge two
    /// faces use, its partner, the two faces bounding the empty space outside them both; on one
    /// more use, the neighbour the other way round from its partner.
    pub fn beside(&self, c: CoedgeId) -> Option<CoedgeId> {
        self.across(c, &self.beside)
    }

    /// The coedges using edge `e`.
    pub fn edge_uses(&self, e: EdgeId) -> &[CoedgeId] {
        entity(&self.uses, e.index())
    }

    /// The coedge across coedge `c`'s edge from it: the edge's other use where two faces use it,
    /// the one `more` names where more do, none where fewer do.
    fn across(&self, c: CoedgeId, more: &BTreeMap<CoedgeId, CoedgeId>) -> Option<CoedgeId> {
        let uses = self.edge_uses(self.coedge(c).edge);
        match uses.len() {
            2 => uses.iter().copied().find(|&other| other != c),
            n if n > 2 => more.get(&c).copied(),
            _ => None,
        }
    }

    /// The coedge across the wedge of material coedge `c`'s face bounds at its edge: the edge's
    /// other use where two faces use it, the paired one where more do. `None` on a pole, and on an
    /// edge of more uses left unpaired.
    pub fn partner(&self, c: CoedgeId) -> Option<CoedgeId> {
        self.across(c, &self.partners)
    }

    /// The edges more than two faces use: where pieces of a solid touch.
    pub fn radial_edges(&self) -> impl Iterator<Item = EdgeId> + '_ {
        self.edge_ids().filter(|&e| self.edge_uses(e).len() > 2)
    }

    /// Pairs the coedges of every edge more than two faces use that are not paired yet, from the
    /// faces' geometry round it: for an arena built from another one's faces.
    pub fn pair_radial(&mut self) {
        let edges: Vec<EdgeId> = self.radial_edges().collect();
        for e in edges {
            let uses = self.edge_uses(e).to_vec();
            if uses.iter().all(|c| self.partners.contains_key(c)) {
                continue;
            }
            let Some((pairs, beside)) = self.radial_order(e, &uses) else { continue };
            let use_of = |i: usize| uses.get(i).copied();
            for (i, j) in pairs {
                if let (Some(a), Some(b)) = (use_of(i), use_of(j)) {
                    self.pair(a, b);
                }
            }
            for (i, j) in beside {
                if let (Some(a), Some(b)) = (use_of(i), use_of(j)) {
                    self.set_beside(a, b);
                }
            }
        }
    }

    /// The partner pairs and the pairs beside each other round edge `e`, from its faces' geometry
    /// at its middle, as indices into its `uses`; `None` for a pole, an edge with no direction
    /// there, or faces that do not alternate round it.
    fn radial_order(&self, e: EdgeId, uses: &[CoedgeId]) -> Option<(radial::Pairs, radial::Pairs)> {
        let edge = self.edge(e);
        let curve = edge.curve.as_ref()?;
        let (middle, d) = curve.d1(0.5 * (edge.range.0 + edge.range.1));
        let t = d.unit()?;
        let len = radial::length(curve, edge.range);
        let sides: Vec<radial::Side> = uses
            .iter()
            .map(|&c| {
                let face = self.face(self.coedge_face(c));
                let travel = if self.coedge(c).reversed { -t } else { t };
                radial::Side { surface: &face.surface, reversed: face.reversed, travel }
            })
            .collect();
        radial::round(middle, t, len, &sides)
    }

    /// Every edge's coedges, worked out again from the coedges.
    pub(crate) fn index_uses(&mut self) {
        self.uses = vec![Vec::new(); self.edges.len()];
        for (i, c) in self.coedges.iter().enumerate() {
            entity_mut(&mut self.uses, c.edge.index()).push(CoedgeId::at(i));
        }
    }

    pub fn vertex(&self, id: VertexId) -> &Vertex {
        entity(&self.vertices, id.index())
    }

    pub fn edge(&self, id: EdgeId) -> &Edge {
        entity(&self.edges, id.index())
    }

    pub fn coedge(&self, id: CoedgeId) -> &Coedge {
        entity(&self.coedges, id.index())
    }

    pub fn lp(&self, id: LoopId) -> &Loop {
        entity(&self.loops, id.index())
    }

    /// Whether loop `id` is a single point inside its face: one coedge of a degenerate edge, where
    /// the solid touches itself there or another piece of it touches the face.
    pub fn is_vertex_loop(&self, id: LoopId) -> bool {
        match self.lp(id).coedges.as_slice() {
            &[only] => self.edge(self.coedge(only).edge).is_degenerate(),
            _ => false,
        }
    }

    pub fn face(&self, id: FaceId) -> &Face {
        entity(&self.faces, id.index())
    }

    pub fn face_mut(&mut self, id: FaceId) -> &mut Face {
        self.forget(id);
        entity_mut(&mut self.faces, id.index())
    }

    /// Drops what has been worked out about face `id`'s geometry.
    fn forget(&mut self, id: FaceId) {
        *entity_mut(&mut self.memos.0, id.index()) = Arc::default();
    }

    /// Replaces every name of every face by what `names_for` gives for it: no names to drop it,
    /// itself to keep it. Nothing else about the solid changes.
    pub fn rename_faces<I>(&mut self, mut names_for: impl FnMut(&FaceName) -> I)
    where
        I: IntoIterator<Item = FaceName>,
    {
        for face in &mut self.faces {
            face.names = face.names.iter().flat_map(&mut names_for).collect();
        }
    }

    /// Renames a face, keeping what has been worked out about its geometry.
    pub fn set_face_names(&mut self, id: FaceId, names: BTreeSet<FaceName>) {
        entity_mut(&mut self.faces, id.index()).names = names;
    }

    pub fn memo(&self, id: FaceId) -> &FaceMemo {
        entity::<Arc<FaceMemo>>(&self.memos.0, id.index())
    }

    pub fn shared_memo(&self, id: FaceId) -> Arc<FaceMemo> {
        #[cfg(debug_assertions)]
        self.memo(id).layout.get_or_init(|| self.layout(id));
        Arc::clone(entity(&self.memos.0, id.index()))
    }

    /// Gives face `id` the memo of a face with exactly its geometry, its loops and coedges in the
    /// same order on edges of the same ranges: the memo keeps some of what it holds by a
    /// coedge's place, and an edge's parameters with it.
    pub fn share_memo(&mut self, id: FaceId, memo: Arc<FaceMemo>) {
        *entity_mut(&mut self.memos.0, id.index()) = memo;
        #[cfg(debug_assertions)]
        if let Some(was) = self.memo(id).layout.get() {
            debug_assert!(
                *was == self.layout(id),
                "face {} takes the memo of a face laid out another way",
                id.index()
            );
        }
    }

    /// Records that faces `a` and `b` meet nowhere but at the edges and vertices they share, on
    /// their memos: it holds for as long as both faces keep their geometry.
    pub fn mark_apart(&self, a: FaceId, b: FaceId) {
        let (memo_a, memo_b) = (entity(&self.memos.0, a.index()), entity(&self.memos.0, b.index()));
        memo_a.note_apart(memo_b);
        memo_b.note_apart(memo_a);
    }

    /// Whether faces `a` and `b`, or faces of the same geometry, were marked apart.
    pub fn known_apart(&self, a: FaceId, b: FaceId) -> bool {
        self.memo(a).is_apart(self.memo(b))
    }

    #[cfg(debug_assertions)]
    fn layout(&self, id: FaceId) -> Vec<Vec<(u64, u64, bool)>> {
        let of = |&c: &CoedgeId| {
            let co = self.coedge(c);
            let (t0, t1) = self.edge(co.edge).range;
            (t0.to_bits(), t1.to_bits(), co.reversed)
        };
        let coedges = |&lp: &LoopId| self.lp(lp).coedges.iter().map(of).collect();
        self.face(id).loops.iter().map(coedges).collect()
    }

    pub fn shell(&self, id: ShellId) -> &Shell {
        entity(&self.shells, id.index())
    }

    pub fn solid(&self, id: SolidId) -> &Solid {
        entity(&self.solids, id.index())
    }

    pub fn vertex_ids(&self) -> impl Iterator<Item = VertexId> + use<> {
        (0..self.vertices.len()).map(VertexId::at)
    }

    pub fn edge_ids(&self) -> impl Iterator<Item = EdgeId> + use<> {
        (0..self.edges.len()).map(EdgeId::at)
    }

    pub fn coedge_ids(&self) -> impl Iterator<Item = CoedgeId> + use<> {
        (0..self.coedges.len()).map(CoedgeId::at)
    }

    pub fn loop_ids(&self) -> impl Iterator<Item = LoopId> + use<> {
        (0..self.loops.len()).map(LoopId::at)
    }

    pub fn face_ids(&self) -> impl Iterator<Item = FaceId> + use<> {
        (0..self.faces.len()).map(FaceId::at)
    }

    pub fn shell_ids(&self) -> impl Iterator<Item = ShellId> + use<> {
        (0..self.shells.len()).map(ShellId::at)
    }

    pub fn solid_ids(&self) -> impl Iterator<Item = SolidId> + use<> {
        (0..self.solids.len()).map(SolidId::at)
    }

    /// The radius within which vertex `v` is its point: [`tol::LINEAR`] unless it was imported
    /// with a local tolerance.
    pub fn vertex_tolerance(&self, v: VertexId) -> f64 {
        self.tolerances
            .vertices
            .get(&v)
            .copied()
            .unwrap_or(tol::LINEAR)
    }

    /// The radius of the tube round edge `e`'s curve within which its faces' surfaces pass.
    pub fn edge_tolerance(&self, e: EdgeId) -> f64 {
        self.tolerances
            .edges
            .get(&e)
            .copied()
            .unwrap_or(tol::LINEAR)
    }

    /// Gives an imported vertex a tolerance; a value at or below [`tol::LINEAR`] clears it.
    pub fn set_vertex_tolerance(&mut self, v: VertexId, t: f64) {
        let old = self.vertex_tolerance(v);
        if t > tol::LINEAR {
            self.tolerances.vertices.insert(v, t);
        } else {
            self.tolerances.vertices.remove(&v);
        }
        if self.vertex_tolerance(v) != old {
            self.forget_faces_of(|b, e| b.edge(e).start == v || b.edge(e).end == v);
        }
    }

    /// Gives an imported edge a tolerance; a value at or below [`tol::LINEAR`] clears it.
    pub fn set_edge_tolerance(&mut self, e: EdgeId, t: f64) {
        let old = self.edge_tolerance(e);
        if t > tol::LINEAR {
            self.tolerances.edges.insert(e, t);
        } else {
            self.tolerances.edges.remove(&e);
        }
        if self.edge_tolerance(e) != old {
            self.forget_faces_of(|_, x| x == e);
        }
    }

    /// Drops what has been worked out about the faces with an edge `uses` picks: a tolerance
    /// changes whether their geometry is sound.
    fn forget_faces_of(&mut self, uses: impl Fn(&Brep, EdgeId) -> bool) {
        let faces: Vec<FaceId> = self
            .coedges
            .iter()
            .filter(|c| uses(self, c.edge))
            .map(|c| self.lp(c.lp).face)
            .collect();
        for f in faces {
            self.forget(f);
        }
    }

    /// The vertices and edges with a tolerance above [`tol::LINEAR`], each with it.
    pub fn tolerant(&self) -> Tolerant {
        (
            self.tolerances
                .vertices
                .iter()
                .map(|(&v, &t)| (v, t))
                .collect(),
            self.tolerances
                .edges
                .iter()
                .map(|(&e, &t)| (e, t))
                .collect(),
        )
    }

    /// Whether any vertex or edge carries a tolerance above [`tol::LINEAR`].
    pub fn has_tolerances(&self) -> bool {
        !self.tolerances.vertices.is_empty() || !self.tolerances.edges.is_empty()
    }

    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// The faces of a solid, over all its shells.
    pub fn solid_faces(&self, s: SolidId) -> Vec<FaceId> {
        self.solid(s)
            .shells
            .iter()
            .flat_map(|&sh| self.shell(sh).faces.iter().copied())
            .collect()
    }

    /// The vertex a coedge starts at, in its own direction.
    pub fn coedge_start(&self, c: CoedgeId) -> VertexId {
        let co = self.coedge(c);
        let e = self.edge(co.edge);
        if co.reversed { e.end } else { e.start }
    }

    pub fn coedge_end(&self, c: CoedgeId) -> VertexId {
        let co = self.coedge(c);
        let e = self.edge(co.edge);
        if co.reversed { e.start } else { e.end }
    }

    pub fn coedge_face(&self, c: CoedgeId) -> FaceId {
        self.lp(self.coedge(c).lp).face
    }

    /// The coedge's parameter-plane points at edge parameters `ts`, given in the coedge's own
    /// direction (descending when it is reversed).
    pub fn coedge_uvs(&self, c: CoedgeId, ts: &[f64]) -> Vec<Vec2> {
        let coedge = self.coedge(c);
        let surface = &self.face(self.coedge_face(c)).surface;
        let periodic = surface.u_period().is_some() || surface.v_period().is_some();
        if !periodic
            || ts.len() < 2
            || matches!(coedge.pcurve, PCurve::Affine { .. } | PCurve::Fitted { .. })
        {
            return self.coedge_uvs_at(c, ts);
        }
        let (Some(curve), PCurve::Projected { seed }, &[t0, .., t1]) =
            (&self.edge(coedge.edge).curve, &coedge.pcurve, ts)
        else {
            return self.coedge_uvs_at(c, ts);
        };
        // An edge winding round its surface several times (a thread's helix) can turn more than
        // a period between the points asked for, and continuation then reads a turn as none.
        let steps = turn_steps(surface, curve, *seed, (t0, t1), ts.len() - 1);
        let per = (steps / (ts.len() - 1) + 1).min(4096);
        if per <= 1 {
            return self.coedge_uvs_at(c, ts);
        }
        let uvs = self.coedge_uvs_at(c, &densified(ts, per));
        uvs.into_iter().step_by(per).collect()
    }

    /// [`Brep::coedge_uvs`] continued from each point asked for to the next.
    fn coedge_uvs_at(&self, c: CoedgeId, ts: &[f64]) -> Vec<Vec2> {
        let co = self.coedge(c);
        let edge = self.edge(co.edge);
        match &co.pcurve {
            PCurve::Affine { origin, dir } => ts.iter().map(|&t| *origin + *dir * t).collect(),
            PCurve::Fitted { .. } => co.pcurve.fitted_uvs(ts).unwrap_or_default(),
            PCurve::Projected { seed } => {
                let surface = &self.face(self.coedge_face(c)).surface;
                let mut hint = *seed;
                let mut out = Vec::with_capacity(ts.len());
                for (i, &t) in ts.iter().enumerate() {
                    let beside = if i == 0 { ts.get(1) } else { ts.get(i - 1) };
                    let inside = t > edge.range.0 && t < edge.range.1;
                    let uv = match (&edge.curve, beside) {
                        (Some(curve), Some(&b)) if inside && i > 0 => {
                            pcurve::follow(surface, curve, (b, t), hint)
                        }
                        (Some(curve), Some(&b)) => pcurve::project_on(surface, curve, t, b, hint),
                        (Some(curve), None) => surface.project_near(curve.eval(t), hint),
                        (None, _) => surface.project_near(self.vertex(edge.start).point, hint),
                    };
                    hint = uv;
                    out.push(uv);
                }
                out
            }
        }
    }

    /// How many faces carry each name, over the whole arena.
    pub fn name_counts(&self) -> BTreeMap<String, usize> {
        let mut out = BTreeMap::new();
        for f in &self.faces {
            for n in &f.names {
                *out.entry(n.to_string()).or_insert(0) += 1;
            }
        }
        out
    }

    /// Moves every entity by `t`. Parameters are unchanged: frames and curves move with the
    /// geometry, so pcurves stay valid.
    pub fn transform(&mut self, t: &Transform) {
        for v in &mut self.vertices {
            v.point = t.point(v.point);
        }
        for e in &mut self.edges {
            if let Some(c) = &e.curve {
                e.curve = Some(c.transformed(t));
            }
        }
        for f in &mut self.faces {
            f.surface = f.surface.transformed(t);
        }
        // A face's volume share about its own anchor, and whether its loops are clean, are the
        // same wherever a rigid motion puts it: an assembly sweep moves a part before each
        // boolean, and its spline faces would otherwise be integrated again at every step.
        // Slid, a face's box slides with it; turned, the turned box's own would be looser.
        let slid = t.r == Transform::IDENTITY.r;
        for m in &mut self.memos.0 {
            let fresh = FaceMemo::default();
            if let (true, Some(b)) = (slid, m.loose_box.get()) {
                let _ = fresh.loose_box.set(crate::math::Aabb {
                    min: b.min + t.t,
                    max: b.max + t.t,
                });
            }
            if let Some(v) = m.volume.get() {
                let _ = fresh.volume.set(FaceVolume {
                    anchor: t.point(v.anchor),
                    volume: v.volume,
                    vector_area: t.vector(v.vector_area),
                });
            }
            if m.clean.get().is_some() {
                let _ = fresh.clean.set(());
            }
            *m = Arc::new(fresh);
        }
    }

    /// Copies every entity of `other` into `self`, and returns the new ids of its solids.
    pub fn append(&mut self, other: &Brep) -> Vec<SolidId> {
        let (v0, e0, c0, l0, f0, sh0, so0) = (
            self.vertices.len(),
            self.edges.len(),
            self.coedges.len(),
            self.loops.len(),
            self.faces.len(),
            self.shells.len(),
            self.solids.len(),
        );
        self.vertices.extend(other.vertices.iter().cloned());
        self.edges.extend(other.edges.iter().map(|e| Edge {
            curve: e.curve.clone(),
            range: e.range,
            start: VertexId::at(e.start.index() + v0),
            end: VertexId::at(e.end.index() + v0),
        }));
        self.coedges.extend(other.coedges.iter().map(|c| Coedge {
            edge: EdgeId::at(c.edge.index() + e0),
            reversed: c.reversed,
            pcurve: c.pcurve.clone(),
            lp: LoopId::at(c.lp.index() + l0),
        }));
        self.loops.extend(other.loops.iter().map(|l| {
            Loop {
                coedges: l
                    .coedges
                    .iter()
                    .map(|c| CoedgeId::at(c.index() + c0))
                    .collect(),
                face: FaceId::at(l.face.index() + f0),
            }
        }));
        self.faces.extend(other.faces.iter().map(|f| Face {
            surface: f.surface.clone(),
            reversed: f.reversed,
            loops: f.loops.iter().map(|l| LoopId::at(l.index() + l0)).collect(),
            names: f.names.clone(),
            shell: f.shell.map(|s| ShellId::at(s.index() + sh0)),
        }));
        self.memos.0.extend(other.memos.0.iter().cloned());
        self.shells.extend(other.shells.iter().map(|s| Shell {
            faces: s.faces.iter().map(|f| FaceId::at(f.index() + f0)).collect(),
            solid: s.solid.map(|s| SolidId::at(s.index() + so0)),
        }));
        self.solids.extend(other.solids.iter().map(|s| {
            Solid {
                shells: s
                    .shells
                    .iter()
                    .map(|sh| ShellId::at(sh.index() + sh0))
                    .collect(),
                outer: s.outer,
            }
        }));
        for (&a, &b) in &other.partners {
            self.partners
                .insert(CoedgeId::at(a.index() + c0), CoedgeId::at(b.index() + c0));
        }
        for (&a, &b) in &other.beside {
            self.beside
                .insert(CoedgeId::at(a.index() + c0), CoedgeId::at(b.index() + c0));
        }
        self.index_uses();
        for (&v, &t) in &other.tolerances.vertices {
            self.tolerances
                .vertices
                .insert(VertexId::at(v.index() + v0), t);
        }
        for (&e, &t) in &other.tolerances.edges {
            self.tolerances.edges.insert(EdgeId::at(e.index() + e0), t);
        }
        (so0..self.solids.len()).map(SolidId::at).collect()
    }
}
