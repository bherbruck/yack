// yack: the arena's own entities, lent to the encoder and taken back from it.
// Adapted from springrep@a03e8a1 src/topo/codec.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: the id check and the arena's assembly split from the encoding (M2.2 T4a), which moved to
// codec/topo.rs; `Parts` / `Entities` added so the encoder need not see inside `Brep`; then to yack-topo (M3 W4), paths and visibility only.
//! The arena's own entities, as the encoder writes them: everything but the memos, which are what
//! a process has worked out about the faces, and each edge's uses, which follow from the coedges.

use std::collections::BTreeMap;
use std::sync::Arc;

use super::memo::Memos;
use super::*;

/// The arena's entities, borrowed.
pub struct Parts<'a> {
    pub vertices: &'a Vec<Vertex>,
    pub edges: &'a Vec<Edge>,
    pub coedges: &'a Vec<Coedge>,
    pub loops: &'a Vec<Loop>,
    pub faces: &'a Vec<Face>,
    pub shells: &'a Vec<Shell>,
    pub solids: &'a Vec<Solid>,
    pub tolerances: &'a Tolerances,
    pub partners: &'a BTreeMap<CoedgeId, CoedgeId>,
    pub beside: &'a BTreeMap<CoedgeId, CoedgeId>,
}

/// The arena's entities, owned: what [`Brep::from_parts`] assembles an arena from.
pub struct Entities {
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Edge>,
    pub coedges: Vec<Coedge>,
    pub loops: Vec<Loop>,
    pub faces: Vec<Face>,
    pub shells: Vec<Shell>,
    pub solids: Vec<Solid>,
    pub tolerances: Box<Tolerances>,
    pub partners: BTreeMap<CoedgeId, CoedgeId>,
    pub beside: BTreeMap<CoedgeId, CoedgeId>,
}

impl Brep {
    /// The arena's entities, for the encoder.
    pub fn parts(&self) -> Parts<'_> {
        Parts {
            vertices: &self.vertices,
            edges: &self.edges,
            coedges: &self.coedges,
            loops: &self.loops,
            faces: &self.faces,
            shells: &self.shells,
            solids: &self.solids,
            tolerances: &self.tolerances,
            partners: &self.partners,
            beside: &self.beside,
        }
    }

    /// The arena of these entities, with fresh memos and its edges' uses worked out; `None` when an
    /// id points outside its own arrays.
    pub fn from_parts(e: Entities) -> Option<Brep> {
        // Memos are what a process has worked out about the faces: they start empty.
        let memos = (0..e.faces.len()).map(|_| Arc::default()).collect();
        let mut b = Brep {
            vertices: e.vertices,
            edges: e.edges,
            coedges: e.coedges,
            loops: e.loops,
            faces: e.faces,
            shells: e.shells,
            solids: e.solids,
            memos: Memos(memos),
            tolerances: e.tolerances,
            partners: e.partners,
            beside: e.beside,
            uses: Vec::new(),
        };
        if !b.ids_in_range() {
            return None;
        }
        b.index_uses();
        Some(b)
    }
}

impl Brep {
    /// Every id the arena holds names an entity of it.
    fn ids_in_range(&self) -> bool {
        let v = |id: VertexId| id.index() < self.vertices.len();
        let e = |id: EdgeId| id.index() < self.edges.len();
        let c = |id: CoedgeId| id.index() < self.coedges.len();
        let l = |id: LoopId| id.index() < self.loops.len();
        let f = |id: FaceId| id.index() < self.faces.len();
        let sh = |id: ShellId| id.index() < self.shells.len();
        let so = |id: SolidId| id.index() < self.solids.len();
        self.partners
            .iter()
            .chain(&self.beside)
            .all(|(&a, &b)| c(a) && c(b))
            && self.edges.iter().all(|x| v(x.start) && v(x.end))
            && self.coedges.iter().all(|x| e(x.edge) && l(x.lp))
            && self
                .loops
                .iter()
                .all(|x| f(x.face) && x.coedges.iter().all(|&i| c(i)))
            && self
                .faces
                .iter()
                .all(|x| x.loops.iter().all(|&i| l(i)) && x.shell.is_none_or(sh))
            && self
                .shells
                .iter()
                .all(|x| x.faces.iter().all(|&i| f(i)) && x.solid.is_none_or(so))
            && self.solids.iter().all(|x| x.shells.iter().all(|&i| sh(i)))
            && self.tolerances.vertices.keys().all(|&i| v(i))
            && self.tolerances.edges.keys().all(|&i| e(i))
    }
}
