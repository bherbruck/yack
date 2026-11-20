// yack: [`Codec`] for the topology arena. A decoded arena keeps every id, so each name and every
// Adapted from springrep@a03e8a1 src/topo/codec.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: face memos are no longer encoded (M2.2 T4b): a decoded arena starts with fresh memos,
// so the bytes, and the digest, no longer depend on what was asked of a solid before.
//! [`Codec`] for the topology arena. A decoded arena keeps every id, so each name and every
//! reference into it holds; one whose ids point outside its own arrays is refused.

use std::sync::Arc;

use super::memo::Memos;
use super::*;
use crate::codec::{Codec, Reader, Writer, fields};

macro_rules! ids {
    ($($t:ident),*) => {$(
        impl Codec for $t {
            fn put(&self, w: &mut Writer) {
                self.0.put(w);
            }
            fn get(r: &mut Reader) -> Option<$t> {
                Some($t(u32::get(r)?))
            }
        }
    )*};
}
ids!(VertexId, EdgeId, CoedgeId, LoopId, FaceId, ShellId, SolidId);

fields!(FaceName { feature, face });
fields!(Vertex { point });
fields!(Edge {
    curve,
    range,
    start,
    end
});
fields!(Coedge {
    edge,
    reversed,
    pcurve,
    lp
});
fields!(Loop { coedges, face });
fields!(Face {
    surface,
    reversed,
    loops,
    names,
    shell
});
fields!(Shell { faces, solid });
fields!(Solid { shells, outer });
fields!(Tolerances { vertices, edges });

impl Codec for PCurve {
    fn put(&self, w: &mut Writer) {
        match self {
            PCurve::Affine { origin, dir } => {
                0u8.put(w);
                origin.put(w);
                dir.put(w);
            }
            PCurve::Projected { seed } => {
                1u8.put(w);
                seed.put(w);
            }
        }
    }
    fn get(r: &mut Reader) -> Option<PCurve> {
        Some(match u8::get(r)? {
            0 => PCurve::Affine {
                origin: Codec::get(r)?,
                dir: Codec::get(r)?,
            },
            1 => PCurve::Projected {
                seed: Codec::get(r)?,
            },
            _ => return None,
        })
    }
}

impl Codec for Brep {
    fn put(&self, w: &mut Writer) {
        let Brep {
            vertices,
            edges,
            coedges,
            loops,
            faces,
            shells,
            solids,
            memos: _,
            tolerances,
            partners,
            beside,
            uses: _,
        } = self;
        vertices.put(w);
        edges.put(w);
        coedges.put(w);
        loops.put(w);
        faces.put(w);
        shells.put(w);
        solids.put(w);
        tolerances.put(w);
        partners.put(w);
        beside.put(w);
    }

    fn get(r: &mut Reader) -> Option<Brep> {
        let vertices: Vec<Vertex> = Codec::get(r)?;
        let edges: Vec<Edge> = Codec::get(r)?;
        let coedges: Vec<Coedge> = Codec::get(r)?;
        let loops: Vec<Loop> = Codec::get(r)?;
        let faces: Vec<Face> = Codec::get(r)?;
        let shells: Vec<Shell> = Codec::get(r)?;
        let solids: Vec<Solid> = Codec::get(r)?;
        // Memos are what a process has worked out about the faces: they start empty.
        let memos = (0..faces.len()).map(|_| Arc::default()).collect();
        let tolerances: Box<Tolerances> = Codec::get(r)?;
        let partners: BTreeMap<CoedgeId, CoedgeId> = Codec::get(r)?;
        let beside: BTreeMap<CoedgeId, CoedgeId> = Codec::get(r)?;
        let mut b = Brep {
            vertices,
            edges,
            coedges,
            loops,
            faces,
            shells,
            solids,
            memos: Memos(memos),
            tolerances,
            partners,
            beside,
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
