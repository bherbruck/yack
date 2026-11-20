// yack: [`Codec`] for the topology arena. A decoded arena keeps every id, so each name and every
// Adapted from springrep@a03e8a1 src/topo/codec.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: face memos are no longer encoded (M2.2 T4b); moved from topo/codec.rs to codec/topo.rs
// (M2.2 T4a), reading the arena through `Brep::parts` and `Brep::from_parts`; the bytes are the same.
//! [`Codec`] for the topology arena. A decoded arena keeps every id, so each name and every
//! reference into it holds; one whose ids point outside its own arrays is refused.

use super::{Codec, Reader, Writer};
use crate::topo::{
    Brep, Coedge, CoedgeId, Edge, EdgeId, Entities, Face, FaceId, FaceName, Loop, LoopId, PCurve,
    Parts, Shell, ShellId, Solid, SolidId, Tolerances, Vertex, VertexId,
};

macro_rules! ids {
    ($($t:ident),*) => {$(
        impl Codec for $t {
            fn put(&self, w: &mut Writer) {
                self.raw().put(w);
            }
            fn get(r: &mut Reader) -> Option<$t> {
                Some($t::from_raw(u32::get(r)?))
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
        let Parts {
            vertices,
            edges,
            coedges,
            loops,
            faces,
            shells,
            solids,
            tolerances,
            partners,
            beside,
        } = self.parts();
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
        let vertices = Codec::get(r)?;
        let edges = Codec::get(r)?;
        let coedges = Codec::get(r)?;
        let loops = Codec::get(r)?;
        let faces = Codec::get(r)?;
        let shells = Codec::get(r)?;
        let solids = Codec::get(r)?;
        let tolerances = Codec::get(r)?;
        let partners = Codec::get(r)?;
        let beside = Codec::get(r)?;
        Brep::from_parts(Entities {
            vertices,
            edges,
            coedges,
            loops,
            faces,
            shells,
            solids,
            tolerances,
            partners,
            beside,
        })
    }
}
