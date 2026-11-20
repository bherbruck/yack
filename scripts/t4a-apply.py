# M2.2 step 3 (T4a): the arena's encoding moves from topo/codec.rs into codec/topo.rs. Pure move:
# the same bytes are written. topo gains Brep::parts / Brep::from_parts so codec need not see inside.
# Usage: python3 t4a-apply.py <yack-springrep dir>
import os, sys

src = os.path.join(sys.argv[1], "src")
p = lambda *a: os.path.join(src, *a)


def edit(f, pairs):
    s = open(p(f)).read()
    for a, b in pairs:
        if s.count(a) != 1:
            raise SystemExit(f"{f}: expected one {a[:70]!r}, found {s.count(a)}")
        s = s.replace(a, b)
    open(p(f), "w").write(s)


old = open(p("topo/codec.rs")).read()
i_brep = old.index("impl Codec for Brep {")
i_ids = old.index("impl Brep {\n    /// Every id the arena holds names an entity of it.")
body = old[old.index("macro_rules! ids {"):i_brep]          # ids!, fields!, PCurve
ids_in_range = old[i_ids:]                                    # impl Brep { fn ids_in_range .. }

body = body.replace("Some($t(u32::get(r)?))", "Some($t::from_raw(u32::get(r)?))").replace(
    "                self.0.put(w);", "                self.raw().put(w);")
if "from_raw" not in body or "self.raw()" not in body:
    raise SystemExit("ids! macro not as expected")

open(p("codec/topo.rs"), "w").write("""// yack: [`Codec`] for the topology arena. A decoded arena keeps every id, so each name and every
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

""" + body + """impl Codec for Brep {
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
""")
os.remove(p("topo/codec.rs"))

open(p("topo/parts.rs"), "w").write("""// yack: the arena's own entities, lent to the encoder and taken back from it.
// Adapted from springrep@a03e8a1 src/topo/codec.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: the id check and the arena's assembly split from the encoding (M2.2 T4a), which moved to
// codec/topo.rs; `Parts` / `Entities` added so the encoder need not see inside `Brep`.
//! The arena's own entities, as the encoder writes them: everything but the memos, which are what
//! a process has worked out about the faces, and each edge's uses, which follow from the coedges.

use std::collections::BTreeMap;
use std::sync::Arc;

use super::memo::Memos;
use super::*;

/// The arena's entities, borrowed.
pub(crate) struct Parts<'a> {
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
pub(crate) struct Entities {
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
    pub(crate) fn parts(&self) -> Parts<'_> {
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
    pub(crate) fn from_parts(e: Entities) -> Option<Brep> {
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

""" + ids_in_range)

edit("topo/mod.rs", [
    ("mod codec;\n", ""),
    ("mod names;\n", "mod names;\nmod parts;\n"),
    ("pub use pcurve::PCurve;\n", "pub(crate) use parts::{Entities, Parts};\npub use pcurve::PCurve;\n"),
    ("""            fn at(i: usize) -> $name {
                $name(u32::try_from(i).expect("arena index fits in u32"))
            }""", """            fn at(i: usize) -> $name {
                $name(u32::try_from(i).expect("arena index fits in u32"))
            }

            /// The id as the encoder writes it.
            pub(crate) fn raw(self) -> u32 {
                self.0
            }

            /// The id the encoder wrote as `raw`; the arena checks it names one of its entities.
            pub(crate) fn from_raw(raw: u32) -> $name {
                $name(raw)
            }"""),
    ("#[derive(Clone, Debug, Default, PartialEq)]\nstruct Tolerances {\n    vertices: BTreeMap<VertexId, f64>,\n    edges: BTreeMap<EdgeId, f64>,\n}",
     "#[derive(Clone, Debug, Default, PartialEq)]\npub(crate) struct Tolerances {\n    pub(crate) vertices: BTreeMap<VertexId, f64>,\n    pub(crate) edges: BTreeMap<EdgeId, f64>,\n}"),
])
edit("codec.rs", [("#[cfg(test)]\nmod tests;", "mod topo;\n\n#[cfg(test)]\nmod tests;")])
print("ok")
