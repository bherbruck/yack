// yack: building a solid's arena from raw topology: the assembly, healing and checks a reader or a rebuild ends in.
//! Building a solid's arena from its raw topology (`raw`), healing it onto its surfaces (`heal`,
//! `adjust`) and checking it (`assemble`), and a solid from a closed triangle mesh (`mesh_solid`). Moved
//! out of `step/` and `mesh/` in M2 (T5) so the blends and edit::affine reach them below step, and the
//! mesher no longer calls the checker.

mod adjust;
mod assemble;
pub(crate) mod heal;
mod mesh_solid;
mod raw;

pub(crate) use assemble::assemble;
pub use heal::{Healed, MAX_HEAL};
pub(crate) use mesh_solid::from_indexed;
pub use mesh_solid::solid as solid_from_mesh;
pub(crate) use raw::{
    PartFace, Parts, Raw, RawEdge, RawFace, Use, edge_range, param_of, parts_of, rebuilt, solid_of,
};
