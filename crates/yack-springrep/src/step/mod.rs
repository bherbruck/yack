// yack: STEP AP214 import and export: [`write()`] turns an arena's solids into an exchange file whose
// Adapted from springrep@a03e8a1 src/step/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! STEP AP214 import and export: [`write()`] turns an arena's solids into an exchange file whose
//! faces carry their names, and [`write_assembly`] several parts placed by their instances;
//! [`read`] turns a file's advanced B-reps, placed by its product structure, into checked kernel
//! solids, each with its product's name and its colour.

mod geometry;
mod part21;
mod read;
mod topology;
mod write;

pub use crate::build::{Healed, MAX_HEAL};
pub use read::{Body, Crossed, Healing, Import, Tolerant, read, read_part};
pub use write::{Instance, Part, write, write_assembly, write_faces};

/// The faces of a STEP file, each its own with its own vertices and edges, sewn into one solid
/// where their edges lie within `tolerance` of each other, then healed onto their surfaces.
pub fn sew(text: &[u8], tolerance: f64) -> crate::Result<crate::topo::Brep> {
    let file = part21::parse(text)?;
    let mut geo = geometry::Geometry::new(&file);
    topology::sew(&mut geo, tolerance)
}

/// The `originating_system` this kernel writes; a file that names it carries the kernel's face names.
pub const WRITER: &str = "springrep";

#[cfg(test)]
mod tests;
