// yack: STEP B-rep files measured exactly from their own geometry, by ISO 10303-42's rules.
// Adapted from springrep@a03e8a1 src/step/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/step/mod.rs to yack-step (M3 W9c); paths and visibility only.
//! An exchange file's solid measured as the standard defines it: its volume and area from its
//! faces' surfaces bounded by their edges' curves.


// springrep's step code indexes by position; D26's hardening pass moves it onto checked access,
// as for yack-linalg until M5.
#![allow(clippy::indexing_slicing)]

// springrep's foundation modules (D26), under the names the moved code uses.
use yack_base::kernel::Error;
use yack_base::kernel::Result;
use yack_boolean as boolean;
use yack_build::build;
use yack_check as check;
use yack_geom as geom;
use yack_math::kernel as math;
use yack_numeric as numeric;
use yack_tol as tol;
use yack_topo as topo;
#[cfg(test)]
use yack_build::loft;
#[cfg(test)]
use yack_profile as profile;
#[cfg(test)]
use yack_build::sweep;
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

#[cfg(feature = "wip")]
mod measure;

#[cfg(feature = "wip")]
pub use measure::{FaceMeasure, FileMeasure, measure_file};
