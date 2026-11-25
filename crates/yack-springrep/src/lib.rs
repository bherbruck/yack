// yack: root of the temporary yack-springrep crate (PLAN §3.2), deleted by M3.
// Provenance of this crate: https://github.com/springcraft-ai/springrep, commit
//   a03e8a1978d3cc1ac51d38b9fc767b76cfff1f31 (v1.0.1), paths src/, tests/, build.rs, README.md;
//   MIT, (c) 2026 Springcraft, Inc. (LICENSE-springrep). Copied by script; every .rs file carries
//   its own header. Only crate paths (springrep:: -> yack_springrep::) and headers differ.
// Adapted from springrep@a03e8a1 src/lib.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: this header added; one line `pub mod facade;` (yack glue, src/facade/).
#![doc = include_str!("../README.md")]
#![forbid(unsafe_code)]
#![warn(unreachable_pub)]

pub mod blend;
pub mod boolean;
pub use yack_build::build;
pub use yack_check as check;
pub use yack_codec as codec;
pub use yack_locate::distance;
pub mod draw;
pub mod edit;
// yack: glue the yack facade calls (not springrep code).
pub mod facade;
use yack_base::kernel as error;
pub use yack_geom as geom;
pub(crate) use yack_intersect as intersect;
pub(crate) use yack_locate as locate;
pub use yack_build::loft;
#[cfg(test)]
mod moved_tests;
pub use yack_math::kernel as math;
pub use yack_measure as measure;
pub use yack_mesh as mesh;
pub use yack_topo::names;
use yack_numeric as numeric;
pub mod overlap;
pub use yack_build::primitive;
pub mod probe;
pub use yack_profile as profile;
pub use yack_locate::rays;
pub mod step;
pub use yack_build::sweep;
pub use yack_tol as tol;
pub use yack_topo as topo;
pub(crate) use yack_tweak as tweak;

pub use error::{Error, Result};

/// A digest of the kernel's sources, unit tests aside: two builds with the same one build the same
/// geometry, so a cache of built parts keys on it.
pub const SOURCE: &str = env!("SPRINGREP_SOURCE");

/// The release this build is, as published: what an adapter reports as the kernel's version.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
