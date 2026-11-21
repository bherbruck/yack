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
pub mod check;
pub mod codec;
pub mod distance;
pub mod draw;
pub mod edit;
// yack: glue the yack facade calls (not springrep code).
pub mod facade;
mod error;
pub mod geom;
pub(crate) mod intersect;
pub(crate) mod locate;
pub mod loft;
pub mod math;
pub mod measure;
pub mod mesh;
pub mod names;
mod numeric;
pub mod overlap;
pub mod primitive;
pub mod probe;
pub mod profile;
pub mod rays;
pub mod step;
pub mod sweep;
pub mod tol;
pub mod topo;
pub(crate) mod tweak;

pub use error::{Error, Result};

/// A digest of the kernel's sources, unit tests aside: two builds with the same one build the same
/// geometry, so a cache of built parts keys on it.
pub const SOURCE: &str = env!("SPRINGREP_SOURCE");

/// The release this build is, as published: what an adapter reports as the kernel's version.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
