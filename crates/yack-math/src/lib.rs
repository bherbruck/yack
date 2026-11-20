// yack: value types and small geometry maths (PLAN §4.1; the `reexport` list in layers.toml).
//! yack-math: points, vectors, directions, angles, frames, boxes and rigid/similarity/affine
//! maps, plus planar loop validation.
//!
//! Internal crate; `yack` re-exports the value types as they are.
#![deny(missing_docs)]

mod aabb;
mod angle;
mod frame;
pub mod planar;
mod vector;
mod view;
mod xform;

pub use aabb::Aabb;
pub use angle::{Angle, deg, rad};
pub use frame::Frame;
pub use vector::{Dir3, Point2, Point3, Vec3};
pub use view::{mean, mesh_volume, scale2, scale_length, view_axes, view_point};
pub use xform::{Affine, Rigid, Similarity};
