// yack: fixture kernel crate at layer 1.
// Adapted from springrep@a03e8a1 src/geom.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: fixture.
//! Geom.
pub mod curve;

/// A point.
pub struct Point3 {
    /// x
    pub x: f64,
}
