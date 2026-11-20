// yack: The kernel's tolerances. Every geometric comparison uses one of these, or a value derived from
// Adapted from springrep@a03e8a1 src/tol.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! The kernel's tolerances. Every geometric comparison uses one of these, or a value derived from
//! one in the place it is used; no module keeps a tolerance of its own.

/// Two points closer than this are the same point, in mm.
pub const LINEAR: f64 = 1e-7;

/// The largest model the tolerances are sized for, in mm.
pub const MODEL_SIZE: f64 = 1000.0;

/// Two directions closer than this are the same direction, in radians. A direction within it moves
/// a point at most [`LINEAR`] across a model of [`MODEL_SIZE`].
pub const ANGULAR: f64 = LINEAR / MODEL_SIZE;

/// The shortest stretch over which the kernel tells two surfaces that meet at a small angle apart
/// from two that touch, in mm.
pub const RESOLVED: f64 = 0.1;

/// The sine of the angle between two surfaces (or a ray and a surface) at or below which they are
/// tangent: they stay within [`LINEAR`] of each other for [`RESOLVED`] along their meeting.
pub const TANGENT: f64 = LINEAR / RESOLVED;

/// The parameter tolerance on a curve whose speed at the point is `speed` mm per unit parameter.
pub fn param(speed: f64) -> f64 {
    LINEAR / speed.max(1e-300)
}

/// The distance within which two entities whose local tolerances are `a` and `b` coincide: the
/// sum of the two, less the one [`LINEAR`] two exact entities share, so that those still coincide
/// within [`LINEAR`]. Only imported vertices and edges, and a vertex a boolean places where two
/// solids graze, carry a tolerance above it.
pub fn between(a: f64, b: f64) -> f64 {
    (a + b - LINEAR).max(LINEAR)
}

/// `d <= t`, read with a few ulps of the coordinates' size at `at` to spare, so that what lies at
/// exactly a tolerance counts as within it however its coordinates round: a copy moved by exactly
/// [`LINEAR`] lies 1e-7 mm plus a few ulps away.
pub fn within(d: f64, t: f64, at: crate::math::Point3) -> bool {
    d <= t + 16.0 * f64::EPSILON * (1.0 + at.x.abs().max(at.y.abs()).max(at.z.abs()))
}
