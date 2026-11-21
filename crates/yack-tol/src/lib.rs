// yack: the only home of tolerance values (PLAN §4.1, AGENTS.md rule 6).
//! yack-tol: the model tolerances. Lengths are mm.
//!
//! Internal crate; `yack::tol` exposes [`LINEAR`] and [`PICK`].
#![deny(missing_docs)]

pub mod cadrs;

/// Two points closer than this are the same point (mm).
pub const LINEAR: f64 = 1e-7;
/// The model size the tolerances are scaled for (mm).
pub const MODEL_SIZE: f64 = 1000.0;
/// Two unit directions closer than this (radians, or unit-vector difference) are the same.
pub const ANGULAR: f64 = LINEAR / MODEL_SIZE;
/// The smallest feature yack resolves (mm).
pub const RESOLVED: f64 = 0.1;
/// Directions closer than this are treated as tangent.
pub const TANGENT: f64 = LINEAR / RESOLVED;
/// How far from an entity a selection point may be (mm).
pub const PICK: f64 = 10.0 * LINEAR;

/// The tolerance between two entities with tolerances `a` and `b`: the larger.
pub fn between(a: f64, b: f64) -> f64 {
    a.max(b)
}

/// Whether a deviation `d` is within tolerance `t`, scaled up for values of size `magnitude`
/// beyond [`MODEL_SIZE`].
pub fn within(d: f64, t: f64, magnitude: f64) -> bool {
    let scale = (magnitude.abs() / MODEL_SIZE).max(1.0);
    d.abs() <= t * scale
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn within_scales_past_model_size() {
        assert!(within(0.5 * LINEAR, LINEAR, 1.0));
        assert!(!within(2.0 * LINEAR, LINEAR, 1.0));
        assert!(within(2.0 * LINEAR, LINEAR, 2.0 * MODEL_SIZE));
    }
}
