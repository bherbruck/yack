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

// From springrep's tol.rs (M3 W1', D26).
/// The parameter tolerance on a curve whose speed at the point is `speed` mm per unit parameter.
pub fn param(speed: f64) -> f64 {
    LINEAR / speed.max(1e-300)
}

/// The distance within which two entities whose local tolerances are `a` and `b` coincide: the
/// sum of the two, less the one [`LINEAR`] two exact entities share, so that those still coincide
/// within [`LINEAR`]. Only imported vertices and edges, and a vertex a boolean places where two
/// solids graze, carry a tolerance above it.
pub fn coincidence(a: f64, b: f64) -> f64 {
    (a + b - LINEAR).max(LINEAR)
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
