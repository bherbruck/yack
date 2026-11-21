// yack: The kernel's tolerance model: the distances under which two things are taken as one,....
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/tol.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: Tolerances::exact_only (yack: refuse instead of meshing); M4.0a: VTOL moved here from
//   boolean.rs; M4.1a: the values and Tolerances moved to yack_tol::cadrs, re-exported here;
//   Tolerances::validate is the free fn validate.
//! The kernel's tolerance model: the distances under which two things are taken as one, named
//! once here and carried explicitly (in [`Tolerances`], held by the kernel session) instead of
//! as literals and globals.
//!
//! Lengths are in mm. Three levels, each ten times the one below:
//! - [`CONFUSION`] (1e-7): exact geometry. Two points closer are the same point; an entity's own
//!   tolerance ([`super::topo::Vertex::tol`], [`super::topo::Edge::tol`]) never goes below it
//!   (OCCT's `Precision::Confusion`).
//! - [`POINT`] (1e-6): what an operation merges. A boolean pools vertices closer than this and
//!   puts edge ends on edges within it; intersections computed numerically land within it. A
//!   body's [`super::topo::Body::check`] accepts an edge end or a p-curve this far off when the
//!   entity's own tolerance is smaller.
//! - [`LOOP_GAP`] (1e-5): a p-curve fitted to an edge may miss the next coedge's start by this
//!   much (in model space, through the surface's metric), which `check` allows at a vertex of
//!   smaller tolerance.
//!
//! Above these, entities carry their own tolerance: a read file's or a healed body's vertices
//! and edges hold the gaps actually there ([`super::topo::Body::fit_tolerances`], as OCCT), and
//! operations take the larger of their own and the entities'.
//!
//! Models lie within [`SIZE`] of the origin: beyond it a double no longer resolves
//! [`CONFUSION`] (2⁻⁵² · 1e8 ≈ 2e-8 mm). Readers refuse files with geometry outside.
//!
//! A boolean whose first run leaves an open shell runs again at the [`Tolerances::retries`]
//! (1e-5, then 1e-4), as OCCT grows its tolerances; the result's entities record the tolerance
//! the run used.
//!
//! Below these, the round-off margins of exact computations are named by meaning, so a reader
//! sees which comparison is relative and which is absolute (all 1e-9 today, which keeps every
//! result as it was when they were literals):
//! - [`REL`]: relative, always scaled by what it compares (`REL * size`, `REL * (1.0 + |t|)`,
//!   a fraction of a span, `r * (1.0 - REL)`, a parameter fraction in [0, 1]);
//! - [`ANGULAR`]: directions and angles: unit vectors parallel (`1 − cos`, `|sin|`, `|u × v|`) or
//!   square (`|u · v|`), angles and spans in radians (`TAU - ANGULAR`);
//! - [`ROUNDOFF`]: absolute, for values of order one: lengths (mm) and parameters of exactly
//!   built geometry, far below [`CONFUSION`], and floors on sizes;
//! - [`TINY`]: the guard of a division or a normalisation against an exact zero, far below any
//!   meaningful value.
//!
//! A thousandth of those (1e-12), for comparisons of values computed exactly (closed forms,
//! knots, parameters of analytic curves) where 1e-9 would merge distinct ones:
//! - [`REL_FINE`], [`ANGULAR_FINE`] and [`ROUNDOFF_FINE`]: as [`REL`], [`ANGULAR`] and
//!   [`ROUNDOFF`];
//! - [`DEGENERATE`]: a computed length, norm or denominator below which it is taken as zero (a
//!   vector with no direction, `try_normalize(DEGENERATE)`; a span or a denominator of nothing):
//!   values that cancel leave round-off of this order, far above [`TINY`].
//!
//! At the merge level (1e-6, [`POINT`]'s), for what an operation takes as the same:
//! - [`POINT`]: absolute distances (mm);
//! - [`REL_POINT`]: a fraction of the size, span or parameter range it multiplies;
//! - [`ANGULAR_POINT`]: directions taken as parallel or square, angles (radians) taken as 0 or π;
//! - [`DIFF_STEP`]: the step of a finite difference, or of a nudge off a point into a face or
//!   along a curve (a parameter step, or a fraction of the span it multiplies).

pub use yack_tol::cadrs::{
    small, Tolerances, ANGULAR, ANGULAR_FINE, ANGULAR_POINT, CONFUSION, DEGENERATE, DIFF_STEP,
    LOOP_GAP, POINT, REL, REL_FINE, REL_POINT, ROUNDOFF, ROUNDOFF_FINE, SIZE, TINY, VTOL,
};

/// Checks that the tolerances are usable: finite, positive and growing
/// (`confusion <= point <= retries`).
pub fn validate(t: &Tolerances) -> crate::Result<()> {
    if !t.usable() {
        return Err(crate::KernelError::InvalidParameter(format!(
            "tolerances must be positive and growing: {t:?}"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The defaults are the documented levels, and the constants the kernel used before
    /// (`topo::TOL_MIN`, `boolean::VTOL`, the boolean's retries) are these.
    #[test]
    fn defaults_are_the_model() {
        let t = Tolerances::default();
        assert_eq!(
            (t.confusion, t.point, t.retries),
            (1e-7, 1e-6, [1e-5, 1e-4])
        );
        assert_eq!(super::super::topo::TOL_MIN, CONFUSION);
        assert_eq!(super::super::tol::VTOL, POINT);
        assert!(validate(&t).is_ok());
        // The named round-off margins are the literals they replaced, bit for bit.
        assert_eq!([REL, ANGULAR, ROUNDOFF], [1e-9; 3]);
        assert_eq!(
            [REL_FINE, ANGULAR_FINE, ROUNDOFF_FINE, DEGENERATE],
            [1e-12; 4]
        );
        assert_eq!([POINT, REL_POINT, ANGULAR_POINT, DIFF_STEP], [1e-6; 4]);
        assert!(validate(&Tolerances { point: f64::NAN, ..t }).is_err());
        assert!(validate(&Tolerances { point: 1e-3, ..t }).is_err());
        assert!(validate(&Tolerances { confusion: 0.0, ..t }).is_err());
    }

    /// The session holds its tolerances, refusing unusable ones.
    #[test]
    fn session_tolerances() {
        let mut k = super::super::NativeKernel::new();
        assert_eq!(k.tolerances(), Tolerances::default());
        assert!(
            k.set_tolerances(Tolerances {
                point: -1.0,
                ..Tolerances::default()
            })
            .is_err()
        );
        assert_eq!(k.tolerances(), Tolerances::default());
        k.set_tolerances(Tolerances {
            point: 2e-6,
            ..Tolerances::default()
        })
        .unwrap();
        assert_eq!(k.tolerances().point, 2e-6);
    }
}
