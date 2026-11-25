// yack: Rational B-spline curves and surfaces of any degree up to [`MAX_DEGREE`], with explicit knot
// Adapted from springrep@a03e8a1 src/geom/nurbs.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/nurbs.rs to yack-geom (M3 W3); its parts split into
// submodules of their own (M3 hardening): basis, bezier, curve, iso, surface, project, tests.
//! Rational B-spline curves and surfaces of any degree up to [`MAX_DEGREE`], with explicit knot
//! vectors, as STEP files carry them. A curve or surface whose two ends meet is periodic in that
//! parameter: evaluation wraps into the knot domain, so an edge or loop may run across the join.

// springrep's spline code indexes by position; the M3 hardening pass (D26) moves each module onto
// checked access and drops its allow here.
mod basis;
mod bezier;
mod curve;
mod iso;
mod project;
#[allow(clippy::indexing_slicing)]
mod seeds;
mod surface;
#[cfg(test)]
pub(crate) mod tests;

pub(crate) use basis::{basis_derivatives, span_poly};
pub use bezier::{Bezier, Patch};
pub use curve::NurbsCurve;
pub use iso::IsoLine;
pub use surface::NurbsSurface;

use crate::tol;

pub(super) const MAX_DEGREE: usize = 15;
/// How near the surface, as a share of the seed spacing, a local projection is taken as nearest.
const NEAR_SHARE: f64 = 0.05;
/// How short a stalled Newton step, and how small the distance gradient along the surface, a
/// search that stops there counts as settled at.
const STALLED: f64 = 1e-3 * tol::LINEAR; // tol: rounding of a foot, a thousandth of the tolerance
const M: usize = MAX_DEGREE + 1;
/// The highest degree the span polynomials evaluate (see [`NurbsSurface::by_polys`]).
const MAX_POLY_DEGREE: usize = 6;
