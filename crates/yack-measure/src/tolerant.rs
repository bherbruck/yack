// yack: a solid's volume to a stated tolerance rather than the last digits, and checks against one.
//! Checks that compare measures to a stated agreement need them only that well: a swept spline
//! body's exact integrals took most of a minute (bench swept/ring-carried-along-a-spline). There is
//! no area to a tolerance: summed over faces at 1e-8 it read conformance slabs-f7989a77c5e3f560's
//! area 2e-5 off, so areas stay exact.

use super::{EXACT, reference, solid_volume_to, volume};
use crate::topo::Brep;

/// [`super::volume`] to `rel` of the model's size cubed rather than to the last digits: each
/// face's integral stops refining once two levels agree that well.
pub fn volume_to(b: &Brep, rel: f64) -> f64 {
    b.solid_ids().map(|s| solid_volume_to(b, s, rel)).sum()
}

/// How much tighter than the agreement asked [`volume_agrees`] integrates, and how near the
/// boundary of that agreement it checks again exactly.
const MARGIN: f64 = 1e-3; // tol: a share of the agreement, not a length
const NEAR_EDGE: f64 = 0.1;

/// Whether the volume of `b` is `expected` to `rel`, the gap taken against max(|`expected`|, 1):
/// integrated to an absolute tolerance of [`MARGIN`] of that bound (absolute, so a thin body is
/// held as tightly as a thick one), and worked out exactly where the estimate lands within
/// [`NEAR_EDGE`] of the bound. False for a non-finite `expected` or `rel`.
pub fn volume_agrees(b: &Brep, expected: f64, rel: f64) -> bool {
    if !(expected.is_finite() && rel.is_finite() && rel >= 0.0) {
        return false;
    }
    // As the bridge's and the engines' agreement checks measure a gap: against |expected|, or
    // against 1 below it.
    let bound = rel * expected.abs().max(1.0);
    let (_, scale) = reference(b);
    let to = (MARGIN * bound / (scale * scale * scale)).max(EXACT);
    let off = (volume_to(b, to) - expected).abs();
    if (off - bound).abs() <= NEAR_EDGE * bound {
        (volume(b) - expected).abs() <= bound
    } else {
        off <= bound
    }
}
