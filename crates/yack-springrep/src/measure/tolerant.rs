// yack: a solid's volume integrated to a stated tolerance rather than to the last digits.
//! Checks that compare measures to a stated agreement need them only that well: a swept spline
//! body's exact integrals took most of a minute (bench swept/ring-carried-along-a-spline). There is
//! no area to a tolerance: summed over faces at 1e-8 it read conformance slabs-f7989a77c5e3f560's
//! area 2e-5 off, so areas stay exact.

use super::solid_volume_to;
use crate::topo::Brep;

/// [`super::volume`] to `rel` of the model's size cubed rather than to the last digits: each
/// face's integral stops refining once two levels agree that well.
pub fn volume_to(b: &Brep, rel: f64) -> f64 {
    b.solid_ids().map(|s| solid_volume_to(b, s, rel)).sum()
}
