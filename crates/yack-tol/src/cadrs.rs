// yack: cadrs's tolerance model (its named levels and the session's Tolerances), held here so yack-tol stays the only home of tolerance values.
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/tol.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: M4.1a: moved out of yack-cadrs native/tol.rs (values unchanged, bit for bit); validate's
//   error stays in yack-cadrs, so here it is `usable` (the same test, written without indexing).
//! cadrs's tolerance model, used by the cadrs-derived code while it is re-hosted (M4). Lengths
//! are in mm. `yack_cadrs::native::tol` documents the levels; the values are these.

/// Exact geometry: points closer than this are one point (mm).
pub const CONFUSION: f64 = 1e-7;
/// The default merge distance of an operation (mm).
pub const POINT: f64 = 1e-6;
/// The default distance under which two points of a boolean are one (mm; [`POINT`]).
pub const VTOL: f64 = POINT;
/// The largest gap between consecutive p-curves of a loop that is still closed (mm).
pub const LOOP_GAP: f64 = 1e-5;
/// The size box: coordinates and radii of a model stay within ±`SIZE` (mm, 100 km).
pub const SIZE: f64 = 1e8;
/// The relative round-off margin: a fraction of the size, span or value it is multiplied by.
pub const REL: f64 = 1e-9;
/// The angular round-off margin (radians, or `1 − cos`, `|sin|` for unit vectors).
pub const ANGULAR: f64 = 1e-9;
/// The absolute round-off margin for lengths (mm) and parameters of order one.
pub const ROUNDOFF: f64 = 1e-9;
/// The guard against dividing by (or normalising) an exact zero.
pub const TINY: f64 = 1e-300;
/// The fine relative round-off margin (a thousandth of [`REL`]).
pub const REL_FINE: f64 = 1e-12;
/// The fine angular round-off margin (a thousandth of [`ANGULAR`]).
pub const ANGULAR_FINE: f64 = 1e-12;
/// The fine absolute round-off margin (a thousandth of [`ROUNDOFF`]).
pub const ROUNDOFF_FINE: f64 = 1e-12;
/// A computed length, norm or denominator below this is zero.
pub const DEGENERATE: f64 = 1e-12;
/// [`POINT`] as a fraction of a size, span or range.
pub const REL_POINT: f64 = 1e-6;
/// Directions closer than this are the same at the merge level.
pub const ANGULAR_POINT: f64 = 1e-6;
/// The step of a finite difference, or of a nudge off a point.
pub const DIFF_STEP: f64 = 1e-6;

/// What a length that is absolute for a part of a millimetre or more is scaled by for a
/// smaller part: 1 from a millimetre up, the part's size in mm below.
pub fn small(size: f64) -> f64 {
    if size.is_finite() && size > 0.0 {
        size.min(1.0)
    } else {
        1.0
    }
}

/// The tolerances an operation runs with, carried by the kernel session. `Default` is the
/// model above.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerances {
    /// The merge distance of a boolean's first run ([`POINT`]).
    pub point: f64,
    /// The merge distances of the runs after a first that leaves an open shell, in order.
    pub retries: [f64; 2],
    /// The smallest tolerance an entity holds ([`CONFUSION`]).
    pub confusion: f64,
    /// yack: refuse an answer cadrs would only approximate by a mesh (a moving solid's sweep it
    /// cannot make exactly) at once, instead of meshing it.
    pub exact_only: bool,
}

impl Default for Tolerances {
    fn default() -> Self {
        // (Written out: 10.0 * POINT isn't exactly 1e-5 in floating point.)
        Self {
            point: POINT,
            retries: [LOOP_GAP, 1e-4],
            confusion: CONFUSION,
            exact_only: false,
        }
    }
}

impl Tolerances {
    /// The tolerances for a model of this size (its box's diagonal, mm): the defaults for a
    /// millimetre and more, to the bit; below that the merge distance and the retries shrink
    /// with the model ([`small`]), down to the smallest tolerance an entity holds.
    pub fn for_model(diagonal: f64) -> Self {
        let by = small(diagonal);
        let d = Self::default();
        let at_least = |x: f64| (x * by).max(d.confusion);
        Self {
            point: at_least(d.point),
            retries: [at_least(d.retries[0]), at_least(d.retries[1])],
            confusion: d.confusion,
            exact_only: d.exact_only,
        }
    }

    /// Whether the tolerances are usable: finite, positive and growing
    /// (`confusion <= point <= retries`).
    pub fn usable(&self) -> bool {
        let [r0, r1] = self.retries;
        let all = [self.confusion, self.point, r0, r1];
        all.iter().all(|t| t.is_finite() && *t > 0.0) && self.confusion <= self.point && self.point <= r0 && r0 <= r1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The named levels are the literals cadrs used, bit for bit.
    #[test]
    fn levels_are_cadrs() {
        let t = Tolerances::default();
        assert_eq!((t.confusion, t.point, t.retries), (1e-7, 1e-6, [1e-5, 1e-4]));
        assert_eq!([REL, ANGULAR, ROUNDOFF], [1e-9; 3]);
        assert_eq!([REL_FINE, ANGULAR_FINE, ROUNDOFF_FINE, DEGENERATE], [1e-12; 4]);
        assert_eq!([POINT, REL_POINT, ANGULAR_POINT, DIFF_STEP, VTOL], [1e-6; 5]);
        assert!(t.usable());
        assert!(!Tolerances { point: f64::NAN, ..t }.usable());
        assert!(!Tolerances { point: 1e-3, ..t }.usable());
        assert_eq!(Tolerances::for_model(10.0), t);
        assert_eq!(Tolerances::for_model(0.5).point, 5e-7);
    }
}
