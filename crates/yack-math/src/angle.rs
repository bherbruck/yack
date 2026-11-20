// yack: angles, built only from degrees or radians (PLAN §4.2 rule 4, DECISIONS D6).

/// An angle. Built only by [`deg`] or [`rad`]; there is no `From<f64>`, so a bare number can never
/// be taken in the wrong unit. Negative angles turn clockwise.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Angle(f64);

/// An angle of `d` degrees.
///
/// ```
/// use yack_math::{deg, rad};
/// assert!(deg(90.0) < deg(91.0) && deg(-1.0).radians() < 0.0);
/// assert_eq!(rad(1.5).radians(), 1.5);
/// ```
pub fn deg(d: f64) -> Angle {
    Angle(d.to_radians())
}

/// An angle of `r` radians.
pub const fn rad(r: f64) -> Angle {
    Angle(r)
}

impl Angle {
    /// The angle in radians.
    pub const fn radians(self) -> f64 {
        self.0
    }
    /// The angle in degrees.
    pub fn degrees(self) -> f64 {
        self.0.to_degrees()
    }
    /// Whether the angle is a whole turn either way (within `tol::ANGULAR`).
    pub fn is_full_turn(self) -> bool {
        (self.0.abs() - core::f64::consts::TAU).abs() <= yack_tol::ANGULAR
    }
    /// Whether the angle is finite.
    pub fn is_finite(self) -> bool {
        self.0.is_finite()
    }
}
