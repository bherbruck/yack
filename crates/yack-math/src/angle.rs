// yack: angles, built only from degrees or radians (PLAN §4.2 rule 4, DECISIONS D6).

/// An angle. Built only by [`deg`] or [`rad`]; there is no `From<f64>`, so a bare number can never
/// be taken in the wrong unit. Negative angles turn clockwise.
///
/// ```
/// use yack_math::*;
/// let a = deg(90.0);
/// assert!((a.radians() - std::f64::consts::FRAC_PI_2).abs() < 1e-15 && (a.degrees() - 90.0).abs() < 1e-12);
/// assert!(a.is_finite() && !a.is_full_turn() && deg(360.0).is_full_turn());
/// # Ok::<(), yack_base::Error>(())
/// ```
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

/// An angle of `n` whole turns (`n · 2π` radians).
///
/// ```
/// use yack_math::*;
/// assert_eq!(turns(0.5).radians(), std::f64::consts::PI);
/// # Ok::<(), yack_base::Error>(())
/// ```
pub fn turns(n: f64) -> Angle {
    Angle(n * core::f64::consts::TAU)
}

/// An angle of `r` radians.
///
/// ```
/// use yack_math::*;
/// assert_eq!(rad(1.5).radians(), 1.5);
/// # Ok::<(), yack_base::Error>(())
/// ```
pub const fn rad(r: f64) -> Angle {
    Angle(r)
}

impl Angle {
    /// A whole turn, counter-clockwise (`ccw`) or clockwise.
    ///
    /// ```
    /// use yack_math::*;
    /// assert!(Angle::full_turn(true).is_full_turn() && Angle::full_turn(false).radians() < 0.0);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub const fn full_turn(ccw: bool) -> Angle {
        if ccw { rad(core::f64::consts::TAU) } else { rad(-core::f64::consts::TAU) }
    }
    /// The angle in radians.
    ///
    /// (Its example is on [`Angle`].)
    pub const fn radians(self) -> f64 {
        self.0
    }
    /// The angle in degrees.
    ///
    /// (Its example is on [`Angle`].)
    pub fn degrees(self) -> f64 {
        self.0.to_degrees()
    }
    /// Whether the angle is a whole turn either way (within `tol::ANGULAR`).
    ///
    /// (Its example is on [`Angle`].)
    pub fn is_full_turn(self) -> bool {
        (self.0.abs() - core::f64::consts::TAU).abs() <= yack_tol::ANGULAR
    }
    /// Whether the angle is finite.
    ///
    /// (Its example is on [`Angle`].)
    pub fn is_finite(self) -> bool {
        self.0.is_finite()
    }
    /// The angle's sine.
    ///
    /// ```
    /// use yack_math::*;
    /// assert!((deg(30.0).sin() - 0.5).abs() < 1e-15);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn sin(self) -> f64 {
        self.0.sin()
    }
    /// The angle's cosine.
    ///
    /// ```
    /// use yack_math::*;
    /// assert!((deg(60.0).cos() - 0.5).abs() < 1e-15);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn cos(self) -> f64 {
        self.0.cos()
    }
    /// The angle's tangent.
    ///
    /// ```
    /// use yack_math::*;
    /// assert!((deg(45.0).tan() - 1.0).abs() < 1e-15);
    /// # Ok::<(), yack_base::Error>(())
    /// ```
    pub fn tan(self) -> f64 {
        self.0.tan()
    }
}
