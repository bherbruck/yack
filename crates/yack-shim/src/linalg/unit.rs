// yack: nalgebra's `Unit` wrapper: a value known to have norm 1.

use core::ops::{Deref, Mul, Neg};

use super::matrix::Matrix;
use super::storage::{Sh, Shape};

/// What can be normalized: matrices and quaternions.
pub trait Normed: Copy {
    /// The Euclidean norm.
    fn norm(&self) -> f64;
    /// The squared norm.
    fn norm_squared(&self) -> f64;
    /// Divided by `n`, element by element.
    fn unscale(&self, n: f64) -> Self;
}

impl<const R: usize, const C: usize> Normed for Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C>,
{
    fn norm(&self) -> f64 {
        Matrix::norm(self)
    }
    fn norm_squared(&self) -> f64 {
        Matrix::norm_squared(self)
    }
    fn unscale(&self, n: f64) -> Self {
        Matrix::unscale(self, n)
    }
}

/// `value` of norm 1.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Unit<T> {
    pub(crate) value: T,
}

impl<T> Unit<T> {
    /// `value`, taken to be of norm 1 already.
    pub const fn new_unchecked(value: T) -> Self {
        Unit { value }
    }
    /// The value.
    pub fn into_inner(self) -> T {
        self.value
    }
    /// The value (nalgebra's deprecated spelling).
    pub fn unwrap(self) -> T {
        self.value
    }
    /// The value, mutably; the caller keeps it of norm 1.
    pub fn as_mut_unchecked(&mut self) -> &mut T {
        &mut self.value
    }
    /// A reference to the value, unit-wrapped.
    pub fn from_ref_unchecked(value: &T) -> &T {
        value
    }
}

impl<T: Normed> Unit<T> {
    /// `value` divided by its norm.
    pub fn new_normalize(value: T) -> Self {
        Self::new_and_get(value).0
    }
    /// `value` divided by its norm, and that norm.
    pub fn new_and_get(value: T) -> (Self, f64) {
        let n = value.norm();
        (Unit { value: value.unscale(n) }, n)
    }
    /// `value` divided by its norm, unless its squared norm is `<= min_norm²`.
    pub fn try_new(value: T, min_norm: f64) -> Option<Self> {
        Self::try_new_and_get(value, min_norm).map(|r| r.0)
    }
    /// [`Unit::try_new`] and the norm.
    pub fn try_new_and_get(value: T, min_norm: f64) -> Option<(Self, f64)> {
        let sq = value.norm_squared();
        if sq > min_norm * min_norm {
            let n = sq.sqrt();
            Some((Unit { value: value.unscale(n) }, n))
        } else {
            None
        }
    }
    /// Normalized again (after drift); returns the norm it had.
    pub fn renormalize(&mut self) -> f64 {
        let n = self.value.norm();
        self.value = self.value.unscale(n);
        n
    }
}

impl<T> Deref for Unit<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.value
    }
}

impl<T> AsRef<T> for Unit<T> {
    fn as_ref(&self) -> &T {
        &self.value
    }
}

impl<const R: usize, const C: usize> Neg for Unit<Matrix<f64, R, C>>
where
    Sh: Shape<f64, R, C>,
{
    type Output = Self;
    fn neg(self) -> Self {
        Unit { value: -self.value }
    }
}
impl<const R: usize, const C: usize> Neg for &Unit<Matrix<f64, R, C>>
where
    Sh: Shape<f64, R, C>,
{
    type Output = Unit<Matrix<f64, R, C>>;
    fn neg(self) -> Unit<Matrix<f64, R, C>> {
        Unit { value: -self.value }
    }
}

impl<const R: usize, const C: usize> Mul<f64> for Unit<Matrix<f64, R, C>>
where
    Sh: Shape<f64, R, C>,
{
    type Output = Matrix<f64, R, C>;
    fn mul(self, k: f64) -> Matrix<f64, R, C> {
        self.value * k
    }
}
impl<const R: usize, const C: usize> Mul<f64> for &Unit<Matrix<f64, R, C>>
where
    Sh: Shape<f64, R, C>,
{
    type Output = Matrix<f64, R, C>;
    fn mul(self, k: f64) -> Matrix<f64, R, C> {
        self.value * k
    }
}
impl<const R: usize, const C: usize> Mul<Unit<Matrix<f64, R, C>>> for f64
where
    Sh: Shape<f64, R, C>,
{
    type Output = Matrix<f64, R, C>;
    fn mul(self, u: Unit<Matrix<f64, R, C>>) -> Matrix<f64, R, C> {
        self * u.value
    }
}
impl<'a, const R: usize, const C: usize> Mul<&'a Unit<Matrix<f64, R, C>>> for f64
where
    Sh: Shape<f64, R, C>,
{
    type Output = Matrix<f64, R, C>;
    fn mul(self, u: &'a Unit<Matrix<f64, R, C>>) -> Matrix<f64, R, C> {
        self * u.value
    }
}
