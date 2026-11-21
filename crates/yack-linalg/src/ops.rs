// yack: operators on the nalgebra-shaped fixed-size matrices (f64).
//! `+ − *` between matrices, with scalars, owned or borrowed, as nalgebra allows them. A product's
//! element `(i, j)` is summed over `k` in order, as nalgebra's gemm does for these sizes.

use core::iter::Sum;
use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use super::matrix::Matrix;
use super::storage::{Sh, Shape};

macro_rules! elementwise {
    ($Tr:ident, $f:ident, $TrA:ident, $fa:ident, $op:tt) => {
        impl<const R: usize, const C: usize> $Tr<Matrix<f64, R, C>> for Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            type Output = Matrix<f64, R, C>;
            fn $f(self, b: Matrix<f64, R, C>) -> Matrix<f64, R, C> {
                Matrix::from_fn(|r, c| self[(r, c)] $op b[(r, c)])
            }
        }
        impl<'a, const R: usize, const C: usize> $Tr<&'a Matrix<f64, R, C>> for Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            type Output = Matrix<f64, R, C>;
            fn $f(self, b: &'a Matrix<f64, R, C>) -> Matrix<f64, R, C> {
                self $op *b
            }
        }
        impl<'a, const R: usize, const C: usize> $Tr<Matrix<f64, R, C>> for &'a Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            type Output = Matrix<f64, R, C>;
            fn $f(self, b: Matrix<f64, R, C>) -> Matrix<f64, R, C> {
                *self $op b
            }
        }
        impl<'a, 'b, const R: usize, const C: usize> $Tr<&'b Matrix<f64, R, C>> for &'a Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            type Output = Matrix<f64, R, C>;
            fn $f(self, b: &'b Matrix<f64, R, C>) -> Matrix<f64, R, C> {
                *self $op *b
            }
        }
        impl<const R: usize, const C: usize> $TrA<Matrix<f64, R, C>> for Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            fn $fa(&mut self, b: Matrix<f64, R, C>) {
                *self = *self $op b;
            }
        }
        impl<'a, const R: usize, const C: usize> $TrA<&'a Matrix<f64, R, C>> for Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            fn $fa(&mut self, b: &'a Matrix<f64, R, C>) {
                *self = *self $op *b;
            }
        }
    };
}
elementwise!(Add, add, AddAssign, add_assign, +);
elementwise!(Sub, sub, SubAssign, sub_assign, -);

impl<const R: usize, const C: usize> Neg for Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C>,
{
    type Output = Matrix<f64, R, C>;
    fn neg(self) -> Matrix<f64, R, C> {
        self.map(|v| -v)
    }
}
impl<const R: usize, const C: usize> Neg for &Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C>,
{
    type Output = Matrix<f64, R, C>;
    fn neg(self) -> Matrix<f64, R, C> {
        self.map(|v| -v)
    }
}

macro_rules! scalar {
    ($Tr:ident, $f:ident, $TrA:ident, $fa:ident, $op:tt) => {
        impl<const R: usize, const C: usize> $Tr<f64> for Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            type Output = Matrix<f64, R, C>;
            fn $f(self, k: f64) -> Matrix<f64, R, C> {
                self.map(|v| v $op k)
            }
        }
        impl<const R: usize, const C: usize> $Tr<f64> for &Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            type Output = Matrix<f64, R, C>;
            fn $f(self, k: f64) -> Matrix<f64, R, C> {
                self.map(|v| v $op k)
            }
        }
        impl<const R: usize, const C: usize> $TrA<f64> for Matrix<f64, R, C>
        where
            Sh: Shape<f64, R, C>,
        {
            fn $fa(&mut self, k: f64) {
                *self = self.map(|v| v $op k);
            }
        }
    };
}
scalar!(Mul, mul, MulAssign, mul_assign, *);
scalar!(Div, div, DivAssign, div_assign, /);

impl<const R: usize, const C: usize> Mul<Matrix<f64, R, C>> for f64
where
    Sh: Shape<f64, R, C>,
{
    type Output = Matrix<f64, R, C>;
    fn mul(self, m: Matrix<f64, R, C>) -> Matrix<f64, R, C> {
        m.map(|v| self * v)
    }
}
impl<'a, const R: usize, const C: usize> Mul<&'a Matrix<f64, R, C>> for f64
where
    Sh: Shape<f64, R, C>,
{
    type Output = Matrix<f64, R, C>;
    fn mul(self, m: &'a Matrix<f64, R, C>) -> Matrix<f64, R, C> {
        m.map(|v| self * v)
    }
}

/// `a b` with each element summed over `k` in order.
pub(crate) fn matmul<const R: usize, const K: usize, const C: usize>(
    a: &Matrix<f64, R, K>,
    b: &Matrix<f64, K, C>,
) -> Matrix<f64, R, C>
where
    Sh: Shape<f64, R, K> + Shape<f64, K, C> + Shape<f64, R, C>,
{
    Matrix::from_fn(|i, j| {
        if K == 0 {
            return 0.0;
        }
        let mut s = a[(i, 0)] * b[(0, j)];
        for k in 1..K {
            s += a[(i, k)] * b[(k, j)];
        }
        s
    })
}

macro_rules! matmul_impl {
    ($($lt:lifetime)*; $A:ty, $B:ty) => {
        impl<$($lt,)* const R: usize, const K: usize, const C: usize> Mul<$B> for $A
        where
            Sh: Shape<f64, R, K> + Shape<f64, K, C> + Shape<f64, R, C>,
        {
            type Output = Matrix<f64, R, C>;
            fn mul(self, b: $B) -> Matrix<f64, R, C> {
                #[allow(clippy::needless_borrow)]
                matmul(&self, &b)
            }
        }
    };
}
matmul_impl!(; Matrix<f64, R, K>, Matrix<f64, K, C>);
matmul_impl!('b; Matrix<f64, R, K>, &'b Matrix<f64, K, C>);
matmul_impl!('a; &'a Matrix<f64, R, K>, Matrix<f64, K, C>);
matmul_impl!('a 'b; &'a Matrix<f64, R, K>, &'b Matrix<f64, K, C>);

impl<const R: usize, const C: usize> MulAssign<Matrix<f64, C, C>> for Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C> + Shape<f64, C, C>,
{
    fn mul_assign(&mut self, b: Matrix<f64, C, C>) {
        *self = matmul(self, &b);
    }
}
impl<'a, const R: usize, const C: usize> MulAssign<&'a Matrix<f64, C, C>> for Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C> + Shape<f64, C, C>,
{
    fn mul_assign(&mut self, b: &'a Matrix<f64, C, C>) {
        *self = matmul(self, b);
    }
}

impl<const R: usize, const C: usize> Sum for Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C>,
{
    fn sum<I: Iterator<Item = Self>>(it: I) -> Self {
        it.fold(Self::zeros(), |s, v| s + v)
    }
}
impl<'a, const R: usize, const C: usize> Sum<&'a Matrix<f64, R, C>> for Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C>,
{
    fn sum<I: Iterator<Item = &'a Matrix<f64, R, C>>>(it: I) -> Self {
        it.fold(Self::zeros(), |s, v| s + v)
    }
}
