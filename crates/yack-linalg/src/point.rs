// yack: the nalgebra-shaped point: a position, told apart from a displacement by type.

use core::fmt;
use core::ops::{
    Add, AddAssign, Deref, DerefMut, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub,
    SubAssign,
};

use super::matrix::Matrix;
use super::ops::matmul;
use super::storage::{Scalar, Sh, Shape, XY, XYZ, XYZW};

/// A point in `D` dimensions: `coords` from the origin.
#[derive(Clone, Copy, PartialEq)]
pub struct Point<T, const D: usize>
where
    Sh: Shape<T, D, 1>,
{
    /// The coordinates.
    pub coords: Matrix<T, D, 1>,
}

pub type Point1<T> = Point<T, 1>;
pub type Point2<T> = Point<T, 2>;
pub type Point3<T> = Point<T, 3>;
pub type Point4<T> = Point<T, 4>;
/// nalgebra's `OPoint` for fixed sizes.
pub type OPoint<T, const D: usize> = Point<T, D>;

impl<T: Scalar, const D: usize> fmt::Debug for Point<T, D>
where
    Sh: Shape<T, D, 1>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let c: Vec<T> = self.coords.iter().copied().collect();
        f.debug_struct("Point").field("coords", &c).finish()
    }
}

impl<T: Scalar + fmt::Display, const D: usize> fmt::Display for Point<T, D>
where
    Sh: Shape<T, D, 1>,
{
    #[allow(clippy::indexing_slicing)] fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{{")?;
        for i in 0..D {
            if i > 0 {
                write!(f, ", ")?;
            }
            fmt::Display::fmt(&self.coords[i], f)?;
        }
        write!(f, "}}")
    }
}

impl<T: Scalar, const D: usize> Point<T, D>
where
    Sh: Shape<T, D, 1>,
{
    /// The point at `coords`.
    pub fn from_coords(coords: Matrix<T, D, 1>) -> Self {
        Self { coords }
    }
    /// The point with the first `D` elements of `s`.
    pub fn from_slice(s: &[T]) -> Self {
        Self { coords: Matrix::from_column_slice(s) }
    }
    /// The coordinates, in order.
    pub fn iter(&self) -> impl DoubleEndedIterator<Item = &T> + ExactSizeIterator + Clone + '_ {
        self.coords.iter()
    }
    /// The coordinates, mutably.
    pub fn iter_mut(&mut self) -> std::vec::IntoIter<&mut T> {
        self.coords.iter_mut()
    }
    /// `f` of each coordinate.
    pub fn map<U: Scalar>(&self, f: impl FnMut(T) -> U) -> Point<U, D>
    where
        Sh: Shape<U, D, 1>,
    {
        Point { coords: self.coords.map(f) }
    }
    /// Each coordinate replaced by `f` of it.
    pub fn apply(&mut self, f: impl FnMut(&mut T)) {
        self.coords.apply(f)
    }
    /// `D`.
    pub fn len(&self) -> usize {
        D
    }
    /// Whether `D` is 0.
    pub fn is_empty(&self) -> bool {
        D == 0
    }
}

impl<const D: usize> Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    /// The origin.
    pub fn origin() -> Self {
        Self { coords: Matrix::zeros() }
    }
    /// `self (1 − t) + b t`.
    pub fn lerp(&self, b: &Self, t: f64) -> Self {
        Self { coords: self.coords.lerp(&b.coords, t) }
    }
    /// Whether every coordinate is finite.
    pub fn is_finite(&self) -> bool {
        self.coords.is_finite()
    }
    /// The same point (nalgebra's numeric cast, f64 to f64).
    pub fn cast<U: From<f64> + Scalar>(&self) -> Point<U, D>
    where
        Sh: Shape<U, D, 1>,
    {
        Point { coords: self.coords.cast() }
    }
    /// Coordinate by coordinate maximum.
    pub fn sup(&self, b: &Self) -> Self {
        Self { coords: self.coords.sup(&b.coords) }
    }
    /// Coordinate by coordinate minimum.
    pub fn inf(&self, b: &Self) -> Self {
        Self { coords: self.coords.inf(&b.coords) }
    }
}

impl<T: Scalar> Point<T, 1> {
    pub fn new(x: T) -> Self {
        Self { coords: Matrix::<T, 1, 1>::new(x) }
    }
}
impl<T: Scalar> Point<T, 2> {
    pub fn new(x: T, y: T) -> Self {
        Self { coords: Matrix::<T, 2, 1>::new(x, y) }
    }
}
impl<T: Scalar> Point<T, 3> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { coords: Matrix::<T, 3, 1>::new(x, y, z) }
    }
}
impl<T: Scalar> Point<T, 4> {
    pub fn new(x: T, y: T, z: T, w: T) -> Self {
        Self { coords: Matrix::<T, 4, 1>::new(x, y, z, w) }
    }
}

impl Point<f64, 2> {
    /// (x, y, 1).
    pub fn to_homogeneous(&self) -> Matrix<f64, 3, 1> {
        self.coords.push(1.0)
    }
    /// (x, y).
    pub fn xy(&self) -> Self {
        *self
    }
}
impl Point<f64, 3> {
    /// (x, y, z, 1).
    pub fn to_homogeneous(&self) -> Matrix<f64, 4, 1> {
        self.coords.push(1.0)
    }
    /// (x, y).
    pub fn xy(&self) -> Point<f64, 2> {
        Point::<f64, 2>::new(self.x, self.y)
    }
    /// (x, z).
    pub fn xz(&self) -> Point<f64, 2> {
        Point::<f64, 2>::new(self.x, self.z)
    }
    /// (y, z).
    pub fn yz(&self) -> Point<f64, 2> {
        Point::<f64, 2>::new(self.y, self.z)
    }
    /// Itself.
    pub fn xyz(&self) -> Self {
        *self
    }
    /// The point of homogeneous coordinates `v` (`None` when `w` is 0).
    pub fn from_homogeneous(v: Matrix<f64, 4, 1>) -> Option<Self> {
        if v.w == 0.0 { None } else { Some(Self::new(v.x / v.w, v.y / v.w, v.z / v.w)) }
    }
}

impl<T: Scalar> Deref for Point<T, 2> {
    type Target = XY<T>;
    fn deref(&self) -> &XY<T> {
        &self.coords.data
    }
}
impl<T: Scalar> DerefMut for Point<T, 2> {
    fn deref_mut(&mut self) -> &mut XY<T> {
        &mut self.coords.data
    }
}
impl<T: Scalar> Deref for Point<T, 3> {
    type Target = XYZ<T>;
    fn deref(&self) -> &XYZ<T> {
        &self.coords.data
    }
}
impl<T: Scalar> DerefMut for Point<T, 3> {
    fn deref_mut(&mut self) -> &mut XYZ<T> {
        &mut self.coords.data
    }
}
impl<T: Scalar> Deref for Point<T, 4> {
    type Target = XYZW<T>;
    fn deref(&self) -> &XYZW<T> {
        &self.coords.data
    }
}
impl<T: Scalar> DerefMut for Point<T, 4> {
    fn deref_mut(&mut self) -> &mut XYZW<T> {
        &mut self.coords.data
    }
}

impl<T: Scalar, const D: usize> Index<usize> for Point<T, D>
where
    Sh: Shape<T, D, 1>,
{
    type Output = T;
    #[allow(clippy::indexing_slicing)] fn index(&self, i: usize) -> &T {
        &self.coords[i]
    }
}
impl<T: Scalar, const D: usize> IndexMut<usize> for Point<T, D>
where
    Sh: Shape<T, D, 1>,
{
    #[allow(clippy::indexing_slicing)] fn index_mut(&mut self, i: usize) -> &mut T {
        &mut self.coords[i]
    }
}

impl<T: Scalar, const D: usize> From<Matrix<T, D, 1>> for Point<T, D>
where
    Sh: Shape<T, D, 1>,
{
    fn from(coords: Matrix<T, D, 1>) -> Self {
        Self { coords }
    }
}
impl<T: Scalar, const D: usize> From<[T; D]> for Point<T, D>
where
    Sh: Shape<T, D, 1>,
{
    fn from(a: [T; D]) -> Self {
        Self { coords: Matrix::from(a) }
    }
}
impl<T: Scalar, const D: usize> From<Point<T, D>> for [T; D]
where
    Sh: Shape<T, D, 1>,
{
    fn from(p: Point<T, D>) -> Self {
        p.coords.into()
    }
}
impl From<Point<f64, 3>> for Matrix<f64, 4, 1> {
    fn from(p: Point<f64, 3>) -> Self {
        p.to_homogeneous()
    }
}
impl From<Point<f64, 2>> for Matrix<f64, 3, 1> {
    fn from(p: Point<f64, 2>) -> Self {
        p.to_homogeneous()
    }
}

/// `(a + b) / 2`, as nalgebra's `center`: `(a + b) * 0.5`.
pub fn center<const D: usize>(a: &Point<f64, D>, b: &Point<f64, D>) -> Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    Point { coords: (a.coords + b.coords) * 0.5 }
}

/// `|b − a|`.
pub fn distance<const D: usize>(a: &Point<f64, D>, b: &Point<f64, D>) -> f64
where
    Sh: Shape<f64, D, 1>,
{
    (b.coords - a.coords).norm()
}

/// `|b − a|²`.
pub fn distance_squared<const D: usize>(a: &Point<f64, D>, b: &Point<f64, D>) -> f64
where
    Sh: Shape<f64, D, 1>,
{
    (b.coords - a.coords).norm_squared()
}

// ---- operators ----

macro_rules! by_ref {
    ($Tr:ident, $f:ident, $L:ty, $Rhs:ty, $Out:ty, |$a:ident, $b:ident| $e:expr) => {
        impl<const D: usize> $Tr<$Rhs> for $L
        where
            Sh: Shape<f64, D, 1>,
        {
            type Output = $Out;
            fn $f(self, $b: $Rhs) -> $Out {
                let $a = self;
                $e
            }
        }
        impl<'a, const D: usize> $Tr<&'a $Rhs> for $L
        where
            Sh: Shape<f64, D, 1>,
        {
            type Output = $Out;
            fn $f(self, $b: &'a $Rhs) -> $Out {
                let $a = self;
                let $b = *$b;
                $e
            }
        }
        impl<'a, const D: usize> $Tr<$Rhs> for &'a $L
        where
            Sh: Shape<f64, D, 1>,
        {
            type Output = $Out;
            fn $f(self, $b: $Rhs) -> $Out {
                let $a = *self;
                $e
            }
        }
        impl<'a, 'b, const D: usize> $Tr<&'b $Rhs> for &'a $L
        where
            Sh: Shape<f64, D, 1>,
        {
            type Output = $Out;
            fn $f(self, $b: &'b $Rhs) -> $Out {
                let $a = *self;
                let $b = *$b;
                $e
            }
        }
    };
}

by_ref!(Sub, sub, Point<f64, D>, Point<f64, D>, Matrix<f64, D, 1>, |a, b| a.coords - b.coords);
by_ref!(Add, add, Point<f64, D>, Matrix<f64, D, 1>, Point<f64, D>, |a, b| Point { coords: a.coords + b });
by_ref!(Sub, sub, Point<f64, D>, Matrix<f64, D, 1>, Point<f64, D>, |a, b| Point { coords: a.coords - b });

impl<const D: usize> AddAssign<Matrix<f64, D, 1>> for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    fn add_assign(&mut self, v: Matrix<f64, D, 1>) {
        self.coords += v;
    }
}
impl<'a, const D: usize> AddAssign<&'a Matrix<f64, D, 1>> for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    fn add_assign(&mut self, v: &'a Matrix<f64, D, 1>) {
        self.coords += v;
    }
}
impl<const D: usize> SubAssign<Matrix<f64, D, 1>> for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    fn sub_assign(&mut self, v: Matrix<f64, D, 1>) {
        self.coords -= v;
    }
}
impl<'a, const D: usize> SubAssign<&'a Matrix<f64, D, 1>> for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    fn sub_assign(&mut self, v: &'a Matrix<f64, D, 1>) {
        self.coords -= v;
    }
}

impl<const D: usize> Mul<f64> for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn mul(self, k: f64) -> Point<f64, D> {
        Point { coords: self.coords * k }
    }
}
impl<const D: usize> Mul<f64> for &Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn mul(self, k: f64) -> Point<f64, D> {
        Point { coords: self.coords * k }
    }
}
impl<const D: usize> Mul<Point<f64, D>> for f64
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn mul(self, p: Point<f64, D>) -> Point<f64, D> {
        Point { coords: self * p.coords }
    }
}
impl<'a, const D: usize> Mul<&'a Point<f64, D>> for f64
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn mul(self, p: &'a Point<f64, D>) -> Point<f64, D> {
        Point { coords: self * p.coords }
    }
}
impl<const D: usize> Div<f64> for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn div(self, k: f64) -> Point<f64, D> {
        Point { coords: self.coords / k }
    }
}
impl<const D: usize> Div<f64> for &Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn div(self, k: f64) -> Point<f64, D> {
        Point { coords: self.coords / k }
    }
}
impl<const D: usize> MulAssign<f64> for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    fn mul_assign(&mut self, k: f64) {
        self.coords *= k;
    }
}
impl<const D: usize> DivAssign<f64> for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    fn div_assign(&mut self, k: f64) {
        self.coords /= k;
    }
}
impl<const D: usize> Neg for Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn neg(self) -> Point<f64, D> {
        Point { coords: -self.coords }
    }
}
impl<const D: usize> Neg for &Point<f64, D>
where
    Sh: Shape<f64, D, 1>,
{
    type Output = Point<f64, D>;
    fn neg(self) -> Point<f64, D> {
        Point { coords: -self.coords }
    }
}

macro_rules! mat_point {
    ($($lt:lifetime)*; $A:ty, $P:ty) => {
        impl<$($lt,)* const R: usize, const C: usize> Mul<$P> for $A
        where
            Sh: Shape<f64, R, C> + Shape<f64, C, 1> + Shape<f64, R, 1>,
        {
            type Output = Point<f64, R>;
            fn mul(self, p: $P) -> Point<f64, R> {
                #[allow(clippy::needless_borrow)]
                Point { coords: matmul(&self, &p.coords) }
            }
        }
    };
}
mat_point!(; Matrix<f64, R, C>, Point<f64, C>);
mat_point!('b; Matrix<f64, R, C>, &'b Point<f64, C>);
mat_point!('a; &'a Matrix<f64, R, C>, Point<f64, C>);
mat_point!('a 'b; &'a Matrix<f64, R, C>, &'b Point<f64, C>);
