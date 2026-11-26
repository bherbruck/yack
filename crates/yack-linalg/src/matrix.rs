// yack: the nalgebra-shaped fixed-size matrix: construction, access and the f64 numerics.
//! `Matrix<T, R, C>` is nalgebra's `SMatrix<T, R, C>`: column-major, `R` rows and `C` columns.
//! The arithmetic follows nalgebra's evaluation order (sums run in index order, `normalize`
//! divides by the norm, `try_normalize` refuses norms `<=` the bound, `Unit::try_new` compares
//! squared norms) so that results match it to the last bit wherever the formulas are the same.

use core::fmt;
use core::ops::{Deref, DerefMut, Index, IndexMut};

use super::storage::{Scalar, Sh, Shape, Storage, XY, XYZ, XYZW};
use super::unit::Unit;

/// A fixed-size `R × C` matrix of `T`, stored column by column.
#[derive(Clone, Copy, PartialEq)]
pub struct Matrix<T, const R: usize, const C: usize>
where
    Sh: Shape<T, R, C>,
{
    /// The elements.
    pub data: <Sh as Shape<T, R, C>>::S,
}

/// nalgebra's `SMatrix`.
pub type SMatrix<T, const R: usize, const C: usize> = Matrix<T, R, C>;
/// nalgebra's `SVector`: a column.
pub type SVector<T, const D: usize> = Matrix<T, D, 1>;
/// A row.
pub type RowSVector<T, const D: usize> = Matrix<T, 1, D>;

pub type Vector1<T> = Matrix<T, 1, 1>;
pub type Vector2<T> = Matrix<T, 2, 1>;
pub type Vector3<T> = Matrix<T, 3, 1>;
pub type Vector4<T> = Matrix<T, 4, 1>;
pub type Vector5<T> = Matrix<T, 5, 1>;
pub type Vector6<T> = Matrix<T, 6, 1>;
pub type RowVector2<T> = Matrix<T, 1, 2>;
pub type RowVector3<T> = Matrix<T, 1, 3>;
pub type RowVector4<T> = Matrix<T, 1, 4>;
pub type Matrix1<T> = Matrix<T, 1, 1>;
pub type Matrix2<T> = Matrix<T, 2, 2>;
pub type Matrix3<T> = Matrix<T, 3, 3>;
pub type Matrix4<T> = Matrix<T, 4, 4>;
pub type Matrix5<T> = Matrix<T, 5, 5>;
pub type Matrix6<T> = Matrix<T, 6, 6>;
pub type Matrix2x3<T> = Matrix<T, 2, 3>;
pub type Matrix2x4<T> = Matrix<T, 2, 4>;
pub type Matrix3x2<T> = Matrix<T, 3, 2>;
pub type Matrix3x4<T> = Matrix<T, 3, 4>;
pub type Matrix4x3<T> = Matrix<T, 4, 3>;
pub type Matrix6x4<T> = Matrix<T, 6, 4>;
pub type Matrix4x6<T> = Matrix<T, 4, 6>;
pub type Matrix3x6<T> = Matrix<T, 3, 6>;
pub type Matrix6x3<T> = Matrix<T, 6, 3>;

impl<T: Scalar, const R: usize, const C: usize> fmt::Debug for Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let cols: Vec<Vec<T>> = (0..C).map(|c| (0..R).map(|r| self[(r, c)]).collect()).collect();
        f.debug_struct("Matrix").field("data", &cols).finish()
    }
}

impl<T: Scalar + fmt::Display, const R: usize, const C: usize> fmt::Display for Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for r in 0..R {
            write!(f, "[")?;
            for c in 0..C {
                if c > 0 {
                    write!(f, ", ")?;
                }
                fmt::Display::fmt(&self[(r, c)], f)?;
            }
            writeln!(f, "]")?;
        }
        Ok(())
    }
}

// ---- access, for any element type ----

impl<T: Scalar, const R: usize, const C: usize> Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    /// The matrix with `f(row, column)` at each place.
    pub fn from_fn(mut f: impl FnMut(usize, usize) -> T) -> Self {
        Self { data: <Sh as Shape<T, R, C>>::S::from_lin(|i| f(i % R, i / R)) }
    }
    /// Every element `v`.
    pub fn from_element(v: T) -> Self {
        Self::from_fn(|_, _| v)
    }
    /// Every element `v`.
    pub fn repeat(v: T) -> Self {
        Self::from_element(v)
    }
    /// The first `R × C` elements of `s`, column by column (panics if `s` is shorter).
    #[allow(clippy::indexing_slicing)] pub fn from_column_slice(s: &[T]) -> Self {
        assert!(s.len() >= R * C, "from_column_slice: slice too short");
        Self { data: <Sh as Shape<T, R, C>>::S::from_lin(|i| s[i]) }
    }
    /// The first `R × C` elements of `s`, row by row (panics if `s` is shorter).
    #[allow(clippy::indexing_slicing)] pub fn from_row_slice(s: &[T]) -> Self {
        assert!(s.len() >= R * C, "from_row_slice: slice too short");
        Self::from_fn(|r, c| s[r * C + c])
    }
    /// `R`.
    pub fn nrows(&self) -> usize {
        R
    }
    /// `C`.
    pub fn ncols(&self) -> usize {
        C
    }
    /// `(R, C)`.
    pub fn shape(&self) -> (usize, usize) {
        (R, C)
    }
    /// The number of elements.
    pub fn len(&self) -> usize {
        R * C
    }
    /// Whether there are no elements.
    pub fn is_empty(&self) -> bool {
        R * C == 0
    }
    /// The element at linear (column-major) index `i`, if there is one.
    pub fn get(&self, i: usize) -> Option<&T> {
        (i < R * C).then(|| self.data.at(i))
    }
    /// The elements, column by column.
    pub fn iter(&self) -> impl DoubleEndedIterator<Item = &T> + ExactSizeIterator + Clone + '_ {
        (0..R * C).map(move |i| self.data.at(i))
    }
    /// The elements, column by column, mutably.
    pub fn iter_mut(&mut self) -> std::vec::IntoIter<&mut T> {
        self.data.all_mut().into_iter()
    }
    /// The elements as a vector, column by column.
    pub fn to_vec(&self) -> Vec<T> {
        self.iter().copied().collect()
    }
    /// `f` of each element.
    pub fn map<U: Scalar>(&self, mut f: impl FnMut(T) -> U) -> Matrix<U, R, C>
    where
        Sh: Shape<U, R, C>,
    {
        Matrix::from_fn(|r, c| f(self[(r, c)]))
    }
    /// Each element replaced by `f` of it.
    pub fn apply(&mut self, mut f: impl FnMut(&mut T)) {
        for v in self.data.all_mut() {
            f(v);
        }
    }
    /// `f` of each pair of elements.
    pub fn zip_map<U: Scalar, V: Scalar>(
        &self,
        other: &Matrix<U, R, C>,
        mut f: impl FnMut(T, U) -> V,
    ) -> Matrix<V, R, C>
    where
        Sh: Shape<U, R, C> + Shape<V, R, C>,
    {
        Matrix::from_fn(|r, c| f(self[(r, c)], other[(r, c)]))
    }
    /// The transpose.
    pub fn transpose(&self) -> Matrix<T, C, R>
    where
        Sh: Shape<T, C, R>,
    {
        Matrix::from_fn(|r, c| self[(c, r)])
    }
    /// Column `j`, as a vector of its own (nalgebra returns a view; `into_owned` it the same).
    pub fn column(&self, j: usize) -> Matrix<T, R, 1>
    where
        Sh: Shape<T, R, 1>,
    {
        Matrix::from_fn(|r, _| self[(r, j)])
    }
    /// Row `i`, as a row of its own.
    pub fn row(&self, i: usize) -> Matrix<T, 1, C>
    where
        Sh: Shape<T, 1, C>,
    {
        Matrix::from_fn(|_, c| self[(i, c)])
    }
    /// Sets column `j`.
    pub fn set_column(&mut self, j: usize, v: &Matrix<T, R, 1>)
    where
        Sh: Shape<T, R, 1>,
    {
        for r in 0..R {
            self[(r, j)] = v[r];
        }
    }
    /// Sets row `i`.
    pub fn set_row(&mut self, i: usize, v: &Matrix<T, 1, C>)
    where
        Sh: Shape<T, 1, C>,
    {
        for c in 0..C {
            self[(i, c)] = v[c];
        }
    }
    /// Every element `v`.
    pub fn fill(&mut self, v: T) {
        for e in self.data.all_mut() {
            *e = v;
        }
    }
    /// Rows `a` and `b` swapped.
    pub fn swap_rows(&mut self, a: usize, b: usize) {
        if a != b {
            for c in 0..C {
                let t = self[(a, c)];
                self[(a, c)] = self[(b, c)];
                self[(b, c)] = t;
            }
        }
    }
    /// Columns `a` and `b` swapped.
    pub fn swap_columns(&mut self, a: usize, b: usize) {
        if a != b {
            for r in 0..R {
                let t = self[(r, a)];
                self[(r, a)] = self[(r, b)];
                self[(r, b)] = t;
            }
        }
    }
    /// Elements `a` and `b` (row, column) swapped.
    pub fn swap(&mut self, a: (usize, usize), b: (usize, usize)) {
        let t = self[a];
        self[a] = self[b];
        self[b] = t;
    }
    /// Itself (nalgebra's views become owned matrices with this).
    pub fn into_owned(self) -> Self {
        self
    }
    /// A copy.
    pub fn clone_owned(&self) -> Self {
        *self
    }
}

impl<T: Scalar, const R: usize, const C: usize> Index<usize> for Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    type Output = T;
    fn index(&self, i: usize) -> &T {
        self.data.at(i)
    }
}

impl<T: Scalar, const R: usize, const C: usize> IndexMut<usize> for Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    fn index_mut(&mut self, i: usize) -> &mut T {
        self.data.at_mut(i)
    }
}

impl<T: Scalar, const R: usize, const C: usize> Index<(usize, usize)> for Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    type Output = T;
    fn index(&self, (r, c): (usize, usize)) -> &T {
        assert!(r < R && c < C, "matrix index ({r}, {c}) out of bounds ({R} × {C})");
        self.data.at(r + c * R)
    }
}

impl<T: Scalar, const R: usize, const C: usize> IndexMut<(usize, usize)> for Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut T {
        assert!(r < R && c < C, "matrix index ({r}, {c}) out of bounds ({R} × {C})");
        self.data.at_mut(r + c * R)
    }
}

impl<T: Scalar> Deref for Matrix<T, 2, 1> {
    type Target = XY<T>;
    fn deref(&self) -> &XY<T> {
        &self.data
    }
}
impl<T: Scalar> DerefMut for Matrix<T, 2, 1> {
    fn deref_mut(&mut self) -> &mut XY<T> {
        &mut self.data
    }
}
impl<T: Scalar> Deref for Matrix<T, 3, 1> {
    type Target = XYZ<T>;
    fn deref(&self) -> &XYZ<T> {
        &self.data
    }
}
impl<T: Scalar> DerefMut for Matrix<T, 3, 1> {
    fn deref_mut(&mut self) -> &mut XYZ<T> {
        &mut self.data
    }
}
impl<T: Scalar> Deref for Matrix<T, 4, 1> {
    type Target = XYZW<T>;
    fn deref(&self) -> &XYZW<T> {
        &self.data
    }
}
impl<T: Scalar> DerefMut for Matrix<T, 4, 1> {
    fn deref_mut(&mut self) -> &mut XYZW<T> {
        &mut self.data
    }
}

impl<T: Scalar, const D: usize> From<[T; D]> for Matrix<T, D, 1>
where
    Sh: Shape<T, D, 1>,
{
    #[allow(clippy::indexing_slicing)] fn from(a: [T; D]) -> Self {
        Self::from_fn(|r, _| a[r])
    }
}

impl<T: Scalar, const D: usize> From<Matrix<T, D, 1>> for [T; D]
where
    Sh: Shape<T, D, 1>,
{
    #[allow(clippy::indexing_slicing)] fn from(m: Matrix<T, D, 1>) -> Self {
        core::array::from_fn(|i| m[i])
    }
}

impl<T: Scalar, const R: usize, const C: usize> From<[[T; R]; C]> for Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    #[allow(clippy::indexing_slicing)] fn from(a: [[T; R]; C]) -> Self {
        Self::from_fn(|r, c| a[c][r])
    }
}

impl<T: Scalar, const R: usize, const C: usize> AsRef<Matrix<T, R, C>> for Matrix<T, R, C>
where
    Sh: Shape<T, R, C>,
{
    fn as_ref(&self) -> &Self {
        self
    }
}

/// `as_slice` for the shapes stored as arrays (all but the 2-, 3- and 4-element columns, whose
/// named fields can't be viewed as a slice without `unsafe`: those have `to_array`).
macro_rules! array_as_slice {
    ($(($r:literal, $c:literal)),* $(,)?) => {
        $(impl<T: Scalar> Matrix<T, $r, $c> {
            /// The elements, column by column.
            pub fn as_slice(&self) -> &[T] {
                self.data.as_flattened()
            }
            /// The elements, column by column, mutably.
            pub fn as_mut_slice(&mut self) -> &mut [T] {
                self.data.as_flattened_mut()
            }
        })*
    };
}
array_as_slice!(
    (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6),
    (5, 1), (6, 1),
    (2, 2), (2, 3), (2, 4), (2, 5), (2, 6),
    (3, 2), (3, 3), (3, 4), (3, 5), (3, 6),
    (4, 2), (4, 3), (4, 4), (4, 5), (4, 6),
    (5, 2), (5, 3), (5, 4), (5, 5), (5, 6),
    (6, 2), (6, 3), (6, 4), (6, 5), (6, 6),
);

impl<T: Scalar, const D: usize> Matrix<T, D, 1>
where
    Sh: Shape<T, D, 1>,
{
    /// The elements as an array.
    pub fn to_array(&self) -> [T; D] {
        core::array::from_fn(|i| self[i])
    }
}

// ---- f64 numerics, any size ----

impl<const R: usize, const C: usize> Default for Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C>,
{
    fn default() -> Self {
        Self::zeros()
    }
}

impl<const R: usize, const C: usize> Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C>,
{
    /// All zeros.
    pub fn zeros() -> Self {
        Self::from_element(0.0)
    }
    /// Ones on the diagonal, zeros elsewhere.
    pub fn identity() -> Self {
        Self::from_fn(|r, c| if r == c { 1.0 } else { 0.0 })
    }
    /// The columns `cols` side by side (panics if there are fewer than `C`).
    #[allow(clippy::indexing_slicing)] pub fn from_columns(cols: &[Matrix<f64, R, 1>]) -> Self
    where
        Sh: Shape<f64, R, 1>,
    {
        assert!(cols.len() >= C, "from_columns: too few columns");
        Self::from_fn(|r, c| cols[c][r])
    }
    /// The rows `rows` one under another (panics if there are fewer than `R`).
    #[allow(clippy::indexing_slicing)] pub fn from_rows(rows: &[Matrix<f64, 1, C>]) -> Self
    where
        Sh: Shape<f64, 1, C>,
    {
        assert!(rows.len() >= R, "from_rows: too few rows");
        Self::from_fn(|r, c| rows[r][c])
    }
    /// Σ aᵢ bᵢ in index order.
    pub fn dot(&self, b: &Self) -> f64 {
        let mut s = 0.0;
        for i in 0..R * C {
            s += self[i] * b[i];
        }
        s
    }
    /// Σ aᵢ² in index order.
    pub fn norm_squared(&self) -> f64 {
        self.dot(self)
    }
    /// The Euclidean (Frobenius) norm.
    pub fn norm(&self) -> f64 {
        self.norm_squared().sqrt()
    }
    /// The norm (nalgebra's alias).
    pub fn magnitude(&self) -> f64 {
        self.norm()
    }
    /// The squared norm (nalgebra's alias).
    pub fn magnitude_squared(&self) -> f64 {
        self.norm_squared()
    }
    /// Divided by its norm.
    pub fn normalize(&self) -> Self {
        self.unscale(self.norm())
    }
    /// Divided by its norm in place; returns the norm.
    pub fn normalize_mut(&mut self) -> f64 {
        let n = self.norm();
        *self = self.unscale(n);
        n
    }
    /// Divided by its norm, unless that is `<= min_norm`.
    pub fn try_normalize(&self, min_norm: f64) -> Option<Self> {
        let n = self.norm();
        if n <= min_norm { None } else { Some(self.unscale(n)) }
    }
    /// Each element times `k`.
    pub fn scale(&self, k: f64) -> Self {
        self.map(|v| v * k)
    }
    /// Each element divided by `k`.
    pub fn unscale(&self, k: f64) -> Self {
        self.map(|v| v / k)
    }
    /// Each element's absolute value.
    pub fn abs(&self) -> Self {
        self.map(f64::abs)
    }
    /// The largest absolute value (0 for an empty matrix).
    pub fn amax(&self) -> f64 {
        self.iter().fold(0.0, |m, v| if v.abs() > m { v.abs() } else { m })
    }
    /// The smallest absolute value.
    pub fn amin(&self) -> f64 {
        let mut it = self.iter();
        let first = it.next().map_or(0.0, |v| v.abs());
        it.fold(first, |m, v| if v.abs() < m { v.abs() } else { m })
    }
    /// The largest element.
    pub fn max(&self) -> f64 {
        let mut it = self.iter();
        let first = it.next().copied().unwrap_or(0.0);
        it.fold(first, |m, &v| if v > m { v } else { m })
    }
    /// The smallest element.
    pub fn min(&self) -> f64 {
        let mut it = self.iter();
        let first = it.next().copied().unwrap_or(0.0);
        it.fold(first, |m, &v| if v < m { v } else { m })
    }
    /// The index of the first largest element.
    pub fn imax(&self) -> usize {
        self.arg(|v, m| v > m)
    }
    /// The index of the first smallest element.
    pub fn imin(&self) -> usize {
        self.arg(|v, m| v < m)
    }
    /// The index of the first largest absolute value.
    pub fn iamax(&self) -> usize {
        self.map(f64::abs).imax()
    }
    /// The index of the first smallest absolute value.
    pub fn iamin(&self) -> usize {
        self.map(f64::abs).imin()
    }
    fn arg(&self, better: impl Fn(f64, f64) -> bool) -> usize {
        let mut best = 0;
        for i in 1..R * C {
            if better(self[i], self[best]) {
                best = i;
            }
        }
        best
    }
    /// The sum of the elements, in index order.
    pub fn sum(&self) -> f64 {
        self.iter().fold(0.0, |s, v| s + v)
    }
    /// The product of the elements.
    pub fn product(&self) -> f64 {
        self.iter().fold(1.0, |s, v| s * v)
    }
    /// The mean of the elements.
    pub fn mean(&self) -> f64 {
        self.sum() / (R * C) as f64
    }
    /// Element by element product.
    pub fn component_mul(&self, b: &Self) -> Self {
        self.zip_map(b, |x, y| x * y)
    }
    /// Element by element quotient.
    pub fn component_div(&self, b: &Self) -> Self {
        self.zip_map(b, |x, y| x / y)
    }
    /// Element by element maximum.
    pub fn sup(&self, b: &Self) -> Self {
        self.zip_map(b, f64::max)
    }
    /// Element by element minimum.
    pub fn inf(&self, b: &Self) -> Self {
        self.zip_map(b, f64::min)
    }
    /// `self (1 − t) + b t`.
    pub fn lerp(&self, b: &Self, t: f64) -> Self {
        self.zip_map(b, |x, y| x * (1.0 - t) + y * t)
    }
    /// `selfᵀ b`.
    pub fn tr_mul<const C2: usize>(&self, b: &Matrix<f64, R, C2>) -> Matrix<f64, C, C2>
    where
        Sh: Shape<f64, R, C2> + Shape<f64, C, C2>,
    {
        Matrix::from_fn(|i, j| {
            let mut s = 0.0;
            for k in 0..R {
                s += self[(k, i)] * b[(k, j)];
            }
            s
        })
    }
    /// `selfᵀ · b` for same-shaped matrices.
    pub fn tr_dot(&self, b: &Self) -> f64 {
        self.dot(b)
    }
    /// The angle between the two, in [0, π] (0 when either is zero).
    pub fn angle(&self, b: &Self) -> f64 {
        let prod = self.dot(b);
        let n1 = self.norm();
        let n2 = b.norm();
        if n1 == 0.0 || n2 == 0.0 {
            0.0
        } else {
            (prod / (n1 * n2)).clamp(-1.0, 1.0).acos()
        }
    }
    /// Whether every element is within `eps` of the identity's.
    pub fn is_identity(&self, eps: f64) -> bool {
        (0..R).all(|r| (0..C).all(|c| (self[(r, c)] - if r == c { 1.0 } else { 0.0 }).abs() <= eps))
    }
    /// Whether every element is finite.
    pub fn is_finite(&self) -> bool {
        self.iter().all(|v| v.is_finite())
    }
    /// Whether any element is NaN.
    pub fn has_nan(&self) -> bool {
        self.iter().any(|v| v.is_nan())
    }
    /// The same matrix (nalgebra's numeric cast, f64 to f64).
    pub fn cast<U: From<f64> + Scalar>(&self) -> Matrix<U, R, C>
    where
        Sh: Shape<U, R, C>,
    {
        self.map(U::from)
    }
}

// ---- square ----

impl<const D: usize> Matrix<f64, D, D>
where
    Sh: Shape<f64, D, D> + Shape<f64, D, 1>,
{
    /// The diagonal matrix of `d`.
    pub fn from_diagonal(d: &Matrix<f64, D, 1>) -> Self {
        Self::from_fn(|r, c| if r == c { d[r] } else { 0.0 })
    }
    /// The diagonal matrix with `v` on its diagonal.
    pub fn from_diagonal_element(v: f64) -> Self {
        Self::from_fn(|r, c| if r == c { v } else { 0.0 })
    }
    /// The diagonal.
    pub fn diagonal(&self) -> Matrix<f64, D, 1> {
        Matrix::from_fn(|r, _| self[(r, r)])
    }
    /// The sum of the diagonal.
    pub fn trace(&self) -> f64 {
        let mut s = 0.0;
        for i in 0..D {
            s += self[(i, i)];
        }
        s
    }
    /// The determinant: closed form up to 3 × 3, through LU past that (as nalgebra).
    pub fn determinant(&self) -> f64 {
        match D {
            0 => 1.0,
            1 => self[(0, 0)],
            2 => self[(0, 0)] * self[(1, 1)] - self[(1, 0)] * self[(0, 1)],
            3 => {
                let (m11, m12, m13) = (self[(0, 0)], self[(0, 1)], self[(0, 2)]);
                let (m21, m22, m23) = (self[(1, 0)], self[(1, 1)], self[(1, 2)]);
                let (m31, m32, m33) = (self[(2, 0)], self[(2, 1)], self[(2, 2)]);
                let minor_m12_m23 = m22 * m33 - m32 * m23;
                let minor_m11_m23 = m21 * m33 - m31 * m23;
                let minor_m11_m22 = m21 * m32 - m31 * m22;
                m11 * minor_m12_m23 - m12 * minor_m11_m23 + m13 * minor_m11_m22
            }
            _ => self.lu().determinant(),
        }
    }
    /// The inverse, unless the matrix is singular: closed form up to 3 × 3, LU past that.
    pub fn try_inverse(&self) -> Option<Self> {
        let mut out = *self;
        out.try_inverse_mut().then_some(out)
    }
    /// Inverts in place; false (and the matrix unspecified) when singular.
    pub fn try_inverse_mut(&mut self) -> bool {
        match D {
            0 => true,
            1 => {
                let d = self[(0, 0)];
                if d == 0.0 {
                    false
                } else {
                    self[(0, 0)] = 1.0 / d;
                    true
                }
            }
            2 => {
                let (m11, m12, m21, m22) = (self[(0, 0)], self[(0, 1)], self[(1, 0)], self[(1, 1)]);
                let det = m11 * m22 - m21 * m12;
                if det == 0.0 {
                    false
                } else {
                    self[(0, 0)] = m22 / det;
                    self[(0, 1)] = -m12 / det;
                    self[(1, 0)] = -m21 / det;
                    self[(1, 1)] = m11 / det;
                    true
                }
            }
            3 => {
                let (m11, m12, m13) = (self[(0, 0)], self[(0, 1)], self[(0, 2)]);
                let (m21, m22, m23) = (self[(1, 0)], self[(1, 1)], self[(1, 2)]);
                let (m31, m32, m33) = (self[(2, 0)], self[(2, 1)], self[(2, 2)]);
                let minor_m12_m23 = m22 * m33 - m32 * m23;
                let minor_m11_m23 = m21 * m33 - m31 * m23;
                let minor_m11_m22 = m21 * m32 - m31 * m22;
                let det = m11 * minor_m12_m23 - m12 * minor_m11_m23 + m13 * minor_m11_m22;
                if det == 0.0 {
                    false
                } else {
                    self[(0, 0)] = minor_m12_m23 / det;
                    self[(0, 1)] = (m13 * m32 - m33 * m12) / det;
                    self[(0, 2)] = (m12 * m23 - m22 * m13) / det;
                    self[(1, 0)] = -minor_m11_m23 / det;
                    self[(1, 1)] = (m11 * m33 - m31 * m13) / det;
                    self[(1, 2)] = (m13 * m21 - m23 * m11) / det;
                    self[(2, 0)] = minor_m11_m22 / det;
                    self[(2, 1)] = (m12 * m31 - m32 * m11) / det;
                    self[(2, 2)] = (m11 * m22 - m21 * m12) / det;
                    true
                }
            }
            _ => match self.lu().try_inverse() {
                Some(inv) => {
                    *self = inv;
                    true
                }
                None => false,
            },
        }
    }
    /// Whether it is invertible.
    pub fn is_invertible(&self) -> bool {
        self.try_inverse().is_some()
    }
    /// The symmetric part `(A + Aᵀ)/2`... as nalgebra's `symmetric_part`.
    pub fn symmetric_part(&self) -> Self {
        Self::from_fn(|r, c| (self[(r, c)] + self[(c, r)]) * 0.5)
    }
}

// ---- vectors by size ----

impl<T: Scalar> Matrix<T, 1, 1> {
    pub fn new(x: T) -> Self {
        Self::from_fn(|_, _| x)
    }
}
impl<T: Scalar> Matrix<T, 2, 1> {
    pub fn new(x: T, y: T) -> Self {
        Self { data: XY { x, y } }
    }
}
impl<T: Scalar> Matrix<T, 3, 1> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { data: XYZ { x, y, z } }
    }
}
impl<T: Scalar> Matrix<T, 4, 1> {
    pub fn new(x: T, y: T, z: T, w: T) -> Self {
        Self { data: XYZW { x, y, z, w } }
    }
}
impl<T: Scalar> Matrix<T, 5, 1> {
    pub fn new(a: T, b: T, c: T, d: T, e: T) -> Self {
        Self::from([a, b, c, d, e])
    }
}
impl<T: Scalar> Matrix<T, 6, 1> {
    pub fn new(a: T, b: T, c: T, d: T, e: T, f: T) -> Self {
        Self::from([a, b, c, d, e, f])
    }
}
impl<T: Scalar> Matrix<T, 1, 2> {
    pub fn new(x: T, y: T) -> Self {
        Self::from_column_slice(&[x, y])
    }
}
impl<T: Scalar> Matrix<T, 1, 3> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self::from_column_slice(&[x, y, z])
    }
}
impl<T: Scalar> Matrix<T, 1, 4> {
    pub fn new(x: T, y: T, z: T, w: T) -> Self {
        Self::from_column_slice(&[x, y, z, w])
    }
}

/// Square-matrix `new` taking the elements row by row, as nalgebra's.
macro_rules! square_new {
    ($d:literal; $($a:ident),*) => {
        impl<T: Scalar> Matrix<T, $d, $d> {
            #[allow(clippy::too_many_arguments)]
            pub fn new($($a: T),*) -> Self {
                Self::from_row_slice(&[$($a),*])
            }
        }
    };
}
square_new!(2; m11, m12, m21, m22);
square_new!(3; m11, m12, m13, m21, m22, m23, m31, m32, m33);
square_new!(4; m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, m41, m42, m43, m44);

impl<T: Scalar> Matrix<T, 2, 3> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(m11: T, m12: T, m13: T, m21: T, m22: T, m23: T) -> Self {
        Self::from_row_slice(&[m11, m12, m13, m21, m22, m23])
    }
}
impl<T: Scalar> Matrix<T, 3, 2> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(m11: T, m12: T, m21: T, m22: T, m31: T, m32: T) -> Self {
        Self::from_row_slice(&[m11, m12, m21, m22, m31, m32])
    }
}

impl Matrix<f64, 2, 1> {
    /// (1, 0).
    pub fn x() -> Self {
        Self::new(1.0, 0.0)
    }
    /// (0, 1).
    pub fn y() -> Self {
        Self::new(0.0, 1.0)
    }
    /// (1, 0) as a unit vector.
    pub fn x_axis() -> Unit<Self> {
        Unit::new_unchecked(Self::x())
    }
    /// (0, 1) as a unit vector.
    pub fn y_axis() -> Unit<Self> {
        Unit::new_unchecked(Self::y())
    }
    /// `self.x b.y − self.y b.x` (nalgebra's 2-D cross product).
    pub fn perp(&self, b: &Self) -> f64 {
        self.x * b.y - self.y * b.x
    }
    /// (x, y, v).
    pub fn push(&self, v: f64) -> Matrix<f64, 3, 1> {
        Matrix::<f64, 3, 1>::new(self.x, self.y, v)
    }
    /// (x, y, 0).
    pub fn to_homogeneous(&self) -> Matrix<f64, 3, 1> {
        self.push(0.0)
    }
    /// The 2-D cross product as a 1 × 1 matrix (nalgebra's `cross` of 2-vectors is not this).
    pub fn yx(&self) -> Self {
        Self::new(self.y, self.x)
    }
}

impl Matrix<f64, 3, 1> {
    /// (1, 0, 0).
    pub fn x() -> Self {
        Self::new(1.0, 0.0, 0.0)
    }
    /// (0, 1, 0).
    pub fn y() -> Self {
        Self::new(0.0, 1.0, 0.0)
    }
    /// (0, 0, 1).
    pub fn z() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }
    pub fn x_axis() -> Unit<Self> {
        Unit::new_unchecked(Self::x())
    }
    pub fn y_axis() -> Unit<Self> {
        Unit::new_unchecked(Self::y())
    }
    pub fn z_axis() -> Unit<Self> {
        Unit::new_unchecked(Self::z())
    }
    /// The cross product, nalgebra's formula.
    pub fn cross(&self, b: &Self) -> Self {
        Self::new(
            self.y * b.z - self.z * b.y,
            self.z * b.x - self.x * b.z,
            self.x * b.y - self.y * b.x,
        )
    }
    /// The matrix `[v]×` with `[v]× w = v × w`.
    pub fn cross_matrix(&self) -> Matrix<f64, 3, 3> {
        Matrix::<f64, 3, 3>::new(0.0, -self.z, self.y, self.z, 0.0, -self.x, -self.y, self.x, 0.0)
    }
    /// (x, y).
    pub fn xy(&self) -> Matrix<f64, 2, 1> {
        Matrix::<f64, 2, 1>::new(self.x, self.y)
    }
    /// (x, z).
    pub fn xz(&self) -> Matrix<f64, 2, 1> {
        Matrix::<f64, 2, 1>::new(self.x, self.z)
    }
    /// (y, z).
    pub fn yz(&self) -> Matrix<f64, 2, 1> {
        Matrix::<f64, 2, 1>::new(self.y, self.z)
    }
    /// Itself.
    pub fn xyz(&self) -> Self {
        *self
    }
    /// (x, y, z, v).
    pub fn push(&self, v: f64) -> Matrix<f64, 4, 1> {
        Matrix::<f64, 4, 1>::new(self.x, self.y, self.z, v)
    }
    /// (x, y, z, 0).
    pub fn to_homogeneous(&self) -> Matrix<f64, 4, 1> {
        self.push(0.0)
    }
}

impl Matrix<f64, 4, 1> {
    pub fn x() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }
    pub fn y() -> Self {
        Self::new(0.0, 1.0, 0.0, 0.0)
    }
    pub fn z() -> Self {
        Self::new(0.0, 0.0, 1.0, 0.0)
    }
    pub fn w() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0)
    }
    /// (x, y, z).
    pub fn xyz(&self) -> Matrix<f64, 3, 1> {
        Matrix::<f64, 3, 1>::new(self.x, self.y, self.z)
    }
    /// (x, y).
    pub fn xy(&self) -> Matrix<f64, 2, 1> {
        Matrix::<f64, 2, 1>::new(self.x, self.y)
    }
}

impl Matrix<f64, 3, 3> {
    /// The 4 × 4 matrix with this as its top-left block and 1 at (3, 3).
    pub fn to_homogeneous(&self) -> Matrix<f64, 4, 4> {
        Matrix::from_fn(|r, c| match (r, c) {
            (3, 3) => 1.0,
            (3, _) | (_, 3) => 0.0,
            _ => self[(r, c)],
        })
    }
}

impl Matrix<f64, 4, 4> {
    /// The top-left 3 × 3 block.
    pub fn fixed_view_3x3(&self) -> Matrix<f64, 3, 3> {
        Matrix::from_fn(|r, c| self[(r, c)])
    }
}
