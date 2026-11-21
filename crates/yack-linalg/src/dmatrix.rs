// yack: nalgebra-shaped dynamically sized matrices and vectors (f64, column-major).
//! `DMatrix<T>` is `Dynamic<T, Dyn>` and `DVector<T>` is `Dynamic<T, U1>`: one storage, with the
//! column count's kind in the type so that each has nalgebra's own constructors.

use core::fmt;
use core::marker::PhantomData;
use core::ops::{Add, AddAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign};

/// A dimension known only at run time (nalgebra's `Dyn`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Dyn(pub usize);

/// The dimension 1, known at compile time (nalgebra's `U1`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct U1;

/// The column-count kinds of [`Dynamic`]: [`Dyn`] (a matrix) or [`U1`] (a vector).
pub trait ColKind: Copy + PartialEq + fmt::Debug + 'static {}
impl ColKind for Dyn {}
impl ColKind for U1 {}

/// A matrix with sizes known at run time, stored column by column.
#[derive(Clone, PartialEq)]
pub struct Dynamic<T, C: ColKind> {
    nrows: usize,
    ncols: usize,
    data: Vec<T>,
    _c: PhantomData<C>,
}

/// nalgebra's `DMatrix`.
pub type DMatrix<T> = Dynamic<T, Dyn>;
/// nalgebra's `DVector`.
pub type DVector<T> = Dynamic<T, U1>;
/// nalgebra's `OMatrix`/`Matrix` with dynamic sizes, as cadrs names it.
pub type OMatrixDyn<T> = Dynamic<T, Dyn>;

impl<T: fmt::Debug, C: ColKind> fmt::Debug for Dynamic<T, C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DMatrix")
            .field("nrows", &self.nrows)
            .field("ncols", &self.ncols)
            .field("data", &self.data)
            .finish()
    }
}

impl<T: fmt::Display, C: ColKind> fmt::Display for Dynamic<T, C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for r in 0..self.nrows {
            write!(f, "[")?;
            for c in 0..self.ncols {
                if c > 0 {
                    write!(f, ", ")?;
                }
                fmt::Display::fmt(&self.data[r + c * self.nrows], f)?;
            }
            writeln!(f, "]")?;
        }
        Ok(())
    }
}

impl<T: Copy, C: ColKind> Dynamic<T, C> {
    pub(crate) fn raw(nrows: usize, ncols: usize, data: Vec<T>) -> Self {
        assert_eq!(data.len(), nrows * ncols, "dynamic matrix: wrong element count");
        Self { nrows, ncols, data, _c: PhantomData }
    }
    /// The number of rows.
    pub fn nrows(&self) -> usize {
        self.nrows
    }
    /// The number of columns.
    pub fn ncols(&self) -> usize {
        self.ncols
    }
    /// `(rows, columns)`.
    pub fn shape(&self) -> (usize, usize) {
        (self.nrows, self.ncols)
    }
    /// The number of elements.
    pub fn len(&self) -> usize {
        self.data.len()
    }
    /// Whether there are no elements.
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
    /// The elements, column by column.
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }
    /// The elements, column by column, mutably.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.data
    }
    /// The elements, column by column.
    pub fn iter(&self) -> core::slice::Iter<'_, T> {
        self.data.iter()
    }
    /// The elements, column by column, mutably.
    pub fn iter_mut(&mut self) -> core::slice::IterMut<'_, T> {
        self.data.iter_mut()
    }
    /// The element at linear index `i`, if any.
    pub fn get(&self, i: usize) -> Option<&T> {
        self.data.get(i)
    }
    /// `f` of each element.
    pub fn map<U: Copy>(&self, f: impl FnMut(T) -> U) -> Dynamic<U, C> {
        Dynamic {
            nrows: self.nrows,
            ncols: self.ncols,
            data: self.data.iter().copied().map(f).collect(),
            _c: PhantomData,
        }
    }
    /// Each element replaced by `f` of it.
    pub fn apply(&mut self, mut f: impl FnMut(&mut T)) {
        for v in &mut self.data {
            f(v);
        }
    }
    /// The transpose.
    pub fn transpose(&self) -> Dynamic<T, Dyn> {
        let (r, c) = (self.nrows, self.ncols);
        let mut data = Vec::with_capacity(r * c);
        for i in 0..r {
            for j in 0..c {
                data.push(self.data[i + j * r]);
            }
        }
        Dynamic::raw(c, r, data)
    }
    /// Column `j`, as a vector of its own.
    pub fn column(&self, j: usize) -> Dynamic<T, U1> {
        assert!(j < self.ncols, "column {j} out of bounds");
        let r = self.nrows;
        Dynamic::raw(r, 1, self.data[j * r..(j + 1) * r].to_vec())
    }
    /// Row `i`, as a 1-row matrix.
    pub fn row(&self, i: usize) -> Dynamic<T, Dyn> {
        assert!(i < self.nrows, "row {i} out of bounds");
        Dynamic::raw(1, self.ncols, (0..self.ncols).map(|j| self.data[i + j * self.nrows]).collect())
    }
    /// Sets column `j`.
    pub fn set_column<C2: ColKind>(&mut self, j: usize, v: &Dynamic<T, C2>) {
        assert!(j < self.ncols && v.len() == self.nrows, "set_column: shape mismatch");
        let r = self.nrows;
        self.data[j * r..(j + 1) * r].copy_from_slice(&v.data);
    }
    /// Sets row `i`.
    pub fn set_row<C2: ColKind>(&mut self, i: usize, v: &Dynamic<T, C2>) {
        assert!(i < self.nrows && v.len() == self.ncols, "set_row: shape mismatch");
        for j in 0..self.ncols {
            self.data[i + j * self.nrows] = v.data[j];
        }
    }
    /// Every element `v`.
    pub fn fill(&mut self, v: T) {
        self.data.fill(v);
    }
    /// Rows `a` and `b` swapped.
    pub fn swap_rows(&mut self, a: usize, b: usize) {
        if a != b {
            let r = self.nrows;
            for j in 0..self.ncols {
                self.data.swap(a + j * r, b + j * r);
            }
        }
    }
    /// Columns `a` and `b` swapped.
    pub fn swap_columns(&mut self, a: usize, b: usize) {
        if a != b {
            let r = self.nrows;
            for i in 0..r {
                self.data.swap(i + a * r, i + b * r);
            }
        }
    }
    /// Itself.
    pub fn into_owned(self) -> Self {
        self
    }
    /// A copy.
    pub fn clone_owned(&self) -> Self {
        self.clone()
    }
    /// The elements, column by column.
    pub fn data_vec(self) -> Vec<T> {
        self.data
    }
}

impl<T: Copy> Dynamic<T, Dyn> {
    /// The `r × c` matrix with `f(row, column)` at each place (columns filled in turn).
    pub fn from_fn(r: usize, c: usize, mut f: impl FnMut(usize, usize) -> T) -> Self {
        let mut data = Vec::with_capacity(r * c);
        for j in 0..c {
            for i in 0..r {
                data.push(f(i, j));
            }
        }
        Self::raw(r, c, data)
    }
    /// Every element `v`.
    pub fn from_element(r: usize, c: usize, v: T) -> Self {
        Self::raw(r, c, vec![v; r * c])
    }
    /// The first `r c` elements of `s`, column by column.
    pub fn from_column_slice(r: usize, c: usize, s: &[T]) -> Self {
        Self::raw(r, c, s[..r * c].to_vec())
    }
    /// The first `r c` elements of `s`, row by row.
    pub fn from_row_slice(r: usize, c: usize, s: &[T]) -> Self {
        Self::from_fn(r, c, |i, j| s[i * c + j])
    }
    /// `v`, column by column.
    pub fn from_vec(r: usize, c: usize, v: Vec<T>) -> Self {
        Self::raw(r, c, v)
    }
    /// The first `r c` items of `it`, column by column.
    pub fn from_iterator(r: usize, c: usize, it: impl IntoIterator<Item = T>) -> Self {
        let data: Vec<T> = it.into_iter().take(r * c).collect();
        Self::raw(r, c, data)
    }
    /// The columns side by side.
    pub fn from_columns(cols: &[Dynamic<T, U1>]) -> Self {
        let r = cols.first().map_or(0, |c| c.len());
        let mut data = Vec::with_capacity(r * cols.len());
        for c in cols {
            assert_eq!(c.len(), r, "from_columns: columns of different lengths");
            data.extend_from_slice(&c.data);
        }
        Self::raw(r, cols.len(), data)
    }
}

impl<T: Copy> Dynamic<T, U1> {
    /// The vector with `f(i, 0)` at each place.
    pub fn from_fn(n: usize, mut f: impl FnMut(usize, usize) -> T) -> Self {
        Self::raw(n, 1, (0..n).map(|i| f(i, 0)).collect())
    }
    /// Every element `v`.
    pub fn from_element(n: usize, v: T) -> Self {
        Self::raw(n, 1, vec![v; n])
    }
    /// The elements of `s`.
    pub fn from_column_slice(s: &[T]) -> Self {
        Self::raw(s.len(), 1, s.to_vec())
    }
    /// The elements of `s`.
    pub fn from_row_slice(s: &[T]) -> Self {
        Self::from_column_slice(s)
    }
    /// `v`.
    pub fn from_vec(v: Vec<T>) -> Self {
        Self::raw(v.len(), 1, v)
    }
    /// The first `n` items of `it`.
    pub fn from_iterator(n: usize, it: impl IntoIterator<Item = T>) -> Self {
        let data: Vec<T> = it.into_iter().take(n).collect();
        Self::raw(n, 1, data)
    }
}

impl Dynamic<f64, Dyn> {
    /// All zeros.
    pub fn zeros(r: usize, c: usize) -> Self {
        Self::from_element(r, c, 0.0)
    }
    /// Ones on the diagonal.
    pub fn identity(r: usize, c: usize) -> Self {
        Self::from_fn(r, c, |i, j| if i == j { 1.0 } else { 0.0 })
    }
    /// The square diagonal matrix of `d`.
    pub fn from_diagonal(d: &Dynamic<f64, U1>) -> Self {
        let n = d.len();
        Self::from_fn(n, n, |i, j| if i == j { d.data[i] } else { 0.0 })
    }
}

impl Dynamic<f64, U1> {
    /// All zeros.
    pub fn zeros(n: usize) -> Self {
        Self::from_element(n, 0.0)
    }
}

impl<C: ColKind> Dynamic<f64, C> {
    /// Σ aᵢ bᵢ in index order.
    pub fn dot<C2: ColKind>(&self, b: &Dynamic<f64, C2>) -> f64 {
        assert_eq!(self.len(), b.len(), "dot: length mismatch");
        let mut s = 0.0;
        for (x, y) in self.data.iter().zip(&b.data) {
            s += x * y;
        }
        s
    }
    /// Σ aᵢ².
    pub fn norm_squared(&self) -> f64 {
        self.dot(self)
    }
    /// The Euclidean (Frobenius) norm.
    pub fn norm(&self) -> f64 {
        self.norm_squared().sqrt()
    }
    /// Divided by its norm.
    pub fn normalize(&self) -> Self {
        let n = self.norm();
        self.map(|v| v / n)
    }
    /// Each element times `k`.
    pub fn scale(&self, k: f64) -> Self {
        self.map(|v| v * k)
    }
    /// Each element's absolute value.
    pub fn abs(&self) -> Self {
        self.map(f64::abs)
    }
    /// The largest absolute value.
    pub fn amax(&self) -> f64 {
        self.data.iter().fold(0.0, |m, v| if v.abs() > m { v.abs() } else { m })
    }
    /// The largest element.
    pub fn max(&self) -> f64 {
        let first = self.data.first().copied().unwrap_or(0.0);
        self.data.iter().fold(first, |m, &v| if v > m { v } else { m })
    }
    /// The smallest element.
    pub fn min(&self) -> f64 {
        let first = self.data.first().copied().unwrap_or(0.0);
        self.data.iter().fold(first, |m, &v| if v < m { v } else { m })
    }
    /// The index of the first largest element.
    pub fn imax(&self) -> usize {
        let mut best = 0;
        for i in 1..self.data.len() {
            if self.data[i] > self.data[best] {
                best = i;
            }
        }
        best
    }
    /// The index of the first smallest element.
    pub fn imin(&self) -> usize {
        let mut best = 0;
        for i in 1..self.data.len() {
            if self.data[i] < self.data[best] {
                best = i;
            }
        }
        best
    }
    /// The index of the first largest absolute value.
    pub fn iamax(&self) -> usize {
        self.abs().imax()
    }
    /// The sum of the elements.
    pub fn sum(&self) -> f64 {
        self.data.iter().fold(0.0, |s, v| s + v)
    }
    /// Whether every element is finite.
    pub fn is_finite(&self) -> bool {
        self.data.iter().all(|v| v.is_finite())
    }
    /// `selfᵀ b`.
    pub fn tr_mul<C2: ColKind>(&self, b: &Dynamic<f64, C2>) -> Dynamic<f64, C2> {
        assert_eq!(self.nrows, b.nrows, "tr_mul: shape mismatch");
        let (n, m, p) = (self.nrows, self.ncols, b.ncols);
        let mut data = Vec::with_capacity(m * p);
        for j in 0..p {
            for i in 0..m {
                let mut s = 0.0;
                for k in 0..n {
                    s += self.data[k + i * n] * b.data[k + j * n];
                }
                data.push(s);
            }
        }
        Dynamic::raw(m, p, data)
    }
}

impl<T: Copy, C: ColKind> Index<usize> for Dynamic<T, C> {
    type Output = T;
    fn index(&self, i: usize) -> &T {
        &self.data[i]
    }
}
impl<T: Copy, C: ColKind> IndexMut<usize> for Dynamic<T, C> {
    fn index_mut(&mut self, i: usize) -> &mut T {
        &mut self.data[i]
    }
}
impl<T: Copy, C: ColKind> Index<(usize, usize)> for Dynamic<T, C> {
    type Output = T;
    fn index(&self, (r, c): (usize, usize)) -> &T {
        assert!(r < self.nrows && c < self.ncols, "index ({r}, {c}) out of bounds");
        &self.data[r + c * self.nrows]
    }
}
impl<T: Copy, C: ColKind> IndexMut<(usize, usize)> for Dynamic<T, C> {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut T {
        assert!(r < self.nrows && c < self.ncols, "index ({r}, {c}) out of bounds");
        &mut self.data[r + c * self.nrows]
    }
}

// ---- operators ----

fn zip_with<C: ColKind>(a: &Dynamic<f64, C>, b: &Dynamic<f64, C>, f: impl Fn(f64, f64) -> f64) -> Dynamic<f64, C> {
    assert_eq!(a.shape(), b.shape(), "matrix shapes differ");
    Dynamic::raw(a.nrows, a.ncols, a.data.iter().zip(&b.data).map(|(x, y)| f(*x, *y)).collect())
}

macro_rules! dyn_elementwise {
    ($Tr:ident, $f:ident, $TrA:ident, $fa:ident, $op:tt) => {
        impl<C: ColKind> $Tr<Dynamic<f64, C>> for Dynamic<f64, C> {
            type Output = Dynamic<f64, C>;
            fn $f(self, b: Dynamic<f64, C>) -> Dynamic<f64, C> {
                zip_with(&self, &b, |x, y| x $op y)
            }
        }
        impl<'b, C: ColKind> $Tr<&'b Dynamic<f64, C>> for Dynamic<f64, C> {
            type Output = Dynamic<f64, C>;
            fn $f(self, b: &'b Dynamic<f64, C>) -> Dynamic<f64, C> {
                zip_with(&self, b, |x, y| x $op y)
            }
        }
        impl<'a, C: ColKind> $Tr<Dynamic<f64, C>> for &'a Dynamic<f64, C> {
            type Output = Dynamic<f64, C>;
            fn $f(self, b: Dynamic<f64, C>) -> Dynamic<f64, C> {
                zip_with(self, &b, |x, y| x $op y)
            }
        }
        impl<'a, 'b, C: ColKind> $Tr<&'b Dynamic<f64, C>> for &'a Dynamic<f64, C> {
            type Output = Dynamic<f64, C>;
            fn $f(self, b: &'b Dynamic<f64, C>) -> Dynamic<f64, C> {
                zip_with(self, b, |x, y| x $op y)
            }
        }
        impl<C: ColKind> $TrA<Dynamic<f64, C>> for Dynamic<f64, C> {
            fn $fa(&mut self, b: Dynamic<f64, C>) {
                *self = zip_with(self, &b, |x, y| x $op y);
            }
        }
        impl<'b, C: ColKind> $TrA<&'b Dynamic<f64, C>> for Dynamic<f64, C> {
            fn $fa(&mut self, b: &'b Dynamic<f64, C>) {
                *self = zip_with(self, b, |x, y| x $op y);
            }
        }
    };
}
dyn_elementwise!(Add, add, AddAssign, add_assign, +);
dyn_elementwise!(Sub, sub, SubAssign, sub_assign, -);

impl<C: ColKind> Neg for Dynamic<f64, C> {
    type Output = Dynamic<f64, C>;
    fn neg(self) -> Dynamic<f64, C> {
        self.map(|v| -v)
    }
}
impl<C: ColKind> Neg for &Dynamic<f64, C> {
    type Output = Dynamic<f64, C>;
    fn neg(self) -> Dynamic<f64, C> {
        self.map(|v| -v)
    }
}
impl<C: ColKind> Mul<f64> for Dynamic<f64, C> {
    type Output = Dynamic<f64, C>;
    fn mul(self, k: f64) -> Dynamic<f64, C> {
        self.map(|v| v * k)
    }
}
impl<C: ColKind> Mul<f64> for &Dynamic<f64, C> {
    type Output = Dynamic<f64, C>;
    fn mul(self, k: f64) -> Dynamic<f64, C> {
        self.map(|v| v * k)
    }
}
impl<C: ColKind> Mul<Dynamic<f64, C>> for f64 {
    type Output = Dynamic<f64, C>;
    fn mul(self, m: Dynamic<f64, C>) -> Dynamic<f64, C> {
        m.map(|v| self * v)
    }
}
impl<'a, C: ColKind> Mul<&'a Dynamic<f64, C>> for f64 {
    type Output = Dynamic<f64, C>;
    fn mul(self, m: &'a Dynamic<f64, C>) -> Dynamic<f64, C> {
        m.map(|v| self * v)
    }
}
impl<C: ColKind> core::ops::Div<f64> for Dynamic<f64, C> {
    type Output = Dynamic<f64, C>;
    fn div(self, k: f64) -> Dynamic<f64, C> {
        self.map(|v| v / k)
    }
}
impl<C: ColKind> core::ops::Div<f64> for &Dynamic<f64, C> {
    type Output = Dynamic<f64, C>;
    fn div(self, k: f64) -> Dynamic<f64, C> {
        self.map(|v| v / k)
    }
}
impl<C: ColKind> MulAssign<f64> for Dynamic<f64, C> {
    fn mul_assign(&mut self, k: f64) {
        for v in &mut self.data {
            *v *= k;
        }
    }
}
impl<C: ColKind> core::ops::DivAssign<f64> for Dynamic<f64, C> {
    fn div_assign(&mut self, k: f64) {
        for v in &mut self.data {
            *v /= k;
        }
    }
}

/// `a b`, each element summed over `k` in order (nalgebra's gemm order).
pub(crate) fn dmatmul<C1: ColKind, C2: ColKind>(a: &Dynamic<f64, C1>, b: &Dynamic<f64, C2>) -> Dynamic<f64, C2> {
    assert_eq!(a.ncols, b.nrows, "matrix product: shape mismatch");
    let (n, k, p) = (a.nrows, a.ncols, b.ncols);
    let mut data = vec![0.0; n * p];
    for j in 0..p {
        for kk in 0..k {
            let bkj = b.data[kk + j * k];
            for i in 0..n {
                let t = a.data[i + kk * n] * bkj;
                if kk == 0 {
                    data[i + j * n] = t;
                } else {
                    data[i + j * n] += t;
                }
            }
        }
    }
    Dynamic::raw(n, p, data)
}

macro_rules! dyn_matmul {
    ($($lt:lifetime)*; $A:ty, $B:ty) => {
        impl<$($lt,)* C2: ColKind> Mul<$B> for $A {
            type Output = Dynamic<f64, C2>;
            #[allow(clippy::needless_borrow)]
            fn mul(self, b: $B) -> Dynamic<f64, C2> {
                dmatmul(&self, &b)
            }
        }
    };
}
dyn_matmul!(; Dynamic<f64, Dyn>, Dynamic<f64, C2>);
dyn_matmul!('b; Dynamic<f64, Dyn>, &'b Dynamic<f64, C2>);
dyn_matmul!('a; &'a Dynamic<f64, Dyn>, Dynamic<f64, C2>);
dyn_matmul!('a 'b; &'a Dynamic<f64, Dyn>, &'b Dynamic<f64, C2>);
