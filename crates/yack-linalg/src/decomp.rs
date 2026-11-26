// yack: LU, symmetric eigen and SVD decompositions in nalgebra's shape (f64).
//! LU is nalgebra's own algorithm step for step: partial pivoting on the first largest |value|,
//! the column scaled by the pivot's reciprocal, rank-1 updates as axpys, then forward and back
//! substitution (so solutions agree with nalgebra's to the last bit). The symmetric eigen
//! decomposition and the SVD are cyclic Jacobi methods: as accurate as nalgebra's QR-based ones,
//! but not bit-identical; eigenvalues come in no particular order (as nalgebra's), singular values
//! sorted from the largest down (as nalgebra's).

use core::marker::PhantomData;

use super::dmatrix::{ColKind, DMatrix, DVector, Dyn, Dynamic};
use super::matrix::Matrix;
use super::storage::{Scalar, Sh, Shape};

/// Element access shared by the fixed and dynamic matrices.
trait Grid {
    fn rows(&self) -> usize;
    fn cols(&self) -> usize;
    fn at(&self, r: usize, c: usize) -> f64;
    fn put(&mut self, r: usize, c: usize, v: f64);
    fn swap_rows_in(&mut self, a: usize, b: usize, cols: core::ops::Range<usize>) {
        for c in cols {
            let t = self.at(a, c);
            self.put(a, c, self.at(b, c));
            self.put(b, c, t);
        }
    }
}

impl<const R: usize, const C: usize> Grid for Matrix<f64, R, C>
where
    Sh: Shape<f64, R, C>,
{
    fn rows(&self) -> usize {
        R
    }
    fn cols(&self) -> usize {
        C
    }
    fn at(&self, r: usize, c: usize) -> f64 {
        self[(r, c)]
    }
    fn put(&mut self, r: usize, c: usize, v: f64) {
        self[(r, c)] = v;
    }
}

impl<K: ColKind> Grid for Dynamic<f64, K> {
    fn rows(&self) -> usize {
        self.nrows()
    }
    fn cols(&self) -> usize {
        self.ncols()
    }
    fn at(&self, r: usize, c: usize) -> f64 {
        self[(r, c)]
    }
    fn put(&mut self, r: usize, c: usize, v: f64) {
        self[(r, c)] = v;
    }
}

/// Factors `m` in place; `piv[i]` is the row swapped with row `i` at step `i` (or `i`).
// (`i` indexes the matrix as well as `piv`: a range is the clearer loop.)
#[allow(clippy::needless_range_loop)]
#[allow(clippy::indexing_slicing)] fn lu_factor<G: Grid>(m: &mut G, piv: &mut [usize]) {
    let (nr, nc) = (m.rows(), m.cols());
    for i in 0..nr.min(nc) {
        let mut p = i;
        let mut best = m.at(i, i).abs();
        for r in i + 1..nr {
            let v = m.at(r, i).abs();
            if v > best {
                best = v;
                p = r;
            }
        }
        let diag = m.at(p, i);
        if diag == 0.0 {
            continue;
        }
        let inv_diag = 1.0 / diag;
        if p != i {
            piv[i] = p;
            m.swap_rows_in(i, p, 0..i);
            // gauss_step_swap: the pivot column's two entries swapped, the rest scaled...
            let t = m.at(i, i);
            m.put(i, i, m.at(p, i));
            m.put(p, i, t);
            for r in i + 1..nr {
                m.put(r, i, m.at(r, i) * inv_diag);
            }
            // ...then column by column the pivot rows swapped and the column updated.
            for k in i + 1..nc {
                let t = m.at(i, k);
                m.put(i, k, m.at(p, k));
                m.put(p, k, t);
                let pk = m.at(i, k);
                for r in i + 1..nr {
                    m.put(r, k, -pk * m.at(r, i) + m.at(r, k));
                }
            }
        } else {
            for r in i + 1..nr {
                m.put(r, i, m.at(r, i) * inv_diag);
            }
            for k in i + 1..nc {
                let pk = m.at(i, k);
                for r in i + 1..nr {
                    m.put(r, k, -pk * m.at(r, i) + m.at(r, k));
                }
            }
        }
    }
}

/// Solves `lu x = b` in place (permuting `b` first); false when `U` has a zero on its diagonal.
fn lu_solve<G: Grid, B: Grid>(lu: &G, piv: &[usize], b: &mut B) -> bool {
    let n = lu.rows();
    let cols = b.cols();
    for (i, &p) in piv.iter().enumerate() {
        if p != i {
            b.swap_rows_in(i, p, 0..cols);
        }
    }
    for k in 0..cols {
        for i in 0..n.saturating_sub(1) {
            let coeff = b.at(i, k) / 1.0;
            for r in i + 1..n {
                b.put(r, k, -coeff * lu.at(r, i) + b.at(r, k));
            }
        }
    }
    for k in 0..cols {
        for i in (0..n).rev() {
            let diag = lu.at(i, i);
            if diag == 0.0 {
                return false;
            }
            let coeff = b.at(i, k) / diag;
            b.put(i, k, coeff);
            for r in 0..i {
                b.put(r, k, -coeff * lu.at(r, i) + b.at(r, k));
            }
        }
    }
    true
}

fn lu_determinant<G: Grid>(lu: &G, piv: &[usize]) -> f64 {
    let mut res = 1.0;
    for i in 0..lu.rows() {
        res *= lu.at(i, i);
    }
    let swaps = piv.iter().enumerate().filter(|&(i, &p)| p != i).count();
    if swaps % 2 == 0 { res } else { -res }
}

/// The LU decomposition of a fixed-size square matrix.
#[derive(Clone, Copy, Debug)]
pub struct LU<T: Scalar, const D: usize>
where
    Sh: Shape<T, D, D>,
{
    lu: Matrix<T, D, D>,
    piv: [usize; D],
}

impl<const D: usize> Matrix<f64, D, D>
where
    Sh: Shape<f64, D, D>,
{
    /// The LU decomposition with partial pivoting.
    pub fn lu(self) -> LU<f64, D> {
        let mut lu = self;
        let mut piv: [usize; D] = core::array::from_fn(|i| i);
        lu_factor(&mut lu, &mut piv);
        LU { lu, piv }
    }
    /// nalgebra's `full_piv_lu` stand-in (partial pivoting here).
    pub fn full_piv_lu(self) -> LU<f64, D> {
        self.lu()
    }
}

impl<const D: usize> LU<f64, D>
where
    Sh: Shape<f64, D, D>,
{
    /// `x` with `A x = b`, unless `A` is singular.
    pub fn solve<const C: usize>(&self, b: &Matrix<f64, D, C>) -> Option<Matrix<f64, D, C>>
    where
        Sh: Shape<f64, D, C>,
    {
        let mut x = *b;
        lu_solve(&self.lu, &self.piv, &mut x).then_some(x)
    }
    /// Solves in place; false when singular.
    pub fn solve_mut<const C: usize>(&self, b: &mut Matrix<f64, D, C>) -> bool
    where
        Sh: Shape<f64, D, C>,
    {
        lu_solve(&self.lu, &self.piv, b)
    }
    /// The inverse, unless singular.
    pub fn try_inverse(&self) -> Option<Matrix<f64, D, D>> {
        let mut x = Matrix::<f64, D, D>::identity();
        lu_solve(&self.lu, &self.piv, &mut x).then_some(x)
    }
    /// The determinant.
    pub fn determinant(&self) -> f64 {
        lu_determinant(&self.lu, &self.piv)
    }
    /// Whether the matrix is invertible.
    #[allow(clippy::indexing_slicing)] pub fn is_invertible(&self) -> bool {
        (0..D).all(|i| self.lu[(i, i)] != 0.0)
    }
}

/// The LU decomposition of a dynamic square matrix.
#[derive(Clone, Debug)]
pub struct DLU<T> {
    lu: DMatrix<T>,
    piv: Vec<usize>,
}

impl DMatrix<f64> {
    /// The LU decomposition with partial pivoting.
    pub fn lu(self) -> DLU<f64> {
        let mut lu = self;
        let mut piv: Vec<usize> = (0..lu.nrows().min(lu.ncols())).collect();
        lu_factor(&mut lu, &mut piv);
        DLU { lu, piv }
    }
    /// The inverse, unless singular (closed form up to 3 × 3, LU past that, as nalgebra).
    pub fn try_inverse(&self) -> Option<DMatrix<f64>> {
        if self.nrows() != self.ncols() {
            return None;
        }
        match self.nrows() {
            1 => Matrix::<f64, 1, 1>::from_column_slice(self.as_slice())
                .try_inverse()
                .map(|m| DMatrix::from_column_slice(1, 1, &m.to_vec())),
            2 => Matrix::<f64, 2, 2>::from_column_slice(self.as_slice())
                .try_inverse()
                .map(|m| DMatrix::from_column_slice(2, 2, &m.to_vec())),
            3 => Matrix::<f64, 3, 3>::from_column_slice(self.as_slice())
                .try_inverse()
                .map(|m| DMatrix::from_column_slice(3, 3, &m.to_vec())),
            _ => self.clone().lu().try_inverse(),
        }
    }
    /// The determinant.
    pub fn determinant(&self) -> f64 {
        assert_eq!(self.nrows(), self.ncols(), "determinant: not square");
        match self.nrows() {
            0 => 1.0,
            1 => self[(0, 0)],
            2 => Matrix::<f64, 2, 2>::from_column_slice(self.as_slice()).determinant(),
            3 => Matrix::<f64, 3, 3>::from_column_slice(self.as_slice()).determinant(),
            _ => self.clone().lu().determinant(),
        }
    }
}

impl DLU<f64> {
    /// `x` with `A x = b`, unless `A` is singular.
    pub fn solve<C: ColKind>(&self, b: &Dynamic<f64, C>) -> Option<Dynamic<f64, C>> {
        if self.lu.nrows() != b.nrows() || self.lu.nrows() != self.lu.ncols() {
            return None;
        }
        let mut x = b.clone();
        lu_solve(&self.lu, &self.piv, &mut x).then_some(x)
    }
    /// Solves in place; false when singular.
    pub fn solve_mut<C: ColKind>(&self, b: &mut Dynamic<f64, C>) -> bool {
        lu_solve(&self.lu, &self.piv, b)
    }
    /// The inverse, unless singular.
    pub fn try_inverse(&self) -> Option<DMatrix<f64>> {
        let n = self.lu.nrows();
        let mut x = DMatrix::identity(n, n);
        lu_solve(&self.lu, &self.piv, &mut x).then_some(x)
    }
    /// The determinant.
    pub fn determinant(&self) -> f64 {
        lu_determinant(&self.lu, &self.piv)
    }
}

// ---- symmetric eigen (cyclic Jacobi) ----

/// Diagonalises the symmetric `n × n` matrix `a` (column-major, only its lower half read):
/// returns the eigenvalues and the eigenvectors as columns (column-major).
#[allow(clippy::indexing_slicing)] fn jacobi_eigen(n: usize, a: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let at = |r: usize, c: usize| r + c * n;
    // Symmetrise from the lower triangle, as nalgebra reads it.
    let mut m = vec![0.0; n * n];
    for c in 0..n {
        for r in c..n {
            m[at(r, c)] = a[at(r, c)];
            m[at(c, r)] = a[at(r, c)];
        }
    }
    let mut v = vec![0.0; n * n];
    for i in 0..n {
        v[at(i, i)] = 1.0;
    }
    for _sweep in 0..100 {
        let mut off = 0.0;
        let mut total = 0.0;
        for c in 0..n {
            for r in 0..n {
                let x = m[at(r, c)] * m[at(r, c)];
                total += x;
                if r != c {
                    off += x;
                }
            }
        }
        if off <= f64::EPSILON * f64::EPSILON * total || off == 0.0 {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                let apq = m[at(p, q)];
                if apq == 0.0 {
                    continue;
                }
                let app = m[at(p, p)];
                let aqq = m[at(q, q)];
                let theta = (aqq - app) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let mkp = m[at(k, p)];
                    let mkq = m[at(k, q)];
                    m[at(k, p)] = c * mkp - s * mkq;
                    m[at(k, q)] = s * mkp + c * mkq;
                }
                for k in 0..n {
                    let mpk = m[at(p, k)];
                    let mqk = m[at(q, k)];
                    m[at(p, k)] = c * mpk - s * mqk;
                    m[at(q, k)] = s * mpk + c * mqk;
                }
                m[at(p, q)] = 0.0;
                m[at(q, p)] = 0.0;
                for k in 0..n {
                    let vkp = v[at(k, p)];
                    let vkq = v[at(k, q)];
                    v[at(k, p)] = c * vkp - s * vkq;
                    v[at(k, q)] = s * vkp + c * vkq;
                }
            }
        }
    }
    ((0..n).map(|i| m[at(i, i)]).collect(), v)
}

/// The eigen decomposition of a fixed-size symmetric matrix.
#[derive(Clone, Copy, Debug)]
pub struct SymmetricEigen<T: Scalar, const D: usize>
where
    Sh: Shape<T, D, D> + Shape<T, D, 1>,
{
    /// The eigenvectors, as columns, in the order of `eigenvalues`.
    pub eigenvectors: Matrix<T, D, D>,
    /// The eigenvalues, in no particular order.
    pub eigenvalues: Matrix<T, D, 1>,
}

impl<const D: usize> SymmetricEigen<f64, D>
where
    Sh: Shape<f64, D, D> + Shape<f64, D, 1>,
{
    /// The decomposition of `m` (its lower triangle is read).
    pub fn new(m: Matrix<f64, D, D>) -> Self {
        let (vals, vecs) = jacobi_eigen(D, &m.to_vec());
        Self {
            eigenvectors: Matrix::from_column_slice(&vecs),
            eigenvalues: Matrix::from_column_slice(&vals),
        }
    }
    /// `V Λ Vᵀ`.
    #[allow(clippy::indexing_slicing)] pub fn recompose(&self) -> Matrix<f64, D, D> {
        Matrix::from_fn(|r, c| {
            let mut s = 0.0;
            for k in 0..D {
                s += self.eigenvectors[(r, k)] * self.eigenvalues[k] * self.eigenvectors[(c, k)];
            }
            s
        })
    }
}

impl<const D: usize> Matrix<f64, D, D>
where
    Sh: Shape<f64, D, D> + Shape<f64, D, 1>,
{
    /// The eigen decomposition, the matrix taken to be symmetric.
    pub fn symmetric_eigen(self) -> SymmetricEigen<f64, D> {
        SymmetricEigen::new(self)
    }
    /// The eigenvalues, the matrix taken to be symmetric.
    pub fn symmetric_eigenvalues(&self) -> Matrix<f64, D, 1> {
        SymmetricEigen::new(*self).eigenvalues
    }
}

/// The eigen decomposition of a dynamic symmetric matrix.
#[derive(Clone, Debug)]
pub struct DSymmetricEigen<T> {
    /// The eigenvectors, as columns.
    pub eigenvectors: DMatrix<T>,
    /// The eigenvalues, in no particular order.
    pub eigenvalues: DVector<T>,
}

impl DMatrix<f64> {
    /// The eigen decomposition, the matrix taken to be symmetric.
    pub fn symmetric_eigen(self) -> DSymmetricEigen<f64> {
        assert_eq!(self.nrows(), self.ncols(), "symmetric_eigen: not square");
        let n = self.nrows();
        let (vals, vecs) = jacobi_eigen(n, self.as_slice());
        DSymmetricEigen {
            eigenvectors: DMatrix::from_vec(n, n, vecs),
            eigenvalues: DVector::from_vec(vals),
        }
    }
    /// The eigenvalues, the matrix taken to be symmetric.
    pub fn symmetric_eigenvalues(&self) -> DVector<f64> {
        self.clone().symmetric_eigen().eigenvalues
    }
}

// ---- SVD (one-sided Jacobi) ----

/// The thin singular value decomposition `A = U Σ Vᵀ` of a dynamic matrix.
#[derive(Clone, Debug)]
pub struct SVD<T, R, C> {
    /// `U` (`m × min(m, n)`), if asked for.
    pub u: Option<DMatrix<T>>,
    /// `Vᵀ` (`min(m, n) × n`), if asked for.
    pub v_t: Option<DMatrix<T>>,
    /// The singular values, largest first.
    pub singular_values: DVector<T>,
    _dims: PhantomData<(R, C)>,
}

/// One-sided Jacobi on the columns of `a` (`m × n`, `m >= n`): returns `U` (`m × n`), `σ` and
/// `V` (`n × n`), sorted by `σ` from the largest down.
#[allow(clippy::indexing_slicing)] fn jacobi_svd(m: usize, n: usize, a: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut u = a.to_vec();
    let mut v = vec![0.0; n * n];
    for i in 0..n {
        v[i + i * n] = 1.0;
    }
    for _sweep in 0..100 {
        let mut rotated = false;
        for p in 0..n {
            for q in p + 1..n {
                let (mut alpha, mut beta, mut gamma) = (0.0, 0.0, 0.0);
                for k in 0..m {
                    let x = u[k + p * m];
                    let y = u[k + q * m];
                    alpha += x * x;
                    beta += y * y;
                    gamma += x * y;
                }
                if gamma == 0.0 || gamma.abs() <= f64::EPSILON * (alpha * beta).sqrt() {
                    continue;
                }
                rotated = true;
                let zeta = (beta - alpha) / (2.0 * gamma);
                let t = zeta.signum() / (zeta.abs() + (1.0 + zeta * zeta).sqrt());
                let t = if zeta == 0.0 { 1.0 } else { t };
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = c * t;
                for k in 0..m {
                    let x = u[k + p * m];
                    let y = u[k + q * m];
                    u[k + p * m] = c * x - s * y;
                    u[k + q * m] = s * x + c * y;
                }
                for k in 0..n {
                    let x = v[k + p * n];
                    let y = v[k + q * n];
                    v[k + p * n] = c * x - s * y;
                    v[k + q * n] = s * x + c * y;
                }
            }
        }
        if !rotated {
            break;
        }
    }
    let mut sigma: Vec<f64> = (0..n)
        .map(|j| (0..m).map(|k| u[k + j * m] * u[k + j * m]).sum::<f64>().sqrt())
        .collect();
    for (j, &s) in sigma.iter().enumerate() {
        for k in 0..m {
            u[k + j * m] = if s > 0.0 { u[k + j * m] / s } else { 0.0 };
        }
    }
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&x, &y| sigma[y].total_cmp(&sigma[x]));
    let us: Vec<f64> = order.iter().flat_map(|&j| u[j * m..(j + 1) * m].to_vec()).collect();
    let vs: Vec<f64> = order.iter().flat_map(|&j| v[j * n..(j + 1) * n].to_vec()).collect();
    sigma = order.iter().map(|&j| sigma[j]).collect();
    (us, sigma, vs)
}

impl DMatrix<f64> {
    /// The thin SVD; `U` and `Vᵀ` only when asked for.
    pub fn svd(self, compute_u: bool, compute_v: bool) -> SVD<f64, Dyn, Dyn> {
        let (m, n) = self.shape();
        let (u, sigma, v_t) = if m >= n {
            let (u, s, v) = jacobi_svd(m, n, self.as_slice());
            let u = DMatrix::from_vec(m, n, u);
            let v_t = DMatrix::from_vec(n, n, v).transpose();
            (u, s, v_t)
        } else {
            // Aᵀ = U' Σ V'ᵀ, so A = V' Σ U'ᵀ.
            let at = self.transpose();
            let (u2, s, v2) = jacobi_svd(n, m, at.as_slice());
            let u = DMatrix::from_vec(m, m, v2);
            let v_t = DMatrix::from_vec(n, m, u2).transpose();
            (u, s, v_t)
        };
        SVD {
            u: compute_u.then_some(u),
            v_t: compute_v.then_some(v_t),
            singular_values: DVector::from_vec(sigma),
            _dims: PhantomData,
        }
    }
    /// The singular values, largest first.
    pub fn singular_values(&self) -> DVector<f64> {
        self.clone().svd(false, false).singular_values
    }
    /// The pseudo-inverse, singular values `<= eps` taken as zero.
    pub fn pseudo_inverse(self, eps: f64) -> Result<DMatrix<f64>, &'static str> {
        self.svd(true, true).pseudo_inverse(eps)
    }
}

impl SVD<f64, Dyn, Dyn> {
    /// The least-squares `x` of `A x = b` (singular values `<= eps` taken as zero), as nalgebra.
    #[allow(clippy::indexing_slicing)] pub fn solve<C: ColKind>(&self, b: &Dynamic<f64, C>, eps: f64) -> Result<Dynamic<f64, C>, &'static str> {
        if eps < 0.0 {
            return Err("SVD solve: the epsilon must be non-negative.");
        }
        let (Some(u), Some(v_t)) = (&self.u, &self.v_t) else {
            return Err("SVD solve: U and V^t have not been computed.");
        };
        let mut ut_b = u.tr_mul(b);
        let k = self.singular_values.len();
        for j in 0..ut_b.ncols() {
            for i in 0..k {
                let val = self.singular_values[i];
                let x = ut_b[(i, j)];
                ut_b[(i, j)] = if val > eps { x / val } else { 0.0 };
            }
        }
        Ok(v_t.tr_mul(&ut_b))
    }
    /// The pseudo-inverse.
    pub fn pseudo_inverse(&self, eps: f64) -> Result<DMatrix<f64>, &'static str> {
        let (Some(u), Some(_)) = (&self.u, &self.v_t) else {
            return Err("SVD pseudo inverse: the epsilon must be non-negative.");
        };
        let ident = DMatrix::identity(u.nrows(), u.nrows());
        self.solve(&ident, eps)
    }
    /// The number of singular values above `eps`.
    pub fn rank(&self, eps: f64) -> usize {
        self.singular_values.iter().filter(|&&s| s > eps).count()
    }
    /// `U Σ Vᵀ`.
    #[allow(clippy::indexing_slicing)] pub fn recompose(self) -> Result<DMatrix<f64>, &'static str> {
        let (Some(u), Some(v_t)) = (self.u, self.v_t) else {
            return Err("SVD recomposition: U and V^t have not been computed.");
        };
        let k = self.singular_values.len();
        let us = DMatrix::from_fn(u.nrows(), k, |i, j| u[(i, j)] * self.singular_values[j]);
        Ok(us * v_t)
    }
}


/// The SVD `A = U Σ Vᵀ` of a fixed-size square matrix (what nalgebra's `svd` gives for one).
#[derive(Clone, Copy, Debug)]
pub struct SquareSVD<T: Scalar, const D: usize>
where
    Sh: Shape<T, D, D> + Shape<T, D, 1>,
{
    /// `U`, if asked for.
    pub u: Option<Matrix<T, D, D>>,
    /// `Vᵀ`, if asked for.
    pub v_t: Option<Matrix<T, D, D>>,
    /// The singular values, largest first.
    pub singular_values: Matrix<T, D, 1>,
}

impl<const D: usize> Matrix<f64, D, D>
where
    Sh: Shape<f64, D, D> + Shape<f64, D, 1>,
{
    /// The SVD; `U` and `Vᵀ` only when asked for.
    pub fn svd(self, compute_u: bool, compute_v: bool) -> SquareSVD<f64, D> {
        let (u, s, v) = jacobi_svd(D, D, &self.to_vec());
        SquareSVD {
            u: compute_u.then(|| Matrix::from_column_slice(&u)),
            v_t: compute_v.then(|| Matrix::<f64, D, D>::from_column_slice(&v).transpose()),
            singular_values: Matrix::from_column_slice(&s),
        }
    }
    /// The singular values, largest first.
    pub fn singular_values(&self) -> Matrix<f64, D, 1> {
        self.svd(false, false).singular_values
    }
}
