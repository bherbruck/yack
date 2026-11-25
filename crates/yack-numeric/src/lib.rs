// yack: Linear solves, root finding and Kronrod rules shared by the geometry and measurement code.
// Adapted from springrep@a03e8a1 src/numeric.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's numeric.rs (M3 W1', D26); the Gauss-Legendre rules and their
// quadrature stay in springrep until hardening; paths and visibility only.; the Gauss rules joined in M3 W5;
// hardening (w-h1): the Gauss rules are const tables of the same bits, not a OnceLock cache; the
// solvers work on row slices with checked access and return errors for short inputs.
//! Linear solves, root finding and Kronrod rules shared by the geometry and measurement code.

use yack_base::kernel::{Error, Result};

/// The 15-point Kronrod rule on [−1, 1] as `(x, weight, 7-point Gauss weight)` for `x ≥ 0`, each
/// node but the centre standing for `±x`: the Gauss rule is the nodes with a weight of their own
/// (Kronrod, Nodes and Weights of Quadrature Formulas, 1965).
pub const KRONROD_15: [(f64, f64, f64); 8] = [
    (0.991_455_371_120_812_6, 0.022_935_322_010_529_225, 0.0),
    (
        0.949_107_912_342_758_5,
        0.063_092_092_629_978_55,
        0.129_484_966_168_869_7,
    ),
    (0.864_864_423_359_769_1, 0.104_790_010_322_250_18, 0.0),
    (
        0.741_531_185_599_394_4,
        0.140_653_259_715_525_92,
        0.279_705_391_489_276_7,
    ),
    (0.586_087_235_467_691_1, 0.169_004_726_639_267_9, 0.0),
    (
        0.405_845_151_377_397_2,
        0.190_350_578_064_785_4,
        0.381_830_050_505_118_9,
    ),
    (0.207_784_955_007_898_47, 0.204_432_940_075_298_9, 0.0),
    (0.0, 0.209_482_141_084_727_83, 0.417_959_183_673_469_4),
];

/// The 7-point Kronrod rule on [−1, 1] with the 3-point Gauss rule inside it, as [`KRONROD_15`]
/// (Kronrod, Nodes and Weights of Quadrature Formulas, 1965).
pub const KRONROD_7: [(f64, f64, f64); 4] = [
    (0.960_491_268_708_020_3, 0.104_656_226_026_467_27, 0.0),
    (
        0.774_596_669_241_483_4,
        0.268_488_089_868_333_44,
        0.555_555_555_555_555_6,
    ),
    (0.434_243_749_346_802_56, 0.401_397_414_775_962_2, 0.0),
    (0.0, 0.450_916_538_658_474_14, 0.888_888_888_888_889),
];

/// The `i`th row of a row-major matrix `width` values wide, or `None` past its end. A zero width
/// gives every row empty.
fn row(m: &[f64], width: usize, i: usize) -> Option<&[f64]> {
    m.get(i * width..(i + 1) * width)
}

/// [`row`], mutably.
fn row_mut(m: &mut [f64], width: usize, i: usize) -> Option<&mut [f64]> {
    m.get_mut(i * width..(i + 1) * width)
}

/// Adds `v` to `m[i]`. Every caller's `i` is in range by construction; a debug build checks it.
fn add_at(m: &mut [f64], i: usize, v: f64) {
    debug_assert!(i < m.len(), "index {i} is past the {} values", m.len());
    if let Some(x) = m.get_mut(i) {
        *x += v;
    }
}

/// The error of a system with no usable pivot.
fn singular() -> Error {
    Error::Numerical("singular linear system".into())
}

/// The error of an input slice shorter than the system needs.
fn too_short(what: &str, need: usize, got: usize) -> Error {
    Error::InvalidInput(format!("{what} needs {need} values, not {got}"))
}

/// The error of a right-hand side shorter than `n` rows of `k`.
fn short_rhs(n: usize, k: usize, got: usize) -> Error {
    too_short("the right-hand side", n * k, got)
}

/// Swaps rows `i` and `j` of a row-major matrix `width` values wide.
fn swap_rows(m: &mut [f64], width: usize, i: usize, j: usize) {
    let (lo, hi) = (i.min(j), i.max(j));
    if lo == hi || width == 0 {
        return;
    }
    let (head, tail) = m.split_at_mut(hi * width);
    let rows = (head.chunks_exact_mut(width).nth(lo), tail.chunks_exact_mut(width).next());
    if let (Some(a), Some(b)) = rows {
        a.swap_with_slice(b);
    }
}

/// Solves `a x = b` by Gaussian elimination with partial pivoting. `a` is row-major `n × n`, and
/// `b` holds `k` right-hand sides as `n` rows of `k`.
pub fn solve_dense(mut a: Vec<f64>, mut b: Vec<f64>, n: usize, k: usize) -> Result<Vec<f64>> {
    let (a_len, b_len) = (a.len(), b.len());
    let matrix = a.get_mut(..n * n).ok_or_else(|| too_short("the matrix", n * n, a_len))?;
    let rhs = b.get_mut(..n * k).ok_or_else(|| too_short("the right-hand side", n * k, b_len))?;
    // Forward elimination: below each pivot, every row loses its multiple of the pivot row.
    for col in 0..n {
        let (pivot, size) = matrix
            .chunks_exact(n)
            .enumerate()
            .skip(col)
            .filter_map(|(r, values)| Some((r, values.get(col)?.abs())))
            .max_by(|x, y| x.1.total_cmp(&y.1))
            .ok_or_else(singular)?;
        if size < 1e-300 {
            return Err(singular());
        }
        swap_rows(matrix, n, pivot, col);
        swap_rows(rhs, k, pivot, col);
        let (above, below) = matrix.split_at_mut((col + 1) * n);
        let (rhs_above, rhs_below) = rhs.split_at_mut((col + 1) * k);
        let pivot_row = row(above, n, col).ok_or_else(singular)?;
        let pivot_rhs = row(rhs_above, k, col).ok_or_else(singular)?;
        let Some((&lead, pivot_rest)) = pivot_row.split_at(col).1.split_first() else {
            return Err(singular());
        };
        for (r, values) in below.chunks_exact_mut(n).enumerate() {
            let Some((first, rest)) = values.split_at_mut(col).1.split_first_mut() else {
                continue;
            };
            let f = *first / lead;
            if f == 0.0 {
                continue;
            }
            *first -= f * lead;
            for (x, &p) in rest.iter_mut().zip(pivot_rest) {
                *x -= f * p;
            }
            for (x, &p) in row_mut(rhs_below, k, r).into_iter().flatten().zip(pivot_rhs) {
                *x -= f * p;
            }
        }
    }
    // Back substitution, last row first.
    for col in (0..n).rev() {
        let values = row(matrix, n, col).ok_or_else(singular)?;
        let Some((&lead, rest)) = values.split_at(col).1.split_first() else {
            return Err(singular());
        };
        let (done_and_this, later) = rhs.split_at_mut((col + 1) * k);
        let this = row_mut(done_and_this, k, col).ok_or_else(singular)?;
        for (j, x) in this.iter_mut().enumerate() {
            let s = rest
                .iter()
                .zip(later.iter().skip(j).step_by(k))
                .fold(*x, |s, (&a, &y)| s - a * y);
            *x = s / lead;
        }
    }
    Ok(b)
}

/// One row of a sparse system: its first column and the values of the columns that follow.
pub(crate) type SparseRow = (usize, Vec<f64>);

/// A row still being eliminated: where its run of columns starts, the run, and its right-hand
/// sides.
struct Pending {
    start: usize,
    values: Vec<f64>,
    rhs: Vec<f64>,
}

impl Pending {
    fn lead(&self) -> f64 {
        self.values.first().copied().unwrap_or(0.0)
    }
}

/// Solves `a x = b` where each row of `a` is a short run of columns, by Gaussian elimination with
/// partial pivoting among the rows that reach each column. Banded rows (B-spline collocation) cost
/// `O(n w²)` time and `O(n w)` memory for bandwidth `w`; `b` is as in [`solve_dense`].
pub fn solve_banded(rows: Vec<SparseRow>, b: Vec<f64>, k: usize) -> Result<Vec<f64>> {
    let n = rows.len();
    let mut pending = rows
        .into_iter()
        .enumerate()
        .map(|(r, (start, values))| {
            let rhs = row(&b, k, r).ok_or_else(|| short_rhs(n, k, b.len()))?;
            Ok(Pending { start, values, rhs: rhs.to_vec() })
        })
        .collect::<Result<Vec<_>>>()?;
    // A stable sort: rows starting in one column keep their order.
    pending.sort_by_key(|p| p.start);
    let mut pending = pending.into_iter().peekable();
    let mut active: Vec<Pending> = Vec::new();
    // The pivot row of each column, for back substitution.
    let mut pivots: Vec<Pending> = Vec::with_capacity(n);
    for col in 0..n {
        while let Some(p) = pending.next_if(|p| p.start <= col) {
            active.push(p);
        }
        let pivot = active
            .iter()
            .enumerate()
            .filter(|(_, p)| p.start == col)
            .map(|(i, p)| (i, p.lead().abs()))
            .max_by(|x, y| x.1.total_cmp(&y.1))
            .filter(|&(_, size)| size >= 1e-300)
            .ok_or_else(singular)?
            .0;
        let pivot = active.swap_remove(pivot);
        let lead = pivot.values.first().copied().ok_or_else(singular)?;
        for p in &mut active {
            if p.start != col {
                return Err(singular());
            }
            let f = p.lead() / lead;
            let at = |v: &[f64], j: usize| v.get(j).copied().unwrap_or(0.0);
            let len = p.values.len().max(pivot.values.len());
            p.values = (1..len).map(|j| at(&p.values, j) - f * at(&pivot.values, j)).collect();
            p.start = col + 1;
            for (x, &q) in p.rhs.iter_mut().zip(&pivot.rhs) {
                *x -= f * q;
            }
        }
        pivots.push(pivot);
    }
    // Back substitution, last column first: each pivot row's run reaches only later columns.
    let mut x = vec![0.0; n * k];
    for (col, pivot) in pivots.iter().enumerate().rev() {
        let Some((&lead, rest)) = pivot.values.split_first() else {
            return Err(singular());
        };
        if k > 0 && col + 1 + rest.len() > n {
            return Err(Error::InvalidInput(format!("row {col}'s run passes the last column")));
        }
        let (done_and_this, later) = x.split_at_mut((col + 1) * k);
        let this = row_mut(done_and_this, k, col).ok_or_else(singular)?;
        for (j, (out, &r)) in this.iter_mut().zip(&pivot.rhs).enumerate() {
            let s = rest
                .iter()
                .zip(later.iter().skip(j).step_by(k))
                .fold(r, |s, (&a, &y)| s - a * y);
            *out = s / lead;
        }
    }
    Ok(x)
}

/// Solves `a x = b` where each row of `a` is a short run of columns that may wrap past the last
/// column to the first, as a periodic spline's collocation does. Each row's largest value should
/// lie on the diagonal: the last `w` rows and columns (`w` the longest run) are solved as a border
/// round the banded rest, so the cost stays linear in `n`.
pub fn solve_cyclic(rows: Vec<SparseRow>, b: Vec<f64>, k: usize) -> Result<Vec<f64>> {
    let n = rows.len();
    let width = rows.iter().map(|(_, values)| values.len()).max().unwrap_or(0);
    if n <= 4 * width + 4 {
        let mut a = vec![0.0; n * n];
        for (r, (start, values)) in rows.iter().enumerate() {
            for (j, &v) in values.iter().enumerate() {
                add_at(&mut a, r * n + (start + j) % n, v);
            }
        }
        return solve_dense(a, b, n, k);
    }
    if width == 0 {
        // Every row is empty.
        return Err(singular());
    }
    let Bordered { band_rows, band_rhs, border } = Bordered::split(rows, &b, k, width)?;
    let wide = k + width;
    let y = solve_banded(band_rows, band_rhs, wide)?;
    let border_rhs = b
        .get(border.inner * k..n * k)
        .ok_or_else(|| short_rhs(n, k, b.len()))?;
    let (schur, schur_rhs) = border.schur_complement(border_rhs, &y)?;
    let border = solve_dense(schur, schur_rhs, width, k)?;
    // Each inner unknown is its banded solution less its border columns' share of the border's.
    let mut x = Vec::with_capacity(n * k);
    for y_row in y.chunks_exact(wide) {
        let (y_b, y_border) = y_row.split_at(k);
        x.extend(y_b.iter().enumerate().map(|(j, &yv)| {
            y_border
                .iter()
                .zip(border.iter().skip(j).step_by(k))
                .fold(yv, |v, (&yq, &bq)| v - yq * bq)
        }));
    }
    x.extend_from_slice(&border);
    Ok(x)
}

/// A cyclic system of `n` rows split round its last `width` rows and columns: the inner rows as a
/// banded system whose right-hand sides are the `k` of `b` followed by the inner rows' values in
/// the border columns, and the border rows.
struct Bordered {
    band_rows: Vec<SparseRow>,
    band_rhs: Vec<f64>,
    border: Border,
}

/// The border rows of a [`Bordered`] system: their values in the `inner` columns (`lower`,
/// `width × inner`) and in the border columns (`corner`, `width × width`), for `k` right-hand
/// sides.
struct Border {
    k: usize,
    width: usize,
    inner: usize,
    lower: Vec<f64>,
    corner: Vec<f64>,
}

impl Bordered {
    /// Splits `rows`, each a run of columns from its start, wrapping past the last.
    fn split(rows: Vec<SparseRow>, b: &[f64], k: usize, width: usize) -> Result<Bordered> {
        let n = rows.len();
        let inner = n - width;
        let wide = k + width;
        let mut system = Bordered {
            band_rows: Vec::with_capacity(inner),
            band_rhs: vec![0.0; inner * wide],
            border: Border {
                k,
                width,
                inner,
                lower: vec![0.0; width * inner],
                corner: vec![0.0; width * width],
            },
        };
        for (r, (start, values)) in rows.into_iter().enumerate() {
            let columns = values.into_iter().enumerate().map(|(j, v)| ((start + j) % n, v));
            if let Some(border_row) = r.checked_sub(inner) {
                let border = &mut system.border;
                let lower_row = row_mut(&mut border.lower, inner, border_row).ok_or_else(singular)?;
                let corner_row =
                    row_mut(&mut border.corner, width, border_row).ok_or_else(singular)?;
                for (c, v) in columns {
                    match c.checked_sub(inner) {
                        None => add_at(lower_row, c, v),
                        Some(q) => add_at(corner_row, q, v),
                    }
                }
                continue;
            }
            let rhs_row = row_mut(&mut system.band_rhs, wide, r).ok_or_else(singular)?;
            let (rhs, border_columns) = rhs_row.split_at_mut(k);
            let mut band: Vec<(usize, f64)> = Vec::new();
            for (c, v) in columns {
                match c.checked_sub(inner) {
                    None => band.push((c, v)),
                    Some(q) => add_at(border_columns, q, v),
                }
            }
            let lo = band.iter().map(|e| e.0).min().unwrap_or(r);
            let hi = band.iter().map(|e| e.0).max().unwrap_or(r);
            let mut run = vec![0.0; hi + 1 - lo];
            for (c, v) in band {
                add_at(&mut run, c - lo, v);
            }
            system.band_rows.push((lo, run));
            let b_row = row(b, k, r).ok_or_else(|| short_rhs(n, k, b.len()))?;
            rhs.copy_from_slice(b_row);
        }
        Ok(system)
    }
}

impl Border {
    /// The border's Schur complement `corner − lower · y_border` and its right-hand side
    /// `b_border − lower · y_b`, given the banded solution `y` (rows of `k + width`).
    fn schur_complement(self, border_rhs: &[f64], y: &[f64]) -> Result<(Vec<f64>, Vec<f64>)> {
        let Border { k, width, inner, lower, corner } = self;
        let mut schur = corner;
        let mut schur_rhs = border_rhs.to_vec();
        for r in 0..width {
            let lower_row = row(&lower, inner, r).ok_or_else(singular)?;
            let schur_row = row_mut(&mut schur, width, r).ok_or_else(singular)?;
            for (&l, y_row) in lower_row.iter().zip(y.chunks_exact(k + width)) {
                if l == 0.0 {
                    continue;
                }
                let (y_b, y_border) = y_row.split_at(k);
                for (s, &yv) in schur_row.iter_mut().zip(y_border) {
                    *s -= l * yv;
                }
                for (s, &yv) in row_mut(&mut schur_rhs, k, r).into_iter().flatten().zip(y_b) {
                    *s -= l * yv;
                }
            }
        }
        Ok((schur, schur_rhs))
    }
}

/// Solves the 2 × 2 system `[a b; c d] x = r`, or `None` when it is singular.
pub fn solve2(a: f64, b: f64, c: f64, d: f64, r0: f64, r1: f64) -> Option<(f64, f64)> {
    let det = a * d - b * c;
    let scale = (a.abs() + b.abs()) * (c.abs() + d.abs());
    if det.abs() <= 1e-300 || det.abs() <= 1e-14 * scale {
        return None;
    }
    Some(((r0 * d - b * r1) / det, (a * r1 - c * r0) / det))
}

/// A root of `f` in [a, b] where `f(a)` and `f(b)` differ in sign, by bisection to full precision.
pub fn bisect(f: &mut impl FnMut(f64) -> f64, mut a: f64, mut b: f64) -> f64 {
    let mut fa = f(a);
    for _ in 0..200 {
        let m = 0.5 * (a + b);
        if m <= a || m >= b {
            break;
        }
        let fm = f(m);
        if fm == 0.0 {
            return m;
        }
        if (fm < 0.0) == (fa < 0.0) {
            a = m;
            fa = fm;
        } else {
            b = m;
        }
    }
    0.5 * (a + b)
}

/// Every root of `f` on [a, b] found by sampling `n` intervals and bisecting each sign change, plus
/// sampled points where |f| touches zero without changing sign.
pub fn roots(f: &mut impl FnMut(f64) -> f64, a: f64, b: f64, n: usize) -> Vec<f64> {
    let mut out = Vec::new();
    let mut prev_t = a;
    let mut prev = f(a);
    if prev == 0.0 {
        out.push(a);
    }
    for i in 1..=n {
        let t = a + (b - a) * i as f64 / n as f64;
        let v = f(t);
        if v == 0.0 {
            out.push(t);
        } else if prev != 0.0 && (v < 0.0) != (prev < 0.0) {
            out.push(bisect(f, prev_t, t));
        }
        prev_t = t;
        prev = v;
    }
    out
}

// The Gauss-Legendre rules and their quadrature, from springrep's numeric.rs (M3 W5). The tables
// are springrep's Newton iteration for the Legendre roots (`tests::legendre_rule`), printed with
// round-trip precision; a test checks them bit for bit.
/// The 8-point Gauss-Legendre rule on [-1, 1] as `(node, weight)`, largest node first.
const GAUSS_8: [(f64, f64); 8] = [
    (0.9602898564975363, 0.10122853629037679),
    (0.7966664774136268, 0.22238103445337445),
    (0.525532409916329, 0.3137066458778874),
    (0.18343464249564978, 0.362683783378362),
    (-0.18343464249564978, 0.362683783378362),
    (-0.525532409916329, 0.3137066458778874),
    (-0.7966664774136268, 0.22238103445337445),
    (-0.9602898564975363, 0.10122853629037679),
];

/// The 12-point Gauss-Legendre rule on [-1, 1] as `(node, weight)`, largest node first.
const GAUSS_12: [(f64, f64); 12] = [
    (0.9815606342467192, 0.047175336386511835),
    (0.9041172563704748, 0.10693932599531818),
    (0.7699026741943047, 0.16007832854334633),
    (0.5873179542866175, 0.20316742672306584),
    (0.3678314989981802, 0.23349253653835478),
    (0.12523340851146894, 0.24914704581340288),
    (-0.1252334085114689, 0.24914704581340288),
    (-0.3678314989981802, 0.23349253653835478),
    (-0.5873179542866175, 0.20316742672306584),
    (-0.7699026741943047, 0.16007832854334633),
    (-0.9041172563704748, 0.10693932599531818),
    (-0.9815606342467192, 0.047175336386511835),
];

/// The 16-point Gauss-Legendre rule on [-1, 1] as `(node, weight)`, largest node first.
const GAUSS_16: [(f64, f64); 16] = [
    (0.9894009349916499, 0.027152459411754058),
    (0.9445750230732326, 0.062253523938647776),
    (0.8656312023878318, 0.0951585116824929),
    (0.755404408355003, 0.12462897125553395),
    (0.6178762444026438, 0.14959598881657682),
    (0.45801677765722737, 0.16915651939500256),
    (0.2816035507792589, 0.1826034150449236),
    (0.09501250983763744, 0.18945061045506847),
    (-0.09501250983763744, 0.18945061045506847),
    (-0.2816035507792589, 0.1826034150449236),
    (-0.45801677765722737, 0.16915651939500256),
    (-0.6178762444026438, 0.14959598881657682),
    (-0.755404408355003, 0.12462897125553395),
    (-0.8656312023878318, 0.0951585116824929),
    (-0.9445750230732326, 0.062253523938647776),
    (-0.9894009349916499, 0.027152459411754058),
];

/// The 24-point Gauss-Legendre rule on [-1, 1] as `(node, weight)`, largest node first.
const GAUSS_24: [(f64, f64); 24] = [
    (0.9951872199970213, 0.012341229799987334),
    (0.9747285559713095, 0.028531388628933813),
    (0.9382745520027328, 0.044277438817419676),
    (0.8864155270044011, 0.05929858491543666),
    (0.820001985973903, 0.07334648141108027),
    (0.7401241915785544, 0.08619016153195322),
    (0.6480936519369755, 0.0976186521041139),
    (0.5454214713888396, 0.10744427011596562),
    (0.4337935076260452, 0.11550566805372561),
    (0.3150426796961634, 0.12167047292780335),
    (0.1911188674736163, 0.12583745634682839),
    (0.06405689286260563, 0.12793819534675224),
    (-0.06405689286260563, 0.12793819534675224),
    (-0.1911188674736163, 0.12583745634682839),
    (-0.3150426796961634, 0.12167047292780335),
    (-0.4337935076260452, 0.11550566805372561),
    (-0.5454214713888396, 0.10744427011596562),
    (-0.6480936519369755, 0.0976186521041139),
    (-0.7401241915785544, 0.08619016153195322),
    (-0.820001985973903, 0.07334648141108027),
    (-0.8864155270044011, 0.05929858491543666),
    (-0.9382745520027328, 0.044277438817419676),
    (-0.9747285559713095, 0.028531388628933813),
    (-0.9951872199970213, 0.012341229799987334),
];

/// The `n`-point Gauss-Legendre rule on [-1, 1] as `(node, weight)` pairs, for `n` of 8, 12 or
/// 16; any other `n` gives the 24-point rule.
pub fn gauss(n: usize) -> &'static [(f64, f64)] {
    match n {
        8 => &GAUSS_8,
        12 => &GAUSS_12,
        16 => &GAUSS_16,
        _ => &GAUSS_24,
    }
}

/// ∫ f over [a, b] with the `n`-point rule of [`gauss`].
pub(crate) fn integrate_fixed(f: &mut impl FnMut(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let (mid, half) = ((a + b) * 0.5, (b - a) * 0.5);
    gauss(n).iter().map(|&(x, w)| w * f(mid + half * x)).sum::<f64>() * half
}

/// ∫ f over [a, b], halving until two levels agree to `abs_tol` (spread over the interval).
pub fn integrate(f: &mut impl FnMut(f64) -> f64, a: f64, b: f64, abs_tol: f64) -> f64 {
    let whole = integrate_fixed(f, a, b, 16);
    integrate_halves(f, a, b, whole, abs_tol, 0)
}

/// ∫ f over [a, b] given `whole`, its one-rule estimate: the two halves' sum when it agrees with
/// `whole` to `tol` or the depth reaches 24, else each half refined to half the tolerance.
fn integrate_halves(
    f: &mut impl FnMut(f64) -> f64,
    a: f64,
    b: f64,
    whole: f64,
    tol: f64,
    depth: u32,
) -> f64 {
    let mid = (a + b) * 0.5;
    let left = integrate_fixed(f, a, mid, 16);
    let right = integrate_fixed(f, mid, b, 16);
    if (left + right - whole).abs() <= tol || depth >= 24 {
        return left + right;
    }
    integrate_halves(f, a, mid, left, tol * 0.5, depth + 1)
        + integrate_halves(f, mid, b, right, tol * 0.5, depth + 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gauss-Legendre nodes and weights of order `n` on [-1, 1], by Newton's method on the Legendre
    /// polynomial: the generator of the const tables.
    fn legendre_rule(n: usize) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
            let mut dp = 1.0;
            for _ in 0..100 {
                let (mut p0, mut p1) = (1.0, x);
                for k in 2..=n {
                    let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                    p0 = p1;
                    p1 = p2;
                }
                dp = n as f64 * (x * p1 - p0) / (x * x - 1.0);
                let dx = p1 / dp;
                x -= dx;
                if dx.abs() < 1e-16 {
                    break;
                }
            }
            out.push((x, 2.0 / ((1.0 - x * x) * dp * dp)));
        }
        out
    }

    #[test]
    fn the_gauss_tables_are_the_generated_rules_bit_for_bit() {
        for n in [8, 12, 16, 24] {
            let bits = |rule: &[(f64, f64)]| -> Vec<(u64, u64)> {
                rule.iter().map(|&(x, w)| (x.to_bits(), w.to_bits())).collect()
            };
            assert_eq!(bits(gauss(n)), bits(&legendre_rule(n)), "order {n}");
        }
        assert_eq!(gauss(5).len(), 24);
    }

    #[test]
    fn gauss_rules_integrate_polynomials_exactly() {
        for n in [8, 12, 16, 24] {
            let s: f64 = gauss(n).iter().map(|&(_, w)| w).sum();
            assert!((s - 2.0).abs() < 1e-14, "order {n}");
            let mut f = |x: f64| x.powi(2 * n as i32 - 2);
            let v = integrate_fixed(&mut f, 0.0, 1.0, n);
            assert!(
                (v - 1.0 / (2 * n - 1) as f64).abs() < 1e-14,
                "order {n}: {v}"
            );
        }
    }

    #[test]
    fn adaptive_quadrature_handles_a_kink() {
        let v = integrate(&mut |x: f64| (x - 0.3).abs(), 0.0, 1.0, 1e-14);
        assert!((v - (0.045 + 0.245)).abs() < 1e-12);
    }

    /// The Kronrod rule is exact to degree 22 and the Gauss rule inside it to degree 13:
    /// `∫ xᵏ` over [−1, 1] is `2 / (k + 1)` for even `k`.
    #[test]
    fn the_kronrod_pair_integrates_polynomials_exactly() {
        let sum = |k: i32, gauss: bool| -> f64 {
            KRONROD_15
                .iter()
                .map(|&(x, wk, wg)| {
                    let w = if gauss { wg } else { wk };
                    w * x.powi(k) * if x == 0.0 { 1.0 } else { 2.0 }
                })
                .sum()
        };
        for k in (0..=22).step_by(2) {
            assert!(
                (sum(k, false) - 2.0 / (k + 1) as f64).abs() < 1e-15,
                "Kronrod x^{k}"
            );
        }
        for k in (0..=12).step_by(2) {
            assert!(
                (sum(k, true) - 2.0 / (k + 1) as f64).abs() < 1e-15,
                "Gauss x^{k}"
            );
        }
        assert!((sum(14, true) - 2.0 / 15.0).abs() > 1e-6);
    }

    /// The 7-point Kronrod rule is exact to degree 10 and the Gauss rule inside it to degree 5.
    #[test]
    fn the_short_kronrod_pair_integrates_polynomials_exactly() {
        let sum = |k: i32, gauss: bool| -> f64 {
            KRONROD_7
                .iter()
                .map(|&(x, wk, wg)| {
                    let w = if gauss { wg } else { wk };
                    w * x.powi(k) * if x == 0.0 { 1.0 } else { 2.0 }
                })
                .sum()
        };
        for k in (0..=10).step_by(2) {
            assert!(
                (sum(k, false) - 2.0 / (k + 1) as f64).abs() < 1e-15,
                "Kronrod x^{k}"
            );
        }
        for k in (0..=4).step_by(2) {
            assert!(
                (sum(k, true) - 2.0 / (k + 1) as f64).abs() < 1e-15,
                "Gauss x^{k}"
            );
        }
        assert!((sum(12, false) - 2.0 / 13.0).abs() > 1e-6);
    }

    #[test]
    fn banded_and_cyclic_solves_match_dense() {
        let n = 40;
        let mut seed = 7u64;
        let mut rnd = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for cyclic in [false, true] {
            let mut rows = Vec::new();
            let mut a = vec![0.0; n * n];
            for r in 0..n {
                let lo = if cyclic {
                    (r + n - 1) % n
                } else {
                    r.saturating_sub(1)
                };
                let len = if cyclic { 3 } else { 3.min(n - lo) };
                let vals: Vec<f64> = (0..len)
                    .map(|j| {
                        if (lo + j) % n == r {
                            2.0 + rnd()
                        } else {
                            rnd() - 0.5
                        }
                    })
                    .collect();
                for (j, v) in vals.iter().enumerate() {
                    a[r * n + (lo + j) % n] += v;
                }
                rows.push((lo, vals));
            }
            let b: Vec<f64> = (0..2 * n).map(|_| rnd()).collect();
            let want = solve_dense(a, b.clone(), n, 2).unwrap();
            let got = if cyclic {
                solve_cyclic(rows, b, 2).unwrap()
            } else {
                solve_banded(rows, b, 2).unwrap()
            };
            for (g, w) in got.iter().zip(&want) {
                assert!((g - w).abs() < 1e-12, "cyclic {cyclic}: {g} vs {w}");
            }
        }
    }

    #[test]
    fn dense_solve_pivots() {
        let x = solve_dense(vec![0.0, 1.0, 1.0, 1.0], vec![2.0, 3.0], 2, 1).unwrap();
        assert_eq!(x, vec![1.0, 2.0]);
    }
}
