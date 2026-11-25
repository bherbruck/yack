// yack: a knot vector and its B-spline basis: values, derivatives and span polynomials.
// Adapted from springrep@a03e8a1 src/geom/nurbs.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/nurbs.rs to yack-geom (M3 W3); split out of
// nurbs.rs into its own module (M3 hardening). M3 hardening: the recurrences read each function's
// knots and neighbours from one iterator instead of index arithmetic, the domain is the active
// knots' ends by slice pattern; no indexing, the same arithmetic in the same order.
//! A knot vector and its B-spline basis: values, derivatives and span polynomials.

use std::iter::once;

use super::{M, MAX_DEGREE};
use crate::{Result, error::invalid};

/// A knot vector and the degree of the basis it defines.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct Basis {
    pub(super) degree: usize,
    pub(super) knots: Vec<f64>,
    /// The number of basis functions (control points along this parameter).
    pub(super) n: usize,
    /// `1 / (uₖ₊d − uₖ)` for each `k`, zero where the knots coincide, for `d` the degree and
    /// the degree less one: the derivative recurrence divides by these at every evaluation.
    inverse: [Vec<f64>; 2],
}

/// `a / b`, or zero where `b` is: a quotient of the recurrences at a repeated knot.
fn ratio(a: f64, b: f64) -> f64 {
    if b == 0.0 { 0.0 } else { a / b }
}

impl Basis {
    pub(super) fn new(degree: usize, knots: Vec<f64>, n: usize) -> Result<Basis> {
        if degree == 0 || degree > MAX_DEGREE {
            return invalid(format!("B-spline degree {degree} is not 1 to {MAX_DEGREE}"));
        }
        if n <= degree {
            return invalid("a B-spline needs more control points than its degree");
        }
        if knots.len() != n + degree + 1 {
            return invalid("B-spline knot count must be control points + degree + 1");
        }
        let decreases = knots.iter().zip(knots.iter().skip(1)).any(|(a, b)| b < a);
        if knots.iter().any(|k| !k.is_finite()) || decreases {
            return invalid("B-spline knots must be finite and not decrease");
        }
        let inverse = [degree, degree - 1].map(|d| {
            knots
                .iter()
                .zip(knots.iter().skip(d))
                .map(|(&a, &b)| {
                    let den = b - a;
                    if den == 0.0 { 0.0 } else { 1.0 / den }
                })
                .collect()
        });
        let b = Basis { degree, knots, n, inverse };
        let (a, z) = b.domain();
        if z - a <= 0.0 {
            return invalid("B-spline has an empty domain");
        }
        Ok(b)
    }

    /// The knots from the domain's start to its end, `knots[degree..=n]`.
    fn active(&self) -> &[f64] {
        self.knots.get(self.degree..=self.n).unwrap_or_default()
    }

    pub(super) fn domain(&self) -> (f64, f64) {
        debug_assert!(self.active().len() >= 2, "`new` keeps two active knots at least");
        match *self.active() {
            [start, .., end] => (start, end),
            // `new` keeps more control points than the degree: two knots at least.
            _ => (f64::NAN, f64::NAN),
        }
    }

    pub(super) fn wrap(&self, t: f64, period: Option<f64>) -> f64 {
        let (a, b) = self.domain();
        match period {
            Some(p) => {
                let w = a + (t - a).rem_euclid(p);
                if w >= b { a } else { w }
            }
            None => t,
        }
    }

    /// The parameter range `[knots[i], knots[i + 1]]` of knot span `i`; no numbers for a span
    /// past the knots, which [`Basis::span`] never gives.
    pub(super) fn span_range(&self, i: usize) -> (f64, f64) {
        debug_assert!(i + 1 < self.knots.len(), "span {i} lies in the knots");
        match self.knots.get(i..=i + 1) {
            Some(&[start, end]) => (start, end),
            _ => (f64::NAN, f64::NAN),
        }
    }

    /// The nonempty knot span `i` (`knots[i] ≤ t < knots[i + 1]`, `degree ≤ i < n`) that holds
    /// `t`; past an end, the end's span.
    pub(super) fn span(&self, t: f64) -> usize {
        let (p, m) = (self.degree, self.n);
        let (start, end) = self.domain();
        let knot = |i: usize| self.knots.get(i).copied();
        if t >= end {
            let mut i = m - 1;
            while i > p && knot(i).is_some_and(|k| k >= end) {
                i -= 1;
            }
            return i;
        }
        if t <= start {
            let mut i = p;
            while i + 1 < m && knot(i + 1).is_some_and(|k| k <= start) {
                i += 1;
            }
            return i;
        }
        let (mut lo, mut hi) = (p, m);
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if knot(mid).is_some_and(|k| t < k) { hi = mid } else { lo = mid }
        }
        lo
    }

    /// Basis values and derivatives up to `nd` (at most 2) at `t` in span `i`: `out[k][j]` is the
    /// `k`-th derivative of function `i − degree + j`. Values are raised a degree at a time,
    /// keeping the two lower degrees for the derivative recurrence.
    /// `N` is any length past the degree: the arrays are only that long, so a low degree does
    /// not clear and copy the room a degree-15 basis needs.
    pub(super) fn ders<const N: usize>(&self, i: usize, t: f64, nd: usize) -> [[f64; N]; 3] {
        let p = self.degree;
        // The values at the degree reached, then at one and two degrees below it.
        let mut n = [0.0; N];
        let mut lower = [[0.0; N]; 2];
        if let Some(constant) = n.first_mut() {
            *constant = 1.0;
        }
        for q in 1..=p {
            lower[1] = lower[0];
            lower[0] = n;
            n = raised(&self.knots, i, t, q, &lower[0]);
        }
        let mut out = [n, [0.0; N], [0.0; N]];
        if nd >= 1 {
            out[1] = self.deriv(&lower[0], p, i);
        }
        if nd >= 2 && p >= 2 {
            let below = self.deriv(&lower[1], p - 1, i);
            out[2] = self.deriv(&below, p, i);
        }
        out
    }

    /// `f` of [`Basis::ders`] in arrays sized for the degree.
    pub(super) fn ders_then<R>(
        &self,
        i: usize,
        t: f64,
        nd: usize,
        f: impl FnOnce([&[f64]; 3]) -> R,
    ) -> R {
        match self.degree {
            0..4 => {
                let [values, d1, d2] = self.ders::<4>(i, t, nd);
                f([&values, &d1, &d2])
            }
            4..8 => {
                let [values, d1, d2] = self.ders::<8>(i, t, nd);
                f([&values, &d1, &d2])
            }
            _ => {
                let [values, d1, d2] = self.ders::<M>(i, t, nd);
                f([&values, &d1, &d2])
            }
        }
    }

    /// The derivatives of the degree-`deg` functions on span `i` from the values (or derivatives)
    /// `a` of the degree-`deg − 1` ones:
    /// `N'ₖ,d = d·(Nₖ,d−1 / (uₖ₊d − uₖ) − Nₖ₊₁,d−1 / (uₖ₊d₊₁ − uₖ₊₁))`.
    fn deriv<const N: usize>(&self, a: &[f64; N], deg: usize, i: usize) -> [f64; N] {
        let mut out = [0.0; N];
        debug_assert!(self.degree - deg < 2, "derivatives come from the degree or one below");
        let Some(inv) = self.inverse.get(self.degree - deg) else {
            return out;
        };
        let d = deg as f64;
        // Function `j` (`k = i − deg + j`) takes `1 / (u_{k+d} − u_k)` and the next.
        let inverses = inv.iter().skip(i - deg).zip(inv.iter().skip(i - deg + 1));
        let lower = once(None).chain(a.iter().map(Some)).zip(a);
        for (j, (o, ((&inv_k, &inv_next), (left, &right)))) in
            out.iter_mut().zip(inverses.zip(lower)).enumerate().take(deg + 1)
        {
            let mut v = 0.0;
            if let Some(&left) = left {
                v += left * inv_k;
            }
            if j < deg {
                v -= right * inv_next;
            }
            *o = d * v;
        }
        out
    }

    pub(super) fn span_poly(&self, i: usize, end: bool) -> Vec<Vec<f64>> {
        span_poly(self.degree, &self.knots, i, end)
    }

    /// The distinct knots strictly inside (t0, t1), repeated each period when periodic.
    pub(super) fn breakpoints(&self, t0: f64, t1: f64, period: Option<f64>) -> Vec<f64> {
        let (a, b) = self.domain();
        let inner = self.active();
        let mut out: Vec<f64> = Vec::new();
        let mut push = |k: f64| {
            if k > t0 && k < t1 && out.last().is_none_or(|&l| k > l) {
                out.push(k);
            }
        };
        match period {
            Some(p) => {
                let start = ((t0 - a) / p).floor() as i64;
                let end = ((t1 - a) / p).ceil() as i64;
                for w in start..=end {
                    for &k in inner {
                        push(k + w as f64 * p);
                    }
                }
            }
            None => {
                for &k in inner {
                    if k > a && k < b {
                        push(k);
                    }
                }
            }
        }
        out
    }

    /// The distinct knots of the domain, ends included.
    pub(super) fn spans(&self) -> Vec<f64> {
        let mut out: Vec<f64> = Vec::new();
        for &k in self.active() {
            if out.last().is_none_or(|&l| k > l) {
                out.push(k);
            }
        }
        out
    }
}

pub(super) fn check_weights(weights: &Option<Vec<f64>>, n: usize) -> Result<()> {
    if let Some(w) = weights {
        if w.len() != n {
            return invalid("B-spline weight count must match its control points");
        }
        if w.iter().any(|&x| !(x > 0.0 && x.is_finite())) {
            return invalid("B-spline weights must be positive");
        }
    }
    Ok(())
}

/// For each degree-`q` function `j` nonzero on span `i` (function `f = i − q + j`), its knots
/// `[u_f, u_{f+1}, u_{f+q}, u_{f+q+1}]` and the values in `below` of its two neighbours of degree
/// `q − 1`: entry `j − 1` and entry `j`, `None` past either end.
fn neighbours<'a, const N: usize>(
    u: &'a [f64],
    i: usize,
    q: usize,
    below: &'a [f64; N],
) -> impl Iterator<Item = ([f64; 4], Option<f64>, Option<f64>)> + 'a {
    debug_assert!(i + q + 1 < u.len(), "the knots of span {i}'s degree-{q} functions exist");
    let from = u.get(i - q..).unwrap_or_default();
    let knots = from
        .iter()
        .zip(from.iter().skip(1))
        .zip(from.iter().skip(q))
        .zip(from.iter().skip(q + 1))
        .map(|(((&start, &next_start), &end), &next_end)| [start, next_start, end, next_end]);
    let left = once(None).chain(below.iter().copied().map(Some));
    let right = below.iter().copied().take(q).map(Some).chain(once(None));
    knots.zip(left).zip(right).take(q + 1).map(|((knots, left), right)| (knots, left, right))
}

/// The values at `t` of the degree-`q` B-spline functions of knots `u` nonzero on span `i`, from
/// those of degree `q − 1` in `below`: entry `j` is function `i − q + j`. A function of degree q
/// is the blend of its two neighbours of degree q − 1, each weighted by where `t` lies in its
/// support; a neighbour that is zero on this span drops out, and a quotient with a zero
/// denominator, at a repeated knot, counts as zero.
fn raised<const N: usize>(u: &[f64], i: usize, t: f64, q: usize, below: &[f64; N]) -> [f64; N] {
    let mut out = [0.0; N];
    for (o, ([start, next_start, end, next_end], left, right)) in
        out.iter_mut().zip(neighbours(u, i, q, below))
    {
        if let Some(left) = left {
            *o += ratio(t - start, end - start) * left;
        }
        if let Some(right) = right {
            *o += ratio(next_end - t, next_end - next_start) * right;
        }
    }
    out
}

/// The values and first `nd` derivatives at `t` of the degree-`p` B-spline functions of knots `u`
/// that are nonzero on span `i`: `out[k][j]` is the `k`-th derivative of function `i − p + j`.
pub(crate) fn basis_derivatives<const N: usize>(
    p: usize,
    u: &[f64],
    i: usize,
    t: f64,
    nd: usize,
) -> [[f64; N]; N] {
    debug_assert!(p < N, "the arrays hold the degree's {} functions", p + 1);
    // by_degree[q][j] is the value of the degree-q function `i − q + j`.
    let mut by_degree = [[0.0; N]; N];
    if let Some(constant) = by_degree.first_mut().and_then(|row| row.first_mut()) {
        *constant = 1.0;
    }
    for q in 1..=p {
        let Some(below) = by_degree.get(q - 1).copied() else {
            break;
        };
        if let Some(row) = by_degree.get_mut(q) {
            *row = raised(u, i, t, q, &below);
        }
    }
    // A derivative of a degree-q function is q times the difference of its two neighbours of
    // degree q − 1, each over the width of its support. So the k-th derivative at degree p is
    // the values at degree p − k, differenced k times on the way back up.
    let mut out = [[0.0; N]; N];
    for (k, slot) in out.iter_mut().enumerate().take(nd.min(p) + 1) {
        let mut row = by_degree.get(p - k).copied().unwrap_or([0.0; N]);
        for q in p - k + 1..=p {
            let mut raised = [0.0; N];
            for (r, ([start, next_start, end, next_end], left, right)) in
                raised.iter_mut().zip(neighbours(u, i, q, &row))
            {
                let mut d = 0.0;
                if let Some(left) = left {
                    d += ratio(left, end - start);
                }
                if let Some(right) = right {
                    d -= ratio(right, next_end - next_start);
                }
                *r = q as f64 * d;
            }
            row = raised;
        }
        *slot = row;
    }
    out
}

/// The degree-`p` B-spline functions of knots `u` nonzero on span `i`, as polynomials in
/// `s = (t − e)/(tᵢ₊₁ − tᵢ)`, `e` the span's start, or its end when `end`: `out[j][k]` is the
/// coefficient of `sᵏ` in function `i − p + j`. Each is the Taylor series at `e`, from every
/// derivative there, scaled by the span's width so the coefficients stay of the functions' own
/// size.
pub(crate) fn span_poly(p: usize, u: &[f64], i: usize, end: bool) -> Vec<Vec<f64>> {
    debug_assert!(i + 1 < u.len(), "span {i} lies in the knots");
    let (Some(&start), Some(&stop)) = (u.get(i), u.get(i + 1)) else {
        return vec![vec![0.0; p + 1]; p + 1];
    };
    let e = if end { stop } else { start };
    let width = stop - start;
    let ders = basis_derivatives::<M>(p, u, i, e, p);
    // The series term of order k is the k-th derivative over k!, times width^k.
    let mut out = vec![Vec::with_capacity(p + 1); p + 1];
    let mut scale = 1.0;
    for (k, derivative) in ders.iter().enumerate().take(p + 1) {
        for (series, &value) in out.iter_mut().zip(derivative) {
            series.push(value * scale);
        }
        scale *= width / (k + 1) as f64;
    }
    out
}
