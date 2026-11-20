// yack: B-spline basis functions and rational curve and surface evaluation, from their definitions.
//! Non-uniform rational B-splines as ISO 10303-42 defines them (`b_spline_curve_with_knots`,
//! `b_spline_surface_with_knots`, and their rational forms): the knot vector expanded from its
//! multiplicities, the Cox–de Boor basis and its first derivatives, and the weighted sums.

use yack_math::{Point3, Vec3};

/// A knot vector with its degree.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Basis {
    pub(crate) degree: usize,
    pub(crate) knots: Vec<f64>,
}

impl Basis {
    /// The knot vector from distinct knots and their multiplicities, as the standard writes it.
    pub(crate) fn from_multiplicities(degree: usize, mults: &[i64], knots: &[f64]) -> Option<Basis> {
        if degree == 0 || mults.len() != knots.len() {
            return None;
        }
        let mut k = Vec::new();
        for (&m, &x) in mults.iter().zip(knots) {
            let m = usize::try_from(m).ok()?;
            k.extend(std::iter::repeat_n(x, m));
        }
        Some(Basis { degree, knots: k })
    }

    /// The number of control points this basis spans.
    pub(crate) fn count(&self) -> usize {
        self.knots.len().saturating_sub(self.degree + 1)
    }

    /// The parameter range: knots `[p]` to `[n]`.
    pub(crate) fn domain(&self) -> (f64, f64) {
        let a = self.knots.get(self.degree).copied().unwrap_or(0.0);
        let b = self.knots.get(self.count()).copied().unwrap_or(1.0);
        (a, b)
    }

    /// The knot span holding `t` (clamped to the domain).
    fn span(&self, t: f64) -> usize {
        let p = self.degree;
        let n = self.count();
        if n == 0 {
            return p;
        }
        let (a, b) = self.domain();
        if t >= b {
            // The last non-empty span.
            let mut s = n - 1;
            while s > p && self.knots.get(s).copied() == self.knots.get(s + 1).copied() {
                s -= 1;
            }
            return s;
        }
        if t <= a {
            return p;
        }
        let (mut lo, mut hi) = (p, n);
        while hi - lo > 1 {
            let mid = (lo + hi) / 2;
            if t < self.knots.get(mid).copied().unwrap_or(f64::INFINITY) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        lo
    }

    /// The span and the `degree + 1` nonzero basis functions there with their first derivatives.
    pub(crate) fn eval(&self, t: f64) -> (usize, Vec<f64>, Vec<f64>) {
        let p = self.degree;
        let s = self.span(t);
        let k = |i: usize| self.knots.get(i).copied().unwrap_or(0.0);
        // Basis functions of every degree up to p (the triangle of Cox–de Boor).
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        if let Some(c) = ndu.get_mut(0).and_then(|r| r.get_mut(0)) {
            *c = 1.0;
        }
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        for j in 1..=p {
            if let Some(l) = left.get_mut(j) {
                *l = t - k(s + 1 - j);
            }
            if let Some(r) = right.get_mut(j) {
                *r = k(s + j) - t;
            }
            let mut saved = 0.0;
            for r in 0..j {
                let rr = right.get(r + 1).copied().unwrap_or(0.0);
                let ll = left.get(j - r).copied().unwrap_or(0.0);
                let den = rr + ll;
                let below = ndu.get(r).and_then(|row| row.get(j - 1)).copied().unwrap_or(0.0);
                let temp = if den != 0.0 { below / den } else { 0.0 };
                // The knot differences above the diagonal, the functions on and below it.
                if let Some(c) = ndu.get_mut(j).and_then(|row| row.get_mut(r)) {
                    *c = den;
                }
                if let Some(c) = ndu.get_mut(r).and_then(|row| row.get_mut(j)) {
                    *c = saved + rr * temp;
                }
                saved = ll * temp;
            }
            if let Some(c) = ndu.get_mut(j).and_then(|row| row.get_mut(j)) {
                *c = saved;
            }
        }
        let n: Vec<f64> = (0..=p).map(|j| ndu.get(j).and_then(|row| row.get(p)).copied().unwrap_or(0.0)).collect();
        // First derivatives: p · (N_{j,p−1}/(k_{j+p}−k_j) − N_{j+1,p−1}/(k_{j+p+1}−k_{j+1})).
        let lower = |j: usize| -> f64 {
            // N_{s−p+j, p−1}(t), j in 0..=p (0 outside 0..p).
            if j >= p { 0.0 } else { ndu.get(j).and_then(|row| row.get(p - 1)).copied().unwrap_or(0.0) }
        };
        let mut d = vec![0.0; p + 1];
        for (j, dj) in d.iter_mut().enumerate() {
            let i = s - p + j;
            let a = {
                let den = k(i + p) - k(i);
                if j >= 1 && den != 0.0 { lower(j - 1) / den } else { 0.0 }
            };
            let b = {
                let den = k(i + p + 1) - k(i + 1);
                if den != 0.0 { lower(j) / den } else { 0.0 }
            };
            *dj = p as f64 * (a - b);
        }
        (s, n, d)
    }
}

/// Homogeneous point: `w·p` and `w`.
#[derive(Clone, Copy, Debug, Default)]
struct H {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

impl H {
    fn of(p: Point3, w: f64) -> H {
        H { x: p.x * w, y: p.y * w, z: p.z * w, w }
    }
    fn add(self, o: H, s: f64) -> H {
        H { x: self.x + o.x * s, y: self.y + o.y * s, z: self.z + o.z * s, w: self.w + o.w * s }
    }
}

/// The point and first derivative of the rational curve from homogeneous `a` (point) and `da`.
fn project(a: H, da: H) -> (Point3, Vec3) {
    let w = if a.w != 0.0 { a.w } else { 1.0 };
    let p = Point3::new(a.x / w, a.y / w, a.z / w);
    let d = Vec3::new((da.x - p.x * da.w) / w, (da.y - p.y * da.w) / w, (da.z - p.z * da.w) / w);
    (p, d)
}

/// `x` moved by whole periods into `[lo, lo + period)` where the direction is closed.
pub(crate) fn wrap(x: f64, lo: f64, period: Option<f64>) -> f64 {
    match period {
        Some(p) if p > 0.0 && x.is_finite() => lo + (x - lo).rem_euclid(p),
        _ => x,
    }
}

/// A rational B-spline curve (weights all 1 when polynomial).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Curve {
    pub(crate) basis: Basis,
    pub(crate) ctrl: Vec<Point3>,
    pub(crate) weights: Vec<f64>,
    /// The domain's length where the curve closes on itself (its ends meet): then it is read as
    /// periodic, so that an edge or a surface may run across its join.
    pub(crate) period: Option<f64>,
}

impl Curve {
    /// The point and first derivative at `t`.
    pub(crate) fn d1(&self, t: f64) -> (Point3, Vec3) {
        let t = wrap(t, self.basis.domain().0, self.period);
        let (s, n, d) = self.basis.eval(t);
        let p = self.basis.degree;
        let (mut a, mut da) = (H::default(), H::default());
        for j in 0..=p {
            let i = s - p + j;
            let (Some(&c), Some(&w)) = (self.ctrl.get(i), self.weights.get(i)) else { continue };
            let h = H::of(c, w);
            a = a.add(h, n.get(j).copied().unwrap_or(0.0));
            da = da.add(h, d.get(j).copied().unwrap_or(0.0));
        }
        project(a, da)
    }
}

/// A rational B-spline surface: `ctrl[i][j]`, i along u, j along v.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Surface {
    pub(crate) u: Basis,
    pub(crate) v: Basis,
    pub(crate) ctrl: Vec<Vec<Point3>>,
    pub(crate) weights: Vec<Vec<f64>>,
    /// The periods of the directions in which the surface closes on itself (as [`Curve::period`]).
    pub(crate) periods: [Option<f64>; 2],
}

impl Surface {
    /// The point and its partial derivatives at `(u, v)`.
    pub(crate) fn d1(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        let u = wrap(u, self.u.domain().0, self.periods[0]);
        let v = wrap(v, self.v.domain().0, self.periods[1]);
        let (su, nu, du) = self.u.eval(u);
        let (sv, nv, dv) = self.v.eval(v);
        let (p, q) = (self.u.degree, self.v.degree);
        let (mut a, mut au, mut av) = (H::default(), H::default(), H::default());
        for (ii, (&n_u, &d_u)) in nu.iter().zip(&du).enumerate() {
            let i = su - p + ii;
            for (jj, (&n_v, &d_v)) in nv.iter().zip(&dv).enumerate() {
                let j = sv - q + jj;
                let (Some(&c), Some(&w)) = (self.ctrl.get(i).and_then(|r| r.get(j)), self.weights.get(i).and_then(|r| r.get(j))) else {
                    continue;
                };
                let h = H::of(c, w);
                a = a.add(h, n_u * n_v);
                au = au.add(h, d_u * n_v);
                av = av.add(h, n_u * d_v);
            }
        }
        let (pt, su_) = project(a, au);
        let (_, sv_) = project(a, av);
        (pt, su_, sv_)
    }

    /// The parameter ranges in u and v.
    pub(crate) fn domain(&self) -> ((f64, f64), (f64, f64)) {
        (self.u.domain(), self.v.domain())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_quarter_circle_as_a_rational_quadratic() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let c = Curve {
            basis: Basis::from_multiplicities(2, &[3, 3], &[0.0, 1.0]).unwrap(),
            ctrl: vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            weights: vec![1.0, w, 1.0],
            period: None,
        };
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            let (p, d) = c.d1(t);
            assert!((p.x * p.x + p.y * p.y - 1.0).abs() < 1e-14, "{p:?}");
            assert!((p.x * d.x + p.y * d.y).abs() < 1e-12, "tangent {d:?}");
        }
        let (p, _) = c.d1(1.0);
        assert!((p.x).abs() < 1e-15 && (p.y - 1.0).abs() < 1e-15);
    }

    #[test]
    fn a_bilinear_patch_and_its_partials() {
        let b = Basis::from_multiplicities(1, &[2, 2], &[0.0, 1.0]).unwrap();
        let s = Surface {
            u: b.clone(),
            v: b,
            ctrl: vec![vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 2.0, 0.0)], vec![Point3::new(3.0, 0.0, 0.0), Point3::new(3.0, 2.0, 1.0)]],
            weights: vec![vec![1.0; 2]; 2],
            periods: [None, None],
        };
        let (p, su, sv) = s.d1(0.5, 0.5);
        assert!((p.x - 1.5).abs() < 1e-15 && (p.y - 1.0).abs() < 1e-15 && (p.z - 0.25).abs() < 1e-15);
        assert!((su.x - 3.0).abs() < 1e-15 && (su.z - 0.5).abs() < 1e-15);
        // S_v = (1 − u)(P01 − P00) + u(P11 − P10) = (0, 2, 0.5).
        assert!((sv.y - 2.0).abs() < 1e-15 && (sv.z - 0.5).abs() < 1e-15, "{sv:?}");
    }
}
