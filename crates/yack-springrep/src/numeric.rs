// yack: Quadrature, linear solves and root finding shared by the geometry and measurement modules.
// Adapted from springrep@a03e8a1 src/numeric.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: all but the Gauss-Legendre rules moved to yack-numeric (M3 W1', D26), re-exported here.
//! Quadrature, linear solves and root finding shared by the geometry and measurement modules.

use std::sync::OnceLock;

pub(crate) use yack_numeric::{KRONROD_15, KRONROD_7, solve2, solve_dense};

/// Gauss–Legendre nodes and weights of order `N` on [-1, 1].
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

pub(crate) fn gauss(n: usize) -> &'static [(f64, f64)] {
    static RULES: [OnceLock<Vec<(f64, f64)>>; 4] = [
        OnceLock::new(),
        OnceLock::new(),
        OnceLock::new(),
        OnceLock::new(),
    ];
    let slot = match n {
        8 => 0,
        12 => 1,
        16 => 2,
        _ => 3,
    };
    let order = [8, 12, 16, 24][slot];
    RULES[slot].get_or_init(|| legendre_rule(order))
}

/// ∫ f over [a, b] with an `n`-point Gauss rule.
pub(crate) fn gauss_fixed(f: &mut impl FnMut(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let (m, h) = ((a + b) * 0.5, (b - a) * 0.5);
    gauss(n).iter().map(|&(x, w)| w * f(m + h * x)).sum::<f64>() * h
}

/// ∫ f over [a, b], halving until two levels agree to `abs_tol` (spread over the interval).
pub(crate) fn integrate(f: &mut impl FnMut(f64) -> f64, a: f64, b: f64, abs_tol: f64) -> f64 {
    let whole = gauss_fixed(f, a, b, 16);
    adapt(f, a, b, whole, abs_tol, 0)
}

fn adapt(f: &mut impl FnMut(f64) -> f64, a: f64, b: f64, whole: f64, tol: f64, depth: u32) -> f64 {
    let m = (a + b) * 0.5;
    let left = gauss_fixed(f, a, m, 16);
    let right = gauss_fixed(f, m, b, 16);
    if (left + right - whole).abs() <= tol || depth >= 24 {
        return left + right;
    }
    adapt(f, a, m, left, tol * 0.5, depth + 1) + adapt(f, m, b, right, tol * 0.5, depth + 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gauss_rules_integrate_polynomials_exactly() {
        for n in [8, 12, 16, 24] {
            let s: f64 = gauss(n).iter().map(|&(_, w)| w).sum();
            assert!((s - 2.0).abs() < 1e-14, "order {n}");
            let mut f = |x: f64| x.powi(2 * n as i32 - 2);
            let v = gauss_fixed(&mut f, 0.0, 1.0, n);
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
}
