// yack: the roots of a function of one parameter: sampled, between stationary points, of a trig sum.
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: split out of csi.rs (M3 hardening): named limits, iterators and checked access instead of
// indexing; the same arithmetic, and the same calls of each (possibly stateful) function, in the same
// order.
//! The roots of a function of one parameter, for [`super`]'s closed forms and searches: a sum
//! `A + B cos t + C sin t`, a sampled function, a polynomial split at its stationary points, and a
//! function on stretches that each hold one root at most.

use std::f64::consts::TAU;

use super::Hits;
use crate::{numeric, tol};

/// Two angles within this share of `1 + |angle|` of each other are one, moved by rounding.
const ANGLE_ROUNDING: f64 = 1e-14; // tol: relative angle rounding

/// Two parameters within this share of `1 + |t|` of each other are one root found twice.
const PARAM_ROUNDING: f64 = 1e-12; // tol: relative parameter rounding

/// The golden-section search stops when its bracket is this share of `1 + |a|`.
const GOLDEN_BRACKET: f64 = 1e-15; // tol: relative parameter rounding

/// The most golden-section steps.
const GOLDEN_STEPS: usize = 200;

/// The step for a slope by central differences, as a share of the stretch searched.
const SLOPE_STEP: f64 = 1e-3; // tol: a difference step, not a tolerance

/// Below this a coefficient or pivot is zero.
pub(super) const UNDERFLOW: f64 = 1e-300; // tol: an underflow guard

/// The value at an extremum below which the curve touches the surface there rather than crossing
/// it twice: a curve that dips into the surface by no more than the tolerance lies on it there, and
/// its sign changes sit about √(dip / curvature) away, far more than the tolerance along the curve.
const TOUCH: f64 = tol::LINEAR;

/// `n` evenly spaced parameters from `a` toward `b`, `b` itself left out.
pub(super) fn evenly(a: f64, b: f64, n: usize) -> impl Iterator<Item = f64> {
    (0..n).map(move |i| a + (b - a) * i as f64 / n as f64)
}

/// `n + 1` evenly spaced parameters from `a` to `b`, the last computed as the others are.
pub(super) fn evenly_through(a: f64, b: f64, n: usize) -> impl Iterator<Item = f64> {
    (0..=n).map(move |i| a + (b - a) * i as f64 / n as f64)
}

/// The spans between consecutive `cuts`, as pairs of ends.
pub(super) fn spans(cuts: &[f64]) -> impl Iterator<Item = (f64, f64)> + '_ {
    cuts.windows(2).filter_map(|w| match *w {
        [a, b] => Some((a, b)),
        _ => None,
    })
}

/// The parameters `inner`, which split `(lo, hi)` into spans, with the range's ends about them.
pub(super) fn with_ends(inner: impl IntoIterator<Item = f64>, (lo, hi): (f64, f64)) -> Vec<f64> {
    let mut cuts = vec![lo];
    cuts.extend(inner);
    cuts.push(hi);
    cuts
}

/// Moves an angle by whole turns into `range` when it fits there.
pub(super) fn angle_in(t: f64, range: (f64, f64)) -> Option<f64> {
    let k = ((range.0 - t) / TAU).ceil();
    let a = t + k * TAU;
    let eps = ANGLE_ROUNDING * (1.0 + a.abs());
    if a <= range.1 + eps {
        Some(a.min(range.1))
    } else if (t + (k - 1.0) * TAU - range.0).abs() <= eps {
        Some(range.0)
    } else {
        None
    }
}

/// Where `f(t) = A + B cos t + C sin t` vanishes over `range`, for a curve and a surface that meet
/// where it does.
pub(super) fn trig_roots(a: f64, b: f64, c: f64, range: (f64, f64)) -> Hits {
    let r = b.hypot(c);
    if r <= tol::LINEAR {
        return Hits { ts: Vec::new(), on_surface: a.abs() <= tol::LINEAR };
    }
    let phi = c.atan2(b);
    let mut ts = Vec::new();
    if a.abs() >= r - tol::LINEAR {
        // At most one touching point, where the curve comes nearest.
        if a.abs() - r <= tol::LINEAR {
            ts.push(if a < 0.0 { phi } else { phi + TAU / 2.0 });
        }
    } else {
        let w = (-a / r).acos();
        ts.push(phi + w);
        ts.push(phi - w);
    }
    Hits { ts: ts.into_iter().filter_map(|t| angle_in(t, range)).collect(), on_surface: false }
}

/// Golden-section minimisation of `f` on [a, b].
pub(super) fn minimise(f: &mut impl FnMut(f64) -> f64, mut a: f64, mut b: f64) -> f64 {
    let g = 0.5 * (5f64.sqrt() - 1.0);
    let mut c = b - g * (b - a);
    let mut d = a + g * (b - a);
    let (mut fc, mut fd) = (f(c), f(d));
    for _ in 0..GOLDEN_STEPS {
        if (b - a).abs() <= GOLDEN_BRACKET * (1.0 + a.abs()) {
            break;
        }
        if fc < fd {
            b = d;
            d = c;
            fd = fc;
            c = b - g * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + g * (b - a);
            fd = f(d);
        }
    }
    0.5 * (a + b)
}

/// Where `f`'s slope changes sign in [a, b], if it does: an extremum pinned by the sign of the slope,
/// to rounding.
fn stationary(f: &mut impl FnMut(f64) -> f64, a: f64, b: f64) -> Option<f64> {
    let h = SLOPE_STEP * (b - a);
    let mut slope = |t: f64| f(t + h) - f(t - h);
    let (sa, sb) = (slope(a + h), slope(b - h));
    ((sa < 0.0) != (sb < 0.0) && sa != 0.0 && sb != 0.0)
        .then(|| numeric::bisect(&mut slope, a + h, b - h))
}

/// A root bisected between samples, moved onto the extremum it lies beside when the curve only
/// touches the surface there, held to [lo, hi]. A touch leaves the surface on one side of it both
/// ways; a curve that runs along the surface has slope changes from rounding alone.
fn settle(f: &mut impl FnMut(f64) -> f64, t: f64, w: f64, (lo, hi): (f64, f64)) -> f64 {
    let (fa, fb) = (f(t - w), f(t + w));
    if (fa < 0.0) != (fb < 0.0) || fa.abs().min(fb.abs()) <= tol::LINEAR {
        return t;
    }
    match stationary(f, t - w, t + w) {
        Some(s) if f(s).abs() <= TOUCH => s.clamp(lo, hi),
        _ => t,
    }
}

/// Sign changes and near-zero minima of `f` on the spans between `cuts`, `n` samples per span;
/// also whether `f` stays within the tolerance of zero at every sample.
pub(super) fn sampled_roots(
    f: &mut impl FnMut(f64) -> f64,
    cuts: &[f64],
    n: usize,
) -> (Vec<f64>, bool) {
    let (Some(&first), Some(&last)) = (cuts.first(), cuts.last()) else {
        return (Vec::new(), false);
    };
    let mut samples: Vec<(f64, f64)> = Vec::new();
    for (a, b) in spans(cuts) {
        samples.extend(evenly(a, b, n).map(|t| (t, f(t))));
    }
    samples.push((last, f(last)));
    if samples.iter().all(|&(_, v)| v.abs() <= tol::LINEAR) {
        return (Vec::new(), true);
    }
    let mut ts = Vec::new();
    for (k, &(t, v)) in samples.iter().enumerate() {
        if v == 0.0 {
            ts.push(t);
            continue;
        }
        let next = samples.get(k + 1).copied();
        if let Some((t2, v2)) = next
            && v2 != 0.0
            && (v < 0.0) != (v2 < 0.0)
        {
            let r = numeric::bisect(f, t, t2);
            ts.push(settle(f, r, t2 - t, (first, last)));
        }
        let before = k.checked_sub(1).and_then(|j| samples.get(j)).copied();
        if let (Some((tp, vp)), Some((tn, vn))) = (before, next) {
            let same = (vp < 0.0) == (v < 0.0) && (vn < 0.0) == (v < 0.0);
            if same && v.abs() <= vp.abs() && v.abs() <= vn.abs() {
                // The extremum between the samples, not where |f| first reaches zero: a curve that
                // touches the surface to within the tolerance dips through it by rounding, and its
                // zeros then lie √(dip / curvature) either side of the touching point.
                let s = v.signum();
                let tm = minimise(&mut |x: f64| s * f(x), tp, tn);
                let fm = f(tm);
                if fm.abs() <= tol::LINEAR {
                    ts.push(tm);
                } else if (fm < 0.0) != (v < 0.0) {
                    ts.push(numeric::bisect(f, tp, tm));
                    ts.push(numeric::bisect(f, tm, tn));
                }
            }
        }
    }
    // A touch is reached from the sign changes either side of it and from the extremum between.
    (in_order(ts), false)
}

/// The value at `t` of the polynomial `c[0] + c[1]·t + …`.
fn poly(c: &[f64], t: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &k| acc * t + k)
}

/// The coefficients of the polynomial's derivative.
fn derivative(c: &[f64]) -> Vec<f64> {
    c.iter().enumerate().skip(1).map(|(i, &k)| k * i as f64).collect()
}

/// The polynomial's stationary points inside (lo, hi), in order: each is the one root of its
/// derivative on a stretch where the derivative is monotone, found from the next derivative's
/// stationary points in turn, so a double root is found as a stationary point, not missed between
/// samples.
pub(super) fn stationary_points(c: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    let d = derivative(c);
    if d.len() < 2 {
        return Vec::new();
    }
    let knots = with_ends(stationary_points(&d, lo, hi), (lo, hi));
    let mut out = Vec::new();
    for (a, b) in spans(&knots) {
        let (da, db) = (poly(&d, a), poly(&d, b));
        if da == 0.0 && a > lo {
            out.push(a);
        } else if da != 0.0 && db != 0.0 && (da < 0.0) != (db < 0.0) {
            out.push(numeric::bisect(&mut |t| poly(&d, t), a, b));
        }
    }
    out
}

/// Where `f`, a distance in mm, is zero over [lo, hi], given `knots` that split the range into
/// stretches on which it has at most one root. A knot within the tolerance of the surface is a
/// meeting there, a crossing or a touch alike; a stretch whose ends lie beyond it on opposite
/// sides has its crossing bisected; a stretch with one end on the surface adds nothing.
pub(super) fn roots_between(f: &mut impl FnMut(f64) -> f64, knots: &[f64]) -> Vec<f64> {
    let side = |v: f64| -> i8 {
        if v > tol::LINEAR {
            1
        } else if v < -tol::LINEAR {
            -1
        } else {
            0
        }
    };
    let signs: Vec<i8> = knots.iter().map(|&t| side(f(t))).collect();
    let mut ts = Vec::new();
    for (i, (&t, &sign)) in knots.iter().zip(&signs).enumerate() {
        if sign == 0 {
            ts.push(t);
        }
        if let (Some(&next), Some(&next_sign)) = (knots.get(i + 1), signs.get(i + 1))
            && sign * next_sign < 0
        {
            ts.push(numeric::bisect(f, t, next));
        }
    }
    ts
}

/// The coefficients `c[0] + c[1]·s + …` of the polynomial of degree `n` through `n + 1` values at
/// `s = 0, 1/n, …, 1`, by Gauss–Jordan elimination with partial pivoting; `None` where the fit is
/// singular.
pub(super) fn fitted(values: &[f64]) -> Option<Vec<f64>> {
    let n = values.len();
    let mut m: Vec<Vec<f64>> = values
        .iter()
        .enumerate()
        .map(|(j, &value)| {
            let s = j as f64 / (n - 1).max(1) as f64;
            let mut row: Vec<f64> = (0..n).map(|k| s.powi(k as i32)).collect();
            row.push(value);
            row
        })
        .collect();
    for col in 0..n {
        let size = |row: &[f64]| row.get(col).map_or(0.0, |x| x.abs());
        let (below, _) = m
            .get(col..)?
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| size(a).total_cmp(&size(b)))?;
        let pivot = col + below;
        let by = m.get(pivot)?.clone();
        let &lead = by.get(col)?;
        if lead.abs() < UNDERFLOW {
            return None;
        }
        m.swap(col, pivot);
        for (row, values) in m.iter_mut().enumerate() {
            if row != col {
                let f = *values.get(col)? / lead;
                for (x, p) in values.iter_mut().zip(&by).skip(col) {
                    *x -= f * p;
                }
            }
        }
    }
    m.iter().enumerate().map(|(i, row)| Some(*row.get(n)? / *row.get(i)?)).collect()
}

/// Parameters in order, two within rounding of each other as one: a crossing at the join of two
/// pieces is found from both.
pub(super) fn in_order(mut ts: Vec<f64>) -> Vec<f64> {
    ts.sort_by(f64::total_cmp);
    ts.dedup_by(|a, b| (*a - *b).abs() <= PARAM_ROUNDING * (1.0 + a.abs().max(b.abs())));
    ts
}
