// yack: Section curves as clamped rational B-splines in homogeneous coordinates, and the operations
// Adapted from springrep@a03e8a1 src/loft/rail.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/loft/rail.rs to yack-build (M3 W7); paths and visibility only;
// M3 hardening (w-h3): knot insertion, degree elevation, blossoms, Cox–de Boor and the fits walk
// their arrays by iterators, windows and checked access instead of indexing, in the same order of
// arithmetic; a knot that cannot be inserted stops the insertion instead of panicking.
//! Section curves as clamped rational B-splines in homogeneous coordinates, and the operations
//! that make a column of them share one degree and knot vector, so their control points can be
//! skinned into a surface.

use std::f64::consts::FRAC_PI_2;

use crate::{
    Error, Result,
    geom::Curve,
    math::{Frame, Point3, Vec3},
    numeric::solve_dense,
};

/// `(w·x, w·y, w·z, w)`.
pub type Hom = [f64; 4];

pub fn hom(p: Point3, w: f64) -> Hom {
    [p.x * w, p.y * w, p.z * w, w]
}

/// The point a rail without control points starts and ends at: nowhere.
const NOWHERE: Point3 = Vec3::new(f64::NAN, f64::NAN, f64::NAN);

pub fn point(h: Hom) -> Point3 {
    Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3])
}

fn mix(a: Hom, b: Hom, t: f64) -> Hom {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
        a[3] + (b[3] - a[3]) * t,
    ]
}

/// A clamped B-spline on `[0, 1]` once [`Rail::normalized`].
#[derive(Clone, Debug, PartialEq)]
pub struct Rail {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub pw: Vec<Hom>,
}

impl Rail {
    /// The curve from `t0` to `t1` (either order) of a line, conic or polynomial spline.
    pub(crate) fn of_curve(curve: &Curve, t0: f64, t1: f64) -> Result<Rail> {
        let r = match curve {
            Curve::Line(_) => Rail {
                degree: 1,
                knots: vec![0.0, 0.0, 1.0, 1.0],
                pw: vec![hom(curve.eval(t0), 1.0), hom(curve.eval(t1), 1.0)],
            },
            Curve::Circle(c) => conic(&c.frame, c.radius, c.radius, t0, t1),
            Curve::Ellipse(e) => conic(&e.frame, e.major, e.minor, t0, t1),
            Curve::BSpline(b) => {
                let whole = Rail {
                    degree: b.degree(),
                    knots: b.knots().to_vec(),
                    pw: b.control_points().iter().map(|&p| hom(p, 1.0)).collect(),
                };
                let sub = whole.sub(t0.min(t1), t0.max(t1))?;
                if t0 > t1 { sub.reversed() } else { sub }
            }
            other => {
                return Err(Error::Unsupported(format!(
                    "a loft section made of a {} curve",
                    other.kind()
                )));
            }
        };
        Ok(r.normalized())
    }

    /// The parameters it runs over: NaN for a rail whose knots are too few for its control points.
    pub fn domain(&self) -> (f64, f64) {
        let knot = |i: usize| self.knots.get(i).copied().unwrap_or(f64::NAN);
        (knot(self.degree), knot(self.pw.len()))
    }

    /// Its first control point, NaN for a rail with none.
    pub fn start(&self) -> Point3 {
        self.pw.first().map_or(NOWHERE, |&h| point(h))
    }

    /// Its last control point, NaN for a rail with none.
    pub fn end(&self) -> Point3 {
        self.pw.last().map_or(NOWHERE, |&h| point(h))
    }

    fn multiplicity(&self, t: f64) -> usize {
        self.knots.iter().filter(|&&k| k == t).count()
    }

    /// Inserts `t` once (Boehm's algorithm); false, the rail unchanged, where its knots and
    /// control points do not make a B-spline.
    fn insert(&mut self, t: f64) -> bool {
        let (p, n) = (self.degree, self.pw.len());
        let u = &self.knots;
        // The span `t` falls in: control points k − p ..= k shape it.
        let mut k = p;
        while k + 1 < n && u.get(k + 1).is_some_and(|&x| x <= t) {
            k += 1;
        }
        let Some(first_moved) = k.checked_sub(p) else { return false };
        let (Some(kept_before), Some(blended), Some(kept_after), Some(lows), Some(highs)) = (
            self.pw.get(..=first_moved),
            self.pw.get(first_moved..=k),
            self.pw.get(k..),
            u.get(first_moved + 1..=k),
            u.get(k + 1..=k + p),
        ) else {
            return false;
        };
        // Points up to the span's first stay, from its last on they move up one place, and each
        // between is its two neighbours mixed by where `t` falls in its knots.
        let mut q = Vec::with_capacity(n + 1);
        q.extend_from_slice(kept_before);
        let neighbours = blended.iter().zip(blended.iter().skip(1));
        for ((&before, &after), (&lo, &hi)) in neighbours.zip(lows.iter().zip(highs)) {
            let den = hi - lo;
            let a = if den > 0.0 { (t - lo) / den } else { 0.0 };
            q.push(mix(before, after, a));
        }
        q.extend_from_slice(kept_after);
        self.pw = q;
        self.knots.insert(k + 1, t);
        true
    }

    fn insert_to(&mut self, t: f64, m: usize) {
        while self.multiplicity(t) < m {
            if !self.insert(t) {
                break;
            }
        }
    }

    /// The clamped piece over `[t0, t1]`, `t0 < t1` inside the domain.
    fn sub(&self, t0: f64, t1: f64) -> Result<Rail> {
        let p = self.degree;
        let mut r = self.clone();
        r.insert_to(t0, p);
        r.insert_to(t1, p);
        let first = r.knots.iter().position(|&k| k == t0).unwrap_or(0);
        let j0 = first + r.multiplicity(t0) - 1 - p;
        let last_start = r.knots.iter().position(|&k| k == t1).unwrap_or(0);
        let j1 = last_start - 1;
        let mut knots = vec![t0; p + 1];
        knots.extend(r.knots.iter().filter(|&&k| k > t0 && k < t1));
        knots.extend(std::iter::repeat_n(t1, p + 1));
        let malformed = || Error::Numerical("B-spline piece has a malformed knot vector".into());
        let pw = r.pw.get(j0..=j1).ok_or_else(malformed)?.to_vec();
        if knots.len() != pw.len() + p + 1 {
            return Err(malformed());
        }
        Ok(Rail {
            degree: p,
            knots,
            pw,
        })
    }

    pub fn reversed(&self) -> Rail {
        // With no knots there are none to map, and the ends are never read.
        let a = self.knots.first().copied().unwrap_or_default();
        let b = self.knots.last().copied().unwrap_or_default();
        Rail {
            degree: self.degree,
            knots: self.knots.iter().rev().map(|&k| a + b - k).collect(),
            pw: self.pw.iter().rev().copied().collect(),
        }
    }

    /// The same curve with its domain mapped onto `[0, 1]`.
    pub(crate) fn normalized(&self) -> Rail {
        let (a, b) = self.domain();
        let mut knots: Vec<f64> = self.knots.iter().map(|&k| (k - a) / (b - a)).collect();
        for k in &mut knots {
            *k = k.clamp(0.0, 1.0);
        }
        Rail {
            knots,
            ..self.clone()
        }
    }

    /// The distinct knots strictly inside the domain.
    pub fn breaks(&self) -> Vec<f64> {
        let (a, b) = self.domain();
        let mut out: Vec<f64> = Vec::new();
        for &k in &self.knots {
            if k > a && k < b && out.last().is_none_or(|&l| k > l) {
                out.push(k);
            }
        }
        out
    }

    /// The same curve at degree `q ≥ degree`, split into Bézier pieces to raise each.
    pub fn elevated(&self, q: usize) -> Rail {
        let p = self.degree;
        if q <= p {
            return self.clone();
        }
        let mut r = self.clone();
        let breaks = r.breaks();
        for &b in &breaks {
            r.insert_to(b, p);
        }
        let segs = (r.pw.len() - 1) / p;
        let mut pw: Vec<Hom> = Vec::new();
        // Each Bézier piece's p + 1 points, sharing its ends with its neighbours.
        let pieces = (0..segs).filter_map(|s| r.pw.get(s * p..=s * p + p));
        for (s, piece) in pieces.enumerate() {
            let mut bez: Vec<Hom> = piece.to_vec();
            for d in p..q {
                let mut up = Vec::with_capacity(d + 2);
                up.extend(bez.first());
                for (i, (&before, &here)) in bez.iter().zip(bez.iter().skip(1)).enumerate() {
                    let a = (i + 1) as f64 / (d + 1) as f64;
                    up.push(mix(here, before, a));
                }
                up.extend(bez.last());
                bez = up;
            }
            // A piece after the first starts where the one before ended.
            pw.extend(bez.into_iter().skip(usize::from(s > 0)));
        }
        let (a, b) = r.domain();
        let mut knots = vec![a; q + 1];
        for &k in &breaks {
            knots.extend(std::iter::repeat_n(k, q));
        }
        knots.extend(std::iter::repeat_n(b, q + 1));
        Rail {
            degree: q,
            knots,
            pw,
        }
    }

    /// The same curve with every one of `breaks` a knot of full multiplicity.
    pub fn refined(&self, breaks: &[f64]) -> Rail {
        let mut r = self.clone();
        for &b in breaks {
            r.insert_to(b, r.degree);
        }
        r
    }
}

/// The blossom of the Bézier `pts` at `ts`, one parameter per de Casteljau level.
fn blossom(pts: &[Hom], ts: &[f64]) -> Hom {
    let mut q = pts.to_vec();
    // Each de Casteljau level mixes every point with the next, one fewer each time.
    for &t in ts {
        q = q.iter().zip(q.iter().skip(1)).map(|(&a, &b)| mix(a, b, t)).collect();
    }
    q.first().copied().unwrap_or_default()
}

/// The clamped B-spline `(knots, pw)` of `degree` carried on past both ends of its domain by
/// `delta`, each end span's polynomial continued; unchanged inside the domain.
pub(crate) fn extended(
    degree: usize,
    knots: &[f64],
    pw: &[Hom],
    delta: f64,
) -> (Vec<f64>, Vec<Hom>) {
    let p = degree;
    let mut r = Rail {
        degree,
        knots: knots.to_vec(),
        pw: pw.to_vec(),
    };
    let (a, b) = r.domain();
    let first = r.knots.iter().copied().find(|&k| k > a).unwrap_or(b);
    if first < b {
        r.insert_to(first, p);
    }
    let last = r.knots.iter().rev().copied().find(|&k| k < b).unwrap_or(a);
    if last > a {
        r.insert_to(last, p);
    }
    let lo = -delta / (first - a);
    let n = r.pw.len();
    // The first and last spans' control points: a rail with fewer than p + 1 has no spans.
    let (Some(first_span), Some(last_span)) =
        (r.pw.get(..=p), n.checked_sub(p + 1).and_then(|from| r.pw.get(from..)))
    else {
        return (r.knots, r.pw);
    };
    let head: Vec<Hom> = (0..=p)
        .map(|i| {
            let ts: Vec<f64> = (0..p).map(|k| if k < p - i { lo } else { 1.0 }).collect();
            blossom(first_span, &ts)
        })
        .collect();
    let hi = (b + delta - last) / (b - last);
    let tail: Vec<Hom> = (0..=p)
        .map(|i| {
            let ts: Vec<f64> = (0..p).map(|k| if k < p - i { 0.0 } else { hi }).collect();
            blossom(last_span, &ts)
        })
        .collect();
    if n == p + 1 {
        // One span: both ends continue the same polynomial.
        let up = (b + delta - a) / (b - a);
        let ts =
            |i: usize| -> Vec<f64> { (0..p).map(|k| if k < p - i { lo } else { up }).collect() };
        let pw = (0..=p).map(|i| blossom(&r.pw, &ts(i))).collect();
        let mut knots = vec![a - delta; p + 1];
        knots.extend(std::iter::repeat_n(b + delta, p + 1));
        return (knots, pw);
    }
    for (slot, h) in r.pw.iter_mut().zip(head) {
        *slot = h;
    }
    for (slot, h) in r.pw.iter_mut().skip(n - 1 - p).zip(tail) {
        *slot = h;
    }
    let m = r.knots.len();
    for k in r.knots.iter_mut().take(p + 1) {
        *k = a - delta;
    }
    for k in r.knots.iter_mut().skip(m.saturating_sub(p + 1)) {
        *k = b + delta;
    }
    (r.knots, r.pw)
}

/// An arc of `frame.origin + a·cos t·x + b·sin t·y` from `t0` to `t1` as rational quadratics, one
/// per quarter turn or less, each with end weights 1.
fn conic(frame: &Frame, a: f64, b: f64, t0: f64, t1: f64) -> Rail {
    let n = ((t1 - t0).abs() / FRAC_PI_2 - 1e-9).ceil().max(1.0) as usize;
    let step = (t1 - t0) / n as f64;
    let at =
        |t: f64, s: f64| frame.origin + frame.x * (a * t.cos() * s) + frame.y * (b * t.sin() * s);
    let w = (step / 2.0).cos();
    let mut pw = vec![hom(at(t0, 1.0), 1.0)];
    let mut knots = vec![0.0; 3];
    for k in 0..n {
        let ta = t0 + step * k as f64;
        pw.push(hom(at(ta + step / 2.0, 1.0 / w), w));
        pw.push(hom(at(ta + step, 1.0), 1.0));
        let e = (k + 1) as f64;
        knots.extend(if k + 1 == n { vec![e; 3] } else { vec![e; 2] });
    }
    Rail {
        degree: 2,
        knots,
        pw,
    }
}

/// Every rail of one column at one degree on one knot vector.
pub fn make_compatible(rails: &mut [Rail]) {
    let alike = |x: &Rail, y: &Rail| {
        x.degree == y.degree
            && x.knots.len() == y.knots.len()
            && x.knots.iter().zip(&y.knots).all(|(a, b)| (a - b).abs() <= 1e-12)
    };
    let same = rails.iter().zip(rails.iter().skip(1)).all(|(x, y)| alike(x, y));
    if same {
        let Some(first) = rails.first() else { return };
        let knots = first.knots.clone();
        for r in rails.iter_mut() {
            r.knots = knots.clone();
        }
        return;
    }
    let q = rails.iter().map(|r| r.degree).max().unwrap_or(1);
    let mut all: Vec<f64> = rails.iter().flat_map(|r| r.breaks()).collect();
    all.sort_by(f64::total_cmp);
    let mut breaks: Vec<f64> = Vec::new();
    for k in all {
        if breaks.last().is_none_or(|&l| k - l > 1e-12) {
            breaks.push(k);
        }
    }
    for r in rails.iter_mut() {
        let mut e = r.elevated(q);
        for k in &mut e.knots {
            if let Some(&b) = breaks.iter().find(|&&b| (b - *k).abs() <= 1e-12) {
                *k = b;
            }
        }
        *r = e.refined(&breaks);
    }
}

/// A linear map from data at `n` sections to the control points of a curve across them:
/// control `k` is `Σ_s coef[k·n + s]·Q_s`, the B-spline of `degree` on `knots`.
#[derive(Clone, Debug)]
pub struct Across {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub coef: Vec<f64>,
    pub n: usize,
}

impl Across {
    pub(crate) fn controls(&self) -> usize {
        self.knots.len() - self.degree - 1
    }

    pub fn apply(&self, data: &[Hom]) -> Vec<Hom> {
        (0..self.controls())
            .map(|k| {
                let mut c = [0.0; 4];
                for (s, d) in data.iter().enumerate() {
                    // Every coefficient the data reaches is there, `controls() · n` of them.
                    let Some(&f) = self.coef.get(k * self.n + s) else { continue };
                    for (ci, di) in c.iter_mut().zip(d) {
                        *ci += f * di;
                    }
                }
                c
            })
            .collect()
    }

    pub fn eval(&self, ctrl: &[Hom], v: f64) -> Hom {
        let vals = basis(self.degree, &self.knots, v);
        let mut out = [0.0; 4];
        for (c, b) in ctrl.iter().zip(vals) {
            for (oi, ci) in out.iter_mut().zip(c) {
                *oi += b * ci;
            }
        }
        out
    }
}

/// Every basis function of `degree` on `knots` at `t` (Cox–de Boor), right-continuous except at
/// the domain's end.
pub fn basis(degree: usize, knots: &[f64], t: f64) -> Vec<f64> {
    let Some(n) = knots.len().checked_sub(degree + 1) else { return Vec::new() };
    let Some(&end) = knots.get(n) else { return Vec::new() };
    // Degree 0: one function a span, 1 on it.
    let mut nb: Vec<f64> = knots
        .iter()
        .zip(knots.iter().skip(1))
        .map(|(&k0, &k1)| {
            let inside = k0 <= t && t < k1;
            let last = t >= end && k0 < end && k1 >= end;
            if inside || last { 1.0 } else { 0.0 }
        })
        .collect();
    // Each degree's functions from the last degree's, one fewer each time: function i spans
    // knots i ..= i + d + 1, and mixes functions i and i + 1 of the degree below.
    for d in 1..=degree {
        let below = nb.iter().zip(nb.iter().skip(1));
        nb = knots
            .windows(d + 2)
            .zip(below)
            .map(|(span, (&left, &right))| {
                let (Some(&k0), Some(&k1), Some(&kd), Some(&kd1)) =
                    (span.first(), span.get(1), span.get(d), span.last())
                else {
                    return 0.0;
                };
                let l = kd - k0;
                let r = kd1 - k1;
                let a = if l > 0.0 { (t - k0) / l * left } else { 0.0 };
                let b = if r > 0.0 { (kd1 - t) / r * right } else { 0.0 };
                a + b
            })
            .collect();
    }
    nb.truncate(n);
    nb
}

/// Adds `x` to `v[i]`; every index the fits use is in range by their matrices' sizes.
fn add_at(v: &mut [f64], i: usize, x: f64) {
    if let Some(e) = v.get_mut(i) {
        *e += x;
    }
}

/// Sets `v[i]` to `x`, under [`add_at`]'s promise.
fn set_at(v: &mut [f64], i: usize, x: f64) {
    if let Some(e) = v.get_mut(i) {
        *e = x;
    }
}

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
}

/// The Bézier of `degree` through the first and last data points that fits the others at
/// `params` by least squares.
pub(crate) fn bezier_fit(params: &[f64], degree: usize) -> Result<Across> {
    let n = params.len();
    let p = degree;
    let bern =
        |k: usize, v: f64| binomial(p, k) * v.powi(k as i32) * (1.0 - v).powi((p - k) as i32);
    let mut coef = vec![0.0; (p + 1) * n];
    set_at(&mut coef, 0, 1.0);
    set_at(&mut coef, p * n + n - 1, 1.0);
    let m = p - 1;
    if m > 0 {
        let rows = n - 2;
        // The parameters between the first and last, which the fit is free at.
        let inner = params.get(1..n - 1).unwrap_or_default();
        let mut ata = vec![0.0; m * m];
        let mut atb = vec![0.0; m * rows];
        for (r, &v) in inner.iter().enumerate() {
            for a in 0..m {
                let ba = bern(a + 1, v);
                set_at(&mut atb, a * rows + r, ba);
                for b in 0..m {
                    add_at(&mut ata, a * m + b, ba * bern(b + 1, v));
                }
            }
        }
        let x = solve_dense(ata, atb, m, rows)?;
        for a in 0..m {
            let k = a + 1;
            for (r, &v) in inner.iter().enumerate() {
                let g = x.get(a * rows + r).copied().unwrap_or_default();
                add_at(&mut coef, k * n + r + 1, g);
                add_at(&mut coef, k * n, -(g * bern(0, v)));
                add_at(&mut coef, k * n + n - 1, -(g * bern(p, v)));
            }
        }
    }
    let mut knots = vec![0.0; p + 1];
    knots.extend(std::iter::repeat_n(1.0, p + 1));
    Ok(Across {
        degree: p,
        knots,
        coef,
        n,
    })
}

/// The cubic (quadratic, linear for fewer sections) through every data point at `params`, with
/// not-a-knot ends.
pub(crate) fn interpolation(params: &[f64]) -> Result<Across> {
    let (Some(&first), Some(&last)) = (params.first(), params.last()) else {
        return Err(Error::InvalidInput("an interpolation through no points".into()));
    };
    let n = params.len();
    let p = (n - 1).min(3);
    let mut knots = vec![first; p + 1];
    if p == 3 {
        knots.extend_from_slice(params.get(2..n - 2).unwrap_or_default());
    }
    knots.extend(std::iter::repeat_n(last, p + 1));
    let mut a = vec![0.0; n * n];
    for (row, &t) in params.iter().enumerate() {
        for (j, b) in basis(p, &knots, t).into_iter().enumerate() {
            set_at(&mut a, row * n + j, b);
        }
    }
    let mut id = vec![0.0; n * n];
    for i in 0..n {
        set_at(&mut id, i * n + i, 1.0);
    }
    let coef = solve_dense(a, id, n, n)?;
    Ok(Across {
        degree: p,
        knots,
        coef,
        n,
    })
}
