// yack: rational B-spline surfaces of any degree: evaluation, iso lines, boxes and patches.
// Adapted from springrep@a03e8a1 src/geom/nurbs.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/nurbs.rs to yack-geom (M3 W3); split out of
// nurbs.rs into its own module (M3 hardening). M3 hardening: the net is read by control point
// `(i, j)` through one checked accessor, span slots are `Option`s instead of a `usize::MAX`
// sentinel, the series and sums run over iterators and chunks, and the rational quotient
// destructures its derivatives; no indexing, the same arithmetic in the same order.
//! Rational B-spline surfaces of any degree: evaluation, iso lines, boxes and patches.

use std::{cell::RefCell, sync::OnceLock};

use super::{
    IsoLine, M, MAX_POLY_DEGREE, NurbsCurve,
    basis::{Basis, check_weights},
    bezier::{Bezier, Patch, bernstein_from_values, insert_knot},
    seeds::Seeds,
};
use crate::{
    Result,
    error::invalid,
    math::{Aabb, Point3, Transform, Vec2, Vec3},
    tol,
};

/// A rational (or polynomial) B-spline surface. Control points are stored row by row: index
/// `i·nv + j` is the `i`-th along `u` and the `j`-th along `v`.
#[derive(Clone, Debug)]
pub struct NurbsSurface {
    pub(super) bu: Basis,
    pub(super) bv: Basis,
    pub(super) ctrl: Vec<Point3>,
    pub(super) weights: Option<Vec<f64>>,
    pub(super) u_period: Option<f64>,
    pub(super) v_period: Option<f64>,
    /// Sample points of the surface that seed global projection, built on first use.
    seeds: OnceLock<Seeds>,
    /// Small patches whose boxes together hold the surface, built on first use.
    patches: OnceLock<Vec<Patch>>,
    /// Each patch as a Bézier patch, built on first use.
    beziers: OnceLock<Vec<Bezier>>,
    /// The surface over each pair of spans as a polynomial, built on first use.
    polys: OnceLock<SpanPolys>,
}

/// The surface over each pair of nonempty spans as a polynomial in the spans' local parameters,
/// in homogeneous coordinates: evaluating it is a Horner pass with no basis recurrence.
#[derive(Clone, Debug)]
struct SpanPolys {
    /// Each span's slot among the nonempty ones, by span number; `None` for an empty span.
    u_slot: Vec<Option<usize>>,
    v_slot: Vec<Option<usize>>,
    /// Each nonempty span's functions as series about its start and about its end.
    u_basis: Vec<[Vec<Vec<f64>>; 2]>,
    v_basis: Vec<[Vec<Vec<f64>>; 2]>,
    /// Row by row along u, `(degree_u + 1)·(degree_v + 1)` coefficients each, by slot pair and
    /// the end each series is about.
    cells: Vec<OnceLock<Box<[[f64; 4]]>>>,
}

impl PartialEq for NurbsSurface {
    fn eq(&self, o: &NurbsSurface) -> bool {
        self.bu == o.bu && self.bv == o.bv && self.ctrl == o.ctrl && self.weights == o.weights
    }
}

impl NurbsSurface {
    pub fn new(
        degrees: (usize, usize),
        knots: (Vec<f64>, Vec<f64>),
        counts: (usize, usize),
        ctrl: Vec<Point3>,
        weights: Option<Vec<f64>>,
    ) -> Result<NurbsSurface> {
        if ctrl.len() != counts.0 * counts.1 {
            return invalid("B-spline surface control net has the wrong size");
        }
        let bu = Basis::new(degrees.0, knots.0, counts.0)?;
        let bv = Basis::new(degrees.1, knots.1, counts.1)?;
        check_weights(&weights, ctrl.len())?;
        let weights =
            weights.filter(|w| w.first().is_some_and(|&first| w.iter().any(|&x| x != first)));
        let mut s = NurbsSurface {
            bu,
            bv,
            ctrl,
            weights,
            u_period: None,
            v_period: None,
            seeds: OnceLock::new(),
            patches: OnceLock::new(),
            beziers: OnceLock::new(),
            polys: OnceLock::new(),
        };
        let ((u0, u1), (v0, v1)) = (s.bu.domain(), s.bv.domain());
        let n = 12;
        let closed_u = (0..=n).all(|k| {
            let v = v0 + (v1 - v0) * k as f64 / n as f64;
            s.eval(u0, v).dist(s.eval(u1, v)) <= tol::LINEAR
        });
        let closed_v = (0..=n).all(|k| {
            let u = u0 + (u1 - u0) * k as f64 / n as f64;
            s.eval(u, v0).dist(s.eval(u, v1)) <= tol::LINEAR
        });
        s.u_period = closed_u.then_some(u1 - u0);
        s.v_period = closed_v.then_some(v1 - v0);
        Ok(s)
    }

    /// The surface exactly as [`NurbsSurface::new`] made it, from its parts and its periods.
    pub fn restore(
        degrees: (usize, usize),
        knots: (Vec<f64>, Vec<f64>),
        counts: (usize, usize),
        ctrl: Vec<Point3>,
        weights: Option<Vec<f64>>,
        periods: (Option<f64>, Option<f64>),
    ) -> Result<NurbsSurface> {
        if ctrl.len() != counts.0 * counts.1 {
            return invalid("B-spline surface control net has the wrong size");
        }
        check_weights(&weights, ctrl.len())?;
        Ok(NurbsSurface {
            bu: Basis::new(degrees.0, knots.0, counts.0)?,
            bv: Basis::new(degrees.1, knots.1, counts.1)?,
            ctrl,
            weights,
            u_period: periods.0,
            v_period: periods.1,
            seeds: OnceLock::new(),
            patches: OnceLock::new(),
            beziers: OnceLock::new(),
            polys: OnceLock::new(),
        })
    }

    pub fn degrees(&self) -> (usize, usize) {
        (self.bu.degree, self.bv.degree)
    }

    pub fn knots(&self) -> (&[f64], &[f64]) {
        (&self.bu.knots, &self.bv.knots)
    }

    /// The number of control points along `u` and along `v`.
    pub fn counts(&self) -> (usize, usize) {
        (self.bu.n, self.bv.n)
    }

    pub fn control_points(&self) -> &[Point3] {
        &self.ctrl
    }

    pub fn weights(&self) -> Option<&[f64]> {
        self.weights.as_deref()
    }

    pub fn domain(&self) -> ((f64, f64), (f64, f64)) {
        (self.bu.domain(), self.bv.domain())
    }

    pub fn u_period(&self) -> Option<f64> {
        self.u_period
    }

    pub fn v_period(&self) -> Option<f64> {
        self.v_period
    }

    /// The number of knot spans along `u` and along `v`.
    pub fn span_counts(&self) -> (usize, usize) {
        (self.bu.spans().len() - 1, self.bv.spans().len() - 1)
    }

    pub fn u_breakpoints(&self, u0: f64, u1: f64) -> Vec<f64> {
        self.bu.breakpoints(u0, u1, self.u_period)
    }

    pub fn v_breakpoints(&self, v0: f64, v1: f64) -> Vec<f64> {
        self.bv.breakpoints(v0, v1, self.v_period)
    }

    /// The point and its two first partial derivatives.
    pub fn d1(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        match self.bu.degree.max(self.bv.degree) {
            0..4 => self.d1_n::<4>(u, v),
            4..8 => self.d1_n::<8>(u, v),
            _ => self.d1_n::<M>(u, v),
        }
    }

    pub(super) fn d1_n<const N: usize>(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        let u = self.bu.wrap(u, self.u_period);
        let v = self.bv.wrap(v, self.v_period);
        let (pu, pv) = (self.bu.degree, self.bv.degree);
        let (iu, iv) = (self.bu.span(u), self.bv.span(v));
        let [nu0, nu1, _] = self.bu.ders::<N>(iu, u, 1);
        let [nv0, nv1, _] = self.bv.ders::<N>(iv, v, 1);
        // The point, and the sums for its u and v derivatives.
        let (mut a0, mut a1, mut a2) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        let (mut w0, mut w1, mut w2) = (0.0, 0.0, 0.0);
        for (k, (&fu, &du)) in nu0.iter().zip(&nu1).enumerate().take(pu + 1) {
            // The row summed along v for the value and its v derivative.
            let (mut r0, mut r1) = (Vec3::ZERO, Vec3::ZERO);
            let (mut rw0, mut rw1) = (0.0, 0.0);
            for (l, (&fv, &dv)) in nv0.iter().zip(&nv1).enumerate().take(pv + 1) {
                let (q, wt) = self.net_point(iu - pu + k, iv - pv + l);
                let pw = q * wt;
                r0 += pw * fv;
                rw0 += wt * fv;
                r1 += pw * dv;
                rw1 += wt * dv;
            }
            a0 += r0 * fu;
            w0 += rw0 * fu;
            a1 += r0 * du;
            w1 += rw0 * du;
            a2 += r1 * fu;
            w2 += rw1 * fu;
        }
        let s = a0 / w0;
        (s, (a1 - s * w1) / w0, (a2 - s * w2) / w0)
    }

    /// Control point `(i, j)`, the `i`-th along `u` and the `j`-th along `v`, and its weight (1 on
    /// a polynomial surface); no numbers past the net, which no span reaches.
    fn net_point(&self, i: usize, j: usize) -> (Point3, f64) {
        let k = i * self.bv.n + j;
        debug_assert!(k < self.ctrl.len(), "control point ({i}, {j}) lies in the net");
        let point = self.ctrl.get(k).copied().unwrap_or(Vec3::new(f64::NAN, f64::NAN, f64::NAN));
        let weight = self.weights.as_ref().map_or(1.0, |w| w.get(k).copied().unwrap_or(f64::NAN));
        (point, weight)
    }

    pub fn eval(&self, u: f64, v: f64) -> Point3 {
        self.d1(u, v).0
    }

    /// [`NurbsSurface::d1`] from the span polynomials where they evaluate the surface: a Horner
    /// pass in place of the basis recurrence, equal to rounding.
    pub(crate) fn d1_by_polys(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        if !self.by_polys() {
            return self.d1(u, v);
        }
        let (h, n) = self.poly_ders(u, v, 1);
        let [s, su, sv, ..] = rational(&h, n);
        (s, su, sv)
    }

    /// The surface along one parameter with the other held at `at`, as a curve of its own: along
    /// `v` with `u = at` when `along_v`, else along `u` with `v = at`. Each control point of the
    /// curve is a row of the net, in homogeneous coordinates, blended by the held parameter's basis
    /// there, so the curve is the surface exactly. A row of a valid net is a valid curve, so an
    /// error here means the net was not.
    pub fn iso_curve(&self, along_v: bool, at: f64) -> Result<NurbsCurve> {
        let (held, free, period) = if along_v {
            (&self.bu, &self.bv, self.u_period)
        } else {
            (&self.bv, &self.bu, self.v_period)
        };
        let at = held.wrap(at, period);
        let (p, i) = (held.degree, held.span(at));
        let [funs, ..] = held.ders::<M>(i, at, 0);
        let net_point = |h: usize, f: usize| {
            if along_v { self.net_point(h, f) } else { self.net_point(f, h) }
        };
        let mut ctrl = Vec::with_capacity(free.n);
        let mut weights = Vec::with_capacity(free.n);
        for f in 0..free.n {
            let (mut sum, mut w) = (Vec3::ZERO, 0.0);
            for (k, n) in funs.iter().enumerate().take(p + 1) {
                let (q, weight) = net_point(i - p + k, f);
                let wk = weight * n;
                sum += q * wk;
                w += wk;
            }
            ctrl.push(sum / w);
            weights.push(w);
        }
        NurbsCurve::new(free.degree, free.knots.clone(), ctrl, Some(weights))
    }

    /// The surface along one parameter with the other held at `at`: [`IsoLine::d1`] gives what
    /// [`NurbsSurface::d1`] does there, the held parameter's basis worked out once.
    pub fn iso(&self, along_v: bool, at: f64) -> IsoLine<'_> {
        let (held, free) = if along_v { (&self.bu, &self.bv) } else { (&self.bv, &self.bu) };
        let at = held.wrap(at, if along_v { self.u_period } else { self.v_period });
        let (p, i) = (held.degree, held.span(at));
        if self.by_polys() {
            let (start, stop) = held.span_range(i);
            let w = stop - start;
            let x = (at - start) / w;
            let (x, end) = if x > 0.5 { (x - 1.0, 1) } else { (x, 0) };
            return IsoLine {
                surface: self,
                along_v,
                rows: Vec::new(),
                held: Some((i, x, end, w)),
                series: RefCell::new(Vec::new()),
            };
        }
        let net_point = |h: usize, f: usize| {
            if along_v { self.net_point(h, f) } else { self.net_point(f, h) }
        };
        let rows = held.ders_then(i, at, 1, |[nh0, nh1, _]| {
            let mut rows = Vec::with_capacity(free.n);
            for j in 0..free.n {
                let (mut r0, mut r1) = (Vec3::ZERO, Vec3::ZERO);
                let (mut w0, mut w1) = (0.0, 0.0);
                for (k, (&h0, &h1)) in nh0.iter().zip(nh1).enumerate().take(p + 1) {
                    let (q, wt) = net_point(i - p + k, j);
                    let pw = q * wt;
                    r0 += pw * h0;
                    w0 += wt * h0;
                    r1 += pw * h1;
                    w1 += wt * h1;
                }
                rows.push(([r0, r1], [w0, w1]));
            }
            rows
        });
        IsoLine { surface: self, along_v, rows, held: None, series: RefCell::new(Vec::new()) }
    }

    /// The control points that move the point at `(u, v)`, as `(index, R)`: the point moves by
    /// `R` times a control point's displacement (the rational basis function).
    pub fn basis(&self, u: f64, v: f64) -> Vec<(usize, f64)> {
        let u = self.bu.wrap(u, self.u_period);
        let v = self.bv.wrap(v, self.v_period);
        let (pu, pv) = (self.bu.degree, self.bv.degree);
        let (iu, iv) = (self.bu.span(u), self.bv.span(v));
        let nu = self.bu.ders::<M>(iu, u, 0);
        let nv = self.bv.ders::<M>(iv, v, 0);
        let stride = self.bv.n;
        let mut out = Vec::with_capacity((pu + 1) * (pv + 1));
        let mut total = 0.0;
        for (k, bu) in nu[0].iter().enumerate().take(pu + 1) {
            for (l, bv) in nv[0].iter().enumerate().take(pv + 1) {
                let idx = (iu - pu + k) * stride + iv - pv + l;
                let (_, weight) = self.net_point(iu - pu + k, iv - pv + l);
                let r = weight * bu * bv;
                total += r;
                out.push((idx, r));
            }
        }
        for o in &mut out {
            o.1 /= total;
        }
        out
    }

    /// The same surface with its control points moved to `ctrl`.
    pub fn with_control_points(&self, ctrl: Vec<Point3>) -> Result<NurbsSurface> {
        NurbsSurface::new(
            (self.bu.degree, self.bv.degree),
            (self.bu.knots.clone(), self.bv.knots.clone()),
            (self.bu.n, self.bv.n),
            ctrl,
            self.weights.clone(),
        )
    }

    pub fn transformed(&self, t: &Transform) -> NurbsSurface {
        // A rigid motion carries what is already worked out with it: the seeds are points of the
        // surface, and each span polynomial's homogeneous coefficients map as the control points
        // do, `(R·c + t·w, w)`.
        let seeds = OnceLock::new();
        if let Some(old) = self.seeds.get() {
            let moved = old.pts.iter().map(|&(uv, p)| (uv, t.point(p))).collect();
            let _ = seeds.set(Seeds::new(moved, old.spacing));
        }
        let polys = OnceLock::new();
        if let Some(sp) = self.polys.get() {
            let cells = sp
                .cells
                .iter()
                .map(|cell| {
                    let out = OnceLock::new();
                    if let Some(c) = cell.get() {
                        let moved: Box<[[f64; 4]]> = c
                            .iter()
                            .map(|&[x, y, z, w]| {
                                let v = t.vector(Vec3::new(x, y, z)) + t.t * w;
                                [v.x, v.y, v.z, w]
                            })
                            .collect();
                        let _ = out.set(moved);
                    }
                    out
                })
                .collect();
            let _ = polys.set(SpanPolys {
                u_slot: sp.u_slot.clone(),
                v_slot: sp.v_slot.clone(),
                u_basis: sp.u_basis.clone(),
                v_basis: sp.v_basis.clone(),
                cells,
            });
        }
        NurbsSurface {
            bu: self.bu.clone(),
            bv: self.bv.clone(),
            ctrl: self.ctrl.iter().map(|&p| t.point(p)).collect(),
            weights: self.weights.clone(),
            u_period: self.u_period,
            v_period: self.v_period,
            seeds,
            patches: OnceLock::new(),
            beziers: OnceLock::new(),
            polys,
        }
    }

    /// Works out now what projection and evaluation otherwise build on first use, so that copies
    /// moved rigidly from this one (see [`NurbsSurface::transformed`]) start with it.
    pub fn prepare(&self) {
        self.seed_grid();
        if self.by_polys() {
            let polys = self.polys();
            for (iu, &su) in polys.u_slot.iter().enumerate() {
                for (iv, &sv) in polys.v_slot.iter().enumerate() {
                    if su.is_some() && sv.is_some() {
                        for e in 0..4 {
                            self.cell(iu, iv, e / 2, e % 2);
                        }
                    }
                }
            }
        }
    }

    /// The box of the control points, which holds the surface (its weights are positive).
    pub fn hull_box(&self) -> crate::math::Aabb {
        let mut bb = crate::math::Aabb::EMPTY;
        for &p in &self.ctrl {
            bb.add(p);
        }
        bb
    }

    /// Boxes whose union holds the surface over its whole domain: the knot spans, each cut into
    /// pieces about a sixteenth of the surface's longer side, and each piece's box the hull of its
    /// own control points after knot insertion (weights are positive, so the hull holds it).
    pub fn patches(&self) -> &[Patch] {
        self.patches.get_or_init(|| {
            let lengths = |along_u: bool| -> f64 {
                let (a0, a1) = if along_u { self.bu.domain() } else { self.bv.domain() };
                let (b0, b1) = if along_u { self.bv.domain() } else { self.bu.domain() };
                (0..=4)
                    .map(|j| {
                        let y = b0 + (b1 - b0) * j as f64 / 4.0;
                        let at = |x: f64| {
                            if along_u { self.eval(x, y) } else { self.eval(y, x) }
                        };
                        (0..32)
                            .map(|i| {
                                let x = |k: usize| a0 + (a1 - a0) * k as f64 / 32.0;
                                at(x(i)).dist(at(x(i + 1)))
                            })
                            .sum::<f64>()
                    })
                    .fold(0.0, f64::max)
            };
            let (lu, lv) = (lengths(true), lengths(false));
            let h = lu.max(lv) / 16.0;
            let splits = |b: &Basis, len: f64| -> Vec<f64> {
                let spans = b.spans();
                let (a, z) = b.domain();
                let mut out = Vec::new();
                for (lo, hi) in spans.iter().copied().zip(spans.iter().copied().skip(1)) {
                    let share = len * (hi - lo) / (z - a);
                    let k = if h > 0.0 { ((share / h).ceil() as usize).clamp(1, 16) } else { 1 };
                    out.extend((1..k).map(|i| lo + (hi - lo) * i as f64 / k as f64));
                }
                out
            };
            let (nu, nv) = (self.bu.n, self.bv.n);
            let mut net: Vec<Vec<[f64; 4]>> = (0..nu)
                .map(|i| {
                    (0..nv)
                        .map(|j| {
                            let (p, w) = self.net_point(i, j);
                            [p.x * w, p.y * w, p.z * w, w]
                        })
                        .collect()
                })
                .collect();
            let mut kv = self.bv.knots.clone();
            let mut rows: Vec<Vec<[f64; 4]>> = Vec::new();
            for (r, row) in net.iter().enumerate() {
                let mut knots = self.bv.knots.clone();
                let mut pts = row.clone();
                for &t in &splits(&self.bv, lv) {
                    insert_knot(&mut knots, self.bv.degree, &mut pts, t);
                }
                if r == 0 {
                    kv = knots;
                }
                rows.push(pts);
            }
            net = rows;
            let mut ku = self.bu.knots.clone();
            let cols = net.first().map_or(0, Vec::len);
            let mut columns: Vec<Vec<[f64; 4]>> = Vec::new();
            for c in 0..cols {
                let mut knots = self.bu.knots.clone();
                let mut pts: Vec<[f64; 4]> =
                    net.iter().filter_map(|row| row.get(c).copied()).collect();
                for &t in &splits(&self.bu, lu) {
                    insert_knot(&mut knots, self.bu.degree, &mut pts, t);
                }
                if c == 0 {
                    ku = knots;
                }
                columns.push(pts);
            }
            let (pu, pv) = (self.bu.degree, self.bv.degree);
            let mut out = Vec::new();
            // Each knot span `i` past the first `p`, as `(i, (start, end))`.
            let spans = |knots: &[f64], p: usize| -> Vec<(usize, (f64, f64))> {
                let pairs = knots.iter().copied().zip(knots.iter().copied().skip(1));
                pairs.enumerate().skip(p).take(knots.len().saturating_sub(2 * p + 1)).collect()
            };
            let v_spans = spans(&kv, pv);
            for (i, (u_lo, u_hi)) in spans(&ku, pu) {
                if u_hi <= u_lo {
                    continue;
                }
                for &(j, (v_lo, v_hi)) in &v_spans {
                    if v_hi <= v_lo {
                        continue;
                    }
                    let mut bbox = Aabb::EMPTY;
                    for col in columns.iter().skip(j - pv).take(pv + 1) {
                        for q in col.iter().skip(i - pu).take(pu + 1) {
                            bbox.add(Vec3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3]));
                        }
                    }
                    out.push(Patch { bbox, lo: Vec2::new(u_lo, v_lo), hi: Vec2::new(u_hi, v_hi) });
                }
            }
            out
        })
    }

    /// The surface at `(u, v)` in homogeneous coordinates: the weighted sum of its control points
    /// and the sum of the weights. A polynomial over each pair of spans, where the point is not.
    fn homogeneous(&self, u: f64, v: f64) -> [f64; 4] {
        let u = self.bu.wrap(u, self.u_period);
        let v = self.bv.wrap(v, self.v_period);
        let (pu, pv) = (self.bu.degree, self.bv.degree);
        let (iu, iv) = (self.bu.span(u), self.bv.span(v));
        let nu = self.bu.ders::<M>(iu, u, 0);
        let nv = self.bv.ders::<M>(iv, v, 0);
        let mut out = [0.0; 4];
        for (k, bu) in nu[0].iter().enumerate().take(pu + 1) {
            for (l, bv) in nv[0].iter().enumerate().take(pv + 1) {
                let (p, weight) = self.net_point(iu - pu + k, iv - pv + l);
                let w = weight * bu * bv;
                out = [out[0] + w * p.x, out[1] + w * p.y, out[2] + w * p.z, out[3] + w];
            }
        }
        out
    }

    /// Each of [`NurbsSurface::patches`] as a Bézier patch. A patch lies within one span of each
    /// parameter, where the surface is a polynomial in homogeneous coordinates, so its values at a
    /// grid of points fix its Bézier control points.
    pub fn beziers(&self) -> &[Bezier] {
        self.beziers.get_or_init(|| {
            let (pu, pv) = (self.bu.degree, self.bv.degree);
            let (mu, mv) = (bernstein_from_values(pu), bernstein_from_values(pv));
            self.patches()
                .iter()
                .map(|patch| {
                    // Values a hair inside the patch's far sides, on its own spans' polynomial.
                    let node = |lo: f64, hi: f64, j: usize, n: usize| {
                        lo + (hi - lo) * (j as f64 / n.max(1) as f64).min(1.0 - 1e-12) // tol: a hair inside
                    };
                    let values: Vec<Vec<[f64; 4]>> = (0..=pu)
                        .map(|i| {
                            let u = node(patch.lo.x, patch.hi.x, i, pu);
                            (0..=pv)
                                .map(|j| self.homogeneous(u, node(patch.lo.y, patch.hi.y, j, pv)))
                                .collect()
                        })
                        .collect();
                    // Along v in each row, then along u in each column.
                    let across: Vec<Vec<[f64; 4]>> = values
                        .iter()
                        .map(|row| {
                            mv.iter()
                                .take(pv + 1)
                                .map(|mk| {
                                    std::array::from_fn(|c| {
                                        let products = mk.iter().zip(row).take(pv + 1);
                                        products.map(|(m, q)| m * coordinate(q, c)).sum()
                                    })
                                })
                                .collect()
                        })
                        .collect();
                    let mut net = Vec::with_capacity((pu + 1) * (pv + 1));
                    for weights in &mu {
                        for col in 0..=pv {
                            net.push(std::array::from_fn(|c| {
                                weights
                                    .iter()
                                    .zip(&across)
                                    .map(|(w, row)| {
                                        w * row.get(col).map_or(f64::NAN, |q| coordinate(q, c))
                                    })
                                    .sum()
                            }));
                        }
                    }
                    Bezier { lo: patch.lo, hi: patch.hi, rows: pu + 1, cols: pv + 1, net }
                })
                .collect()
        })
    }

    /// The seeds, and the spacing they were laid out at along the surface.
    pub(super) fn seed_grid(&self) -> &Seeds {
        self.seeds.get_or_init(|| {
            let samples = |b: &Basis, total: usize| -> Vec<f64> {
                let spans = b.spans();
                let per = (24 / (spans.len() - 1)).clamp(2, 6);
                let per = per.max(total.div_ceil(spans.len() - 1)).min(256);
                let mut out = Vec::new();
                for (lo, hi) in spans.iter().copied().zip(spans.iter().copied().skip(1)) {
                    for k in 0..per {
                        out.push(lo + (hi - lo) * k as f64 / per as f64);
                    }
                }
                out.extend(spans.last());
                out
            };
            let (us, vs) = (samples(&self.bu, 0), samples(&self.bv, 0));
            // A long narrow patch (a twisted loft's side between close corners) needs seeds about
            // as close along its length as across it, or the nearest seed can lie in the basin of
            // a farther point.
            let length = |a: &[f64], b: &[f64], swap: bool| -> f64 {
                b.iter()
                    .map(|&y| {
                        let at = |x: f64| {
                            if swap { self.eval(y, x) } else { self.eval(x, y) }
                        };
                        let steps = a.iter().zip(a.iter().skip(1));
                        steps.map(|(&x0, &x1)| at(x0).dist(at(x1))).sum::<f64>()
                    })
                    .fold(0.0, f64::max)
            };
            let (lu, lv) = (length(&us, &vs, false), length(&vs, &us, true));
            // At most about 2048 seeds, since every global projection measures them all.
            let h = (lu.min(lv) / 4.0).max((lu * lv / 2048.0).sqrt());
            let (us, vs) = if h > 0.0 {
                let n = |l: f64| ((l / h).ceil() as usize).min(256);
                (samples(&self.bu, n(lu)), samples(&self.bv, n(lv)))
            } else {
                (us, vs)
            };
            let mut out = Vec::with_capacity(us.len() * vs.len());
            for &u in &us {
                for &v in &vs {
                    out.push((Vec2::new(u, v), self.eval(u, v)));
                }
            }
            Seeds::new(out, h)
        })
    }

    /// The point, its first partial derivatives and its second, `(S, Su, Sv, Suu, Suv, Svv)`.
    pub fn d2(&self, u: f64, v: f64) -> [Vec3; 6] {
        if !self.by_polys() {
            return if self.bu.degree.max(self.bv.degree) < 8 {
                self.d2_n::<8>(u, v)
            } else {
                self.d2_n::<M>(u, v)
            };
        }
        let (h, n) = self.poly_ders(u, v, 2);
        rational(&h, n)
    }

    /// Whether the span polynomials evaluate this surface: past degree 6 their monomial
    /// coefficients cancel by more than the precision the kernel keeps, and the basis recurrence
    /// does instead.
    pub(super) fn by_polys(&self) -> bool {
        self.bu.degree.max(self.bv.degree) <= MAX_POLY_DEGREE
    }

    fn polys(&self) -> &SpanPolys {
        self.polys.get_or_init(|| {
            let slots = |b: &Basis| {
                let mut slot = vec![None; b.n];
                let mut basis = Vec::new();
                for (i, s) in slot.iter_mut().enumerate().skip(b.degree) {
                    let (start, stop) = b.span_range(i);
                    if stop > start {
                        *s = Some(basis.len());
                        basis.push([b.span_poly(i, false), b.span_poly(i, true)]);
                    }
                }
                (slot, basis)
            };
            let (u_slot, u_basis) = slots(&self.bu);
            let (v_slot, v_basis) = slots(&self.bv);
            let cells = (0..4 * u_basis.len() * v_basis.len()).map(|_| OnceLock::new()).collect();
            SpanPolys { u_slot, v_slot, u_basis, v_basis, cells }
        })
    }

    /// The homogeneous coefficients over spans `iu` and `iv`, about the ends `eu` and `ev` (each
    /// 0 for the start, 1 for the end), row by row along u.
    pub(super) fn cell(&self, iu: usize, iv: usize, eu: usize, ev: usize) -> &[[f64; 4]] {
        let polys = self.polys();
        let slot = |slots: &[Option<usize>], i: usize| slots.get(i).copied().flatten();
        debug_assert!(
            slot(&polys.u_slot, iu).is_some() && slot(&polys.v_slot, iv).is_some(),
            "spans ({iu}, {iv}) are nonempty, as `Basis::span` gives them"
        );
        let (Some(su), Some(sv)) = (slot(&polys.u_slot, iu), slot(&polys.v_slot, iv)) else {
            // Only a nonempty span has a polynomial, and `Basis::span` gives no other.
            return &[];
        };
        fn series(basis: &[[Vec<Vec<f64>>; 2]], s: usize, e: usize) -> Option<&Vec<Vec<f64>>> {
            basis.get(s).and_then(|ends| ends.get(e))
        }
        let (Some(cu), Some(cv)) = (series(&polys.u_basis, su, eu), series(&polys.v_basis, sv, ev))
        else {
            return &[];
        };
        let at = (2 * su + eu) * 2 * polys.v_basis.len() + 2 * sv + ev;
        let Some(cell) = polys.cells.get(at) else {
            return &[];
        };
        cell.get_or_init(|| {
            let (pu, pv) = (self.bu.degree, self.bv.degree);
            // Each control row summed against the v polynomials, then the rows against u's.
            let mut rows = vec![[0.0; 4]; (pu + 1) * (pv + 1)];
            for (a, row) in rows.chunks_mut(pv + 1).enumerate() {
                for (b, series) in cv.iter().enumerate().take(pv + 1) {
                    let (p, w) = self.net_point(iu - pu + a, iv - pv + b);
                    let pw = [p.x * w, p.y * w, p.z * w, w];
                    for (r, &c) in row.iter_mut().zip(series) {
                        for (r, x) in r.iter_mut().zip(pw) {
                            *r += x * c;
                        }
                    }
                }
            }
            let mut out = vec![[0.0; 4]; (pu + 1) * (pv + 1)];
            for (k, out_row) in out.chunks_mut(pv + 1).enumerate() {
                for (row, series) in rows.chunks(pv + 1).zip(cu) {
                    let Some(&c) = series.get(k) else {
                        continue;
                    };
                    for (o, x) in out_row.iter_mut().zip(row) {
                        for (o, x) in o.iter_mut().zip(x) {
                            *o += x * c;
                        }
                    }
                }
            }
            out.into_boxed_slice()
        })
    }

    /// The homogeneous point and its derivatives up to `order` (at most 2), in the order
    /// `(0,0), (1,0), (0,1), (2,0), (1,1), (0,2)`, and how many of them are filled.
    fn poly_ders(&self, u: f64, v: f64, order: usize) -> ([[f64; 4]; 6], usize) {
        let u = self.bu.wrap(u, self.u_period);
        let v = self.bv.wrap(v, self.v_period);
        let (pu, pv) = (self.bu.degree, self.bv.degree);
        let (iu, iv) = (self.bu.span(u), self.bv.span(v));
        let ((u_start, u_stop), (v_start, v_stop)) =
            (self.bu.span_range(iu), self.bv.span_range(iv));
        let (hu, hv) = (u_stop - u_start, v_stop - v_start);
        // The series about the nearer end of each span: exact at both ends, and |s| ≤ ½.
        let local = |x: f64| if x > 0.5 { (x - 1.0, 1) } else { (x, 0) };
        let ((s, eu), (t, ev)) = (local((u - u_start) / hu), local((v - v_start) / hv));
        let c = self.cell(iu, iv, eu, ev);
        debug_assert_eq!(c.len(), (pu + 1) * (pv + 1), "a cell holds every coefficient");
        // Each row along u evaluated in t with its first two t derivatives (the second halved).
        let mut r = [[[0.0; 4]; 3]; MAX_POLY_DEGREE + 1];
        for (slot, row) in r.iter_mut().zip(c.chunks(pv + 1)).take(pu + 1) {
            let Some((&top, lower)) = row.split_last() else {
                continue;
            };
            let (mut p0, mut p1, mut p2) = (top, [0.0; 4], [0.0; 4]);
            for &coefficient in lower.iter().rev() {
                if order >= 2 {
                    p2 = horner_step(p2, p1, t);
                }
                if order >= 1 {
                    p1 = horner_step(p1, p0, t);
                }
                p0 = horner_step(p0, coefficient, t);
            }
            *slot = [p0, p1, p2.map(|x| 2.0 * x)];
        }
        // The rows' values (`pick` the row's value, t derivative or second t derivative), each
        // in s with its first two s derivatives (the second halved).
        let rows = r.get(..=pu).unwrap_or_default();
        let horner = |pick: fn(&[[f64; 4]; 3]) -> [f64; 4], want: usize| {
            let Some((top, lower)) = rows.split_last() else {
                return ([0.0; 4], [0.0; 4], [0.0; 4]);
            };
            let (mut q0, mut q1, mut q2) = (pick(top), [0.0; 4], [0.0; 4]);
            for row in lower.iter().rev() {
                if want >= 2 {
                    q2 = horner_step(q2, q1, s);
                }
                if want >= 1 {
                    q1 = horner_step(q1, q0, s);
                }
                q0 = horner_step(q0, pick(row), s);
            }
            (q0, q1, q2.map(|x| 2.0 * x))
        };
        let scale = |a: [f64; 4], k: f64| a.map(|x| x * k);
        let mut out = [[0.0; 4]; 6];
        let (a, a_s, a_ss) = horner(|row| row[0], order);
        out[0] = a;
        if order == 0 {
            return (out, 1);
        }
        let (a_t, a_st, _) = horner(|row| row[1], order - 1);
        out[1] = scale(a_s, 1.0 / hu);
        out[2] = scale(a_t, 1.0 / hv);
        if order == 1 {
            return (out, 3);
        }
        let (a_tt, _, _) = horner(|row| row[2], 0);
        out[3] = scale(a_ss, 1.0 / (hu * hu));
        out[4] = scale(a_st, 1.0 / (hu * hv));
        out[5] = scale(a_tt, 1.0 / (hv * hv));
        (out, 6)
    }

    pub(super) fn d2_n<const N: usize>(&self, u: f64, v: f64) -> [Vec3; 6] {
        let u = self.bu.wrap(u, self.u_period);
        let v = self.bv.wrap(v, self.v_period);
        let (pu, pv) = (self.bu.degree, self.bv.degree);
        let (iu, iv) = (self.bu.span(u), self.bv.span(v));
        let [nu0, nu1, nu2] = self.bu.ders::<N>(iu, u, 2);
        let [nv0, nv1, nv2] = self.bv.ders::<N>(iv, v, 2);
        // Homogeneous sums for the orders (0,0), (1,0), (0,1), (2,0), (1,1), (0,2), each row of
        // the net summed along v first for the three v orders.
        let mut a = [Vec3::ZERO; 6];
        let mut w = [0.0; 6];
        let along_u = nu0.iter().zip(&nu1).zip(&nu2);
        for (k, ((&f0, &f1), &f2)) in along_u.enumerate().take(pu + 1) {
            let mut ra = [Vec3::ZERO; 3];
            let mut rw = [0.0; 3];
            let along_v = nv0.iter().zip(&nv1).zip(&nv2);
            for (l, ((&g0, &g1), &g2)) in along_v.enumerate().take(pv + 1) {
                let (q, wt) = self.net_point(iu - pu + k, iv - pv + l);
                let pw = q * wt;
                for ((ra, rw), g) in ra.iter_mut().zip(&mut rw).zip([g0, g1, g2]) {
                    *ra += pw * g;
                    *rw += wt * g;
                }
            }
            let ([r0, r1, r2], [rw0, rw1, rw2]) = (ra, rw);
            // The v order's row sum and the u order's function, for each order in turn.
            let terms = [
                (r0, rw0, f0),
                (r0, rw0, f1),
                (r1, rw1, f0),
                (r0, rw0, f2),
                (r1, rw1, f1),
                (r2, rw2, f0),
            ];
            for ((a, w), (r, rw, f)) in a.iter_mut().zip(&mut w).zip(terms) {
                *a += r * f;
                *w += rw * f;
            }
        }
        let ([a0, a1, a2, a3, a4, a5], [w0, w1, w2, w3, w4, w5]) = (a, w);
        let s = a0 / w0;
        let su = (a1 - s * w1) / w0;
        let sv = (a2 - s * w2) / w0;
        let suu = (a3 - su * (2.0 * w1) - s * w3) / w0;
        let suv = (a4 - su * w2 - sv * w1 - s * w4) / w0;
        let svv = (a5 - sv * (2.0 * w2) - s * w5) / w0;
        [s, su, sv, suu, suv, svv]
    }
}

/// `a·k + b`, coordinate by coordinate: one step of a Horner pass.
fn horner_step(a: [f64; 4], b: [f64; 4], k: f64) -> [f64; 4] {
    let mut out = a;
    for (o, b) in out.iter_mut().zip(b) {
        *o = *o * k + b;
    }
    out
}

/// Homogeneous coordinate `c` of `q`; no number past the fourth.
fn coordinate(q: &[f64; 4], c: usize) -> f64 {
    debug_assert!(c < 4, "homogeneous coordinate {c} is one of four");
    q.get(c).copied().unwrap_or(f64::NAN)
}

/// The point and its derivatives on a rational surface from the homogeneous ones `h`, the first
/// `n` of `(0,0), (1,0), (0,1), (2,0), (1,1), (0,2)` filled; the rest come back zero.
fn rational(h: &[[f64; 4]; 6], n: usize) -> [Vec3; 6] {
    let [h0, h1, h2, h3, h4, h5] = *h;
    let a = |q: [f64; 4]| Vec3::new(q[0], q[1], q[2]);
    let w = |q: [f64; 4]| q[3];
    let s = a(h0) / w(h0);
    let mut out = [s, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO];
    if n >= 3 {
        out[1] = (a(h1) - s * w(h1)) / w(h0);
        out[2] = (a(h2) - s * w(h2)) / w(h0);
    }
    if n >= 6 {
        let (su, sv) = (out[1], out[2]);
        out[3] = (a(h3) - su * (2.0 * w(h1)) - s * w(h3)) / w(h0);
        out[4] = (a(h4) - su * w(h2) - sv * w(h1) - s * w(h4)) / w(h0);
        out[5] = (a(h5) - sv * (2.0 * w(h2)) - s * w(h5)) / w(h0);
    }
    out
}
