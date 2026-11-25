// yack: a spline surface along one parameter, the other held.
// Adapted from springrep@a03e8a1 src/geom/nurbs.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/nurbs.rs to yack-geom (M3 W3); split out of
// nurbs.rs into its own module (M3 hardening). M3 hardening: the held parameter's place is passed
// to the polynomial path instead of a counted `panic!`, the series are Horner passes over
// iterators, and a span's rows and a cell's lines come by iterators; no indexing, the same
// arithmetic in the same order.
//! A spline surface along one parameter, the other held.

use std::cell::RefCell;

use super::{NurbsSurface, basis::Basis};
use crate::math::{Point3, Vec3};

/// A NURBS surface along one parameter, from [`NurbsSurface::iso`]: the span polynomials with the
/// held parameter put in where they evaluate the surface, and otherwise, for each control point
/// along the free parameter, the held parameter's weighted sums of the control net and of its
/// derivative.
pub struct IsoLine<'a> {
    pub(super) surface: &'a NurbsSurface,
    pub(super) along_v: bool,
    pub(super) rows: Vec<([Vec3; 2], [f64; 2])>,
    /// Where the span polynomials evaluate the surface: the held parameter's span, its local
    /// parameter about the span's nearer end, that end, and the span's width.
    pub(super) held: Option<Held>,
    /// For each free span and end used so far, the span polynomial with the held parameter put
    /// in: the homogeneous point's series and its held derivative's, by power of the free one.
    pub(super) series: RefCell<Vec<(usize, usize, IsoSeries)>>,
}

/// The held parameter's span, its local parameter about the span's nearer end, that end (0 for
/// the start, 1 for the end), and the span's width.
pub(super) type Held = (usize, f64, usize, f64);

/// An iso line's homogeneous point and held derivative over one free span, by power of the free
/// parameter.
pub(super) type IsoSeries = Vec<[[f64; 4]; 2]>;

impl IsoLine<'_> {
    /// The point and the partial derivatives along `u` and `v` at free parameter `t`.
    pub fn d1(&self, t: f64) -> (Point3, Vec3, Vec3) {
        let s = self.surface;
        let (free, period) = if self.along_v { (&s.bv, s.v_period) } else { (&s.bu, s.u_period) };
        let t = free.wrap(t, period);
        let (p, i) = (free.degree, free.span(t));
        if let Some(held) = self.held {
            return self.d1_poly(free, i, t, held);
        }
        free.ders_then(i, t, 1, |nf| self.d1_from(nf, p, i))
    }

    /// [`IsoLine::d1`] by a Horner pass over the span polynomial in the free parameter.
    fn d1_poly(&self, free: &Basis, i: usize, t: f64, held: Held) -> (Point3, Vec3, Vec3) {
        let (start, stop) = free.span_range(i);
        let h = stop - start;
        let x = (t - start) / h;
        let (x, end) = if x > 0.5 { (x - 1.0, 1) } else { (x, 0) };
        let mut cache = self.series.borrow_mut();
        if let Some((_, _, series)) = cache.iter().find(|c| c.0 == i && c.1 == end) {
            return self.horner(series, x, h);
        }
        let series = self.collapse(i, end, held);
        let out = self.horner(&series, x, h);
        cache.push((i, end, series));
        out
    }

    /// The point and its derivatives at local parameter `x` of a free span of width `h` from its
    /// series.
    fn horner(&self, series: &IsoSeries, x: f64, h: f64) -> (Point3, Vec3, Vec3) {
        debug_assert!(!series.is_empty(), "a span polynomial has a constant term");
        let Some((&[mut a0, mut a1], lower)) = series.split_last() else {
            let none = Vec3::new(f64::NAN, f64::NAN, f64::NAN);
            return (none, none, none);
        };
        let mut a2 = [0.0; 4];
        for &[c0, c1] in lower.iter().rev() {
            for (((a2, a0), a1), (c0, c1)) in
                a2.iter_mut().zip(&mut a0).zip(&mut a1).zip(c0.into_iter().zip(c1))
            {
                *a2 = *a2 * x + *a0;
                *a0 = *a0 * x + c0;
                *a1 = *a1 * x + c1;
            }
        }
        let v = |a: [f64; 4]| Vec3::new(a[0], a[1], a[2]);
        let pt = v(a0) / a0[3];
        let held = (v(a1) - pt * a1[3]) / a0[3];
        let along = (v(a2) - pt * a2[3]) / (a0[3] * h);
        if self.along_v { (pt, held, along) } else { (pt, along, held) }
    }

    /// The span polynomial over free span `i` about its end `end`, the held parameter put in.
    fn collapse(&self, i: usize, end: usize, (hi, hx, he, hw): Held) -> IsoSeries {
        let s = self.surface;
        let (pu, pv) = (s.bu.degree, s.bv.degree);
        // Row by row along u, `pv + 1` coefficients each.
        let cell = if self.along_v { s.cell(hi, i, he, end) } else { s.cell(i, hi, end, he) };
        let pf = if self.along_v { pv } else { pu };
        // The coefficients along the held parameter for free index `f`, lowest power first.
        let line = |f: usize| -> Vec<[f64; 4]> {
            if self.along_v {
                cell.iter().skip(f).step_by(pv + 1).copied().collect()
            } else {
                cell.iter().skip(f * (pv + 1)).take(pv + 1).copied().collect()
            }
        };
        (0..=pf)
            .map(|f| {
                let line = line(f);
                debug_assert!(!line.is_empty(), "a cell's line has a constant term");
                let Some((&top, lower)) = line.split_last() else {
                    return [[f64::NAN; 4]; 2];
                };
                let (mut val, mut der) = (top, [0.0; 4]);
                for &next in lower.iter().rev() {
                    for ((d, v), n) in der.iter_mut().zip(&mut val).zip(next) {
                        *d = *d * hx + *v;
                        *v = *v * hx + n;
                    }
                }
                [val, der.map(|d| d / hw)]
            })
            .collect()
    }

    /// [`IsoLine::d1`] from the free parameter's basis values and derivatives `nf` on span `i`.
    fn d1_from(&self, [n0, n1, _]: [&[f64]; 3], p: usize, i: usize) -> (Point3, Vec3, Vec3) {
        // a[0], w[0]: the point; [1]: along the held parameter; [2]: along the free one.
        let (mut a0, mut a1, mut a2) = (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        let (mut w0, mut w1, mut w2) = (0.0, 0.0, 0.0);
        let rows = self.rows.iter().skip(i - p).take(p + 1);
        for ((r, rw), (&f0, &f1)) in rows.zip(n0.iter().zip(n1)) {
            a0 += r[0] * f0;
            w0 += rw[0] * f0;
            a1 += r[1] * f0;
            w1 += rw[1] * f0;
            a2 += r[0] * f1;
            w2 += rw[0] * f1;
        }
        let pt = a0 / w0;
        let held = (a1 - pt * w1) / w0;
        let along = (a2 - pt * w2) / w0;
        if self.along_v { (pt, held, along) } else { (pt, along, held) }
    }
}
