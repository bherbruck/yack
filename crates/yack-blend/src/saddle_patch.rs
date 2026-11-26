// yack: The corner patch where four fillets meet at a saddle: two concave and two convex, alternating.
// Adapted from springrep@a03e8a1 src/blend/saddle_patch.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/blend/saddle_patch.rs to yack-blend (M3 W9a); paths and visibility only;
// checked access outside its numeric kernels, and no panics (hardening SP1).
//! The corner patch where four fillets meet at a saddle: two concave and two convex, alternating.
//!
//! The hole they leave has four sides, each on one fillet: the concave fillets end on quarter
//! circles square to their edges, and the convex ones on curves joining those circles' ends. Each
//! corner is a right angle: there a convex fillet's end leaves along its cylinder's generator or
//! goes round it, square to the circle beside. A circle cut in a plane cannot leave along a
//! generator, so a convex end is a rational curve of degree 4 on its cylinder: the circle in the
//! parameter `s = u²` (or `1 − (1 − u)²`), whose speed is nil at the generator end, and the distance
//! along the axis over the same denominator, still at the other end, where the curve goes round.
//!
//! The patch is a rational B-spline of degree [`DEGREE`] each way whose boundary rows and columns
//! are those sides exactly. Every fillet is a cylinder, whose tangent plane holds its axis `A` and
//! the direction round it `T`, so the homogeneous cross derivative along a side is built as
//! `λ·Cʷ + κ·T + ν·A`: whatever `λ`, `κ` and `ν`, the patch leaves the side in the fillet's tangent
//! plane, and meets it G1 to rounding. The four corners, where two sides' cross derivatives share
//! a control point, are linear conditions on them. The unknowns fit a Gregory surface on the same
//! sides and tangent planes (J. A. Gregory, "Smooth interpolation without twist constraints", in
//! Computer Aided Geometric Design, Academic Press, 1974), tangent to every fillet but not a
//! B-spline: the cross
//! derivatives its ribbons, and the inner control points its positions.

use crate::{
    Error, Result,
    geom::NurbsSurface,
    math::{Point3, Vec3},
    numeric,
};

/// How far the patch's normal may turn from a fillet's along their shared side, in radians.
pub(super) const CREASE: f64 = 1e-11;

/// A fillet's cylinder by a point on its axis and the axis's unit direction.
#[derive(Clone, Copy, Debug)]
pub(super) struct Cylinder {
    pub origin: Point3,
    pub axis: Vec3,
}

impl Cylinder {
    pub(super) fn axial(&self, p: Point3) -> f64 {
        (p - self.origin).dot(self.axis)
    }

    pub(super) fn offset(&self, p: Point3) -> Vec3 {
        let d = p - self.origin;
        d - self.axis * d.dot(self.axis)
    }

    /// The unit normal at `p`, away from the axis.
    fn normal(&self, p: Point3) -> Vec3 {
        self.offset(p).unit_or_zero()
    }
}

type H = [f64; 4];

fn h(p: Vec3, w: f64) -> H {
    [p.x * w, p.y * w, p.z * w, w]
}

/// Coordinate `d` (0 to 3) of a homogeneous point.
fn coord(q: &H, d: usize) -> f64 {
    match d {
        0 => q[0],
        1 => q[1],
        2 => q[2],
        _ => q[3],
    }
}

/// A knot vector with no span at a parameter the fit asks for.
fn no_span() -> Error {
    Error::Numerical("a saddle patch's knots have no span there".into())
}

/// Entry `i` of a table the fit builds to its own sizes; an index past it is a broken fit.
fn entry<T: Copy>(table: &[T], i: usize) -> Result<T> {
    table.get(i).copied().ok_or_else(|| Error::Numerical(format!("a saddle patch's table of {} has no entry {i}", table.len())))
}

fn binom(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
}

fn bernstein(n: usize, t: f64) -> Vec<f64> {
    (0..=n)
        .map(|k| binom(n, k) * t.powi(k as i32) * (1.0 - t).powi((n - k) as i32))
        .collect()
}

/// A rational Bézier curve by its homogeneous control points.
#[derive(Clone, Debug)]
pub(super) struct Rational {
    pub h: Vec<H>,
}

impl Rational {
    /// The point and first derivative at `t`.
    pub(super) fn d1(&self, t: f64) -> (Point3, Vec3) {
        let n = self.h.len() - 1;
        let b = bernstein(n, t);
        let lower = bernstein(n - 1, t);
        let mut c = [0.0; 4];
        let mut dc = [0.0; 4];
        for (k, (hk, &bk)) in self.h.iter().zip(&b).enumerate() {
            // `lower` has `n` coefficients: none before the first or past the last.
            let db = n as f64
                * (lower.get(k.wrapping_sub(1)).copied().unwrap_or(0.0) - lower.get(k).copied().unwrap_or(0.0));
            for ((cd, dcd), &hd) in c.iter_mut().zip(dc.iter_mut()).zip(hk) {
                *cd += bk * hd;
                *dcd += db * hd;
            }
        }
        let p = Vec3::new(c[0], c[1], c[2]) / c[3];
        let dp = (Vec3::new(dc[0], dc[1], dc[2]) - p * dc[3]) / c[3];
        (p, dp)
    }

    /// The same curve one degree higher.
    fn elevated(&self) -> Rational {
        let n = self.h.len() - 1;
        let mut out: Vec<H> = self.h.first().copied().into_iter().collect();
        for (k, pair) in (1..=n).zip(self.h.windows(2)) {
            let &[lo, hi] = pair else { continue };
            let a = k as f64 / (n + 1) as f64;
            let mut q = [0.0; 4];
            for ((qd, &l), &u) in q.iter_mut().zip(&lo).zip(&hi) {
                *qd = a * l + (1.0 - a) * u;
            }
            out.push(q);
        }
        out.extend(self.h.last().copied());
        Rational { h: out }
    }
}

/// The rational quadratic arc about `centre` from `a` to `b`, the short way.
fn quadratic_arc(centre: Point3, a: Point3, b: Point3) -> Result<Rational> {
    let (oa, ob) = (a - centre, b - centre);
    let half = 0.5 * oa.cross(ob).norm().atan2(oa.dot(ob));
    let w = half.cos();
    let Some(mid) = (oa + ob).unit() else {
        return Err(Error::Unsupported(
            "a saddle's arc that turns a half turn".into(),
        ));
    };
    let m = centre + mid * (oa.norm() / w);
    Ok(Rational {
        h: vec![h(a, 1.0), h(m, w), h(b, 1.0)],
    })
}

/// A concave fillet's end: the quarter circle of `cyl` from `a` to `b`, both on one section.
pub(super) fn concave_end(cyl: &Cylinder, a: Point3, b: Point3) -> Result<Rational> {
    let centre = cyl.origin + cyl.axis * cyl.axial(a);
    quadratic_arc(centre, a, b)
}

/// A convex fillet's end on `cyl` from `from` to `to`: leaving `from` along the generator when
/// `generator_first`, else reaching `to` along it, and going round the cylinder at its other end.
pub(super) fn convex_end(
    cyl: &Cylinder,
    from: Point3,
    to: Point3,
    generator_first: bool,
) -> Result<Rational> {
    // The circle part: offsets from the axis, as a rational quadratic in s.
    let (oa, ob) = (cyl.offset(from), cyl.offset(to));
    let circle = quadratic_arc(Vec3::ZERO, oa, ob)?;
    let s_of = |u: f64| {
        if generator_first {
            u * u
        } else {
            1.0 - (1.0 - u) * (1.0 - u)
        }
    };
    let homog_circle = |u: f64| -> H {
        let b = bernstein(2, s_of(u));
        let mut c = [0.0; 4];
        for (hk, &bk) in circle.h.iter().zip(&b) {
            for (cd, &hd) in c.iter_mut().zip(hk) {
                *cd += bk * hd;
            }
        }
        c
    };
    // Its Bernstein coefficients of degree 4, by interpolation at five points.
    let us: Vec<f64> = (0..=4).map(|k| k as f64 / 4.0).collect();
    let mut a = Vec::with_capacity(25);
    let mut rhs = Vec::with_capacity(20);
    for &u in &us {
        a.extend(bernstein(4, u));
        rhs.extend(homog_circle(u));
    }
    let coef = numeric::solve_dense(a, rhs, 5, 4)?;
    // Five homogeneous coefficients, each (x, y, z, w).
    let &[c0, c1, c2, c3, c4] = coef.as_chunks::<4>().0 else {
        return Err(Error::Numerical("a saddle's convex end has no shape".into()));
    };
    let rows = [c0, c1, c2, c3, c4];
    let w = rows.map(|c| c[3]);
    let (xa, xb) = (cyl.axial(from), cyl.axial(to));
    // The axial numerator: its ends fixed, still where the curve goes round, the rest following
    // a quarter sine from the generator end, which leaves it at the generator's own speed.
    let mut x = [0.0; 5];
    x[0] = xa * w[0];
    x[4] = xb * w[4];
    let (fixed_mid, free) = if generator_first {
        x[3] = x[4] - xb * (w[4] - w[3]);
        (3, [1, 2])
    } else {
        x[1] = x[0] + xa * (w[1] - w[0]);
        (1, [2, 3])
    };
    let target = |u: f64| {
        if generator_first {
            xa + (xb - xa) * (0.5 * std::f64::consts::PI * u).sin()
        } else {
            xb + (xa - xb) * (0.5 * std::f64::consts::PI * u).cos()
        }
    };
    let (mut n11, mut n12, mut n22, mut r1, mut r2) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for k in 0..=40 {
        let u = k as f64 / 40.0;
        let &[b0, b1, b2, b3, b4] = bernstein(4, u).as_slice() else { continue };
        let b = [b0, b1, b2, b3, b4];
        let wu: f64 = b.iter().zip(&w).map(|(bi, wi)| bi * wi).sum();
        let (bm, xm) = if fixed_mid == 3 { (b3, x[3]) } else { (b1, x[1]) };
        let known: f64 = [b0 * x[0], bm * xm, b4 * x[4]].iter().sum();
        let r = target(u) * wu - known;
        let (c1, c2) = if free == [1, 2] { (b1, b2) } else { (b2, b3) };
        n11 += c1 * c1;
        n12 += c1 * c2;
        n22 += c2 * c2;
        r1 += c1 * r;
        r2 += c2 * r;
    }
    let (x1, x2) = numeric::solve2(n11, n12, n12, n22, r1, r2)
        .ok_or_else(|| Error::Numerical("a saddle's convex end has no shape".into()))?;
    if free == [1, 2] {
        (x[1], x[2]) = (x1, x2);
    } else {
        (x[2], x[3]) = (x1, x2);
    }
    let h = rows
        .iter()
        .zip(&x)
        .map(|(&[cx, cy, cz, wk], &xk)| {
            let p = cyl.origin * wk + cyl.axis * xk + Vec3::new(cx, cy, cz);
            [p.x, p.y, p.z, wk]
        })
        .collect();
    Ok(Rational { h })
}

/// The four sides and the fillet each lies on: `bottom` (v = 0) and `top` (v = 1) along u from
/// `left`'s ends to `right`'s, `left` (u = 0) and `right` (u = 1) along v from `bottom` to `top`.
pub(super) struct Sides {
    pub bottom: (Rational, Cylinder),
    pub top: (Rational, Cylinder),
    pub left: (Rational, Cylinder),
    pub right: (Rational, Cylinder),
}

/// The Gregory surface on the sides: positions and cross derivatives along each, the cross
/// derivative in the fillet's tangent plane, its parts along and across the side interpolated
/// between the ends, where they are the next sides' tangents.
struct Gregory<'a> {
    s: &'a Sides,
    twist: [[(Vec3, Vec3); 2]; 2],
}

fn ribbon(side: &(Rational, Cylinder), d0: Vec3, d1: Vec3, t: f64) -> Vec3 {
    let parts = |t: f64, d: Vec3| {
        let (p, dp) = side.0.d1(t);
        let tu = dp.unit_or_zero();
        let m = side.1.normal(p).cross(tu);
        (d.dot(tu), d.dot(m))
    };
    let (a0, b0) = parts(0.0, d0);
    let (a1, b1) = parts(1.0, d1);
    let (p, dp) = side.0.d1(t);
    let tu = dp.unit_or_zero();
    let m = side.1.normal(p).cross(tu);
    tu * (a0 + (a1 - a0) * t) + m * (b0 + (b1 - b0) * t)
}

impl<'a> Gregory<'a> {
    fn new(s: &'a Sides) -> Gregory<'a> {
        let mut g = Gregory {
            s,
            twist: [[(Vec3::ZERO, Vec3::ZERO); 2]; 2],
        };
        let mut twist = g.twist;
        for (i, row) in twist.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                let e = 1e-6;
                let fd = |f: &dyn Fn(f64) -> Vec3, t: f64| {
                    let (a, b) = ((t - e).max(0.0), (t + e).min(1.0));
                    (f(b) - f(a)) / (b - a)
                };
                let along_u = fd(&|t| g.cross_v(j, t), i as f64);
                let along_v = fd(&|t| g.cross_u(i, t), j as f64);
                *cell = (along_u, along_v);
            }
        }
        g.twist = twist;
        g
    }

    /// The cross derivative d/dv along the side v = j, at u = t.
    fn cross_v(&self, j: usize, t: f64) -> Vec3 {
        let side = if j == 0 { &self.s.bottom } else { &self.s.top };
        ribbon(
            side,
            self.s.left.0.d1(j as f64).1,
            self.s.right.0.d1(j as f64).1,
            t,
        )
    }

    /// The cross derivative d/du along the side u = i, at v = t.
    fn cross_u(&self, i: usize, t: f64) -> Vec3 {
        let side = if i == 0 { &self.s.left } else { &self.s.right };
        ribbon(
            side,
            self.s.bottom.0.d1(i as f64).1,
            self.s.top.0.d1(i as f64).1,
            t,
        )
    }

    fn eval(&self, u: f64, v: f64) -> Point3 {
        let herm = |t: f64| {
            [
                1.0 - 3.0 * t * t + 2.0 * t * t * t,
                3.0 * t * t - 2.0 * t * t * t,
                t - 2.0 * t * t + t * t * t,
                -t * t + t * t * t,
            ]
        };
        let (hu, hv) = (herm(u), herm(v));
        let s = self.s;
        let lv = s.bottom.0.d1(u).0 * hv[0]
            + s.top.0.d1(u).0 * hv[1]
            + self.cross_v(0, u) * hv[2]
            + self.cross_v(1, u) * hv[3];
        let lu = s.left.0.d1(v).0 * hu[0]
            + s.right.0.d1(v).0 * hu[1]
            + self.cross_u(0, v) * hu[2]
            + self.cross_u(1, v) * hu[3];
        let mut t = Vec3::ZERO;
        for (i, twists) in self.twist.iter().enumerate() {
            for (j, &(a, b)) in twists.iter().enumerate() {
                let row = if j == 0 { &s.bottom.0 } else { &s.top.0 };
                let col = if i == 0 { &s.left.0 } else { &s.right.0 };
                let (p, pu) = row.d1(i as f64);
                let pv = col.d1(j as f64).1;
                // Gregory's twist (`a`, `b`): each side's own wins along it.
                let du = if i == 0 { u } else { 1.0 - u };
                let dv = if j == 0 { v } else { 1.0 - v };
                let puv = if du + dv == 0.0 {
                    (a + b) * 0.5
                } else {
                    (b * du + a * dv) / (du + dv)
                };
                let ui = if i == 0 {
                    [hu[0], hu[2]]
                } else {
                    [hu[1], hu[3]]
                };
                let vj = if j == 0 {
                    [hv[0], hv[2]]
                } else {
                    [hv[1], hv[3]]
                };
                t = t
                    + p * (ui[0] * vj[0])
                    + pu * (ui[1] * vj[0])
                    + pv * (ui[0] * vj[1])
                    + puv * (ui[1] * vj[1]);
            }
        }
        lu + lv - t
    }
}

/// A symmetric positive definite band of half-width `bw`, factored in place as L·Lᵀ.
struct Band {
    n: usize,
    bw: usize,
    a: Vec<f64>,
}

// A numeric kernel: its indices stay inside the band it allocates (`at` is below `n·(bw + 1)` for
// `j ≤ i < n`, `i − j ≤ bw`).
#[allow(clippy::indexing_slicing)]
impl Band {
    fn new(n: usize, bw: usize) -> Band {
        Band {
            n,
            bw,
            a: vec![0.0; n * (bw + 1)],
        }
    }

    /// Row `i`, column `j ≤ i`.
    fn at(&self, i: usize, j: usize) -> usize {
        i * (self.bw + 1) + (i - j)
    }

    /// Adds `x` at (`a`, `b`), `a ≤ b`, the lower triangle holding it.
    fn add(&mut self, a: usize, b: usize, x: f64) {
        let k = self.at(b, a);
        self.a[k] += x;
    }

    fn factor(&mut self) -> Result<()> {
        for i in 0..self.n {
            let lo = i.saturating_sub(self.bw);
            for j in lo..=i {
                let mut sum = self.a[self.at(i, j)];
                for k in lo.max(j.saturating_sub(self.bw))..j {
                    sum -= self.a[self.at(i, k)] * self.a[self.at(j, k)];
                }
                if i == j {
                    if sum <= 0.0 {
                        return Err(Error::Numerical("a saddle patch's fit is singular".into()));
                    }
                    let k = self.at(i, i);
                    self.a[k] = sum.sqrt();
                } else {
                    let k = self.at(i, j);
                    self.a[k] = sum / self.a[self.at(j, j)];
                }
            }
        }
        Ok(())
    }

    fn solve(&self, mut b: Vec<f64>) -> Vec<f64> {
        for i in 0..self.n {
            let lo = i.saturating_sub(self.bw);
            let sum: f64 = (lo..i).map(|k| self.a[self.at(i, k)] * b[k]).sum();
            b[i] = (b[i] - sum) / self.a[self.at(i, i)];
        }
        for i in (0..self.n).rev() {
            let hi = (i + self.bw).min(self.n - 1);
            let sum: f64 = (i + 1..=hi).map(|k| self.a[self.at(k, i)] * b[k]).sum();
            b[i] = (b[i] - sum) / self.a[self.at(i, i)];
        }
        b
    }
}

/// How many slots apart two free control points within the degrees of each other can lie.
fn slot_reach(slot: &[usize], nu: usize, nv: usize, pu: usize, pv: usize) -> usize {
    let mut reach = 0;
    for i in 0..nu {
        for j in 0..nv {
            let Some(&a) = slot.get(i * nv + j) else { continue };
            if a == usize::MAX {
                continue;
            }
            for ii in i..=(i + pu).min(nu - 1) {
                for jj in j.saturating_sub(pv)..=(j + pv).min(nv - 1) {
                    if let Some(&b) = slot.get(ii * nv + jj)
                        && b != usize::MAX
                    {
                        reach = reach.max(a.abs_diff(b));
                    }
                }
            }
        }
    }
    reach
}

/// A Bézier's control points refined to `knots` by knot insertion; `None` for a knot outside them.
// A numeric kernel: its indices stay inside the knots and points it builds.
#[allow(clippy::indexing_slicing)]
fn refined(r: &Rational, inner: &[f64]) -> Option<(Vec<H>, Vec<f64>)> {
    let p = r.h.len() - 1;
    let mut ctrl = r.h.clone();
    let mut knots: Vec<f64> = std::iter::repeat_n(0.0, p + 1)
        .chain(std::iter::repeat_n(1.0, p + 1))
        .collect();
    for &t in inner {
        let s = (0..knots.len() - 1).rfind(|&i| knots[i] <= t && t < knots[i + 1])?;
        let mut next = Vec::with_capacity(ctrl.len() + 1);
        for i in 0..=ctrl.len() {
            if i + p <= s {
                next.push(ctrl[i]);
            } else if i > s {
                next.push(ctrl[i - 1]);
            } else {
                let a = (t - knots[i]) / (knots[i + p] - knots[i]);
                let mut q = [0.0; 4];
                for d in 0..4 {
                    q[d] = (1.0 - a) * ctrl[i - 1][d] + a * ctrl[i][d];
                }
                next.push(q);
            }
        }
        ctrl = next;
        knots.insert(s + 1, t);
    }
    Some((ctrl, knots))
}

/// The values and first derivatives of every degree-`p` basis function at `t`; `None` without a
/// span there.
// A numeric kernel: its indices stay inside the knots and the tables it allocates.
#[allow(clippy::indexing_slicing)]
fn basis(knots: &[f64], p: usize, t: f64) -> Option<(Vec<f64>, Vec<f64>)> {
    let n = knots.len() - p - 1;
    let t = t.clamp(knots[p], knots[n] - 1e-13);
    let span = (p..n).rfind(|&i| knots[i] <= t)?;
    // Cox–de Boor up to degree p, keeping degree p − 1 for the derivative.
    let mut nb = vec![0.0; n + p];
    nb[span] = 1.0;
    let mut prev = nb.clone();
    for k in 1..=p {
        prev = nb.clone();
        let mut next = vec![0.0; n + p];
        for i in span.saturating_sub(k)..=span {
            let a = if knots[i + k] > knots[i] {
                (t - knots[i]) / (knots[i + k] - knots[i]) * nb[i]
            } else {
                0.0
            };
            let b = if knots[i + k + 1] > knots[i + 1] {
                (knots[i + k + 1] - t) / (knots[i + k + 1] - knots[i + 1]) * nb[i + 1]
            } else {
                0.0
            };
            next[i] = a + b;
        }
        nb = next;
    }
    let mut d = vec![0.0; n];
    for i in 0..n {
        let a = if knots[i + p] > knots[i] {
            p as f64 / (knots[i + p] - knots[i]) * prev[i]
        } else {
            0.0
        };
        let b = if knots[i + p + 1] > knots[i + 1] {
            p as f64 / (knots[i + p + 1] - knots[i + 1]) * prev[i + 1]
        } else {
            0.0
        };
        d[i] = a - b;
    }
    Some((nb[..n].to_vec(), d))
}

/// Solves the symmetric positive semidefinite `a`·x = `b` by Cholesky with diagonal pivoting,
/// leaving out the directions whose pivots fall to rounding: nearly dependent constraints.
// A numeric kernel: its indices stay inside the `m × m` matrix and the vectors of `m` it is given.
#[allow(clippy::indexing_slicing)]
fn psd_solve(a: &[f64], b: &[f64], m: usize) -> Vec<f64> {
    let mut l = a.to_vec();
    let mut order: Vec<usize> = (0..m).collect();
    let top = (0..m).map(|i| a[i * m + i]).fold(0.0f64, f64::max);
    let mut rank = 0;
    while rank < m {
        // `rank < m`: the range is never empty.
        let Some(k) = (rank..m).max_by(|&i, &j| l[order[i] * m + order[i]].total_cmp(&l[order[j] * m + order[j]]))
        else {
            break;
        };
        let pk = order[k];
        let d = l[pk * m + pk];
        if d <= 1e-13 * top {
            break;
        }
        order.swap(rank, k);
        let s = d.sqrt();
        for &i in &order[rank..] {
            l[i * m + pk] /= s;
        }
        for x in rank + 1..m {
            let i = order[x];
            let li = l[i * m + pk];
            for &j in &order[rank + 1..=x] {
                l[i * m + j] -= li * l[j * m + pk];
                l[j * m + i] = l[i * m + j];
            }
        }
        rank += 1;
    }
    // L, row order[x] column order[y] for y ≤ x, solved forward then back over the rank kept.
    let col = |x: usize, y: usize| l[order[x] * m + order[y]];
    let mut z = vec![0.0; rank];
    for x in 0..rank {
        let sum: f64 = (0..x).map(|y| col(x, y) * z[y]).sum();
        z[x] = (b[order[x]] - sum) / col(x, x);
    }
    for x in (0..rank).rev() {
        let sum: f64 = (x + 1..rank).map(|y| col(y, x) * z[y]).sum();
        z[x] = (z[x] - sum) / col(x, x);
    }
    let mut out = vec![0.0; m];
    for x in 0..rank {
        out[order[x]] = z[x];
    }
    out
}

/// The patch's degree each way. A side's cross derivative is built from polynomials of degree
/// `DEGREE` less the side's own, so the convex sides (degree 4) leave quadratics there.
const DEGREE: usize = 6;

/// The rational Bézier coefficients of `r` raised to degree `p`.
fn raised(r: &Rational, p: usize) -> Rational {
    let mut out = r.clone();
    while out.h.len() - 1 < p {
        out = out.elevated();
    }
    out
}

/// The degree `m + d` Bézier coefficients of the Bernstein polynomial `B^m_j` times the degree `d`
/// curve with coefficients `c`.
// A numeric kernel: `j + k ≤ m + d` and `x < 4` stay inside the table it allocates.
#[allow(clippy::indexing_slicing)]
fn times_bernstein(m: usize, j: usize, c: &[H]) -> Vec<H> {
    let d = c.len() - 1;
    let mut out = vec![[0.0; 4]; m + d + 1];
    for (k, ck) in c.iter().enumerate() {
        let f = binom(m, j) * binom(d, k) / binom(m + d, j + k);
        for x in 0..4 {
            out[j + k][x] += f * ck[x];
        }
    }
    out
}

// A numeric kernel: `x < 4` stays inside the homogeneous points.
#[allow(clippy::indexing_slicing)]
fn bezier(c: &[H], t: f64) -> H {
    let b = bernstein(c.len() - 1, t);
    let mut out = [0.0; 4];
    for (bk, ck) in b.iter().zip(c) {
        for x in 0..4 {
            out[x] += bk * ck[x];
        }
    }
    out
}

/// Values over the unknowns: each unknown's index and its coefficient.
type Terms<T> = Vec<(usize, T)>;

/// One side's cross derivative, as unknowns: the homogeneous derivative row is
/// `Q = λ·Cʷ + κ·T + ν·A`, with `Cʷ` the side's homogeneous curve, `T = A × (Cʷ − O·w)` going round
/// its cylinder, `A` the cylinder's axis (weight nil), `λ` and `κ` polynomials of degree `m` and
/// `ν` a spline on the patch's knots. Projected, `S' = (κ·T + ν·A) / w`: in the fillet's tangent
/// plane whatever the unknowns, so the patch meets the fillet G1 exactly.
struct Cross<'a> {
    side: &'a (Rational, Cylinder),
    /// The degree of `λ` and `κ`.
    m: usize,
    /// The spline coefficients of `B^m_j·Cʷ` and of `B^m_j·T`, for each `j`.
    lambda: Vec<Vec<H>>,
    kappa: Vec<Vec<H>>,
    /// `T`'s Bézier coefficients, at the side's own degree.
    round: Vec<H>,
    /// Control points along the side.
    n: usize,
    /// Where the side's unknowns start: `λ`, then `κ`, then `ν`.
    at: usize,
}

impl<'a> Cross<'a> {
    fn new(side: &'a (Rational, Cylinder), inner: &[f64], at: usize) -> Result<Cross<'a>> {
        let c = &side.0.h;
        let cyl = side.1;
        let m = DEGREE - (c.len() - 1);
        let round: Vec<H> = c
            .iter()
            .map(|q| {
                let t = cyl
                    .axis
                    .cross(Vec3::new(q[0], q[1], q[2]) - cyl.origin * q[3]);
                [t.x, t.y, t.z, 0.0]
            })
            .collect();
        let spline = |coef: Vec<H>| {
            refined(&Rational { h: coef }, inner)
                .map(|(ctrl, _)| ctrl)
                .ok_or_else(|| Error::Numerical("a saddle patch's knot lies outside its side".into()))
        };
        let lambda = (0..=m).map(|j| spline(times_bernstein(m, j, c))).collect::<Result<Vec<Vec<H>>>>()?;
        let kappa = (0..=m)
            .map(|j| spline(times_bernstein(m, j, &round)))
            .collect::<Result<Vec<Vec<H>>>>()?;
        let n = lambda.first().map_or(0, Vec::len);
        Ok(Cross {
            side,
            m,
            lambda,
            kappa,
            round,
            n,
            at,
        })
    }

    fn unknowns(&self) -> usize {
        2 * (self.m + 1) + self.n
    }

    /// The `i`th spline coefficient of `Q`, as (unknown, its homogeneous coefficient).
    fn coefficient(&self, i: usize) -> Result<Vec<(usize, H)>> {
        let a = self.side.1.axis;
        let mut out = Vec::with_capacity(2 * self.m + 3);
        for (j, (lambda, kappa)) in self.lambda.iter().zip(&self.kappa).enumerate() {
            out.push((self.at + j, entry(lambda, i)?));
            out.push((self.at + self.m + 1 + j, entry(kappa, i)?));
        }
        out.push((self.at + 2 * (self.m + 1) + i, [a.x, a.y, a.z, 0.0]));
        Ok(out)
    }

    /// At `t`: the cross derivative `S'` and the weight's derivative, each as (unknown, value).
    fn at_t(&self, knots: &[f64], t: f64) -> Result<(Terms<Vec3>, Terms<f64>)> {
        let c = bezier(&self.side.0.h, t);
        let w = c[3];
        let r = bezier(&self.round, t);
        let round = Vec3::new(r[0], r[1], r[2]) / w;
        let bm = bernstein(self.m, t);
        let (nb, _) = basis(knots, DEGREE, t).ok_or_else(no_span)?;
        let a = self.side.1.axis / w;
        let mut s = Vec::new();
        let mut wt = Vec::new();
        for (j, &b) in bm.iter().enumerate() {
            wt.push((self.at + j, b * w));
            s.push((self.at + self.m + 1 + j, round * b));
        }
        for (i, &x) in nb.iter().enumerate() {
            if x != 0.0 {
                s.push((self.at + 2 * (self.m + 1) + i, a * x));
            }
        }
        Ok((s, wt))
    }
}

/// A dense symmetric positive definite matrix's Cholesky solve, by the band with full width.
fn dense_band(n: usize) -> Band {
    Band::new(n, n.saturating_sub(1))
}

/// The fitted patch on `spans` uniform spans each way, and the largest crease along its sides.
fn fit(s: &Sides, g: &Gregory, spans: usize) -> Result<(NurbsSurface, f64)> {
    let p = DEGREE;
    let inner: Vec<f64> = (1..spans).map(|k| k as f64 / spans as f64).collect();
    let side = |r: &Rational| refined(&raised(r, p), &inner).ok_or_else(no_span);
    let (bb, knots) = side(&s.bottom.0)?;
    let (bt, _) = side(&s.top.0)?;
    let (bl, _) = side(&s.left.0)?;
    let (br, _) = side(&s.right.0)?;
    let n = bb.len();
    // Each side's cross derivative, solved as one system: its fit to the Gregory ribbon, and its
    // first and last coefficients and the corner points beside them shared with the next sides.
    let mut sides: Vec<Cross> = Vec::new();
    let mut at = 0;
    for side in [&s.bottom, &s.top, &s.left, &s.right] {
        let c = Cross::new(side, &inner, at)?;
        at += c.unknowns();
        sides.push(c);
    }
    let total = at;
    let mut normal = dense_band(total);
    let mut rhs = vec![0.0; total];
    let mut row = |terms: &[(usize, f64)], target: f64, wt: f64| -> Result<()> {
        for &(a, xa) in terms {
            *rhs.get_mut(a).ok_or_else(no_span)? += wt * xa * target;
            for &(b, xb) in terms {
                if b >= a {
                    normal.add(a, b, wt * xa * xb);
                }
            }
        }
        Ok(())
    };
    // The derivative's step from the side to the next row in: `Q` scaled by this, with the sign
    // of the side it comes from (in from v = 1 and u = 1 is backwards).
    let step = (entry(&knots, p + 1)? - entry(&knots, 1)?) / p as f64;
    let [bottom, top, left, right] = sides.as_slice() else {
        return Err(Error::Numerical("a saddle patch needs four sides".into()));
    };
    let opposite = [&bt, &bb, &br, &bl];
    for (k, ((cross, own), opposite)) in sides.iter().zip([&bb, &bt, &bl, &br]).zip(opposite).enumerate() {
        for q in 0..8 * spans {
            let t = (q as f64 + 0.5) / (8 * spans) as f64;
            let target = match k {
                0 => g.cross_v(0, t),
                1 => g.cross_v(1, t),
                2 => g.cross_u(0, t),
                _ => g.cross_u(1, t),
            };
            let (sd, wd) = cross.at_t(&knots, t)?;
            for d in 0..3 {
                let terms: Vec<(usize, f64)> = sd.iter().map(|&(a, v)| (a, v[d])).collect();
                row(&terms, target[d], 1.0)?;
            }
            // The weight's derivative, loosely: towards the opposite side's weights.
            let (b, _) = basis(&knots, p, t).ok_or_else(no_span)?;
            let w = |c: &[H]| -> f64 { b.iter().zip(c).map(|(x, h)| x * h[3]).sum() };
            row(&wd, w(opposite) - w(own), 1e-4)?;
        }
    }
    // Shared values, each a homogeneous point: (terms of the unknowns, value).
    let mut cons: Vec<(Vec<(usize, H)>, H)> = Vec::new();
    let scaled = |c: &Cross, i: usize, f: f64| -> Result<Vec<(usize, H)>> {
        Ok(c.coefficient(i)?
            .into_iter()
            .map(|(a, h)| (a, [h[0] * f, h[1] * f, h[2] * f, h[3] * f]))
            .collect())
    };
    let minus = |a: H, b: H| [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]];
    let last = n - 1;
    // Each side's first two and last two points.
    let ends = |c: &[H]| -> Result<[H; 4]> { Ok([entry(c, 0)?, entry(c, 1)?, entry(c, last - 1)?, entry(c, last)?]) };
    let ([bb0, bb1, bbm, bbl], [bt0, bt1, btm, btl]) = (ends(&bb)?, ends(&bt)?);
    let ([bl0, bl1, blm, bll], [br0, br1, brm, brl]) = (ends(&bl)?, ends(&br)?);
    // The ends: each side's first and last steps land on the next sides' second points.
    cons.push((scaled(bottom, 0, step)?, minus(bl1, bb0)));
    cons.push((scaled(bottom, last, step)?, minus(br1, bbl)));
    cons.push((scaled(top, 0, -step)?, minus(blm, bt0)));
    cons.push((scaled(top, last, -step)?, minus(brm, btl)));
    cons.push((scaled(left, 0, step)?, minus(bb1, bl0)));
    cons.push((scaled(left, last, step)?, minus(bt1, bll)));
    cons.push((scaled(right, 0, -step)?, minus(bbm, br0)));
    cons.push((scaled(right, last, -step)?, minus(btm, brl)));
    // The corners in from them, reached from both sides alike.
    let corner = |a: &Cross, ia: usize, fa: f64, b: &Cross, ib: usize, fb: f64, value: H| -> Result<(Vec<(usize, H)>, H)> {
        let mut terms = scaled(a, ia, fa)?;
        terms.extend(scaled(b, ib, -fb)?);
        Ok((terms, value))
    };
    cons.push(corner(bottom, 1, step, left, 1, step, minus(bl1, bb1))?);
    cons.push(corner(bottom, last - 1, step, right, 1, -step, minus(br1, bbm))?);
    cons.push(corner(top, 1, -step, left, last - 1, step, minus(blm, bt1))?);
    cons.push(corner(top, last - 1, -step, right, last - 1, -step, minus(brm, btm))?);
    // Scalar rows, then the constrained minimum by the Schur complement.
    let rows: Vec<(Vec<(usize, f64)>, f64)> = cons
        .iter()
        .flat_map(|(terms, value)| {
            (0..4).map(move |d| (terms.iter().map(|&(a, h)| (a, coord(&h, d))).collect(), coord(value, d)))
        })
        .collect();
    normal.factor()?;
    let nb = normal.solve(rhs);
    let y = rows
        .iter()
        .map(|(terms, _)| -> Result<Vec<f64>> {
            let mut col = vec![0.0; total];
            for &(a, x) in terms {
                *col.get_mut(a).ok_or_else(no_span)? += x;
            }
            Ok(normal.solve(col))
        })
        .collect::<Result<Vec<Vec<f64>>>>()?;
    let m = rows.len();
    let apply = |terms: &[(usize, f64)], v: &[f64]| terms.iter().map(|&(a, x)| Ok(x * entry(v, a)?)).sum::<Result<f64>>();
    // The Schur complement, row by row.
    let mut sm = Vec::with_capacity(m * m);
    for (terms, _) in &rows {
        for yq in &y {
            sm.push(apply(terms, yq)?);
        }
    }
    let mut mu = vec![0.0; m];
    let mut z = nb.clone();
    for _ in 0..3 {
        let e = rows
            .iter()
            .map(|(terms, value)| Ok(apply(terms, &z)? - value))
            .collect::<Result<Vec<f64>>>()?;
        let dmu = psd_solve(&sm, &e, m);
        for (mq, &dq) in mu.iter_mut().zip(&dmu) {
            *mq += dq;
        }
        for (a, (za, &nba)) in z.iter_mut().zip(&nb).enumerate() {
            *za = nba - y.iter().zip(&mu).map(|(yq, &mq)| Ok(entry(yq, a)? * mq)).sum::<Result<f64>>()?;
        }
    }
    let q_of = |c: &Cross, i: usize| -> Result<H> {
        let mut out = [0.0; 4];
        for (a, h) in c.coefficient(i)? {
            let za = entry(&z, a)?;
            for (od, &hd) in out.iter_mut().zip(&h) {
                *od += za * hd;
            }
        }
        Ok(out)
    };
    let along = |base: &H, q: H, f: f64| -> H {
        [
            base[0] + f * q[0],
            base[1] + f * q[1],
            base[2] + f * q[2],
            base[3] + f * q[3],
        ]
    };
    // The net, homogeneous: the sides, the rows and columns next to them from the cross
    // derivatives, and inside them points fitted to the Gregory surface on Coons-blended weights.
    let mut net: Vec<Option<H>> = vec![None; n * n];
    let idx = |i: usize, j: usize| i * n + j;
    let mut put = |k: usize, q: H| -> Result<()> {
        *net.get_mut(k).ok_or_else(no_span)? = Some(q);
        Ok(())
    };
    for i in 0..n {
        put(idx(i, 0), entry(&bb, i)?)?;
        put(idx(i, last), entry(&bt, i)?)?;
        put(idx(0, i), entry(&bl, i)?)?;
        put(idx(last, i), entry(&br, i)?)?;
    }
    for i in 1..last {
        put(idx(i, 1), along(&entry(&bb, i)?, q_of(bottom, i)?, step))?;
        put(idx(i, last - 1), along(&entry(&bt, i)?, q_of(top, i)?, -step))?;
        put(idx(1, i), along(&entry(&bl, i)?, q_of(left, i)?, step))?;
        put(idx(last - 1, i), along(&entry(&br, i)?, q_of(right, i)?, -step))?;
    }
    let greville = |i: usize| -> Result<f64> {
        Ok(knots.get(i + 1..i + p + 1).ok_or_else(no_span)?.iter().sum::<f64>() / p as f64)
    };
    // The weights, row by row as the net.
    let mut w = Vec::with_capacity(n * n);
    for i in 0..n {
        for j in 0..n {
            w.push(match entry(&net, idx(i, j))? {
                Some(h) => h[3],
                None => {
                    let (a, b) = (greville(i)?, greville(j)?);
                    (1.0 - b) * entry(&bb, i)?[3] + b * entry(&bt, i)?[3] + (1.0 - a) * entry(&bl, j)?[3] + a * entry(&br, j)?[3]
                        - ((1.0 - a) * (1.0 - b) * bb0[3]
                            + a * (1.0 - b) * bbl[3]
                            + (1.0 - a) * b * bt0[3]
                            + a * b * btl[3])
                }
            });
        }
    }
    if w.iter().any(|&x| x <= 0.0) {
        return Err(Error::Numerical(
            "a saddle patch's weights turn negative".into(),
        ));
    }
    let cart = |q: &H| Vec3::new(q[0], q[1], q[2]) / q[3];
    let fixed: Vec<Option<Point3>> = net.iter().map(|h| h.as_ref().map(cart)).collect();
    let free: Vec<usize> = fixed.iter().enumerate().filter(|(_, f)| f.is_none()).map(|(k, _)| k).collect();
    let mut slot = vec![usize::MAX; n * n];
    for (s, &k) in free.iter().enumerate() {
        *slot.get_mut(k).ok_or_else(no_span)? = s;
    }
    let nf = free.len();
    let mut band = Band::new(nf, slot_reach(&slot, n, n, p, p));
    let mut rhs = vec![Vec3::ZERO; nf];
    let mut ts: Vec<f64> = Vec::new();
    for k in 0..spans {
        for q in 0..4 {
            ts.push((k as f64 + (q as f64 + 0.5) / 4.0) / spans as f64);
        }
    }
    for &u in &ts {
        let (nu_, _) = basis(&knots, p, u).ok_or_else(no_span)?;
        for &v in &ts {
            let (nv_, _) = basis(&knots, p, v).ok_or_else(no_span)?;
            let mut ww = 0.0;
            let mut terms = Vec::new();
            for (i, &ni) in nu_.iter().enumerate().take(n) {
                if ni == 0.0 {
                    continue;
                }
                for (j, &nj) in nv_.iter().enumerate().take(n) {
                    if nj != 0.0 {
                        let x = ni * nj * entry(&w, idx(i, j))?;
                        ww += x;
                        terms.push((idx(i, j), x));
                    }
                }
            }
            let mut known = Vec3::ZERO;
            let mut c = Vec::new();
            for (k, x) in terms {
                match entry(&fixed, k)? {
                    Some(q) => known += q * (x / ww),
                    None => c.push((entry(&slot, k)?, x / ww)),
                }
            }
            let target = g.eval(u, v) - known;
            for &(a, xa) in &c {
                *rhs.get_mut(a).ok_or_else(no_span)? += target * xa;
                for &(b, xb) in &c {
                    if b >= a {
                        band.add(a, b, xa * xb);
                    }
                }
            }
        }
    }
    band.factor()?;
    let xs = [
        band.solve(rhs.iter().map(|r| r.x).collect()),
        band.solve(rhs.iter().map(|r| r.y).collect()),
        band.solve(rhs.iter().map(|r| r.z).collect()),
    ];
    let [xs0, xs1, xs2] = &xs;
    let ctrl = fixed
        .iter()
        .zip(&slot)
        .map(|(f, &s)| match f {
            Some(q) => Ok(*q),
            None => Ok(Vec3::new(entry(xs0, s)?, entry(xs1, s)?, entry(xs2, s)?)),
        })
        .collect::<Result<Vec<Point3>>>()?;
    let surf = NurbsSurface::new((p, p), (knots.clone(), knots), (n, n), ctrl, Some(w))?;
    let mut crease = 0.0f64;
    for k in 0..=200 {
        let t = k as f64 / 200.0;
        for (u, v, side) in [
            (t, 0.0, &s.bottom),
            (t, 1.0, &s.top),
            (0.0, t, &s.left),
            (1.0, t, &s.right),
        ] {
            let (q, su, sv) = surf.d1(u, v);
            let c = su.cross(sv).unit_or_zero().cross(side.1.normal(q)).norm();
            crease = crease.max(c);
        }
    }
    Ok((surf, crease))
}

/// The patch on `sides`.
pub(super) fn patch(sides: &Sides) -> Result<NurbsSurface> {
    let (surf, crease) = fit(sides, &Gregory::new(sides), SPANS)?;
    if crease > CREASE {
        return Err(Error::Numerical(format!(
            "a saddle patch creases by {crease:.1e} rad against its fillets"
        )));
    }
    Ok(surf)
}

/// Spans each way.
const SPANS: usize = 8;

#[cfg(test)]
mod tests {
    use super::*;

    /// The overhang's saddle at (10, 5, 10), r = 1: the boss's side y = 5, its underside and the
    /// block's top z = 10, the block's side x = 10.
    fn overhang() -> Sides {
        let (p1, p2, p3, p4) = (
            Vec3::new(9.0, 5.0, 11.0),
            Vec3::new(11.0, 4.0, 10.0),
            Vec3::new(10.0, 4.0, 9.0),
            Vec3::new(9.0, 6.0, 10.0),
        );
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let boss_edge = Cylinder {
            origin: Vec3::new(0.0, 4.0, 11.0),
            axis: x,
        };
        let block_edge = Cylinder {
            origin: Vec3::new(9.0, 0.0, 9.0),
            axis: y,
        };
        let along_side = Cylinder {
            origin: Vec3::new(0.0, 6.0, 11.0),
            axis: x,
        };
        let under = Cylinder {
            origin: Vec3::new(11.0, 0.0, 9.0),
            axis: y,
        };
        Sides {
            bottom: (convex_end(&boss_edge, p1, p2, true).unwrap(), boss_edge),
            top: (convex_end(&block_edge, p4, p3, false).unwrap(), block_edge),
            left: (concave_end(&along_side, p1, p4).unwrap(), along_side),
            right: (concave_end(&under, p2, p3).unwrap(), under),
        }
    }

    #[test]
    fn a_saddle_s_sides_lie_on_their_fillets_and_meet_square() {
        let s = overhang();
        for side in [&s.bottom, &s.top, &s.left, &s.right] {
            for k in 0..=20 {
                let p = side.0.d1(k as f64 / 20.0).0;
                assert!((side.1.offset(p).norm() - 1.0).abs() < 1e-12, "{p:?}");
            }
        }
        let corners = [
            (s.bottom.0.d1(0.0).1, s.left.0.d1(0.0).1),
            (s.bottom.0.d1(1.0).1, s.right.0.d1(0.0).1),
            (s.top.0.d1(0.0).1, s.left.0.d1(1.0).1),
            (s.top.0.d1(1.0).1, s.right.0.d1(1.0).1),
        ];
        for (a, b) in corners {
            assert!(a.unit().unwrap().dot(b.unit().unwrap()).abs() < 1e-12);
        }
    }

    #[test]
    fn a_saddle_patch_carries_its_sides_and_meets_each_fillet_g1() {
        let s = overhang();
        let surf = patch(&s).unwrap();
        // Its boundary is the sides, and its normal meets each fillet's within the crease.
        for k in 0..=40 {
            let t = k as f64 / 40.0;
            for (u, v, side) in [
                (t, 0.0, &s.bottom),
                (t, 1.0, &s.top),
                (0.0, t, &s.left),
                (1.0, t, &s.right),
            ] {
                let p = surf.eval(u, v);
                let q = side.0.d1(t).0;
                assert!(p.dist(q) < 1e-12, "{p:?} for {q:?}");
                if k > 0 && k < 40 {
                    let (_, su, sv) = surf.d1(u, v);
                    let n = su.cross(sv).unit().unwrap();
                    assert!(
                        n.cross(side.1.normal(p)).norm() <= CREASE,
                        "crease at {p:?}"
                    );
                }
            }
        }
        // Regular all over, and bending no tighter than a radius of 0.4 mm.
        for i in 0..=30 {
            for j in 0..=30 {
                let (u, v) = (i as f64 / 30.0, j as f64 / 30.0);
                let [_, su, sv, suu, suv, svv] = surf.d2(u, v);
                let n = su.cross(sv);
                assert!(n.norm() > 1.0, "nearly singular at ({u}, {v})");
                let n = n.unit().unwrap();
                let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
                let (l, m, nn) = (suu.dot(n), suv.dot(n), svv.dot(n));
                let k = (l * nn - m * m) / (e * g - f * f);
                let hm = (e * nn - 2.0 * f * m + g * l) / (2.0 * (e * g - f * f));
                let kmax = hm.abs() + (hm * hm - k).max(0.0).sqrt();
                assert!(kmax < 1.0 / 0.4, "radius {} at ({u}, {v})", 1.0 / kmax);
            }
        }
    }
}
