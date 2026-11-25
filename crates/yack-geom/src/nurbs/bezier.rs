// yack: Bézier patches of a spline surface, and the knot insertion that cuts them out.
// Adapted from springrep@a03e8a1 src/geom/nurbs.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/geom/nurbs.rs to yack-geom (M3 W3); split out of
// nurbs.rs into its own module (M3 hardening). M3 hardening: the net is read as rows (chunks)
// and lines of points, de Casteljau shrinks its level a step at a time, and the Bernstein inverse
// takes no counted `panic!`; no indexing, the same arithmetic in the same order.
//! Bézier patches of a spline surface, and the knot insertion that cuts them out.

use crate::math::{Aabb, Vec2, Vec3};

/// A piece of a surface's parameter domain and a box that holds the surface over it.
#[derive(Clone, Copy, Debug)]
pub struct Patch {
    pub bbox: Aabb,
    pub lo: Vec2,
    pub hi: Vec2,
}

/// A piece of a surface over a rectangle of its parameters as a Bézier patch: its control points
/// in homogeneous coordinates, `rows` along u of `cols` along v. The patch lies in their convex
/// hull, and so does each half a split leaves, in its own.
#[derive(Clone, Debug)]
pub struct Bezier {
    pub lo: Vec2,
    pub hi: Vec2,
    pub(super) rows: usize,
    pub(super) cols: usize,
    pub(super) net: Vec<[f64; 4]>,
}

/// A homogeneous point.
type Homogeneous = [f64; 4];

/// `(1 − x)·a + x·b`, coordinate by coordinate.
fn mix(a: Homogeneous, b: Homogeneous, x: f64) -> Homogeneous {
    let mut out = a;
    for (o, b) in out.iter_mut().zip(b) {
        *o = (1.0 - x) * *o + x * b;
    }
    out
}

/// The middle of `a` and `b`, coordinate by coordinate.
fn halfway(a: Homogeneous, b: Homogeneous) -> Homogeneous {
    let mut out = a;
    for (o, b) in out.iter_mut().zip(b) {
        *o = 0.5 * (*o + b);
    }
    out
}

/// The Cartesian point of a homogeneous one.
fn cartesian(q: &Homogeneous) -> Vec3 {
    Vec3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3])
}

/// Each point of `points` paired with the next.
fn neighbours(points: &[Homogeneous]) -> impl Iterator<Item = (&Homogeneous, &Homogeneous)> {
    points.iter().zip(points.iter().skip(1))
}

impl Bezier {
    /// The control net row by row: `rows` rows of `cols` points along v.
    fn net_rows(&self) -> std::slice::Chunks<'_, Homogeneous> {
        self.net.chunks(self.cols.max(1))
    }

    /// The box of the control points, which holds the patch.
    pub fn bbox(&self) -> Aabb {
        let mut bb = Aabb::EMPTY;
        for q in &self.net {
            bb.add(cartesian(q));
        }
        bb
    }

    /// The patch's two halves, cut across u or across v at the middle (de Casteljau).
    pub fn halves(&self, across_u: bool) -> [Bezier; 2] {
        let (lines, len) = if across_u { (self.cols, self.rows) } else { (self.rows, self.cols) };
        let at = |line: usize, k: usize| {
            if across_u { k * self.cols + line } else { line * self.cols + k }
        };
        let (mut first, mut second) = (self.net.clone(), self.net.clone());
        for line in 0..lines {
            // Each step halves every neighbouring pair: its first point ends the first half and
            // its last starts the second.
            let mut level: Vec<Homogeneous> =
                (0..len).filter_map(|k| self.net.get(at(line, k)).copied()).collect();
            for step in 0..len {
                if let (Some(slot), Some(&q)) = (first.get_mut(at(line, step)), level.first()) {
                    *slot = q;
                }
                let back = at(line, len - 1 - step);
                if let (Some(slot), Some(&q)) = (second.get_mut(back), level.last()) {
                    *slot = q;
                }
                level = neighbours(&level).map(|(&a, &b)| halfway(a, b)).collect();
            }
        }
        let mid = (self.lo + self.hi) * 0.5;
        let (first_hi, second_lo) = if across_u {
            (Vec2::new(mid.x, self.hi.y), Vec2::new(mid.x, self.lo.y))
        } else {
            (Vec2::new(self.hi.x, mid.y), Vec2::new(self.lo.x, mid.y))
        };
        let half = |lo: Vec2, hi: Vec2, net: Vec<Homogeneous>| Bezier {
            lo,
            hi,
            rows: self.rows,
            cols: self.cols,
            net,
        };
        [half(self.lo, first_hi, first), half(second_lo, self.hi, second)]
    }

    /// The patch with each control point, in homogeneous coordinates, put through a linear map:
    /// the same piece of surface in other coordinates.
    pub fn mapped(&self, f: impl Fn(Homogeneous) -> Homogeneous) -> Bezier {
        Bezier {
            lo: self.lo,
            hi: self.hi,
            rows: self.rows,
            cols: self.cols,
            net: self.net.iter().map(|&q| f(q)).collect(),
        }
    }

    /// The control points.
    pub fn points(&self) -> &[Homogeneous] {
        &self.net
    }

    /// The least and greatest slope of homogeneous coordinate `c` across u and across v, each
    /// over the patch's own unit square: its derivative is a Bézier patch whose control values
    /// are the steps between neighbouring control points, so it lies between their extremes.
    pub fn slopes(&self, c: usize) -> [(f64, f64); 2] {
        // A coordinate past the fourth is no number.
        debug_assert!(c < 4, "homogeneous coordinate {c} is one of four");
        let coordinate = |q: &Homogeneous| q.get(c).copied().unwrap_or(f64::NAN);
        let span = |values: &mut dyn Iterator<Item = f64>| {
            values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)))
        };
        let (pu, pv) = ((self.rows - 1) as f64, (self.cols - 1) as f64);
        let rows = self.net_rows();
        let across_u = span(&mut rows.clone().zip(rows.clone().skip(1)).flat_map(|(row, next)| {
            row.iter().zip(next).map(|(a, b)| pu * (coordinate(b) - coordinate(a)))
        }));
        let across_v =
            span(&mut rows.flat_map(|row| {
                neighbours(row).map(|(a, b)| pv * (coordinate(b) - coordinate(a)))
            }));
        [across_u, across_v]
    }

    /// The patch at `(s, t)` of its own unit square, and its slopes across u and across v there,
    /// all in homogeneous coordinates (de Casteljau).
    pub fn at(&self, s: f64, t: f64) -> [Homogeneous; 3] {
        // A line of control points down to its last two, which give the value and the slope.
        let fold = |mut level: Vec<Homogeneous>, x: f64| -> (Homogeneous, Homogeneous) {
            let n = level.len().saturating_sub(1) as f64;
            while level.len() > 2 {
                level = neighbours(&level).map(|(&a, &b)| mix(a, b, x)).collect();
            }
            match *level.as_slice() {
                [only] => (only, [0.0; 4]),
                [a, b] => {
                    let mut slope = b;
                    for (d, a) in slope.iter_mut().zip(a) {
                        *d = n * (*d - a);
                    }
                    (mix(a, b, x), slope)
                }
                _ => ([0.0; 4], [0.0; 4]),
            }
        };
        let along: Vec<(Homogeneous, Homogeneous)> =
            self.net_rows().map(|row| fold(row.to_vec(), t)).collect();
        let (value, across_u) = fold(along.iter().map(|r| r.0).collect(), s);
        let (across_v, _) = fold(along.iter().map(|r| r.1).collect(), s);
        [value, across_u, across_v]
    }

    /// Whether the patch runs further across u than across v, by its control points' spread.
    pub fn longer_in_u(&self) -> bool {
        debug_assert!(!self.net.is_empty(), "a patch has control points");
        let corners = |row: &[Homogeneous]| Some((cartesian(row.first()?), cartesian(row.last()?)));
        let mut rows = self.net_rows();
        let (Some((first_start, first_end)), Some((last_start, last_end))) = (
            rows.next().and_then(corners),
            rows.next_back().and_then(corners).or_else(|| {
                // A net of one row: its first row is its last.
                self.net_rows().next().and_then(corners)
            }),
        ) else {
            return false;
        };
        let across_u = first_start.dist(last_start) + first_end.dist(last_end);
        let across_v = first_start.dist(first_end) + last_start.dist(last_end);
        across_u >= across_v
    }
}

/// The matrix that turns a polynomial's values at `0, 1/n, …, 1` into its Bézier coefficients.
pub(super) fn bernstein_from_values(n: usize) -> Vec<Vec<f64>> {
    let binomial = |k: usize| (0..k).fold(1.0, |b, i| b * (n - i) as f64 / (i + 1) as f64);
    let size = n + 1;
    // Each row: the Bernstein functions at one node, then that row of the identity.
    let mut m: Vec<Vec<f64>> = (0..size)
        .map(|j| {
            let s = j as f64 / n.max(1) as f64;
            let mut row: Vec<f64> = (0..size)
                .map(|k| binomial(k) * s.powi(k as i32) * (1.0 - s).powi((n - k) as i32))
                .collect();
            row.extend((0..size).map(|k| if k == j { 1.0 } else { 0.0 }));
            row
        })
        .collect();
    debug_assert!(m.iter().all(|row| row.len() == 2 * size), "each row is values then identity");
    // Gauss–Jordan with partial pivoting.
    for col in 0..size {
        let magnitude = |m: &[Vec<f64>], r: usize| {
            m.get(r).and_then(|row| row.get(col)).map_or(0.0, |x| x.abs())
        };
        let pivot = (col..size)
            .max_by(|&a, &b| magnitude(&m, a).total_cmp(&magnitude(&m, b)))
            .unwrap_or(col);
        m.swap(col, pivot);
        let Some(by) = m.get(col).cloned() else {
            continue;
        };
        let Some(&by_col) = by.get(col) else {
            continue;
        };
        for (row, values) in m.iter_mut().enumerate() {
            if row == col {
                continue;
            }
            let f = values.get(col).copied().unwrap_or(0.0) / by_col;
            for (x, p) in values.iter_mut().zip(&by) {
                *x -= f * p;
            }
        }
    }
    // The right half, each row over its diagonal entry.
    m.iter()
        .enumerate()
        .map(|(i, row)| {
            let diagonal = row.get(i).copied().unwrap_or(f64::NAN);
            row.iter().skip(size).map(|&x| x / diagonal).collect()
        })
        .collect()
}

/// Inserts knot `t` once into a curve of homogeneous control points (Boehm's algorithm).
pub(super) fn insert_knot(knots: &mut Vec<f64>, p: usize, pts: &mut Vec<Homogeneous>, t: f64) {
    let n = pts.len();
    let mut k = p;
    while k + 1 < n && knots.get(k + 1).is_some_and(|&next| next <= t) {
        k += 1;
    }
    // Control points `k − p + 1 ..= k` become blends of each with the one before it.
    let mut out = Vec::with_capacity(n + 1);
    out.extend(pts.iter().take(k - p + 1));
    let starts = knots.iter().skip(k - p + 1);
    let ends = knots.iter().skip(k + 1);
    let blended = pts.iter().skip(k - p + 1).zip(pts.iter().skip(k - p));
    for ((&lo, &hi), (q, r)) in starts.zip(ends).zip(blended).take(p) {
        let den = hi - lo;
        let a = if den > 0.0 { (t - lo) / den } else { 0.0 };
        let mut point = *q;
        for (x, r) in point.iter_mut().zip(r) {
            *x = a * *x + (1.0 - a) * r;
        }
        out.push(point);
    }
    out.extend(pts.iter().skip(k));
    *pts = out;
    knots.insert(k + 1, t);
}
