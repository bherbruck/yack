// yack: the seed points that start a spline surface's global projection, and the nearest of them.
//! The seed points that start a spline surface's global projection, and the four nearest of them
//! to a point. Each run of seeds in layout order keeps a box, and a run whose box lies no nearer
//! than the fourth seed held so far is passed over. A seed is taken only when it is strictly
//! nearer than that fourth, and none in such a run can be, so the seeds chosen and their order are
//! exactly those a scan of every seed chooses.

use crate::math::{Point3, Vec2, Vec3};

/// The seeds in one run, which share a box.
const RUN: usize = 16;

/// The most seeds a global projection searches from, nearest first, when none of the four nearest
/// settles near the surface.
const SEARCHED: usize = 16;

#[derive(Clone, Debug)]
pub(super) struct Seeds {
    /// Each seed's parameters and point, row by row along `u`.
    pub(super) pts: Vec<(Vec2, Point3)>,
    /// The spacing the seeds were laid out at along the surface.
    pub(super) spacing: f64,
    /// Each run's lowest and highest coordinates.
    boxes: Vec<(Point3, Point3)>,
}

impl Seeds {
    pub(super) fn new(pts: Vec<(Vec2, Point3)>, spacing: f64) -> Seeds {
        let boxes = pts
            .chunks(RUN)
            .map(|run| {
                // A seed with a coordinate that is not a number is left out: it is never nearer.
                let (inf, ninf) = (f64::INFINITY, f64::NEG_INFINITY);
                let empty = (Vec3::new(inf, inf, inf), Vec3::new(ninf, ninf, ninf));
                run.iter().fold(empty, |(lo, hi), &(_, q)| {
                    (
                        Vec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z)),
                        Vec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z)),
                    )
                })
            })
            .collect();
        Seeds { pts, spacing, boxes }
    }

    /// The `N` nearest seeds to `p` at distinct points, nearest first, with their squared
    /// distances; unfilled places are infinitely far. A pole's row of seeds is one point, and four
    /// of them would be one search. A mapped surface's pole row agrees only to rounding, so one
    /// point is any within a millionth of the seeds' spacing.
    pub(super) fn nearest<const N: usize>(&self, p: Point3) -> [(f64, Vec2); N] {
        let same = (1e-6 * self.spacing).powi(2);
        let mut near = [(f64::INFINITY, Vec2::ZERO); N];
        let mut at = [Point3::ZERO; N];
        let Some(last) = N.checked_sub(1) else {
            return near;
        };
        for (run, &(lo, hi)) in self.pts.chunks(RUN).zip(&self.boxes) {
            // Each coordinate gap to the box is no larger than the seed's own in floating point,
            // as subtraction, squaring and addition round monotonically: the box's squared
            // distance, summed in the order `norm2` sums, bounds every seed's from below.
            let gap = |x: f64, l: f64, h: f64| {
                if x < l {
                    l - x
                } else if x > h {
                    x - h
                } else {
                    0.0
                }
            };
            let g = Vec3::new(gap(p.x, lo.x, hi.x), gap(p.y, lo.y, hi.y), gap(p.z, lo.z, hi.z));
            if g.norm2() >= near[last].0 {
                continue;
            }
            for &(uv, q) in run {
                let d2 = (q - p).norm2();
                let taken = |i: usize| near[i].0.is_finite() && (at[i] - q).norm2() <= same;
                if d2 < near[last].0 && !(0..N).any(taken) {
                    let mut i = last;
                    while i > 0 && near[i - 1].0 > d2 {
                        near[i] = near[i - 1];
                        at[i] = at[i - 1];
                        i -= 1;
                    }
                    near[i] = (d2, uv);
                    at[i] = q;
                }
            }
        }
        near
    }
}

impl super::NurbsSurface {
    /// The parameters of the surface point nearest `p`, periodic parameters in the knot domain.
    pub fn project(&self, p: Point3) -> Vec2 {
        let near = self.seed_grid().nearest::<4>(p);
        let mut best = (f64::INFINITY, Vec2::ZERO);
        if self.search_from(p, &near, &[], &mut best) {
            return best.1;
        }
        // Four seeds settle a point of a surface whose far parts keep a seed spacing away. A
        // squashed or sheared one can bring a far part nearer than all four, and every search from
        // them stops short of the point (conformance stretches-369ef83ce9d5f331, a sheared cone's
        // narrow end beside its wide rim): the next nearest seeds are searched too.
        let more = self.seed_grid().nearest::<SEARCHED>(p);
        self.search_from(p, &more, &near, &mut best);
        best.1
    }

    /// Searches from each seed in turn but those in `done`, keeping the nearest foot in `best`;
    /// whether one settled so near the surface that it is the nearest.
    fn search_from(&self, p: Point3, seeds: &[(f64, Vec2)], done: &[(f64, Vec2)], best: &mut (f64, Vec2)) -> bool {
        for &(d2, seed) in seeds {
            if d2.is_infinite() {
                break;
            }
            if done.iter().any(|s| s.1 == seed) {
                continue;
            }
            let (uv, d, settled) = self.search(p, seed, None);
            if d < best.0 {
                *best = (d, uv);
            }
            // A foot this near the surface is the nearest, as `project_near` takes it: the seed
            // grid resolves the surface, so no other sheet comes that near it. A search that
            // stopped short of a foot says nothing of the kind.
            if settled && best.0 <= self.near_radius() {
                return true;
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every seed measured, in layout order: what [`Seeds::nearest`] must choose exactly.
    fn scan(s: &Seeds, p: Point3) -> [(f64, Vec2); 4] {
        let same = (1e-6 * s.spacing).powi(2); // tol: the reference scan's, as nearest's
        let mut near = [(f64::INFINITY, Vec2::ZERO); 4];
        let mut at = [Point3::ZERO; 4];
        for &(uv, q) in &s.pts {
            let d2 = (q - p).norm2();
            let taken = |i: usize| near[i].0.is_finite() && (at[i] - q).norm2() <= same;
            if d2 < near[3].0 && !(0..4).any(taken) {
                let mut i = 3;
                while i > 0 && near[i - 1].0 > d2 {
                    near[i] = near[i - 1];
                    at[i] = at[i - 1];
                    i -= 1;
                }
                near[i] = (d2, uv);
                at[i] = q;
            }
        }
        near
    }

    #[test]
    fn passing_over_runs_chooses_what_a_full_scan_does() {
        // A wavy sheet with a pole row (every seed of the first row one point, to rounding) and a
        // run left short at the end.
        let mut pts = Vec::new();
        for i in 0..37 {
            for j in 0..23 {
                let (u, v) = (i as f64 / 36.0, j as f64 / 22.0);
                let r = if i == 0 { 1e-17 * j as f64 } else { u }; // tol: a pole to rounding
                let a = v * std::f64::consts::TAU;
                pts.push((Vec2::new(u, v), Vec3::new(r * a.cos(), r * a.sin(), (3.0 * u).sin())));
            }
        }
        let s = Seeds::new(pts, 0.05);
        let mut x = 12345u64;
        let mut next = || {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (x >> 11) as f64 / (1u64 << 53) as f64 * 3.0 - 1.5
        };
        for _ in 0..2000 {
            let p = Vec3::new(next(), next(), next());
            let (a, b) = (s.nearest::<4>(p), scan(&s, p));
            for k in 0..4 {
                assert_eq!(a[k].0.to_bits(), b[k].0.to_bits(), "{p:?}");
                assert_eq!((a[k].1.x.to_bits(), a[k].1.y.to_bits()), (b[k].1.x.to_bits(), b[k].1.y.to_bits()));
            }
        }
        // On a seed itself, and at the pole.
        for &(_, q) in s.pts.iter().step_by(7) {
            assert!(s.nearest::<4>(q)[0].0 <= 1e-30); // tol: a pole row's seeds are one point
            assert_eq!(s.nearest::<4>(q).map(|n| n.0.to_bits()), scan(&s, q).map(|n| n.0.to_bits()));
        }
        assert_eq!(s.nearest::<4>(Vec3::ZERO).map(|n| n.0.to_bits()), scan(&s, Vec3::ZERO).map(|n| n.0.to_bits()));
    }
}
