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
        // The runs nearest first find `N` distinct seeds within some distance at once; the scan in
        // layout order, which chooses, then passes over every run whose box lies farther than
        // that by more than a few times the distance two seeds count as one (a seed held there
        // could only stand in for one as far, never for one it chooses). A scan in layout order
        // alone holds nothing to pass runs over by until it has met `N` seeds (bench
        // swept/ring-carried-along-a-spline: a fifth of its time here).
        let gaps: Vec<f64> = self.boxes.iter().map(|&(lo, hi)| gap2(p, lo, hi)).collect();
        let mut order: Vec<usize> = (0..gaps.len()).collect();
        order.sort_by(|&i, &j| gaps[i].total_cmp(&gaps[j]));
        let runs: Vec<&[(Vec2, Point3)]> = self.pts.chunks(RUN).collect();
        let first = scan::<N>(p, same, order.iter().map(|&i| (gaps[i], runs[i])), true);
        let within = match first.last() {
            Some(&(d2, _)) if d2.is_finite() => (d2.sqrt() + 4.0 * same.sqrt()).powi(2) * (1.0 + 1e-12), // tol: rounding of the bound's root and square
            _ => f64::INFINITY,
        };
        scan::<N>(p, same, gaps.iter().copied().zip(runs.iter().copied()).filter(|&(g, _)| g <= within), false)
    }
}

/// The squared distance from `p` to the box `(lo, hi)`: each coordinate gap to the box is no
/// larger than a seed's own in floating point, as subtraction, squaring and addition round
/// monotonically, so summed in the order `norm2` sums it bounds every seed's from below.
fn gap2(p: Point3, lo: Point3, hi: Point3) -> f64 {
    let gap = |x: f64, l: f64, h: f64| {
        if x < l {
            l - x
        } else if x > h {
            x - h
        } else {
            0.0
        }
    };
    Vec3::new(gap(p.x, lo.x, hi.x), gap(p.y, lo.y, hi.y), gap(p.z, lo.z, hi.z)).norm2()
}

/// The `N` nearest of the seeds of `runs` (each with its box's squared distance) at distinct
/// points, taken in the order given: a run whose box lies no nearer than the `N`th held is passed
/// over, and with `sorted` (the runs nearest first) so is every one after it.
fn scan<'a, const N: usize>(p: Point3, same: f64, runs: impl Iterator<Item = (f64, &'a [(Vec2, Point3)])>, sorted: bool) -> [(f64, Vec2); N] {
    let mut near = [(f64::INFINITY, Vec2::ZERO); N];
    let mut at = [Point3::ZERO; N];
    let Some(last) = N.checked_sub(1) else {
        return near;
    };
    for (g, run) in runs {
        if g >= near[last].0 {
            if sorted {
                break;
            }
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

impl super::NurbsSurface {
    /// The parameters of the surface point nearest `p`, periodic parameters in the knot domain.
    pub fn project(&self, p: Point3) -> Vec2 {
        let near = self.seed_grid().nearest::<4>(p);
        let mut best = (f64::INFINITY, Vec2::ZERO);
        self.search_from(p, &near, &[], &mut best);
        best.1
    }

    /// [`NurbsSurface::project`] for a point that should lie on the surface. Four seeds settle a
    /// point of a surface whose far parts keep a seed spacing away. A squashed or sheared one can
    /// bring a far part nearer than all four, and every search from them stops short of the point
    /// (conformance stretches-369ef83ce9d5f331, a sheared cone's narrow end beside its wide rim):
    /// the next nearest seeds are searched too. A point truly off the surface (a rolling ball's
    /// centre) pays that on every call, so only a caller expecting the surface asks for it.
    pub fn project_onto(&self, p: Point3) -> Vec2 {
        let near = self.seed_grid().nearest::<4>(p);
        let mut best = (f64::INFINITY, Vec2::ZERO);
        if self.search_from(p, &near, &[], &mut best) {
            return best.1;
        }
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
