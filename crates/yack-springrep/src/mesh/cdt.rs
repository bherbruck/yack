// yack: A constrained Delaunay triangulation in the plane: points go in by location and Lawson flips,
// Adapted from springrep@a03e8a1 src/mesh/cdt.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! A constrained Delaunay triangulation in the plane: points go in by location and Lawson flips,
//! constraint segments by Sloan's flips, and every edge that is not a constraint is kept locally
//! Delaunay. The first four vertices are a box round everything, so every inserted point lands
//! inside a triangle.

use std::collections::VecDeque;

use super::predicates::{in_circle, orient};
use crate::math::Vec2;

pub(super) const NONE: u32 = u32::MAX;

/// The box corners are vertices `0..BOX`.
pub(super) const BOX: u32 = 4;

/// Where a point falls in the triangulation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Located {
    Inside(u32),
    /// On edge `k` of the triangle.
    OnEdge(u32, usize),
    Vertex(u32),
    /// The walk would cross a constraint, or left the triangulation.
    Blocked,
}

#[derive(Clone, Debug)]
pub(super) struct Cdt {
    pub p: Vec<Vec2>,
    /// Counter-clockwise; edge `k` runs from corner `k` to corner `k + 1`.
    pub tri: Vec<[u32; 3]>,
    /// The triangle across each edge, or [`NONE`] on the box.
    pub nbr: Vec<[u32; 3]>,
    pub fixed: Vec<[bool; 3]>,
    /// One triangle at each vertex.
    vt: Vec<u32>,
    hint: u32,
}

fn nx(k: usize) -> usize {
    (k + 1) % 3
}

fn pv(k: usize) -> usize {
    (k + 2) % 3
}

impl Cdt {
    /// An empty triangulation of a box holding `lo..hi` well inside it.
    pub(super) fn new(lo: Vec2, hi: Vec2) -> Cdt {
        let span = (hi.x - lo.x).max(hi.y - lo.y).max(1e-9);
        let (lo, hi) = (lo - Vec2::new(span, span), hi + Vec2::new(span, span));
        Cdt {
            p: vec![lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)],
            tri: vec![[0, 1, 2], [0, 2, 3]],
            nbr: vec![[NONE, NONE, 1], [0, NONE, NONE]],
            fixed: vec![[false; 3]; 2],
            vt: vec![0, 0, 0, 1],
            hint: 0,
        }
    }

    fn at(&self, v: u32) -> Vec2 {
        self.p[v as usize]
    }

    fn corner(&self, t: u32, v: u32) -> Option<usize> {
        self.tri[t as usize].iter().position(|&x| x == v)
    }

    fn set(&mut self, t: u32, tri: [u32; 3]) {
        if t as usize == self.tri.len() {
            self.tri.push(tri);
            self.nbr.push([NONE; 3]);
            self.fixed.push([false; 3]);
        } else {
            self.tri[t as usize] = tri;
        }
        for v in tri {
            self.vt[v as usize] = t;
        }
    }

    /// Makes edge `k` of `t` face `n` (whose edge runs the other way), with the given flag.
    fn link(&mut self, t: u32, k: usize, n: u32, fixed: bool) {
        self.nbr[t as usize][k] = n;
        self.fixed[t as usize][k] = fixed;
        if n == NONE {
            return;
        }
        let (a, b) = (self.tri[t as usize][k], self.tri[t as usize][nx(k)]);
        let back = (0..3)
            .find(|&j| self.tri[n as usize][j] == b && self.tri[n as usize][nx(j)] == a)
            .expect("neighbour shares the edge");
        self.nbr[n as usize][back] = t;
        self.fixed[n as usize][back] = fixed;
    }

    /// The triangle and edge index of the directed edge `a → b`, if it exists.
    pub(super) fn find_edge(&self, a: u32, b: u32) -> Option<(u32, usize)> {
        self.around(a)
            .into_iter()
            .find_map(|(t, i)| (self.tri[t as usize][nx(i)] == b).then_some((t, i)))
    }

    /// Every triangle at `v`, with `v`'s corner index in it.
    pub(super) fn around(&self, v: u32) -> Vec<(u32, usize)> {
        let start = self.vt[v as usize];
        let mut out = Vec::new();
        let mut t = start;
        // Clockwise: across the edge leaving `v`.
        loop {
            let i = self.corner(t, v).expect("vertex triangle holds it");
            out.push((t, i));
            let n = self.nbr[t as usize][i];
            if n == NONE {
                break;
            }
            if n == start {
                return out;
            }
            t = n;
        }
        // Open fan on the box: go the other way from the start.
        let mut t = start;
        loop {
            let i = self.corner(t, v).expect("vertex triangle holds it");
            let n = self.nbr[t as usize][pv(i)];
            if n == NONE {
                return out;
            }
            t = n;
            let j = self.corner(t, v).expect("fan triangle holds it");
            out.push((t, j));
        }
    }

    /// Walks from `start` to the triangle holding `q`, never across a constraint.
    pub(super) fn locate(&self, start: u32, q: Vec2) -> Located {
        // A point far outside, such as a sliver's circumcentre at 1e306, overflows the orientation
        // test to NaN, which would read as inside whatever triangle the walk starts from.
        let (lo, hi) = (self.p[0], self.p[2]);
        if !(q.x > lo.x && q.x < hi.x && q.y > lo.y && q.y < hi.y) {
            return Located::Blocked;
        }
        let mut t = if (start as usize) < self.tri.len() {
            start
        } else {
            0
        };
        let cap = 4 * self.tri.len() + 64;
        for step in 0..cap {
            let off = step % 3;
            let mut moved = false;
            for i in 0..3 {
                let k = (i + off) % 3;
                let [a, b] = [self.tri[t as usize][k], self.tri[t as usize][nx(k)]];
                if orient(self.at(a), self.at(b), q) < 0.0 {
                    let n = self.nbr[t as usize][k];
                    if n == NONE || self.fixed[t as usize][k] {
                        return Located::Blocked;
                    }
                    t = n;
                    moved = true;
                    break;
                }
            }
            if !moved {
                return self.classify_in(t, q);
            }
        }
        Located::Blocked
    }

    fn classify_in(&self, t: u32, q: Vec2) -> Located {
        let tri = self.tri[t as usize];
        let o: Vec<f64> = (0..3)
            .map(|k| orient(self.at(tri[k]), self.at(tri[nx(k)]), q))
            .collect();
        let zeros: Vec<usize> = (0..3).filter(|&k| o[k] == 0.0).collect();
        match zeros.len() {
            0 => Located::Inside(t),
            1 => Located::OnEdge(t, zeros[0]),
            // On two edges: the corner they share.
            _ => {
                let (k1, k2) = (zeros[0], zeros[1]);
                Located::Vertex(if k2 == nx(k1) { tri[k2] } else { tri[k1] })
            }
        }
    }

    /// Adds a vertex at `q` (inside the box), keeping the triangulation constrained Delaunay. See
    /// [`Cdt::insert_at`] for what it returns.
    pub(super) fn insert(&mut self, q: Vec2, start: u32) -> Result<u32, u32> {
        let loc = self.locate(start, q);
        self.insert_at(q, loc)
    }

    /// As [`Cdt::insert`] with the location already known. `Err` holds the vertex already at `q`,
    /// or [`NONE`] when the point cannot go in.
    pub(super) fn insert_at(&mut self, q: Vec2, loc: Located) -> Result<u32, u32> {
        let v = self.p.len() as u32;
        match loc {
            Located::Blocked => return Err(NONE),
            Located::Vertex(x) => return Err(x),
            Located::OnEdge(t, k) => {
                if self.fixed[t as usize][k] || self.nbr[t as usize][k] == NONE {
                    return Err(NONE);
                }
                self.p.push(q);
                self.vt.push(t);
                self.split_edge(t, k, v);
            }
            Located::Inside(t) => {
                self.p.push(q);
                self.vt.push(t);
                self.split_triangle(t, v);
            }
        }
        self.legalize_around(v);
        self.hint = self.vt[v as usize];
        Ok(v)
    }

    /// Adds a vertex that the caller links in itself with [`Cdt::split_edge`].
    pub(super) fn push_point(&mut self, q: Vec2, t: u32) {
        self.p.push(q);
        self.vt.push(t);
    }

    pub(super) fn hint(&self) -> u32 {
        self.hint
    }

    fn split_triangle(&mut self, t: u32, v: u32) {
        let [a, b, c] = self.tri[t as usize];
        let n = self.nbr[t as usize];
        let f = self.fixed[t as usize];
        let t1 = self.tri.len() as u32;
        let t2 = t1 + 1;
        self.set(t, [a, b, v]);
        self.set(t1, [b, c, v]);
        self.set(t2, [c, a, v]);
        self.link(t, 0, n[0], f[0]);
        self.link(t1, 0, n[1], f[1]);
        self.link(t2, 0, n[2], f[2]);
        self.link(t, 1, t1, false);
        self.link(t, 2, t2, false);
        self.link(t1, 1, t2, false);
    }

    /// Splits edge `k` of `t` at the new vertex `v`, which lies on it.
    pub(super) fn split_edge(&mut self, t: u32, k: usize, v: u32) {
        let ti = t as usize;
        let (a, b, c) = (self.tri[ti][k], self.tri[ti][nx(k)], self.tri[ti][pv(k)]);
        let (n_bc, f_bc) = (self.nbr[ti][nx(k)], self.fixed[ti][nx(k)]);
        let (n_ca, f_ca) = (self.nbr[ti][pv(k)], self.fixed[ti][pv(k)]);
        let fab = self.fixed[ti][k];
        let u = self.nbr[ti][k];
        let ui = u as usize;
        let j = (0..3)
            .find(|&j| self.tri[ui][j] == b && self.tri[ui][nx(j)] == a)
            .expect("neighbour shares the edge");
        let d = self.tri[ui][pv(j)];
        let (n_ad, f_ad) = (self.nbr[ui][nx(j)], self.fixed[ui][nx(j)]);
        let (n_db, f_db) = (self.nbr[ui][pv(j)], self.fixed[ui][pv(j)]);
        let t1 = self.tri.len() as u32;
        let t2 = t1 + 1;
        self.set(t, [c, a, v]);
        self.set(t1, [b, c, v]);
        self.set(u, [a, d, v]);
        self.set(t2, [d, b, v]);
        self.link(t, 0, n_ca, f_ca);
        self.link(t1, 0, n_bc, f_bc);
        self.link(u, 0, n_ad, f_ad);
        self.link(t2, 0, n_db, f_db);
        self.link(t, 1, u, fab);
        self.link(t, 2, t1, false);
        self.link(t1, 2, t2, fab);
        self.link(u, 1, t2, false);
    }

    /// Flips edge `k` of `t` when the two triangles that would replace it are both
    /// counter-clockwise; returns the two triangles, or `None` when the quad is not convex.
    pub(super) fn flip(&mut self, t: u32, k: usize) -> Option<(u32, u32)> {
        let ti = t as usize;
        let (a, b, c) = (self.tri[ti][k], self.tri[ti][nx(k)], self.tri[ti][pv(k)]);
        let u = self.nbr[ti][k];
        if u == NONE || self.fixed[ti][k] {
            return None;
        }
        let ui = u as usize;
        let j = (0..3)
            .find(|&j| self.tri[ui][j] == b && self.tri[ui][nx(j)] == a)
            .expect("neighbour shares the edge");
        let d = self.tri[ui][pv(j)];
        if orient(self.at(c), self.at(a), self.at(d)) <= 0.0
            || orient(self.at(d), self.at(b), self.at(c)) <= 0.0
        {
            return None;
        }
        let (n_ca, f_ca) = (self.nbr[ti][pv(k)], self.fixed[ti][pv(k)]);
        let (n_bc, f_bc) = (self.nbr[ti][nx(k)], self.fixed[ti][nx(k)]);
        let (n_ad, f_ad) = (self.nbr[ui][nx(j)], self.fixed[ui][nx(j)]);
        let (n_db, f_db) = (self.nbr[ui][pv(j)], self.fixed[ui][pv(j)]);
        self.set(t, [c, a, d]);
        self.set(u, [d, b, c]);
        self.link(t, 0, n_ca, f_ca);
        self.link(t, 1, n_ad, f_ad);
        self.link(u, 0, n_db, f_db);
        self.link(u, 1, n_bc, f_bc);
        self.link(t, 2, u, false);
        Some((t, u))
    }

    /// Whether edge `k` of `t` should be flipped to restore the Delaunay property.
    fn illegal(&self, t: u32, k: usize) -> bool {
        let ti = t as usize;
        let u = self.nbr[ti][k];
        if u == NONE || self.fixed[ti][k] {
            return false;
        }
        let (a, b, c) = (self.tri[ti][k], self.tri[ti][nx(k)], self.tri[ti][pv(k)]);
        let d = self.tri[u as usize]
            .iter()
            .copied()
            .find(|&x| x != a && x != b)
            .expect("neighbour has a third corner");
        in_circle(self.at(a), self.at(b), self.at(c), self.at(d))
    }

    /// Lawson flips of the edges opposite a new vertex, spreading as they flip.
    fn legalize_around(&mut self, v: u32) {
        let mut stack: Vec<u32> = self.around(v).into_iter().map(|(t, _)| t).collect();
        let mut budget = 64 + 16 * stack.len();
        while let Some(t) = stack.pop() {
            if budget == 0 {
                break;
            }
            budget -= 1;
            let Some(i) = self.corner(t, v) else { continue };
            let k = nx(i);
            if self.illegal(t, k)
                && let Some((t1, t2)) = self.flip(t, k)
            {
                budget += 2;
                stack.push(t1);
                stack.push(t2);
            }
        }
    }

    /// Flips every illegal edge until none is left, or until the work cap.
    pub(super) fn legalize_all(&mut self) {
        let mut stack: Vec<(u32, usize)> = (0..self.tri.len() as u32)
            .flat_map(|t| (0..3).map(move |k| (t, k)))
            .collect();
        let mut budget = 20 * stack.len() + 64;
        while let Some((t, k)) = stack.pop() {
            if budget == 0 {
                break;
            }
            budget -= 1;
            if self.illegal(t, k)
                && let Some((t1, t2)) = self.flip(t, k)
            {
                for x in [t1, t2] {
                    for j in 0..3 {
                        stack.push((x, j));
                    }
                }
            }
        }
    }

    /// Makes `a — b` an edge of the triangulation and marks it a constraint. Fails when the
    /// segment runs through another vertex or crosses another constraint.
    pub(super) fn insert_segment(&mut self, a: u32, b: u32) -> Result<(), &'static str> {
        if a == b {
            return Ok(());
        }
        let (pa, pb) = (self.at(a), self.at(b));
        let mut first = None;
        for (t, i) in self.around(a) {
            let tri = self.tri[t as usize];
            let (c1, c2) = (tri[nx(i)], tri[pv(i)]);
            if c1 == b {
                self.mark(t, i);
                return Ok(());
            }
            if c2 == b {
                self.mark(t, pv(i));
                return Ok(());
            }
            let (o1, o2) = (orient(pa, self.at(c1), pb), orient(pa, self.at(c2), pb));
            for (o, c) in [(o1, c1), (o2, c2)] {
                if o == 0.0 && (self.at(c) - pa).dot(pb - pa) > 0.0 {
                    return Err("a face boundary runs through another boundary point");
                }
            }
            if o1 > 0.0 && o2 < 0.0 {
                first = Some((t, c1, c2));
                break;
            }
        }
        let Some((mut t, mut r, mut l)) = first else {
            return Err("a face boundary segment has no way out of its start");
        };
        let mut crossed = VecDeque::new();
        loop {
            let ti = t as usize;
            let k = (0..3)
                .find(|&k| self.tri[ti][k] == r && self.tri[ti][nx(k)] == l)
                .expect("walk keeps the crossed edge");
            if self.fixed[ti][k] {
                return Err("face boundaries cross");
            }
            crossed.push_back((r, l));
            let n = self.nbr[ti][k];
            if n == NONE {
                return Err("a face boundary segment leaves the triangulation");
            }
            let v = self.tri[n as usize]
                .iter()
                .copied()
                .find(|&x| x != r && x != l)
                .expect("neighbour has a third corner");
            if v == b {
                break;
            }
            let o = orient(pa, pb, self.at(v));
            if o == 0.0 {
                return Err("a face boundary runs through another boundary point");
            }
            if o > 0.0 {
                l = v;
            } else {
                r = v;
            }
            t = n;
            if crossed.len() > self.tri.len() {
                return Err("a face boundary segment walk does not end");
            }
        }
        let cap = 64 + 64 * crossed.len() * crossed.len().min(4096);
        let mut iters = 0;
        while let Some((u, w)) = crossed.pop_front() {
            iters += 1;
            if iters > cap {
                return Err("a face boundary segment cannot be recovered");
            }
            let Some((t, k)) = self.find_edge(u, w).or_else(|| self.find_edge(w, u)) else {
                return Err("a crossed edge went missing");
            };
            let c = self.tri[t as usize][pv(k)];
            match self.flip(t, k) {
                None => crossed.push_back((u, w)),
                Some(_) => {
                    let d = self.tri[t as usize][2];
                    let (oc, od) = (orient(pa, pb, self.at(c)), orient(pa, pb, self.at(d)));
                    let ends = c == a || c == b || d == a || d == b;
                    if !ends && ((oc > 0.0 && od < 0.0) || (oc < 0.0 && od > 0.0)) {
                        crossed.push_back((c, d));
                    }
                }
            }
        }
        match self.find_edge(a, b) {
            Some((t, k)) => {
                self.mark(t, k);
                Ok(())
            }
            None => Err("a face boundary segment was not recovered"),
        }
    }

    fn mark(&mut self, t: u32, k: usize) {
        let n = self.nbr[t as usize][k];
        self.link(t, k, n, true);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_mesh(c: &Cdt) {
        for t in 0..c.tri.len() {
            let [a, b, d] = c.tri[t];
            assert!(orient(c.p[a as usize], c.p[b as usize], c.p[d as usize]) > 0.0);
            for k in 0..3 {
                let n = c.nbr[t][k];
                if n != NONE {
                    assert!(c.nbr[n as usize].contains(&(t as u32)));
                }
            }
        }
    }

    #[test]
    fn a_point_far_outside_the_box_is_blocked() {
        let mut c = Cdt::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0));
        for q in [
            Vec2::new(0.2, 0.2),
            Vec2::new(0.8, 0.3),
            Vec2::new(0.5, 0.9),
        ] {
            let h = c.hint();
            c.insert(q, h).unwrap();
        }
        // A sliver's circumcentre: the orientation test overflows to NaN this far out.
        let far = Vec2::new(1e308, 1e308);
        assert_eq!(c.locate(c.hint(), far), Located::Blocked);
        assert!(c.insert(far, c.hint()).is_err());
        check_mesh(&c);
    }

    #[test]
    fn grid_points_and_a_diagonal_constraint() {
        let mut c = Cdt::new(Vec2::new(0.0, 0.0), Vec2::new(4.0, 4.0));
        let mut ids = Vec::new();
        for i in 0..5 {
            for j in 0..5 {
                let h = c.hint();
                ids.push(c.insert(Vec2::new(i as f64, j as f64), h).unwrap());
            }
        }
        check_mesh(&c);
        // Two points collinear with a segment between them: the segment is refused.
        assert!(c.insert_segment(ids[0], ids[24]).is_err());
        let mut c2 = c.clone();
        c2.insert_segment(ids[1], ids[22]).unwrap();
        check_mesh(&c2);
        assert!(c2.find_edge(ids[1], ids[22]).is_some() || c2.find_edge(ids[22], ids[1]).is_some());
        c2.legalize_all();
        check_mesh(&c2);
        let (t, k) = c2
            .find_edge(ids[1], ids[22])
            .or_else(|| c2.find_edge(ids[22], ids[1]))
            .unwrap();
        assert!(c2.fixed[t as usize][k]);
    }
}
