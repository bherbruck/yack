// yack: A constrained Delaunay triangulation in the plane: points go in by location and Lawson flips,
// Adapted from springrep@a03e8a1 src/mesh/cdt.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/mesh/cdt.rs to yack-mesh (M3 W6); paths and visibility only;
// hardening (w-h1): one `Tri` per triangle in place of three parallel arrays, read and written
// through checked accessors; a broken segment walk is an error.
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

/// One triangle: its corners counter-clockwise, edge `k` running from corner `k` to corner `k + 1`;
/// the triangle across each edge, or [`NONE`] on the box; and which edges are constraints.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Tri {
    pub corners: [u32; 3],
    pub across: [u32; 3],
    pub fixed: [bool; 3],
}

impl Tri {
    /// What a missing triangle reads as: no corners, no neighbours, no constraints.
    const MISSING: Tri = Tri { corners: [NONE; 3], across: [NONE; 3], fixed: [false; 3] };

    fn new(corners: [u32; 3]) -> Tri {
        Tri { corners, across: [NONE; 3], fixed: [false; 3] }
    }

    /// Corner `k`.
    pub fn corner(&self, k: usize) -> u32 {
        third(self.corners, k)
    }

    /// The triangle across edge `k`.
    pub fn across(&self, k: usize) -> u32 {
        third(self.across, k)
    }

    /// Whether edge `k` is a constraint.
    pub fn is_fixed(&self, k: usize) -> bool {
        third(self.fixed, k)
    }

    /// The edge running `a → b`, if this triangle has it.
    fn edge(&self, a: u32, b: u32) -> Option<usize> {
        (0..3).find(|&k| self.corner(k) == a && self.corner(nx(k)) == b)
    }

    /// The corner that is neither `a` nor `b`.
    fn other(&self, a: u32, b: u32) -> Option<u32> {
        self.corners.iter().copied().find(|&x| x != a && x != b)
    }
}

/// Element `k` of a triangle's three, for `k` in `0..3`.
pub(super) fn third<T: Copy>([x, y, z]: [T; 3], k: usize) -> T {
    match k {
        0 => x,
        1 => y,
        _ => z,
    }
}

/// Element `k` of a triangle's three, mutably.
fn third_mut<T>(three: &mut [T; 3], k: usize) -> &mut T {
    let [x, y, z] = three;
    match k {
        0 => x,
        1 => y,
        _ => z,
    }
}

#[derive(Clone, Debug)]
pub(super) struct Cdt {
    p: Vec<Vec2>,
    tris: Vec<Tri>,
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
        let mut tris = vec![Tri::new([0, 1, 2]), Tri::new([0, 2, 3])];
        if let [first, second] = tris.as_mut_slice() {
            first.across = [NONE, NONE, 1];
            second.across = [0, NONE, NONE];
        }
        Cdt {
            p: vec![lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)],
            tris,
            vt: vec![0, 0, 0, 1],
            hint: 0,
        }
    }

    /// The number of triangles, the box's included.
    pub(super) fn triangle_count(&self) -> usize {
        self.tris.len()
    }

    /// The number of vertices, the box's included.
    pub(super) fn point_count(&self) -> usize {
        self.p.len()
    }

    /// Triangle `t`. Every id the triangulation hands out names one; a debug build checks it.
    pub(super) fn tri(&self, t: u32) -> Tri {
        let tri = self.tris.get(t as usize).copied();
        debug_assert!(tri.is_some(), "triangle {t} is past the {}", self.tris.len());
        tri.unwrap_or(Tri::MISSING)
    }

    /// Vertex `v`'s position. Every vertex id names one; a debug build checks it.
    pub(super) fn point(&self, v: u32) -> Vec2 {
        let p = self.p.get(v as usize).copied();
        debug_assert!(p.is_some(), "vertex {v} is past the {}", self.p.len());
        p.unwrap_or_default()
    }

    fn tri_mut(&mut self, t: u32) -> Option<&mut Tri> {
        self.tris.get_mut(t as usize)
    }

    /// The triangle recorded at vertex `v`.
    fn vertex_tri(&self, v: u32) -> u32 {
        self.vt.get(v as usize).copied().unwrap_or(NONE)
    }

    fn corner(&self, t: u32, v: u32) -> Option<usize> {
        self.tri(t).corners.iter().position(|&x| x == v)
    }

    fn set(&mut self, t: u32, corners: [u32; 3]) {
        if t as usize == self.tris.len() {
            self.tris.push(Tri::new(corners));
        } else if let Some(tri) = self.tri_mut(t) {
            tri.corners = corners;
        }
        for v in corners {
            if let Some(at) = self.vt.get_mut(v as usize) {
                *at = t;
            }
        }
    }

    /// Edge `k` of `t` and its neighbour's across it: its neighbour and flag.
    fn set_side(&mut self, t: u32, k: usize, n: u32, fixed: bool) {
        if let Some(tri) = self.tri_mut(t) {
            *third_mut(&mut tri.across, k) = n;
            *third_mut(&mut tri.fixed, k) = fixed;
        }
    }

    /// Makes edge `k` of `t` face `n` (whose edge runs the other way), with the given flag.
    #[allow(clippy::panic)] fn link(&mut self, t: u32, k: usize, n: u32, fixed: bool) {
        self.set_side(t, k, n, fixed);
        if n == NONE {
            return;
        }
        let tri = self.tri(t);
        let (a, b) = (tri.corner(k), tri.corner(nx(k)));
        let back = self.tri(n).edge(b, a).unwrap_or_else(|| panic!("neighbour shares the edge"));
        self.set_side(n, back, t, fixed);
    }

    /// The triangle and edge index of the directed edge `a → b`, if it exists.
    pub(super) fn find_edge(&self, a: u32, b: u32) -> Option<(u32, usize)> {
        self.around(a)
            .into_iter()
            .find_map(|(t, i)| (self.tri(t).corner(nx(i)) == b).then_some((t, i)))
    }

    /// Every triangle at `v`, with `v`'s corner index in it.
    #[allow(clippy::panic)] pub(super) fn around(&self, v: u32) -> Vec<(u32, usize)> {
        let start = self.vertex_tri(v);
        let mut out = Vec::new();
        let mut t = start;
        // Clockwise: across the edge leaving `v`.
        loop {
            let i = self.corner(t, v).unwrap_or_else(|| panic!("vertex triangle holds it"));
            out.push((t, i));
            let n = self.tri(t).across(i);
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
            let i = self.corner(t, v).unwrap_or_else(|| panic!("vertex triangle holds it"));
            let n = self.tri(t).across(pv(i));
            if n == NONE {
                return out;
            }
            t = n;
            let j = self.corner(t, v).unwrap_or_else(|| panic!("fan triangle holds it"));
            out.push((t, j));
        }
    }

    /// Walks from `start` to the triangle holding `q`, never across a constraint.
    pub(super) fn locate(&self, start: u32, q: Vec2) -> Located {
        // A point far outside, such as a sliver's circumcentre at 1e306, overflows the orientation
        // test to NaN, which would read as inside whatever triangle the walk starts from.
        let (lo, hi) = (self.point(0), self.point(2));
        if !(q.x > lo.x && q.x < hi.x && q.y > lo.y && q.y < hi.y) {
            return Located::Blocked;
        }
        let mut t = if (start as usize) < self.tris.len() {
            start
        } else {
            0
        };
        let cap = 4 * self.tris.len() + 64;
        for step in 0..cap {
            let off = step % 3;
            let mut moved = false;
            let tri = self.tri(t);
            for i in 0..3 {
                let k = (i + off) % 3;
                let [a, b] = [tri.corner(k), tri.corner(nx(k))];
                if orient(self.point(a), self.point(b), q) < 0.0 {
                    let n = tri.across(k);
                    if n == NONE || tri.is_fixed(k) {
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
        let tri = self.tri(t);
        let side = |k: usize| orient(self.point(tri.corner(k)), self.point(tri.corner(nx(k))), q);
        let o = [0, 1, 2].map(side);
        let zeros: Vec<usize> = (0..3).filter(|&k| third(o, k) == 0.0).collect();
        match zeros.as_slice() {
            [] => Located::Inside(t),
            &[k] => Located::OnEdge(t, k),
            // On two edges: the corner they share.
            &[k1, k2, ..] => {
                Located::Vertex(if k2 == nx(k1) { tri.corner(k2) } else { tri.corner(k1) })
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
                let tri = self.tri(t);
                if tri.is_fixed(k) || tri.across(k) == NONE {
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
        self.hint = self.vertex_tri(v);
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
        let Tri { corners: [a, b, c], across: n, fixed: f } = self.tri(t);
        let t1 = self.tris.len() as u32;
        let t2 = t1 + 1;
        self.set(t, [a, b, v]);
        self.set(t1, [b, c, v]);
        self.set(t2, [c, a, v]);
        let ([n0, n1, n2], [f0, f1, f2]) = (n, f);
        self.link(t, 0, n0, f0);
        self.link(t1, 0, n1, f1);
        self.link(t2, 0, n2, f2);
        self.link(t, 1, t1, false);
        self.link(t, 2, t2, false);
        self.link(t1, 1, t2, false);
    }

    /// Splits edge `k` of `t` at the new vertex `v`, which lies on it.
    #[allow(clippy::panic)] pub(super) fn split_edge(&mut self, t: u32, k: usize, v: u32) {
        let tri = self.tri(t);
        let (a, b, c) = (tri.corner(k), tri.corner(nx(k)), tri.corner(pv(k)));
        let (n_bc, f_bc) = (tri.across(nx(k)), tri.is_fixed(nx(k)));
        let (n_ca, f_ca) = (tri.across(pv(k)), tri.is_fixed(pv(k)));
        let fab = tri.is_fixed(k);
        let u = tri.across(k);
        let other = self.tri(u);
        let j = other.edge(b, a).unwrap_or_else(|| panic!("neighbour shares the edge"));
        let d = other.corner(pv(j));
        let (n_ad, f_ad) = (other.across(nx(j)), other.is_fixed(nx(j)));
        let (n_db, f_db) = (other.across(pv(j)), other.is_fixed(pv(j)));
        let t1 = self.tris.len() as u32;
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
    #[allow(clippy::panic)] pub(super) fn flip(&mut self, t: u32, k: usize) -> Option<(u32, u32)> {
        let tri = self.tri(t);
        let (a, b, c) = (tri.corner(k), tri.corner(nx(k)), tri.corner(pv(k)));
        let u = tri.across(k);
        if u == NONE || tri.is_fixed(k) {
            return None;
        }
        let other = self.tri(u);
        let j = other.edge(b, a).unwrap_or_else(|| panic!("neighbour shares the edge"));
        let d = other.corner(pv(j));
        if orient(self.point(c), self.point(a), self.point(d)) <= 0.0
            || orient(self.point(d), self.point(b), self.point(c)) <= 0.0
        {
            return None;
        }
        let (n_ca, f_ca) = (tri.across(pv(k)), tri.is_fixed(pv(k)));
        let (n_bc, f_bc) = (tri.across(nx(k)), tri.is_fixed(nx(k)));
        let (n_ad, f_ad) = (other.across(nx(j)), other.is_fixed(nx(j)));
        let (n_db, f_db) = (other.across(pv(j)), other.is_fixed(pv(j)));
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
    #[allow(clippy::panic)] fn illegal(&self, t: u32, k: usize) -> bool {
        let tri = self.tri(t);
        let u = tri.across(k);
        if u == NONE || tri.is_fixed(k) {
            return false;
        }
        let (a, b, c) = (tri.corner(k), tri.corner(nx(k)), tri.corner(pv(k)));
        let d = self.tri(u).other(a, b).unwrap_or_else(|| panic!("neighbour has a third corner"));
        in_circle(self.point(a), self.point(b), self.point(c), self.point(d))
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
        let mut stack: Vec<(u32, usize)> = (0..self.tris.len() as u32)
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
        let (pa, pb) = (self.point(a), self.point(b));
        let mut first = None;
        for (t, i) in self.around(a) {
            let tri = self.tri(t);
            let (c1, c2) = (tri.corner(nx(i)), tri.corner(pv(i)));
            if c1 == b {
                self.mark(t, i);
                return Ok(());
            }
            if c2 == b {
                self.mark(t, pv(i));
                return Ok(());
            }
            let (o1, o2) = (orient(pa, self.point(c1), pb), orient(pa, self.point(c2), pb));
            for (o, c) in [(o1, c1), (o2, c2)] {
                if o == 0.0 && (self.point(c) - pa).dot(pb - pa) > 0.0 {
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
            let tri = self.tri(t);
            let Some(k) = tri.edge(r, l) else {
                return Err("a face boundary segment walk lost its crossed edge");
            };
            if tri.is_fixed(k) {
                return Err("face boundaries cross");
            }
            crossed.push_back((r, l));
            let n = tri.across(k);
            if n == NONE {
                return Err("a face boundary segment leaves the triangulation");
            }
            let Some(v) = self.tri(n).other(r, l) else {
                return Err("a face boundary segment walk met a triangle with no third corner");
            };
            if v == b {
                break;
            }
            let o = orient(pa, pb, self.point(v));
            if o == 0.0 {
                return Err("a face boundary runs through another boundary point");
            }
            if o > 0.0 {
                l = v;
            } else {
                r = v;
            }
            t = n;
            if crossed.len() > self.tris.len() {
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
            let c = self.tri(t).corner(pv(k));
            match self.flip(t, k) {
                None => crossed.push_back((u, w)),
                Some(_) => {
                    let [.., d] = self.tri(t).corners;
                    let (oc, od) = (orient(pa, pb, self.point(c)), orient(pa, pb, self.point(d)));
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
        let n = self.tri(t).across(k);
        self.link(t, k, n, true);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_mesh(c: &Cdt) {
        for t in 0..c.triangle_count() as u32 {
            let tri = c.tri(t);
            let [a, b, d] = tri.corners;
            assert!(orient(c.point(a), c.point(b), c.point(d)) > 0.0);
            for n in tri.across {
                if n != NONE {
                    assert!(c.tri(n).across.contains(&t));
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
        assert!(c2.tri(t).is_fixed(k));
    }
}
