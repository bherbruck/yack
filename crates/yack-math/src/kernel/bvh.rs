// yack: A bounding volume hierarchy over boxes, and the closest points and crossings of points,
// Adapted from springrep@a03e8a1 src/mesh/bvh.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from mesh/bvh.rs to math/bvh.rs (M2); `Bvh::new` takes the triangles' positions and
// indices rather than a mesh; moved to yack_math::kernel::bvh (M3 W2', D26); paths and visibility only;
// hardening (w-h1): the four box queries share one walk, nodes and runs are read with checked
// access, and `Bvh::new` asserts its corner indices.
//! A bounding volume hierarchy over boxes, and the closest points and crossings of points,
//! segments and triangles: the search structures the thin-place probe, distances and rays share.

use super::{Aabb, Point3, Transform, Vec3};

/// A bounding volume hierarchy over boxes: a mesh's triangles, or any primitives' boxes.
pub struct Bvh {
    boxes: Vec<Aabb>,
    /// Node: its box, and either two children or a run of `order`.
    nodes: Vec<(Aabb, Node)>,
    order: Vec<usize>,
}

/// A node of a [`Bvh`]: two children by index into its nodes, or a run `lo..hi` of its order.
enum Node {
    Inner(usize, usize),
    Leaf(usize, usize),
}

/// The leaves hold at most this many boxes.
const LEAF_SIZE: usize = 4;

impl Bvh {
    /// The hierarchy over the boxes of `triangles`, each three indices into `positions`.
    ///
    /// # Panics
    ///
    /// When a corner index is past the end of `positions`. A mesh's triangles index its own
    /// positions, so such an index is the caller's bug, never a refusal (a contract panic, counted
    /// in scores/lints.txt; the firewall reports it as `Internal`).
    pub fn new(positions: &[Point3], triangles: &[[u32; 3]]) -> Bvh {
        let boxes = triangles
            .iter()
            .map(|corners| {
                let mut b = Aabb::EMPTY;
                for &i in corners {
                    let corner = positions.get(i as usize);
                    assert!(
                        corner.is_some(),
                        "Bvh::new: corner index {i} is past the {} positions",
                        positions.len()
                    );
                    if let Some(&p) = corner {
                        b.add(p);
                    }
                }
                b
            })
            .collect();
        Bvh::of_boxes(boxes)
    }

    /// The hierarchy over `boxes`, whose indices a query returns.
    pub fn of_boxes(boxes: Vec<Aabb>) -> Bvh {
        let mut bvh = Bvh {
            order: (0..boxes.len()).collect(),
            boxes,
            nodes: Vec::new(),
        };
        if !bvh.order.is_empty() {
            bvh.split(0, bvh.order.len());
        }
        bvh
    }

    /// The boxes of the run `lo..hi` of the order, as indices. Every node's run lies in the
    /// order; a debug build checks it.
    fn run(&self, lo: usize, hi: usize) -> &[usize] {
        let run = self.order.get(lo..hi);
        debug_assert!(
            run.is_some(),
            "Bvh run {lo}..{hi} is outside its {} boxes",
            self.order.len()
        );
        run.unwrap_or_default()
    }

    /// The box of index `i`. Every index in the order has one; a debug build checks it.
    fn bounds(&self, i: usize) -> &Aabb {
        let bounds = self.boxes.get(i);
        debug_assert!(bounds.is_some(), "Bvh box {i} is past its {} boxes", self.boxes.len());
        bounds.unwrap_or(&Aabb::EMPTY)
    }

    /// Adds the node over the run `lo..hi` and, past [`LEAF_SIZE`] boxes, its two halves split at
    /// the median of the box centres along the run's longest side. Returns the node's index.
    fn split(&mut self, lo: usize, hi: usize) -> usize {
        let mut bb = Aabb::EMPTY;
        for &i in self.run(lo, hi) {
            bb.union(self.bounds(i));
        }
        let at = self.nodes.len();
        self.nodes.push((bb, Node::Leaf(lo, hi)));
        if hi - lo <= LEAF_SIZE {
            return at;
        }
        let ext = bb.max - bb.min;
        let centre = |b: &Aabb| (b.min + b.max) * 0.5;
        let key: fn(Vec3) -> f64 = if ext.x >= ext.y && ext.x >= ext.z {
            |c| c.x
        } else if ext.y >= ext.z {
            |c| c.y
        } else {
            |c| c.z
        };
        let mid = (lo + hi) / 2;
        let boxes = &self.boxes;
        let along = |i: usize| boxes.get(i).map_or(0.0, |b| key(centre(b)));
        if let Some(run) = self.order.get_mut(lo..hi) {
            run.sort_by(|&i, &j| along(i).total_cmp(&along(j)));
        }
        let l = self.split(lo, mid);
        let r = self.split(mid, hi);
        if let Some(node) = self.nodes.get_mut(at) {
            node.1 = Node::Inner(l, r);
        }
        at
    }

    /// Fills `out` with every box `keep` holds, searching down from the root through the nodes
    /// `skip` does not rule out.
    fn collect(
        &self,
        skip: impl Fn(&Aabb) -> bool,
        keep: impl Fn(&Aabb) -> bool,
        out: &mut Vec<usize>,
    ) {
        out.clear();
        if self.nodes.is_empty() {
            return;
        }
        let mut stack = vec![0];
        while let Some(n) = stack.pop() {
            let Some((bb, node)) = self.nodes.get(n) else {
                continue;
            };
            if skip(bb) {
                continue;
            }
            match *node {
                Node::Inner(l, r) => {
                    stack.push(l);
                    stack.push(r);
                }
                Node::Leaf(lo, hi) => out.extend(
                    self.run(lo, hi)
                        .iter()
                        .copied()
                        .filter(|&i| keep(self.bounds(i))),
                ),
            }
        }
    }

    /// Every box that, slid any distance either way along `d`, meets `q`: every triangle a line
    /// along `d` through `q` can cross, so a ray's crossings keep their parity.
    pub fn along_line(&self, q: &Aabb, d: Vec3, out: &mut Vec<usize>) {
        self.collect(|b| !line_meets(b, q, d), |b| line_meets(b, q, d), out);
    }

    /// Every box within straight-line distance `d` of `q`.
    pub fn within(&self, q: &Aabb, d: f64, out: &mut Vec<usize>) {
        self.collect(|b| box_gap(b, q) > d, |b| box_gap(b, q) <= d, out);
    }

    /// Every box, widened by `pad` on each side, that the segment `o + t·d` for `t` in `[t0, t1]`
    /// passes through.
    pub fn along(
        &self,
        o: Point3,
        d: Vec3,
        (t0, t1): (f64, f64),
        pad: f64,
        out: &mut Vec<usize>,
    ) {
        let meets = |b: &Aabb| {
            let (mut lo, mut hi) = (t0, t1);
            for (o, d, a, z) in [
                (o.x, d.x, b.min.x - pad, b.max.x + pad),
                (o.y, d.y, b.min.y - pad, b.max.y + pad),
                (o.z, d.z, b.min.z - pad, b.max.z + pad),
            ] {
                if d.abs() < 1e-300 {
                    if o < a || o > z {
                        return false;
                    }
                    continue;
                }
                let (s0, s1) = ((a - o) / d, (z - o) / d);
                lo = lo.max(s0.min(s1));
                hi = hi.min(s0.max(s1));
                if lo > hi {
                    return false;
                }
            }
            true
        };
        self.collect(|b| !meets(b), meets, out);
    }

    /// Whether `near` holds for some box of `self` and some box of `other` moved by `t` that lie
    /// within `d` of each other, trying pairs nearest first. Neither hierarchy is rebuilt: `other`'s
    /// boxes are moved whole, which keeps them around what they held.
    pub fn any_within(
        &self,
        other: &Bvh,
        t: &Transform,
        d: f64,
        mut near: impl FnMut(usize, usize) -> bool,
    ) -> bool {
        if self.nodes.is_empty() || other.nodes.is_empty() {
            return false;
        }
        let mut stack = vec![(0usize, 0usize)];
        while let Some((a, b)) = stack.pop() {
            let (Some((ba, na)), Some((bb, nb))) = (self.nodes.get(a), other.nodes.get(b)) else {
                continue;
            };
            let bb = moved_box(bb, t);
            if box_gap(ba, &bb) > d {
                continue;
            }
            match (na, nb) {
                (&Node::Leaf(alo, ahi), &Node::Leaf(blo, bhi)) => {
                    for &j in other.run(blo, bhi) {
                        let bj = moved_box(other.bounds(j), t);
                        for &i in self.run(alo, ahi) {
                            if box_gap(self.bounds(i), &bj) <= d && near(i, j) {
                                return true;
                            }
                        }
                    }
                }
                (&Node::Inner(l, r), Node::Leaf(..)) => {
                    stack.push((l, b));
                    stack.push((r, b));
                }
                (Node::Leaf(..), &Node::Inner(l, r)) => {
                    stack.push((a, l));
                    stack.push((a, r));
                }
                (&Node::Inner(al, ar), &Node::Inner(bl, br)) => {
                    // Open the larger box first.
                    let ea = ba.max - ba.min;
                    let eb = bb.max - bb.min;
                    if ea.dot(ea) >= eb.dot(eb) {
                        stack.push((al, b));
                        stack.push((ar, b));
                    } else {
                        stack.push((a, bl));
                        stack.push((a, br));
                    }
                }
            }
        }
        false
    }

    /// Every box that meets `q`.
    pub fn query(&self, q: &Aabb, out: &mut Vec<usize>) {
        let meets = |b: &Aabb| {
            b.min.x <= q.max.x
                && q.min.x <= b.max.x
                && b.min.y <= q.max.y
                && q.min.y <= b.max.y
                && b.min.z <= q.max.z
                && q.min.z <= b.max.z
        };
        self.collect(|b| !meets(b), meets, out);
    }
}

/// Whether `b` slid some distance either way along `d` meets `q`: on each axis the slide carries
/// it over `q` for a span of distances, and those spans share one.
fn line_meets(b: &Aabb, q: &Aabb, d: Vec3) -> bool {
    let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
    for (lo, hi, step) in [
        (q.min.x - b.max.x, q.max.x - b.min.x, d.x),
        (q.min.y - b.max.y, q.max.y - b.min.y, d.y),
        (q.min.z - b.max.z, q.max.z - b.min.z, d.z),
    ] {
        if step.abs() < 1e-12 {
            if lo > 0.0 || hi < 0.0 {
                return false;
            }
            continue;
        }
        let (x, y) = (lo / step, hi / step);
        t0 = t0.max(x.min(y));
        t1 = t1.min(x.max(y));
    }
    t0 <= t1
}

/// The straight-line distance between two boxes; zero where they meet.
pub fn box_gap(a: &Aabb, b: &Aabb) -> f64 {
    let gap = |lo1: f64, hi1: f64, lo2: f64, hi2: f64| (lo2 - hi1).max(lo1 - hi2).max(0.0);
    let (x, y, z) = (
        gap(a.min.x, a.max.x, b.min.x, b.max.x),
        gap(a.min.y, a.max.y, b.min.y, b.max.y),
        gap(a.min.z, a.max.z, b.min.z, b.max.z),
    );
    (x * x + y * y + z * z).sqrt()
}

/// The box round `b`'s corners moved by `t`.
pub fn moved_box(b: &Aabb, t: &Transform) -> Aabb {
    let mut out = Aabb::EMPTY;
    for x in [b.min.x, b.max.x] {
        for y in [b.min.y, b.max.y] {
            for z in [b.min.z, b.max.z] {
                out.add(t.point(Point3::new(x, y, z)));
            }
        }
    }
    out
}

/// The three sides of triangle `t`, each from a corner to the next.
fn sides(t: &[Point3; 3]) -> [(Point3, Point3); 3] {
    let [a, b, c] = *t;
    [(a, b), (b, c), (c, a)]
}

/// The distance between two triangles: zero where they cross, else the nearest of each one's
/// sides to the other.
pub fn triangles(a: &[Point3; 3], b: &[Point3; 3]) -> f64 {
    let mut best = f64::INFINITY;
    for (s, t) in [(a, b), (b, a)] {
        for (p, q) in sides(s) {
            best = best.min(segment_triangle(p, q, t).0);
            if best == 0.0 {
                return 0.0;
            }
        }
    }
    best
}

/// Where the line `p + t·d` crosses the triangle, as `t`: where it meets the triangle's plane,
/// if that point lies within the three sides.
pub fn ray_triangle(p: Point3, d: Vec3, a: Point3, b: Point3, c: Point3) -> Option<f64> {
    let (side_b, side_c) = (b - a, c - a);
    let normal = side_b.cross(side_c);
    let approach = d.dot(normal);
    // A ray in the triangle's plane, or within rounding of it, crosses nowhere it can place: its
    // approach is rounding, and the crossing it gives anywhere along the ray.
    let flat = 1e-12 * d.norm() * side_b.norm() * side_c.norm();
    if approach.abs() <= flat || approach.abs() < 1e-300 {
        return None;
    }
    let t = (a - p).dot(normal) / approach;
    // The meeting point as `a + u·side_b + v·side_c`: each weight is the share of the
    // triangle's area that the point spans with the other side.
    let from_a = p + d * t - a;
    let area = normal.dot(normal);
    let u = from_a.cross(side_c).dot(normal) / area;
    let v = side_b.cross(from_a).dot(normal) / area;
    (u >= 0.0 && v >= 0.0 && u + v <= 1.0).then_some(t)
}

/// The point of segment `ab` nearest `p`.
pub fn on_segment(p: Point3, a: Point3, b: Point3) -> Point3 {
    let d = b - a;
    let l = d.dot(d);
    if l <= 0.0 {
        return a;
    }
    a + d * ((p - a).dot(d) / l).clamp(0.0, 1.0)
}

/// The closest points of two segments, and their distance. The squared distance is a convex
/// quadratic over the square of the two parameters, so its least value is where its gradient
/// vanishes, if that lies in the square, and otherwise on the square's rim: an end of one segment
/// against the other.
pub fn segments(p1: Point3, q1: Point3, p2: Point3, q2: Point3) -> (f64, Point3, Point3) {
    let (d1, d2, apart) = (q1 - p1, q2 - p2, p1 - p2);
    let (l1, l2, lean) = (d1.dot(d1), d2.dot(d2), d1.dot(d2));
    let mut best = (f64::INFINITY, p1, p2);
    let mut offer = |c1: Point3, c2: Point3| {
        let d = c1.dist(c2);
        if d < best.0 {
            best = (d, c1, c2);
        }
    };
    offer(p1, on_segment(p1, p2, q2));
    offer(q1, on_segment(q1, p2, q2));
    offer(on_segment(p2, p1, q1), p2);
    offer(on_segment(q2, p1, q1), q2);
    // Where both lines are nearest, when they are not parallel.
    let det = l1 * l2 - lean * lean;
    if det > 0.0 {
        let (pull1, pull2) = (d1.dot(apart), d2.dot(apart));
        let s = (lean * pull2 - pull1 * l2) / det;
        let t = (l1 * pull2 - lean * pull1) / det;
        if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
            offer(p1 + d1 * s, p2 + d2 * t);
        }
    }
    best
}

/// The point of triangle `t` nearest `p`: the foot of `p` on the triangle's plane where that
/// lies within the triangle, and otherwise the nearest point of its three sides.
pub fn on_triangle(p: Point3, t: &[Point3; 3]) -> Point3 {
    let [a, b, c] = *t;
    let (side_b, side_c, from_a) = (b - a, c - a, p - a);
    // The foot as `a + u·side_b + v·side_c`, from the two conditions that `p − foot` is square
    // to both sides.
    let (bb, bc, cc) = (side_b.dot(side_b), side_b.dot(side_c), side_c.dot(side_c));
    let det = bb * cc - bc * bc;
    if det > 0.0 {
        let (pb, pc) = (from_a.dot(side_b), from_a.dot(side_c));
        let u = (cc * pb - bc * pc) / det;
        let v = (bb * pc - bc * pb) / det;
        if u >= 0.0 && v >= 0.0 && u + v <= 1.0 {
            return a + side_b * u + side_c * v;
        }
    }
    let mut best = on_segment(p, a, b);
    for (from, to) in [(b, c), (c, a)] {
        let q = on_segment(p, from, to);
        if q.dist(p) < best.dist(p) {
            best = q;
        }
    }
    best
}

/// The closest points of a segment and a triangle: where the segment crosses the triangle, or the
/// nearest of its ends to the triangle and its sides to the segment.
pub fn segment_triangle(p: Point3, q: Point3, t: &[Point3; 3]) -> (f64, Point3, Point3) {
    if let Some(x) = crossing(p, q, t) {
        return (0.0, x, x);
    }
    let mut best = (f64::INFINITY, p, t[0]);
    for e in [p, q] {
        let c = on_triangle(e, t);
        let d = e.dist(c);
        if d < best.0 {
            best = (d, e, c);
        }
    }
    for (a, b) in sides(t) {
        let x = segments(p, q, a, b);
        if x.0 < best.0 {
            best = x;
        }
    }
    best
}

/// Where segment `pq` crosses triangle `t`, if it does.
pub fn crossing(p: Point3, q: Point3, t: &[Point3; 3]) -> Option<Point3> {
    let [a, b, c] = *t;
    let d = q - p;
    let s = ray_triangle(p, d, a, b, c)?;
    (0.0..=1.0).contains(&s).then(|| p + d * s)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every query finds what testing each box alone finds, in some order.
    #[test]
    fn queries_match_testing_every_box() {
        let mut seed = 11u64;
        let mut rnd = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut point = || Vec3::new(rnd() * 10.0, rnd() * 10.0, rnd() * 10.0);
        let boxes: Vec<Aabb> = (0..200)
            .map(|_| {
                let mut b = Aabb::EMPTY;
                b.add(point());
                b.add(point() * 0.1 + b.min);
                b
            })
            .collect();
        let bvh = Bvh::of_boxes(boxes.clone());
        let mut q = Aabb::EMPTY;
        q.add(Vec3::new(3.0, 3.0, 3.0));
        q.add(Vec3::new(5.0, 4.0, 6.0));
        let sorted = |mut v: Vec<usize>| {
            v.sort_unstable();
            v
        };
        let every = |keep: &dyn Fn(&Aabb) -> bool| -> Vec<usize> {
            (0..boxes.len()).filter(|&i| keep(&boxes[i])).collect()
        };
        let mut out = Vec::new();
        bvh.query(&q, &mut out);
        assert_eq!(sorted(out.clone()), every(&|b| box_gap(b, &q) == 0.0));
        bvh.within(&q, 1.5, &mut out);
        assert_eq!(sorted(out.clone()), every(&|b| box_gap(b, &q) <= 1.5));
        let d = Vec3::new(1.0, 0.5, 0.0);
        bvh.along_line(&q, d, &mut out);
        assert_eq!(sorted(out.clone()), every(&|b| line_meets(b, &q, d)));
        assert!(!out.is_empty());
    }

    #[test]
    fn a_segment_in_a_tilted_triangle_s_plane_does_not_cross_it_from_afar() {
        // Two gears' bottom faces, coplanar, tilted half a radian: a side of one's mesh triangle
        // lies in the other's plane, which rounding puts a hair off, about 4.9 from it.
        let tri = [
            Vec3::new(14.508628775221682, -16.768516875320916, -3.068077540287854),
            Vec3::new(23.328340064120123, -3.3624006213025357, 4.255717148417697),
            Vec3::new(15.411143728305284, -16.19447442838339, -2.754476722249861),
        ];
        let other = [
            Vec3::new(55.55756088708297, -0.7777271174955624, 5.667730718980721),
            Vec3::new(55.55756088708297, -3.2222728825044435, 4.3322692810192756),
            Vec3::new(28.176531602590885, -3.004352420640489, 4.451319771923463),
        ];
        for (s, t) in [(&tri, &other), (&other, &tri)] {
            for k in 0..3 {
                let (p, q) = (s[k], s[(k + 1) % 3]);
                assert!(crossing(p, q, t).is_none(), "side {k} crosses");
                assert!(segment_triangle(p, q, t).0 > 4.0, "side {k} is near");
            }
        }
    }
}
