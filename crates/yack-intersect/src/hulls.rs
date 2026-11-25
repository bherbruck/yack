// yack: Boxes that hold a spline curve piece by piece, from each knot span's control points, and a tree
// Adapted from springrep@a03e8a1 src/boolean/hulls.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from boolean/hulls.rs to intersect/hulls.rs (M2.1, T2); paths only; then to yack-intersect (M3 W4), paths and visibility only.
// M3 hardening: named pieces, windows over the knots and control points, an owned tree walked
// without indexing; the same arithmetic in the same order.
//! Boxes that hold a spline curve piece by piece, from each knot span's control points, and a tree
//! of them for finding which pieces of one curve come near another.
//!
//! A B-spline's piece over a knot span lies in the convex hull of the span's `degree + 1` control
//! points, and so does a rational one's, its weights being positive: the box of those points
//! holds the piece, with no sampling.

use std::ops::Range;

use crate::{
    geom::Curve,
    math::{Aabb, Point3},
};

/// A piece of a spline curve: a parameter interval and a box holding the curve over it.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Piece {
    pub(crate) span: (f64, f64),
    pub(crate) hull: Aabb,
}

/// The pieces of `c` over `range`, in order; `None` for a curve with no control points, or an open
/// one whose range runs past its ends, where it extends beyond its hull.
pub(crate) fn pieces(c: &Curve, range: (f64, f64)) -> Option<Vec<Piece>> {
    let (degree, knots, ctrl, period): (usize, &[f64], &[Point3], Option<f64>) = match c {
        Curve::Nurbs(n) => (n.degree(), n.knots(), n.control_points(), n.period()),
        _ => {
            let b = c.spline()?;
            (b.degree(), b.knots(), b.control_points(), b.period())
        }
    };
    let (start, end) = (*knots.get(degree)?, *knots.get(ctrl.len())?);
    let shifts: Vec<f64> = match period {
        Some(p) if p > 0.0 => {
            let first = ((range.0 - end) / p).floor() as i64;
            let last = ((range.1 - start) / p).ceil() as i64;
            (first..=last).map(|k| k as f64 * p).collect()
        }
        _ if range.0 < start || range.1 > end => return None,
        _ => vec![0.0],
    };
    // Span `i` (from `degree` on) runs from knot `i` to knot `i + 1`, held by control points
    // `i - degree ..= i`.
    let spans = knots.get(degree..)?.windows(2);
    let mut out = Vec::new();
    for shift in shifts {
        for (span, hull_points) in spans.clone().zip(ctrl.windows(degree + 1)) {
            let &[k0, k1] = span else { continue };
            let (t0, t1) = (k0 + shift, k1 + shift);
            if t1 <= t0 || t1 < range.0 || t0 > range.1 {
                continue;
            }
            let mut hull = Aabb::EMPTY;
            for &q in hull_points {
                hull.add(q);
            }
            out.push(Piece { span: (t0.max(range.0), t1.min(range.1)), hull });
        }
    }
    (!out.is_empty()).then_some(out)
}

/// The parameter intervals of `c1`'s pieces over `r1`, in order, whose boxes come within `reach`
/// of a box of `c2`'s over `r2`: away from them `c1` is farther than `reach` from `c2`. `None`
/// where either curve has no hulls.
pub(crate) fn near(
    c1: &Curve,
    r1: (f64, f64),
    c2: &Curve,
    r2: (f64, f64),
    reach: f64,
) -> Option<Vec<(f64, f64)>> {
    let (p1, p2) = (pieces(c1, r1)?, pieces(c2, r2)?);
    let t1 = Tree::new(p1.iter().map(|p| p.hull).collect());
    let t2 = Tree::new(p2.iter().map(|p| p.hull).collect());
    let marks = t1.near(&t2, reach);
    Some(p1.iter().zip(marks).filter(|&(_, near)| near).map(|(p, _)| p.span).collect())
}

/// The gap between two boxes, as the largest gap along an axis: no more than the true distance.
pub(crate) fn gap(a: &Aabb, b: &Aabb) -> f64 {
    let below = (a.min - b.max).to_array();
    let above = (b.min - a.max).to_array();
    below.into_iter().zip(above).map(|(l, h)| l.max(h)).fold(0.0, f64::max)
}

/// A binary tree over a curve's consecutive pieces, each node boxing the pieces under it.
struct Tree {
    boxes: Vec<Aabb>,
    root: Option<Node>,
}

struct Node {
    bounds: Aabb,
    /// The boxes under this node, as a range of the tree's.
    under: Range<usize>,
    kids: Option<Box<[Node; 2]>>,
}

/// The most boxes a leaf holds.
const LEAF: usize = 4;

impl Node {
    /// The node over `boxes`, the tree's boxes from `first` on.
    fn new(boxes: &[Aabb], first: usize) -> Node {
        let under = first..first + boxes.len();
        if boxes.len() <= LEAF {
            let mut bounds = Aabb::EMPTY;
            for b in boxes {
                bounds.union(b);
            }
            return Node { bounds, under, kids: None };
        }
        let half = boxes.len() / 2;
        let (left, right) = boxes.split_at(half);
        let kids = [Node::new(left, first), Node::new(right, first + half)];
        let [l, r] = &kids;
        let mut bounds = l.bounds;
        bounds.union(&r.bounds);
        Node { bounds, under, kids: Some(Box::new(kids)) }
    }
}

impl Tree {
    fn new(boxes: Vec<Aabb>) -> Tree {
        let root = (!boxes.is_empty()).then(|| Node::new(&boxes, 0));
        Tree { boxes, root }
    }

    /// For each of this tree's boxes, whether some box of `other` comes within `reach` of it.
    fn near(&self, other: &Tree, reach: f64) -> Vec<bool> {
        let mut out = vec![false; self.boxes.len()];
        if let (Some(mine), Some(theirs)) = (&self.root, &other.root) {
            self.mark(mine, other, theirs, reach, &mut out);
        }
        out
    }

    /// Marks in `out` this tree's boxes under `mine` that come within `reach` of a box of
    /// `other`'s under `theirs`, splitting the larger node of the two first.
    fn mark(&self, mine: &Node, other: &Tree, theirs: &Node, reach: f64, out: &mut [bool]) {
        let Some(marks) = out.get_mut(mine.under.clone()) else { return };
        if gap(&mine.bounds, &theirs.bounds) > reach || marks.iter().all(|&m| m) {
            return;
        }
        match (&mine.kids, &theirs.kids) {
            (None, None) => {
                let (Some(own), Some(near)) =
                    (self.boxes.get(mine.under.clone()), other.boxes.get(theirs.under.clone()))
                else {
                    return;
                };
                for (m, bb) in marks.iter_mut().zip(own) {
                    if !*m {
                        *m = near.iter().any(|q| gap(bb, q) <= reach);
                    }
                }
            }
            (Some(kids), None) => {
                for kid in kids.iter() {
                    self.mark(kid, other, theirs, reach, out);
                }
            }
            (None, Some(kids)) => {
                for kid in kids.iter() {
                    self.mark(mine, other, kid, reach, out);
                }
            }
            (Some(my_kids), Some(their_kids)) => {
                if mine.under.len() >= theirs.under.len() {
                    for kid in my_kids.iter() {
                        self.mark(kid, other, theirs, reach, out);
                    }
                } else {
                    for kid in their_kids.iter() {
                        self.mark(mine, other, kid, reach, out);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geom::{BSpline, Line},
        math::Vec3,
    };

    #[test]
    fn each_piece_lies_in_its_box() {
        // A cubic through points on a wave: every sample of each piece falls in that piece's box.
        let pts: Vec<Vec3> =
            (0..12).map(|i| Vec3::new(i as f64, (i as f64).sin(), 0.5 * (i % 3) as f64)).collect();
        let c = Curve::BSpline(BSpline::interpolate(&pts).unwrap());
        let (a, b) = c.domain();
        let ps = pieces(&c, (a + 0.3, b - 0.2)).unwrap();
        assert_eq!(ps.first().unwrap().span.0, a + 0.3);
        assert_eq!(ps.last().unwrap().span.1, b - 0.2);
        for Piece { span: (t0, t1), hull } in ps {
            for k in 0..=50 {
                let p = c.eval(t0 + (t1 - t0) * k as f64 / 50.0);
                assert!((0..3).all(|j| p[j] >= hull.min[j] - 1e-12 && p[j] <= hull.max[j] + 1e-12));
            }
        }
        // Past an open curve's end it extends beyond its hull; a line has no control points.
        assert!(pieces(&c, (a - 1.0, b)).is_none());
        let line = Curve::Line(Line { origin: Vec3::ZERO, dir: Vec3::X });
        assert!(pieces(&line, (0.0, 1.0)).is_none());
    }

    #[test]
    fn the_tree_marks_the_boxes_within_reach() {
        // Unit boxes along x at 0, 2, 4, …, 18 against one box at x 7.5 to 8.5: the boxes at 6
        // and 8 reach it, 6 only from 0.5 away.
        let unit = |x: f64| Aabb { min: Vec3::new(x, 0.0, 0.0), max: Vec3::new(x + 1.0, 1.0, 1.0) };
        let a = Tree::new((0..10).map(|i| unit(2.0 * i as f64)).collect());
        let b = Tree::new(vec![unit(7.5)]);
        let want = |reach: f64, hits: &[usize]| {
            let got = a.near(&b, reach);
            let marked: Vec<usize> = (0..10).filter(|&i| got[i]).collect();
            assert_eq!(marked, hits, "reach {reach}");
        };
        want(0.0, &[4]);
        want(0.5, &[3, 4]);
        want(1.5, &[3, 4, 5]);
    }
}
