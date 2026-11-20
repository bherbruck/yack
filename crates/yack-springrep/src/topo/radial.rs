// yack: The faces round an edge more than two of them use, where pieces of a solid touch: which pairs
// Adapted from springrep@a03e8a1 src/topo/radial.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! The faces round an edge more than two of them use, where pieces of a solid touch: which pairs
//! bound a wedge of material between them, partners, and which an empty wedge, beside each other.

use crate::geom::{Curve, Surface};
use crate::math::{Point3, Vec3};
use crate::tol;

/// Pairs of indices into the sides round an edge.
pub(crate) type Pairs = Vec<(usize, usize)>;

/// One face's use of an edge: its surface, whether the face is the surface turned inside out, and
/// the way its loop runs along the edge.
pub(crate) struct Side<'a> {
    pub surface: &'a Surface,
    pub reversed: bool,
    pub travel: Vec3,
}

/// An edge's length over `range`, near enough to size a step by: a closed edge's ends meet, so its
/// chord says nothing.
pub(crate) fn length(c: &Curve, range: (f64, f64)) -> f64 {
    let n = 16;
    (0..n)
        .map(|k| {
            let at = |k: usize| c.eval(range.0 + (range.1 - range.0) * k as f64 / n as f64);
            at(k).dist(at(k + 1))
        })
        .sum()
}

fn outward(s: &Side, p: Point3) -> Vec3 {
    let uv = s.surface.project(p);
    let n = s.surface.normal(uv.x, uv.y);
    if s.reversed { -n } else { n }
}

/// The `sides` round the edge through `m`, running along unit `t` and `len` long, ordered round it:
/// the partner pairs, each bounding a wedge of material, and the pairs beside each other across an
/// empty wedge, as indices into `sides`. Each side's place round the edge is read a short step
/// into its face, where faces that leave the edge tangent to each other, as a cylinder lying on a
/// plane does, have parted. Sides still level there, as the walls of a sliver where they cross,
/// take the order material and space alternate in. `None` where the sides do not alternate.
pub(crate) fn round(m: Point3, t: Vec3, len: f64, sides: &[Side]) -> Option<(Pairs, Pairs)> {
    // A step within the tolerance reads only rounding, however short the edge.
    let step = 1e-3 * len.clamp(1e3 * tol::LINEAR, 1.0);
    let mut around: Vec<(f64, bool, usize)> = Vec::with_capacity(sides.len());
    let mut axes: Option<(Vec3, Vec3)> = None;
    for (i, s) in sides.iter().enumerate() {
        // A loop keeps its face's material on its left, seen from outside.
        let into = outward(s, m).cross(s.travel).unit()?;
        let uv = s.surface.project(m + into * step);
        let q = s.surface.eval(uv.x, uv.y);
        let w = q - m;
        let w = (w - t * w.dot(t)).unit()?;
        let (a, b) = *axes.get_or_insert((w, t.cross(w)));
        let angle = w.dot(b).atan2(w.dot(a));
        // Whether the face's material lies ahead of it, turning the way the angle grows.
        let ahead = (-outward(s, q)).dot(t.cross(w)) > 0.0;
        around.push((angle, ahead, i));
    }
    around.sort_by(|x, y| x.0.total_cmp(&y.0));
    let k = around.len();
    // Level: the two stepped points within the tolerance of each other.
    let level = |x: f64, y: f64| {
        let d = (x - y).rem_euclid(std::f64::consts::TAU);
        d.min(std::f64::consts::TAU - d) * step <= tol::LINEAR
    };
    for i in 0..k {
        let (j, l) = ((i + 1) % k, (i + 2) % k);
        if around[i].1 == around[j].1
            && around[j].1 != around[l].1
            && level(around[j].0, around[l].0)
        {
            around.swap(j, l);
        }
    }
    let (mut partners, mut beside) = (Vec::new(), Vec::new());
    for i in 0..k {
        let j = (i + 1) % k;
        if around[i].1 == around[j].1 {
            return None;
        }
        if around[i].1 {
            partners.push((around[i].2, around[j].2));
        } else {
            beside.push((around[i].2, around[j].2));
        }
    }
    Some((partners, beside))
}
