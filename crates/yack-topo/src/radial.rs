// yack: The faces round an edge more than two of them use, where pieces of a solid touch: which pairs
// Adapted from springrep@a03e8a1 src/topo/radial.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/topo/radial.rs to yack-topo (M3 W4); paths and visibility only;
// M3 hardening: each side's place round the edge is a named struct, and the walk round the edge
// pairs neighbours by iterators instead of modular indexing; the same arithmetic in the same order.
//! The faces round an edge more than two of them use, where pieces of a solid touch: which pairs
//! bound a wedge of material between them, partners, and which an empty wedge, beside each other.

use std::f64::consts::TAU;

use crate::geom::{Curve, Surface};
use crate::math::{Point3, Vec3};
use crate::tol;

/// Pairs of indices into the sides round an edge.
pub type Pairs = Vec<(usize, usize)>;

/// One face's use of an edge: its surface, whether the face is the surface turned inside out, and
/// the way its loop runs along the edge.
pub struct Side<'a> {
    pub surface: &'a Surface,
    pub reversed: bool,
    pub travel: Vec3,
}

/// The chords an edge's length is summed over.
const LENGTH_CHORDS: usize = 16;

/// An edge's length over `range`, near enough to size a step by: a closed edge's ends meet, so its
/// chord says nothing.
pub fn length(curve: &Curve, (t0, t1): (f64, f64)) -> f64 {
    let at = |k: usize| curve.eval(t0 + (t1 - t0) * k as f64 / LENGTH_CHORDS as f64);
    (0..LENGTH_CHORDS).map(|k| at(k).dist(at(k + 1))).sum()
}

/// The side's outward normal at the surface point nearest `p`.
fn outward(side: &Side, p: Point3) -> Vec3 {
    let uv = side.surface.project(p);
    let normal = side.surface.normal(uv.x, uv.y);
    if side.reversed { -normal } else { normal }
}

/// Where one side's face lies round the edge.
#[derive(Clone, Copy)]
struct Placed {
    /// Its angle about the edge, from the first side's.
    angle: f64,
    /// Whether its material lies ahead of it, turning the way the angle grows.
    ahead: bool,
    /// Its index into the sides.
    side: usize,
}

/// Each element with the one after it, the last with the first.
fn round_pairs<T>(items: &[T]) -> impl Iterator<Item = (&T, &T)> {
    items.iter().zip(items.iter().cycle().skip(1))
}

/// The `sides` round the edge through `m`, running along unit `t` and `len` long, ordered round it:
/// the partner pairs, each bounding a wedge of material, and the pairs beside each other across an
/// empty wedge, as indices into `sides`. Each side's place round the edge is read a short step
/// into its face, where faces that leave the edge tangent to each other, as a cylinder lying on a
/// plane does, have parted. Sides still level there, as the walls of a sliver where they cross,
/// take the order material and space alternate in. `None` where the sides do not alternate.
pub fn round(m: Point3, t: Vec3, len: f64, sides: &[Side]) -> Option<(Pairs, Pairs)> {
    // A step within the tolerance reads only rounding, however short the edge.
    let step = 1e-3 * len.clamp(1e3 * tol::LINEAR, 1.0);
    let mut around: Vec<Placed> = Vec::with_capacity(sides.len());
    let mut axes: Option<(Vec3, Vec3)> = None;
    for (index, side) in sides.iter().enumerate() {
        // A loop keeps its face's material on its left, seen from outside.
        let into = outward(side, m).cross(side.travel).unit()?;
        let uv = side.surface.project(m + into * step);
        let stepped = side.surface.eval(uv.x, uv.y);
        let offset = stepped - m;
        let across = (offset - t * offset.dot(t)).unit()?;
        let (a, b) = *axes.get_or_insert((across, t.cross(across)));
        around.push(Placed {
            angle: across.dot(b).atan2(across.dot(a)),
            ahead: (-outward(side, stepped)).dot(t.cross(across)) > 0.0,
            side: index,
        });
    }
    around.sort_by(|x, y| x.angle.total_cmp(&y.angle));
    // Level: the two stepped points within the tolerance of each other.
    let level = |x: f64, y: f64| {
        let d = (x - y).rem_euclid(TAU);
        d.min(TAU - d) * step <= tol::LINEAR
    };
    // Where a side is followed by one of its own kind and then, level with that one, one of the
    // other kind, the level two swap: material and space alternate.
    let count = around.len();
    for i in 0..count {
        let (j, l) = ((i + 1) % count, (i + 2) % count);
        let (Some(&first), Some(&second), Some(&third)) =
            (around.get(i), around.get(j), around.get(l))
        else {
            continue;
        };
        let alternates = second.ahead != third.ahead && level(second.angle, third.angle);
        if first.ahead == second.ahead && alternates {
            around.swap(j, l);
        }
    }
    let (mut partners, mut beside) = (Vec::new(), Vec::new());
    for (here, next) in round_pairs(&around) {
        if here.ahead == next.ahead {
            return None;
        }
        let pair = (here.side, next.side);
        if here.ahead {
            partners.push(pair);
        } else {
            beside.push(pair);
        }
    }
    Some((partners, beside))
}
