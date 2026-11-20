// yack: 2-D loop and profile validation: continuity, closure, orientation, crossings.
//! Checks on closed planar loops given as [`Piece`]s, used by the profile constructors.
use yack_base::{Error, Result};

use crate::{Angle, Point2};

/// One piece of a planar loop, as the validator sees it.
#[derive(Clone, Debug, PartialEq)]
pub enum Piece {
    /// A straight segment from `a` to `b`.
    Line {
        /// Start.
        a: Point2,
        /// End.
        b: Point2,
    },
    /// A circular arc; `sweep > 0` turns counter-clockwise.
    Arc {
        /// Centre.
        center: Point2,
        /// Radius.
        radius: f64,
        /// Start angle.
        start: Angle,
        /// Signed sweep.
        sweep: Angle,
    },
    /// A rational quadratic from `a` to `b` with control point `apex` and weight `weight`.
    Conic {
        /// Start.
        a: Point2,
        /// Control point.
        apex: Point2,
        /// End.
        b: Point2,
        /// Weight of `apex`, `> 0`.
        weight: f64,
    },
    /// A curve from `hull[0]` to `hull[last]` lying in its control polygon `hull`. It counts for
    /// orientation; crossings against it are not checked here.
    Hull(Vec<Point2>),
}

/// A validated loop: its signed area (positive counter-clockwise) and a chordal polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct LoopShape {
    /// Signed area of the chordal polyline, positive for a counter-clockwise loop.
    pub area: f64,
    /// The chordal polyline, closed implicitly (last vertex joins the first).
    pub polyline: Vec<Point2>,
    /// For each polyline edge, the piece it came from, and whether crossings are checked on it.
    pieces: Vec<(usize, bool)>,
}

/// Whether two planar points are the same point, within `tol::LINEAR`.
pub fn coincident(a: Point2, b: Point2) -> bool {
    a.distance(b) <= yack_tol::LINEAR
}

/// The four quarters of the axis-aligned ellipse about `center` with semi-axes `a` (along x) and
/// `b` (along y), counter-clockwise from `(center.x + a, center.y)`: each `(start, apex, end)` of a
/// rational quadratic with apex weight `1/√2`.
pub fn ellipse_quarters(center: Point2, a: f64, b: f64) -> [(Point2, Point2, Point2); 4] {
    let at = |x: f64, y: f64| Point2::new(center.x + x, center.y + y);
    [
        (at(a, 0.0), at(a, b), at(0.0, b)),
        (at(0.0, b), at(-a, b), at(-a, 0.0)),
        (at(-a, 0.0), at(-a, -b), at(0.0, -b)),
        (at(0.0, -b), at(a, -b), at(a, 0.0)),
    ]
}

/// The two ends of an arc.
pub fn arc_ends(center: Point2, radius: f64, start: Angle, sweep: Angle) -> [Point2; 2] {
    let at = |t: f64| {
        let (s, c) = t.sin_cos();
        Point2::new(center.x + radius * c, center.y + radius * s)
    };
    [at(start.radians()), at(start.radians() + sweep.radians())]
}

/// The arc from `from` to `to` about `center`, counter-clockwise if `ccw`, else clockwise:
/// `(radius, start, sweep)` with `0 < |sweep| ≤ 2π` (`from == to` is a full circle). Refuses ends
/// whose radii differ beyond `tol::LINEAR`, naming both.
pub fn arc_to(op: &'static str, from: Point2, to: Point2, center: Point2, ccw: bool) -> Result<(f64, Angle, Angle)> {
    let r0 = from.distance(center);
    let r1 = to.distance(center);
    if !(r0.is_finite() && r1.is_finite()) || r0 <= yack_tol::LINEAR {
        return Err(Error::invalid(op, "center", format!("centre ({}, {}) gives no arc", center.x, center.y)));
    }
    if !yack_tol::within(r0 - r1, yack_tol::LINEAR, r0) {
        return Err(Error::invalid(
            op,
            "center",
            format!("the ends lie at radii {r0} and {r1} from ({}, {})", center.x, center.y),
        ));
    }
    let a0 = (from.y - center.y).atan2(from.x - center.x);
    let a1 = (to.y - center.y).atan2(to.x - center.x);
    let tau = core::f64::consts::TAU;
    let mut sweep = if ccw { (a1 - a0).rem_euclid(tau) } else { -(a0 - a1).rem_euclid(tau) };
    if from.distance(to) <= yack_tol::LINEAR {
        sweep = if ccw { tau } else { -tau };
    }
    Ok((r0, crate::rad(a0), crate::rad(sweep)))
}

fn ends(p: &Piece) -> Option<[Point2; 2]> {
    match p {
        Piece::Line { a, b } | Piece::Conic { a, b, .. } => Some([*a, *b]),
        Piece::Arc { center, radius, start, sweep } => Some(arc_ends(*center, *radius, *start, *sweep)),
        Piece::Hull(h) => Some([*h.first()?, *h.last()?]),
    }
}

fn finite(op: &'static str, i: usize, p: &Piece) -> Result<()> {
    let bad = |what: &str| Err(Error::invalid(op, "segments", format!("segment {i}: {what}")));
    match p {
        Piece::Line { a, b } => {
            if !(a.is_finite() && b.is_finite()) {
                return bad("a point is not finite");
            }
            if a.distance(*b) <= yack_tol::LINEAR {
                return bad("the line has zero length");
            }
        }
        Piece::Arc { center, radius, start, sweep } => {
            if !(center.is_finite() && radius.is_finite() && start.is_finite() && sweep.is_finite()) {
                return bad("a value is not finite");
            }
            if *radius <= yack_tol::LINEAR {
                return bad("the radius is not positive");
            }
            if sweep.radians().abs() * radius <= yack_tol::LINEAR {
                return bad("the arc has zero sweep");
            }
            if sweep.radians().abs() > core::f64::consts::TAU {
                return bad("the arc sweeps more than a full turn");
            }
        }
        Piece::Conic { a, apex, b, weight } => {
            if !(a.is_finite() && apex.is_finite() && b.is_finite() && weight.is_finite()) {
                return bad("a value is not finite");
            }
            if *weight <= 0.0 {
                return bad("the weight is not positive");
            }
        }
        Piece::Hull(h) => {
            if h.len() < 2 || !h.iter().all(|p| p.is_finite()) {
                return bad("the control points are missing or not finite");
            }
        }
    }
    Ok(())
}

/// Chord count for an arc of `sweep` radians: about 64 per turn.
fn chords(sweep: f64) -> usize {
    let n = (sweep.abs() / core::f64::consts::TAU * 64.0).ceil();
    if n.is_finite() && n >= 4.0 { n as usize } else { 4 }
}

/// Polyline vertices of a piece, excluding its end point.
fn samples(p: &Piece, out: &mut Vec<Point2>) -> bool {
    match p {
        Piece::Line { a, .. } => {
            out.push(*a);
            true
        }
        Piece::Arc { center, radius, start, sweep } => {
            let n = chords(sweep.radians());
            for k in 0..n {
                let t = start.radians() + sweep.radians() * (k as f64) / (n as f64);
                let (s, c) = t.sin_cos();
                out.push(Point2::new(center.x + radius * c, center.y + radius * s));
            }
            true
        }
        Piece::Conic { a, apex, b, weight } => {
            let n = 16usize;
            for k in 0..n {
                let t = (k as f64) / (n as f64);
                let (b0, b1, b2) = ((1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t) * weight, t * t);
                let w = b0 + b1 + b2;
                out.push(Point2::new((b0 * a.x + b1 * apex.x + b2 * b.x) / w, (b0 * a.y + b1 * apex.y + b2 * b.y) / w));
            }
            true
        }
        Piece::Hull(h) => {
            let k = h.len().saturating_sub(1);
            out.extend(h.iter().take(k).copied());
            false
        }
    }
}

fn orient(a: Point2, b: Point2, c: Point2) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/// Distance from `p` to the segment `[a, b]`.
fn to_segment(p: Point2, a: Point2, b: Point2) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (((p.x - a.x) * dx + (p.y - a.y) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    p.distance(Point2::new(a.x + t * dx, a.y + t * dy))
}

/// Whether segments `[a, b]` and `[c, d]` cross or come within `eps`.
fn meet(a: Point2, b: Point2, c: Point2, d: Point2, eps: f64) -> bool {
    let (o1, o2, o3, o4) = (orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b));
    if ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) && ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0)) {
        return true;
    }
    to_segment(c, a, b) <= eps || to_segment(d, a, b) <= eps || to_segment(a, c, d) <= eps || to_segment(b, c, d) <= eps
}

fn edges(s: &LoopShape) -> impl Iterator<Item = (usize, Point2, Point2, (usize, bool))> + '_ {
    let n = s.polyline.len();
    s.polyline.iter().zip(s.pieces.iter()).enumerate().filter_map(move |(i, (a, piece))| {
        let b = s.polyline.get((i + 1) % n.max(1))?;
        Some((i, *a, *b, *piece))
    })
}

/// Validates a closed loop: finite values, each piece ending where the next starts (and the last
/// where the first starts), non-zero area, and no piece crossing or touching another.
pub fn check_loop(op: &'static str, pieces: &[Piece]) -> Result<LoopShape> {
    if pieces.is_empty() {
        return Err(Error::invalid(op, "segments", "a loop needs at least one segment"));
    }
    let mut size = 0.0f64;
    for (i, p) in pieces.iter().enumerate() {
        finite(op, i, p)?;
        if let Some([a, b]) = ends(p) {
            size = size.max(a.x.abs()).max(a.y.abs()).max(b.x.abs()).max(b.y.abs());
        }
    }
    let n = pieces.len();
    for (i, p) in pieces.iter().enumerate() {
        let next = pieces.get((i + 1) % n);
        if let (Some([_, end]), Some([start, _])) = (ends(p), next.and_then(ends)) {
            let gap = end.distance(start);
            if !yack_tol::within(gap, yack_tol::LINEAR, size) {
                return Err(Error::invalid(
                    op,
                    "segments",
                    format!(
                        "segment {i} ends at ({}, {}) but segment {} starts at ({}, {}), {gap} mm away",
                        end.x,
                        end.y,
                        (i + 1) % n,
                        start.x,
                        start.y
                    ),
                ));
            }
        }
    }
    let mut polyline = Vec::new();
    let mut owner = Vec::new();
    for (i, p) in pieces.iter().enumerate() {
        let before = polyline.len();
        let checked = samples(p, &mut polyline);
        owner.extend(core::iter::repeat_n((i, checked), polyline.len() - before));
    }
    let m = polyline.len();
    let mut area = 0.0;
    for (i, a) in polyline.iter().enumerate() {
        if let Some(b) = polyline.get((i + 1) % m) {
            area += a.x * b.y - b.x * a.y;
        }
    }
    let area = 0.5 * area;
    if area.abs() <= yack_tol::LINEAR * size.max(1.0) {
        return Err(Error::invalid(op, "segments", "the loop encloses no area"));
    }
    let shape = LoopShape { area, polyline, pieces: owner };
    let eps = yack_tol::LINEAR * size.max(1.0);
    for (i, a, b, (pi, ci)) in edges(&shape) {
        for (j, c, d, (pj, cj)) in edges(&shape).skip(i + 1) {
            let adjacent = j == i + 1 || (i == 0 && j + 1 == m);
            if adjacent || !(ci && cj) {
                continue;
            }
            if meet(a, b, c, d, eps) {
                return Err(Error::invalid(op, "segments", format!("segment {pi} crosses or touches segment {pj}")));
            }
        }
    }
    Ok(shape)
}

/// Whether `p` is inside the closed polyline (non-zero winding).
fn inside(p: Point2, poly: &[Point2]) -> bool {
    let n = poly.len();
    let mut wind = 0i32;
    for (i, a) in poly.iter().enumerate() {
        let Some(b) = poly.get((i + 1) % n) else { continue };
        if a.y <= p.y {
            if b.y > p.y && orient(*a, *b, p) > 0.0 {
                wind += 1;
            }
        } else if b.y <= p.y && orient(*a, *b, p) < 0.0 {
            wind -= 1;
        }
    }
    wind != 0
}

fn apart(op: &'static str, s: &LoopShape, t: &LoopShape, what: &str) -> Result<()> {
    let size = s.polyline.iter().chain(t.polyline.iter()).fold(1.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()));
    let eps = yack_tol::LINEAR * size;
    for (_, a, b, (_, ca)) in edges(s) {
        for (_, c, d, (_, cb)) in edges(t) {
            if ca && cb && meet(a, b, c, d, eps) {
                return Err(Error::invalid(op, "holes", format!("{what} cross or touch")));
            }
        }
    }
    Ok(())
}

/// Validates a profile: the outer loop counter-clockwise, each hole clockwise, inside the outer
/// loop, outside every other hole, and touching nothing.
pub fn check_profile(op: &'static str, outer: &LoopShape, holes: &[LoopShape]) -> Result<()> {
    if outer.area <= 0.0 {
        return Err(Error::invalid(op, "outer", "the outer loop runs clockwise; it must run counter-clockwise"));
    }
    for (i, h) in holes.iter().enumerate() {
        if h.area >= 0.0 {
            return Err(Error::invalid(op, "holes", format!("hole {i} runs counter-clockwise; holes run clockwise")));
        }
        apart(op, outer, h, &format!("hole {i} and the outer loop"))?;
        let Some(p) = h.polyline.first() else { continue };
        if !inside(*p, &outer.polyline) {
            return Err(Error::invalid(op, "holes", format!("hole {i} is not inside the outer loop")));
        }
        for (j, g) in holes.iter().enumerate().take(i) {
            apart(op, g, h, &format!("holes {j} and {i}"))?;
            let nested = h.polyline.first().is_some_and(|p| inside(*p, &g.polyline))
                || g.polyline.first().is_some_and(|p| inside(*p, &h.polyline));
            if nested {
                return Err(Error::invalid(op, "holes", format!("holes {j} and {i} are nested")));
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::deg;

    fn square(c: [[f64; 2]; 4]) -> Vec<Piece> {
        (0..4).map(|i| Piece::Line { a: c[i].into(), b: c[(i + 1) % 4].into() }).collect()
    }

    #[test]
    fn square_is_ccw() {
        let s = check_loop("t", &square([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])).unwrap();
        assert!(s.area > 0.0);
    }

    #[test]
    fn bowtie_crosses() {
        // an unbalanced bow tie: non-zero area, so only the crossing check refuses it
        let e = check_loop("t", &square([[0.0, 0.0], [3.0, 1.0], [3.0, 0.0], [0.0, 2.0]])).unwrap_err();
        assert!(e.detail.contains("crosses"), "{e}");
    }

    #[test]
    fn gap_is_refused() {
        let mut p = square([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
        p[3] = Piece::Line { a: [0.0, 1.0].into(), b: [0.0, 0.5].into() };
        assert!(check_loop("t", &p).is_err());
    }

    #[test]
    fn circle_with_hole() {
        let c = Point2::new(0.0, 0.0);
        let outer = check_loop("t", &[Piece::Arc { center: c, radius: 2.0, start: deg(0.0), sweep: deg(360.0) }]).unwrap();
        let hole = check_loop("t", &[Piece::Arc { center: c, radius: 1.0, start: deg(0.0), sweep: deg(-360.0) }]).unwrap();
        check_profile("t", &outer, core::slice::from_ref(&hole)).unwrap();
        assert!(check_profile("t", &hole, &[]).is_err());
    }

    #[test]
    fn slot_from_arc_to() {
        let (r, start, sweep) = arc_to("t", [1.0, 0.0].into(), [-1.0, 0.0].into(), [0.0, 0.0].into(), true).unwrap();
        assert_eq!(r, 1.0);
        assert!(start.radians().abs() < yack_tol::LINEAR);
        assert!((sweep.radians() - core::f64::consts::PI).abs() < yack_tol::LINEAR);
        let (_, _, cw) = arc_to("t", [0.0, 1.0].into(), [1.0, 0.0].into(), [0.0, 0.0].into(), false).unwrap();
        assert!((cw.radians() + core::f64::consts::FRAC_PI_2).abs() < yack_tol::LINEAR);
    }
}
