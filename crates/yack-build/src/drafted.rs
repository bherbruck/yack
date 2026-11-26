// yack: a profile extruded with drafted sides: each side a plane or a cone leaning in (or out) by the draft as it rises, the caps the profile and its sharp offset.
//! [`extrude_drafted`]: `profile` extruded `height` along its frame's normal with every side
//! leaning in by `draft` as it rises (out when negative). A drafted side keeps its kind: a line's
//! side is a plane, an arc's a cone about the arc's axis, and the side faces meet sharp, as the
//! undrafted prism's do, so the top is the profile offset in by `height·tan(draft)` with each
//! corner where its two moved sides meet (not rounded, which would be the points within that
//! distance). With `both_ways` the same again below the profile's plane, the halves meeting in a
//! crease there.
//!
//! Faces: `start` (the bottom cap), `end` (the top) and, for each side, `side` with the names of
//! the profile segment it rises from, so a side traces to its segment as an undrafted
//! extrusion's does.

use std::f64::consts::TAU;

use crate::{
    Error, Result,
    loft::{Section, loft},
    math::{Frame, Vec2},
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    tol,
    topo::Brep,
};

/// The steepest draft accepted, in radians: a side leaning further than this from the frame's
/// normal is all but flat, and its offset grows past any part.
const STEEPEST: f64 = 1.5; // about 86°

/// `profile` (drawn on `frame`) extruded `height` along `frame`'s normal with its sides drafted by
/// `draft` (radians; positive leans them in as they rise), named under `feature`. Declines, as
/// `Unsupported`, what it does not build exactly:
/// - a height that is not a positive length, or a draft not within ±[`STEEPEST`];
/// - a profile with holes;
/// - a segment other than a line, an arc or a whole circle;
/// - an offset whose sides would cross, vanish or turn back: two moved sides that no longer meet
///   near their corner, an arc whose radius closes, a line that shrinks to nothing.
pub fn extrude_drafted(frame: &Frame, profile: &Profile, height: f64, draft: f64, both_ways: bool, feature: &str) -> Result<Brep> {
    if !(height.is_finite() && height > tol::LINEAR) {
        return decline("a drafted extrusion's height must be a positive length");
    }
    if !(draft.is_finite() && draft.abs() <= STEEPEST) {
        return decline("a draft steeper than the steepest a drafted extrusion builds");
    }
    let base = profile.normalized()?;
    let [outer] = base.loops.as_slice() else {
        return decline("a drafted extrusion of a profile with holes");
    };
    // In by h·tan(draft) at the top: the outer loop runs counter-clockwise, so its inside is to
    // the left of travel and an inward move is a move to the left.
    let inward = height * draft.tan();
    let top = Profile { loops: vec![offset(outer, inward)?] };
    let lift = |d: f64| frame.translated(frame.z * d);
    let sections = if both_ways {
        vec![
            Section { profile: top.clone(), frame: lift(-height) },
            Section { profile: base.clone(), frame: *frame },
            Section { profile: top, frame: lift(height) },
        ]
    } else {
        vec![Section { profile: base.clone(), frame: *frame }, Section { profile: top, frame: lift(height) }]
    };
    loft(&sections, true, feature)
}

fn decline<T>(why: &str) -> Result<T> {
    Err(Error::Unsupported(why.into()))
}

/// The carrier of a moved side: a line through `p` along `t`, or a circle about `c` of radius `r`.
#[derive(Clone, Copy, Debug)]
enum Carrier {
    Line { p: Vec2, t: Vec2 },
    Circle { c: Vec2, r: f64 },
}

/// The counter-clockwise loop `lp` with every segment moved `d` to its left (into the region),
/// each corner where the two moved segments meet, every segment keeping its names. Declines where
/// a moved segment vanishes or turns back, or two no longer meet near their corner.
fn offset(lp: &ProfileLoop, d: f64) -> Result<ProfileLoop> {
    let segs = &lp.segments;
    if let [only] = segs.as_slice() {
        let Segment::Circle { center, radius, ccw } = only.segment else {
            return decline("a drafted extrusion of a one-segment loop other than a circle");
        };
        // The inside is to the left: inside a counter-clockwise circle, outside a clockwise one.
        let r = if ccw { radius - d } else { radius + d };
        if r.is_nan() || r <= tol::LINEAR {
            return decline("a drafted circle that closes before the top");
        }
        return Ok(ProfileLoop { segments: vec![NamedSegment { segment: Segment::Circle { center, radius: r, ccw }, names: only.names.clone() }] });
    }
    let carriers: Vec<Carrier> = segs.iter().map(|s| moved(&s.segment, d)).collect::<Result<_>>()?;
    let n = segs.len();
    // Corner k joins segment k − 1's end to segment k's start: each segment with its carrier,
    // paired with the one before it round the loop.
    let sides: Vec<(&NamedSegment, Carrier)> = segs.iter().zip(carriers.iter().copied()).collect();
    let befores = sides.iter().cycle().skip(n.saturating_sub(1));
    let mut corners = Vec::with_capacity(n);
    for (&(prev, a), &(next, b)) in befores.zip(sides.iter()) {
        let at = next.segment.start();
        let end_a = moved_end(&prev.segment, a, true);
        let start_b = moved_end(&next.segment, b, false);
        // Tangent sides move together: their moved ends coincide, and that is the corner.
        let corner = if end_a.dist(start_b) <= tol::LINEAR {
            start_b
        } else {
            nearest_meet(a, b, at).ok_or_else(|| Error::Unsupported("two drafted sides that no longer meet near their corner".into()))?
        };
        // A corner that moves further than the sides do, by many times, is a near-tangent meet
        // the offset cannot place well.
        if corner.dist(at) > 1e3 * d.abs().max(tol::LINEAR) {
            return decline("a drafted corner whose sides meet almost tangent");
        }
        corners.push(corner);
    }
    let mut out = Vec::with_capacity(n);
    // Segment k runs from corner k to corner k + 1.
    for ((&(s, carrier), &from), &to) in sides.iter().zip(&corners).zip(corners.iter().cycle().skip(1)) {
        let segment = match (s.segment.clone(), carrier) {
            (Segment::Line { a, b }, Carrier::Line { .. }) => {
                // The moved side must run the way the side did, and not shrink to nothing.
                if (to - from).dot(b - a) <= tol::LINEAR * (b - a).norm() {
                    return decline("a drafted side that shrinks to nothing or turns back");
                }
                Segment::Line { a: from, b: to }
            }
            (Segment::Arc { sweep, .. }, Carrier::Circle { c, r }) => {
                let a0 = angle(from - c);
                let a1 = angle(to - c);
                // The same turning sense and about the same turn as the arc it moved from.
                let mut sw = a1 - a0;
                if sweep > 0.0 {
                    while sw <= 0.0 {
                        sw += TAU;
                    }
                } else {
                    while sw >= 0.0 {
                        sw -= TAU;
                    }
                }
                if (sw - sweep).abs() > 0.5 * TAU / 2.0 || sw.abs() <= tol::ANGULAR {
                    return decline("a drafted arc that vanishes or turns back");
                }
                Segment::Arc { center: c, radius: r, start: a0, sweep: sw }
            }
            _ => return decline("a drafted extrusion of a segment other than a line, an arc or a circle"),
        };
        out.push(NamedSegment { segment, names: s.names.clone() });
    }
    Ok(ProfileLoop { segments: out })
}

/// The carrier of segment `s` moved `d` to its left.
fn moved(s: &Segment, d: f64) -> Result<Carrier> {
    match *s {
        Segment::Line { a, b } => {
            let Some(t) = unit(b - a) else { return decline("a drafted side of no length") };
            let left = Vec2::new(-t.y, t.x);
            Ok(Carrier::Line { p: a + left * d, t })
        }
        Segment::Arc { center, radius, sweep, .. } => {
            // A counter-clockwise arc has its centre to its left: moving left shrinks it.
            let r = if sweep > 0.0 { radius - d } else { radius + d };
            if r.is_nan() || r <= tol::LINEAR {
                return decline("a drafted arc whose radius closes before the top");
            }
            Ok(Carrier::Circle { c: center, r })
        }
        _ => decline("a drafted extrusion of a segment other than a line, an arc or a circle"),
    }
}

/// Segment `s`'s end (or start) moved onto its carrier `c`.
fn moved_end(s: &Segment, c: Carrier, end: bool) -> Vec2 {
    let p = if end { s.end() } else { s.start() };
    match c {
        Carrier::Line { p: q, t } => q + t * (p - q).dot(t),
        Carrier::Circle { c, r } => c + unit(p - c).unwrap_or(Vec2::new(1.0, 0.0)) * r,
    }
}

/// `v` scaled to length 1; `None` for a vector of no length.
fn unit(v: Vec2) -> Option<Vec2> {
    let n = v.norm();
    (n > 0.0 && n.is_finite()).then(|| v * (1.0 / n))
}

fn angle(v: Vec2) -> f64 {
    v.y.atan2(v.x)
}

/// Where carriers `a` and `b` meet nearest `near`; `None` where they do not.
fn nearest_meet(a: Carrier, b: Carrier, near: Vec2) -> Option<Vec2> {
    let pts: Vec<Vec2> = match (a, b) {
        (Carrier::Line { p, t }, Carrier::Line { p: q, t: u }) => {
            let det = t.x * u.y - t.y * u.x;
            if det.abs() <= tol::ANGULAR {
                return None;
            }
            let w = q - p;
            let s = (w.x * u.y - w.y * u.x) / det;
            vec![p + t * s]
        }
        (Carrier::Line { p, t }, Carrier::Circle { c, r }) | (Carrier::Circle { c, r }, Carrier::Line { p, t }) => {
            // |p + t·s − c| = r: s² + 2s·t·(p − c) + |p − c|² − r² = 0.
            let w = p - c;
            let (bq, cq) = (t.dot(w), w.dot(w) - r * r);
            let disc = bq * bq - cq;
            if disc < 0.0 {
                return None;
            }
            let root = disc.sqrt();
            vec![p + t * (-bq - root), p + t * (-bq + root)]
        }
        (Carrier::Circle { c, r }, Carrier::Circle { c: e, r: q }) => {
            let dv = e - c;
            let dist = dv.norm();
            if dist <= tol::LINEAR || dist > r + q || dist < (r - q).abs() {
                return None;
            }
            let along = (r * r - q * q + dist * dist) / (2.0 * dist);
            let h = (r * r - along * along).max(0.0).sqrt();
            let (u, v) = (dv * (1.0 / dist), Vec2::new(-dv.y / dist, dv.x / dist));
            vec![c + u * along + v * h, c + u * along - v * h]
        }
    };
    pts.into_iter().min_by(|x, y| x.dist(near).total_cmp(&y.dist(near)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;
    use std::f64::consts::PI;

    fn world() -> Frame {
        Frame::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).unwrap()
    }

    fn close(v: f64, want: f64) {
        assert!((v - want).abs() <= 1e-9 * want.abs().max(1.0), "{v} against {want}");
    }

    #[test]
    fn a_drafted_square_is_a_frustum() {
        // 10 × 10 up 10 leaning in 5°: the top side 10 − 20·tan 5°; h/3·(A + √(A·a) + a).
        let (h, d) = (10.0, 5f64.to_radians());
        let top = 10.0 - 2.0 * h * d.tan();
        let f = extrude_drafted(&world(), &Profile::rect(Vec2::new(0.0, 0.0), 10.0, 10.0), h, d, false, "x").unwrap();
        close(crate::measure::volume(&f), h / 3.0 * (100.0 + 10.0 * top + top * top));
    }

    #[test]
    fn a_drafted_circle_is_a_cone_frustum() {
        // Radius 4 up 6 leaning out 10°: the top radius 4 + 6·tan 10°; πh/3·(r² + r·r' + r'²).
        let (h, d) = (6.0, -10f64.to_radians());
        let r1 = 4.0 - h * d.tan();
        let f = extrude_drafted(&world(), &Profile::circle(Vec2::new(1.0, 2.0), 8.0), h, d, false, "x").unwrap();
        close(crate::measure::volume(&f), PI * h / 3.0 * (16.0 + 4.0 * r1 + r1 * r1));
    }

    #[test]
    fn no_draft_is_the_prism() {
        let p = Profile::rect(Vec2::new(0.0, 0.0), 3.0, 2.0);
        let f = extrude_drafted(&world(), &p, 4.0, 0.0, false, "x").unwrap();
        close(crate::measure::volume(&f), 24.0);
        let both = extrude_drafted(&world(), &p, 4.0, 0.0, true, "x").unwrap();
        close(crate::measure::volume(&both), 48.0);
    }

    #[test]
    fn a_drafted_slot_follows_its_offset_area() {
        // A stadium of straight length 3 and radius 1 leaning in 3° up 8: its sides move in by
        // s = z·tan 3° at height z, the area A − P·s + π·s² of a convex region with smooth turns,
        // so the volume is A·h − P·t·h²/2 + π·t²·h³/3 with t = tan 3°.
        let slot = Profile {
            loops: vec![ProfileLoop {
                segments: vec![
                    NamedSegment::new(Segment::Line { a: Vec2::new(-1.5, -1.0), b: Vec2::new(1.5, -1.0) }, "a"),
                    NamedSegment::new(Segment::Arc { center: Vec2::new(1.5, 0.0), radius: 1.0, start: -PI / 2.0, sweep: PI }, "b"),
                    NamedSegment::new(Segment::Line { a: Vec2::new(1.5, 1.0), b: Vec2::new(-1.5, 1.0) }, "c"),
                    NamedSegment::new(Segment::Arc { center: Vec2::new(-1.5, 0.0), radius: 1.0, start: PI / 2.0, sweep: PI }, "d"),
                ],
            }],
        };
        let (h, t) = (8.0, 3f64.to_radians().tan());
        let (a, p) = (6.0 + PI, 6.0 + 2.0 * PI);
        let f = extrude_drafted(&world(), &slot, h, 3f64.to_radians(), false, "x").unwrap();
        close(crate::measure::volume(&f), a * h - p * t * h * h / 2.0 + PI * t * t * h * h * h / 3.0);
        // Each side keeps its segment's name.
        let named = |n: &str| f.face_ids().filter(|&g| f.face(g).names.iter().any(|x| x.face == n)).count();
        assert_eq!((named("a"), named("b"), named("c"), named("d")), (1, 1, 1, 1));
        assert_eq!((named("start"), named("end")), (1, 1));
    }

    #[test]
    fn a_draft_that_closes_the_top_is_declined() {
        // 2 × 2 up 10 leaning in 10°: the sides move in 1.76 each, past the half-width 1.
        let r = extrude_drafted(&world(), &Profile::rect(Vec2::new(0.0, 0.0), 2.0, 2.0), 10.0, 10f64.to_radians(), false, "x");
        assert!(matches!(r, Err(Error::Unsupported(_))), "{r:?}");
    }
}
