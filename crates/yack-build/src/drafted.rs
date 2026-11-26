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
    Error, Result, check,
    geom::{Cone, Curve, Line, Plane, Surface},
    math::{Frame, Point3, Vec2},
    profile::{NamedSegment, PlacedSegment, Profile, ProfileLoop, Segment},
    tol,
    topo::{Brep, CoedgeSpec, EdgeId, FaceName, PCurve, VertexId},
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
    let top = offset(outer, inward)?;
    let lift = |d: f64| frame.translated(frame.z * d);
    let rings = if both_ways {
        vec![(lift(-height), top.clone()), (*frame, outer.clone()), (lift(height), top)]
    } else {
        vec![(*frame, outer.clone()), (lift(height), top)]
    };
    build(&rings, feature)
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
    let mut tangents = Vec::with_capacity(n);
    for (&(prev, a), &(next, b)) in befores.zip(sides.iter()) {
        let at = next.segment.start();
        let end_a = moved_end(&prev.segment, a, true);
        let start_b = moved_end(&next.segment, b, false);
        // Tangent sides move together: their moved ends coincide, and that is the corner.
        let tangent = end_a.dist(start_b) <= tol::LINEAR;
        tangents.push(tangent);
        let corner = if tangent {
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
    let ends = tangents.iter().zip(tangents.iter().cycle().skip(1));
    for (((&(s, carrier), &from), &to), (&smooth_in, &smooth_out)) in sides.iter().zip(&corners).zip(corners.iter().cycle().skip(1)).zip(ends) {
        // A drafted arc is a cone only between rulings: where it meets its neighbours tangent, its
        // moved ends stay on its own radii. At a sharp corner the cone meets the next side in a
        // conic, not along the straight rise this builds.
        if matches!(s.segment, Segment::Arc { .. }) && !(smooth_in && smooth_out) {
            return decline("a drafted arc that meets a neighbour at a sharp corner");
        }
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

/// The solid between `rings` (each a frame and a counter-clockwise loop on it, every loop with the
/// same segments in order, their corners joined by straight rises): a side face for each segment
/// in each band between two rings, a plane through a line's two edges or the exact cone through an
/// arc's, caps on the first and last rings.
fn build(rings: &[(Frame, ProfileLoop)], feature: &str) -> Result<Brep> {
    let (Some((first, _)), Some((last, _))) = (rings.first(), rings.last()) else {
        return decline("a drafted extrusion with no sections");
    };
    let n = first.z;
    let mut b = Brep::new();
    let lo = b.add_face(Surface::Plane(Plane { frame: *first }), true, [FaceName::new(feature, FaceName::START)]);
    let hi = b.add_face(Surface::Plane(Plane { frame: *last }), false, [FaceName::new(feature, FaceName::END)]);
    let mut faces = vec![lo, hi];
    // Each ring's placed segments, the corners where each starts, and its edges.
    let mut placed: Vec<Vec<PlacedSegment>> = Vec::with_capacity(rings.len());
    let mut corners: Vec<Vec<Point3>> = Vec::with_capacity(rings.len());
    let mut edges: Vec<Vec<EdgeId>> = Vec::with_capacity(rings.len());
    for (frame, lp) in rings {
        let segs: Vec<PlacedSegment> = lp.segments.iter().map(|s| s.segment.place(frame)).collect();
        let pts: Vec<Point3> = segs.iter().map(|s| s.curve.eval(traversal(s).0)).collect();
        let vs: Vec<VertexId> = pts.iter().map(|&p| b.add_vertex(p)).collect();
        let es: Vec<EdgeId> = segs
            .iter()
            .zip(vs.iter().zip(vs.iter().cycle().skip(1)))
            .map(|(s, (&v0, &v1))| {
                let (a, z) = if s.forward { (v0, v1) } else { (v1, v0) };
                b.add_edge(s.curve.clone(), s.range, a, z)
            })
            .collect();
        placed.push(segs);
        corners.push(pts);
        edges.push(es);
    }
    let names: Vec<&crate::profile::Names> = rings.first().map(|(_, lp)| lp.segments.iter().map(|s| &s.names).collect()).unwrap_or_default();
    // Each band: its rises (corner k of one ring to corner k of the next), then its side faces.
    for band in 0..rings.len().saturating_sub(1) {
        let (Some(below), Some(above), Some(lo_pts), Some(hi_pts)) = (placed.get(band), placed.get(band + 1), corners.get(band), corners.get(band + 1)) else {
            return decline("a drafted extrusion's rings do not pair");
        };
        let (Some(lo_e), Some(hi_e)) = (edges.get(band), edges.get(band + 1)) else {
            return decline("a drafted extrusion's rings do not pair");
        };
        let mut rises = Vec::with_capacity(lo_pts.len());
        for (&p, &q) in lo_pts.iter().zip(hi_pts) {
            let Some((line, len)) = Line::through(p, q) else { return decline("a drafted rise of no length") };
            let (Some(v0), Some(v1)) = (vertex_at(&b, lo_e, below, p), vertex_at(&b, hi_e, above, q)) else {
                return decline("a drafted rise without its corners");
            };
            rises.push((b.add_edge(Curve::Line(line.clone()), (0.0, len), v0, v1), line));
        }
        let sides = below.iter().zip(above).zip(lo_e.iter().zip(hi_e)).zip(rises.iter().zip(rises.iter().cycle().skip(1))).zip(&names);
        for ((((s, t), (&bottom_edge, &top_edge)), ((rise, rl), (next_rise, nl))), name) in sides {
            let (rise, next_rise) = (*rise, *next_rise);
            let mut fnames = vec![FaceName::new(feature, FaceName::SIDE)];
            fnames.extend(FaceName::all(feature, name.iter().filter(|x| !x.is_empty())));
            let (ta, tb) = traversal(s);
            let (face, loop_) = match (&s.curve, &t.curve) {
                (Curve::Line(l), Curve::Line(_)) => {
                    // The plane through the bottom line and its rise, its natural normal outward.
                    let up = rl.dir;
                    let Some(z) = l.dir.cross(up).unit() else { return decline("a drafted side that is not a plane") };
                    let x = l.dir;
                    let frame = Frame { origin: l.origin, x, y: z.cross(x), z };
                    // Outward is to the right of the loop's travel, away from its left (the inside).
                    let travel = if s.forward { l.dir } else { -l.dir };
                    let outward = travel.cross(n);
                    let reversed = frame.z.dot(outward) < 0.0;
                    let face = b.add_face(Surface::Plane(Plane { frame }), reversed, fnames);
                    let at = |c: &Curve, s0: f64| c.eval(s0);
                    let lp = vec![
                        coedge(bottom_edge, !s.forward, plane_pcurve(&frame, &s.curve, at(&s.curve, ta))),
                        coedge(next_rise, false, plane_pcurve(&frame, &Curve::Line(nl.clone()), nl.origin)),
                        coedge(top_edge, t.forward, plane_pcurve(&frame, &t.curve, at(&t.curve, traversal(t).0))),
                        coedge(rise, true, plane_pcurve(&frame, &Curve::Line(rl.clone()), rl.origin)),
                    ];
                    (face, lp)
                }
                (Curve::Circle(c), Curve::Circle(c1)) => {
                    // The cone on the arc's own frame: its u is the arc's parameter, its v the
                    // length along a ruling, up the band's height h from radius r0 to r1.
                    let h = (c1.frame.origin - c.frame.origin).dot(c.frame.z);
                    if h <= tol::LINEAR {
                        return decline("a drafted band of no height");
                    }
                    let half_angle = ((c1.radius - c.radius) / h).atan();
                    let slant = h / half_angle.cos();
                    let cone = Cone { frame: c.frame, radius: c.radius, half_angle };
                    let face = b.add_face(Surface::Cone(cone), !s.forward, fnames);
                    let lp = vec![
                        coedge(bottom_edge, !s.forward, affine(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0))),
                        coedge(next_rise, false, affine(Vec2::new(tb, 0.0), Vec2::new(0.0, 1.0))),
                        coedge(top_edge, s.forward, affine(Vec2::new(0.0, slant), Vec2::new(1.0, 0.0))),
                        coedge(rise, true, affine(Vec2::new(ta, 0.0), Vec2::new(0.0, 1.0))),
                    ];
                    (face, lp)
                }
                _ => return decline("a drafted side that is neither a plane nor a cone"),
            };
            b.add_loop(face, loop_);
            faces.push(face);
        }
    }
    // The caps: the first ring's edges against its travel, the last ring's along it.
    if let (Some(segs), Some(es)) = (placed.first(), edges.first()) {
        let lp: Vec<CoedgeSpec> = segs.iter().zip(es).rev().map(|(s, &e)| coedge(e, s.forward, plane_pcurve(first, &s.curve, s.curve.eval(traversal(s).1)))).collect();
        b.add_loop(lo, lp);
    }
    if let (Some(segs), Some(es)) = (placed.last(), edges.last()) {
        let lp: Vec<CoedgeSpec> = segs.iter().zip(es).map(|(s, &e)| coedge(e, !s.forward, plane_pcurve(last, &s.curve, s.curve.eval(traversal(s).0)))).collect();
        b.add_loop(hi, lp);
    }
    let shell = b.add_shell(faces);
    b.add_solid(vec![shell]);
    let issues = check::check(&b);
    if issues.is_empty() { Ok(b) } else { Err(Error::InvalidResult(issues.into_iter().map(|i| i.to_string()).collect())) }
}

/// The vertex of ring `es` at point `p`: the start of the edge whose segment starts there.
fn vertex_at(b: &Brep, es: &[EdgeId], segs: &[PlacedSegment], p: Point3) -> Option<VertexId> {
    es.iter().zip(segs).find_map(|(&e, s)| {
        let edge = b.edge(e);
        let v = if s.forward { edge.start } else { edge.end };
        (b.vertex(v).point.dist(p) <= tol::LINEAR).then_some(v)
    })
}

/// The parameters a placed segment starts and ends at, in the loop's direction.
fn traversal(s: &PlacedSegment) -> (f64, f64) {
    if s.forward { s.range } else { (s.range.1, s.range.0) }
}

fn affine(origin: Vec2, dir: Vec2) -> PCurve {
    PCurve::Affine { origin, dir }
}

/// The pcurve of `curve` on the plane of `frame`, starting at `start`.
fn plane_pcurve(frame: &Frame, curve: &Curve, start: Point3) -> PCurve {
    let uv = |p: Point3| {
        let l = frame.to_local(p);
        Vec2::new(l.x, l.y)
    };
    match curve {
        Curve::Line(l) => affine(uv(l.origin), Vec2::new(l.dir.dot(frame.x), l.dir.dot(frame.y))),
        _ => PCurve::Projected { seed: uv(start) },
    }
}

fn coedge(edge: EdgeId, reversed: bool, pcurve: PCurve) -> CoedgeSpec {
    CoedgeSpec { edge, reversed, pcurve }
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
    fn a_drafted_rectangles_sides_are_planes_both_ways() {
        // PROBE-ish check (w-m11): the kinds of the faces a drafted rectangle gets, one way and both.
        for both in [false, true] {
            let f = extrude_drafted(&world(), &Profile::rect(Vec2::new(11.5, 0.0), 3.0, 4.0), 4.0, 2f64.to_radians(), both, "x").unwrap();
            let kinds: Vec<&str> = f.face_ids().map(|g| f.face(g).surface.kind()).collect();
            eprintln!("KINDS both {both}: {kinds:?}");
            assert!(kinds.iter().all(|k| *k == "plane"), "both {both}: {kinds:?}");
        }
    }

    #[test]
    fn a_draft_that_closes_the_top_is_declined() {
        // 2 × 2 up 10 leaning in 10°: the sides move in 1.76 each, past the half-width 1.
        let r = extrude_drafted(&world(), &Profile::rect(Vec2::new(0.0, 0.0), 2.0, 2.0), 10.0, 10f64.to_radians(), false, "x");
        assert!(matches!(r, Err(Error::Unsupported(_))), "{r:?}");
    }
}
