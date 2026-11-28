// yack: a profile carried along a polyline and mitred at every corner: one prism piece per run, each cut by the planes bisecting its corners.
//! [`sweep_mitered`]: `profile` (drawn on `frame`, its z along the first run) carried along the
//! polyline through `corners`, back to the first with `closed`. Each run is a prism piece, its
//! section the profile carried by a rotation-minimising frame (at each corner the frame turns
//! about `d_in × d_out` by the angle between them) and cut at each end by the corner's mitre
//! plane, the plane through the corner whose normal bisects `d_in` and `d_out`. An open path ends
//! square to its first and last runs.
//!
//! Why the pieces meet exactly: the reflection S in a mitre plane takes `d_in` to `−d_out`, fixes
//! `a = d_in × d_out`, and so takes `b = a × d_in` to `−(a × −d_out) = a × d_out`, which is where
//! the frame's rotation R takes it too. S and R agree on the section plane, so the outgoing prism
//! is the incoming one reflected, and both cut the mitre plane, which S fixes, in the same curve:
//! a line for a line segment, and for an arc the ellipse of semi-axes `r/(d·m)` and `r`.
//!
//! Why the pieces do not overlap: each piece lies between its two mitre planes, so neighbours lie
//! on opposite sides of the plane they share. A piece's length along the run at a section point,
//! `t_end − t_start`, is affine in the point, so it is positive over the section when it is
//! positive at the vertices and at each arc's circle's nearest point. Pieces further apart than
//! neighbours are disjoint when their runs are further apart than the sum of their reaches: a
//! piece lies within `ρ/cos(θ/2)` of its run, `ρ` the section's reach from the path and `θ` the
//! larger turn at the run's ends.
//!
//! Faces: `start` and `end` (an open path's square ends) and, for each run and profile segment,
//! `side` with the names of the segment: a plane for a line, a cylinder for an arc.

use crate::{
    Error, Result,
    build::{PartFace, Parts, solid_of},
    geom::{Circle, Curve, Cylinder, Ellipse, Line, Plane, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    profile::{Profile, Segment},
    tol,
    topo::{Brep, FaceName},
};

fn decline<T>(why: &str) -> Result<T> {
    Err(Error::Unsupported(why.into()))
}

/// `items[i]`; an index past the end (a bug: every index is built from the lists' own lengths)
/// declines, so the next rung answers.
fn at<T: Copy>(items: &[T], i: usize) -> Result<T> {
    items.get(i).copied().map_or_else(|| decline("a mitred sweep's index past its list"), Ok)
}

/// A profile segment as the sweep needs it: its start in the section, and its circle (centre,
/// radius, counter-clockwise) when it is an arc.
struct Seg {
    start: Vec2,
    arc: Option<(Vec2, f64, bool)>,
}

/// `v` turned by the least rotation taking unit `from` to unit `to` (not opposite).
fn turned(v: Vec3, from: Vec3, to: Vec3) -> Vec3 {
    let k = from.cross(to);
    let c = from.dot(to);
    let s2 = k.dot(k);
    v * c + k.cross(v) + k * (k.dot(v) * (1.0 - c) / s2)
}

/// The least distance between segments `[p0, p1]` and `[q0, q1]`.
fn segment_gap(p0: Point3, p1: Point3, q0: Point3, q1: Point3) -> f64 {
    let (d1, d2, r) = (p1 - p0, q1 - q0, p0 - q0);
    let (a, e, f) = (d1.dot(d1), d2.dot(d2), d2.dot(r));
    let (c, b) = (d1.dot(r), d1.dot(d2));
    let den = a * e - b * b;
    let mut s = if den > 0.0 { ((b * f - c * e) / den).clamp(0.0, 1.0) } else { 0.0 };
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    (p0 + d1 * s).dist(q0 + d2 * t)
}

/// `profile` swept along `corners` (closed back to the first with `closed`) and mitred at each
/// corner, named under `feature`. Declines (`Unsupported`):
/// - a profile with holes, or a segment other than a line, an arc or a whole circle;
/// - a run shorter than the linear tolerance, a corner with no turn, or one that doubles back;
/// - a closed path whose frame does not come back to itself (a path that is not planar may not);
/// - a piece whose mitres cross inside its section, and two pieces that may overlap.
pub fn sweep_mitered(frame: &Frame, profile: &Profile, corners: &[Point3], closed: bool, feature: &str) -> Result<Brep> {
    let [outline] = profile.loops.as_slice() else {
        return decline("a mitred sweep of a profile with holes");
    };
    let mut outline = outline.clone();
    if outline.signed_area2() < 0.0 {
        outline.reverse();
    }
    let segs = outline
        .segments
        .iter()
        .map(|s| match s.segment {
            Segment::Line { a, .. } => Ok(Seg { start: a, arc: None }),
            Segment::Arc { center, radius, sweep, .. } => Ok(Seg { start: s.segment.start(), arc: Some((center, radius, sweep > 0.0)) }),
            Segment::Circle { center, radius, ccw } => Ok(Seg { start: s.segment.start(), arc: Some((center, radius, ccw)) }),
            _ => decline("a mitred sweep of a segment other than a line, an arc or a circle"),
        })
        .collect::<Result<Vec<_>>>()?;
    let ns = segs.len();
    let nc = corners.len();
    if nc < 2 || (closed && nc < 3) {
        return decline("a mitred path needs two corners, three when closed");
    }
    let runs = if closed { nc } else { nc - 1 };
    let end_of = |i: usize| (i + 1) % nc;
    let corner = |k: usize| at(corners, k);
    // each run's direction and frame
    let mut dirs = Vec::with_capacity(runs);
    for i in 0..runs {
        let (a, b) = (corner(i)?, corner(end_of(i))?);
        let Some(d) = (b - a).unit().filter(|_| a.dist(b) > tol::LINEAR) else {
            return decline("a mitred run shorter than the linear tolerance");
        };
        dirs.push(d);
    }
    let mut frames = vec![(frame.x, frame.y)];
    for pair in dirs.windows(2) {
        let (Some(&(x, y)), [from, to]) = (frames.last(), pair) else {
            return decline("a mitred sweep's frames out of step");
        };
        frames.push((turned(x, *from, *to), turned(y, *from, *to)));
    }
    let dir = |i: usize| at(&dirs, i);
    let frame_of = |i: usize| at(&frames, i);
    // each corner's mitre normal
    let mut mitres = Vec::with_capacity(nc);
    for k in 0..nc {
        let open_end = !closed && (k == 0 || k == nc - 1);
        let (din, dout) = match (closed, k) {
            (false, 0) => (dir(0)?, dir(0)?),
            (false, k) if k == nc - 1 => (dir(k - 1)?, dir(k - 1)?),
            (_, k) => (dir((k + runs - 1) % runs)?, dir(k % runs)?),
        };
        if !open_end && din.cross(dout).norm() <= tol::ANGULAR {
            return decline("a mitred corner with no turn");
        }
        let Some(m) = (din + dout).unit().filter(|m| m.dot(din) > tol::ANGULAR) else {
            return decline("a mitred corner that doubles back");
        };
        mitres.push(m);
    }
    let mitre = |k: usize| at(&mitres, k);
    if closed {
        let (x, _) = frame_of(runs - 1)?;
        if turned(x, dir(runs - 1)?, dir(0)?).dist(frame.x) > tol::ANGULAR {
            return decline("a closed mitred path whose frame does not come back to itself");
        }
    }
    // the section point `p` of run `i` on corner `k`'s mitre plane, and how far along the run
    let on_plane = |i: usize, k: usize, p: Vec2| -> Result<(Point3, f64)> {
        let ((x, y), d, m) = (frame_of(i)?, dir(i)?, mitre(k)?);
        let q = corner(i)? + x * p.x + y * p.y;
        let t = (corner(k)? - q).dot(m) / d.dot(m);
        Ok((q + d * t, t))
    };
    // every piece is longer than the tolerance across its whole section
    for i in 0..runs {
        let len = |p: Vec2| -> Result<f64> { Ok(on_plane(i, end_of(i), p)?.1 - on_plane(i, i, p)?.1) };
        let l0 = len(Vec2::ZERO)?;
        let grad = Vec2::new(len(Vec2::new(1.0, 0.0))? - l0, len(Vec2::new(0.0, 1.0))? - l0);
        let mut shortest = f64::INFINITY;
        for s in &segs {
            let l = match s.arc {
                None => len(s.start)?,
                Some((c, r, _)) => len(c)? - r * grad.norm(),
            };
            shortest = shortest.min(l);
        }
        if shortest <= tol::LINEAR {
            return decline("a mitred piece whose mitres cross inside its section");
        }
    }
    // pieces that are not neighbours are further apart than their reaches
    let reach = segs.iter().map(|s| s.arc.map_or(s.start.norm(), |(c, r, _)| c.norm() + r)).fold(0.0, f64::max);
    let piece_reach = |i: usize| -> Result<f64> {
        let d = dir(i)?;
        Ok(reach / d.dot(mitre(i)?).min(d.dot(mitre(end_of(i))?)))
    };
    for i in 0..runs {
        for j in i + 2..runs {
            if closed && i == 0 && j == runs - 1 {
                continue;
            }
            let gap = segment_gap(corner(i)?, corner(end_of(i))?, corner(j)?, corner(end_of(j))?);
            if gap <= piece_reach(i)? + piece_reach(j)? + tol::LINEAR {
                return decline("two mitred pieces that may overlap");
            }
        }
    }
    // the run each corner's section is taken from: its outgoing run, an open path's last its incoming
    let run_at = |k: usize| if k < runs { k } else { runs - 1 };
    let mut parts = Parts::default();
    for k in 0..nc {
        for s in &segs {
            parts.points.push(on_plane(run_at(k), k, s.start)?.0);
        }
    }
    let point = |k: usize, j: usize| k * ns + j % ns;
    // section edges, corner by corner: (edge, whether it runs against the profile)
    let mut sections = Vec::with_capacity(nc * ns);
    for k in 0..nc {
        let i = run_at(k);
        let ((x, y), d, m) = (frame_of(i)?, dir(i)?, mitre(k)?);
        for (j, s) in segs.iter().enumerate() {
            let (a, b) = (point(k, j), point(k, j + 1));
            let (curve, against) = match s.arc {
                None => {
                    let (pa, pb) = (at(&parts.points, a)?, at(&parts.points, b)?);
                    (Curve::Line(Line { origin: pa, dir: (pb - pa).unit_or_zero() }), false)
                }
                Some((c, r, ccw)) => {
                    let centre = on_plane(i, k, c)?.0;
                    let cos = d.dot(m);
                    let curve = if 1.0 - cos <= tol::ANGULAR {
                        Curve::Circle(Circle { frame: Frame { origin: centre, x, y, z: d }, radius: r })
                    } else {
                        let w = d.cross(m).unit_or_zero();
                        Curve::Ellipse(Ellipse { frame: Frame { origin: centre, x: w.cross(m), y: w, z: m }, major: r / cos, minor: r })
                    };
                    (curve, !ccw)
                }
            };
            sections.push((parts.edges.len(), against));
            parts.edges.push(if against { (curve, b, a) } else { (curve, a, b) });
        }
    }
    let section = |k: usize, j: usize| at(&sections, k * ns + j % ns);
    // the edges along each run, one per profile vertex
    let mut along = Vec::with_capacity(runs * ns);
    for i in 0..runs {
        let d = dir(i)?;
        for j in 0..ns {
            let (a, b) = (point(i, j), point(end_of(i), j));
            along.push(parts.edges.len());
            parts.edges.push((Curve::Line(Line { origin: at(&parts.points, a)?, dir: d }), a, b));
        }
    }
    let along_at = |i: usize, j: usize| at(&along, i * ns + j % ns);
    let face = |surface: Surface, reversed: bool, loop_: Vec<(usize, bool)>, names: Vec<FaceName>| PartFace {
        surface,
        reversed,
        loops: vec![loop_],
        names: names.into_iter().collect(),
        shell: 0,
    };
    let nexts = segs.iter().cycle().skip(1);
    for i in 0..runs {
        let ((x, y), d, c0) = (frame_of(i)?, dir(i)?, corner(i)?);
        for (j, ((s, next), named)) in segs.iter().zip(nexts.clone()).zip(&outline.segments).enumerate() {
            let ((lo, lo_against), (hi, hi_against)) = (section(i, j)?, section(end_of(i), j)?);
            let loop_ = vec![(lo, lo_against), (along_at(i, j + 1)?, false), (hi, !hi_against), (along_at(i, j)?, true)];
            let mut names = vec![FaceName::new(feature, FaceName::SIDE)];
            names.extend(FaceName::all(feature, named.names.iter().filter(|n| !n.is_empty())));
            let (surface, reversed) = match s.arc {
                None => {
                    let t = next.start - s.start;
                    let n = (x * t.y - y * t.x).unit_or_zero();
                    let origin = at(&parts.points, point(i, j))?;
                    (Surface::Plane(Plane { frame: Frame { origin, x: d, y: n.cross(d), z: n } }), false)
                }
                Some((c, r, ccw)) => (Surface::Cylinder(Cylinder { frame: Frame { origin: c0 + x * c.x + y * c.y, x, y, z: d }, radius: r }), !ccw),
            };
            parts.faces.push(face(surface, reversed, loop_, names));
        }
    }
    if !closed {
        let (x0, y0) = frame_of(0)?;
        let start = (0..ns).rev().map(|j| section(0, j).map(|(e, against)| (e, !against))).collect::<Result<Vec<_>>>()?;
        let plane0 = Plane { frame: Frame { origin: corner(0)?, x: x0, y: y0, z: dir(0)? } };
        parts.faces.push(face(Surface::Plane(plane0), true, start, vec![FaceName::new(feature, FaceName::START)]));
        let (xl, yl) = frame_of(runs - 1)?;
        let end = (0..ns).map(|j| section(nc - 1, j)).collect::<Result<Vec<_>>>()?;
        let plane1 = Plane { frame: Frame { origin: corner(nc - 1)?, x: xl, y: yl, z: dir(runs - 1)? } };
        parts.faces.push(face(Surface::Plane(plane1), false, end, vec![FaceName::new(feature, FaceName::END)]));
    }
    solid_of(&parts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::{NamedSegment, ProfileLoop};
    use std::f64::consts::PI;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    fn along_x() -> Frame {
        Frame { origin: Vec3::ZERO, x: Vec3::Y, y: Vec3::Z, z: Vec3::X }
    }

    fn swept(f: &Frame, pr: &Profile, c: &[Point3], closed: bool) -> Brep {
        let b = sweep_mitered(f, pr, c, closed, "m").unwrap();
        assert!(crate::check::check(&b).is_empty(), "{:?}", crate::check::check(&b));
        b
    }

    fn kinds(b: &Brep) -> (usize, usize) {
        let planes = b.face_ids().filter(|&f| matches!(b.face(f).surface, Surface::Plane(_))).count();
        let cylinders = b.face_ids().filter(|&f| matches!(b.face(f).surface, Surface::Cylinder(_))).count();
        (planes, cylinders)
    }

    #[test]
    fn a_square_bar_round_a_corner_holds_its_section_times_the_path() {
        // A piece's length at a section point is affine in the point, so the piece holds the
        // section's area times its length at the centroid: on the path, the run's length.
        // 4 × 4 along 20 + 30: 16·50.
        let b = swept(&along_x(), &Profile::rect(Vec2::ZERO, 4.0, 4.0), &[p(0.0, 0.0, 0.0), p(20.0, 0.0, 0.0), p(20.0, 30.0, 0.0)], false);
        let v = crate::measure::volume(&b);
        assert!((v - 800.0).abs() <= 1e-9 * 800.0, "{v}");
        assert_eq!(kinds(&b), (10, 0));
    }

    #[test]
    fn a_round_pipe_on_three_axes_holds_pi_r_squared_times_the_path() {
        // r = 2 along 20 + 15 + 25: 4π·60; three cylinders, two caps, and ellipses at the mitres.
        let b = swept(&along_x(), &Profile::circle(Vec2::ZERO, 4.0), &[p(0.0, 0.0, 0.0), p(20.0, 0.0, 0.0), p(20.0, 15.0, 0.0), p(20.0, 15.0, 25.0)], false);
        let v = crate::measure::volume(&b);
        assert!((v - 240.0 * PI).abs() <= 1e-9 * 240.0 * PI, "{v}");
        assert_eq!(kinds(&b), (2, 3));
        assert!(b.edge_ids().any(|e| matches!(b.edge(e).curve, Some(Curve::Ellipse(_)))));
    }

    #[test]
    fn a_closed_frame_off_its_path_gains_twice_its_offset_per_right_angle() {
        // A 4 × 3 section from 2 to 6 outward of a 40 × 24 rectangle: at a corner turning by θ
        // the piece's length at the centroid, s̄ = 4 out, grows by s̄·tan(θ/2) at each end, so
        // V = A·(P + Σ 2·s̄·tan 45°) = 12·(128 + 8·4).
        let c = [p(20.0, -12.0, 0.0), p(-20.0, -12.0, 0.0), p(-20.0, 12.0, 0.0), p(20.0, 12.0, 0.0)];
        let f = Frame { origin: c[0], x: -Vec3::Y, y: Vec3::Z, z: -Vec3::X };
        let b = swept(&f, &Profile::rect(Vec2::new(4.0, 1.5), 4.0, 3.0), &c, true);
        let v = crate::measure::volume(&b);
        assert!((v - 1920.0).abs() <= 1e-9 * 1920.0, "{v}");
        assert_eq!(kinds(&b), (16, 0));
    }

    #[test]
    fn a_concave_arc_makes_a_reversed_cylinder() {
        // A 6 × 6 square less the quarter disc of radius 2 about its corner (3, 3) (a concave
        // arc), along 30 + 30, turning toward the profile's +x (world +y) about its y. A section
        // point at profile x = s is s·tan 45° shorter at each side of the corner, so
        // V = A·(60 − 2·x̄), x̄ the centroid's x: A·x̄ = −π·(3 − 8/(3π)), the quarter disc's
        // centroid 4r/(3π) in from its corner. V = 60·(36 − π) + 2·(3π − 8/3).
        let pts = [Vec2::new(-3.0, -3.0), Vec2::new(3.0, -3.0), Vec2::new(3.0, 1.0)];
        let segments = vec![
            NamedSegment::line(pts[0], pts[1], "a"),
            NamedSegment::line(pts[1], pts[2], "b"),
            NamedSegment::arc(Vec2::new(3.0, 3.0), 2.0, -PI / 2.0, -PI / 2.0, "c"),
            NamedSegment::line(Vec2::new(1.0, 3.0), Vec2::new(-3.0, 3.0), "d"),
            NamedSegment::line(Vec2::new(-3.0, 3.0), pts[0], "e"),
        ];
        let pr = Profile { loops: vec![ProfileLoop { segments }] };
        let b = swept(&along_x(), &pr, &[p(0.0, 0.0, 0.0), p(30.0, 0.0, 0.0), p(30.0, 30.0, 0.0)], false);
        let want = 60.0 * (36.0 - PI) + 2.0 * (3.0 * PI - 8.0 / 3.0);
        let v = crate::measure::volume(&b);
        assert!((v - want).abs() <= 1e-9 * want, "{v} {want}");
        assert_eq!(kinds(&b), (10, 2));
        assert!(b.face_ids().any(|f| matches!(b.face(f).surface, Surface::Cylinder(_)) && b.face(f).reversed));
    }

    #[test]
    fn it_declines_what_it_cannot_mitre_exactly() {
        let bar = Profile::rect(Vec2::ZERO, 10.0, 10.0);
        let un = |c: &[Point3], closed: bool| matches!(sweep_mitered(&along_x(), &bar, c, closed, "m"), Err(Error::Unsupported(_)));
        // a first run of 3 under a bar 10 wide turning a right angle: the inner side's mitre is behind its start
        assert!(un(&[p(0.0, 0.0, 0.0), p(3.0, 0.0, 0.0), p(3.0, 20.0, 0.0)], false));
        // a corner with no turn
        assert!(un(&[p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0), p(20.0, 0.0, 0.0)], false));
        // a corner that doubles back
        assert!(un(&[p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0), p(0.0, 0.0, 0.0)], false));
        // runs one and three of a U 14 apart, each within 5/cos 45° of its own: they may overlap
        assert!(un(&[p(0.0, 0.0, 0.0), p(40.0, 0.0, 0.0), p(40.0, 14.0, 0.0), p(0.0, 14.0, 0.0)], false));
        // a profile with a hole
        let holed = Profile::rect(Vec2::ZERO, 10.0, 10.0).with_holes(Profile::rect(Vec2::ZERO, 2.0, 2.0).loops);
        assert!(matches!(sweep_mitered(&along_x(), &holed, &[p(0.0, 0.0, 0.0), p(30.0, 0.0, 0.0)], false, "m"), Err(Error::Unsupported(_))));
    }
}
