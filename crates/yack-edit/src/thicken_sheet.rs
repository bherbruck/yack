// yack: a sheet of a solid's faces thickened where the mitring thicken stops: bends the move closes, round joins at the creases it opens.
//! A sheet of a solid's faces thickened into a slab of its own, past the two limits of the
//! mitring [`crate::thicken`]:
//!
//! - **Bends the move closes.** Thickened inward past a bend's radius (a round between two faces
//!   of the sheet), the bend's offset turns inside out. The points within the thickness of the
//!   sheet there are those within it of the sharp edge the bend rounded, so the sheet is moved with
//!   the bend taken out ([`crate::tweak::drop_vanishing_rounds`]) and the slab is the solid less
//!   that.
//! - **Round joins.** Thickened outward, the moved faces of two sheet faces that meet at a convex
//!   crease part. The round join closes the gap with every point within the thickness of the crease
//!   edge: a cylinder of radius `t` about the edge, tangent to both moved faces, so the moved
//!   solid's crease edge filleted at `t` (a ball's patch about a corner where three creases meet,
//!   likewise). That is the mitre's crease edges filleted at `t`.
//!
//! Every other case is the mitring thicken itself. Each step declines (`Unsupported`) outside what
//! its reasoning covers, for the next rung to try.

use std::collections::BTreeMap;
use std::f64::consts::{PI, TAU};

use crate::{
    blend, boolean, measure, sweep,
    geom::{Curve, Surface},
    error::{Error, Result},
    tol,
    topo::{self, Brep, EdgeId, FaceId, FaceName},
};

/// A step's refusal of its input, past this rung's own checks: a case the rung does not build.
fn declined(e: Error) -> Error {
    match e {
        Error::InvalidInput(m) => Error::Unsupported(format!("a thickened sheet: {m}")),
        e => e,
    }
}

/// The sheet `faces` of `s` thickened by `thickness` along its outward normal (negative: inward)
/// into a solid of its own, the creases it opens joined round when `round` (sharp otherwise), its
/// free rims swept square. Faces it makes are `<feature>.side`, the join rounds `<feature>.round_<k>`.
pub fn thicken_sheet(
    s: &Brep,
    faces: &[FaceId],
    thickness: f64,
    round: bool,
    feature: &str,
) -> Result<Brep> {
    if thickness.is_nan() || thickness.abs() <= tol::LINEAR {
        return Err(Error::InvalidInput(
            "a sheet must be thickened by more than the tolerance".into(),
        ));
    }
    if let Some(sector) = sector_to_its_centre(s, faces, thickness, feature) {
        return Ok(sector);
    }
    if let Some(prism) = prism_through(s, faces, thickness, feature) {
        return Ok(prism);
    }
    if let Some(turned) = turned_slab(s, faces, thickness, feature) {
        return Ok(turned);
    }
    // Past those, a free rim is swept along its neighbour face; where that neighbour meets the sheet
    // at a slant, the sweep is not along the sheet's normal.
    if !topo::local::rims_square(s, faces) {
        return Err(Error::Unsupported(
            "a thickened sheet: a rim meets its neighbour at a slant, which this rung would sweep \
             along the neighbour, not the sheet's normal"
                .into(),
        ));
    }
    let by: BTreeMap<FaceId, f64> = faces.iter().map(|&f| (f, thickness)).collect();
    // Bends the move closes are taken out of the solid the sheet is moved from; `kept` takes each
    // kept face of `s` to its face there.
    let (base, kept): (Brep, BTreeMap<FaceId, FaceId>) =
        match crate::tweak::drop_vanishing_rounds(s, &by).map_err(declined)? {
            Some((sharp, kept)) => (sharp, kept),
            None => (s.clone(), s.face_ids().map(|f| (f, f)).collect()),
        };
    let sheet: Vec<FaceId> = faces.iter().filter_map(|f| kept.get(f)).copied().collect();
    let by: BTreeMap<FaceId, f64> = sheet.iter().map(|&f| (f, thickness)).collect();
    let moved = crate::offset_faces(&base, &by).map_err(declined)?;
    let creases = opened_creases(&base, &sheet, thickness);
    let moved = if round && !creases.is_empty() {
        // The rounds are named for the crease edges of `s` they round, so `base` must be `s`.
        if base.face_ids().count() != s.face_ids().count() {
            return Err(Error::Unsupported(
                "a round join on a sheet whose bends the move also takes out".into(),
            ));
        }
        round_joins(&base, moved, &creases, thickness, feature)?
    } else {
        moved
    };
    let slab = if thickness > 0.0 {
        boolean::difference(&moved, s)?
    } else {
        boolean::difference(s, &moved)?
    };
    Ok(name_new(slab, feature))
}

/// A spherical sheet thickened inward by exactly its radius: every point between the sphere and
/// its centre, the sector the sphere's face spans. Where every other face of `s` is a plane
/// through the centre, `s` is that sector itself (a half ball thickened from its dome), and the
/// slab is `s`: its sphere the sheet, its planes the rims swept to the centre (`<feature>.side`).
/// `None` for every other case.
fn sector_to_its_centre(s: &Brep, faces: &[FaceId], thickness: f64, feature: &str) -> Option<Brep> {
    let &[f] = faces else { return None };
    let face = s.face(f);
    let Surface::Sphere(ball) = &face.surface else { return None };
    // A convex sphere (not a cavity) moved in by its radius.
    if face.reversed || (thickness + ball.radius).abs() > tol::LINEAR {
        return None;
    }
    let centre = ball.frame.origin;
    let through_centre = |g: FaceId| match blend::shaped::plane_of(s, g) {
        Ok((p, n)) => (centre - p).dot(n).abs() <= tol::LINEAR,
        Err(_) => false,
    };
    if !s.face_ids().filter(|&g| g != f).all(through_centre) {
        return None;
    }
    let mut out = s.clone();
    for g in out.face_ids().filter(|&g| g != f).collect::<Vec<_>>() {
        out.set_face_names(g, std::iter::once(FaceName::new(feature, "side")).collect());
    }
    Some(out)
}

/// A plane sheet thickened inward right through a prism on it: where every other face of `s` is
/// either the plane parallel to the sheet at the thickness behind it (the moved sheet's own place)
/// or a rim square to the sheet (a plane or a cylinder along its normal), the slab is `s` itself
/// (a plate thickened through its own thickness). The moved sheet would land on that opposite face,
/// which the face move cannot build. The opposite face is `<feature>.inner_f<sheet>`, the rims
/// `<feature>.side`. `None` for every other case.
fn prism_through(s: &Brep, faces: &[FaceId], thickness: f64, feature: &str) -> Option<Brep> {
    let &[f] = faces else { return None };
    if thickness >= 0.0 {
        return None;
    }
    let (origin, normal) = blend::shaped::plane_of(s, f).ok()?;
    let mut opposite = None;
    for g in s.face_ids().filter(|&g| g != f) {
        let square = match &s.face(g).surface {
            Surface::Plane(_) => {
                let (p, m) = blend::shaped::plane_of(s, g).ok()?;
                if m.dot(normal) < -1.0 + tol::ANGULAR
                    && ((origin - p).dot(normal) + thickness).abs() <= tol::LINEAR
                {
                    if opposite.replace(g).is_some() {
                        return None;
                    }
                    continue;
                }
                m.dot(normal).abs() <= tol::ANGULAR
            }
            Surface::Cylinder(c) => c.frame.z.cross(normal).norm() <= tol::ANGULAR,
            _ => false,
        };
        if !square {
            return None;
        }
    }
    let opposite = opposite?;
    let mut out = s.clone();
    for g in out.face_ids().filter(|&g| g != f).collect::<Vec<_>>() {
        let part = if g == opposite { format!("inner_f{}", f.index()) } else { "side".to_string() };
        out.set_face_names(g, std::iter::once(FaceName::new(feature, part)).collect());
    }
    Some(out)
}

/// A sheet face of revolution whose rims the move along its normal sweeps slantwise: a sphere band
/// or cap, or a cone band, bounded by whole circles about its axis. Its slab is the turn of its
/// section: the sheet's generatrix, its copy moved by `thickness` along the normal, and the two
/// normals joining their ends, which turn to cones (or planes) the faces square to the sheet. The
/// sheet's own and moved faces keep the sheet's names, the rims are `<feature>.side`. `None` for
/// every other case (a rim swept square, a cone reaching its apex, a move past the axis).
fn turned_slab(s: &Brep, faces: &[FaceId], thickness: f64, feature: &str) -> Option<Brep> {
    use crate::{
        math::{Frame, Point3, Vec2, Vec3},
        profile::{NamedSegment, Profile, ProfileLoop, Segment},
    };
    let &[f] = faces else { return None };
    if thickness.abs() <= tol::LINEAR {
        return None;
    }
    // The face's rims, each a whole circle; seams (used twice) and poles pass.
    let face = s.face(f);
    let mut used: BTreeMap<EdgeId, usize> = BTreeMap::new();
    for &l in &face.loops {
        for &c in &s.lp(l).coedges {
            *used.entry(s.coedge(c).edge).or_default() += 1;
        }
    }
    let mut rims = Vec::new();
    for (&e, &n) in &used {
        let edge = s.edge(e);
        if n > 1 || edge.is_degenerate() {
            continue;
        }
        let Some(Curve::Circle(c)) = &edge.curve else { return None };
        if ((edge.range.1 - edge.range.0).abs() - TAU).abs() > tol::LINEAR {
            return None;
        }
        rims.push(c.clone());
    }
    let (origin, axis, sphere) = match &face.surface {
        Surface::Sphere(sp) => (sp.frame.origin, rims.first()?.frame.z, Some(sp.radius)),
        Surface::Cone(cn) => (cn.frame.origin, cn.frame.z, None),
        _ => return None,
    };
    let on_axis = |p: Point3| {
        let d = p - origin;
        (d - axis * d.dot(axis)).norm() <= tol::LINEAR
    };
    // Each rim as (radius, height) along the axis.
    let mut section: Vec<Vec2> = Vec::new();
    for c in &rims {
        if c.frame.z.cross(axis).norm() > tol::ANGULAR || !on_axis(c.frame.origin) {
            return None;
        }
        section.push(Vec2::new(c.radius, (c.frame.origin - origin).dot(axis)));
    }
    section.sort_by(|p, q| p.y.total_cmp(&q.y));
    let samples = measure::face_samples(s, f, 8);
    let (p, n) = *samples.first()?;
    let d = p - origin;
    let h = d.dot(axis);
    let radial = (d - axis * h).unit()?;
    let normal = Vec2::new(n.dot(radial), n.dot(axis));
    let mut x = axis.cross(Vec3::new(1.0, 0.0, 0.0));
    if x.norm() < 0.5 {
        x = axis.cross(Vec3::new(0.0, 1.0, 0.0));
    }
    let x = x.unit()?;
    let plane = Frame::new(origin, x.cross(axis), x).ok()?;
    let line = |a: Vec2, b: Vec2, name: &str| NamedSegment::new(Segment::Line { a, b }, name);
    let segments = match sphere {
        Some(radius) => {
            // The band between the rims' latitudes, or from the one rim to the pole on the
            // sheet's side; a rim on the equator sweeps square and is not this case.
            if section.iter().all(|r| r.y.abs() <= tol::LINEAR) {
                return None;
            }
            let lat = |r: &Vec2| r.y.atan2(r.x);
            let (lo, hi) = match section.as_slice() {
                [r] => {
                    let above = samples.iter().all(|(q, _)| (*q - origin).dot(axis) > r.y);
                    let below = samples.iter().all(|(q, _)| (*q - origin).dot(axis) < r.y);
                    match (above, below) {
                        (true, false) => (lat(r), PI / 2.0),
                        (false, true) => (-PI / 2.0, lat(r)),
                        _ => return None,
                    }
                }
                [r0, r1] => (lat(r0), lat(r1)),
                _ => return None,
            };
            let moved = radius + if d.dot(n) > 0.0 { thickness } else { -thickness };
            if moved <= tol::LINEAR {
                return None;
            }
            let at = |r: f64, a: f64| Vec2::new(r * a.cos(), r * a.sin());
            let arc = |r: f64, start: f64, sweep: f64, name: &str| {
                NamedSegment::new(Segment::Arc { center: Vec2::new(0.0, 0.0), radius: r, start, sweep }, name)
            };
            vec![
                arc(radius, lo, hi - lo, "sheet"),
                line(at(radius, hi), at(moved, hi), "side"),
                arc(moved, hi, lo - hi, "moved"),
                line(at(moved, lo), at(radius, lo), "side"),
            ]
        }
        None => {
            // The band between its two rims (one rim means the cone reaches its apex).
            let &[p0, p1] = section.as_slice() else { return None };
            let g = p1 - p0;
            let len = g.norm();
            if len <= tol::LINEAR {
                return None;
            }
            let mut m = Vec2::new(g.y / len, -g.x / len);
            if m.dot(normal) < 0.0 {
                m = -m;
            }
            let (q0, q1) = (p0 + m * thickness, p1 + m * thickness);
            if q0.x <= tol::LINEAR || q1.x <= tol::LINEAR {
                return None;
            }
            vec![line(p0, p1, "sheet"), line(p1, q1, "side"), line(q1, q0, "moved"), line(q0, p0, "side")]
        }
    };
    let profile = Profile { loops: vec![ProfileLoop { segments }] };
    let mut out = sweep::revolve(&profile, &plane, origin, axis, TAU, feature).ok()?;
    let sheet: [FaceName; 2] = [FaceName::new(feature, "sheet"), FaceName::new(feature, "moved")];
    for g in out.face_ids().collect::<Vec<_>>() {
        if out.face(g).names.iter().any(|n| sheet.contains(n)) {
            out.set_face_names(g, face.names.clone());
        }
    }
    Some(out)
}

/// The edges between two faces of `sheet` that the move by `thickness` opens: convex ones moving
/// out, concave ones moving in.
fn opened_creases(b: &Brep, sheet: &[FaceId], thickness: f64) -> Vec<EdgeId> {
    let mut edges: Vec<EdgeId> = sheet
        .iter()
        .flat_map(|&f| {
            b.face(f)
                .loops
                .iter()
                .flat_map(|&lp| b.lp(lp).coedges.iter().map(|&c| b.coedge(c).edge))
        })
        .collect();
    edges.sort();
    edges.dedup();
    edges.retain(|&e| {
        let on: Vec<FaceId> = b.edge_uses(e).iter().map(|&c| b.coedge_face(c)).collect();
        matches!(*on.as_slice(), [f, g] if f != g && sheet.contains(&f) && sheet.contains(&g))
            && topo::local::convexity(b, e) == Some(thickness > 0.0)
    });
    edges
}

/// `moved` (`base` with the sheet moved out) with the opened `creases` of `base` rounded at
/// `thickness`: each crease edge's moved copy filleted at the thickness, so the round is the
/// cylinder about the crease edge itself. Only outward, and only where the move kept `base`'s
/// topology, so that the crease's moved copy has the crease's id.
fn round_joins(
    base: &Brep,
    moved: Brep,
    creases: &[EdgeId],
    thickness: f64,
    feature: &str,
) -> Result<Brep> {
    if thickness < 0.0 {
        return Err(Error::Unsupported("a round join of a sheet thickened inward".into()));
    }
    if !crate::same_topology(base, &moved) {
        return Err(Error::Unsupported(
            "a round join where the thickened sheet's moved faces change its topology".into(),
        ));
    }
    let mut out = blend::fillet_edges_mitred(&moved, creases, thickness, feature).map_err(declined)?;
    // The fillet names each round `round_<k>` for the k-th of `creases`; the round is the join about
    // that crease edge of the sheet, so it is named `round_e<id>` for it (an Edge origin).
    for f in out.face_ids().collect::<Vec<_>>() {
        let names = out.face(f).names.clone();
        let renamed: std::collections::BTreeSet<FaceName> = names
            .iter()
            .map(|n| {
                let crease = (n.feature == feature)
                    .then(|| n.face.strip_prefix("round_"))
                    .flatten()
                    .and_then(|k| k.parse::<usize>().ok())
                    .and_then(|k| creases.get(k));
                match crease {
                    Some(e) => FaceName::new(feature, format!("round_e{}", e.index())),
                    None => n.clone(),
                }
            })
            .collect();
        if renamed != names {
            out.set_face_names(f, renamed);
        }
    }
    Ok(out)
}

/// `b` with every face that has no name named `<feature>.side`: the faces carried over keep their
/// names, the rims' sweeps take this one.
fn name_new(mut b: Brep, feature: &str) -> Brep {
    let fresh: Vec<FaceId> = b.face_ids().filter(|&f| b.face(f).names.is_empty()).collect();
    for f in fresh {
        b.set_face_names(f, std::iter::once(FaceName::new(feature, "side")).collect());
    }
    b
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Frame, Vec2, Vec3};

    fn close(got: f64, want: f64) {
        assert!((got - want).abs() < 1e-7 * want.max(1.0), "{got} against {want}");
    }

    /// The faces of `b` whose boxes lie where `keep` says.
    fn faces_where(b: &Brep, keep: impl Fn(crate::math::Aabb) -> bool) -> Vec<FaceId> {
        b.face_ids().filter(|&f| keep(measure::face_bbox(b, f))).collect()
    }

    #[test]
    fn an_angle_bracket_thickened_round_rounds_its_crease_about_the_edge() {
        // bench thicken/angle-bracket-rounded: the block 10 × 8 × 10's floor and its side x = 0,
        // thickened 1 out: two slabs 10 × 8 × 1 and a quarter round of radius 1 about the crease,
        // 8 long: 168 − 8(1 − π/4).
        let b = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 8.0, 10.0), "b").unwrap();
        let sheet = faces_where(&b, |bb| bb.max.z < 1e-9 || bb.max.x < 1e-9);
        assert_eq!(sheet.len(), 2);
        let out = thicken_sheet(&b, &sheet, 1.0, true, "t").unwrap();
        close(measure::volume(&out), 168.0 - 8.0 * (1.0 - PI / 4.0));
        assert!(crate::check::check(&out).is_empty());
        assert!(out.face_ids().all(|f| !out.face(f).names.is_empty()));
        // The round is named for the crease edge of the block it rounds.
        assert!(out.face_ids().any(|f| out.face(f).names.iter().any(|n| n.face.starts_with("round_e"))));
        assert!(!out.face_ids().any(|f| out.face(f).names.iter().any(|n| {
            n.face.strip_prefix("round_").is_some_and(|k| k.parse::<usize>().is_ok())
        })));
    }

    #[test]
    fn an_open_box_thickened_round_gets_quarter_rounds_and_ball_corners() {
        // bench thicken/open-box-outward-rounded: the cube 10's floor and four sides thickened 1
        // out: five slabs of 100, eight quarter rounds of radius 1 and 10 long, four eighths of a
        // ball at the floor's corners: 500 + 20π + 2π/3.
        let b = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
        let sheet = faces_where(&b, |bb| bb.min.z < 10.0 - 1e-9);
        assert_eq!(sheet.len(), 5);
        let out = thicken_sheet(&b, &sheet, 1.0, true, "t").unwrap();
        close(measure::volume(&out), 500.0 + 20.0 * PI + 2.0 * PI / 3.0);
        assert!(crate::check::check(&out).is_empty());
    }

    #[test]
    fn a_cup_thickened_inward_past_its_bend_loses_the_bend_inside() {
        // bench thicken/cup-past-its-bend: a cylinder of radius 6 and height 10 whose floor's rim is
        // rounded to 2, its floor, bend and side thickened 3 in: the bend vanishes from the inside,
        // which is the sharp cylinder of radius 3 from z = 3 up: V = (the cup) − 63π.
        let puck = crate::primitive::cylinder(&Frame::WORLD, 6.0, 10.0, "c").unwrap();
        let floor_rim: Vec<EdgeId> = crate::convex_edges(&puck)
            .unwrap()
            .into_iter()
            .filter(|&e| puck.vertex(puck.edge(e).start).point.z < 1e-9)
            .collect();
        let cup = blend::fillet_edges(&puck, &floor_rim, 2.0, "r").unwrap();
        let sheet = faces_where(&cup, |bb| bb.min.z < 10.0 - 1e-9);
        let out = thicken_sheet(&cup, &sheet, -3.0, true, "t").unwrap();
        close(measure::volume(&out), measure::volume(&cup) - 63.0 * PI);
        close(measure::volume(&out), 903.1013080004373);
        assert!(crate::check::check(&out).is_empty());
    }

    #[test]
    fn a_dome_thickened_in_by_its_radius_fills_the_half_ball() {
        // bench thicken/hemisphere-to-its-center: the half ball of radius 5's dome thickened 5 in:
        // everything between the dome and its centre, the half ball itself, 2π·125/3.
        let ball = crate::primitive::sphere(&Frame::WORLD, 5.0, "s").unwrap();
        let (dome, _) = boolean::split(&ball, crate::math::Point3::ZERO, Vec3::new(0.0, 0.0, 1.0)).unwrap();
        let sphere = faces_where(&dome, |bb| bb.max.z > 1.0);
        assert_eq!(sphere.len(), 1);
        let out = thicken_sheet(&dome, &sphere, -5.0, true, "t").unwrap();
        close(measure::volume(&out), 2.0 * PI * 125.0 / 3.0);
        assert!(crate::check::check(&out).is_empty());
        assert!(out.face_ids().all(|f| !out.face(f).names.is_empty()));
        // Short of the centre, or past it, this case is not taken.
        assert!(sector_to_its_centre(&dome, &sphere, -4.0, "t").is_none());
    }

    #[test]
    fn a_plate_thickened_in_by_its_own_thickness_is_the_plate() {
        // bench thicken/plate-with-holes: the 20 × 20 plate 1 thick with two holes of radius 3, its
        // top thickened 1 up and 1 down: two slabs of 400 − 18π each, the lower one the plate.
        use crate::profile::{NamedSegment, Profile, ProfileLoop, Segment};
        let line = |a: [f64; 2], b: [f64; 2]| {
            NamedSegment::new(Segment::Line { a: Vec2::new(a[0], a[1]), b: Vec2::new(b[0], b[1]) }, "e")
        };
        let hole = |cx: f64| ProfileLoop {
            segments: vec![NamedSegment::new(
                Segment::Circle { center: Vec2::new(cx, 0.0), radius: 3.0, ccw: false },
                "h",
            )],
        };
        let corners = [[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]];
        let outer = ProfileLoop {
            segments: (0..4).map(|k| line(corners[k], corners[(k + 1) % 4])).collect(),
        };
        let profile = Profile { loops: vec![outer, hole(-5.0), hole(5.0)] };
        let at = Frame::new(Vec3::new(0.0, 0.0, -1.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let plate = crate::sweep::extrude(&profile, &at, 1.0, "p").unwrap();
        let top = faces_where(&plate, |bb| bb.min.z > -1e-9);
        let slab = 400.0 - 18.0 * PI;
        let down = thicken_sheet(&plate, &top, -1.0, true, "t").unwrap();
        close(measure::volume(&down), slab);
        assert!(down.face_ids().all(|f| !down.face(f).names.is_empty()));
        let up = thicken_sheet(&plate, &top, 1.0, true, "t").unwrap();
        close(measure::volume(&up), slab);
        close(measure::volume(&boolean::union(&up, &down).unwrap()), 2.0 * slab);
        // Less than the plate's thickness, the face move builds it; this case is not taken.
        assert!(prism_through(&plate, &top, -0.5, "t").is_none());
    }

    #[test]
    fn a_spherical_cap_thickens_inside_its_cone() {
        // The cap of the ball of radius 10 above z = 6 (cos α = 3/5, sin α = 4/5), its sphere
        // thickened 2 out: the shell between radii a = 10 and b = 12 inside the cone of half angle α.
        // Its volume is the shell's share of the solid angle, 2π(1 − cos α)(b³ − a³)/3; its area two
        // sphere caps 2π(1 − cos α)r² and the cone's band π sin α (b² − a²). Then 2 in: 8 to 10.
        let ball = crate::primitive::sphere(&Frame::WORLD, 10.0, "s").unwrap();
        let (cap, _) = boolean::split(&ball, Vec3::new(0.0, 0.0, 6.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        let sheet = faces_where(&cap, |bb| bb.max.z > 9.0);
        let shell = |a: f64, b: f64| 2.0 * PI * (1.0 - 0.6) * (b * b * b - a * a * a) / 3.0;
        let out = thicken_sheet(&cap, &sheet, 2.0, true, "t").unwrap();
        close(measure::volume(&out), shell(10.0, 12.0));
        close(measure::area(&out), 2.0 * PI * 0.4 * (144.0 + 100.0) + PI * 0.8 * (144.0 - 100.0));
        assert_eq!(out.face_ids().count(), 3);
        assert!(out.face_ids().all(|f| !out.face(f).names.is_empty()));
        let inward = thicken_sheet(&cap, &sheet, -2.0, true, "t").unwrap();
        close(measure::volume(&inward), shell(8.0, 10.0));
        // A hemisphere's rim sweeps square, a whole sphere has none: neither is this case.
        let (half, _) = boolean::split(&ball, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        assert!(turned_slab(&half, &faces_where(&half, |bb| bb.max.z > 9.0), 1.0, "t").is_none());
        assert!(turned_slab(&ball, &faces_where(&ball, |_| true), 1.0, "t").is_none());
    }

    #[test]
    fn a_cone_frustum_thickens_along_its_normal() {
        // The side of the frustum from radius 6 at z = 0 to 3 at z = 4 (slant 5, outward normal
        // (4/5, 3/5) in its section), thickened 1 out: the section is the 5 × 1 rectangle whose
        // centroid lies 4.5 + 0.4 from the axis, so by Pappus V = 2π·4.9·5; its area is each side's
        // length by 2π its midpoint's radius (4.5, 5.3, 6.4, 3.4). Thickened 1 in, the centroid is 4.1.
        let frustum = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 4.0, "c").unwrap();
        let side = faces_where(&frustum, |bb| bb.max.z - bb.min.z > 3.0);
        let out = thicken_sheet(&frustum, &side, 1.0, true, "t").unwrap();
        close(measure::volume(&out), 2.0 * PI * 4.9 * 5.0);
        close(measure::area(&out), 2.0 * PI * (4.5 * 5.0 + 5.3 * 5.0 + 6.4 + 3.4));
        assert_eq!(out.face_ids().count(), 4);
        assert!(out.face_ids().all(|f| !out.face(f).names.is_empty()));
        let inward = thicken_sheet(&frustum, &side, -1.0, true, "t").unwrap();
        close(measure::volume(&inward), 2.0 * PI * 4.1 * 5.0);
        // Moved past the axis at its narrow end: not this case, and its rims meet the caps at a
        // slant, so the rung declines it.
        assert!(turned_slab(&frustum, &side, -4.0, "t").is_none());
        assert!(matches!(thicken_sheet(&frustum, &side, -4.0, true, "t"), Err(Error::Unsupported(_))));
    }

    #[test]
    fn a_slanted_rim_that_is_not_a_turned_slab_declines() {
        // The sheets below meet a neighbour at a slant, so their rims would be swept along that
        // neighbour, not their normal, and none is a sphere or cone bounded by whole circles about
        // its axis: each declines, as the facade's check declined them before.
        // A frustum's flat top (its rim meets the cone at a slant).
        let frustum = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 4.0, "c").unwrap();
        let top = faces_where(&frustum, |bb| bb.min.z > 4.0 - 1e-9);
        assert!(!topo::local::rims_square(&frustum, &top));
        assert!(matches!(thicken_sheet(&frustum, &top, 1.0, true, "t"), Err(Error::Unsupported(_))));
        // A trapezoid prism's flat top, both sides leaning in.
        let wedge = crate::sweep::extrude(
            &crate::profile::Profile::polygon(
                &[Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(8.0, 4.0), Vec2::new(2.0, 4.0)],
                &["a", "b", "c", "d"],
            ),
            &Frame::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
            5.0,
            "w",
        )
        .unwrap();
        let roof = faces_where(&wedge, |bb| bb.min.z > 4.0 - 1e-9);
        assert_eq!(roof.len(), 1);
        assert!(!topo::local::rims_square(&wedge, &roof));
        assert!(matches!(thicken_sheet(&wedge, &roof, 1.0, true, "t"), Err(Error::Unsupported(_))));
        // Half a frustum's side: a cone, but its rims are half circles and two lines.
        let (half, _) =
            boolean::split(&frustum, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)).unwrap();
        let side = faces_where(&half, |bb| bb.max.z - bb.min.z > 3.0 && bb.max.y > 1.0);
        assert_eq!(side.len(), 1);
        assert!(turned_slab(&half, &side, 1.0, "t").is_none());
        assert!(matches!(thicken_sheet(&half, &side, 1.0, true, "t"), Err(Error::Unsupported(_))));
    }

    #[test]
    fn a_round_join_inward_declines() {
        // The bracket's crease thickened 1 in opens on its concave side, which this rung does not
        // round.
        let b = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 8.0, 10.0), "b").unwrap();
        let notch = crate::primitive::block(&Frame::WORLD, Vec3::new(5.0, 8.0, 5.0), "n").unwrap();
        let l = boolean::difference(&b, &notch).unwrap();
        let inner = faces_where(&l, |bb| {
            (bb.min.z - 5.0).abs() < 1e-9 && bb.max.z < 5.0 + 1e-9
                || (bb.min.x - 5.0).abs() < 1e-9 && bb.max.x < 5.0 + 1e-9
        });
        assert_eq!(inner.len(), 2);
        assert!(matches!(thicken_sheet(&l, &inner, -1.0, true, "t"), Err(Error::Unsupported(_))));
    }
}
