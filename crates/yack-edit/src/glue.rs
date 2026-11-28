// yack: glue for yack's edits: offsets, face moves, drafts and transforms over this crate's edits (moved from springrep's facade/edit.rs, M3 W10).
//! The edits `yack::Solid` exposes, as springrep calls with the protocol's argument shapes.

use std::collections::BTreeMap;

use crate::{
    Error, Result,
    math::{Frame, Transform, Vec3},
    topo::{Brep, EdgeId, FaceId},
};

fn v3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

/// The 3×3 part and the translation of a 3×4 row matrix.
fn split(m: [[f64; 4]; 3]) -> ([[f64; 3]; 3], Vec3) {
    (m.map(|r| [r[0], r[1], r[2]]), Vec3::new(m[0][3], m[1][3], m[2][3]))
}

/// Offsets every face by `distance` (positive grows). Declines a growth that closes a hole exactly,
/// which this offset builds wrong: the hole's faces meet at its axis, the solid touching itself
/// along it. (A growth past a hole fills it and dimples its rims; an inward offset declines its own
/// collapses.)
pub fn offset(b: &Brep, distance: f64) -> Result<Brep> {
    // A concave round the growth takes past its radius is not a hole: the growth takes it out
    // first (crate::offset_in::offset_faces_past_rounds), so only what is left can close.
    let by: BTreeMap<FaceId, f64> = b.face_ids().map(|f| (f, distance)).collect();
    let sharp = if distance > 0.0 { crate::tweak::drop_vanishing_rounds(b, &by).ok().flatten() } else { None };
    if distance > 0.0 && closes_exactly(sharp.as_ref().map_or(b, |(s, _)| s), distance) {
        return Err(Error::Unsupported("an offset that closes a hole exactly (its faces meet at its axis)".into()));
    }
    crate::offset(b, distance, "offset")
}

/// Whether `b` has a hole of radius `distance`: a concave cylinder, sphere or torus tube.
fn closes_exactly(b: &Brep, distance: f64) -> bool {
    use crate::geom::Surface;
    b.face_ids().any(|f| {
        let face = b.face(f);
        let r = match &face.surface {
            Surface::Cylinder(c) => c.radius,
            Surface::Sphere(s) => s.radius,
            Surface::Torus(t) => t.minor,
            _ => return false,
        };
        face.reversed && (r - distance).abs() <= crate::tol::LINEAR
    })
}

/// Offsets each of `faces` by the same `distance`.
fn offset_faces(b: &Brep, faces: &[FaceId], distance: f64) -> Result<Brep> {
    let by: BTreeMap<FaceId, f64> = faces.iter().map(|&f| (f, distance)).collect();
    crate::offset_faces(b, &by)
}

/// Removes `faces`, healing from their neighbours.
fn delete_faces(b: &Brep, faces: &[FaceId]) -> Result<Brep> {
    crate::delete_faces(b, faces)
}

/// Moves `faces` by the rigid motion with rows `m`.
fn move_faces(b: &Brep, faces: &[FaceId], m: [[f64; 4]; 3]) -> Result<Brep> {
    let (r, t) = split(m);
    crate::move_faces_by(b, faces, &Transform::rigid(r, t)?)
}

/// Thickens `faces` by `thickness`.
fn thicken(b: &Brep, faces: &[FaceId], thickness: f64) -> Result<Brep> {
    crate::thicken(b, faces, thickness)
}

/// Whether the inward shell of `b` to `thickness`, open through `open`, deals with every round thinner
/// than its wall: by taking it out of the cavity (tori, spheres and cones about an axis, see
/// [`crate::tweak::drop_vanishing_rounds`]), or because the only ones are straight cylindrical rounds
/// between plane faces, which the cavity's offset follows closed. False where a round of another
/// kind would still close the move, and where none does.
pub fn shell_goes_past_rounds(b: &Brep, open: &[FaceId], thickness: f64) -> bool {
    let by: BTreeMap<FaceId, f64> = b.face_ids().map(|f| (f, if open.contains(&f) { thickness } else { -thickness })).collect();
    matches!(crate::tweak::drop_vanishing_rounds(b, &by), Ok(Some(_))) || only_straight_rounds_close(b, thickness)
}

/// Whether the rounds of `b` that a wall of `thickness` closes are all straight cylinders, each
/// between plane faces only, and there is one.
fn only_straight_rounds_close(b: &Brep, thickness: f64) -> bool {
    use crate::geom::Surface;
    let closing: Vec<FaceId> = b
        .face_ids()
        .filter(|&f| {
            let face = b.face(f);
            !face.reversed
                && match &face.surface {
                    Surface::Cylinder(c) => c.radius <= thickness,
                    Surface::Sphere(s) => s.radius <= thickness,
                    Surface::Torus(t) => t.minor <= thickness,
                    _ => false,
                }
        })
        .collect();
    !closing.is_empty()
        && closing.iter().all(|&f| {
            matches!(b.face(f).surface, Surface::Cylinder(_))
                && b.face(f).loops.iter().all(|&lp| {
                    b.lp(lp).coedges.iter().all(|&c| {
                        let e = b.coedge(c).edge;
                        b.edge_uses(e).iter().map(|&u| b.coedge_face(u)).all(|g| g == f || matches!(b.face(g).surface, Surface::Plane(_)))
                    })
                })
        })
}

/// Whether `out`, the inward shell of `b` to `thickness`, is a wall that can be one: some solid in it
/// and some cavity taken, and no more solid than a skin of the whole surface's area by the
/// thickness holds.
fn is_a_wall(b: &Brep, out: &Brep, thickness: f64) -> bool {
    let (whole, wall) = (crate::measure::volume(b), crate::measure::volume(out));
    wall > 0.0 && whole - wall > 0.0 && wall <= crate::measure::area(b) * thickness * (1.0 + 1e-6) // tol: slack on an exact bound
}

/// Hollows to a wall of `thickness` inside, removing `open`.
pub fn shell(b: &Brep, open: &[FaceId], thickness: f64) -> Result<Brep> {
    let out = crate::shell(b, open, thickness, crate::Wall::Inward, "shell")?;
    // A round closed by following the offset is checked, not taken on trust.
    if only_straight_rounds_close(b, thickness) && !is_a_wall(b, &out, thickness) {
        return Err(Error::Unsupported("a shell whose straight rounds closed on the way left no sound wall".into()));
    }
    Ok(out)
}

/// Wraps the solid in a wall of `thickness` outside it, removing `open`: the faces become the wall's inside.
pub fn shell_outward(b: &Brep, open: &[FaceId], thickness: f64) -> Result<Brep> {
    crate::shell(b, open, thickness, crate::Wall::Outward, "shell")
}

/// Drafts `faces` by `angle` radians about the neutral plane through `origin` normal to `pull`.
fn draft(b: &Brep, faces: &[FaceId], origin: [f64; 3], pull: [f64; 3], angle: f64) -> Result<Brep> {
    let z = v3(pull);
    let hint = if z.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let neutral = Frame::new(v3(origin), z, hint)?;
    crate::draft(b, faces, &neutral, angle)
}

/// The image under the similarity with rows `m` (a stretch or shear is refused).
pub fn transform(b: &Brep, m: [[f64; 4]; 3]) -> Result<Brep> {
    let (r, t) = split(m);
    let mut out = b.clone();
    out.transform_similar(r, t)?;
    Ok(out)
}

/// The image under the invertible affine map with rows `m`.
pub fn affine(b: &Brep, m: [[f64; 4]; 3]) -> Result<Brep> {
    let (m, t) = split(m);
    crate::affine(b, &crate::Affine { m, t })
}

/// `b` with every face that has no name named `<feature>.<part>`: springrep's face edits keep the
/// names of the faces they carry over and leave the faces they make unnamed.
fn name_new(mut b: Brep, feature: &str, part: &str) -> Brep {
    let fresh: Vec<FaceId> = b.face_ids().filter(|&f| b.face(f).names.is_empty()).collect();
    for f in fresh {
        b.set_face_names(f, std::iter::once(crate::topo::FaceName::new(feature, part)).collect());
    }
    b
}

/// [`offset_faces`], naming the faces it makes `<feature>.side`.
pub fn offset_faces_named(b: &Brep, faces: &[FaceId], distance: f64, feature: &str) -> Result<Brep> {
    offset_faces(b, faces, distance).map(|r| name_new(r, feature, "side"))
}

/// [`delete_faces`], naming the faces it makes `<feature>.fill`.
pub fn delete_faces_named(b: &Brep, faces: &[FaceId], feature: &str) -> Result<Brep> {
    delete_faces(b, faces).map(|r| name_new(r, feature, "fill"))
}

/// [`move_faces`], naming the faces it makes `<feature>.side`.
pub fn move_faces_named(b: &Brep, faces: &[FaceId], m: [[f64; 4]; 3], feature: &str) -> Result<Brep> {
    move_faces(b, faces, m).map(|r| name_new(r, feature, "side"))
}

/// [`thicken`], naming the faces it makes `<feature>.side`.
///
/// This rung only mitres the sheet's creases: asked to round them (`round`) where a crease opens, it
/// declines, as the round would change the solid.
pub fn thicken_named(b: &Brep, faces: &[FaceId], thickness: f64, round: bool, feature: &str) -> Result<Brep> {
    if round && opens_a_crease(b, faces, thickness) {
        return Err(Error::Unsupported(
            "a round join where the thickened sheet's faces part at a crease (this rung only mitres)".into(),
        ));
    }
    thicken(b, faces, thickness).map(|r| name_new(r, feature, "side"))
}

/// Whether thickening `faces` by `thickness` opens a gap at a crease between two of them, which a
/// round join fills with a cylinder where a mitre extends the faces: a convex crease when the
/// sheet grows outward, a concave one when it grows inward. Tangent creases open nothing.
fn opens_a_crease(b: &Brep, faces: &[FaceId], thickness: f64) -> bool {
    let mut edges: Vec<EdgeId> = Vec::new();
    for &f in faces {
        for &lp in &b.face(f).loops {
            edges.extend(b.lp(lp).coedges.iter().map(|&c| b.coedge(c).edge));
        }
    }
    edges.sort();
    edges.dedup();
    edges.into_iter().any(|e| {
        let on: Vec<FaceId> = b.edge_uses(e).iter().map(|&c| b.coedge_face(c)).collect();
        matches!(*on.as_slice(), [f, g] if f != g && faces.contains(&f) && faces.contains(&g))
            && crate::topo::local::convexity(b, e) == Some(thickness > 0.0)
    })
}

/// [`draft`], naming the faces it makes `<feature>.side`.
pub fn draft_named(
    b: &Brep,
    faces: &[FaceId],
    origin: [f64; 3],
    pull: [f64; 3],
    angle: f64,
    feature: &str,
) -> Result<Brep> {
    draft(b, faces, origin, pull, angle).map(|r| name_new(r, feature, "side"))
}

#[cfg(test)]
mod tests {
    use super::opens_a_crease;
    use crate::{
        math::{Frame, Vec3},
        topo::{Brep, FaceId},
    };

    fn face_toward(b: &Brep, n: Vec3) -> FaceId {
        b.face_ids()
            .find(|&f| crate::measure::face_bbox(b, f).center().dot(n) > 2.9)
            .unwrap()
    }

    #[test]
    fn an_offset_that_closes_a_hole_declines() {
        // brep-bench offset/drilled-plate-grown-by-its-hole: a 20 × 20 × 4 plate with a bore of
        // radius 3, grown by 3, closes the bore, which this offset would build wrong (V 6317.2
        // where the case is 6300.9).
        let plate = crate::primitive::block(&Frame::WORLD.translated(Vec3::new(-10.0, -10.0, 0.0)), Vec3::new(20.0, 20.0, 4.0), "p").unwrap();
        let bore = crate::primitive::cylinder(&Frame::WORLD.translated(Vec3::new(0.0, 0.0, -1.0)), 3.0, 6.0, "b").unwrap();
        let drilled = crate::boolean::difference(&plate, &bore).unwrap();
        assert!(matches!(super::offset(&drilled, 3.0), Err(crate::Error::Unsupported(_))));
        // Short of the bore's radius it still answers, and past it (the bore filled, its rims
        // dimpled).
        assert!(super::offset(&drilled, 2.0).is_ok());
        assert!(super::offset(&drilled, 4.0).is_ok());
    }

    #[test]
    fn a_round_join_is_asked_only_where_the_sheet_opens_a_crease() {
        // A 6 mm cube about the origin: its top and a side meet at a convex crease.
        let cube = crate::primitive::block(&Frame::WORLD.translated(Vec3::new(-3.0, -3.0, -3.0)), Vec3::new(6.0, 6.0, 6.0), "c").unwrap();
        let (top, side) = (face_toward(&cube, Vec3::Z), face_toward(&cube, Vec3::X));
        assert!(opens_a_crease(&cube, &[top, side], 1.0));
        // Grown inward the two faces close over the crease; one face alone has none.
        assert!(!opens_a_crease(&cube, &[top, side], -1.0));
        assert!(!opens_a_crease(&cube, &[top], 1.0));
    }

    #[test]
    fn a_shell_follows_a_straight_round_thinner_than_its_wall_closed() {
        // A 10 cube with one vertical edge rounded by 1 (it holds 1000 less (1 - pi/4) 1^2 10), shelled 2
        // thick, so the round closes in the cavity, whose corner stays sharp: open at the top the cavity
        // is 6 x 6 x 8, closed 6 x 6 x 6.
        let cube = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "c").unwrap();
        let edge = cube
            .edge_ids()
            .find(|&e| {
                let (a, b) = (cube.vertex(cube.edge(e).start).point, cube.vertex(cube.edge(e).end).point);
                (a.x - 10.0).abs() < 1e-9 && (a.y - 10.0).abs() < 1e-9 && (b.x - 10.0).abs() < 1e-9 && (b.y - 10.0).abs() < 1e-9 // tol: the edge x = y = 10
            })
            .unwrap();
        let rounded = crate::blend::fillet_edges(&cube, &[edge], 1.0, "r").unwrap();
        let top = face_toward(&rounded, Vec3::Z);
        let held = 1000.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 10.0;
        assert!(super::shell_goes_past_rounds(&rounded, &[top], 2.0));
        // A wall thinner than the round leaves it standing: nothing closes.
        assert!(!super::shell_goes_past_rounds(&rounded, &[top], 0.5));
        let cup = super::shell(&rounded, &[top], 2.0).unwrap();
        assert!((crate::measure::volume(&cup) - (held - 6.0 * 6.0 * 8.0)).abs() < 1e-6, "{}", crate::measure::volume(&cup));
        let sealed = super::shell(&rounded, &[], 2.0).unwrap();
        assert!((crate::measure::volume(&sealed) - (held - 6.0 * 6.0 * 6.0)).abs() < 1e-6, "{}", crate::measure::volume(&sealed));
    }
}
