// yack: the rounded shells: a wall of every point within the thickness of the solid's faces other than the open ones.
//! The rounded shells, built from the offsets and fillets this crate already has.
//!
//! Inward, the wall is every point of the solid within the thickness `t` of its faces other than
//! the open ones: the cavity is what lies farther than `t` from them. At a convex edge the faces
//! moved `t` inward meet, so the cavity's edge there is sharp, as the sharp shell's is. At a concave
//! edge the moved faces part, and the points farther than `t` from both faces and from the edge
//! itself are bounded by the cylinder of radius `t` about the old edge: the moved solid's concave
//! edge rounded at `t`, whose fillet is tangent to both moved faces with radius `t` and so has the
//! old edge for its axis (a ball's patch about a concave corner, likewise). So the cavity is the
//! sharp shell's cavity, its concave edges filleted at `t`, and where the solid has no concave edge
//! the rounded shell is the sharp one.
//!
//! Outward, the wall is every point outside a convex polyhedron within `t` of it, up to the planes
//! of the open faces: the solid grown by `t` (its faces moved out, its edges rounded, its corners
//! balls), cut at each open face's plane, less the solid.
//!
//! Each declines (`Unsupported`) outside what that reasoning covers, for the next rung to try.

use std::collections::{BTreeMap, BTreeSet};

use crate::{
    blend::{self, shaped::plane_of},
    boolean,
    error::{Error, Result},
    geom::Surface,
    math::{Point3, Vec3},
    tol,
    topo::{self, Brep, FaceId, FaceName},
};

fn thick_enough(thickness: f64) -> Result<()> {
    if thickness.is_nan() || thickness <= tol::LINEAR {
        return Err(Error::InvalidInput(
            "a shell's wall must be thicker than the tolerance".into(),
        ));
    }
    Ok(())
}

/// A step's refusal of its input, past this rung's own checks: a case the rung does not build,
/// which the next rung may.
fn declined(e: Error) -> Error {
    match e {
        Error::InvalidInput(m) => Error::Unsupported(format!("a rounded shell: {m}")),
        e => e,
    }
}

/// `s` hollowed to a wall of every point within `thickness` of its faces other than `open`, open
/// through those. The cavity's faces are `<feature>.inner`.
pub fn shell_rounded(s: &Brep, open: &[FaceId], thickness: f64, feature: &str) -> Result<Brep> {
    thick_enough(thickness)?;
    // The walls moved in, the open faces moved out so the cavity runs out through them.
    let move_of = |f: FaceId, open: &[FaceId]| if open.contains(&f) { thickness } else { -thickness };
    let by: BTreeMap<FaceId, f64> = s.face_ids().map(|f| (f, move_of(f, open))).collect();
    // A round the walls move in past its radius vanishes from the cavity: the points farther than
    // the thickness from it are those farther from the sharp edge it rounded. So the cavity is cut
    // from the solid with those rounds taken out (`base`), and the wall is the solid itself less
    // the cavity. `of` takes each face of `base` back to the face of `s` it is.
    let (base, of): (Brep, BTreeMap<FaceId, FaceId>) =
        match crate::tweak::drop_vanishing_rounds(s, &by).map_err(declined)? {
            Some((sharp, kept)) => (sharp, kept.into_iter().map(|(old, new)| (new, old)).collect()),
            None => (s.clone(), s.face_ids().map(|f| (f, f)).collect()),
        };
    let open: Vec<FaceId> =
        of.iter().filter(|&(_, old)| open.contains(old)).map(|(&new, _)| new).collect();
    let walls: Vec<FaceId> = base.face_ids().filter(|f| !open.contains(f)).collect();
    if topo::local::radius_collapses(&base, Some(&walls), thickness, true) {
        return Err(Error::Unsupported(
            "a rounded shell past a round face's radius (a round that does not vanish cleanly)".into(),
        ));
    }
    if topo::local::open_at_concave_edge(&base, &open) {
        return Err(Error::Unsupported("a rounded shell open at a concave edge".into()));
    }
    let by: BTreeMap<FaceId, f64> = base.face_ids().map(|f| (f, move_of(f, &open))).collect();
    let moved = crate::offset_faces(&base, &by).map_err(declined)?;
    // Where the moved walls cross (a neck thinner than twice the wall), faces of the moved solid
    // vanish, and the cavity is no longer the moved faces with their concave edges rounded: the
    // erosion bulges round the corners whose walls went (bench shell/dumbbell-neck-filled), which
    // this construction cannot make. Only a move that keeps every face is built here.
    if !crate::same_topology(&base, &moved) {
        return Err(Error::Unsupported(
            "a rounded shell whose moved walls cross or vanish (the cavity's rounds are not the moved solid's)".into(),
        ));
    }
    let inner = FaceName::new(feature, FaceName::INNER);
    // The move kept the topology and with it the ids: each cavity face offsets the face of `base`
    // with its id, which is the face `of` names in `s`.
    let mut moved = moved;
    for f in moved.face_ids().collect::<Vec<_>>() {
        let mut names: BTreeSet<FaceName> = std::iter::once(inner.clone()).collect();
        if let Some(old) = of.get(&f) {
            names.insert(FaceName::new(feature, format!("inner_f{}", old.index())));
        }
        moved.set_face_names(f, names);
    }
    let concave = crate::concave_edges(&moved);
    let mut cavity = if concave.is_empty() {
        moved
    } else {
        if crate::offset_in::rounds_cross(&moved, &concave) {
            return Err(Error::Unsupported(
                "a rounded shell whose rounds meet over a convex edge (they cross in a crease there)".into(),
            ));
        }
        blend::fillet_edges(&moved, &concave, thickness, feature).map_err(declined)?
    };
    for f in cavity.face_ids().collect::<Vec<_>>() {
        if !cavity.face(f).names.contains(&inner) {
            let mut names = cavity.face(f).names.clone();
            names.insert(inner.clone());
            cavity.set_face_names(f, names);
        }
    }
    boolean::difference(s, &cavity)
}

/// `s` wrapped in a wall of every point outside it within `thickness` of its faces other than
/// `open`, up to the planes of the open faces: the grown solid cut at those planes, less `s`. Only
/// a convex polyhedron (every face a plane, no concave edge), with the whole solid behind each open
/// face's plane.
pub fn shell_outward_rounded(
    s: &Brep,
    open: &[FaceId],
    thickness: f64,
    feature: &str,
) -> Result<Brep> {
    thick_enough(thickness)?;
    if s.face_ids().any(|f| !matches!(s.face(f).surface, Surface::Plane(_)))
        || !crate::concave_edges(s).is_empty()
    {
        return Err(Error::Unsupported(
            "an outward rounded shell of a solid that is not a convex polyhedron".into(),
        ));
    }
    let planes: Vec<(Point3, Vec3)> =
        open.iter().map(|&f| plane_of(s, f)).collect::<Result<_>>().map_err(declined)?;
    // A convex polyhedron lies behind a plane when its vertices do.
    let past =
        |o: Point3, n: Vec3| s.vertex_ids().any(|v| (s.vertex(v).point - o).dot(n) > tol::LINEAR);
    if planes.iter().any(|&(o, n)| past(o, n)) {
        return Err(Error::Unsupported(
            "an outward rounded shell of a solid reaching past an open face's plane".into(),
        ));
    }
    let mut wall = crate::offset(s, thickness, feature).map_err(declined)?;
    for &(origin, normal) in &planes {
        // The part behind the plane: the side the normal leaves.
        let (_, behind) = boolean::split(&wall, origin, normal)?;
        wall = behind;
    }
    let mut out = boolean::difference(&wall, s)?;
    // The cut leaves a rim on each open face's plane, between the solid's outline and the wall's:
    // what is left of the open face's place, so it carries that face's names (and its origin), as
    // the sharp outward shell's rim does.
    for f in out.face_ids().collect::<Vec<_>>() {
        let Ok((p, m)) = plane_of(&out, f) else {
            continue;
        };
        let on = |&(&(o, n), _): &(&(Point3, Vec3), &FaceId)| {
            m.dot(n) > 1.0 - tol::ANGULAR && (p - o).dot(n).abs() <= tol::LINEAR
        };
        if let Some((_, &g)) = planes.iter().zip(open).find(on) {
            out.set_face_names(f, s.face(g).names.clone());
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::{
        math::{Frame, Vec2},
        measure,
        profile::Profile,
        sweep::extrude,
    };

    fn close(got: f64, want: f64) {
        assert!((got - want).abs() < 1e-7 * want.max(1.0), "{got} against {want}");
    }

    /// A prism on the polygon `pts`, `h` tall.
    fn prism(pts: &[[f64; 2]], h: f64) -> Brep {
        let pts: Vec<Vec2> = pts.iter().map(|p| Vec2::new(p[0], p[1])).collect();
        let names: Vec<String> = (0..pts.len()).map(|i| format!("e{i}")).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, h, "p").unwrap()
    }

    /// The face of `b` whose box lies at height `z`.
    fn top(b: &Brep, z: f64) -> FaceId {
        b.face_ids().find(|&f| (measure::face_bbox(b, f).min.z - z).abs() < 1e-9).unwrap()
    }

    #[test]
    fn a_box_shelled_nearly_through_leaves_a_hairline_slot() {
        // bench shell/box-hairline-cavity: abc − (a − 2t)(b − 2t)(c − t), t = 3.99.
        let b = prism(&[[0.0, 0.0], [10.0, 0.0], [10.0, 8.0], [0.0, 8.0]], 6.0);
        let out = shell_rounded(&b, &[top(&b, 6.0)], 3.99, "s").unwrap();
        close(measure::volume(&out), 480.0 - 2.02 * 0.02 * 2.01);
        assert!(crate::check::check(&out).is_empty());
    }

    #[test]
    fn an_l_plate_shelled_open_rounds_its_wall_about_the_concave_edge() {
        // bench shell/l-block-open-top: 8·150 − 7(85 − π/4), the cavity's corner at the concave
        // edge a quarter round of radius 1 about it.
        let l = prism(
            &[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]],
            8.0,
        );
        let out = shell_rounded(&l, &[top(&l, 8.0)], 1.0, "s").unwrap();
        close(measure::volume(&out), 605.0 + 7.0 * PI / 4.0);
        assert!(crate::check::check(&out).is_empty());
    }

    #[test]
    fn a_rounded_shell_declines_where_its_walls_cross() {
        // bench shell/dumbbell-neck-filled: the neck is 2 wide, the wall 1.5, so the neck's moved
        // walls cross. The erosion bulges round the neck's corners, which the moved solid lacks:
        // built, it would be the sharp 807 against the true 780 + 9√5/2 + (81/4)·asin(2/3).
        let outline = [
            [0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [14.0, 4.0], [14.0, 0.0], [24.0, 0.0],
            [24.0, 10.0], [14.0, 10.0], [14.0, 6.0], [10.0, 6.0], [10.0, 10.0], [0.0, 10.0],
        ];
        let plate = prism(&outline, 6.0);
        assert!(matches!(shell_rounded(&plate, &[top(&plate, 6.0)], 1.5, "s"), Err(Error::Unsupported(_))));
    }

    #[test]
    fn a_rounded_puck_hollowed_past_its_rounds_loses_them_inside() {
        // bench shell/rounded-puck-hollowed-past-its-rounds: a cylinder r 6, h 10, both rims rounded
        // to 2, hollowed to 3 with no opening. The rounds vanish from the cavity, which is the sharp
        // cylinder r 3 from z 3 to 7: V = (the rounded puck) − 36π.
        let puck = crate::primitive::cylinder(&Frame::WORLD, 6.0, 10.0, "c").unwrap();
        let rims: Vec<_> = crate::convex_edges(&puck).unwrap();
        let rounded = blend::fillet_edges(&puck, &rims, 2.0, "r").unwrap();
        let out = shell_rounded(&rounded, &[], 3.0, "s").unwrap();
        close(measure::volume(&out), measure::volume(&rounded) - 36.0 * PI);
        close(measure::volume(&out), 957.9725995316304);
        assert!(crate::check::check(&out).is_empty());
    }

    #[test]
    fn a_rounded_shell_declines_past_a_round() {
        // A cylinder's side is a round of radius 2: a wall 3 thick would turn it inside out.
        let rod = crate::primitive::cylinder(&Frame::WORLD, 2.0, 10.0, "c").unwrap();
        assert!(matches!(
            shell_rounded(&rod, &[top(&rod, 10.0)], 3.0, "s"),
            Err(Error::Unsupported(_))
        ));
    }

    #[test]
    fn a_box_shelled_outward_open_at_its_top_rounds_its_outside() {
        // bench shell/box-open-top-outward: 2tc(a + b) + πt²c + abt + πt²(a + b)/2 + 2πt³/3.
        let b = prism(&[[0.0, 0.0], [10.0, 0.0], [10.0, 8.0], [0.0, 8.0]], 6.0);
        let out = shell_outward_rounded(&b, &[top(&b, 6.0)], 1.0, "s").unwrap();
        close(measure::volume(&out), 296.0 + 47.0 * PI / 3.0);
        assert!(crate::check::check(&out).is_empty());
        // Every face is named: the inside carries the block's, the outside the offset's, and the
        // rim on the top's plane the top's own.
        let top_names = b.face(top(&b, 6.0)).names.clone();
        assert!(out.face_ids().all(|f| !out.face(f).names.is_empty()));
        assert!(out.face_ids().any(|f| out.face(f).names == top_names));
    }

    #[test]
    fn an_outward_rounded_shell_declines_a_solid_with_a_concave_edge() {
        let l = prism(
            &[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]],
            8.0,
        );
        assert!(matches!(
            shell_outward_rounded(&l, &[top(&l, 8.0)], 1.0, "s"),
            Err(Error::Unsupported(_))
        ));
    }
}
