// yack: shells and drafts on flat, cylindrical and spherical walls, against closed forms.
// Adapted from springrep@a03e8a1 tests/shells_and_drafts.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep's face names
// "<feature>.<part>" are yack roles in the op tagged <feature> (Solid::named, then Solid::face_by /
// faces_by): "end"/"start" are Role::End/Start, a rect's u_min/u_max/v_min/v_max are a block's
// Role::Side("x_min"/"x_max"/"y_min"/"y_max"), a circle's "side" is Role::Side("side"), a profile
// segment's own name is its Pen::label, and a shell's "inner" is Role::Inner; edges between two named
// faces are Solid::edges_between; edit::shell(.., Wall::Inward) is Solid::shell (positive thickness
// hollows inward, the protocol's convention) and edit::draft on the xy frame is Solid::draft with
// origin 0 and pull +z; the trailing unify is dropped (yack results are already merged); springrep
// circles take a diameter, yack's a radius; edit::shell(.., Wall::Outward) is Solid::shell_outward;
// face_count is faces().len().

use std::f64::consts::{PI, TAU};

use yack::prelude::*;

/// The 20 × 20 × 10 block on the xy plane, centred on the origin, tagged "block".
fn cube() -> Solid {
    block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 10.0]).unwrap().named("block")
}

/// A cylinder R`r` × `h` standing on the xy plane, tagged "can".
fn can(r: f64, h: f64) -> Solid {
    cylinder(&Frame::WORLD, r, h).unwrap().named("can")
}

fn side(name: &str) -> Role {
    Role::Side(name.into())
}

/// `solid` with `faces` tilted by `angle` about where they meet the xy plane.
fn drafted(solid: &Solid, faces: &[FaceId], angle: f64) -> Solid {
    solid.draft(faces, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], rad(angle)).unwrap()
}

fn faces_of(b: &Solid, keep: impl Fn(SurfaceKind) -> bool) -> usize {
    b.faces_where(|f| keep(f.kind())).unwrap().len()
}

#[test]
fn a_shell_open_at_the_top_leaves_a_cup() {
    // A 20 × 20 × 10 mm block less a 16 × 16 × 8 cavity open at the top: 4000 − 2048.
    let block = cube();
    let b = block.shell(&[block.face_by("block", Role::End).unwrap()], 2.0).unwrap().named("hollow");
    assert!((b.volume() - 1952.0).abs() < 1e-9);
    assert_eq!(b.faces_by("hollow", Role::Inner).len(), 5);
    assert_eq!(b.faces_by("block", Role::End).len(), 1);
}

#[test]
fn a_shell_grown_outward_keeps_the_inside() {
    // The 20 × 20 × 10 mm block grown 2 mm out on every side but the open top: 24 · 24 · 12 − 4000.
    let block = cube();
    let b = block.shell_outward(&[block.face_by("block", Role::End).unwrap()], 2.0).unwrap().named("skin");
    assert!((b.volume() - (24.0 * 24.0 * 12.0 - 4000.0)).abs() < 1e-9);
    // The inside is the block's own faces, where they were: each keeps its name and is the wall's
    // inside too. The grown outside is new and names nothing; the rim keeps the open top's name.
    assert_eq!(b.faces_by("block", side("x_min")).len(), 1);
    assert_eq!(b.faces_by("block", Role::Start).len(), 1);
    assert_eq!(b.faces_by("skin", Role::Inner).len(), 5);
    assert_eq!(b.faces_by("block", Role::End).len(), 1);
    assert_eq!(b.faces().len(), 11);
}

#[test]
fn a_draft_tapers_the_walls_from_the_neutral_plane() {
    // Two opposite walls of the 20 × 20 × 10 mm block tilted so they come 2 mm in at the top: a
    // trapezoid 20 at the foot and 16 at the top, 10 high, 20 deep.
    let angle = (2.0f64 / 10.0).atan();
    let block = cube();
    let walls = [block.face_by("block", side("x_min")).unwrap(), block.face_by("block", side("x_max")).unwrap()];
    let b = drafted(&block, &walls, angle);
    let want = 20.0 * 10.0 * 36.0 / 2.0;
    assert!((b.volume() - want).abs() < 1e-9 * want);
    assert_eq!(b.faces_by("block", side("x_min")).len(), 1);
}

#[test]
fn a_cylinder_shelled_open_at_the_top_is_a_cup_with_a_round_wall() {
    // R10 × 20 hollowed 2 mm, the top open: the cavity is the cylinder R8 from z = 2 to 20, its
    // wall the cylinder's offset 2 mm in, exactly.
    let c = can(10.0, 20.0);
    let b = c.shell(&[c.face_by("can", Role::End).unwrap()], 2.0).unwrap();
    assert!(b.check().is_empty());
    let want = PI * (100.0 * 20.0 - 64.0 * 18.0);
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
    let walls: Vec<f64> = b
        .faces()
        .into_iter()
        .filter_map(|f| match b.face(f).unwrap().kind() {
            SurfaceKind::Cylinder { radius, .. } => Some(radius),
            _ => None,
        })
        .collect();
    assert_eq!(walls.len(), 2);
    assert!(walls.iter().any(|r| (r - 8.0).abs() < 1e-12), "{walls:?}");
}

#[test]
fn a_cylinder_shelled_outward_grows_its_wall_round_it() {
    // R10 × 20 shelled 2 mm outward, the top open: the outside is R12 from z = −2 to 20 and the
    // inside the original cylinder.
    let c = can(10.0, 20.0);
    let b = c.shell_outward(&[c.face_by("can", Role::End).unwrap()], 2.0).unwrap();
    assert!(b.check().is_empty());
    let want = PI * (144.0 * 22.0 - 100.0 * 20.0);
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
}

#[test]
fn a_dome_shelled_open_at_its_base_is_a_spherical_cap() {
    // An R10 hemisphere standing on its flat, hollowed 1 mm with the flat open: the shell between
    // the spheres R10 and R9, 2π(10³ − 9³)/3.
    let (foot, pole) = ([10.0, 0.0], [0.0, 10.0]);
    let quarter = Loop::starting_at(foot)
        .arc_ccw_to(pole, [0.0, 0.0])
        .label("ball_side")
        .line_to([0.0, 0.0])
        .label("half_u_min")
        .line_to(foot)
        .label("half_v_min")
        .close()
        .unwrap();
    // springrep turns the quarter, drawn on the frame (z = −y, x = x, so its y is world z), about
    // world z; yack's revolve draws in the frame's xz plane and turns about its z.
    let dome = revolve(&Frame::WORLD, &Profile::new(quarter, vec![]).unwrap(), rad(TAU)).unwrap().named("dome");
    let b = dome.shell(&[dome.face_by("dome", side("half_v_min")).unwrap()], 1.0).unwrap();
    assert!(b.check().is_empty());
    let want = 2.0 * PI * (1000.0 - 729.0) / 3.0;
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
}

#[test]
fn a_rounded_block_shelled_keeps_its_rounds_as_offset_cylinders() {
    // The 20 × 20 × 10 block with its four vertical edges rounded R4, hollowed 1 mm with the top
    // open. Each section is the rounded square, and the cavity's the square 2 smaller with rounds
    // R3: (20² − (4 − π)·4²)·10 − (18² − (4 − π)·3²)·9.
    let block = cube();
    let mut edges = Vec::new();
    for (a, b) in [("x_min", "y_min"), ("y_min", "x_max"), ("x_max", "y_max"), ("y_max", "x_min")] {
        let (fa, fb) = (block.face_by("block", side(a)).unwrap(), block.face_by("block", side(b)).unwrap());
        edges.extend(block.edges_between(fa, fb).unwrap());
    }
    let rounded = block.fillet(&edges, 4.0).unwrap();
    let b = rounded.shell(&[rounded.face_by("block", Role::End).unwrap()], 1.0).unwrap();
    assert!(b.check().is_empty());
    let section = |w: f64, r: f64| w * w - (4.0 - PI) * r * r;
    let want = section(20.0, 4.0) * 10.0 - section(18.0, 3.0) * 9.0;
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
}

#[test]
fn a_drafted_cylinder_boss_is_a_frustum() {
    // R10 × 10, its side drafted 5° from the base: a frustum from R10 to R10 − 10·tan 5°.
    let c = can(10.0, 10.0);
    let b = drafted(&c, &[c.face_by("can", side("side")).unwrap()], 5f64.to_radians());
    assert!(b.check().is_empty());
    let r = 10.0 - 10.0 * 5f64.to_radians().tan();
    let want = PI * 10.0 / 3.0 * (100.0 + 10.0 * r + r * r);
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
    assert_eq!(faces_of(&b, |k| matches!(k, SurfaceKind::Cone)), 1);
}

#[test]
fn a_drafted_bore_widens_along_the_pull() {
    // The 20 × 20 × 10 block with a Ø8 through hole, the hole's wall drafted 3° from the base: the
    // hole is a frustum from R4 at z = 0 to R4 + 10·tan 3° at the top.
    let outer = Loop::polygon(&[[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]]).unwrap();
    // A full circle, clockwise as a hole runs.
    let bore = Loop::starting_at([8.0 / 2.0, 0.0])
        .arc_cw_to([8.0 / 2.0, 0.0], [0.0, 0.0])
        .label("bore_side")
        .close()
        .unwrap();
    let drilled = extrude(&Frame::WORLD, &Profile::new(outer, vec![bore]).unwrap(), 10.0).unwrap().named("block");
    let b = drafted(&drilled, &[drilled.face_by("block", side("bore_side")).unwrap()], 3f64.to_radians());
    assert!(b.check().is_empty());
    let r = 4.0 + 10.0 * 3f64.to_radians().tan();
    let want = 4000.0 - PI * 10.0 / 3.0 * (16.0 + 4.0 * r + r * r);
    assert!((b.volume() - want).abs() < 1e-9 * want, "{}", b.volume());
}
