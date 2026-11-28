// yack: yack-edit's unit tests, moved out of lib.rs to keep it under the file size limit
// Adapted from springrep@a03e8a1 src/edit.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from lib.rs's tests module to its own file (paths only); the tests since written for yack's own edits are new.
use super::*;
use crate::topo::FaceName;

#[test]
fn shell_cavities_and_offset_rounds_name_their_sources() {
    // A block's inward shell names each cavity face by the face it offsets; its offset names
    // each round by the edge it rounds (all 12) and each corner patch by its vertex (all 8).
    let block = crate::primitive::block(&Frame::WORLD, Vec3::new(4.0, 3.0, 2.0), "block").unwrap();
    let top = block.face_ids().find(|&f| crate::measure::face_bbox(&block, f).min.z > 1.9).unwrap();
    let b = shell(&block, &[top], 0.5, Wall::Inward, "skin").unwrap();
    let named = |b: &Brep, prefix: &str| {
        let mut ks: Vec<String> = b.face_ids().flat_map(|f| b.face(f).names.clone()).filter(|n| n.face.starts_with(prefix)).map(|n| n.face).collect();
        ks.sort();
        ks.dedup();
        ks
    };
    let inner = named(&b, "inner_f");
    assert!(inner.len() >= 5, "{inner:?}");
    let o = offset(&block, 0.5, "o").unwrap();
    let (rounds, corners) = (named(&o, "round_e"), named(&o, "corner_v"));
    assert_eq!((rounds.len(), corners.len()), (12, 8), "{rounds:?} {corners:?}");
    let edges: Vec<String> = block.edge_ids().map(|e| format!("round_e{}", e.index())).collect();
    assert!(rounds.iter().all(|r| edges.contains(r)), "{rounds:?}");
}

#[test]
fn a_shells_rim_keeps_the_open_faces_name_both_ways() {
    // A 20 × 20 × 10 block open at the top, 2 thick, in and out: the ring of wall top round
    // the opening is what is left of the open face, and carries its names.
    let block = crate::primitive::block(&Frame::WORLD, Vec3::new(20.0, 20.0, 10.0), "block").unwrap();
    let top = block
        .face_ids()
        .find(|&f| crate::measure::face_bbox(&block, f).min.z > 9.9)
        .unwrap();
    let names = block.face(top).names.clone();
    for wall in [Wall::Inward, Wall::Outward] {
        let b = shell(&block, &[top], 2.0, wall, "skin").unwrap();
        let all: Vec<_> = b.face_ids().map(|f| b.face(f).names.clone()).collect();
        let rims = all.iter().filter(|n| n.is_superset(&names)).count();
        assert_eq!(rims, 1, "{wall:?}: {all:?}");
    }
}

#[test]
fn an_arcs_reach_along_an_axis_is_its_own_extreme_not_its_nearest_sample() {
    use crate::profile::{NamedSegment, ProfileLoop, Segment};
    // A half disc of radius 10, extruded: its rim is a half circle. Along an axis that points
    // midway between two of 256 even samples of the rim, the rim reaches the full radius.
    let named = |segment: Segment, name: &str| NamedSegment::new(segment, name);
    let half = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                named(
                    Segment::Arc {
                        center: Vec2::ZERO,
                        radius: 10.0,
                        start: 0.0,
                        sweep: std::f64::consts::PI,
                    },
                    "rim",
                ),
                named(
                    Segment::Line {
                        a: Vec2::new(-10.0, 0.0),
                        b: Vec2::new(10.0, 0.0),
                    },
                    "chord",
                ),
            ],
        }],
    };
    let b = extrude(&half, &Frame::WORLD, 3.0, "h").unwrap();
    let rim = b
        .edge_ids()
        .find(|&e| {
            let edge = b.edge(e);
            matches!(edge.curve, Some(Curve::Circle(_)))
                && b.vertex(edge.start).point.z.abs() < 1e-9
        })
        .unwrap();
    let towards = std::f64::consts::PI * 100.5 / 256.0;
    let x = Vec3::new(towards.cos(), towards.sin(), 0.0);
    let axes = Frame::new(Point3::ZERO, Vec3::Z, x).unwrap();
    let mut bounds = Aabb::EMPTY;
    edge_bounds(&b, rim, &axes, &mut bounds);
    assert!((bounds.max.x - 10.0).abs() < 1e-9, "{}", bounds.max.x);
}

/// A prism on the polygon `pts`, `h` tall, its sides named `e0`, `e1`, … from the first edge.
fn prism(pts: &[[f64; 2]], h: f64) -> Brep {
    let pts: Vec<Vec2> = pts.iter().map(|p| Vec2::new(p[0], p[1])).collect();
    let names: Vec<String> = (0..pts.len()).map(|i| format!("e{i}")).collect();
    let names: Vec<&str> = names.iter().map(String::as_str).collect();
    extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, h, "p").unwrap()
}

fn face(b: &Brep, name: &str) -> FaceId {
    let n = FaceName::new("p", name);
    b.face_ids()
        .find(|&f| b.face(f).names.contains(&n))
        .unwrap()
}

fn close(got: f64, want: f64) {
    assert!(
        (got - want).abs() < 1e-7 * want.max(1.0),
        "{got} against {want}"
    );
}

#[test]
fn a_block_offset_is_the_block_rounded_by_the_distance() {
    // 4 × 3 × 2 offset by 0.5: V + S·d + π(a + b + c)·d² + (4/3)π·d³, Steiner's formula.
    let b = prism(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]], 2.0);
    let d = 0.5;
    let want = 24.0 + 52.0 * d + PI * 9.0 * d * d + 4.0 / 3.0 * PI * d.powi(3);
    close(measure::volume(&offset(&b, d, "o").unwrap()), want);
}

#[test]
fn an_l_s_rounded_edges_swing_round_its_concave_edge() {
    // An L of arms 8 × 3, 5 tall, every convex edge rounded 1: the union of the balls of
    // radius 1 inside it. The two arm boxes rounded, less their overlap (the corner column
    // rounded, with (1 − π/4)·r² short of a square along the concave edge, and 4r³/3 of that
    // at its ends), and at each end of the concave edge what the ball keeps swinging round
    // it, (1 − π/4)·r³·(5/3 − π/2). A box rounded r all over: the inner box, its faces moved
    // out r, its edges' quarter cylinders and its corners' eighth balls.
    // Its area likewise, the swing's torus, (π/2)·r²(π/2 − 1), standing for two cylinder
    // pieces of r²(π/2 − 1), and the flat face keeping the corner's r × r square less its
    // quarter disk. The arms' widths, depths, heights and radii include draws of the
    // conformance suite's L-prisms the swing once failed on.
    let cases = [
        (8.0, 3.0, 8.0, 5.0, 1.0),
        (
            6.199379767061393,
            3.2446824245098926,
            5.944930609453334,
            2.787400445599377,
            0.983805266315506,
        ),
        (
            8.410642646708611,
            2.4606909192521025,
            9.679346455567444,
            7.230068216963447,
            0.2821417462255193,
        ),
        (
            9.564764789551234,
            3.3401708771362455,
            6.900089620242228,
            3.7297726032339753,
            0.7396202116839262,
        ),
        (
            7.342452085970791,
            2.923584026607261,
            6.1264770305629295,
            2.2167560977768446,
            0.8678845173029267,
        ),
        (
            5.52965122853034,
            2.429579012791682,
            9.404723647837528,
            5.938985170662157,
            0.6834351663091055,
        ),
        (
            6.3990454944784565,
            2.3841696916601958,
            8.181436493454777,
            4.839043994693185,
            0.7102451360532922,
        ),
        (
            6.538663752237994,
            3.1564303512067906,
            9.224051200212097,
            5.94676849583945,
            1.2821630460001838,
        ),
    ];
    for (w, t, d, h, r) in cases {
        let b = prism(&[[0.0, 0.0], [w, 0.0], [w, t], [t, t], [t, d], [0.0, d]], h);
        let out = blend::fillet_edges(&b, &convex_edges(&b).unwrap(), r, "fil").unwrap();
        assert!(
            crate::check::check(&out).is_empty(),
            "{:?}",
            crate::check::check(&out)
        );
        let rounded = |x: f64, y: f64, z: f64| {
            let (a, b, c) = (x - 2.0 * r, y - 2.0 * r, z - 2.0 * r);
            (
                a * b * c
                    + 2.0 * r * (a * b + b * c + c * a)
                    + PI * r * r * (a + b + c)
                    + 4.0 / 3.0 * PI * r.powi(3),
                2.0 * (a * b + b * c + c * a) + 2.0 * PI * r * (a + b + c) + 4.0 * PI * r * r,
            )
        };
        let k = 1.0 - PI / 4.0;
        let ((va, sa), (vb, sb), (vc, sc)) =
            (rounded(w, t, h), rounded(t, d, h), rounded(t, t, h));
        let core_h = h - 2.0 * r;
        let volume = va + vb - vc - k * (core_h * r * r + 4.0 / 3.0 * r.powi(3))
            + 2.0 * k * (5.0 / 3.0 - PI / 2.0) * r.powi(3);
        let area = sa + sb
            - sc
            - k * (2.0 * core_h * r + 4.0 * r * r)
            - 2.0 * ((2.0 - PI / 2.0) * (PI / 2.0 - 1.0) - k) * r * r;
        close(measure::volume(&out), volume);
        close(measure::area(&out), area);
        // Written and read back it is the same solid: springrep's moved_tests/step.rs reads
        // these Ls from fixtures (M3 W9b).
    }
}

#[test]
fn an_l_prism_offset_turned_away_is_the_same_size() {
    let (w, t, h) = (9.536367478693311, 2.7154798898513985, 6.344052519590756);
    let b = prism(
        &[[0.0, 0.0], [w, 0.0], [w, t], [t, t], [t, h], [0.0, h]],
        5.222922150702244,
    );
    let d = 0.5756566176826041;
    let here = measure::volume(&offset(&b, d, "o").unwrap());
    let mut moved = b.clone();
    moved.transform(&crate::math::Transform {
        r: [
            [
                -0.2124025946267396,
                -0.9641070956655728,
                -0.15931932049542274,
            ],
            [
                0.9494953687642673,
                -0.1650876859267214,
                -0.26684190197674823,
            ],
            [0.2309625131726732, -0.20795086930006934, 0.9504802751589895],
        ],
        t: Vec3::new(-45.51695804202156, 41.045298753592036, -21.51869167292315),
    });
    close(measure::volume(&offset(&moved, d, "o").unwrap()), here);
}

#[test]
fn a_notch_s_faces_taken_out_leave_the_block_whole() {
    let b = prism(
        &[
            [0.0, 0.0],
            [6.0, 0.0],
            [6.0, 4.0],
            [4.0, 4.0],
            [4.0, 3.0],
            [2.0, 3.0],
            [2.0, 4.0],
            [0.0, 4.0],
        ],
        1.0,
    );
    let notch = [face(&b, "e3"), face(&b, "e4"), face(&b, "e5")];
    close(measure::volume(&delete_faces(&b, &notch).unwrap()), 24.0);
}

#[test]
fn a_face_thickened_is_a_slab_on_it() {
    // The 4 × 2 side of a 4 × 3 × 2 block thickened 0.5 outward, then 1 inward.
    let b = prism(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]], 2.0);
    let side = face(&b, "e0");
    close(measure::volume(&thicken(&b, &[side], 0.5).unwrap()), 4.0);
    close(measure::volume(&thicken(&b, &[side], -1.0).unwrap()), 8.0);
}

#[test]
fn an_l_blocks_shell_open_at_its_step_keeps_the_wall_rising_past_it() {
    // The L of 20 × 5 with a 5 × 15 tower at x 15..20, 20 tall, open at the step face (e4, y = 5).
    // Inward, 1 thick: the cavity is 18 × 4 + 3 × 14 = 114 across, 18 high; the wall past the step
    // stays 1 thick down to the opening: 3500 − 114 · 18. Outward: the L grown by 1 everywhere but
    // at the opening, (22 × 6 + 7 × 16) · 22, less the L itself: 5368 − 3500.
    let l = prism(&[[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [15.0, 20.0], [15.0, 5.0], [0.0, 5.0]], 20.0);
    let open = face(&l, "e4");
    let inward = shell(&l, &[open], 1.0, Wall::Inward, "s").unwrap();
    close(measure::volume(&inward), 3500.0 - 114.0 * 18.0);
    let outward = shell(&l, &[open], 1.0, Wall::Outward, "s").unwrap();
    close(measure::volume(&outward), 5368.0 - 3500.0);
}

#[test]
fn a_cone_shelled_open_at_its_base_is_the_cone_less_its_offset_cone() {
    // A cone of radius 6 and height 8 (sin a = 0.6, tan a = 0.75), 1 thick, open at the base: the
    // cavity is the cone offset in by 1, whose apex has come down 1 / sin a along the axis, so it
    // is a cone of height h' = 8 - 1/0.6 on the same base plane: V = (pi/3)(6^2 8 - r'^2 h').
    let c = crate::primitive::cone(&Frame::WORLD, 6.0, 0.0, 8.0, "c").unwrap();
    let base = c.face_ids().find(|&f| crate::measure::face_bbox(&c, f).max.z < 0.1).unwrap();
    let h = 8.0 - 1.0 / 0.6;
    let r = 0.75 * h;
    let b = shell(&c, &[base], 1.0, Wall::Inward, "s").unwrap();
    close(measure::volume(&b), PI / 3.0 * (36.0 * 8.0 - r * r * h));
}

#[test]
fn a_cones_face_moved_either_way_moves_its_apex_along_the_axis() {
    // The cone of radius 6 and height 8, its side moved by d with the base kept: the apex moves
    // d / sin a = d / 0.6 up the axis (down for a negative d), the cone being the same angle
    // cut by the base plane: V = (pi/3) r'^2 h' with h' = 8 + d / 0.6 and r' = 0.75 h'.
    let c = crate::primitive::cone(&Frame::WORLD, 6.0, 0.0, 8.0, "c").unwrap();
    let side = c.face_ids().find(|&f| crate::measure::face_bbox(&c, f).max.z > 7.9).unwrap();
    for d in [1.0, -1.0] {
        let by: BTreeMap<FaceId, f64> = std::iter::once((side, d)).collect();
        let moved = offset_faces(&c, &by).unwrap();
        let h = 8.0 + d / 0.6;
        close(measure::volume(&moved), PI / 3.0 * (0.75 * h).powi(2) * h);
    }
}

#[test]
fn an_outward_offset_of_a_cone_rounds_its_apex_with_a_ball() {
    // The 3 x 4 cone (slant 5, sin a = 0.6) offset by 1: in the meridian triangle's half plane the
    // solid is the cone 12 pi, the slant strip 19 pi, the base slab 9 pi, and the sectors round the
    // base corner (from the base normal to the slant's, 2 pi (3 t / 2 + 1.6 / 3) with
    // t = asin 0.6 + pi / 2) and the apex (2 pi 0.4 / 3): Pappus' theorem on each piece.
    let c = crate::primitive::cone(&Frame::WORLD, 3.0, 0.0, 4.0, "c").unwrap();
    let t = 0.6_f64.asin() + PI / 2.0;
    let want = 40.0 * PI + 2.0 * PI * (3.0 * t / 2.0 + 1.6 / 3.0) + 2.0 * PI * 0.4 / 3.0;
    let o = offset(&c, 1.0, "o").unwrap();
    close(measure::volume(&o), want);
    assert!(crate::check::check(&o).is_empty());
}

#[test]
fn a_frustums_cavity_closing_below_its_far_face_is_the_cone_of_its_apex() {
    // The 6 to 3 frustum, 4 tall (sin a = 0.6, the whole cone 8 tall), shelled 2.7 thick: the
    // cavity cone's apex is at 8 - 2.7 / 0.6 = 3.5, below the top at 4, and its base on the
    // bottom moved up to 2.7: a cone 0.8 tall of radius 0.6. Open at the top and at both ends
    // alike (the bottom out to -2.7: a cone 3.5 tall from the plane z = 0, radius 2.625).
    let f = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 4.0, "f").unwrap();
    let face = |lo: f64, hi: f64| f.face_ids().find(|&g| {
        let b = crate::measure::face_bbox(&f, g);
        matches!(f.face(g).surface, Surface::Plane(_)) && b.min.z > lo && b.max.z < hi
    }).unwrap();
    let (bottom, top) = (face(-0.1, 0.1), face(3.9, 4.1));
    let whole = PI / 3.0 * 4.0 * (36.0 + 18.0 + 9.0);
    let sealed = shell(&f, &[top], 2.7, Wall::Inward, "s").unwrap();
    close(measure::volume(&sealed), whole - PI / 3.0 * 0.36 * 0.8);
    let through = shell(&f, &[bottom, top], 2.7, Wall::Inward, "s").unwrap();
    close(measure::volume(&through), whole - PI / 3.0 * 2.625 * 2.625 * 3.5);
}

#[test]
fn deleting_the_holes_of_a_plate_fills_them_however_the_plate_lies() {
    // A 10 x 5 plate, 1 thick, with three holes: deleting their walls leaves the whole plate,
    // 50, in any pose (the fill block must not stick out of a turned plate).
    let at = |k: f64| Vec2::new(2.5 * k, 2.5);
    let holes = (1..=3).map(|k| Profile::circle(at(k as f64), 1.2 + 0.1 * k as f64).loops.remove(0));
    let plate = Profile::rect(Vec2::new(5.0, 2.5), 10.0, 5.0).with_holes(holes);
    for z in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 2.0, 3.0).unit().unwrap(), Vec3::new(-2.0, 1.0, 0.5).unit().unwrap()] {
        let pose = Frame::new(Point3::new(1.0, -2.0, 3.0), z, z.any_perpendicular()).unwrap();
        let b = extrude(&plate, &pose, 1.0, "p").unwrap();
        let walls: Vec<FaceId> = b.face_ids().filter(|&f| matches!(b.face(f).surface, Surface::Cylinder(_))).collect();
        assert_eq!(walls.len(), 3);
        let filled = delete_faces(&b, &walls).unwrap();
        close(measure::volume(&filled), 50.0);
    }
}

#[test]
fn deleting_two_holes_leaves_the_one_between_them() {
    // The 10 x 5 plate with holes of diameter 1, 1.5 and 2 along its middle: deleting the first and
    // the last fills them and leaves the middle one, V = 50 - pi (0.75)^2 = 48.2327 (1 thick).
    let holes = [(2.0, 1.0), (5.0, 1.5), (8.0, 2.0)].map(|(x, d)| Profile::circle(Vec2::new(x, 2.5), d).loops.remove(0));
    let b = extrude(&Profile::rect(Vec2::new(5.0, 2.5), 10.0, 5.0).with_holes(holes), &Frame::WORLD, 1.0, "p").unwrap();
    let wall = |x: f64| b.face_ids().find(|&f| matches!(b.face(f).surface, Surface::Cylinder(_)) && (crate::measure::face_bbox(&b, f).center().x - x).abs() < 0.1).unwrap();
    let out = delete_faces(&b, &[wall(2.0), wall(8.0)]).unwrap();
    close(measure::volume(&out), 50.0 - PI * 0.75 * 0.75);
}

#[test]
fn deleting_a_boss_leaves_the_plate_and_a_round_in_an_inside_corner_the_sharp_corner() {
    // A disc of radius 8 and 4 thick under a post of radius 3 and 8 tall: deleting the post's wall and
    // top leaves the disc, V = pi 8^2 4.
    let disc = crate::primitive::cylinder(&Frame::new(Point3::new(0.0, 0.0, -4.0), Vec3::Z, Vec3::X).unwrap(), 8.0, 4.0, "d").unwrap();
    let post = crate::primitive::cylinder(&Frame::WORLD, 3.0, 8.0, "p").unwrap();
    let part = boolean::union(&disc, &post).unwrap();
    let wall = part.face_ids().find(|&f| matches!(&part.face(f).surface, Surface::Cylinder(c) if c.radius < 4.0)).unwrap();
    let top = part.face_ids().find(|&f| matches!(&part.face(f).surface, Surface::Plane(_)) && crate::measure::face_bbox(&part, f).min.z > 7.9).unwrap();
    let out = delete_faces(&part, &[wall, top]).unwrap();
    close(measure::volume(&out), PI * 64.0 * 4.0);
}

#[test]
fn a_shell_goes_past_a_round_thinner_than_its_wall() {
    // A puck of radius 5 and 4 tall, its top rim rounded by 0.5, shelled 1 thick open at the bottom: the
    // round is thinner than the wall and closes away, the cavity a cylinder of radius 4 and height 3.
    // The rounded puck holds pi 25 4 less what the round took off the rim (Pappus: the square less the
    // quarter disc, 0.5^2 (1 - pi/4), its centre of area 0.5 (5/6 - pi/4) / (1 - pi/4) in from the rim).
    let puck = crate::primitive::cylinder(&Frame::WORLD, 5.0, 4.0, "p").unwrap();
    let rim = puck.edge_ids().find(|&e| matches!(puck.edge(e).curve, Some(Curve::Circle(ref c)) if c.frame.origin.z > 3.9)).unwrap();
    let rounded = blend::fillet_edges(&puck, &[rim], 0.5, "r").unwrap();
    let base = rounded.face_ids().find(|&f| matches!(rounded.face(f).surface, Surface::Plane(_)) && crate::measure::face_bbox(&rounded, f).max.z < 0.1).unwrap();
    let area = 0.25 * (1.0 - PI / 4.0);
    let inward = 0.5 * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0);
    let outer = PI * 25.0 * 4.0 - 2.0 * PI * (5.0 - inward) * area;
    let cup = shell(&rounded, &[base], 1.0, Wall::Inward, "s").unwrap();
    close(measure::volume(&cup), outer - PI * 16.0 * 3.0);
}

#[test]
fn a_healed_cone_is_one_face_again() {
    // A frustum 6 to 3, 6 tall, its top rim rounded by 1: deleting the round heals the cone and
    // the top, three faces again (cone, top, bottom), the frustum's 126 pi.
    let cone = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 6.0, "c").unwrap();
    let rim = cone.edge_ids().find(|&e| matches!(cone.edge(e).curve, Some(Curve::Circle(ref c)) if c.frame.origin.z > 5.9)).unwrap();
    let rounded = blend::fillet_edges(&cone, &[rim], 1.0, "r").unwrap();
    let round = rounded.face_ids().find(|&f| matches!(rounded.face(f).surface, Surface::Torus(_))).unwrap();
    let healed = delete_faces(&rounded, &[round]).unwrap();
    close(measure::volume(&healed), 126.0 * PI);
    assert_eq!(healed.face_ids().count(), 3);
    assert!(healed.face_ids().all(|f| !healed.face(f).names.is_empty()));
}

#[test]
fn a_cube_edge_rounded_with_a_radius_growing_along_it() {
    // brep-bench fillet/variable-radius-edge: the 10 cube, the edge x = y = 10 rounded by circles
    // square to it, radius 1 + z / 10: 1000 - (70/3)(1 - pi/4).
    let cube = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "c").unwrap();
    let edge = cube
        .edge_ids()
        .find(|&e| {
            let (a, b) = (cube.vertex(cube.edge(e).start).point, cube.vertex(cube.edge(e).end).point);
            (a.x - 10.0).abs() < 1e-9 && (a.y - 10.0).abs() < 1e-9 && (b.x - 10.0).abs() < 1e-9 && (b.y - 10.0).abs() < 1e-9 // tol: the edge x = y = 10
        })
        .unwrap();
    let rounded = fillet_variable(&cube, edge, Point3::new(10.0, 10.0, 0.0), 1.0, 2.0, "f").unwrap();
    let want = 1000.0 - 70.0 / 3.0 * (1.0 - PI / 4.0);
    assert!((measure::volume(&rounded) - want).abs() < 1e-9 * want, "{} against {want}", measure::volume(&rounded));
    // The other way round is the mirror image.
    let flipped = fillet_variable(&cube, edge, Point3::new(10.0, 10.0, 10.0), 1.0, 2.0, "f").unwrap();
    assert!((measure::volume(&flipped) - want).abs() < 1e-9 * want);
    assert!(rounded.face_ids().any(|f| rounded.face(f).names.iter().any(|n| n.face == "round")));
}

#[test]
fn every_face_of_a_shell_is_named_from_the_start() {
    // The frustum whose cavity closes below its top (the cone of its apex), a puck's rounds taken
    // out of its cavity, and a block wrapped outward: each face is named, the cavity's and the
    // outward wall's each for the face of the solid it offsets.
    let named = |b: &Brep| b.face_ids().all(|f| !b.face(f).names.is_empty());
    let frustum = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 4.0, "f").unwrap();
    let top = frustum.face_ids().find(|&g| matches!(frustum.face(g).surface, Surface::Plane(_)) && crate::measure::face_bbox(&frustum, g).min.z > 3.9).unwrap();
    let sealed = shell(&frustum, &[top], 2.7, Wall::Inward, "s").unwrap();
    assert!(named(&sealed));
    let offsets = |b: &Brep| b.face_ids().filter(|&f| b.face(f).names.iter().any(|n| n.face.starts_with("inner_f"))).count();
    assert_eq!(offsets(&sealed), 2);
    let block = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
    let lid = block.face_ids().find(|&g| crate::measure::face_bbox(&block, g).min.z > 9.9).unwrap();
    let wrapped = shell(&block, &[lid], 1.0, Wall::Outward, "s").unwrap();
    assert!(named(&wrapped));
    assert_eq!(offsets(&wrapped), 5);
}

#[test]
fn a_hemispheres_rim_chamfered_by_distances_along_its_faces() {
    // A hemisphere of radius 5 on its plane z = 0, its rim chamfered 2 along the plane and 1 along the
    // sphere (a chord of 1: a turn of 2 asin(1/10)): the corner left of the chord from (3, 0) to (5 cos 0.2, 5 sin 0.2)
    // in the half plane, turned about the axis: the triangle with the plane, whose centroid is the
    // mean of its corners, and the circular segment out to the arc (Pappus on each).
    let o = Point3::ZERO;
    let f = Frame::from_axis(o, Vec3::Z).unwrap();
    let line = |a: [f64; 2], b: [f64; 2]| Segment::Line { a: Vec2::new(a[0], a[1]), b: Vec2::new(b[0], b[1]) };
    let meridian = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                named(line([0.0, 0.0], [5.0, 0.0]), "base"),
                named(Segment::Arc { center: Vec2::ZERO, radius: 5.0, start: 0.0, sweep: PI / 2.0 }, "dome"),
                named(line([0.0, 5.0], [0.0, 0.0]), "axis"),
            ],
        }],
    };
    let dome = revolve(&meridian, &Frame::new(o, -f.y, f.x).unwrap(), o, f.z, TAU, "h").unwrap();
    let rim = dome.edge_ids().find(|&e| matches!(dome.edge(e).curve, Some(Curve::Circle(ref c)) if c.frame.origin.z.abs() < 1e-9 && (c.radius - 5.0).abs() < 1e-9)).unwrap();
    let base = dome.face_ids().find(|&g| matches!(dome.face(g).surface, Surface::Plane(_))).unwrap();
    let cut = chamfer_two(&dome, rim, base, 2.0, 1.0, "c").unwrap();
    let theta = 2.0 * 0.1_f64.asin();
    let (x2, y2) = (5.0 * theta.cos(), 5.0 * theta.sin());
    let triangle = 2.0 * PI * ((3.0 + 5.0 + x2) / 3.0) * y2;
    let segment_area = 12.5 * (theta - theta.sin());
    let centroid = 4.0 * 5.0 * (theta / 2.0).sin().powi(3) / (3.0 * (theta - theta.sin())) * (theta / 2.0).cos();
    let taken = triangle + 2.0 * PI * centroid * segment_area;
    close(measure::volume(&cut), 2.0 / 3.0 * PI * 125.0 - taken);
    assert!(cut.face_ids().any(|g| cut.face(g).names.iter().any(|n| n.face == "bevel_0")));
}

#[test]
fn a_rounded_frusta_cavity_closing_below_its_top_is_the_cone_of_its_apex() {
    // The frustum 6 to 3, 6 tall (the whole cone 12 tall, sin a = 1/sqrt 5), its bottom rim rounded by
    // 1.5, shelled 2 open at the top. The round is thinner than the wall and goes; the cavity is the
    // whole cone's offset in by 2: its apex 2 sqrt 5 below 12, its base the bottom plane moved up
    // to z = 2, so a cone of height h = 12 - 2 sqrt 5 - 2 and base radius h / 2 (tan a = 1/2): its
    // volume pi h^3 / 12, of which the tip above the top (z > 6) is not in the solid. The wall holds the rounded solid less the rest, which lies clear of the
    // round (the round reaches 2.17 up the side, the cavity 2 in from it).
    let cone = crate::primitive::cone(&Frame::WORLD, 6.0, 3.0, 6.0, "f").unwrap();
    let rim = cone.edge_ids().find(|&e| matches!(cone.edge(e).curve, Some(Curve::Circle(ref c)) if c.frame.origin.z < 0.1)).unwrap();
    let rounded = blend::fillet_edges(&cone, &[rim], 1.5, "r").unwrap();
    let top = rounded.face_ids().find(|&g| matches!(rounded.face(g).surface, Surface::Plane(_)) && crate::measure::face_bbox(&rounded, g).min.z > 5.9).unwrap();
    let cup = shell(&rounded, &[top], 2.0, Wall::Inward, "s").unwrap();
    assert!(cup.face_ids().all(|g| !cup.face(g).names.is_empty()));
    // Less the tip above the top at z = 6, a cone of height k = 6 - 2 sqrt 5, which the wall never held.
    let (h, k) = (10.0 - 2.0 * 5.0_f64.sqrt(), 6.0 - 2.0 * 5.0_f64.sqrt());
    close(measure::volume(&cup), measure::volume(&rounded) - PI * (h.powi(3) - k.powi(3)) / 12.0);
}

#[test]
fn a_concave_rim_is_chamfered_by_filling_its_corner() {
    // A disc of radius 5 and 2 thick under a post of radius 2 and 3 tall: the circle where the post
    // meets the disc's top is concave. Chamfered 1 along the top and 1 up the post, it gains the
    // triangle (2, 2), (3, 2), (2, 3) in the half plane turned about the axis: 2 pi (7/3) (1/2).
    let disc = crate::primitive::cylinder(&Frame::WORLD, 5.0, 2.0, "d").unwrap();
    let post = crate::primitive::cylinder(&Frame::WORLD.translated(Vec3::new(0.0, 0.0, 2.0)), 2.0, 3.0, "p").unwrap();
    let part = boolean::union(&disc, &post).unwrap();
    let rim = part.edge_ids().find(|&e| matches!(part.edge(e).curve, Some(Curve::Circle(ref c)) if (c.radius - 2.0).abs() < 1e-9 && (c.frame.origin.z - 2.0).abs() < 1e-9)).unwrap();
    let top = part.face_ids().find(|&g| matches!(part.face(g).surface, Surface::Plane(_)) && part.edge_uses(rim).iter().any(|&u| part.coedge_face(u) == g)).unwrap();
    let chamfered = chamfer_two(&part, rim, top, 1.0, 1.0, "c").unwrap();
    close(measure::volume(&chamfered), measure::volume(&part) + 7.0 * PI / 3.0);
    assert!(chamfered.face_ids().any(|g| chamfered.face(g).names.iter().any(|n| n.face == "bevel_0")));
}

/// The edge of  through , found again after a cut renumbers the solid.
fn edge_through(b: &Brep, p: Point3) -> EdgeId {
    b.edge_ids()
        .filter_map(|e| {
            let x = b.edge(e);
            let c = x.curve.as_ref()?;
            let t = c.project(p, x.range.0.min(x.range.1), x.range.0.max(x.range.1));
            Some((c.eval(t).dist(p), e))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|x| x.1)
        .unwrap()
}

#[test]
fn a_cube_chamfered_edge_by_edge_ends_each_chamfer_on_the_ones_before() {
    // The 10 cube, all twelve edges chamfered 2 one after another: a^3 - 6 a d^2 + 6 d^3 = 808
    // (twelve prisms d^2/2 (a - 2d) and a corner piece 3 d^3 / 4 at each of eight corners).
    let cube = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "c").unwrap();
    let mids: Vec<Point3> = cube.edge_ids().map(|e| { let x = cube.edge(e); x.curve.as_ref().unwrap().eval(0.5 * (x.range.0 + x.range.1)) }).collect();
    let mut part = cube;
    for p in mids {
        let e = edge_through(&part, p);
        let face = part.edge_uses(e).first().map(|&u| part.coedge_face(u)).unwrap();
        part = chamfer_two_straight(&part, e, face, 2.0, 2.0, "c").unwrap();
    }
    close(measure::volume(&part), 808.0);
    assert_eq!(part.face_ids().count(), 18);
    assert!(crate::check::check(&part).is_empty());
}

#[test]
fn a_boxs_top_edge_chamfered_unequally_loses_the_prism_of_its_triangle() {
    // A 20 x 10 x 6 box, the edge along x at y = 0, z = 6 chamfered 1 along the top and 2 down the
    // side: the triangle 1 x 2 / 2 = 1 times 20.
    let b = crate::primitive::block(&Frame::WORLD, Vec3::new(20.0, 10.0, 6.0), "b").unwrap();
    let e = edge_through(&b, Point3::new(10.0, 0.0, 6.0));
    let top = b.face_ids().find(|&g| crate::measure::face_bbox(&b, g).min.z > 5.9).unwrap();
    let cut = chamfer_two_straight(&b, e, top, 1.0, 2.0, "c").unwrap();
    close(measure::volume(&cut), 1200.0 - 20.0);
}

#[test]
fn a_concave_corner_is_filled_by_the_prism_of_its_triangle() {
    // The L of arms 10 x 3 and 3 x 10, 10 thick: its inside corner chamfered 7 (the walls' whole length)
    // gains the triangle 7 x 7 / 2 = 24.5, times 10: 245.
    let l = Profile::polygon(&[Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 3.0), Vec2::new(3.0, 3.0), Vec2::new(3.0, 10.0), Vec2::new(0.0, 10.0)], &["a", "b", "c", "d", "e", "f"]);
    let s = extrude(&l, &Frame::WORLD, 10.0, "l").unwrap();
    let e = edge_through(&s, Point3::new(3.0, 3.0, 5.0));
    let face = s.edge_uses(e).first().map(|&u| s.coedge_face(u)).unwrap();
    let filled = chamfer_two_straight(&s, e, face, 7.0, 7.0, "c").unwrap();
    close(measure::volume(&filled), measure::volume(&s) + 245.0);
}

#[test]
fn a_straight_chamfer_declines_a_convex_edge_of_a_solid_with_a_concave_one() {
    // The L's outer corner edge: the solid has a concave edge, so the prism might cut other material.
    let l = Profile::polygon(&[Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 3.0), Vec2::new(3.0, 3.0), Vec2::new(3.0, 10.0), Vec2::new(0.0, 10.0)], &["a", "b", "c", "d", "e", "f"]);
    let s = extrude(&l, &Frame::WORLD, 10.0, "l").unwrap();
    let e = edge_through(&s, Point3::new(0.0, 0.0, 5.0));
    let face = s.edge_uses(e).first().map(|&u| s.coedge_face(u)).unwrap();
    assert!(matches!(chamfer_two_straight(&s, e, face, 1.0, 1.0, "c"), Err(Error::Unsupported(_))));
}

/// The solid of `keep`'s half spaces `n . x <= c` inside the cube of side 40 about the origin.
fn polyhedron(planes: &[([f64; 3], f64)]) -> Brep {
    let cube = crate::primitive::block(&Frame::WORLD.translated(Vec3::new(-20.0, -20.0, -20.0)), Vec3::new(40.0, 40.0, 40.0), "p").unwrap();
    planes.iter().fold(cube, |solid, &(n, c)| {
        let n = Vec3::new(n[0], n[1], n[2]);
        let unit = n.unit().unwrap();
        // The box on the normal's side is what the plane takes off: keep the other side.
        let cut = boolean::half_space(&solid, unit * (c / n.norm()), unit).unwrap();
        boolean::difference(&solid, &cut).unwrap()
    })
}

#[test]
fn a_cube_with_every_edge_chamfered_has_its_corners_cut_flat() {
    // a = 10, d = 2. The twelve chamfers' prisms are d^2/2 (a - 2d) each between the corner cubes of side
    // d; in each of the eight corner cubes the chamfers and the flat corner leave only the tetrahedron
    // (d, d, d), (d, d, 0), (d, 0, d), (0, d, d), of d^3 / 6, so the cube loses
    // 12 (d^2/2)(a - 2d) + 8 (d^3 - d^3/6) = 144 + 53.33 = 197.33: V = 2408/3, with 6 + 12 + 8 = 26 faces.
    let cube = polyhedron(&[([1.0, 0.0, 0.0], 10.0), ([-1.0, 0.0, 0.0], 0.0), ([0.0, 1.0, 0.0], 10.0), ([0.0, -1.0, 0.0], 0.0), ([0.0, 0.0, 1.0], 10.0), ([0.0, 0.0, -1.0], 0.0)]);
    let edges: Vec<EdgeId> = cube.edge_ids().collect();
    let cut = chamfer_edges(&cube, &edges, 2.0, "c").unwrap();
    close(measure::volume(&cut), 2408.0 / 3.0);
    assert_eq!(cut.face_ids().count(), 26);
    let named = |name: &str| cut.face_ids().filter(|&g| cut.face(g).names.iter().any(|n| n.feature == "c" && n.face.starts_with(name))).count();
    assert_eq!((named("bevel_"), named("corner")), (12, 8));
    assert!(crate::check::check(&cut).is_empty());
}

#[test]
fn an_octahedron_with_every_edge_chamfered_has_its_corners_cut_square() {
    // |x| + |y| + |z| <= a = 10, chamfer planes |x| + |y| = a - k = 8 (distance sqrt 6, k = 2) and corner
    // planes |x| = 6. Per octant the three wedges beyond the chamfer planes are R = integral of
    // s (a - s) over a - k <= s <= a each, the three pairwise overlaps k^3 each less one triple k^3
    // share, so 3R - k^3 = 44; the six square pyramids beyond the corner planes are (4/3) k^3 = 32/3
    // each: V = 4 a^3 / 3 - 8 (3R - k^3) - 6 (4/3) k^3 = 4000/3 - 352 - 64 = 2752/3, and 26 faces.
    let mut planes = Vec::new();
    for sx in [1.0, -1.0] {
        for sy in [1.0, -1.0] {
            for sz in [1.0, -1.0] {
                planes.push(([sx, sy, sz], 10.0));
            }
        }
    }
    let octa = polyhedron(&planes);
    let edges: Vec<EdgeId> = octa.edge_ids().collect();
    assert_eq!(edges.len(), 12);
    let cut = chamfer_edges(&octa, &edges, 6.0_f64.sqrt(), "c").unwrap();
    let r = 5.0 * (100.0 - 64.0) - (1000.0 - 512.0) / 3.0;
    close(measure::volume(&cut), 4000.0 / 3.0 - 8.0 * (3.0 * r - 8.0) - 6.0 * 4.0 / 3.0 * 8.0);
    assert_eq!(cut.face_ids().count(), 26);
    assert!(crate::check::check(&cut).is_empty());
}

#[test]
fn a_corner_chamfer_declines_what_it_cannot_close_flat() {
    let cube = polyhedron(&[([1.0, 0.0, 0.0], 10.0), ([-1.0, 0.0, 0.0], 0.0), ([0.0, 1.0, 0.0], 10.0), ([0.0, -1.0, 0.0], 0.0), ([0.0, 0.0, 1.0], 10.0), ([0.0, 0.0, -1.0], 0.0)]);
    // Two of the three edges at a vertex chamfered: some but not all.
    let corner = cube.vertex_ids().next().unwrap();
    let two: Vec<EdgeId> = cube.edge_ids().filter(|&e| cube.edge(e).start == corner || cube.edge(e).end == corner).take(2).collect();
    assert!(matches!(chamfer_edges(&cube, &two, 1.0, "c"), Err(Error::Unsupported(_))));
    // A pyramid on a rectangle, 10 x 6 and 8 tall: the four edges at its apex have no flat corner.
    let (w, l, h) = (5.0, 3.0, 8.0);
    let pyramid = polyhedron(&[([0.0, 0.0, -1.0], 0.0), ([h, 0.0, w], h * w), ([-h, 0.0, w], h * w), ([0.0, h, l], h * l), ([0.0, -h, l], h * l)]);
    let all: Vec<EdgeId> = pyramid.edge_ids().collect();
    assert!(matches!(chamfer_edges(&pyramid, &all, 0.5, "c"), Err(Error::Unsupported(_))));
}

#[test]
fn an_l_blocks_inside_corner_rounded_to_the_whole_of_both_faces() {
    // The L of arms 10 x 3 and 3 x 10, 10 thick, its inside corner rounded to r = 7, which is the
    // whole length of both inside faces: the section is the 10 x 10 square less the quarter disc of
    // radius 7 about its far corner: V = 10 (10^2 - pi 7^2 / 4).
    let l = Profile::polygon(&[Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 3.0), Vec2::new(3.0, 3.0), Vec2::new(3.0, 10.0), Vec2::new(0.0, 10.0)], &["a", "b", "c", "d", "e", "f"]);
    let s = extrude(&l, &Frame::WORLD, 10.0, "l").unwrap();
    let e = edge_through(&s, Point3::new(3.0, 3.0, 5.0));
    let face = s.edge_uses(e).first().map(|&u| s.coedge_face(u)).unwrap();
    let round = fillet_edge(&s, e, face, 7.0, "r").unwrap();
    close(measure::volume(&round), 10.0 * (100.0 - PI * 49.0 / 4.0));
    assert!(round.face_ids().any(|g| round.face(g).names.iter().any(|n| n.face == "round_0")));
    assert!(crate::check::check(&round).is_empty());
}

#[test]
fn a_convex_edge_rounded_loses_the_corner_less_the_quarter_disc() {
    // The 10 cube's edge x = y = 10 rounded to r = 2 loses (1 - pi/4) r^2 along its 10.
    let cube = crate::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "c").unwrap();
    let e = edge_through(&cube, Point3::new(10.0, 10.0, 5.0));
    let face = cube.edge_uses(e).first().map(|&u| cube.coedge_face(u)).unwrap();
    let round = fillet_edge(&cube, e, face, 2.0, "r").unwrap();
    close(measure::volume(&round), 1000.0 - (1.0 - PI / 4.0) * 4.0 * 10.0);
}

#[test]
#[ignore = "probe: the opening of the octahedron by 2 (shrunk, then grown)"]
fn probe_octahedron_opening() {
    let mut planes = Vec::new();
    for sx in [1.0, -1.0] {
        for sy in [1.0, -1.0] {
            for sz in [1.0, -1.0] {
                planes.push(([sx, sy, sz], 10.0));
            }
        }
    }
    let octa = polyhedron(&planes);
    let mut report = String::new();
    match offset(&octa, -2.0, "o") {
        Err(e) => report.push_str(&format!("shrink: err {e}\n")),
        Ok(small) => {
            report.push_str(&format!("shrink: ok V {} faces {}\n", measure::volume(&small), small.face_ids().count()));
            match offset(&small, 2.0, "o") {
                Err(e) => report.push_str(&format!("grow: err {e}\n")),
                Ok(big) => report.push_str(&format!("grow: ok V {} A {} faces {} want V 1270.7657089174301 A 619.29504650969118\n", measure::volume(&big), measure::area(&big), big.face_ids().count())),
            }
        }
    }
    panic!("{report}");
}

#[test]
fn a_notched_cubes_inside_corner_is_rounded_by_strips_and_an_eighth_of_a_ball() {
    // The 20 cube less the cube from (10, 10, 10): three concave edges 10 long from the inside corner.
    // Rounded to r = 3 the added material is, along each edge from the corner block on, the strip of
    // section (1 - pi/4) r^2 (3 strips of length 10 - r), and at the corner the cube of side r less the
    // eighth of the ball of radius r about (13, 13, 13): r^3 - (pi/6) r^3 (= (1/8)(4/3) pi r^3).
    // V = 20^3 - 10^3 + 3 (1 - pi/4) r^2 (10 - r) + r^3 (1 - pi/6) = 7216 - 51.75 pi.
    let cube = crate::primitive::block(&Frame::WORLD, Vec3::new(20.0, 20.0, 20.0), "c").unwrap();
    let notch = crate::primitive::block(&Frame::WORLD.translated(Vec3::new(10.0, 10.0, 10.0)), Vec3::new(10.0, 10.0, 10.0), "n").unwrap();
    let s = boolean::difference(&cube, &notch).unwrap();
    let edges = [Point3::new(15.0, 10.0, 10.0), Point3::new(10.0, 15.0, 10.0), Point3::new(10.0, 10.0, 15.0)].map(|p| edge_through(&s, p));
    let rounded = fillet_concave_corner(&s, &edges, 3.0, "r").unwrap();
    close(measure::volume(&rounded), 7216.0 - 51.75 * PI);
    assert!(crate::check::check(&rounded).is_empty());
    let named = |name: &str| rounded.face_ids().filter(|&g| rounded.face(g).names.iter().any(|n| n.feature == "r" && n.face.starts_with(name))).count();
    assert_eq!((named("round_"), named("corner")), (3, 1));
    // Two edges, or edges that are not at right angles, are declined.
    assert!(matches!(fillet_concave_corner(&s, &edges[..2], 3.0, "r"), Err(Error::Unsupported(_))));
}

/// Steiner's polynomial of the convex polyhedron `b` grown by `r`: V + S r + M r^2 + 4 pi r^3 / 3 and
/// S + 2 M r + 4 pi r^2, M the sum over its edges of length x half the turning angle, all read off `b`.
fn steiner(b: &Brep, r: f64) -> (f64, f64) {
    let normal = |f: FaceId| match &b.face(f).surface {
        Surface::Plane(p) => if b.face(f).reversed { -p.frame.z } else { p.frame.z },
        _ => Vec3::Z,
    };
    let m: f64 = b
        .edge_ids()
        .map(|e| {
            let x = b.edge(e);
            let len = b.vertex(x.start).point.dist(b.vertex(x.end).point);
            let faces: Vec<FaceId> = b.edge_uses(e).iter().map(|&u| b.coedge_face(u)).collect();
            len * normal(faces[0]).dot(normal(faces[1])).clamp(-1.0, 1.0).acos() / 2.0
        })
        .sum();
    let (v, s) = (measure::volume(b), measure::area(b));
    (v + s * r + m * r * r + 4.0 * PI * r.powi(3) / 3.0, s + 2.0 * m * r + 4.0 * PI * r * r)
}

fn octahedron(s: f64) -> Brep {
    let mut planes = Vec::new();
    for sx in [1.0, -1.0] {
        for sy in [1.0, -1.0] {
            for sz in [1.0, -1.0] {
                planes.push(([sx, sy, sz], s));
            }
        }
    }
    polyhedron(&planes)
}

#[test]
fn a_convex_polyhedron_grown_by_a_ball_follows_steiners_polynomial() {
    // The octahedron |x| + |y| + |z| <= 10 - 2 sqrt 3 and a 3 x 4 x 5 box, each grown by a ball: the
    // checker is clean, every face and edge and vertex is swept (an octahedron has 8 + 12 + 6 = 26 pieces).
    for (solid, r, pieces) in [(octahedron(10.0 - 2.0 * 3.0_f64.sqrt()), 2.0, Some(26)), (crate::primitive::block(&Frame::WORLD, Vec3::new(3.0, 4.0, 5.0), "b").unwrap(), 1.5, Some(26))] {
        let grown = grow_convex(&solid, r, "g").unwrap();
        let (v, a) = steiner(&solid, r);
        close(measure::volume(&grown), v);
        close(measure::area(&grown), a);
        assert_eq!(grown.face_ids().count(), pieces.unwrap_or(0));
        assert!(crate::check::check(&grown).is_empty());
    }
}

#[test]
fn every_edge_of_an_octahedron_rounded_is_its_opening() {
    // |x| + |y| + |z| <= 10, r = 2: the rolling ball reaches the octahedron shrunk by r (10 - 2 sqrt 3,
    // the faces' planes moved in 2 meet at one point per corner), grown by r.
    let shrunk = octahedron(10.0 - 2.0 * 3.0_f64.sqrt());
    let rounded = round_all_edges(&octahedron(10.0), 2.0, "r").unwrap();
    let (v, a) = steiner(&shrunk, 2.0);
    close(measure::volume(&rounded), v);
    close(measure::area(&rounded), a);
    // A box rounded by more than half its thinnest side loses its faces in the shrink: declined.
    let thin = crate::primitive::block(&Frame::WORLD, Vec3::new(3.0, 4.0, 5.0), "b").unwrap();
    assert!(matches!(round_all_edges(&thin, 2.0, "r"), Err(Error::Unsupported(_) | Error::InvalidInput(_))));
}
