// yack: rigid face moves of rounded solids, against closed forms (w-h1, the move_faces pins).
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::PI;
use yack_build::primitive;
use yack_geom::Surface;
use yack_math::kernel::{Frame, Transform, Vec3};
use yack_measure as measure;

fn close(got: f64, want: f64) {
    assert!((got - want).abs() < 1e-7 * want.max(1.0), "{got} against {want}");
}

#[test]
fn a_rounded_top_moved_up_slides_its_round_along_the_wall() {
    // brep-bench edit/rounded-block-top-moved-up: a 40 × 30 × 10 block, its back top edge rounded
    // 3, the top and the round moved up 6. The round slides up the back wall it is tangent to: the
    // block 16 tall with that edge rounded 3, V = 40·30·16 − (1 − π/4)·9·40, A = 4382 + 64.5π.
    let b = primitive::block(&Frame::WORLD, Vec3::new(40.0, 30.0, 10.0), "b").unwrap();
    let edge = b
        .edge_ids()
        .find(|&e| {
            let x = b.edge(e);
            x.curve.as_ref().is_some_and(|c| c.eval(0.5 * (x.range.0 + x.range.1)).dist(Vec3::new(20.0, 30.0, 10.0)) < 1e-6)
        })
        .unwrap();
    let rounded = yack_blend::fillet_edges(&b, &[edge], 3.0, "r").unwrap();
    let faces: Vec<_> = rounded
        .face_ids()
        .filter(|&f| match &rounded.face(f).surface {
            Surface::Cylinder(_) => true,
            Surface::Plane(p) => p.frame.z.z.abs() > 0.5 && measure::face_bbox(&rounded, f).min.z > 9.9,
            _ => false,
        })
        .collect();
    assert_eq!(faces.len(), 2);
    let moved = yack_edit::move_faces_by(&rounded, &faces, &Transform::translation(Vec3::new(0.0, 0.0, 6.0))).unwrap();
    close(measure::volume(&moved), 40.0 * 30.0 * 16.0 - (1.0 - PI / 4.0) * 9.0 * 40.0);
    close(measure::area(&moved), 4382.0 + 64.5 * PI);
}

#[test]
fn an_l_s_sides_drafted_until_its_arms_close_leave_a_roof() {
    // brep-bench draft/l-block-sides-drafted-until-its-arms-close: the L of arms 20 × 5 and 5 × 15,
    // 8 tall, every side drafted 30° about its foot. At height z its outline is the L moved in by
    // δ = z·tan 30°, of area 150 − 70δ + 4δ² (five square corners and one reflex) and outline
    // 70 − 8δ; the arms close at δ = 2.5, below the top, so the top goes and a ridge is left.
    // V = ∫ area dz, A = 150 + ∫ outline dz / cos 30°.
    let pts: Vec<yack_math::kernel::Vec2> =
        [[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]].iter().map(|p| yack_math::kernel::Vec2::new(p[0], p[1])).collect();
    let names = ["e0", "e1", "e2", "e3", "e4", "e5"];
    let l = yack_build::sweep::extrude(&yack_profile::Profile::polygon(&pts, &names), &Frame::WORLD, 8.0, "l").unwrap();
    let sides: Vec<_> = l.face_ids().filter(|&f| { let bb = measure::face_bbox(&l, f); bb.max.z - bb.min.z > 1.0 }).collect();
    let a = 30f64.to_radians();
    let roof = yack_edit::draft(&l, &sides, &Frame::WORLD, a).unwrap();
    let t = a.tan();
    let z = 2.5 / t;
    close(measure::volume(&roof), 150.0 * z - 35.0 * t * z * z + 4.0 / 3.0 * t * t * z * z * z);
    close(measure::area(&roof), 150.0 + (70.0 * z - 4.0 * t * z * z) / a.cos());
}
