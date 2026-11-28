// yack: rigid face moves of rounded solids, against closed forms (w-h1, the move_faces pins).
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};
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

#[test]
fn an_elliptic_wall_moved_out_is_the_offset_ellipse_s_prism() {
    // brep-bench offset/elliptic-cylinder-grown's face move: the ellipse 5 × 3 as four exact
    // quarter arcs (as the facade draws its conics), 10 tall, every face moved out 1. The walls'
    // offsets are fitted, and where two meet tangent the corner settles to the fits' budget. The
    // moved solid is the prism of the ellipse's offset curve, 12 tall: for a smooth convex curve of
    // area A and perimeter P that curve holds A + Pd + πd² and runs P + 2πd.
    use yack_math::kernel::Vec2;
    let quarter = |k: f64| {
        yack_profile::NamedSegment::new(
            yack_profile::Segment::Ellipse { center: Vec2::new(0.0, 0.0), rotation: 0.0, major: 5.0, minor: 3.0, start: k * FRAC_PI_2, sweep: FRAC_PI_2 },
            "wall",
        )
    };
    let outline = yack_profile::Profile { loops: vec![yack_profile::ProfileLoop { segments: (0..4).map(|k| quarter(f64::from(k))).collect() }] };
    let can = yack_build::sweep::extrude(&outline, &Frame::WORLD, 10.0, "e").unwrap();
    let by: std::collections::BTreeMap<_, _> = can.face_ids().map(|f| (f, 1.0)).collect();
    let moved = yack_edit::offset_faces(&can, &by).unwrap();
    // The ellipse's perimeter by the midpoint rule on 4096 steps (its integrand is smooth and
    // periodic, so the rule is exact to rounding long before that).
    let n = 4096;
    let p: f64 = (0..n)
        .map(|i| {
            let t = TAU * (f64::from(i) + 0.5) / f64::from(n);
            (5.0 * t.sin()).hypot(3.0 * t.cos())
        })
        .sum::<f64>()
        * TAU
        / f64::from(n);
    let (a, d, h) = (15.0 * PI, 1.0, 12.0);
    let (plan, run) = (a + p * d + PI * d * d, p + TAU * d);
    close(measure::volume(&moved), plan * h);
    close(measure::area(&moved), 2.0 * plan + run * h);
}
