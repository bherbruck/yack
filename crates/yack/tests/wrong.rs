// yack: T1 regressions for the 14 brep-bench cases yack M1a answered wrong (runs/brep-bench/yack-m1a.json):
// each must now refuse, or at least not return the wrong volume. Case programs copied from
// crates/yack-bench/src/cases/.
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use yack::prelude::*;

/// The case must refuse, or give a volume other than `wrong`.
fn not(got: Result<Solid>, wrong: f64) {
    if let Ok(s) = got {
        assert!(s.is_exact(), "a faceted answer (strategy {})", s.strategy());
        let v = s.volume();
        assert!((v - wrong).abs() > 1e-6 * wrong.abs().max(1.0), "still the wrong volume {v} (strategy {})", s.strategy());
    }
}

#[test]
fn boss_deleted() {
    not(
        (|| {
            let part = cylinder(&Frame::at([0.0, 0.0, -4.0]), 8.0, 4.0)?.union(&cylinder(&Frame::WORLD, 3.0, 8.0)?)?;
            part.delete_faces(&[part.face_at([0.0, 3.0, 4.0])?, part.face_at([0.0, 0.0, 8.0])?])
        })(),
        1030.4423903774518,
    );
}

#[test]
fn boss_root_round_deleted() {
    not(
        (|| {
            let part = cylinder(&Frame::at([0.0, 0.0, -4.0]), 8.0, 4.0)?.union(&cylinder(&Frame::WORLD, 3.0, 8.0)?)?;
            let rounded = part.fillet(&[part.edge_at([0.0, 3.0, 0.0])?], 1.0)?;
            rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), SurfaceKind::Torus))?)
        })(),
        1034.7887252527948,
    );
}

#[test]
fn l_block_inside_round_deleted() {
    not(
        (|| {
            let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 5.0], [5.0, 5.0], [5.0, 15.0], [0.0, 15.0]])?;
            let b = extrude(&Frame::WORLD, &l, 8.0)?;
            let rounded = b.fillet(&[b.edge_at([5.0, 5.0, 4.0])?], 2.0)?;
            rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), SurfaceKind::Cylinder { .. }))?)
        })(),
        1206.8672587712813,
    );
}

#[test]
fn rounded_box_top_rim_chain_deleted() {
    not(
        (|| {
            let b = block(&Frame::WORLD, [20.0, 12.0, 8.0])?;
            let sides = [b.edge_at([0.0, 0.0, 4.0])?, b.edge_at([20.0, 0.0, 4.0])?, b.edge_at([20.0, 12.0, 4.0])?];
            let b = b.fillet(&[sides[0], sides[1], sides[2], b.edge_at([0.0, 12.0, 4.0])?], 3.0)?;
            let rimmed = b.fillet(&b.edges_of(b.face_at([10.0, 6.0, 8.0])?)?, 1.0)?;
            let chain = rimmed.faces_where(|f| {
                matches!(f.kind(), SurfaceKind::Cylinder { radius, .. } if radius < 2.0) || matches!(f.kind(), SurfaceKind::Torus)
            })?;
            rimmed.delete_faces(&chain)
        })(),
        1845.8666338298958,
    );
}

#[test]
fn shaft_two_flats_deleted() {
    not(
        (|| {
            let s = cylinder(&Frame::WORLD, 5.0, 20.0)?.split(&Plane::new([4.0, 0.0, 0.0], [-1.0, 0.0, 0.0])?)?.front;
            let s = s.split(&Plane::new([-4.0, 0.0, 0.0], [1.0, 0.0, 0.0])?)?.front;
            s.delete_faces(&[s.face_at([4.0, 0.0, 10.0])?, s.face_at([-4.0, 0.0, 10.0])?])
        })(),
        1407.2952180016118,
    );
}

#[test]
fn variable_radius_edge() {
    not(
        (|| {
            let cube = block(&Frame::WORLD, [10.0, 10.0, 10.0])?;
            cube.fillet_variable(cube.edge_at([10.0, 10.0, 5.0])?, [10.0, 10.0, 0.0], 1.0, 2.0)
        })(),
        995.0090582581586,
    );
}

#[test]
fn square_to_round() {
    not(
        (|| {
            let square = Profile::rectangle([-20.0, -20.0], [20.0, 20.0])?;
            loft_through(&Frame::WORLD, &[(0.0, square), (30.0, Profile::circle([0.0, 0.0], 15.0)?)])
        })(),
        33833.87146289993,
    );
}

#[test]
fn drilled_plate_grown_by_its_hole() {
    not(
        (|| {
            let plate = block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 4.0])?;
            plate.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 3.0, 6.0)?)?.offset(3.0)
        })(),
        6317.168026350787,
    );
}

#[test]
fn dumbbell_neck_filled() {
    not(
        (|| {
            let outline = [[0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [14.0, 4.0], [14.0, 0.0], [24.0, 0.0], [24.0, 10.0]];
            let outline = [&outline[..], &[[14.0, 10.0], [14.0, 6.0], [10.0, 6.0], [10.0, 10.0], [0.0, 10.0]]].concat();
            let plate = extrude(&Frame::WORLD, &Profile::polygon(&outline)?, 6.0)?;
            plate.shell(&[plate.face_at([5.0, 5.0, 6.0])?], 1.5)
        })(),
        806.9999999999998,
    );
}

#[test]
fn l_block_open_at_its_step_sharp() {
    not(
        (|| {
            let l = Profile::polygon(&[[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [15.0, 20.0], [15.0, 5.0], [0.0, 5.0]])?;
            let b = extrude(&Frame::WORLD, &l, 20.0)?;
            b.shell(&[b.face_at([7.0, 5.0, 10.0])?], 1.0)
        })(),
        1429.999999999999,
    );
}

#[test]
fn wedge_a_millionth_steep() {
    not(
        (|| {
            let cube = block(&Frame::at([-5.0, -5.0, -10.0]), [10.0, 10.0, 10.0])?;
            Ok(cube.split(&Plane::new([0.0, 0.0, 0.0], [0.0, -1e-6, 1.0])?)?.front)
        })(),
        0.0,
    );
}

#[test]
fn cone_frustum_thickened() {
    not(
        (|| {
            let c = cone(&Frame::WORLD, 6.0, 3.0, 4.0)?;
            c.thicken(&[c.face_at([0.0, 4.5, 2.0])?], 1.0, Join::Round)
        })(),
        161.00662349647666,
    );
}

#[test]
fn spherical_cap_thickened() {
    not(
        (|| {
            let cap = sphere(&Frame::WORLD, 10.0)?.split(&Plane::new([0.0, 0.0, 6.0], [0.0, 0.0, 1.0])?)?.front;
            cap.thicken(&[cap.face_at([0.0, 6.0, 8.0])?], 2.0, Join::Round)
        })(),
        695.3391739945413,
    );
}

#[test]
fn a_shell_as_thick_as_the_part_is_refused() {
    let b = block(&Frame::WORLD, [2.0, 2.0, 2.0]).unwrap();
    assert!(b.shell(&[b.face_at([1.0, 1.0, 2.0]).unwrap()], 1.5).is_err());
}

#[test]
fn a_torus_tube_across_its_axis_is_invalid() {
    let e = torus(&Frame::WORLD, 1.0, 2.0).unwrap_err();
    assert_eq!(e.kind, ErrorKind::Invalid);
    assert!(e.detail.contains("spindle_torus"), "{e}");
    match spindle_torus(&Frame::WORLD, 1.0, 2.0) {
        Ok(s) => assert!(s.volume() > 0.0),
        Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported, "{e}"),
    }
    assert_eq!(spindle_torus(&Frame::WORLD, 3.0, 1.0).unwrap_err().kind, ErrorKind::Invalid);
}

#[test]
fn a_guard_redirects_but_keeps_springreps_answer() {
    // a plate whose hole closes when grown past its radius: springrep's answer is held by the
    // guard and returned when cadrs cannot do better (never refused)
    let plate = block(&Frame::at([-10.0, -10.0, 0.0]), [20.0, 20.0, 4.0]).unwrap();
    let drilled = plate.difference(&cylinder(&Frame::at([0.0, 0.0, -1.0]), 2.0, 6.0).unwrap()).unwrap();
    let grown = drilled.offset(3.0);
    assert!(grown.is_ok(), "{grown:?}");
}

#[test]
fn half_torus_outer_rim_round_deleted() {
    // springrep rounded both rims of the flat face when given the outer one (4 faces, not 3)
    let half = torus(&Frame::WORLD, 6.0, 2.0).unwrap().split(&Plane::new([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]).unwrap()).unwrap().front;
    let rounded = half.fillet(&[half.edge_at([0.0, 8.0, 0.0]).unwrap()], 0.5);
    if let Ok(r) = rounded {
        let rounds = r.faces_where(|f| matches!(f.kind(), SurfaceKind::Torus) && f.area() < 100.0).unwrap();
        assert!(rounds.len() <= 1, "{} small toroidal rounds (strategy {})", rounds.len(), r.strategy());
    }
}

/// swept/square-on-a-planet-carrier: once a faceted 155.132; it must build exactly, 155.336678064.
#[test]
fn planet_carrier_sweep_is_exact() {
    let bar = block(&Frame::at([4.0, -1.0, 0.0]), [2.0, 2.0, 2.0]).unwrap();
    let spin = Motion::turn(Point3::new(5.0, 0.0, 0.0), Dir3::Z, deg(0.0), deg(-1080.0));
    let carry = Motion::turn(Point3::ORIGIN, Dir3::Z, deg(0.0), deg(360.0));
    let s = bar.swept(&Motion::together(vec![spin, carry])).unwrap();
    assert!(s.is_exact() && (s.volume() - 155.336678064).abs() < 1e-6 * 155.34, "{} ({})", s.volume(), s.strategy());
}

/// swept/rack-cut-tooth-space: once a faceted cut, 9042.5268; it must build exactly, 9042.502788700.
#[test]
fn rack_roll_is_exact() {
    let (d, a, b) = (-20.0 * std::f64::consts::FRAC_PI_3, 0.842856, 3.026666);
    let tooth = Profile::polygon(&[[d - a, 18.0], [d + a, 18.0], [d + b, 24.0], [d - b, 24.0]]).unwrap();
    let side = Frame::new([0.0, -2.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]).unwrap();
    let rack = extrude(&side, &tooth, 10.0).unwrap();
    let turn = Motion::turn(Point3::ORIGIN, Dir3::Y, deg(-60.0), deg(60.0));
    let roll = Motion::together(vec![Motion::slide(Vec3::new(0.0, 0.0, -2.0 * d)), turn]);
    let blank = cylinder(&Frame::along([0.0; 3], [0.0, 1.0, 0.0]).unwrap(), 22.0, 6.0).unwrap();
    let s = blank.cut_swept(&rack, &roll).unwrap();
    assert!(s.is_exact() && (s.volume() - 9042.502788700).abs() < 1e-6 * 9042.5, "{} ({})", s.volume(), s.strategy());
}
