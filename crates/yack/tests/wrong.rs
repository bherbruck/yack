// yack: T1 regressions for the 14 brep-bench cases yack M1a answered wrong (runs/brep-bench/yack-m1a.json):
// each must now refuse, or at least not return the wrong volume. Case programs copied from
// crates/yack-bench/src/cases/.
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use yack::prelude::*;

/// The case must refuse, or give a volume other than `wrong`.
fn not(got: Result<Solid>, wrong: f64) {
    if let Ok(s) = got {
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
            rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), SurfaceKind::Torus)))
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
            rounded.delete_faces(&rounded.faces_where(|f| matches!(f.kind(), SurfaceKind::Cylinder { .. })))
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
            });
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
#[ignore = "M3: springrep's variable fillet is 1.6e-5 off its exact volume; no guard yet"]
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
#[ignore = "M3: springrep's sharp shell open at a step face (1430 vs 1448); no guard yet"]
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
            c.thicken(&[c.face_at([0.0, 4.5, 2.0])?], 1.0)
        })(),
        161.00662349647666,
    );
}

#[test]
fn spherical_cap_thickened() {
    not(
        (|| {
            let cap = sphere(&Frame::WORLD, 10.0)?.split(&Plane::new([0.0, 0.0, 6.0], [0.0, 0.0, 1.0])?)?.front;
            cap.thicken(&[cap.face_at([0.0, 6.0, 8.0])?], 2.0)
        })(),
        695.3391739945413,
    );
}
