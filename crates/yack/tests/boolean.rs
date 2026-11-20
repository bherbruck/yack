// yack: T1 checks of booleans and splits through the public API, against closed forms.
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use std::f64::consts::PI;

use yack::prelude::*;

fn close(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-7 * want.abs().max(1.0), "{got} vs {want}");
}

fn cube(at: [f64; 3]) -> Solid {
    block(&Frame::at(at), [2.0, 2.0, 2.0]).unwrap()
}

#[test]
fn overlapping_cubes() {
    let (a, b) = (cube([0.0; 3]), cube([1.0, 1.0, 1.0]));
    close(a.union(&b).unwrap().volume(), 15.0);
    close(a.intersection(&b).unwrap().volume(), 1.0);
    let d = a.difference(&b).unwrap();
    close(d.volume(), 7.0);
    assert!(d.check().is_empty());
}

#[test]
fn a_drilled_block() {
    let b = block(&Frame::at([-5.0, -5.0, 0.0]), [10.0, 10.0, 4.0]).unwrap();
    let rod = cylinder(&Frame::at([0.0, 0.0, -1.0]), 2.0, 6.0).unwrap();
    let d = b.difference(&rod).unwrap();
    assert!(d.check().is_empty());
    close(d.volume(), 400.0 - PI * 4.0 * 4.0);
    assert_eq!(d.betti(), [1, 1, 0]);
}

#[test]
fn empties_are_identities() {
    let a = cube([0.0; 3]);
    close(a.union(&empty()).unwrap().volume(), 8.0);
    assert!(a.intersection(&empty()).unwrap().is_empty());
    close(a.difference(&empty()).unwrap().volume(), 8.0);
    assert!(empty().difference(&a).unwrap().is_empty());
    assert!(Solid::union_all(&[]).unwrap().is_empty());
}

#[test]
fn union_all_and_difference_all() {
    let row: Vec<Solid> = (0..3).map(|i| cube([f64::from(i) * 1.5, 0.0, 0.0])).collect();
    close(Solid::union_all(&row).unwrap().volume(), 8.0 * 3.0 - 2.0 * 2.0);
    let slab = block(&Frame::at([-1.0, 0.5, 0.5]), [8.0, 1.0, 1.0]).unwrap();
    close(slab.difference_all(&row).unwrap().volume(), 8.0 - 5.0);
}

#[test]
fn a_split_cube() {
    let a = cube([0.0; 3]);
    let s = a.split(&Plane::new([0.0, 0.0, 0.5], [0.0, 0.0, 1.0]).unwrap()).unwrap();
    close(s.front.volume(), 6.0);
    close(s.back.volume(), 2.0);
}

#[test]
fn queries_on_a_block() {
    let a = cube([1.0, 2.0, 3.0]);
    let bb = a.bounding_box().unwrap();
    assert!((bb.min.x - 1.0).abs() < 1e-9 && (bb.max.z - 5.0).abs() < 1e-9);
    assert_eq!(a.classify(&[[2.0, 3.0, 4.0], [9.0, 9.0, 9.0], [1.0, 3.0, 4.0]]).unwrap(), vec![Side::In, Side::Out, Side::On]);
    let hits = a.raycast(&[Ray::new([0.0, 3.0, 4.0], [1.0, 0.0, 0.0]).unwrap()]).unwrap();
    assert_eq!(hits[0].len(), 2);
    assert!(a.mesh(0.01).unwrap().triangles.len() >= 12);
    let tops = a.faces_where(|f| matches!(f.kind(), yack::SurfaceKind::Plane { normal } if normal.z > 0.5));
    assert_eq!(tops.len(), 1);
    assert_eq!(a.edges_of(tops[0]).unwrap().len(), 4);
}

#[test]
fn closest_pairs_meshes_and_vertices() {
    let (a, b) = (cube([0.0; 3]), cube([5.0, 0.0, 0.0]));
    let (d, pa, pb) = a.closest_pair(&b).unwrap();
    close(d, 3.0);
    close(pa.x, 2.0);
    close(pb.x, 5.0);
    let m = a.mesh(0.01).unwrap();
    assert!(m.is_watertight());
    close(m.volume(), 8.0);
    let corners: Vec<_> = a.vertices().into_iter().map(|v| a.vertex(v).unwrap().point()).collect();
    assert!(corners.iter().any(|p| p.x == 2.0 && p.y == 2.0 && p.z == 2.0));
    assert!(a.edges_where(|e| e.uses() == 2).len() == 12);
    let touching = a.union(&cube([2.0, 2.0, 0.0])).unwrap();
    assert_eq!(touching.edges_where(|e| e.uses() == 4).len(), 1);
}

#[test]
fn a_spline_loop_through_points_extrudes() {
    let l = Loop::through(&[[0.0, 0.0], [4.0, -1.0], [6.0, 2.0], [3.0, 5.0], [-1.0, 3.0]]).unwrap();
    let s = extrude(&Frame::WORLD, &Profile::new(l, vec![]).unwrap(), 2.0).unwrap();
    assert!(s.check().is_empty());
    assert!(s.volume() > 0.0);
}

#[test]
fn nearest_faces_edge_parameters_and_closest_with_insides() {
    let a = cube([0.0; 3]);
    let (f, d) = a.nearest_face([1.0, 1.0, 5.0]).unwrap();
    close(d, 3.0);
    assert!(a.face(f).unwrap().normal().unwrap().z > 0.99);
    // inside counts: a held point is its own nearest point
    let c = a.closest(&[[1.0, 1.0, 1.0], [1.0, 1.0, 5.0]]).unwrap();
    close(c[0].z, 1.0);
    close(c[1].z, 2.0);
    let top = a.face(f).unwrap();
    close(top.closest([1.0, 1.0, -7.0]).unwrap().z, 2.0);
    let e = a.edge(a.edge_at([1.0, 0.0, 2.0]).unwrap()).unwrap();
    let [t0, t1] = e.range();
    let p = e.point(t0);
    assert!((p.x - e.ends()[0].x).abs() < 1e-12 && t1 > t0);
}

#[test]
fn a_ball_grooves_a_plate_along_a_spline_through_cadrs() {
    let plate = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0]).unwrap();
    let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]]).unwrap();
    let tool = sphere(&Frame::WORLD, 3.0).unwrap().swept(&Motion::Along { path, law: FrameLaw::Fixed }).unwrap();
    assert_eq!(tool.strategy(), "cadrs");
    assert!(tool.check().is_empty());
    let cut = plate.difference(&tool).unwrap();
    assert!(cut.check().is_empty());
    assert!(cut.volume() < 42.0 * 32.0 * 8.0);
}

#[test]
fn a_slid_block_is_a_longer_block() {
    let s = cube([0.0; 3]).swept(&Motion::Slide { by: [3.0, 0.0, 0.0].into() }).unwrap();
    close(s.volume(), 8.0 + 2.0 * 2.0 * 3.0);
    assert_eq!(s.strategy(), "cadrs");
}

#[test]
fn the_same_program_twice_gives_the_same_digest() {
    let run = || {
        let plate = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0]).unwrap();
        let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]]).unwrap();
        let tool = sphere(&Frame::WORLD, 3.0).unwrap().swept(&Motion::Along { path, law: FrameLaw::Fixed }).unwrap();
        let cut = plate.difference(&tool).unwrap();
        let e = cut.edge_at([36.0, 26.0, -4.0]).unwrap();
        cut.fillet(&[e], 1.0).unwrap().named("done").digest()
    };
    assert_eq!(run(), run());
}

#[test]
fn cut_swept_is_the_difference_with_the_swept_tool() {
    let plate = block(&Frame::at([-6.0, -6.0, -8.0]), [42.0, 32.0, 8.0]).unwrap();
    let path = Spline3::bezier(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]]).unwrap();
    let motion = Motion::Along { path, law: FrameLaw::Fixed };
    let ball = sphere(&Frame::WORLD, 3.0).unwrap();
    let direct = plate.cut_swept(&ball, &motion).unwrap();
    let two_step = plate.difference(&ball.swept(&motion).unwrap()).unwrap();
    assert!(direct.check().is_empty());
    assert!((direct.volume() - two_step.volume()).abs() < 1e-3 * two_step.volume());
}
