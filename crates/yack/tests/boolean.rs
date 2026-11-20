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
