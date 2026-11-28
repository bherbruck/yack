// yack: a paraboloid cut a hair above its tip: the loop round the vertex is found however small
//! The solid paraboloid x² + y² <= 8z, z <= 8 (the revolved conic) cut by a block whose top is the plane z = h:
//! the tip, of volume 4π h² (the integral of 8π z dz from 0 to h). The profile's parameter runs from the
//! rim to the vertex, so the vertex is a pole of the surface at its far end: a point of the surface nearer
//! the pole than the seed beside it was once put on the pole itself, a hit of the ray cast on the tip lost
//! to the face, and the solid classified empty.
#![allow(clippy::unwrap_used)] // a test file: a failed setup is a failed test

use std::f64::consts::TAU;

use yack_math::kernel::{Frame, Point3, Vec2, Vec3};
use yack_profile::{NamedSegment, Profile, ProfileLoop, Segment};

fn paraboloid() -> yack_topo::Brep {
    let o = Point3::ZERO;
    let f = Frame::from_axis(o, Vec3::Z).unwrap();
    let named = |s: Segment, n: &str| NamedSegment::new(s, n);
    let conic = Segment::Bezier { ctrl: vec![Vec2::new(0.0, 0.0), Vec2::new(4.0, 0.0), Vec2::new(8.0, 8.0)], weights: vec![1.0, 1.0, 1.0] };
    let meridian = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                named(conic, "bowl"),
                named(Segment::Line { a: Vec2::new(8.0, 8.0), b: Vec2::new(0.0, 8.0) }, "top"),
                named(Segment::Line { a: Vec2::new(0.0, 8.0), b: Vec2::new(0.0, 0.0) }, "axis"),
            ],
        }],
    };
    yack_build::sweep::revolve(&meridian, &Frame::new(o, -f.y, f.x).unwrap(), o, f.z, TAU, "p").unwrap()
}

#[test]
fn the_tip_of_a_paraboloid_cut_a_hair_above_its_vertex_is_found() {
    let para = paraboloid();
    for h in [1.0_f64, 1e-1, 1e-2, 1e-3, 1e-4] {
        let block = yack_build::primitive::block(&Frame::WORLD.translated(Vec3::new(-9.0, -9.0, -1.0)), Vec3::new(18.0, 18.0, 1.0 + h), "b").unwrap();
        let tip = yack_boolean::intersection(&para, &block).unwrap();
        let want = 4.0 * std::f64::consts::PI * h * h;
        let got = yack_measure::volume(&tip);
        assert!((got - want).abs() <= 1e-6 * want, "h = {h}: {got} against {want}");
    }
}

#[test]
fn a_point_of_the_bowl_beside_the_vertex_projects_to_itself() {
    let para = paraboloid();
    let bowl = para.face_ids().find(|&f| matches!(para.face(f).surface, yack_geom::Surface::Revolution(_))).unwrap();
    let surface = para.face(bowl).surface.clone();
    // The profile runs from the rim (v = 0) to the vertex (v = 1): the points a few hundredths of the way
    // from the vertex are nearer the vertex than to the seed beside it.
    for k in 1..=20 {
        let v = 1.0 - f64::from(k) * 0.004;
        let q = surface.eval(0.7, v);
        let uv = surface.project(q);
        assert!(surface.eval(uv.x, uv.y).dist(q) <= 1e-9, "v = {v}");
    }
}
