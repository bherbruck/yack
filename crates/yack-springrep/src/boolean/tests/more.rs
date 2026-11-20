// yack: springrep boolean unit tests, continued (moved out of tests.rs for the file-size cap).
// Adapted from springrep@a03e8a1 src/boolean/tests.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved verbatim from the end of boolean/tests.rs into this child module.
use super::*;

/// Solids whose faces bulge past their own edges: a ball and a ring turned off the axes, a spline
/// profile turned part way round an axis, and a ball stretched into a spline surface.
fn bulging() -> Vec<Brep> {
    use crate::{
        edit::{Affine, affine},
        primitive,
        profile::PointsInterp,
    };
    let tilted = Frame::new(
        Vec3::new(1.0, -2.0, 0.5),
        Vec3::new(0.3, -0.5, 0.8),
        Vec3::new(1.0, 0.2, 0.0),
    )
    .unwrap();
    let outline = [
        Vec2::new(2.0, 0.0),
        Vec2::new(4.5, 1.0),
        Vec2::new(3.0, 2.5),
        Vec2::new(5.0, 4.0),
        Vec2::new(2.5, 5.0),
        Vec2::new(1.5, 2.5),
    ];
    let wavy = Profile::points(&outline, PointsInterp::PeriodicSpline, "w").unwrap();
    let plane = Frame::new(tilted.origin, -tilted.y, tilted.x).unwrap();
    let ball = primitive::sphere(&tilted, 2.0, "s").unwrap();
    let map = Affine {
        m: [[1.1, -0.9, 0.5], [-1.0, 0.6, 0.5], [1.3, 0.6, 0.4]],
        t: Vec3::new(2.0, -3.0, 1.0),
    };
    vec![
        primitive::torus(&tilted, 3.0, 1.0, "t").unwrap(),
        revolve(&wavy, &plane, tilted.origin, tilted.z, 2.2, "r").unwrap(),
        affine(&ball, &map).unwrap(),
        ball,
    ]
}

#[test]
fn a_faces_box_holds_every_point_of_the_face() {
    let fine = crate::mesh::MeshOptions {
        linear: 0.002,
        angular: 0.05,
    };
    for b in bulging() {
        let m = crate::mesh::tessellate(&b, &fine).unwrap();
        for (f, range) in &m.faces {
            let dom = face_domain(&b, *f);
            let bb = face_bbox(&dom);
            for t in &m.triangles[range.clone()] {
                for &i in t {
                    let q = m.positions[i as usize];
                    let out = (0..3)
                        .map(|k| (bb.min[k] - q[k]).max(q[k] - bb.max[k]))
                        .fold(0.0, f64::max);
                    assert!(
                        out <= 1e-9,
                        "{} face: {q:?} is {out} outside {bb:?}",
                        dom.surface.kind()
                    );
                }
            }
        }
    }
}
