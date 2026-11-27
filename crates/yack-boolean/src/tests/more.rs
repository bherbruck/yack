// yack: springrep boolean unit tests, continued (moved out of tests.rs for the file-size cap).
// Adapted from springrep@a03e8a1 src/boolean/tests.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved verbatim from the end of boolean/tests.rs into this child module; then to yack-boolean (M3 W8), paths and visibility only.
use super::*;
use crate::bounds::face_bbox;

/// Solids whose faces bulge past their own edges: a ball and a ring turned off the axes, a spline
/// profile turned part way round an axis, and a ball stretched into a spline surface.
fn bulging() -> Vec<Brep> {
    use crate::{primitive, profile::PointsInterp};
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
    vec![
        primitive::torus(&tilted, 3.0, 1.0, "t").unwrap(),
        revolve(&wavy, &plane, tilted.origin, tilted.z, 2.2, "r").unwrap(),
        // affine-ball fixture: edit::affine's stretch of the ball above, proven by w-h4-9.
        fixture("affine-ball"),
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

// A rod lying along a box's face touches it along a line, from the box's side in: the result keeps
// the line as a slit inside the face, a loop that runs along an edge and back and encloses nothing.
// Whether that face lies along its surface's normal or against it, the slit must not be judged by
// the way it runs. The solids touch only along the line, so the union is both volumes.
#[test]
fn a_rod_lying_along_either_face_of_a_box_unites_with_it() {
    let base = block([0.0; 3], [20.0, 10.0, 10.0]);
    // Radius 2 along x from the box's side x = 0, on its top (axis at z = 12) or under its
    // bottom (axis at z = -2).
    for z in [12.0, -2.0] {
        let bar = rod([0.0, 5.0, z], [1.0, 0.0, 0.0], 2.0, 10.0);
        let u = union(&base, &bar).unwrap();
        let issues = crate::check::check(&u);
        assert!(issues.is_empty(), "z = {z}: {issues:?}");
        let want = 2000.0 + PI * 4.0 * 10.0;
        assert!((measure::volume(&u) - want).abs() < 1e-6, "z = {z}");
    }
}
