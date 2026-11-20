// yack: a hole drilled anywhere round a cylinder face whose seam jogs survives STEP and meshes.
// Adapted from springrep@a03e8a1 tests/seams.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); frames converted from springrep's
// (origin, z, x) to yack's (origin, x, y, z) with y = z × x; `MeshOptions { linear: 0.01, .. }` is chord
// 0.01; springrep's mesh report is `Mesh::is_watertight` and `Mesh::volume`.
//! Faces whose seam does not run straight: two lengths of one cylinder joined, each with its seam
//! at its own angle, are one face round the cylinder whose seam jogs where they meet. Its outline
//! in the parameter plane spans more than one period, and a hole in it lies in one of them.

use std::f64::consts::PI;

use yack::prelude::*;

/// A rod of radius 5 from z = 0 to 30: 10 mm with its seam at +x under 20 mm with its seam at −x.
fn rod() -> Solid {
    let low = cylinder(&Frame::WORLD, 5.0, 10.0).unwrap();
    // springrep Frame::new((0, 0, 10), z = Z, x = −X): y = Z × −X = −Y.
    let frame = Frame::new([0.0, 0.0, 10.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]).unwrap();
    let high = cylinder(&frame, 5.0, 20.0).unwrap();
    low.union(&high).unwrap()
}

/// A blind hole of radius 0.5, 2 mm deep, drilled radially at height 20 and `angle` round from +x.
fn drill(angle: f64) -> Solid {
    let out = Vec3::new(angle.cos(), angle.sin(), 0.0);
    // springrep Frame::new(origin, z = out, x = Z): y = out × Z = (sin, −cos, 0).
    let origin = Point3::new(0.0, 0.0, 20.0) + out * 3.0;
    let frame = Frame::new(origin, [0.0, 0.0, 1.0], [angle.sin(), -angle.cos(), 0.0], out).unwrap();
    cylinder(&frame, 0.5, 4.0).unwrap()
}

#[test]
fn a_hole_in_a_face_whose_seam_jogs_meshes_wherever_round_it_is() {
    let rod = rod();
    assert!(rod.check().is_empty());
    // The two sides are one face: with the two ends, three.
    assert_eq!(rod.faces().len(), 3);
    let chord = 0.01;
    // Either side of each seam: the upper length's outline lies a period on from the lower's
    // for half the way round, so a hole there is not where its own angle alone would put it.
    for angle in [0.2, -0.2, PI - 0.2, PI + 0.2] {
        let cut = rod.difference(&drill(angle)).unwrap();
        assert!(cut.check().is_empty(), "{angle}");
        // As a file gives it, with each edge's place on the face found on its own.
        let text = yack::step::write(&cut).unwrap();
        let drilled = yack::step::read(&text).unwrap();
        let m = drilled.mesh(chord).unwrap_or_else(|e| panic!("{angle}: {e:?}"));
        assert!(m.is_watertight(), "{angle}");
        // Chords 0.01 mm deep leave out under 2 · 0.01 / 5 of the rod's π · 5² · 30.
        let exact = drilled.volume();
        assert!((exact - PI * 750.0).abs() < 2.0, "{angle}: {exact}");
        let volume = m.volume();
        assert!((exact * (1.0 - 0.004)..exact).contains(&volume), "{angle}: {volume}");
    }
}
