// yack: probe for conformance chamfers/rejects-two-distance-cut-deeper-than-a-face: what springrep and
// cadrs each answer for a two-distance chamfer d1 past the box's smallest side but inside the face.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_math::kernel::{Frame, Vec3};

#[test]
fn chamfer_two_deeper_than_the_smallest_side() {
    let w = [3.899505853561688, 5.763529773814428, 4.134682994075912];
    let b = yack_build::primitive::block(&Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap(), Vec3::new(w[0], w[1], w[2]), "b").unwrap();
    let edge = yack_measure::edge_at(&b, Vec3::new(w[0], w[1], w[2] / 2.0)).unwrap().0;
    let face = yack_measure::face_at(&b, Vec3::new(w[0], w[1] / 2.0, w[2] / 2.0)).unwrap().0;
    let mut out = Vec::new();
    for d1 in [4.0, 4.5, 5.0, 5.5, 6.0, 7.0] {
        let d2 = w[0] * 0.3;
        let r = yack_blend::chamfer_two_edges(&b, edge, face, d1, d2, "c");
        out.push(format!("FAILED-PROBE springrep d1 {d1}: {}", match &r { Ok(s) => format!("ok volume {}", yack_measure::volume(s)), Err(e) => format!("{e}") }));
    }
    std::fs::write("/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/chamfer-probe.txt", out.join("\n")).unwrap();
}
