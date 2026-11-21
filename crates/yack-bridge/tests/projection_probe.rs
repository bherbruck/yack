// yack: probe for conformance B38 projections-ae6d1b45b7175f55 (hidden lines of a sphere): springrep's
// drawing of the sphere and of its union with itself, each vertex's distance from the exact outline.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing, clippy::print_stderr)]
use yack_springrep::{
    boolean, draw,
    math::{Frame, Vec3},
    primitive,
};

#[test]
#[ignore = "probe: conformance projections-ae6d1b45b7175f55 (B38, hidden lines of a sphere), under diagnosis by w-m11"]
fn sphere_outline_vertices() {
    let c = Vec3::new(-4.785947441078186, -2.6058400579158914, 0.3014026637442493);
    let r = 0.7969709521203601;
    let d = Vec3::new(0.2735124731022186, 0.046175321589579224, 0.9607594739234202);
    let s = primitive::sphere(&Frame::new(c, Vec3::Z, Vec3::X).unwrap(), r, "s").unwrap();
    let u = boolean::union(&s, &s).unwrap();
    let du = d.unit().unwrap();
    // Off the outline: how far from the circle of radius r about c square to d.
    let off = |p: Vec3| {
        let q = p - c;
        let along = q.dot(du);
        let radial = (q - du * along).norm();
        (along * along + (radial - r) * (radial - r)).sqrt()
    };
    for (name, b) in [("sphere", &s), ("sphere ∪ sphere", &u)] {
        eprintln!("{name}: {} faces, {} edges", b.face_ids().count(), b.edge_ids().count());
        for tol in [0.01, 0.001] {
            let dr = draw::hidden_lines(b, d, tol).unwrap();
            for (kind, lines) in [("visible", &dr.visible), ("hidden", &dr.hidden)] {
                for l in lines.iter() {
                    let len: f64 = l.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
                    let worst = l.iter().map(|&p| off(p)).fold(0.0, f64::max);
                    eprintln!("  tol {tol} {kind}: {} points, length {len:.6} (2πr {:.6}), worst vertex off {worst:.3e}", l.len(), std::f64::consts::TAU * r);
                }
            }
        }
    }
}
