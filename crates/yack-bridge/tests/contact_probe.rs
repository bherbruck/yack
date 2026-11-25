// yack: probe for conformance sphere-in-a-torus-hole (B26): is the union two pieces with a real gap, so
// that a ray crossing both really crosses four times? (printed to the scratchpad)
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_boolean as boolean;
use yack_locate::distance;
use yack_math::kernel::{Frame, Vec3};
use yack_measure as measure;
use yack_build::primitive;

fn frame(o: [f64; 3], z: [f64; 3], x: [f64; 3]) -> Frame {
    Frame::new(Vec3::new(o[0], o[1], o[2]), Vec3::new(z[0], z[1], z[2]), Vec3::new(x[0], x[1], x[2])).unwrap()
}

#[test]
fn sphere_in_a_torus_hole_is_two_pieces() {
    let x = [-0.7148566939609067, -0.3270574454210159, 0.6180722728726482];
    let z = [0.06459752030432298, 0.8492149734521551, 0.5240811857290699];
    let t = primitive::torus(&frame([-2.4536287888669417, -0.4188831209451287, -2.3224440845692773], z, x), 1.6819245919605297, 0.5900110918968451, "t").unwrap();
    let s = primitive::sphere(&frame([-2.3263451224619023, 1.254419428097376, -1.2897887365120186], z, x), 2.000610796586716, "s").unwrap();
    let u = boolean::union(&t, &s).unwrap();
    let c = yack_mesh::closest(&t, &distance::Selection::body(&t), &s, &distance::Selection::body(&s)).unwrap();
    let (a, b) = (c.on_a, c.on_b);
    let mid = (a + b) * 0.5;
    let dir = (b - a).unit().unwrap();
    let rays = yack_mesh::rays(&u).unwrap();
    let lines = [
        format!("pieces in the union: {}", u.solid_ids().count()),
        format!("gap {:e} between {a:?} and {b:?}", c.distance),
        format!("gap middle: {:?}", rays.side(mid)),
        format!("1e-6 into the torus: {:?}", rays.side(a - dir * 1e-6)),
        format!("1e-6 into the sphere: {:?}", rays.side(b + dir * 1e-6)),
        format!("union volume {} = torus {} + sphere {}", measure::volume(&u), measure::volume(&t), measure::volume(&s)),
    ];
    std::fs::write("/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/contact-probe.txt", lines.join("\n")).unwrap();
}
