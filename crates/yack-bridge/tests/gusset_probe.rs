// yack: probe for conformance part-brackets-gusset-bf80d64fac83e0fb (B14: volume 0.689 mm³ over the
// exact 5548.3915786): each op of the case done in springrep, its volume after every step.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing, clippy::print_stderr)]
use yack_springrep::{
    boolean, measure,
    math::{Frame, Vec2, Vec3},
    primitive,
    profile::{NamedSegment, Profile, ProfileLoop, Segment},
    sweep,
};

fn poly(pts: &[[f64; 2]]) -> Profile {
    let n = pts.len();
    let segments = (0..n)
        .map(|i| {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            NamedSegment::new(Segment::Line { a: Vec2::new(a[0], a[1]), b: Vec2::new(b[0], b[1]) }, &format!("s{i}"))
        })
        .collect();
    Profile { loops: vec![ProfileLoop { segments }] }
}

#[test]
#[ignore = "evidence: conformance part-brackets-gusset-bf80d64fac83e0fb expects the full-hole volume; the hole breaks out of the plate (swarm/done/w-m11-conformance-reference-issues.md)"]
fn gusset_volumes_step_by_step() {
    let (a, t, h, d) = (30.819915722559763, 3.876718418902218, 27.708957794316767, 25.236343868878777);
    let g = 18.6537053899006;
    let r = 4.652062102682661;
    let down = |y: f64| Frame::new(Vec3::new(0.0, y, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let l = sweep::extrude(&poly(&[[0.0, 0.0], [a, 0.0], [a, t], [t, t], [t, h], [0.0, h]]), &down(d), d, "l").unwrap();
    let tri = poly(&[[t, t], [g, t], [t, g]]);
    let g1 = sweep::extrude(&tri, &down(7.1225786390179255), 3.337127058686109, "g1").unwrap();
    let g2 = sweep::extrude(&tri, &down(21.45089228854696), 3.337127058686109, "g2").unwrap();
    let c1 = primitive::cylinder(&Frame::new(Vec3::new(24.736810556230182, 12.618171934439388, -1.0), Vec3::Z, Vec3::X).unwrap(), r, 5.876718418902218, "c1").unwrap();
    let c2 = primitive::cylinder(&Frame::new(Vec3::new(-1.0, 12.618171934439388, 23.181331592108684), Vec3::X, Vec3::Y).unwrap(), r, 5.876718418902218, "c2").unwrap();
    let v = measure::volume;
    let exact_l = (a * t + t * (h - t)) * d;
    let exact_g = 0.5 * (g - t) * (g - t) * 3.337127058686109;
    let exact_hole = std::f64::consts::PI * r * r * t;
    eprintln!("L {} (exact {exact_l}); g1 {} g2 {} (exact {exact_g})", v(&l), v(&g1), v(&g2));
    let gg = boolean::union(&g1, &g2).unwrap();
    eprintln!("g1 ∪ g2 {} (exact {})", v(&gg), 2.0 * exact_g);
    let u = boolean::union(&l, &gg).unwrap();
    eprintln!("1 union {} (exact {})", v(&u), exact_l + 2.0 * exact_g);
    let d1 = boolean::difference(&u, &c1).unwrap();
    eprintln!("2 difference {} (exact {})", v(&d1), exact_l + 2.0 * exact_g - exact_hole);
    let d2 = boolean::difference(&d1, &c2).unwrap();
    eprintln!("3 difference {} (exact {})", v(&d2), exact_l + 2.0 * exact_g - 2.0 * exact_hole);
}
