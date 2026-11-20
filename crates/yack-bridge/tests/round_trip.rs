// yack: round trips springrep → cadrs → springrep on one solid of each surface and curve kind (MAPPING.md §5).
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_bridge::{to_cadrs, to_springrep};
use yack_cadrs::native::{Body, P3, classify, mass};
use yack_springrep::{
    boolean, check,
    math::{Frame, Vec3},
    measure, primitive,
    rays::{Rays, Side},
    topo::Brep,
};

fn cadrs_area(b: &Body) -> f64 {
    (0..b.faces().len()).map(|f| mass::face_area(b, f).0).sum()
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

/// Sample points on a 7×7×7 grid over the box, nudged off symmetry planes.
fn samples(s: &Brep) -> Vec<Vec3> {
    let bb = measure::bbox(s);
    let (lo, hi) = (bb.min - Vec3::new(1.0, 1.0, 1.0), bb.max + Vec3::new(1.0, 1.0, 1.0));
    let mut out = Vec::new();
    for i in 0..7 {
        for j in 0..7 {
            for k in 0..7 {
                let f = |n: usize, a: f64, b: f64| a + (b - a) * (n as f64 + 0.37) / 7.0;
                out.push(Vec3::new(f(i, lo.x, hi.x), f(j, lo.y, hi.y), f(k, lo.z, hi.z)));
            }
        }
    }
    out
}

fn round_trip(name: &str, s: &Brep) {
    let (body, names) = to_cadrs(s).unwrap_or_else(|e| panic!("{name}: to cadrs: {e}"));
    assert!(body.check_with(yack_cadrs::native::CheckLevel::Full).is_empty(), "{name}");
    let (v, a) = (measure::volume(s), measure::area(s));
    assert!(rel(mass::body_volume(&body), v) < 1e-9, "{name}: cadrs volume {} vs {v}", mass::body_volume(&body));
    assert!(rel(cadrs_area(&body), a) < 1e-9, "{name}: cadrs area {} vs {a}", cadrs_area(&body));
    let back = to_springrep(&body, &names).unwrap_or_else(|e| panic!("{name}: back: {e}"));
    assert!(check::check(&back).is_empty(), "{name}: {:?}", check::check(&back));
    assert!(rel(measure::volume(&back), v) < 1e-9, "{name}: back volume");
    assert!(rel(measure::area(&back), a) < 1e-9, "{name}: back area");
    let (rs, rb) = (Rays::new(s).unwrap(), Rays::new(&back).unwrap());
    let polys: Vec<_> = body.faces().iter().map(|f| classify::face_polygons(&body, f)).collect();
    for p in samples(s) {
        let want = rs.side(p);
        assert_eq!(rb.side(p), want, "{name}: springrep side at {p:?}");
        let c = classify::classify_point(&body, &polys, &P3::new(p.x, p.y, p.z), 1e-7);
        let same = matches!(
            (want, c),
            (Side::Inside, yack_cadrs::PointClass::Inside)
                | (Side::Outside, yack_cadrs::PointClass::Outside)
                | (Side::On, yack_cadrs::PointClass::OnBoundary)
        );
        assert!(same, "{name}: cadrs class {c:?} vs springrep {want:?} at {p:?}");
    }
    // And the second leg: the body back across equals the first.
    let (again, _) = to_cadrs(&back).unwrap();
    assert!(rel(mass::body_volume(&again), v) < 1e-9, "{name}: C→S→C volume");
}

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame::new(Vec3::new(x, y, z), Vec3::Z, Vec3::X).unwrap()
}

#[test]
fn primitives() {
    round_trip("block", &primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 20.0, 30.0), "b").unwrap());
    round_trip("cylinder", &primitive::cylinder(&at(1.0, 2.0, 3.0), 4.0, 9.0, "c").unwrap());
    round_trip("sphere", &primitive::sphere(&at(0.0, 0.0, 0.0), 5.0, "s").unwrap());
    round_trip("torus", &primitive::torus(&at(0.0, 0.0, 0.0), 6.0, 2.0, "t").unwrap());
    round_trip("cone", &primitive::cone(&at(0.0, 0.0, 0.0), 4.0, 1.0, 6.0, "k").unwrap());
    round_trip("apex", &primitive::cone(&at(0.0, 0.0, 0.0), 4.0, 0.0, 6.0, "k").unwrap());
}

#[test]
fn booleans() {
    let block = primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
    let rod = primitive::cylinder(&at(5.0, 5.0, -1.0), 2.0, 12.0, "c").unwrap();
    round_trip("drilled", &boolean::difference(&block, &rod).unwrap());
    let side = primitive::cylinder(&Frame::new(Vec3::new(-1.0, 5.0, 5.0), Vec3::X, Vec3::Y).unwrap(), 3.0, 12.0, "x").unwrap();
    round_trip("cross-drilled", &boolean::difference(&boolean::difference(&block, &rod).unwrap(), &side).unwrap());
    let ball = primitive::sphere(&at(10.0, 10.0, 10.0), 4.0, "s").unwrap();
    round_trip("corner ball", &boolean::union(&block, &ball).unwrap());
}
