// yack: a cadrs answer the bridge cannot carry, held in cadrs form (the lazy bridge).
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use yack::prelude::*;

/// swept/tumbling-torus: cadrs sweeps it exactly; its swept face folds in (u, v), which springrep
/// cannot carry. Held in cadrs form, it is measured, listed and written by cadrs, and a springrep
/// query on it refuses (Unsupported), never answers empty.
fn tumbling_torus() -> Solid {
    let ring = torus(&Frame::at([3.0, 0.0, 2.0]), 1.0, 0.5).unwrap();
    let roll = Motion::turn(Point3::new(2.0, 0.0, 0.0), Dir3::Z, deg(-18.75), deg(18.75));
    let tumble = Motion::turn(Point3::new(0.0, 0.5, 0.5), Dir3::X, deg(75.0), deg(-75.0));
    ring.swept(&Motion::together(vec![roll, tumble])).unwrap()
}

#[test]
fn a_held_solid_is_measured_listed_and_refuses_springrep_queries() {
    let s = tumbling_torus();
    assert_eq!(s.strategy(), "cadrs");
    assert!(s.volume() > 0.0 && s.area() > 0.0, "{} {}", s.volume(), s.area());
    assert!(!s.faces().is_empty() && !s.edges().is_empty());
    assert!(s.check().is_empty(), "{:?}", s.check());
    // one piece; the tumble sweeps the ring through its own hole, closing it; no void
    assert_eq!(s.betti(), [1, 0, 0]);
    let e = s.face_at([3.0, 0.0, 2.0]).unwrap_err();
    assert_eq!(e.kind, ErrorKind::Unsupported, "{e}");
    let e = s.classify(&[[3.0, 0.0, 2.0]]).unwrap_err();
    assert_eq!(e.kind, ErrorKind::Unsupported, "{e}");
}

#[test]
fn a_held_solid_digests_alike_and_writes_its_step() {
    let (a, b) = (tumbling_torus(), tumbling_torus());
    assert_eq!(a.digest(), b.digest());
    assert_ne!(a.digest(), [0; 32]);
    let text = yack::step::write(&a).unwrap();
    assert!(text.starts_with("ISO-10303-21;"));
}

#[test]
fn a_held_solid_takes_a_cadrs_boolean() {
    let s = tumbling_torus();
    let cut = s.difference(&block(&Frame::at([3.0, -10.0, -10.0]), [20.0, 20.0, 30.0]).unwrap()).unwrap();
    assert!(cut.volume() > 0.0 && cut.volume() < s.volume(), "{} of {}", cut.volume(), s.volume());
}

/// tangent/paraboloid-tip-a-ten-thousandth-high (case header: volume 1.2566370614359173e-7): the
/// slab's sliver of a paraboloid's tip. springrep's intersection says empty (tentative), the
/// bridge refuses cadrs's sliver (held): the held sliver is the answer.
#[test]
fn a_paraboloid_tip_is_not_lost() {
    let side = Loop::starting_at([0.0, 0.0]).conic_to([8.0, 8.0], [4.0, 0.0], 1.0).line_to([0.0, 8.0]).close().unwrap();
    let paraboloid = revolve(&Frame::WORLD, &Profile::new(side, vec![]).unwrap(), deg(360.0)).unwrap();
    let tip = paraboloid.intersection(&block(&Frame::at([-9.0, -9.0, -1.0]), [18.0, 18.0, 1.0001]).unwrap()).unwrap();
    assert!(!tip.is_empty());
    let want = 1.2566370614359173e-7;
    assert!((tip.volume() - want).abs() < 1e-5 * want, "{} ({})", tip.volume(), tip.strategy());
}
