// yack: T1 checks of STEP and codec exchange through the public API (M1.5).
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use yack::prelude::*;

fn close(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-7 * want.abs().max(1.0), "{got} vs {want}");
}

fn part() -> Solid {
    let b = block(&Frame::WORLD, [10.0, 20.0, 30.0]).unwrap();
    b.difference(&cylinder(&Frame::at([5.0, 10.0, -1.0]), 2.0, 32.0).unwrap()).unwrap()
}

#[test]
fn step_round_trips_a_drilled_block() {
    let s = part();
    let text = yack::step::write(&s).unwrap();
    assert!(text.starts_with("ISO-10303-21;"));
    let back = yack::step::read(&text).unwrap();
    assert!(back.check().is_empty());
    close(back.volume(), s.volume());
    close(back.area(), s.area());
    assert_eq!(back.betti(), [1, 1, 0]);
    let sewn = yack::step::sew(&text, 1e-3).unwrap();
    close(sewn.volume(), s.volume());
}

#[test]
fn step_writes_a_ring_pushed_in_at_a_lean_faithfully() {
    let (sn, cs) = (35f64.to_radians().sin(), 35f64.to_radians().cos());
    let ring = torus(&Frame::new([0.0, 0.0, 4.0], [cs, 0.0, -sn], [0.0, 1.0, 0.0], [sn, 0.0, cs]).unwrap(), 3.0, 1.0).unwrap();
    let hole = ring.swept(&Motion::slide([0.0, 0.0, -8.0])).unwrap();
    let s = block(&Frame::at([-8.0, -8.0, -8.0]), [16.0, 16.0, 8.0]).unwrap().difference(&hole).unwrap();
    let back = yack::step::read(&yack::step::write(&s).unwrap()).unwrap();
    // the reader heals on the way back in: its noise is about 1e-6, the write check's bound 1e-5
    assert!((back.volume() - s.volume()).abs() <= 1e-5 * s.volume(), "{} vs {}", back.volume(), s.volume());
    assert!((back.area() - s.area()).abs() <= 1e-5 * s.area(), "{} vs {}", back.area(), s.area());
}

#[test]
fn bad_step_is_refused_not_a_crash() {
    for text in ["", "ISO-10303-21;", "garbage #1=FOO(;", "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"] {
        let e = yack::step::read(text).unwrap_err();
        assert_ne!(e.kind, ErrorKind::Internal, "{e}");
    }
    assert_eq!(yack::step::sew("", -1.0).unwrap_err().arg, Some("tolerance"));
    assert!(yack::step::write(&empty()).is_err());
}

#[test]
fn codec_round_trips_exactly() {
    let s = part();
    let bytes = yack::codec::encode(&s);
    let back = yack::codec::decode(&bytes).unwrap();
    assert_eq!(back.digest(), s.digest());
    close(back.volume(), s.volume());
    assert!(yack::codec::decode(&bytes[..bytes.len() / 2]).is_err());
    assert!(yack::codec::decode(b"not a stream").is_err());
    assert!(yack::codec::decode(&yack::codec::encode(&empty())).unwrap().is_empty());
}
