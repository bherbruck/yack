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
