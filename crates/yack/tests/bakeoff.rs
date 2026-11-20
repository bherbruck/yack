// yack: the bake-off switch forces one engine for whole ops, internal threads included (M1.10).
#![cfg(feature = "bakeoff")]
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use yack::bakeoff::{Engine, force};
use yack::prelude::*;

#[test]
fn a_forced_engine_holds_through_an_op_that_fans_out() {
    force(Engine::Cadrs);
    let a = block(&Frame::WORLD, [4.0, 4.0, 4.0]).unwrap();
    let b = sphere(&Frame::at([4.0, 4.0, 4.0]), 2.5).unwrap();
    let u = a.union(&b).unwrap();
    assert_eq!(u.strategy(), "cadrs");
    let d = a.difference(&b).unwrap();
    assert_eq!(d.strategy(), "cadrs");
    force(Engine::Springrep);
    let e = a.swept(&Motion::Slide { by: [1.0, 0.0, 0.0].into() }).unwrap_err();
    assert_eq!(e.kind, ErrorKind::Unsupported);
    assert_eq!(a.union(&b).unwrap().strategy(), "springrep");
    force(Engine::Both);
}
