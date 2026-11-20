// yack: the bake-off switch forces one engine for whole ops, internal threads included (M1.10).
#![cfg(feature = "bakeoff")]
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use yack::bakeoff::{Engine, force};
use yack::prelude::*;

#[test]
fn a_forced_engine_holds_through_an_op_that_fans_out() {
    force(Engine::Both);
    let a = block(&Frame::WORLD, [4.0, 4.0, 4.0]).unwrap();
    let b = sphere(&Frame::at([4.0, 4.0, 4.0]), 2.5).unwrap();
    force(Engine::Cadrs);
    assert_eq!(a.union(&b).unwrap().strategy(), "cadrs");
    assert_eq!(a.difference(&b).unwrap().strategy(), "cadrs");
    force(Engine::Springrep);
    assert_eq!(a.swept(&Motion::Slide { by: [1.0, 0.0, 0.0].into() }).unwrap_err().kind, ErrorKind::Unsupported);
    assert_eq!(a.union(&b).unwrap().strategy(), "springrep");
    force(Engine::Both);
}

#[test]
fn under_a_forced_engine_every_constructor_builds_on_it_or_refuses() {
    for (engine, name) in [(Engine::Cadrs, "cadrs"), (Engine::Springrep, "springrep")] {
        force(engine);
        let made = [
            block(&Frame::WORLD, [1.0, 2.0, 3.0]),
            cylinder(&Frame::WORLD, 1.0, 2.0),
            sphere(&Frame::WORLD, 1.0),
            torus(&Frame::WORLD, 3.0, 1.0),
            cone(&Frame::WORLD, 2.0, 1.0, 3.0),
            extrude(&Frame::WORLD, &Profile::rectangle([0.0, 0.0], [1.0, 1.0]).unwrap(), 1.0),
        ];
        for m in made {
            match m {
                Ok(s) => assert_eq!(s.strategy(), name),
                Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported, "{e}"),
            }
        }
    }
    force(Engine::Both);
}
