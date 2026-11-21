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
    assert_eq!(yack::bakeoff::built_by(&a.union(&b).unwrap()), "cadrs");
    assert_eq!(yack::bakeoff::built_by(&a.difference(&b).unwrap()), "cadrs");
    force(Engine::Springrep);
    assert_eq!(a.swept(&Motion::slide([1.0, 0.0, 0.0])).unwrap_err().kind, ErrorKind::Unsupported);
    assert_eq!(yack::bakeoff::built_by(&a.union(&b).unwrap()), "springrep");
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
                Ok(s) => assert_eq!(yack::bakeoff::built_by(&s), name),
                Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported, "{e}"),
            }
        }
    }
    force(Engine::Both);
}

#[test]
fn a_block_builds_on_either_forced_engine() {
    for (engine, name) in [(Engine::Cadrs, "cadrs"), (Engine::Springrep, "springrep")] {
        force(engine);
        let b = block(&Frame::at([1.0, 2.0, 3.0]), [2.0, 3.0, 4.0]).unwrap();
        assert_eq!(yack::bakeoff::built_by(&b), name);
        // a cadrs block crosses the bridge: springrep's carried form answers its queries
        assert_eq!(b.strategy(), "springrep");
        assert!((b.volume() - 24.0).abs() < 1e-9);
    }
    force(Engine::Both);
}

/// conformance closing-offsets-f9a2a750057b7630 (modify/push-faces-until-they-merge): an L-block's
/// low top pushed up to the high one. Why the bridge holds the cadrs result (if it does), printed.
#[test]
fn why_a_merged_l_block_is_held() {
    let (w, d, h, step, lo) = (8.270117232320677, 4.286181010494695, 5.3617773736568015, 3.6147345061240137, 3.9611320861080412);
    let l = Profile::polygon(&[[0.0, 0.0], [w, 0.0], [w, lo], [step, lo], [step, h], [0.0, h]]).unwrap();
    let f = Frame::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]).unwrap();
    let b = extrude(&f, &l, d).unwrap();
    let top = b.face_at([5.942425869222346, -2.1430905052473475, lo]).unwrap();
    let s = b.offset_faces(&[top], 1.4006452875487603).unwrap();
    println!("WHY strategy {} roundtrip {:?}", s.strategy(), yack::bakeoff::roundtrip(&s));
    let text = yack::step::write(&s);
    println!("WHY write {:?}", text.as_ref().map(|t| t.len()));
    if let Ok(t) = text {
        println!("WHY read {:?}", yack::step::read(&t).map(|r| r.volume()));
    }
}
