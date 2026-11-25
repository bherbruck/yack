// yack: M3 W8's one-off fixture printer: the solids boolean's tests build with blend or edit (above yack-boolean).
//! A third, the ball edit::affine stretches (above yack-boolean too), for tests/more.rs's bulging solids.
//! Prints each solid's codec bytes as `FIXTUREHEX tag hex`, after proving them: the bytes decode to
//! a solid equal to a fresh build, re-encode to the same bytes, and a second build gives the same
//! bytes. Run with `cargo test -p yack-springrep --test w8_fixtures_print -- --ignored --nocapture`;
//! deleted once W8's apply has written the fixtures.

use yack_springrep::{
    blend::{chamfer, fillet},
    codec,
    edit::{Affine, affine},
    primitive,
    math::{Frame, Vec2, Vec3},
    profile::Profile,
    sweep::extrude,
    topo::Brep,
};

fn print_proven(tag: &str, build: impl Fn() -> Brep) {
    let fresh = build();
    let bytes = codec::to_bytes(std::slice::from_ref(&fresh));
    let decoded = codec::from_bytes(&bytes).and_then(|mut v| v.pop()).expect("the bytes read back");
    assert!(decoded == fresh, "{tag}: decoded differs from a fresh build");
    assert_eq!(codec::to_bytes(std::slice::from_ref(&decoded)), bytes, "{tag}: re-encoding differs");
    assert_eq!(codec::to_bytes(&[build()]), bytes, "{tag}: a second build differs");
    let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
    println!("FIXTUREHEX {tag} {hex}");
}

/// boolean/tests.rs's `rod`, verbatim.
fn rod(base: [f64; 3], axis: [f64; 3], r: f64, len: f64) -> Brep {
    let frame = Frame::from_axis(Vec3::from_array(base), Vec3::from_array(axis)).unwrap();
    extrude(&Profile::circle(Vec2::ZERO, 2.0 * r), &frame, len, "h").unwrap()
}

/// boolean/tests.rs's `square_key`, verbatim.
fn square_key(id: &str, turn: f64, (z0, z1): (f64, f64), chamfer_by: f64) -> Brep {
    let x = Vec3::new(turn.cos(), turn.sin(), 0.0);
    let f = Frame::new(Vec3::new(0.0, 0.0, z0), Vec3::Z, x).unwrap();
    let key = extrude(&Profile::rect(Vec2::ZERO, 4.0, 4.0), &f, z1 - z0, id).unwrap();
    if chamfer_by <= 0.0 {
        return key;
    }
    let end = format!("{id}.end");
    let sides = ["v_min", "u_max", "v_max", "u_min"].map(|s| format!("{id}.{s}"));
    let pairs: Vec<(&str, &str)> = sides.iter().map(|s| (end.as_str(), s.as_str())).collect();
    chamfer(&key, &pairs, chamfer_by, "ch").unwrap()
}

#[test]
#[ignore = "W8's one-off fixture printer"]
fn print_w8_fixtures() {
    // The rod of radius 20 rounded 3 at its top (a_torus_face_is_boxed_by_its_own_extremes_not_the_whole_torus,
    // a_face_hull_box_holds_its_loose_box).
    print_proven("rounded", || {
        let r = rod([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 20.0, 20.0);
        fillet(&r, &[("h.end", "h.side")], 3.0, "f").unwrap()
    });
    // The 4 x 4 key turned 30 degrees, from z -3 to 3, its top chamfered 0.5
    // (a_key_in_a_pocket_of_its_own_size_a_hair_off_meets_the_wall_in_slivers).
    print_proven("key-chamfered", || square_key("key", 30f64.to_radians(), (-3.0, 3.0), 0.5));
    // boolean/tests/more.rs's `bulging`: the ball of radius 2 on the tilted frame, stretched by `map`.
    print_proven("affine-ball", || {
        let tilted = Frame::new(
            Vec3::new(1.0, -2.0, 0.5),
            Vec3::new(0.3, -0.5, 0.8),
            Vec3::new(1.0, 0.2, 0.0),
        )
        .unwrap();
        let ball = primitive::sphere(&tilted, 2.0, "s").unwrap();
        let map = Affine {
            m: [[1.1, -0.9, 0.5], [-1.0, 0.6, 0.5], [1.3, 0.6, 0.4]],
            t: Vec3::new(2.0, -3.0, 1.0),
        };
        affine(&ball, &map).unwrap()
    });
}
