// yack: springrep module src/mesh/tests.rs
// Adapted from springrep@a03e8a1 src/mesh/tests.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/mesh/tests.rs to yack-mesh (M3 W6); paths and visibility only.
use std::f64::consts::TAU;
use crate::math::Vec2;

#[test]
fn images_of_one_point_a_rounding_apart_become_one() {
    // A crescent's loop passes its pinch point twice, once per circle, 4e-13 apart; a seam's two
    // images of one point lie a period apart.
    let mut pts = vec![
        (Vec2::new(4.075, 0.0), 7),
        (Vec2::new(0.0, 4.075), 8),
        (Vec2::new(4.075, -4e-13), 7),
        (Vec2::new(4.075 - TAU, 0.0), 7),
    ];
    super::snap_repeats(&mut pts);
    assert_eq!(pts[2].0, pts[0].0);
    assert_eq!(pts[3].0, Vec2::new(4.075 - TAU, 0.0));
    assert_eq!(pts[1].0, Vec2::new(0.0, 4.075));
}

#[test]
fn a_triangle_folded_onto_itself_beside_a_pole_is_dropped() {
    // An imported part's patch closes in a pole: point 2 is joined to the pole 0 on both sides of the
    // chord 1-2, so 0 1 2 and 1 0 2 cover one triangle both ways and 0-1 is used four times.
    let tris = vec![[1, 0, 3], [4, 0, 1], [0, 1, 2], [1, 0, 2], [0, 2, 5]];
    assert_eq!(super::unfold(tris), vec![[1, 0, 3], [4, 0, 1], [0, 2, 5]]);
    // A repeated triangle facing one way is not a fold.
    assert_eq!(super::unfold(vec![[0, 1, 2], [1, 2, 0]]).len(), 2);
}
