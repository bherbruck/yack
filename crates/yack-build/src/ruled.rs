// yack: the ruled solid between two profiles on two planes: every side face ruled by straight lines from a bottom edge to its top edge.
//! [`ruled`]: the solid whose sides are straight lines from `bottom`'s profile edges to `top`'s,
//! edge `i` to edge `i`, each running start to start as drawn in its own frame, so a top frame
//! turned about the axis twists the rulings (a circle to a turned circle is a hyperboloid of one
//! sheet). Its faces are `start` (the bottom), `end` (the top) and `side`.

use crate::{
    Error, Result,
    loft::{Section, loft},
    math::Frame,
    profile::Profile,
};

/// The ruled solid from `bottom_profile` drawn in `bottom` to `top_profile` drawn in `top`, named
/// under `feature`. Declines profiles it would not rule edge to edge as drawn: either profile with
/// a hole (only the outer loops are ruled), or outer loops of different edge counts (a loft would
/// split them at each other's vertices, not rule them as given).
pub fn ruled(bottom: &Frame, bottom_profile: &Profile, top: &Frame, top_profile: &Profile, feature: &str) -> Result<crate::topo::Brep> {
    let (b, t) = (bottom_profile.normalized()?, top_profile.normalized()?);
    if b.loops.len() != 1 || t.loops.len() != 1 {
        return Err(Error::Unsupported("a ruled solid between profiles with holes".into()));
    }
    let edges = |p: &Profile| p.loops.first().map_or(0, |l| l.segments.len());
    if edges(&b) != edges(&t) {
        return Err(Error::Unsupported("a ruled solid between outer loops of different edge counts".into()));
    }
    let sections = [Section { profile: b, frame: *bottom }, Section { profile: t, frame: *top }];
    loft(&sections, true, feature)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Vec2, Vec3};
    use std::f64::consts::PI;

    fn at(z: f64) -> Frame {
        Frame::new(Vec3::new(0.0, 0.0, z), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).unwrap()
    }

    #[test]
    fn a_circle_ruled_to_itself_turned_a_quarter_is_a_hyperboloid() {
        // Radius 5 at z = ±5, the top a quarter turn on: waist 5/√2, volume ∫ π(25/2 + z²/2) dz
        // over [−5, 5] = 500π/3. Unturned, it is the cylinder, 250π.
        let rim = Profile::circle(Vec2::new(0.0, 0.0), 10.0);
        let top = Frame::new(Vec3::new(0.0, 0.0, 5.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 1.0, 0.0)).unwrap();
        let h = ruled(&at(-5.0), &rim, &top, &rim, "r").unwrap();
        let v = crate::measure::volume(&h);
        assert!((v - 500.0 * PI / 3.0).abs() <= 1e-9 * v, "{v}");
        let c = ruled(&at(-5.0), &rim, &at(5.0), &rim, "r").unwrap();
        let v = crate::measure::volume(&c);
        assert!((v - 250.0 * PI).abs() <= 1e-9 * v, "{v}");
    }

    #[test]
    fn a_square_ruled_to_a_smaller_one_is_a_frustum() {
        // 4 × 4 at z = 0 to 2 × 2 at z = 3: h/3·(A + √(A·a) + a) = 28.
        let sq = |s: f64| Profile::rect(Vec2::new(0.0, 0.0), s, s);
        let f = ruled(&at(0.0), &sq(4.0), &at(3.0), &sq(2.0), "r").unwrap();
        let v = crate::measure::volume(&f);
        assert!((v - 28.0).abs() <= 1e-9 * v, "{v}");
    }

    #[test]
    fn profiles_it_would_not_rule_as_drawn_are_declined() {
        let sq = Profile::rect(Vec2::new(0.0, 0.0), 2.0, 2.0);
        let rim = Profile::circle(Vec2::new(0.0, 0.0), 2.0);
        assert!(ruled(&at(0.0), &sq, &at(1.0), &rim, "r").is_err());
    }
}
