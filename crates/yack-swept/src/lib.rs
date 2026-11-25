// yack: everything a solid passes through as it slides, turns or screws: yack's own swept rungs (M4.1).
//! What a solid passes through as it moves: yack's own rungs for `Solid::swept`, behind cadrs's
//! bridged one until they answer at least as well (lead's ruling A on M4 step 1).
//!
//! Each way of building the swept solid is a private strategy (PLAN §3.5, rule 7). It answers, or
//! declines with why (an [`Error::Unsupported`]); an [`Error::InvalidInput`] ends the ladder. The
//! first strategy that answers wins, and its solid goes through [`finish`]: a solid the checker
//! finds any issue in is a decline, never an answer (rule 8). A strategy declines wherever it is
//! unsure.

mod by_faces;

use yack_base::kernel::{Error, Result};
use yack_math::kernel::{Point3, Vec3};
use yack_topo::Brep;

/// How far a screw rises along its axis as it turns.
#[derive(Clone, Debug, PartialEq)]
pub enum Rise {
    /// This much along the axis each whole turn.
    PerTurn(f64),
    /// This much along the axis each radian.
    PerRadian(f64),
    /// The rise at the turn `t` (radians) as the polynomial `c[0] + c[1]·t + …`.
    Polynomial(Vec<f64>),
}

/// One rigid motion of a solid, from where it is given.
#[derive(Clone, Debug, PartialEq)]
pub enum Motion {
    /// A straight slide by `by`.
    Slide { by: Vec3 },
    /// A turn about the line through `axis_point` along `axis` from the angle `from` to `to`
    /// (radians, counter-clockwise looking down `axis`); the solid is as given at angle 0.
    Turn { axis_point: Point3, axis: Vec3, from: f64, to: f64 },
    /// A turn as [`Motion::Turn`] that rises along the axis by `rise` as it turns.
    Screw { axis_point: Point3, axis: Vec3, from: f64, to: f64, rise: Rise },
}

/// A way of building the swept solid: its name, and its run, which answers or declines.
struct Strategy {
    name: &'static str,
    run: fn(&Brep, &Motion, &str) -> Result<Brep>,
}

/// The strategies in the order they are tried (doc/donors.md orders them once there are two).
const STRATEGIES: &[Strategy] = &[Strategy { name: "by-faces", run: by_faces::swept }];

/// Everything `body` passes through as it moves by `motion`: the first strategy's answer that
/// passes the checker. Faces carried over from `body` keep their names; faces it makes are named
/// under `feature`: `end` where a face of `body` ends up, `side` what an edge sweeps (and `start`,
/// a face's sweep where it starts, should one survive the union). Declines (an [`Error::Unsupported`] naming each strategy's reason) where no
/// strategy answers; an [`Error::InvalidInput`] from a strategy is final.
pub fn swept(body: &Brep, motion: &Motion, feature: &str) -> Result<Brep> {
    let mut why = Vec::new();
    for s in STRATEGIES {
        match (s.run)(body, motion, feature).and_then(finish) {
            Ok(b) => return Ok(b),
            Err(e @ Error::InvalidInput(_)) => return Err(e),
            Err(e) => why.push(format!("{}: {e}", s.name)),
        }
    }
    Err(Error::Unsupported(format!("no swept strategy answers: {}", why.join("; "))))
}

/// `b`, where the checker finds no issue in it; else a decline listing the issues.
fn finish(b: Brep) -> Result<Brep> {
    let issues = yack_check::check(&b);
    if issues.is_empty() {
        Ok(b)
    } else {
        Err(Error::InvalidResult(issues.into_iter().map(|i| i.to_string()).collect()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use yack_math::kernel::Frame;

    fn cube() -> Brep {
        yack_build::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "box").unwrap()
    }

    #[test]
    fn a_box_slid_along_an_edge_is_a_longer_box() {
        let s = swept(&cube(), &Motion::Slide { by: Vec3::new(5.0, 0.0, 0.0) }, "f").unwrap();
        assert!((yack_measure::volume(&s) - 1500.0).abs() < 1e-9, "{}", yack_measure::volume(&s));
    }

    #[test]
    fn a_box_slid_aslant_covers_its_shadow_along_the_slide() {
        // A box slid by (3, 4, 0): itself, and the prisms its two leading faces sweep, 300 + 400.
        let s = swept(&cube(), &Motion::Slide { by: Vec3::new(3.0, 4.0, 0.0) }, "f").unwrap();
        assert!((yack_measure::volume(&s) - 1700.0).abs() < 1e-9, "{}", yack_measure::volume(&s));
        let s = swept(&cube(), &Motion::Slide { by: Vec3::new(1.0, 2.0, 3.0) }, "f").unwrap();
        assert!((yack_measure::volume(&s) - 1600.0).abs() < 1e-9, "{}", yack_measure::volume(&s));
    }

    fn near(got: f64, want: f64) {
        assert!((got - want).abs() <= 1e-6 * want, "{got} against {want}");
    }

    #[test]
    fn a_cylinder_slid_aslant_covers_its_shadow_along_the_slide() {
        // r 2, 10 tall, slid by (3, 0, 4): its volume and 5 times its shadow across the slide, the
        // disc's (cos 0.8) and the side's (2r·h·sin 0.6).
        let c = yack_build::primitive::cylinder(&Frame::WORLD, 2.0, 10.0, "c").unwrap();
        let s = swept(&c, &Motion::Slide { by: Vec3::new(3.0, 0.0, 4.0) }, "f").unwrap();
        let pi = std::f64::consts::PI;
        near(yack_measure::volume(&s), pi * 40.0 + 5.0 * (pi * 4.0 * 0.8 + 40.0 * 0.6));
    }

    #[test]
    fn a_ball_slid_is_a_capsule() {
        let ball = yack_build::primitive::sphere(&Frame::WORLD, 2.0, "ball").unwrap();
        let s = swept(&ball, &Motion::Slide { by: Vec3::new(0.0, 0.0, 5.0) }, "f").unwrap();
        let pi = std::f64::consts::PI;
        near(yack_measure::volume(&s), pi * 32.0 / 3.0 + pi * 4.0 * 5.0);
    }

    #[test]
    fn carried_faces_keep_their_names_and_new_ones_are_the_features() {
        use yack_topo::FaceName;
        let s = swept(&cube(), &Motion::Slide { by: Vec3::new(5.0, 0.0, 0.0) }, "f").unwrap();
        let names: Vec<FaceName> =
            s.face_ids().flat_map(|f| s.face(f).names.iter().cloned()).collect();
        assert!(names.contains(&FaceName::new("f", FaceName::END)), "{names:?}");
        assert!(names.iter().any(|n| n.feature == "box"), "{names:?}");
        assert!(names.iter().all(|n| n.feature == "box" || n.feature == "f"), "{names:?}");
    }

    #[test]
    fn what_is_not_built_yet_declines() {
        let turn = Motion::Turn { axis_point: Point3::ZERO, axis: Vec3::Z, from: 0.0, to: 1.0 };
        assert!(matches!(swept(&cube(), &turn, "f"), Err(Error::Unsupported(_))));
        let torus = yack_build::primitive::torus(&Frame::WORLD, 5.0, 1.0, "ring").unwrap();
        assert!(matches!(
            swept(&torus, &Motion::Slide { by: Vec3::X }, "f"),
            Err(Error::Unsupported(_))
        ));
        let nothing = Motion::Slide { by: Vec3::ZERO };
        assert!(matches!(swept(&cube(), &nothing, "f"), Err(Error::InvalidInput(_))));
    }
}
