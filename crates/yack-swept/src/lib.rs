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
    run: fn(&Brep, &Motion) -> Result<Brep>,
}

/// The strategies in the order they are tried (doc/donors.md orders them once there are two).
const STRATEGIES: &[Strategy] = &[Strategy { name: "by-faces", run: by_faces::swept }];

/// Everything `body` passes through as it moves by `motion`: the first strategy's answer that
/// passes the checker. Declines (an [`Error::Unsupported`] naming each strategy's reason) where no
/// strategy answers; an [`Error::InvalidInput`] from a strategy is final.
pub fn swept(body: &Brep, motion: &Motion) -> Result<Brep> {
    let mut why = Vec::new();
    for s in STRATEGIES {
        match (s.run)(body, motion).and_then(finish) {
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

    #[test]
    fn with_nothing_built_yet_every_motion_declines() {
        let slide = Motion::Slide { by: Vec3::new(1.0, 0.0, 0.0) };
        assert!(matches!(swept(&Brep::new(), &slide), Err(Error::Unsupported(_))));
    }
}
