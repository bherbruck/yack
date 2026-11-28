// yack: rigid motions over one parameter, where each places a body and how fast its points move.
// Adapted from springrep@f2ad60e src/sweep/solid/motion.rs (MIT OR Apache-2.0, (c) 2026 Springcraft, Inc.).
// Changes: written onto yack's kernel types (checked access, no expect: a frame that is undefined
// makes `rate` answer `None`); `Along` and `Then` carried as they are upstream; tests are new and
// check each motion's velocity field against the finite difference of its own placement.
//! Rigid motions over one parameter `τ ∈ [0, 1]`: where each places a body, and how fast each point
//! of the body moves.

use yack_base::kernel::{Error, Result};
use yack_build::sweep::Helix;
use yack_geom::Curve;
use yack_math::kernel::{Point3, Transform, Vec3};
use yack_tol as tol;

/// How a body moves as `τ` runs from 0 to 1. The body is given where it is drawn, and the motion
/// places it at every `τ`.
#[derive(Clone, Debug)]
pub(crate) enum Motion {
    /// Carried straight by the vector, from where it is drawn.
    Slide(Vec3),
    /// Turned right-handed about the line through `origin` along `axis`, from angle `from` to
    /// angle `to` (radians).
    Turn { origin: Point3, axis: Vec3, from: f64, to: f64 },
    /// Carried along `screw` from its turn `from` to its turn `to` (radians).
    Screw { screw: Helix, from: f64, to: f64 },
    /// Carried by a frame riding `path` from `range.0` to `range.1`: its origin on the path, its x
    /// along the path's tangent, its z as near `up` as square to the tangent allows. The body is
    /// drawn in the frame at `range.0`.
    Along { path: Curve, range: (f64, f64), up: Vec3 },
    /// Two motions over one parameter: the first places the body, and the second carries the whole
    /// of it, so at `τ` the body is where the first puts it, moved by the second.
    Then(Box<Motion>, Box<Motion>),
}

/// The velocity of a rigid motion as a field: a point at `x` moves at `omega × x + v`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Twist {
    pub omega: Vec3,
    pub v: Vec3,
}

impl Twist {
    pub(crate) fn at(&self, x: Point3) -> Vec3 {
        self.omega.cross(x) + self.v
    }
}

/// The frame riding a path at one parameter: its origin, its axes, and their rates.
struct Riding {
    origin: Point3,
    axes: [Vec3; 3],
    rates: [Vec3; 3],
    speed: Vec3,
}

/// The frame riding `path` at `t`, `None` where the path has no tangent or runs along `up`.
fn riding(path: &Curve, up: Vec3, t: f64) -> Option<Riding> {
    let (origin, d1, d2) = path.d2(t);
    let len = d1.norm();
    let x = d1.unit()?;
    let dx = (d2 - x * d2.dot(x)) / len;
    let across = up - x * up.dot(x);
    let norm = across.norm();
    let z = across.unit()?;
    let dw = -(dx * up.dot(x) + x * up.dot(dx));
    let dz = (dw - z * dw.dot(z)) / norm;
    let y = z.cross(x);
    let dy = dz.cross(x) + z.cross(dx);
    Some(Riding { origin, axes: [x, y, z], rates: [dx, dy, dz], speed: d1 })
}

/// The placement of a riding frame: its axes as columns, its origin as the translation.
fn frame_transform(r: &Riding) -> Transform {
    let [x, y, z] = r.axes;
    Transform { r: [[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]], t: r.origin }
}

/// How many samples along a path's range `validate` tries for a frame.
const FRAME_SAMPLES: usize = 64;

impl Motion {
    /// Refuses a motion that cannot place a body: no direction, no length, or a path whose frame is
    /// undefined somewhere along it.
    pub(crate) fn validate(&self) -> Result<()> {
        let turns = |from: f64, to: f64| from.is_finite() && to.is_finite() && (to - from).abs() > tol::ANGULAR;
        match self {
            Motion::Slide(d) => {
                if !(d.is_finite() && d.norm() > tol::LINEAR) {
                    return invalid("a slide moves by a vector longer than the tolerance");
                }
            }
            Motion::Turn { axis, from, to, .. } => {
                if axis.unit().is_none() {
                    return invalid("a turn needs an axis");
                }
                if !turns(*from, *to) {
                    return invalid("a turn turns through an angle");
                }
            }
            Motion::Screw { screw, from, to } => {
                if screw.axis.unit().is_none() {
                    return invalid("a screw needs an axis");
                }
                if !turns(*from, *to) {
                    return invalid("a screw motion turns through an angle");
                }
            }
            Motion::Along { path, range, up } => {
                if range.1.partial_cmp(&range.0) != Some(std::cmp::Ordering::Greater) {
                    return invalid("a frame along a path runs over a range of the path");
                }
                let steps = FRAME_SAMPLES as f64;
                for k in 0..=FRAME_SAMPLES {
                    let t = range.0 + (range.1 - range.0) * k as f64 / steps;
                    if riding(path, *up, t).is_none() {
                        return invalid("a frame along a path needs a tangent apart from up");
                    }
                }
            }
            Motion::Then(a, b) => {
                a.validate()?;
                b.validate()?;
            }
        }
        Ok(())
    }

    /// Where the body is at `τ`, and the velocity of the motion there as a field over space; `None`
    /// where a path has no frame (a motion [`Motion::validate`] passed has one everywhere).
    pub(crate) fn rate(&self, tau: f64) -> Option<(Transform, Twist)> {
        Some(match self {
            Motion::Slide(d) => (Transform::translation(*d * tau), Twist { omega: Vec3::ZERO, v: *d }),
            Motion::Turn { origin, axis, from, to } => {
                let k = axis.unit_or_zero();
                let omega = k * (to - from);
                (
                    Transform::rotation(*origin, k, from + (to - from) * tau),
                    Twist { omega, v: -omega.cross(*origin) },
                )
            }
            Motion::Screw { screw, from, to } => {
                let k = screw.axis.unit_or_zero();
                let s = Helix { axis: k, ..*screw };
                let omega = k * (screw.sign * (to - from));
                (
                    s.transform(from + (to - from) * tau),
                    Twist { omega, v: -omega.cross(screw.origin) + k * (screw.rise * (to - from)) },
                )
            }
            Motion::Along { path, range, up } => {
                let dt = range.1 - range.0;
                let (r0, r) = (riding(path, *up, range.0)?, riding(path, *up, range.0 + dt * tau)?);
                let pose = frame_transform(&r0).inverse().then(&frame_transform(&r));
                let half = r.axes.iter().zip(&r.rates).fold(Vec3::ZERO, |s, (a, d)| s + a.cross(*d));
                let omega = half * (0.5 * dt);
                (pose, Twist { omega, v: r.speed * dt - omega.cross(r.origin) })
            }
            Motion::Then(a, b) => {
                let ((pa, ta), (pb, tb)) = (a.rate(tau)?, b.rate(tau)?);
                let turned = pb.vector(ta.omega);
                (
                    pa.then(&pb),
                    Twist { omega: tb.omega + turned, v: tb.v - turned.cross(pb.t) + pb.vector(ta.v) },
                )
            }
        })
    }

    /// The velocity of each point of the body as drawn, in the body's own frame, where it is the
    /// same at every `τ`: a slide, a turn or a screw.
    pub(crate) fn steady(&self) -> Option<Twist> {
        match self {
            Motion::Slide(_) | Motion::Turn { .. } | Motion::Screw { .. } => self.body_twist(0.0),
            _ => None,
        }
    }

    /// The velocity at `τ` of each point of the body as drawn, in the body's own frame: the world
    /// velocity of its image, turned back by the pose.
    pub(crate) fn body_twist(&self, tau: f64) -> Option<Twist> {
        let (pose, w) = self.rate(tau)?;
        let back = pose.inverse();
        Some(Twist { omega: back.vector(w.omega), v: back.vector(w.omega.cross(pose.t) + w.v) })
    }
}

fn invalid(what: &str) -> Result<()> {
    Err(Error::InvalidInput(what.into()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};
    use yack_geom::{Circle, Line};
    use yack_math::kernel::Frame;

    fn close(a: Vec3, b: Vec3, within: f64) {
        assert!((a - b).norm() <= within, "{a:?} against {b:?}");
    }

    /// The motion's velocity field at the image of `p` at `tau`, against the central difference of
    /// where the pose puts `p`.
    fn field_matches_pose(m: &Motion, p: Point3, tau: f64, within: f64) {
        let h = 1e-5;
        let at = |t: f64| m.rate(t).map(|(pose, _)| pose.point(p));
        let (Some(a), Some(b), Some((pose, twist))) = (at(tau - h), at(tau + h), m.rate(tau)) else {
            panic!("no frame");
        };
        close(twist.at(pose.point(p)), (b - a) / (2.0 * h), within);
    }

    fn helix() -> Helix {
        Helix { origin: Vec3::new(1.0, -2.0, 0.5), axis: Vec3::new(0.0, 0.6, 0.8), rise: 0.7, sign: 1.0 }
    }

    /// A circle of radius 5 about (0, 0, 2) in the xy plane, the path of a frame turning once round.
    fn ring() -> Curve {
        let frame = Frame::new(Vec3::new(0.0, 0.0, 2.0), Vec3::Z, Vec3::X).unwrap();
        Curve::Circle(Circle { frame, radius: 5.0 })
    }

    fn all() -> Vec<Motion> {
        let turn = Motion::Turn { origin: Vec3::new(1.0, 2.0, 3.0), axis: Vec3::new(1.0, 1.0, 0.0), from: 0.2, to: 2.2 };
        let slide = Motion::Slide(Vec3::new(1.0, -2.0, 0.5));
        vec![
            slide.clone(),
            turn.clone(),
            Motion::Screw { screw: helix(), from: 0.1, to: 3.0 },
            Motion::Along { path: ring(), range: (0.3, 2.0), up: Vec3::Z },
            Motion::Then(Box::new(turn), Box::new(slide)),
            Motion::Then(
                Box::new(Motion::Turn { origin: Vec3::ZERO, axis: Vec3::Z, from: -0.3, to: 0.3 }),
                Box::new(Motion::Turn { origin: Vec3::new(0.0, 0.5, 0.5), axis: Vec3::X, from: 1.3, to: -1.3 }),
            ),
        ]
    }

    #[test]
    fn every_motions_velocity_field_is_the_derivative_of_its_placement() {
        for m in all() {
            m.validate().unwrap();
            for tau in [0.0, 0.3, 0.9] {
                for p in [Vec3::new(0.4, -1.0, 2.0), Vec3::new(-3.0, 2.0, 0.1)] {
                    field_matches_pose(&m, p, tau, 1e-6);
                }
            }
        }
    }

    #[test]
    fn a_turn_places_a_point_on_its_circle() {
        // About the z axis through the origin, from 0 to a quarter turn: x goes to y at the end.
        let m = Motion::Turn { origin: Vec3::ZERO, axis: Vec3::Z, from: 0.0, to: FRAC_PI_2 };
        let (end, _) = m.rate(1.0).unwrap();
        close(end.point(Vec3::new(2.0, 0.0, 1.0)), Vec3::new(0.0, 2.0, 1.0), 1e-12);
        let (_, twist) = m.rate(0.5).unwrap();
        assert_eq!((twist.omega, twist.v), (Vec3::new(0.0, 0.0, FRAC_PI_2), Vec3::ZERO));
    }

    #[test]
    fn a_screw_rises_by_its_rise_per_radian() {
        let screw = Helix { origin: Vec3::ZERO, axis: Vec3::Z, rise: 0.25, sign: 1.0 };
        let m = Motion::Screw { screw, from: 0.0, to: 2.0 * PI };
        let (end, twist) = m.rate(1.0).unwrap();
        close(end.point(Vec3::new(3.0, 0.0, 0.0)), Vec3::new(3.0, 0.0, 0.25 * 2.0 * PI), 1e-12);
        assert!((twist.v.z - 0.25 * 2.0 * PI).abs() < 1e-12);
    }

    #[test]
    fn only_a_slide_a_turn_or_a_screw_is_steady_in_the_bodys_frame() {
        let motions = all();
        for (k, m) in motions.iter().enumerate() {
            assert_eq!(m.steady().is_some(), k < 3, "motion {k}");
        }
        // Steady means the body-frame field is the same at every τ.
        for m in motions.iter().take(3) {
            let (a, b) = (m.body_twist(0.0).unwrap(), m.body_twist(0.7).unwrap());
            close(a.omega, b.omega, 1e-12);
            close(a.v, b.v, 1e-12);
        }
        // A frame riding a ring turns about its z axis at the rate the ring does: the body's own
        // field then changes with τ only through where the pose turns it, never its size.
        let ride = motions.get(3).unwrap();
        let (a, b) = (ride.body_twist(0.0).unwrap(), ride.body_twist(0.9).unwrap());
        assert!((a.omega.norm() - b.omega.norm()).abs() < 1e-9);
    }

    #[test]
    fn refuses_what_cannot_place_a_body() {
        let axis = Vec3::Z;
        let bad = [
            Motion::Slide(Vec3::ZERO),
            Motion::Turn { origin: Vec3::ZERO, axis: Vec3::ZERO, from: 0.0, to: 1.0 },
            Motion::Turn { origin: Vec3::ZERO, axis, from: 1.0, to: 1.0 },
            Motion::Screw { screw: helix(), from: 0.5, to: 0.5 },
            Motion::Along { path: ring(), range: (1.0, 1.0), up: axis },
            // `up` along the path's tangent at the ring's start (the tangent there is y).
            Motion::Along { path: ring(), range: (0.0, 1.0), up: Vec3::Y },
            Motion::Then(Box::new(Motion::Slide(Vec3::X)), Box::new(Motion::Slide(Vec3::ZERO))),
        ];
        for m in bad {
            assert!(matches!(m.validate(), Err(Error::InvalidInput(_))), "{m:?}");
        }
        let line = Curve::Line(Line { origin: Vec3::ZERO, dir: Vec3::X });
        let straight = Motion::Along { path: line, range: (0.0, 3.0), up: Vec3::Z };
        straight.validate().unwrap();
        // Carried along a straight line the frame only slides: no turn, the origin's speed.
        let (_, twist) = straight.rate(0.5).unwrap();
        close(twist.omega, Vec3::ZERO, 1e-12);
        close(twist.v, Vec3::new(3.0, 0.0, 0.0), 1e-12);
    }
}
