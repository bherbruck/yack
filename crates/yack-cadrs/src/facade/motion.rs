// yack: yack's composite motions over cadrs's sweep of a solid: `Then` as the union of each leg's sweep.
//! cadrs sweeps a solid by one motion at a time ([`crate::yack_glue::swept`]); its `Rigid` steps
//! run *together*, not one after another. A chain of motions sweeps the union of its legs, each
//! leg starting where the one before left the body: exact, given each leg's sweep is.

use std::f64::consts::TAU;

use nalgebra::{Matrix3, Rotation3, Unit, Vector3};

use crate::yack_glue::{Made, Move, swept};
use crate::{BodyId, BoolOp, DefaultKernel, Kernel, KernelError, Motion, Result};

fn v3(v: [f64; 3]) -> Vector3<f64> {
    Vector3::new(v[0], v[1], v[2])
}

fn unit(v: [f64; 3], what: &str) -> Result<Unit<Vector3<f64>>> {
    Unit::try_new(v3(v), 0.0).ok_or_else(|| KernelError::InvalidInput(format!("{what} is zero")))
}

/// How near a whole number of turns (radians) a turn's end is taken as one: [`turn_matrix`].
const WHOLE_TURN: f64 = crate::native::tol::ANGULAR_FINE;

/// The rotation by `angle` about `axis`, computed from the angle reduced to [0, 2π): whole turns
/// (within [`WHOLE_TURN`]) are exactly the identity, and 370° turns exactly as 10° does. A
/// rotation by 2π itself is a hair off the identity (cos 2π, sin 2π rounded), and a toolpath of
/// many whole turns built that up on its tool until a later cut met an earlier one's edges a
/// hair apart (cnc/five-axis-dome's roughing: "edge used by 3 faces" after 17 rings).
fn turn_matrix(axis: &Unit<Vector3<f64>>, angle: f64) -> Matrix3<f64> {
    let a = angle.rem_euclid(TAU);
    if a <= WHOLE_TURN || TAU - a <= WHOLE_TURN {
        Matrix3::identity()
    } else {
        Rotation3::from_axis_angle(axis, a).into_inner()
    }
}

/// The turn by `angle` about the line through `point` along `axis`, then the slide `along`.
fn turned(point: [f64; 3], axis: &Unit<Vector3<f64>>, angle: f64, along: Vector3<f64>) -> Motion {
    let r = turn_matrix(axis, angle);
    let p = v3(point);
    Motion { linear: r, translation: p - r * p + along }
}

/// The frame cadrs's path sweep carries the body by at a point of tangent `t`: z along `up`,
/// x the tangent made square to it, y = z × x (as columns).
fn path_frame(t: Vector3<f64>, up: &Unit<Vector3<f64>>) -> Result<Matrix3<f64>> {
    let x = t - up.into_inner() * t.dot(up);
    let x = Unit::try_new(x, 0.0)
        .ok_or(KernelError::Unsupported("a path running along its up direction"))?
        .into_inner();
    let z = up.into_inner();
    Ok(Matrix3::from_columns(&[x, z.cross(&x), z]))
}

/// The rigid motion that takes the body from where a leg starts to where it ends.
pub fn end_motion(m: &Move) -> Result<Motion> {
    Ok(match *m {
        Move::Slide { by } => Motion { linear: Matrix3::identity(), translation: v3(by) },
        Move::Turn { point, axis, angle } => turned(point, &unit(axis, "a turn's axis")?, angle, Vector3::zeros()),
        Move::Screw { point, axis, angle, rise_per_turn } => {
            let a = unit(axis, "a screw's axis")?;
            let rise = a.into_inner() * (rise_per_turn * angle / TAU);
            turned(point, &a, angle, rise)
        }
        Move::Climb { point, axis, to, from, rise } => {
            let a = unit(axis, "a screw's axis")?;
            let up = a.into_inner() * crate::yack_glue::climb_rise(rise, from, to);
            turned(point, &a, to, up)
        }
        Move::Along { degree, ctrl, up, .. } => {
            // A clamped B-spline starts at its first control point and ends at its last, along
            // the first and last legs of its control polygon.
            let (Some(&p0), Some(&p1)) = (ctrl.first(), ctrl.last()) else {
                return Err(KernelError::InvalidInput("a path with no control points".into()));
            };
            if degree == 0 || ctrl.len() < 2 {
                return Err(KernelError::InvalidInput("a path needs degree 1 or more and two points".into()));
            }
            let n = ctrl.len();
            let t0 = v3(ctrl[1]) - v3(ctrl[0]);
            let t1 = v3(ctrl[n - 1]) - v3(ctrl[n - 2]);
            let up = unit(up, "a path's up direction")?;
            let f0 = path_frame(t0, &up)?;
            let f1 = path_frame(t1, &up)?;
            let linear = f1 * f0.transpose();
            Motion { linear, translation: v3(p1) - linear * v3(p0) }
        }
    })
}

/// Everything `body` passes through as it moves by each of `moves` in turn: the union of each
/// leg's sweep, each leg starting from where the legs before left it.
pub fn swept_then(k: &mut DefaultKernel, body: BodyId, moves: &[Move]) -> Result<Made> {
    crate::yack_glue::exact(k, |k| swept_then_exactly(k, body, moves))
}

fn swept_then_exactly(k: &mut DefaultKernel, body: BodyId, moves: &[Move]) -> Result<Made> {
    match moves {
        [] => return Err(KernelError::InvalidInput("a chain of no motions".into())),
        [m] => return swept(k, body, m),
        _ => {}
    }
    let mut made: Vec<BodyId> = Vec::new();
    let r = (|| {
        let mut pieces = Vec::new();
        let mut at = body;
        for (i, m) in moves.iter().enumerate() {
            let (piece, _) = swept(k, at, m)?;
            made.push(piece);
            pieces.push(piece);
            if i + 1 < moves.len() {
                at = k.transform_motion(at, &end_motion(m)?)?.first()?;
                made.push(at);
            }
        }
        let r = k.boolean(BoolOp::Union, pieces[0], &pieces[1..])?;
        Ok((r.first()?, r.history))
    })();
    // The legs and the moved copies were only steps on the way.
    for id in made {
        if r.as_ref().map_or(true, |(out, _): &Made| *out != id) {
            k.release(id);
        }
    }
    r
}

/// One step of a motion run with others at once ([`together`]): each moves by its own amount
/// at each instant, the instant running from 0 to 1 for all of them alike.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step {
    /// A slide from the offset `from` to `to`.
    #[allow(missing_docs)]
    Slide { from: [f64; 3], to: [f64; 3] },
    /// A turn about the line through `point` along `axis` from the angle `from` to `to` (radians,
    /// right-handed about `axis`).
    #[allow(missing_docs)]
    Turn { point: [f64; 3], axis: [f64; 3], from: f64, to: f64 },
    /// A turn as above while rising `rise_per_turn` along the axis per whole turn.
    #[allow(missing_docs)]
    Screw { point: [f64; 3], axis: [f64; 3], from: f64, to: f64, rise_per_turn: f64 },
}

/// `steps` as cadrs's planar motion when they all lie in one plane: every turn about a line
/// along one normal (either way round), every slide square to it, no screw's rise; `None` when
/// they don't (or there is no turn to fix the plane: slides alone are exact as they are).
fn planar_steps(steps: &[Step]) -> Result<Option<crate::SolidMotion>> {
    use crate::{PlanarStep, SolidMotion};
    let tol = crate::native::tol::ANGULAR;
    let normal = steps.iter().find_map(|s| match *s {
        Step::Turn { axis, .. } => Some(axis),
        _ => None,
    });
    let Some(normal) = normal else { return Ok(None) };
    let n = unit(normal, "a turn's axis")?;
    let mut out = Vec::with_capacity(steps.len());
    for s in steps {
        match *s {
            Step::Turn { point, axis, from, to } => {
                let a = unit(axis, "a turn's axis")?;
                let c = a.dot(&n);
                if (c.abs() - 1.0).abs() > tol {
                    return Ok(None);
                }
                let sign = c.signum();
                out.push(PlanarStep::Turn { about: nalgebra::Point3::from(v3(point)), from: sign * from, to: sign * to });
            }
            Step::Slide { from, to } => {
                let (f, t) = (v3(from), v3(to));
                let reach = f.norm().max(t.norm());
                if f.dot(&n).abs() > tol * reach || t.dot(&n).abs() > tol * reach {
                    return Ok(None);
                }
                out.push(PlanarStep::Slide { from: f, to: t });
            }
            Step::Screw { rise_per_turn, .. } if rise_per_turn != 0.0 => return Ok(None),
            Step::Screw { point, axis, from, to, .. } => {
                let a = unit(axis, "a screw's axis")?;
                let c = a.dot(&n);
                if (c.abs() - 1.0).abs() > tol {
                    return Ok(None);
                }
                out.push(PlanarStep::Turn { about: nalgebra::Point3::from(v3(point)), from: c.signum() * from, to: c.signum() * to });
            }
        }
    }
    Ok(Some(SolidMotion::Planar { normal: n, steps: out }))
}

/// What [`together`] made: the solid and its history. Always cadrs's exact answer: it runs
/// exact_only, so where cadrs would only mesh the region it refuses instead.
#[derive(Clone, Debug)]
pub struct Together {
    /// The body and its history.
    pub made: Made,
}

/// Everything `body` passes through as `steps` all run at once, applied in order at each
/// instant (cadrs's `SolidMotion::Rigid`, or its planar motion where the steps lie in one plane;
/// the body drawn where it is at every step's start). Exact or refused (exact_only).
pub fn together(k: &mut DefaultKernel, body: BodyId, steps: &[Step]) -> Result<Together> {
    crate::yack_glue::exact(k, |k| together_exactly(k, body, steps))
}

fn together_exactly(k: &mut DefaultKernel, body: BodyId, steps: &[Step]) -> Result<Together> {
    let r = k.sweep_solid(body, &motion_of(steps)?)?;
    Ok(Together { made: (r.first()?, r.history) })
}

/// `target` less everything `tool` passes through as `steps` all run at once (cadrs's cut by a
/// moving solid, as cadrs's runner hobs a worm wheel or cuts a bevel gear by its pinion: two
/// turns about crossed axes at a ratio). Exact or refused (exact_only).
pub fn cut_together(k: &mut DefaultKernel, target: BodyId, tool: BodyId, steps: &[Step]) -> Result<Made> {
    crate::yack_glue::exact(k, |k| {
        let r = k.cut_swept(target, tool, &motion_of(steps)?)?;
        Ok((r.first()?, r.history))
    })
}

/// `steps` as cadrs's motion: slides alone as one slide by their sum; steps all in one plane
/// (slides in it, turns about lines square to it) as its planar motion, which it sweeps
/// exactly (as cadrs's runner cuts a rack's tooth space or a planet carrier's square); else
/// the rigid motion.
fn motion_of(steps: &[Step]) -> Result<crate::SolidMotion> {
    use crate::{Axis, RigidStep, SolidMotion};
    if steps.is_empty() {
        return Err(KernelError::InvalidInput("a motion of no steps".into()));
    }
    if steps.iter().all(|s| matches!(s, Step::Slide { .. })) {
        let (mut from, mut to) = (Vector3::zeros(), Vector3::zeros());
        for s in steps {
            if let Step::Slide { from: f, to: t } = *s {
                from += v3(f);
                to += v3(t);
            }
        }
        return Ok(SolidMotion::rigid(vec![RigidStep::slide(from, to)]));
    }
    if let Some(planar) = planar_steps(steps)? {
        return Ok(planar);
    }
    let mut rigid = Vec::new();
    for s in steps {
        match *s {
            Step::Slide { from, to } => rigid.push(RigidStep::slide(v3(from), v3(to))),
            Step::Turn { point, axis, from, to } => {
                let a = unit(axis, "a turn's axis")?;
                rigid.push(RigidStep::turn(Axis { origin: nalgebra::Point3::from(v3(point)), dir: a }, from, to));
            }
            Step::Screw { point, axis, from, to, rise_per_turn } => {
                let a = unit(axis, "a screw's axis")?;
                let rise = |t: f64| a.into_inner() * (rise_per_turn * t / TAU);
                rigid.push(RigidStep::turn(Axis { origin: nalgebra::Point3::from(v3(point)), dir: a }, from, to));
                rigid.push(RigidStep::slide(rise(from), rise(to)));
            }
        }
    }
    Ok(SolidMotion::rigid(rigid))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Primitive;
    use nalgebra::Point3;

    #[test]
    fn whole_turns_end_where_they_started() {
        let z = Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(turn_matrix(&z, TAU), Matrix3::identity());
        assert_eq!(turn_matrix(&z, 2.0 * TAU), Matrix3::identity());
        assert_eq!(turn_matrix(&z, -TAU), Matrix3::identity());
        let e = 1e-3;
        let want = Rotation3::from_axis_angle(&z, e).into_inner();
        assert!((turn_matrix(&z, TAU + e) - want).amax() < 1e-15);
        let m = end_motion(&Move::Turn { point: [5.0, 0.0, 0.0], axis: [0.0, 0.0, 1.0], angle: 4.0 * TAU }).unwrap();
        assert_eq!(m.linear, Matrix3::identity());
        assert_eq!(m.translation, Vector3::zeros());
    }

    #[test]
    fn a_cut_by_steps_at_once_is_the_cut_by_their_motion() {
        // A turn and a slide along its axis at once is a screw: the same cut as the glue's.
        let mut k = DefaultKernel::new();
        let rod = k.primitive(&Primitive::Cylinder { base: Point3::origin(), axis: Unit::new_normalize(Vector3::z()), radius: 5.0, height: 20.0 }).and_then(|r| r.first()).unwrap();
        let bar = k.primitive(&Primitive::Box { min: Point3::new(4.0, -0.5, 2.0), max: Point3::new(7.0, 0.5, 3.0) }).and_then(|r| r.first()).unwrap();
        let steps = [
            Step::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], from: 0.0, to: TAU },
            Step::Slide { from: [0.0; 3], to: [0.0, 0.0, 6.0] },
        ];
        let (a, _) = cut_together(&mut k, rod, bar, &steps).unwrap();
        let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: TAU, rise_per_turn: 6.0 };
        let (b, _) = crate::yack_glue::cut_swept(&mut k, rod, bar, &[m]).unwrap();
        let (va, vb) = (k.mass_properties(a).unwrap().volume, k.mass_properties(b).unwrap().volume);
        assert!((va - vb).abs() < 1e-6 * va, "{va} against {vb}");
    }

    #[test]
    fn planar_steps_sweep_exactly() {
        // brep-bench swept/square-on-a-planet-carrier and swept/rack-cut-tooth-space: cadrs's
        // runner 155.33667806397608 and 9042.502788700229, by its planar motion.
        let mut k = DefaultKernel::new();
        let bar = k.primitive(&Primitive::Box { min: Point3::new(4.0, -1.0, 0.0), max: Point3::new(6.0, 1.0, 2.0) }).and_then(|r| r.first()).unwrap();
        let z = [0.0, 0.0, 1.0];
        let spin = Step::Turn { point: [5.0, 0.0, 0.0], axis: z, from: 0.0, to: -3.0 * TAU };
        let carry = Step::Turn { point: [0.0; 3], axis: z, from: 0.0, to: TAU };
        let t = together(&mut k, bar, &[spin, carry]).unwrap();
        let v = k.mass_properties(t.made.0).unwrap().volume;
        assert!((v - 155.33667806397608).abs() < 1e-6 * v, "{v}");

        let d = -20.0 * std::f64::consts::FRAC_PI_3;
        let (a, b) = (0.842856, 3.026666);
        let tooth = [[d - a, 18.0], [d + a, 18.0], [d + b, 24.0], [d - b, 24.0]];
        let segs: Vec<crate::facade::profile::Seg> = (0..4).map(|i| crate::facade::profile::Seg::Line { a: tooth[i], b: tooth[(i + 1) % 4] }).collect();
        let side = crate::facade::profile::Place { origin: [0.0, -2.0, 0.0], x: [0.0, 0.0, 1.0], y: [1.0, 0.0, 0.0], z: [0.0, 1.0, 0.0] };
        let (rack, _) = crate::facade::profile::extrude(&mut k, &side, &[&segs], 10.0).unwrap();
        let slide = Step::Slide { from: [0.0; 3], to: [0.0, 0.0, -2.0 * d] };
        let turn = Step::Turn { point: [0.0; 3], axis: [0.0, 1.0, 0.0], from: -std::f64::consts::FRAC_PI_3, to: std::f64::consts::FRAC_PI_3 };
        let t = together(&mut k, rack, &[slide, turn]).unwrap();
        let blank = k
            .primitive(&Primitive::Cylinder { base: Point3::new(0.0, 0.0, 0.0), axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 0.0)), radius: 22.0, height: 6.0 })
            .and_then(|r| r.first())
            .unwrap();
        let cut = k.boolean(crate::BoolOp::Subtract, blank, &[t.made.0]).and_then(|r| r.first()).unwrap();
        let v = k.mass_properties(cut).unwrap().volume;
        assert!((v - 9042.502788700229).abs() < 1e-6 * v, "{v}");
    }

    fn cube(k: &mut DefaultKernel) -> BodyId {
        k.primitive(&Primitive::Box { min: Point3::new(0.0, 0.0, 0.0), max: Point3::new(1.0, 1.0, 1.0) })
            .and_then(|r| r.first())
            .unwrap()
    }

    #[test]
    fn two_slides_sweep_what_their_sum_does() {
        let mut k = DefaultKernel::new();
        let b = cube(&mut k);
        let legs = [Move::Slide { by: [2.0, 0.0, 0.0] }, Move::Slide { by: [0.0, 3.0, 0.0] }];
        let (s, _) = swept_then(&mut k, b, &legs).unwrap();
        // An L: 1 × 3 × 1 along x, then 1 × 4 × 1 up y from x = 2, sharing the 1 × 1 × 1 cube there.
        let v = k.mass_properties(s).unwrap().volume;
        assert!((v - (3.0 + 4.0 - 1.0)).abs() < 1e-9, "{v}");
    }

    #[test]
    fn a_screw_ends_turned_and_risen() {
        let m = end_motion(&Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: TAU / 4.0, rise_per_turn: 8.0 }).unwrap();
        let p = m.point(&Point3::new(1.0, 0.0, 0.0));
        assert!((p - Point3::new(0.0, 1.0, 2.0)).norm() < 1e-12, "{p:?}");
    }

    #[test]
    fn a_path_ends_in_its_end_frame() {
        // A quarter turn in the xy plane, up z: the tangent turns from +x to +y.
        let ctrl = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];
        let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let m = end_motion(&Move::Along { degree: 2, knots: &knots, ctrl: &ctrl, up: [0.0, 0.0, 1.0] }).unwrap();
        assert!((m.point(&Point3::new(0.0, 0.0, 0.0)) - Point3::new(1.0, 1.0, 0.0)).norm() < 1e-12);
        assert!((m.vector(&Vector3::x()) - Vector3::y()).norm() < 1e-12);
    }

    #[test]
    fn steps_run_together() {
        let mut k = DefaultKernel::new();
        let b = cube(&mut k);
        // A slide by 2 along x and one by 3 along y at once: the cube swept along the diagonal.
        let steps = [Step::Slide { from: [0.0; 3], to: [2.0, 0.0, 0.0] }, Step::Slide { from: [0.0; 3], to: [0.0, 3.0, 0.0] }];
        let t = together(&mut k, b, &steps).unwrap();
        // Slides at once are one slide by their sum: exact.
        // The unit square swept by (2, 3): area 1 + |2·1| + |3·1| = 6, times height 1.
        let v = k.mass_properties(t.made.0).unwrap().volume;
        assert!((v - 6.0).abs() < 1e-9, "{v}");
        // One slide is exact.
        let t = together(&mut k, b, &[Step::Slide { from: [0.0; 3], to: [2.0, 0.0, 0.0] }]).unwrap();
        let v = k.mass_properties(t.made.0).unwrap().volume;
        assert!((v - 3.0).abs() < 1e-9, "{v}");
    }
}
