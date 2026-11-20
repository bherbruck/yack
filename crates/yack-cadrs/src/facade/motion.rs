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

/// The turn by `angle` about the line through `point` along `axis`, then the slide `along`.
fn turned(point: [f64; 3], axis: &Unit<Vector3<f64>>, angle: f64, along: Vector3<f64>) -> Motion {
    let r = Rotation3::from_axis_angle(axis, angle).into_inner();
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Primitive;
    use nalgebra::Point3;

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
}
