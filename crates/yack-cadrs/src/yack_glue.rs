// yack: glue the yack facade calls for the cadrs engine (PLAN §3.5, M1.10); dies with this crate (M4).
//! Thin functions over cadrs's own API, taking plain arrays, so `crates/yack` routes and never
//! computes (gate 13). Each one does only the arithmetic yack's argument shapes need.

use nalgebra::{Point3, Vector3};

use crate::{Axis, BodyId, Bspline3, DefaultKernel, History, Kernel, KernelError, OpResult, PathCurve, Result, SolidMotion};

/// An operation's one result body and what became of its inputs' faces.
pub type Made = (BodyId, History);

fn made(r: Result<OpResult>) -> Result<Made> {
    let r = r?;
    Ok((r.first()?, r.history))
}

fn p3(p: [f64; 3]) -> Point3<f64> {
    Point3::new(p[0], p[1], p[2])
}

fn v3(v: [f64; 3]) -> Vector3<f64> {
    Vector3::new(v[0], v[1], v[2])
}

/// One rigid motion of a solid, in yack's terms.
#[derive(Clone, Debug, PartialEq)]
pub enum Move<'a> {
    /// A straight slide by `by`.
    #[allow(missing_docs)]
    Slide { by: [f64; 3] },
    /// A turn by `angle` radians about the line through `point` along `axis`.
    #[allow(missing_docs)]
    Turn { point: [f64; 3], axis: [f64; 3], angle: f64 },
    /// A turn by `angle` radians while rising `rise_per_turn` along the axis each whole turn.
    #[allow(missing_docs)]
    Screw { point: [f64; 3], axis: [f64; 3], angle: f64, rise_per_turn: f64 },
    /// Carried along a clamped B-spline by a frame whose z axis stays `up` and whose x axis follows
    /// the tangent.
    #[allow(missing_docs)]
    Along { degree: usize, knots: &'a [f64], ctrl: &'a [[f64; 3]], up: [f64; 3] },
}

/// Everything `body` passes through as it moves by `m` (cadrs's sweep of a solid).
pub fn swept(k: &mut DefaultKernel, body: BodyId, m: &Move) -> Result<Made> {
    let dir = match *m {
        Move::Turn { axis, .. } | Move::Screw { axis, .. } => axis,
        Move::Along { up, .. } => up,
        Move::Slide { by } => by,
    };
    if v3(dir).norm() == 0.0 {
        return Err(KernelError::InvalidInput("a motion needs a non-zero direction".into()));
    }
    let motion = match *m {
        Move::Slide { by } => SolidMotion::slide(v3(by)),
        Move::Turn { point, axis, angle } => SolidMotion::swing(Axis::new(p3(point), v3(axis)), 0.0, angle),
        Move::Screw { point, axis, angle, rise_per_turn } => {
            SolidMotion::helical(Axis::new(p3(point), v3(axis)), rise_per_turn, 0.0, angle)
        }
        Move::Along { degree, knots, ctrl, up } => {
            let spline = Bspline3 {
                degree,
                knots: knots.to_vec(),
                poles: ctrl.iter().map(|&c| p3(c)).collect(),
                weights: Vec::new(),
            };
            SolidMotion::along(vec![PathCurve::Bspline(spline)], v3(up))
        }
    };
    made(k.sweep_solid(body, &motion))
}

/// The unit normal of the plane holding every control point, or `None` when they are collinear
/// or not coplanar (within `tol` relative to their spread).
pub fn plane_normal(ctrl: &[[f64; 3]], tol: f64) -> Option<[f64; 3]> {
    let pts: Vec<Point3<f64>> = ctrl.iter().map(|&c| p3(c)).collect();
    let first = *pts.first()?;
    let spread = pts.iter().map(|p| (p - first).norm()).fold(0.0, f64::max);
    if spread == 0.0 {
        return None;
    }
    let mut n = Vector3::zeros();
    for (i, a) in pts.iter().enumerate() {
        let b = pts.get((i + 1) % pts.len())?;
        n += (a - first).cross(&(b - first));
    }
    let len = n.norm();
    if len <= tol * spread * spread {
        return None;
    }
    let n = n / len;
    if pts.iter().any(|p| (p - first).dot(&n).abs() > tol * spread) {
        return None;
    }
    Some([n.x, n.y, n.z])
}

// ---- ops by face and edge index (the bridge keeps springrep's indices) ----

use crate::{ChamferOpts, ChamferSpec, DraftSpec, EdgeId, FaceId, FaceInput, Motion, OffsetSpec, Primitive, ShellSpec, ThickenSpec};

fn faces(ix: &[u64]) -> Vec<FaceId> {
    ix.iter().map(|&i| FaceId(i)).collect()
}

fn edges(ix: &[u64]) -> Vec<EdgeId> {
    ix.iter().map(|&i| EdgeId(i)).collect()
}

/// `body` less faces `ix`, the gap healed from the neighbours.
pub fn delete_faces(k: &mut DefaultKernel, body: BodyId, ix: &[u64]) -> Result<Made> {
    made(k.delete_faces(body, &faces(ix)))
}

/// Faces `ix` moved by the rows `[R | t]`.
pub fn move_faces(k: &mut DefaultKernel, body: BodyId, ix: &[u64], m: [[f64; 4]; 3]) -> Result<Made> {
    let linear = nalgebra::Matrix3::from_fn(|i, j| m[i][j]);
    let motion = Motion { linear, translation: Vector3::new(m[0][3], m[1][3], m[2][3]) };
    made(k.move_faces(body, &faces(ix), &motion))
}

/// Every face offset by `distance` (positive grows), rounded where faces part.
pub fn offset(k: &mut DefaultKernel, body: BodyId, distance: f64) -> Result<Made> {
    made(k.offset(body, &OffsetSpec { distance, faces: Vec::new(), sharp: false }))
}

/// Faces `ix` offset by `distance`, their neighbours extended.
pub fn offset_faces(k: &mut DefaultKernel, body: BodyId, ix: &[u64], distance: f64) -> Result<Made> {
    let by = faces(ix).into_iter().map(|f| (f, distance)).collect();
    made(k.offset(body, &OffsetSpec { distance: 0.0, faces: by, sharp: true }))
}

/// A shell of wall `thickness`, inward or `outward`, open at faces `open` (none: a closed hollow).
pub fn shell(k: &mut DefaultKernel, body: BodyId, open: &[u64], thickness: f64, outward: bool) -> Result<Made> {
    let spec = ShellSpec { remove: faces(open), thickness, outward, hollow: open.is_empty(), sharp: false };
    made(k.shell_with(body, &spec))
}

/// Faces `ix` thickened by `thickness` (positive along their outward normals).
pub fn thicken(k: &mut DefaultKernel, body: BodyId, ix: &[u64], thickness: f64) -> Result<Made> {
    let inputs = faces(ix).into_iter().map(|face| FaceInput { body, face, source: 0 }).collect();
    let spec = ThickenSpec {
        bodies: Vec::new(),
        faces: inputs,
        profiles: Vec::new(),
        along: thickness.max(0.0),
        against: (-thickness).max(0.0),
        sharp: false,
        source: 0,
    };
    made(k.thicken_surfaces(&spec))
}

/// Faces `ix` drafted by `angle` radians about the neutral plane through `origin` square to `pull`.
pub fn draft(k: &mut DefaultKernel, body: BodyId, ix: &[u64], origin: [f64; 3], pull: [f64; 3], angle: f64) -> Result<Made> {
    if v3(pull).norm() == 0.0 {
        return Err(KernelError::InvalidInput("a draft's pull direction is zero".into()));
    }
    let spec = DraftSpec {
        faces: faces(ix),
        angle,
        pull: nalgebra::Unit::new_normalize(v3(pull)),
        neutral: p3(origin),
        neutral_normal: None,
        tangent_propagation: false,
    };
    made(k.draft(body, &spec))
}

/// Edges `ix` rounded with `radius`.
pub fn fillet(k: &mut DefaultKernel, body: BodyId, ix: &[u64], radius: f64) -> Result<Made> {
    made(k.fillet(body, &edges(ix), radius))
}

/// Edges `ix` chamfered by `spec`, its first distance on face `first` where given.
fn chamfer_spec(k: &mut DefaultKernel, body: BodyId, ix: &[u64], spec: ChamferSpec, first: Option<u64>) -> Result<Made> {
    let mut opts = ChamferOpts::new(spec);
    if let Some(f) = first {
        opts = opts.first_on(FaceId(f));
    }
    made(k.chamfer_with(body, &edges(ix), &opts))
}

/// Edges `ix` chamfered `distance` along each face.
pub fn chamfer(k: &mut DefaultKernel, body: BodyId, ix: &[u64], distance: f64) -> Result<Made> {
    chamfer_spec(k, body, ix, ChamferSpec::EqualDistance(distance), None)
}

/// Edge `e` chamfered `d1` along face `face` and `d2` along the other.
pub fn chamfer_two(k: &mut DefaultKernel, body: BodyId, e: u64, face: u64, d1: f64, d2: f64) -> Result<Made> {
    chamfer_spec(k, body, &[e], ChamferSpec::TwoDistances(d1, d2), Some(face))
}

/// Edge `e` chamfered `distance` along face `face`, the bevel at `angle` radians to that face.
pub fn chamfer_angle(k: &mut DefaultKernel, body: BodyId, e: u64, face: u64, distance: f64, angle: f64) -> Result<Made> {
    chamfer_spec(k, body, &[e], ChamferSpec::DistanceAngle { distance, angle }, Some(face))
}

/// The solid torus about the line through `center` along `axis` (horn and spindle tori too).
pub fn torus(k: &mut DefaultKernel, center: [f64; 3], axis: [f64; 3], major: f64, minor: f64) -> Result<Made> {
    if v3(axis).norm() == 0.0 {
        return Err(KernelError::InvalidInput("a torus's axis is zero".into()));
    }
    let p = Primitive::Torus { center: p3(center), axis: nalgebra::Unit::new_normalize(v3(axis)), major, minor };
    made(k.primitive(&p))
}
