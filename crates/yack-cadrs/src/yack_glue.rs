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

/// The most turns a screw [`swept`] takes on.
pub const SCREW_TURNS_MAX: f64 = 1.0;

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
    // A screw past a whole turn sweeps over itself; cadrs builds it as a mesh-bounded solid, tens
    // of seconds and tens of thousands of faces (swept/fine-thread-with-sharp-crests: > 60 s).
    // cadrs's own runner cuts such threads half a turn at a time from the trimmed tool instead.
    // Refused here at once (a deterministic budget), so the ladder moves on in time.
    if let Move::Screw { angle, .. } = *m
        && angle.abs() > SCREW_TURNS_MAX * std::f64::consts::TAU
    {
        return Err(KernelError::Unsupported("a screw past a whole turn (sweep it a half turn at a time)"));
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

use crate::{ChamferCorners, ChamferMeasure, ChamferOpts, ChamferSpec, DraftSpec, EdgeId, FaceId, FaceInput, Motion, OffsetSpec, Primitive, ShellSpec, ThickenSpec};

fn faces(ix: &[u64]) -> Vec<FaceId> {
    ix.iter().map(|&i| FaceId(i)).collect()
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

/// The shell `shell` and `shell_rounded` build, as cadrs's runner asks for it.
fn shell_spec(open: &[u64], thickness: f64, outward: bool, sharp: bool) -> ShellSpec {
    let spec = if open.is_empty() {
        ShellSpec::hollow(thickness)
    } else if outward {
        ShellSpec::outward(faces(open), thickness)
    } else {
        ShellSpec::inward(faces(open), thickness)
    };
    if sharp { spec.mitred() } else { spec }
}

/// A shell of wall `thickness`, inward or `outward`, open at faces `open` (none: a closed hollow),
/// its walls' corners SHARP (the moved faces extended until they meet), as yack's `shell` and
/// `shell_outward` are.
pub fn shell(k: &mut DefaultKernel, body: BodyId, open: &[u64], thickness: f64, outward: bool) -> Result<Made> {
    made(k.shell_with(body, &shell_spec(open, thickness, outward, true)))
}

/// [`shell`] with ROUNDED joins (cadrs's default, OCCT's arc join: outward a round on each convex
/// edge and a ball at each corner; inward the cavity rounded at the concave edges).
pub fn shell_rounded(k: &mut DefaultKernel, body: BodyId, open: &[u64], thickness: f64, outward: bool) -> Result<Made> {
    made(k.shell_with(body, &shell_spec(open, thickness, outward, false)))
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

/// Edges `ix` with every edge each continues tangentially (yack's fillets and chamfers run along
/// the whole tangent-continuous chain an edge is part of: a rim given by one of its halves is the
/// whole rim), seams left out; each with the index in `ix` of the edge whose chain it is on, in
/// `ix`'s order, then chain order. A blend's `round_<n>`/`bevel_<n>` faces are named by that index
/// ([`crate::facade::names::Naming::blend_seeds`]).
pub fn chained(k: &DefaultKernel, body: BodyId, ix: &[u64]) -> Result<Vec<(u64, usize)>> {
    let info = k.edges(body)?;
    let seam = |e: u64| info.iter().any(|i| i.id.0 == e && i.faces[0].is_some() && i.faces[0] == i.faces[1]);
    let mut out: Vec<(u64, usize)> = Vec::new();
    for (n, &e) in ix.iter().enumerate() {
        if seam(e) || out.iter().any(|&(x, _)| x == e) {
            continue;
        }
        for c in k.tangent_chain(body, EdgeId(e))? {
            if !seam(c.0) && !out.iter().any(|&(x, _)| x == c.0) {
                out.push((c.0, n));
            }
        }
    }
    Ok(out)
}

fn chain_ids(k: &DefaultKernel, body: BodyId, ix: &[u64]) -> Result<Vec<EdgeId>> {
    Ok(chained(k, body, ix)?.into_iter().map(|(e, _)| EdgeId(e)).collect())
}

/// Edges `ix` (and their tangent chains) rounded with `radius`.
pub fn fillet(k: &mut DefaultKernel, body: BodyId, ix: &[u64], radius: f64) -> Result<Made> {
    let es = chain_ids(k, body, ix)?;
    made(k.fillet(body, &es, radius))
}

/// Edges `ix` (and their tangent chains) chamfered with `opts`.
fn chamfer_opts(k: &mut DefaultKernel, body: BodyId, ix: &[u64], opts: &ChamferOpts) -> Result<Made> {
    let es = chain_ids(k, body, ix)?;
    made(k.chamfer_with(body, &es, opts))
}

/// Edges `ix` (and their tangent chains) chamfered `distance` along each face; where three or
/// more chamfers meet, the corner cut flat (as cadrs's runner asks).
pub fn chamfer(k: &mut DefaultKernel, body: BodyId, ix: &[u64], distance: f64) -> Result<Made> {
    let mut opts = ChamferOpts::new(ChamferSpec::EqualDistance(distance));
    opts.corners = ChamferCorners::Flat;
    chamfer_opts(k, body, ix, &opts)
}

/// The tangent-plane distance that lands a straight chord `chord` from the edge on face `f` of
/// `body` (cadrs's `ChamferMeasure::Tangent`): on a ball of radius R, `R tan(2 asin(c / 2R))`;
/// on a face straight across the edge (a plane, a cylinder's or cone's rim) the chord itself.
fn tangent_of_chord(body: &crate::native::topo::Body, f: u64, chord: f64) -> Result<f64> {
    use crate::native::surface::Surface;
    let face = usize::try_from(f).ok().and_then(|i| body.faces.get(i));
    match face.map(|f| &f.surf) {
        Some(Surface::Sphere { r, .. }) => {
            let s = chord / (2.0 * r);
            if !(s < 1.0) {
                return Err(KernelError::InvalidInput("a chamfer chord as long as the ball is wide".into()));
            }
            Ok(r * (2.0 * s.asin()).tan())
        }
        _ => Ok(chord),
    }
}

/// Edge `e` (and its tangent chain) chamfered `d1` along face `face` and `d2` along the other,
/// each a straight chord from the edge in the plane square to it.
pub fn chamfer_two(k: &mut DefaultKernel, body: BodyId, e: u64, face: u64, d1: f64, d2: f64) -> Result<Made> {
    let info = k.edges(body)?;
    let Some(i) = info.iter().find(|i| i.id.0 == e) else {
        return Err(KernelError::InvalidInput("no such edge".into()));
    };
    let other = i.faces.iter().flatten().map(|f| f.0).find(|&f| f != face).unwrap_or(face);
    let (t1, t2) = {
        let b = k.get(body)?;
        (tangent_of_chord(b, face, d1)?, tangent_of_chord(b, other, d2)?)
    };
    let mut opts = ChamferOpts::new(ChamferSpec::TwoDistances(t1, t2)).first_on(FaceId(face));
    opts.measurement = ChamferMeasure::Tangent;
    chamfer_opts(k, body, &[e], &opts)
}

/// Edge `e` (and its tangent chain) chamfered `distance` along face `face`, the bevel at `angle`
/// radians to that face.
pub fn chamfer_angle(k: &mut DefaultKernel, body: BodyId, e: u64, face: u64, distance: f64, angle: f64) -> Result<Made> {
    let opts = ChamferOpts::new(ChamferSpec::DistanceAngle { distance, angle }).first_on(FaceId(face));
    chamfer_opts(k, body, &[e], &opts)
}

/// The solid torus about the line through `center` along `axis` (horn and spindle tori too).
pub fn torus(k: &mut DefaultKernel, center: [f64; 3], axis: [f64; 3], major: f64, minor: f64) -> Result<Made> {
    if v3(axis).norm() == 0.0 {
        return Err(KernelError::InvalidInput("a torus's axis is zero".into()));
    }
    let p = Primitive::Torus { center: p3(center), axis: nalgebra::Unit::new_normalize(v3(axis)), major, minor };
    made(k.primitive(&p))
}

// ---- variable fillets and splits (as cadrs's conformance adapter maps them) ----

use crate::{FilletLaw, Plane, SplitTool};

/// Edge `e` rounded with a radius running from `r_from` at its end nearer `from` to `r_to` at
/// the other.
pub fn fillet_variable(k: &mut DefaultKernel, body: BodyId, e: u64, from: [f64; 3], r_from: f64, r_to: f64) -> Result<Made> {
    let info = k.edges(body)?;
    let Some(i) = info.iter().find(|i| i.id.0 == e) else {
        return Err(KernelError::InvalidInput("no such edge".into()));
    };
    let from = p3(from);
    // cadrs gives the radii from the edge's start.
    let radii = if (i.start - from).norm() <= (i.end - from).norm() {
        vec![(0.0, r_from), (1.0, r_to)]
    } else {
        vec![(0.0, r_to), (1.0, r_from)]
    };
    made(k.fillet_variable(body, &[FilletLaw { edge: EdgeId(e), radii }], true))
}

/// `body` split by the plane through `origin` square to `normal`: the pieces on the normal's
/// side (front), then the rest (back), each `None` when there are none, several pieces of a side
/// gathered in one body. Splitting renumbers the faces, so the pieces carry no history.
pub fn split(k: &mut DefaultKernel, body: BodyId, origin: [f64; 3], normal: [f64; 3]) -> Result<(Option<Made>, Option<Made>)> {
    let n = v3(normal);
    if n.norm() == 0.0 {
        return Err(KernelError::InvalidInput("a split plane's normal is zero".into()));
    }
    let x = if n.x.abs() < 0.9 * n.norm() { Vector3::x() } else { Vector3::y() };
    let o = p3(origin);
    let cut = k.split(body, &SplitTool::Plane(Plane::new(o, x, n)), 0)?.first()?;
    let pieces = k.split_solids(cut)?;
    let (mut front, mut back) = (Vec::new(), Vec::new());
    for piece in pieces {
        let piece = piece.first()?;
        let c = k.mass_properties(piece)?.center_of_mass;
        if (c - o).dot(&n) >= 0.0 { front.push(piece) } else { back.push(piece) }
    }
    k.release(cut);
    let mut gather = |bodies: Vec<BodyId>| -> Result<Option<Made>> {
        match bodies.as_slice() {
            [] => Ok(None),
            [b] => Ok(Some((*b, History::default()))),
            _ => {
                let all = k.compound(&bodies)?.first()?;
                Ok(Some((all, History::default())))
            }
        }
    };
    Ok((gather(front)?, gather(back)?))
}
