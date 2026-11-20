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

/// The most turns a screw [`cut_swept`] takes on at all (a runaway bound).
pub const SCREW_TURNS_MAX: f64 = 64.0;

/// cadrs's motion for `m`.
fn solid_motion(m: &Move) -> SolidMotion {
    match *m {
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
    }
}

/// Whether cadrs made `body` only as a mesh (a thousand flat facets and no curved face: its
/// contouring fallback), not as the solid's own faces.
fn faceted(k: &DefaultKernel, body: BodyId) -> Result<bool> {
    let b = k.get(body)?;
    let flat = b.faces.iter().filter(|f| matches!(f.surf, crate::native::surface::Surface::Plane { .. })).count();
    Ok(flat == b.faces.len() && flat > 1000)
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
    // A screw past a whole turn sweeps over itself; cadrs makes it only as a mesh-bounded solid
    // (seconds, tens of thousands of facets), and the union of its half-turn legs measured 0.1 %
    // off the exact cut (fine-thread: 1278.13 against 1276.70). Refused: cut by it with
    // [`cut_swept`] (cadrs's own cut by a moving tool, exact there).
    if let Move::Screw { angle, .. } = *m
        && angle.abs() > std::f64::consts::TAU
    {
        return Err(KernelError::Unsupported("a screw past a whole turn (cut by it with cut_swept)"));
    }
    let (out, history) = made(k.sweep_solid(body, &solid_motion(m)))?;
    if faceted(k, out)? {
        k.release(out);
        return Err(KernelError::Unsupported("cadrs sweeps this body only as a mesh of flat facets"));
    }
    Ok((out, history))
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

use crate::{ChamferCorners, ChamferMeasure, ChamferOpts, ChamferSpec, DraftSpec, Origin, EdgeId, FaceId, FaceInput, Motion, OffsetSpec, Primitive, ShellSpec, ThickenSpec};

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

// ---- primitives placed in a yack frame ----

/// A primitive in its own frame (origin, x, y, z): the block from the origin along +x, +y, +z;
/// the cylinder and cone on the z axis from the origin up `height`; the ball about the origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Prim {
    /// The block `size[0]` × `size[1]` × `size[2]`.
    #[allow(missing_docs)]
    Block { size: [f64; 3] },
    /// The cylinder of `radius`, `height` tall.
    #[allow(missing_docs)]
    Cylinder { radius: f64, height: f64 },
    /// The ball of `radius`.
    #[allow(missing_docs)]
    Sphere { radius: f64 },
    /// The cone (or frustum) of radius `bottom` at z = 0 and `top` at z = `height`.
    #[allow(missing_docs)]
    Cone { bottom: f64, top: f64, height: f64 },
}

/// The curve ids the history of [`primitive`] names the sides by (as `ProfileCurve` origins, so a
/// [`crate::facade::names::Naming`] with `segments` in this order names them): a block's four
/// sides, then a cylinder's or cone's side, then a ball's skin.
pub const PRIM_SIDES: [&str; 6] = ["x_min", "x_max", "y_min", "y_max", "side", "skin"];

/// `p` built in `frame` (orthonormal and right-handed): made axis-aligned at
/// the world origin, then moved rigidly into the frame. Its history: the face at z = 0 is the
/// start cap, the one at z = height the end cap, the others `ProfileCurve { region: 0, curve }`
/// with `curve` an index into [`PRIM_SIDES`].
pub fn primitive(k: &mut DefaultKernel, frame: &crate::facade::profile::Place, p: Prim) -> Result<Made> {
    let up = nalgebra::Unit::new_unchecked(Vector3::z());
    let prim = match p {
        Prim::Block { size } => Primitive::Box { min: Point3::origin(), max: Point3::new(size[0], size[1], size[2]) },
        Prim::Cylinder { radius, height } => Primitive::Cylinder { base: Point3::origin(), axis: up, radius, height },
        Prim::Sphere { radius } => Primitive::Sphere {
            center: Point3::origin(),
            radius,
            axis: up,
            seam: nalgebra::Unit::new_unchecked(Vector3::x()),
        },
        Prim::Cone { bottom, top, height } => {
            Primitive::Cone { base: Point3::origin(), axis: up, base_radius: bottom, top_radius: top, height }
        }
    };
    let at_origin = k.primitive(&prim)?.first()?;
    // Name each face from where it lies before the move (the move keeps face indices).
    let mut history = History::default();
    {
        use crate::native::surface::Surface;
        let b = k.get(at_origin)?;
        for (i, f) in b.faces.iter().enumerate() {
            let id = FaceId(i as u64);
            let origin = match &f.surf {
                Surface::Plane { f: fr } => {
                    let n = fr.z * if f.sense { 1.0 } else { -1.0 };
                    if n.z < -0.5 {
                        Origin::StartCap { region: 0 }
                    } else if n.z > 0.5 {
                        Origin::EndCap { region: 0 }
                    } else if n.x < -0.5 {
                        Origin::ProfileCurve { region: 0, curve: 0 }
                    } else if n.x > 0.5 {
                        Origin::ProfileCurve { region: 0, curve: 1 }
                    } else if n.y < -0.5 {
                        Origin::ProfileCurve { region: 0, curve: 2 }
                    } else {
                        Origin::ProfileCurve { region: 0, curve: 3 }
                    }
                }
                Surface::Sphere { .. } => Origin::ProfileCurve { region: 0, curve: 5 },
                _ => Origin::ProfileCurve { region: 0, curve: 4 },
            };
            history.generated.push((id, origin));
        }
    }
    let [o, x, y, z] = [frame.origin, frame.x, frame.y, frame.z].map(v3);
    let motion = Motion { linear: nalgebra::Matrix3::from_columns(&[x, y, z]), translation: o };
    let r = k.transform_motion(at_origin, &motion);
    k.release(at_origin);
    Ok((r?.first()?, history))
}

// ---- cutting by a moving tool ----

/// `tool`'s part within reach of `target` under a turn or screw about the line through `point`
/// along `axis`: such a motion keeps every point's distance to the axis, so tool points farther
/// out than all of `target` never meet it. `tool` itself when all of it is within reach (or when
/// trimming fails: the cut is the same, only slower).
fn near_axis(k: &mut DefaultKernel, tool: BodyId, target: BodyId, point: [f64; 3], axis: [f64; 3]) -> Result<BodyId> {
    let a = v3(axis).normalize();
    let o = p3(point);
    let corners = |bb: crate::Aabb| -> Vec<Point3<f64>> {
        (0..8).map(|i| {
            Point3::new(
                if i & 1 == 0 { bb.min.x } else { bb.max.x },
                if i & 2 == 0 { bb.min.y } else { bb.max.y },
                if i & 4 == 0 { bb.min.z } else { bb.max.z },
            )
        })
        .collect()
    };
    let radial = |p: Point3<f64>| {
        let d = p - o;
        (d - a * d.dot(&a)).norm()
    };
    let reach = corners(k.bounding_box(target)?).into_iter().map(radial).fold(0.0, f64::max);
    let tb = k.bounding_box(tool)?;
    if corners(tb).into_iter().map(radial).fold(0.0, f64::max) <= reach {
        return Ok(tool);
    }
    let (s0, s1) = corners(tb).into_iter().map(|p| (p - o).dot(&a)).fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), s| (l.min(s), h.max(s)));
    // Room to spare on every side: a hundredth of the sizes involved.
    let pad = 0.01 * (reach + (s1 - s0));
    let base = o + a * (s0 - pad);
    let cylinder = Primitive::Cylinder { base, axis: nalgebra::Unit::new_normalize(a), radius: reach + pad, height: s1 - s0 + 2.0 * pad };
    let c = k.primitive(&cylinder)?.first()?;
    let r = k.boolean(crate::BoolOp::Intersect, tool, &[c]);
    k.release(c);
    match r.and_then(|r| r.first()) {
        Ok(t) => Ok(t),
        Err(_) => Ok(tool),
    }
}

/// `target` less everything `tool` passes through as it moves by `m` (cadrs's cut by a moving
/// tool, which builds what it removes a stretch at a time). Under a turn or screw only the tool's
/// part within the target's reach of the axis is moved.
fn cut_swept_one(k: &mut DefaultKernel, target: BodyId, tool: BodyId, m: &Move) -> Result<Made> {
    if let Move::Screw { angle, .. } = *m
        && angle.abs() > SCREW_TURNS_MAX * std::f64::consts::TAU
    {
        return Err(KernelError::Unsupported("a screw of more than 64 turns"));
    }
    let used = match *m {
        Move::Turn { point, axis, .. } | Move::Screw { point, axis, .. } => near_axis(k, tool, target, point, axis)?,
        _ => tool,
    };
    // cadrs's cut by a moving tool is exact for a screw (fine-thread: 1276.703 against cadrs's
    // runner's 1276.703) but not for a turn (pin-swung-through-ring-sector: 53.931 against the
    // exact 53.843, MEASURED natively): there the sweep, exact for one turn, is cut instead.
    let r = match *m {
        Move::Screw { .. } => k.cut_swept(target, used, &solid_motion(m)),
        _ => swept(k, used, m).and_then(|(sw, _)| {
            let r = k.boolean(crate::BoolOp::Subtract, target, &[sw]);
            k.release(sw);
            r
        }),
    };
    if used != tool {
        k.release(used);
    }
    let (out, history) = made(r)?;
    if faceted(k, out)? {
        k.release(out);
        return Err(KernelError::Unsupported("cadrs cuts by this moving tool only as a mesh of flat facets"));
    }
    Ok((out, history))
}

/// `target` less everything `tool` passes through as it moves by each of `moves` in turn (one:
/// a single motion; several: legs, each from where the one before left the tool): the target cut
/// by each leg in order. The history is the last cut's (for a single move, the target's faces
/// kept or modified, as a boolean's); Unsupported where cadrs would make the cut only as a mesh.
pub fn cut_swept(k: &mut DefaultKernel, target: BodyId, tool: BodyId, moves: &[Move]) -> Result<Made> {
    let Some((last, before)) = moves.split_last() else {
        return Err(KernelError::InvalidInput("a cut by no motion".into()));
    };
    let mut made_here: Vec<BodyId> = Vec::new();
    let r = (|| {
        let (mut cur, mut at) = (target, tool);
        for m in before {
            let (next, _) = cut_swept_one(k, cur, at, m)?;
            made_here.push(next);
            at = k.transform_motion(at, &crate::facade::motion::end_motion(m)?)?.first()?;
            made_here.push(at);
            cur = next;
        }
        cut_swept_one(k, cur, at, last)
    })();
    for id in made_here {
        if r.as_ref().map_or(true, |(out, _): &Made| *out != id) {
            k.release(id);
        }
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Pick;
    use nalgebra::Point3;

    fn cube(k: &mut DefaultKernel) -> BodyId {
        k.primitive(&Primitive::Box { min: Point3::new(0.0, 0.0, 0.0), max: Point3::new(10.0, 10.0, 10.0) })
            .and_then(|r| r.first())
            .unwrap()
    }

    #[test]
    fn a_split_sorts_its_sides() {
        let mut k = DefaultKernel::new();
        let b = cube(&mut k);
        let (front, back) = split(&mut k, b, [0.0, 0.0, 4.0], [0.0, 0.0, 1.0]).unwrap();
        let v = |k: &DefaultKernel, m: Option<Made>| k.mass_properties(m.unwrap().0).unwrap().volume;
        assert!((v(&k, front) - 600.0).abs() < 1e-9);
        assert!((v(&k, back) - 400.0).abs() < 1e-9);
        let (front, back) = split(&mut k, b, [0.0, 0.0, 20.0], [0.0, 0.0, 1.0]).unwrap_or((None, None));
        assert!(front.is_none() || back.is_none());
    }

    #[test]
    fn a_chamfer_of_every_edge_cuts_its_corners_flat() {
        let mut k = DefaultKernel::new();
        let b = cube(&mut k);
        let all: Vec<u64> = (0..k.edges(b).unwrap().len() as u64).collect();
        let (c, _) = chamfer(&mut k, b, &all, 1.0).unwrap();
        // 6 faces, 12 bevels, 8 flat corner triangles (as cadrs's runner's Flat corners).
        assert_eq!(k.get(c).unwrap().faces.len(), 26);
    }

    #[test]
    fn a_shell_is_sharp_and_a_rounded_shell_rounds() {
        let mut k = DefaultKernel::new();
        let b = cube(&mut k);
        let top = k.face_at(b, Point3::new(5.0, 5.0, 10.0)).unwrap().0;
        let (s, _) = shell(&mut k, b, &[top], 1.0, true).unwrap();
        // Sharp outward: the 12 × 12 × 11 box less the 10 × 10 × 10 cavity, open at the top.
        let v = k.mass_properties(s).unwrap().volume;
        assert!((v - (12.0 * 12.0 * 11.0 - 1000.0)).abs() < 1e-6, "{v}");
        let (r, _) = shell_rounded(&mut k, b, &[top], 1.0, true).unwrap();
        assert!(k.mass_properties(r).unwrap().volume < v);
    }

    #[test]
    fn a_fillet_takes_the_whole_tangent_chain() {
        let mut k = DefaultKernel::new();
        let b = k
            .primitive(&Primitive::cylinder(Point3::origin(), nalgebra::Vector3::z(), 5.0, 10.0))
            .and_then(|r| r.first())
            .unwrap();
        let rim = k.edge_at(b, Point3::new(5.0, 0.0, 10.0)).unwrap().0;
        let chain = chained(&k, b, &[rim]).unwrap();
        assert!(chain.iter().all(|&(_, n)| n == 0) && chain.iter().any(|&(e, _)| e == rim));
        assert!(fillet(&mut k, b, &[rim], 1.0).is_ok());
    }

    #[test]
    fn primitives_land_in_their_frame_with_caps_named() {
        let mut k = DefaultKernel::new();
        // A frame at (10, 0, 0) turned a quarter about z: its x is world y.
        let frame = &crate::facade::profile::Place { origin: [10.0, 0.0, 0.0], x: [0.0, 1.0, 0.0], y: [-1.0, 0.0, 0.0], z: [0.0, 0.0, 1.0] };
        let (b, h) = primitive(&mut k, frame, Prim::Block { size: [4.0, 2.0, 3.0] }).unwrap();
        let m = k.mass_properties(b).unwrap();
        assert!((m.volume - 24.0).abs() < 1e-9);
        // Centre: origin + 2 x + 1 y + 1.5 z = (10 - 1, 2, 1.5).
        assert!((m.center_of_mass - Point3::new(9.0, 2.0, 1.5)).norm() < 1e-9, "{:?}", m.center_of_mass);
        let caps = |h: &History| {
            (
                h.generated.iter().filter(|(_, o)| matches!(o, Origin::StartCap { .. })).count(),
                h.generated.iter().filter(|(_, o)| matches!(o, Origin::EndCap { .. })).count(),
            )
        };
        assert_eq!(caps(&h), (1, 1));
        assert_eq!(h.generated.len(), 6);
        let (c, h) = primitive(&mut k, frame, Prim::Cone { bottom: 3.0, top: 1.0, height: 2.0 }).unwrap();
        let v = k.mass_properties(c).unwrap().volume;
        assert!((v - std::f64::consts::PI * 2.0 / 3.0 * (9.0 + 3.0 + 1.0)).abs() < 1e-9, "{v}");
        assert_eq!(caps(&h), (1, 1));
        let (s, h) = primitive(&mut k, frame, Prim::Sphere { radius: 2.0 }).unwrap();
        assert!((k.mass_properties(s).unwrap().center_of_mass - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-9);
        assert!(h.generated.iter().all(|(_, o)| matches!(o, Origin::ProfileCurve { curve: 5, .. })));
        let (cy, h) = primitive(&mut k, frame, Prim::Cylinder { radius: 1.0, height: 5.0 }).unwrap();
        assert!((k.mass_properties(cy).unwrap().volume - 5.0 * std::f64::consts::PI).abs() < 1e-9);
        assert_eq!(caps(&h), (1, 1));
    }

    #[test]
    fn a_fine_thread_is_cut_in_time() {
        let mut k = DefaultKernel::new();
        let rod = k
            .primitive(&Primitive::cylinder(Point3::origin(), nalgebra::Vector3::z(), 5.0, 20.0))
            .and_then(|r| r.first())
            .unwrap();
        let bar = k
            .primitive(&Primitive::Box { min: Point3::new(4.5, -8.0, -0.45), max: Point3::new(12.0, 8.0, 0.45) })
            .and_then(|r| r.first())
            .unwrap();
        // Three turns (the bench case's 21 take seconds; this test stays quick).
        let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 3.0 * std::f64::consts::TAU, rise_per_turn: 1.0 };
        let (cut, _) = cut_swept(&mut k, rod, bar, &[m]).unwrap();
        let v = k.mass_properties(cut).unwrap().volume;
        assert!(v > 0.0 && v < 500.0 * std::f64::consts::PI, "{v}");
        assert!(!faceted(&k, cut).unwrap());
    }

    #[test]
    fn a_pin_swung_through_a_ring_sector_cuts_exactly() {
        let mut k = DefaultKernel::new();
        let spec = crate::CylindricalBox {
            frame: crate::Plane::new(Point3::new(0.0, 0.0, -1.0), nalgebra::Vector3::x(), nalgebra::Vector3::z()),
            length: 2.0,
            inner_radius: 8.0,
            outer_radius: 12.0,
            start_angle: 0.0,
            end_angle: 60f64.to_radians(),
        };
        let block = k.cylindrical_box(&spec).and_then(|r| r.first()).unwrap();
        let pin = k
            .primitive(&Primitive::cylinder(Point3::new(6.0, 0.0, 0.0), nalgebra::Vector3::x(), 0.5, 5.0))
            .and_then(|r| r.first())
            .unwrap();
        let start = k.transform_motion(pin, &Motion::rotation(&Axis::new(Point3::origin(), nalgebra::Vector3::z()), (-140f64).to_radians())).and_then(|r| r.first()).unwrap();
        let m = Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 280f64.to_radians() };
        let (cut, _) = cut_swept(&mut k, block, start, &[m]).unwrap();
        // cadrs's runner's (sweep, then cut) volume; cadrs's own cut_swept gives 53.931 here.
        let v = k.mass_properties(cut).unwrap().volume;
        assert!((v - 53.843407424025).abs() < 1e-6, "{v}");
    }
}
