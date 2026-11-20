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
    // A screw past a whole turn: cadrs sweeps a ball, a box or a torus exactly (as cadrs's runner,
    // helical-pipe-round-ends, box-screwed-inside-a-rod); where it cannot it falls back to a
    // mesh-bounded solid, refused below. A screw of more turns than any part has is a runaway.
    if let Move::Screw { angle, .. } = *m
        && angle.abs() > SCREW_TURNS_MAX * std::f64::consts::TAU
    {
        return Err(KernelError::Unsupported("a screw of more than 64 turns"));
    }
    let (out, history) = made(k.sweep_solid(body, &solid_motion(m)))?;
    if faceted(k, out)? {
        k.release(out);
        return Err(KernelError::Unsupported("cadrs sweeps this body only as a mesh of flat facets"));
    }
    Ok((out, history))
}

/// The unit normal of the plane holding every control point, or `None` when they are collinear
/// or not coplanar (within `tol` relative to their spread). The plane is the farthest point's
/// direction and the direction of the point farthest from that line (an S-shaped path's control
/// polygon encloses no area, so its area vector would say nothing); its sign makes the largest
/// component positive.
pub fn plane_normal(ctrl: &[[f64; 3]], tol: f64) -> Option<[f64; 3]> {
    let pts: Vec<Point3<f64>> = ctrl.iter().map(|&c| p3(c)).collect();
    let first = *pts.first()?;
    let far = pts.iter().copied().max_by(|a, b| (a - first).norm().total_cmp(&(b - first).norm()))?;
    let spread = (far - first).norm();
    if spread == 0.0 {
        return None;
    }
    let d = (far - first) / spread;
    let off = |p: &Point3<f64>| (p - first) - d * (p - first).dot(&d);
    let side = pts.iter().map(off).max_by(|a, b| a.norm().total_cmp(&b.norm()))?;
    if side.norm() <= tol * spread {
        return None;
    }
    let mut n = d.cross(&side.normalize());
    if pts.iter().any(|p| (p - first).dot(&n).abs() > tol * spread) {
        return None;
    }
    if n.iter().copied().max_by(|a, b| a.abs().total_cmp(&b.abs())).is_some_and(|c| c < 0.0) {
        n = -n;
    }
    Some([n.x, n.y, n.z])
}

/// The `up` of [`Move::Along`] that carries `body` along the path on control points `ctrl` as
/// the protocol's frame law does (`fixed`: the body keeps its orientation; else the path's
/// rotation-minimising frame), or why cadrs cannot. On a planar path the rotation-minimising
/// frame turns about the plane's normal with the tangent: `up` is that normal. A fixed frame is
/// the same motion where turning about that normal through the path's start leaves the body as
/// it is: a ball, or a body of revolution about that line (a mill on its own axis).
pub fn along_up(k: &DefaultKernel, body: BodyId, ctrl: &[[f64; 3]], fixed: bool, tol: f64) -> Result<[f64; 3]> {
    let n = plane_normal(ctrl, tol);
    let faces = &k.get(body)?.faces;
    if fixed {
        if let [f] = faces.as_slice()
            && matches!(f.surf, crate::native::surface::Surface::Sphere { .. })
        {
            return Ok(n.unwrap_or([0.0, 0.0, 1.0]));
        }
        let start = ctrl.first().map(|&c| p3(c));
        let on_line = |n: [f64; 3]| start.is_some_and(|s| of_revolution_about(faces, s, v3(n)));
        return match n {
            Some(n) if on_line(n) => Ok(n),
            _ => Err(KernelError::Unsupported("a fixed-frame path sweep of a solid other than a ball or a body turned about the path's normal at its start")),
        };
    }
    n.ok_or(KernelError::Unsupported("a rotation-minimising sweep along a path that is not planar"))
}

// ---- ops by face and edge index (the bridge keeps springrep's indices) ----

use crate::{ChamferCorners, ChamferMeasure, ChamferOpts, ChamferSpec, DraftSpec, Origin, EdgeId, EdgeInfo, FaceId, FaceInput, Motion, OffsetSpec, Primitive, ShellSpec, ThickenSpec};

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

/// The protocol's fit rule for a blend of `es` on `body` made as `out` ("the ball fits when its
/// contact with each face stays on that face"): every old face a new face of the blend meets must
/// continue a face at one of the blended edges or at one of their ends. A blend whose contact runs
/// off its face onto a third one (a hole's rim round wider than the plate round it) is declined:
/// cadrs trims it by that face and answers a solid that is not the protocol's.
fn fits(k: &DefaultKernel, body: BodyId, es: &[EdgeId], out: &Made) -> Result<()> {
    let info = k.edges(body)?;
    let named: Vec<&EdgeInfo> = info.iter().filter(|i| es.contains(&i.id)).collect();
    let near = |a: &Point3<f64>, b: &Point3<f64>| (a - b).norm() <= crate::native::tol::POINT;
    let ends: Vec<Point3<f64>> = named.iter().flat_map(|i| [i.start, i.end]).collect();
    let mut allowed: Vec<FaceId> = Vec::new();
    for i in &info {
        let at_end = [i.start, i.end].iter().any(|p| ends.iter().any(|q| near(p, q)));
        if es.contains(&i.id) || at_end {
            allowed.extend(i.faces.iter().flatten().copied());
        }
    }
    let (made_body, history) = out;
    let new = |f: FaceId| history.generated.iter().any(|&(g, _)| g == f);
    let old = |f: FaceId| -> Option<FaceId> {
        history.modified.iter().find(|&&(g, _)| g == f).map(|&(_, from)| from.face)
    };
    for e in k.edges(*made_body)? {
        let [Some(a), Some(b)] = e.faces else { continue };
        for (n, o) in [(a, b), (b, a)] {
            if !new(n) || new(o) {
                continue;
            }
            match old(o) {
                Some(f) if allowed.contains(&f) => {}
                _ => {
                    return Err(KernelError::InvalidInput(
                        "the blend does not fit: its contact runs off a face it rounds onto another face".into(),
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Edges `ix` (and their tangent chains) rounded with `radius`.
pub fn fillet(k: &mut DefaultKernel, body: BodyId, ix: &[u64], radius: f64) -> Result<Made> {
    let es = chain_ids(k, body, ix)?;
    let out = made(k.fillet(body, &es, radius))?;
    fits(k, body, &es, &out)?;
    Ok(out)
}

/// Edges `ix` (and their tangent chains) chamfered with `opts`.
fn chamfer_opts(k: &mut DefaultKernel, body: BodyId, ix: &[u64], opts: &ChamferOpts) -> Result<Made> {
    let es = chain_ids(k, body, ix)?;
    let out = made(k.chamfer_with(body, &es, opts))?;
    fits(k, body, &es, &out)?;
    Ok(out)
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
/// Whether `faces` are all of revolution about one axis (spheres, tori, cones and cylinders on
/// it, planes square to it): a body cadrs sweeps by a turn or screw by its envelope.
fn of_revolution(faces: &[crate::native::topo::Face]) -> bool {
    use crate::native::surface::Surface as S;
    let axis = faces.iter().find_map(|f| match &f.surf {
        S::Cylinder { f, .. } | S::Cone { f, .. } | S::Torus { f, .. } => Some((f.o, f.z)),
        _ => None,
    });
    match axis {
        Some((o, a)) => of_revolution_about(faces, o, a),
        // A ball alone.
        None => matches!(faces, [f] if matches!(f.surf, S::Sphere { .. })),
    }
}

/// Whether `faces` are all of revolution about the line through `o` along `a`.
fn of_revolution_about(faces: &[crate::native::topo::Face], o: Point3<f64>, a: Vector3<f64>) -> bool {
    use crate::native::surface::Surface as S;
    let Some(a) = a.try_normalize(0.0) else { return false };
    let on_axis = |p: Point3<f64>| (p - o).cross(&a).norm() <= crate::native::tol::POINT;
    let parallel = |z: Vector3<f64>| z.cross(&a).norm() <= crate::native::tol::ANGULAR;
    !faces.is_empty()
        && faces.iter().all(|f| match &f.surf {
            S::Cylinder { f, .. } | S::Cone { f, .. } | S::Torus { f, .. } | S::Revolution { f, .. } => on_axis(f.o) && parallel(f.z),
            S::Sphere { f, .. } => on_axis(f.o),
            S::Plane { f } => parallel(f.z),
            _ => false,
        })
}

fn cut_swept_one(k: &mut DefaultKernel, target: BodyId, tool: BodyId, m: &Move) -> Result<Made> {
    if let Move::Screw { angle, .. } = *m
        && angle.abs() > SCREW_TURNS_MAX * std::f64::consts::TAU
    {
        return Err(KernelError::Unsupported("a screw of more than 64 turns"));
    }
    // A body of revolution is left whole: cadrs cuts by a turned or screwed one exactly, by its
    // envelope (torus-screwed-through-rod: 4 faces in 0.1 s, v-wheel-screwed-through-rod), and
    // by a trimmed piece of one only as a mesh.
    let whole = k.get(tool).is_ok_and(|b| of_revolution(&b.faces));
    let used = match *m {
        Move::Turn { point, axis, .. } | Move::Screw { point, axis, .. } if !whole => near_axis(k, tool, target, point, axis)?,
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

// ---- booleans with tolerances scaled to the operands' smallest features ----

/// The shortest edge of `bodies` (edges of no length, at poles and apexes, left out).
fn shortest_edge(k: &DefaultKernel, bodies: &[BodyId]) -> Result<f64> {
    let mut least = f64::INFINITY;
    for &b in bodies {
        for e in k.edges(b)? {
            if e.length > 0.0 {
                least = least.min(e.length);
            }
        }
    }
    Ok(least)
}

/// `a` combined with `tools` by `how`, cadrs's boolean. Its merge distances never reach a
/// feature of the operands (D24): where an edge is shorter than a hundred times the session's
/// widest retry, the session runs this boolean with every tolerance a hundredth of that edge
/// and below (a saw cut a millionth wide stays a cut). An empty intersection of solids whose
/// boundaries meet is refused, not answered (cadrs merges a lens thinner than its tolerance
/// away).
pub fn boolean(k: &mut DefaultKernel, how: crate::BoolOp, a: BodyId, tools: &[BodyId]) -> Result<Made> {
    let mut all = vec![a];
    all.extend_from_slice(tools);
    let base = k.tolerances();
    let least = shortest_edge(k, &all)?;
    let scaled = least.is_finite() && least < 100.0 * base.retries[1];
    if scaled {
        let point = base.point.min(least / 100.0);
        let tol = crate::native::tol::Tolerances {
            confusion: base.confusion.min(point / 10.0),
            point,
            retries: [base.retries[0].min(least / 30.0).max(point), base.retries[1].min(least / 10.0).max(point)],
        };
        k.set_tolerances(tol)?;
    }
    let r = k.boolean(how, a, tools);
    if scaled {
        k.set_tolerances(base)?;
    }
    let r = match r {
        Err(KernelError::EmptyResult(why)) if matches!(how, crate::BoolOp::Intersect) => {
            // Nothing in common: right only where the solids stay apart.
            for &t in tools {
                let d = k.min_distance(&crate::DistanceTarget::Body(a), &crate::DistanceTarget::Body(t))?;
                if d.distance <= base.point {
                    return Err(KernelError::Unsupported(
                        "an empty intersection of solids whose boundaries meet (thinner than cadrs's tolerance?)",
                    ));
                }
            }
            return Err(KernelError::EmptyResult(why));
        }
        r => r,
    };
    made(r)
}

// ---- slabs and transforms ----

/// The degree of the clamped knots `knots` over `n` coefficients and their domain.
fn clamped(knots: &[f64], n: usize) -> Result<(usize, (f64, f64))> {
    let p = knots
        .len()
        .checked_sub(n + 1)
        .filter(|&p| p >= 1)
        .ok_or_else(|| KernelError::InvalidInput("a slab needs more knots than coefficients plus one each way".into()))?;
    let (lo, hi) = (knots[p], knots[n]);
    if knots[..=p].iter().any(|&k| k != lo) || knots[n..].iter().any(|&k| k != hi) || !(hi > lo) {
        return Err(KernelError::InvalidInput("a slab's knots must be clamped at both ends".into()));
    }
    Ok((p, (lo, hi)))
}

/// The Greville abscissae of the clamped knots: where each coefficient sits so the spline
/// reproduces a linear map.
fn greville(knots: &[f64], p: usize, n: usize) -> Vec<f64> {
    (0..n).map(|i| knots[i + 1..=i + p].iter().sum::<f64>() / p as f64).collect()
}

/// yack's slab, as cadrs's runner builds it (`domed_block`): the solid under
/// `z = f(x / size[0], y / size[1])` in `frame`, `f` the B-spline on the knots with coefficients
/// `heights[i][j]` at their Greville abscissae (so x and y run linearly with the parameters); a
/// block over the rectangle, its top face replaced by the surface (Replace face).
pub fn slab(
    k: &mut DefaultKernel,
    frame: &crate::facade::profile::Place,
    size: [f64; 2],
    knots_u: &[f64],
    knots_v: &[f64],
    heights: &[Vec<f64>],
) -> Result<Made> {
    let nu = heights.len();
    let nv = heights.first().map_or(0, Vec::len);
    if nu == 0 || nv == 0 || heights.iter().any(|r| r.len() != nv) {
        return Err(KernelError::InvalidInput("a slab's heights must form a grid".into()));
    }
    if !(size[0] > 0.0 && size[1] > 0.0) || heights.iter().flatten().any(|&h| !(h > 0.0 && h.is_finite())) {
        return Err(KernelError::InvalidInput("a slab's size and heights must be positive".into()));
    }
    let (pu, (u0, u1)) = clamped(knots_u, nu)?;
    let (pv, (v0, v1)) = clamped(knots_v, nv)?;
    let (gu, gv) = (greville(knots_u, pu, nu), greville(knots_v, pv, nv));
    let [a, b] = size;
    let (o, x, y, z) = (v3(frame.origin), v3(frame.x), v3(frame.y), v3(frame.z));
    let world = |px: f64, py: f64, pz: f64| o + x * px + y * py + z * pz;
    let points: Vec<Vec<[f64; 3]>> = (0..nu)
        .map(|i| {
            (0..nv)
                .map(|j| {
                    let w = world(a * gu[i], b * gv[j], heights[i][j]);
                    [w.x, w.y, w.z]
                })
                .collect()
        })
        .collect();
    let surface = crate::native::NurbsSurface::from_points(pu, pv, knots_u.to_vec(), knots_v.to_vec(), &points, None)?;
    // The block over the rectangle, as high as the highest coefficient (the field lies within
    // its control net's hull), its top then replaced.
    let top = heights.iter().flatten().fold(0.0, |m: f64, &h| m.max(h));
    let corner = world(a * u0, b * v0, 0.0);
    let place = crate::facade::profile::Place { origin: [corner.x, corner.y, corner.z], ..*frame };
    let (block, history) = primitive(k, &place, Prim::Block { size: [a * (u1 - u0), b * (v1 - v0), top] })?;
    let lid = history
        .generated
        .iter()
        .find(|(_, o)| matches!(o, Origin::EndCap { .. }))
        .map(|(f, _)| *f)
        .ok_or_else(|| KernelError::InvalidInput("the slab's block has no top".into()))?;
    let r = k.replace_faces(
        block,
        &[crate::FaceReplacement { face: lid, surface: surface.into(), domain: Some(((u0, u1), (v0, v1))) }],
    );
    k.release(block);
    made(r)
}

/// `body` mapped by the rows `[A | t]`: a rigid motion or a reflection exactly
/// (cadrs's `transform_motion`), any other invertible map by cadrs's affine image.
pub fn transform(k: &mut DefaultKernel, body: BodyId, m: [[f64; 4]; 3]) -> Result<Made> {
    let linear = nalgebra::Matrix3::from_fn(|i, j| m[i][j]);
    let translation = Vector3::new(m[0][3], m[1][3], m[2][3]);
    if linear.determinant() == 0.0 || !linear.iter().chain(translation.iter()).all(|v| v.is_finite()) {
        return Err(KernelError::InvalidInput("a transform must be finite and invertible".into()));
    }
    let orthonormal = (linear.transpose() * linear - nalgebra::Matrix3::identity()).amax() <= crate::native::tol::REL;
    if orthonormal {
        made(k.transform_motion(body, &Motion { linear, translation }))
    } else {
        made(k.transform_affine(body, linear, translation))
    }
}

/// Edges `ix` blended by a conic of shoulder `rho` whose contacts lie `setback` from the edge on
/// each face (cadrs's conic fillet keeps the contacts of the circle of radius
/// `setback / tan(φ/2)`, φ the angle between the faces' normals). Straight edges between planes
/// only, all at one angle.
pub fn fillet_conic(k: &mut DefaultKernel, body: BodyId, ix: &[u64], setback: f64, rho: f64) -> Result<Made> {
    use crate::native::surface::Surface;
    if !(setback > 0.0 && rho > 0.0 && rho < 1.0) {
        return Err(KernelError::InvalidInput("a conic blend needs a positive setback and 0 < rho < 1".into()));
    }
    let info = k.edges(body)?;
    let b = k.get(body)?;
    let normal = |f: FaceId| -> Option<Vector3<f64>> {
        let face = b.faces.get(usize::try_from(f.0).ok()?)?;
        match &face.surf {
            Surface::Plane { f: fr } => Some(fr.z * if face.sense { 1.0 } else { -1.0 }),
            _ => None,
        }
    };
    let mut phi: Option<f64> = None;
    for &e in ix {
        let i = info.iter().find(|i| i.id.0 == e).ok_or_else(|| KernelError::InvalidInput("no such edge".into()))?;
        let (Some(a), Some(c)) = (i.faces[0].and_then(normal), i.faces[1].and_then(normal)) else {
            return Err(KernelError::Unsupported("a conic blend of an edge not between two planes"));
        };
        let angle = a.angle(&c);
        if phi.is_some_and(|p| (p - angle).abs() > crate::native::tol::ANGULAR) {
            return Err(KernelError::Unsupported("a conic blend of edges at different angles"));
        }
        phi = Some(angle);
    }
    let phi = phi.ok_or_else(|| KernelError::InvalidInput("a conic blend of no edge".into()))?;
    let half = (phi / 2.0).tan();
    if !(half > 0.0) {
        return Err(KernelError::Unsupported("a conic blend of a flat edge"));
    }
    let spec = crate::FilletSpec {
        profile: crate::FilletProfile::Conic { rho },
        ..crate::FilletSpec::radius(setback / half)
    };
    made(k.fillet_with(body, &faces_to_edges(ix), &spec))
}

fn faces_to_edges(ix: &[u64]) -> Vec<EdgeId> {
    ix.iter().map(|&e| EdgeId(e)).collect()
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

    /// A plate drilled through with a hole of radius `rad` at `c`, and its top rim's edges.
    fn drilled(k: &mut DefaultKernel, size: [f64; 3], c: [f64; 2], rad: f64) -> (BodyId, Vec<u64>) {
        let plate = k.primitive(&Primitive::Box { min: Point3::new(0.0, 0.0, 0.0), max: Point3::new(size[0], size[1], size[2]) }).and_then(|r| r.first()).unwrap();
        let hole = k
            .primitive(&Primitive::Cylinder {
                base: Point3::new(c[0], c[1], -1.0),
                axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
                radius: rad,
                height: size[2] + 2.0,
            })
            .and_then(|r| r.first())
            .unwrap();
        let (d, _) = boolean(k, crate::BoolOp::Subtract, plate, &[hole]).unwrap();
        let rim = k
            .edges(d)
            .unwrap()
            .iter()
            .filter(|e| (e.mid.z - size[2]).abs() < 1e-9 && ((e.mid.x - c[0]).hypot(e.mid.y - c[1]) - rad).abs() < 1e-6)
            .map(|e| e.id.0)
            .collect();
        (d, rim)
    }

    #[test]
    fn a_rim_round_that_runs_off_the_plate_is_declined() {
        // brep-conformance seed 2 chain-rim-after-cut-2ee7ef: the round's outer edge (radius 4.75
        // about (4.73, 4.32)) crosses the plate's sides x = 0 and y = 0.
        let mut k = DefaultKernel::new();
        let size = [10.875072829244813, 10.544234942431189, 5.669455797435571];
        let (d, rim) = drilled(&mut k, size, [4.726518256981529, 4.317994990440217], 3.428784132901775);
        assert!(!rim.is_empty());
        assert!(fillet(&mut k, d, &rim[..1], 1.3217145850745369).is_err());
        assert!(chamfer(&mut k, d, &rim[..1], 1.3217145850745369).is_err());
        // One that stays on the top: Pappus (section (1 - pi/4) r^2, its centroid at
        // rad + r (10 - 3 pi) / (12 - 3 pi) from the axis).
        let (r, rad) = (0.5, 3.428784132901775);
        let (f, _) = fillet(&mut k, d, &rim[..1], r).unwrap();
        let v0 = k.mass_properties(d).unwrap().volume;
        let pi = std::f64::consts::PI;
        let cut = 2.0 * pi * (rad + r * (10.0 - 3.0 * pi) / (12.0 - 3.0 * pi)) * (1.0 - pi / 4.0) * r * r;
        assert!((k.mass_properties(f).unwrap().volume - (v0 - cut)).abs() < 1e-6);
        assert!(chamfer(&mut k, d, &rim[..1], r).is_ok());
    }

    #[test]
    fn edge_rounds_meeting_their_end_faces_fit() {
        let mut k = DefaultKernel::new();
        let b = cube(&mut k);
        assert!(fillet(&mut k, b, &[0], 2.0).is_ok());
        let all: Vec<u64> = (0..k.edges(b).unwrap().len() as u64).collect();
        assert!(fillet(&mut k, b, &all, 2.0).is_ok());
        assert!(chamfer(&mut k, b, &[0], 2.0).is_ok());
    }

    #[test]
    fn a_ball_screwed_three_turns_is_a_round_ended_pipe() {
        // brep-bench sweep/helical-pipe-round-ends: pi r^2 L + 4/3 pi r^3 with L the helix's length.
        let mut k = DefaultKernel::new();
        let ball = k.primitive(&Primitive::Sphere { center: Point3::new(10.0, 0.0, 0.0), radius: 1.0, axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)), seam: nalgebra::Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)) }).and_then(|r| r.first()).unwrap();
        let tau = std::f64::consts::TAU;
        let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 3.0 * tau, rise_per_turn: 5.0 };
        let (s, _) = swept(&mut k, ball, &m).unwrap();
        let pi = std::f64::consts::PI;
        let len = 3.0 * (tau * 10.0).hypot(5.0);
        let want = pi * len + 4.0 / 3.0 * pi;
        let v = k.mass_properties(s).unwrap().volume;
        assert!((v - want).abs() < 1e-6 * want, "{v} against {want}");
    }

    #[test]
    fn a_screwed_torus_cuts_whole_and_exactly() {
        // brep-bench swept/torus-screwed-through-rod: cadrs's runner 1460.6300023677913 (4 faces).
        let mut k = DefaultKernel::new();
        let rod = k
            .primitive(&Primitive::Cylinder {
                base: Point3::new(0.0, 0.0, 0.0),
                axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
                radius: 5.0,
                height: 20.0,
            })
            .and_then(|r| r.first())
            .unwrap();
        let h = (std::f64::consts::TAU * 5.5 / 30.0).atan();
        let (s, c) = (-108f64).to_radians().sin_cos();
        let rot = |v: [f64; 3]| [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
        let centre = rot([11.5, 0.0, 0.0]);
        let (wheel, _) = torus(&mut k, [centre[0], centre[1], -9.0], rot([0.0, h.cos(), -h.sin()]), 6.0, 2.0).unwrap();
        let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 456f64.to_radians(), rise_per_turn: 30.0 };
        let (out, _) = cut_swept(&mut k, rod, wheel, &[m]).unwrap();
        let v = k.mass_properties(out).unwrap().volume;
        assert!((v - 1460.6300023677913).abs() < 1e-6 * v, "{v}");
        assert!(k.get(out).unwrap().faces.len() <= 10);
    }

    #[test]
    fn a_v_wheel_screwed_through_a_rod_cuts_exactly() {
        // brep-bench swept/v-wheel-screwed-through-rod: cadrs's runner 1554.3809661193923.
        let mut k = DefaultKernel::new();
        let rod = k
            .primitive(&Primitive::Cylinder {
                base: Point3::new(0.0, 0.0, 0.0),
                axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
                radius: 5.0,
                height: 20.0,
            })
            .and_then(|r| r.first())
            .unwrap();
        let h = (std::f64::consts::TAU * 4.5 / 30.0).atan();
        let along = Vector3::new(0.0, h.cos(), -h.sin());
        let centre = Point3::new(14.0, 0.0, 0.0);
        let x = Vector3::new(-1.0, 0.0, 0.0);
        let on = crate::Plane::new(centre, x, x.cross(&along));
        let w = 2.5 * 30f64.to_radians().tan();
        let section = crate::Profile::of(on, vec![crate::Loop::polygon(&[[10.0, 0.0], [7.5, w], [7.5, -w]])]);
        let wheel = k.revolve(&section, Axis::new(centre, along), std::f64::consts::TAU).and_then(|r| r.first()).unwrap();
        // cadrs's runner turns it from -144 degrees: placed there first.
        let start = crate::facade::motion::end_motion(&Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: (-144f64).to_radians(), rise_per_turn: 30.0 }).unwrap();
        let wheel = k.transform_motion(wheel, &start).and_then(|r| r.first()).unwrap();
        let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 528f64.to_radians(), rise_per_turn: 30.0 };
        let (out, _) = cut_swept(&mut k, rod, wheel, &[m]).unwrap();
        let v = k.mass_properties(out).unwrap().volume;
        assert!((v - 1554.3809661193923).abs() < 1e-6 * v, "{v}");
    }

    #[test]
    fn a_key_carried_along_an_s_shaped_spline() {
        // brep-bench swept/key-carried-along-a-spline: cadrs's runner 10652.879989210425. The
        // S-shaped control polygon encloses no area, yet the path is planar.
        let mut k = DefaultKernel::new();
        let ctrl = [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]];
        assert_eq!(plane_normal(&ctrl, 1e-9), Some([0.0, 0.0, 1.0]));
        let block = |k: &mut DefaultKernel, a: [f64; 3], b: [f64; 3]| {
            k.primitive(&Primitive::Box { min: p3(a), max: p3(b) }).and_then(|r| r.first()).unwrap()
        };
        let plate = block(&mut k, [-6.0, -6.0, -8.0], [36.0, 26.0, 0.0]);
        let key = block(&mut k, [-1.5, -0.8, -1.5], [1.5, 0.8, 2.5]);
        assert!(along_up(&k, key, &ctrl, true, 1e-9).is_err());
        let up = along_up(&k, key, &ctrl, false, 1e-9).unwrap();
        let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let m = Move::Along { degree: 3, knots: &knots, ctrl: &ctrl, up };
        let (out, _) = cut_swept(&mut k, plate, key, &[m]).unwrap();
        let v = k.mass_properties(out).unwrap().volume;
        assert!((v - 10652.879989210425).abs() < 1e-6 * v, "{v}");
        // A post on its own axis at the path's start: a fixed frame is the same motion.
        let post = k
            .primitive(&Primitive::Cylinder { base: Point3::new(0.0, 0.0, -1.5), axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)), radius: 1.0, height: 3.0 })
            .and_then(|r| r.first())
            .unwrap();
        assert_eq!(along_up(&k, post, &ctrl, true, 1e-9).unwrap(), [0.0, 0.0, 1.0]);
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

    #[test]
    fn a_hairline_saw_cut_is_kept() {
        let mut k = DefaultKernel::new();
        let cube = cube(&mut k);
        let saw = k
            .primitive(&Primitive::Box { min: Point3::new(5.0, -1.0, 5.0), max: Point3::new(5.000001, 12.0, 12.0) })
            .and_then(|r| r.first())
            .unwrap();
        let (cut, _) = boolean(&mut k, crate::BoolOp::Subtract, cube, &[saw]).unwrap();
        // The cube's 600 and the slot's two walls, 5 × 10 each... less the top strip.
        let area = k.mass_properties(cut).unwrap().surface_area;
        assert!((area - 700.0).abs() < 1e-3, "{area}");
        // The session's own tolerances are back.
        assert_eq!(k.tolerances(), crate::native::tol::Tolerances::default());
    }

    #[test]
    fn a_lens_thinner_than_the_tolerance_is_refused_not_empty() {
        let mut k = DefaultKernel::new();
        let ball = |k: &mut DefaultKernel, x: f64| {
            k.primitive(&Primitive::Sphere {
                center: Point3::new(x, 0.0, 0.0),
                radius: 5.0,
                axis: nalgebra::Unit::new_normalize(nalgebra::Vector3::z()),
                seam: nalgebra::Unit::new_normalize(nalgebra::Vector3::x()),
            })
            .and_then(|r| r.first())
            .unwrap()
        };
        let (a, b) = (ball(&mut k, 0.0), ball(&mut k, 9.999999));
        match boolean(&mut k, crate::BoolOp::Intersect, a, &[b]) {
            Ok((lens, _)) => {
                let v = k.mass_properties(lens).unwrap().volume;
                assert!(v > 0.0 && v < 1e-9, "{v}");
            }
            Err(KernelError::EmptyResult(_)) => panic!("an empty answer for solids that overlap"),
            Err(_) => {}
        }
    }

    #[test]
    fn a_flat_slab_is_a_block_and_an_affine_map_stretches() {
        let mut k = DefaultKernel::new();
        let world = crate::facade::profile::Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
        // Every coefficient 2: a flat top at z = 2 over a 4 × 3 rectangle.
        let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let heights = vec![vec![2.0; 3]; 3];
        let (s, _) = slab(&mut k, &world, [4.0, 3.0], &knots, &knots, &heights).unwrap();
        let v = k.mass_properties(s).unwrap().volume;
        assert!((v - 24.0).abs() < 1e-6, "{v}");
        let b = cube(&mut k);
        let (t, _) = transform(&mut k, b, [[2.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).unwrap();
        assert!((k.mass_properties(t).unwrap().volume - 2000.0).abs() < 1e-6);
        let (r, _) = transform(&mut k, b, [[0.0, -1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).unwrap();
        assert!((k.mass_properties(r).unwrap().center_of_mass - Point3::new(-5.0, 5.0, 5.0)).norm() < 1e-9);
        // A mirror stays a valid solid of the same volume; a uniform scale cubes the volume.
        let (m, _) = transform(&mut k, b, [[-1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).unwrap();
        assert!((k.mass_properties(m).unwrap().volume - 1000.0).abs() < 1e-6);
        assert!(k.get(m).unwrap().check().is_empty());
        let (u, _) = transform(&mut k, b, [[2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0]]).unwrap();
        assert!((k.mass_properties(u).unwrap().volume - 8000.0).abs() < 1e-6);
    }

    #[test]
    fn a_conic_blend_of_a_square_edge() {
        let mut k = DefaultKernel::new();
        let b = k
            .primitive(&Primitive::Box { min: Point3::origin(), max: Point3::new(20.0, 10.0, 10.0) })
            .and_then(|r| r.first())
            .unwrap();
        let e = {
            use crate::Pick;
            k.edge_at(b, Point3::new(20.0, 5.0, 10.0)).unwrap().0
        };
        let (c, _) = fillet_conic(&mut k, b, &[e], 3.0, 0.5).unwrap();
        // A parabola tangent to both faces at the contacts cuts a third of the corner's triangle
        // (3 × 3 / 2) off the corner: 1.5 per unit length, 10 long.
        let removed = 2000.0 - k.mass_properties(c).unwrap().volume;
        assert!((removed - 15.0).abs() < 1e-6, "{removed}");
    }

    #[test]
    fn a_conic_of_shoulder_root_two_less_one_is_a_quarter_circle() {
        let mut k = DefaultKernel::new();
        let b = k
            .primitive(&Primitive::Box { min: Point3::origin(), max: Point3::new(20.0, 10.0, 10.0) })
            .and_then(|r| r.first())
            .unwrap();
        let e = {
            use crate::Pick;
            k.edge_at(b, Point3::new(20.0, 5.0, 10.0)).unwrap().0
        };
        let (c, _) = fillet_conic(&mut k, b, &[e], 3.0, std::f64::consts::SQRT_2 - 1.0).unwrap();
        let removed = 2000.0 - k.mass_properties(c).unwrap().volume;
        let want = 9.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 10.0;
        assert!((removed - want).abs() < 1e-6, "{removed} {want}");
    }

}
