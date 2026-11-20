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
    /// Turned about the line through `point` along `axis` from the angle `from` to `to` (radians),
    /// the body as given at the angle 0, while rising along the axis by `rise[0] + rise[1] (θ −
    /// from) + rise[2] (θ − from)² + …` (yack's `Motion::screw` with any lead, or `Motion::turn`
    /// with `rise` empty, from any angle).
    #[allow(missing_docs)]
    Climb { point: [f64; 3], axis: [f64; 3], from: f64, to: f64, rise: &'a [f64] },
}

/// `rise` (a polynomial in θ − `from`, constant first) as a polynomial in θ, its trailing zeros
/// dropped: cadrs's [`SolidMotion::Climb`] coefficients.
fn absolute_rise(rise: &[f64], from: f64) -> Vec<f64> {
    // c(θ − f)^k = Σ_j C(k, j) θ^j (−f)^(k − j).
    let mut out = vec![0.0; rise.len()];
    for (k, &c) in rise.iter().enumerate() {
        let mut binom = 1.0;
        for (j, o) in out.iter_mut().enumerate().take(k + 1) {
            *o += c * binom * (-from).powi(i32::try_from(k - j).unwrap_or(i32::MAX));
            binom = binom * (k - j) as f64 / (j + 1) as f64;
        }
    }
    while out.last() == Some(&0.0) {
        out.pop();
    }
    out
}

/// The rise of a [`Move::Climb`] at the angle `a`.
pub fn climb_rise(rise: &[f64], from: f64, a: f64) -> f64 {
    rise.iter().rev().fold(0.0, |acc, c| acc * (a - from) + c)
}

/// Whether a [`Move::Climb`]'s rise is at most linear (a turn, or a screw of constant lead):
/// cadrs sweeps and cuts those as one exact rigid motion.
fn steady(rise: &[f64]) -> bool {
    rise.iter().skip(2).all(|&c| c == 0.0)
}

/// The most turns a screw [`cut_swept`] takes on at all (a runaway bound).
pub const SCREW_TURNS_MAX: f64 = 64.0;

/// Whether `m` screws or climbs more turns than any part has ([`SCREW_TURNS_MAX`]).
fn runaway(m: &Move) -> bool {
    let turned = match *m {
        Move::Screw { angle, .. } => angle.abs(),
        Move::Climb { from, to, .. } => (to - from).abs(),
        _ => 0.0,
    };
    turned > SCREW_TURNS_MAX * std::f64::consts::TAU
}

/// cadrs's motion for `m`.
fn solid_motion(m: &Move) -> SolidMotion {
    match *m {
        Move::Slide { by } => SolidMotion::slide(v3(by)),
        Move::Turn { point, axis, angle } => SolidMotion::swing(Axis::new(p3(point), v3(axis)), 0.0, angle),
        Move::Screw { point, axis, angle, rise_per_turn } => {
            SolidMotion::helical(Axis::new(p3(point), v3(axis)), rise_per_turn, 0.0, angle)
        }
        Move::Climb { point, axis, from, to, rise } => {
            let ax = Axis::new(p3(point), v3(axis));
            if steady(rise) {
                let d = ax.dir.into_inner();
                SolidMotion::rigid(vec![
                    crate::RigidStep::turn(ax, from, to),
                    crate::RigidStep::slide(d * climb_rise(rise, from, from), d * climb_rise(rise, from, to)),
                ])
            } else {
                SolidMotion::Climb { axis: ax, from, to, rise: absolute_rise(rise, from) }
            }
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
        Move::Turn { axis, .. } | Move::Screw { axis, .. } | Move::Climb { axis, .. } => axis,
        Move::Along { up, .. } => up,
        Move::Slide { by } => by,
    };
    if v3(dir).norm() == 0.0 {
        return Err(KernelError::InvalidInput("a motion needs a non-zero direction".into()));
    }
    // A screw past a whole turn: cadrs sweeps a ball, a box or a torus exactly (as cadrs's runner,
    // helical-pipe-round-ends, box-screwed-inside-a-rod); where it cannot it falls back to a
    // mesh-bounded solid, refused below. A screw of more turns than any part has is a runaway.
    if runaway(m) {
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
/// continue a face at one of the blended edges, or one with a corner within the blend's `size` of
/// an end of the chain (where it ends, its corner is shaped by the faces there: a mitred
/// octahedron's), or one bordering a face
/// the blend swallowed (the protocol's narrow face: the ball rolls on past it). A face the
/// history doesn't name is one cadrs kept as it was, so it is one of those. A blend whose contact runs
/// off its face onto a third one (a hole's rim round wider than the plate round it) is declined:
/// cadrs trims it by that face and answers a solid that is not the protocol's.
fn fits(k: &DefaultKernel, body: BodyId, es: &[EdgeId], size: f64, out: &Made) -> Result<()> {
    let info = k.edges(body)?;
    let named: Vec<&EdgeInfo> = info.iter().filter(|i| es.contains(&i.id)).collect();
    let tol = crate::native::tol::POINT;
    let near = |a: &Point3<f64>, b: &Point3<f64>| (a - b).norm() <= tol;
    // The chain's ends: the named edges' end points no other named edge continues from (a closed
    // rim has none). Joints within the chain shape nothing.
    let points: Vec<Point3<f64>> = named.iter().filter(|i| !i.is_closed()).flat_map(|i| [i.start, i.end]).collect();
    let ends: Vec<Point3<f64>> = points.iter().filter(|p| points.iter().filter(|q| near(p, q)).count() == 1).copied().collect();
    let reach = size.abs() + tol;
    let mut allowed: Vec<FaceId> = Vec::new();
    for i in &info {
        let at_end = [i.start, i.end].iter().any(|p| ends.iter().any(|q| (p - q).norm() <= reach));
        if es.contains(&i.id) || at_end {
            allowed.extend(i.faces.iter().flatten().copied());
        }
    }
    let (made_body, history) = out;
    // Past a swallowed face: its neighbours.
    let gone: Vec<FaceId> = history.deleted.iter().filter(|d| d.body == body).map(|d| d.face).collect();
    for i in &info {
        if i.faces.iter().flatten().any(|f| gone.contains(f)) {
            allowed.extend(i.faces.iter().flatten().copied());
        }
    }
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
            if old(o).is_some_and(|f| !allowed.contains(&f)) {
                return Err(KernelError::InvalidInput(
                    "the blend does not fit: its contact runs off a face it rounds onto another face".into(),
                ));
            }
        }
    }
    Ok(())
}

/// Edges `ix` (and their tangent chains) rounded with `radius`.
pub fn fillet(k: &mut DefaultKernel, body: BodyId, ix: &[u64], radius: f64) -> Result<Made> {
    let es = chain_ids(k, body, ix)?;
    let out = made(k.fillet(body, &es, radius))?;
    fits(k, body, &es, radius, &out)?;
    Ok(out)
}

/// Edges `ix` (and their tangent chains) chamfered with `opts`.
fn chamfer_opts(k: &mut DefaultKernel, body: BodyId, ix: &[u64], opts: &ChamferOpts) -> Result<Made> {
    let es = chain_ids(k, body, ix)?;
    let out = made(k.chamfer_with(body, &es, opts))?;
    // How far the bevel reaches from the edge on either face.
    let size = match opts.spec {
        ChamferSpec::EqualDistance(d) => d,
        ChamferSpec::TwoDistances(a, b) => a.abs().max(b.abs()),
        ChamferSpec::DistanceAngle { distance, angle } => distance.abs().max((distance * angle.tan()).abs()),
    };
    fits(k, body, &es, size, &out)?;
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
    // How far the target reaches from the axis: by its edges where they are lines and circles
    // (exact: a line is farthest at an end, a circle within its centre's distance and radius),
    // else by its bounding box's corners.
    let edges = k.edges(target)?;
    let by_edges = edges.iter().all(|e| matches!(e.curve, crate::CurveKind::Line) || e.circle.is_some());
    let reach = if by_edges && !edges.is_empty() {
        edges
            .iter()
            .map(|e| match &e.circle {
                Some(c) => radial(c.center) + c.radius,
                None => radial(e.start).max(radial(e.end)),
            })
            .fold(0.0, f64::max)
    } else {
        corners(k.bounding_box(target)?).into_iter().map(radial).fold(0.0, f64::max)
    };
    let tb = k.bounding_box(tool)?;
    if corners(tb).into_iter().map(radial).fold(0.0, f64::max) <= reach {
        return Ok(tool);
    }
    let (s0, s1) = corners(tb).into_iter().map(|p| (p - o).dot(&a)).fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), s| (l.min(s), h.max(s)));
    // Room to spare: a tenth of the reach round the axis (cadrs's runner trims a rod of radius 5
    // at 5.5), a hundredth of the sizes involved along it.
    let pad = 0.01 * (reach + (s1 - s0));
    let base = o + a * (s0 - pad);
    let cylinder = Primitive::Cylinder { base, axis: nalgebra::Unit::new_normalize(a), radius: 1.1 * reach, height: s1 - s0 + 2.0 * pad };
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
    revolution_axis(faces).is_some()
}

/// The axis `faces` are all of revolution about (a point on it and its direction), if any: a
/// ball's is the vertical through its centre.
fn revolution_axis(faces: &[crate::native::topo::Face]) -> Option<(Point3<f64>, Vector3<f64>)> {
    use crate::native::surface::Surface as S;
    let axis = faces.iter().find_map(|f| match &f.surf {
        S::Cylinder { f, .. } | S::Cone { f, .. } | S::Torus { f, .. } | S::Revolution { f, .. } => Some((f.o, f.z)),
        _ => None,
    });
    match axis {
        Some((o, a)) => of_revolution_about(faces, o, a).then_some((o, a)),
        None => match faces {
            [f] => match &f.surf {
                S::Sphere { f, .. } => Some((f.o, Vector3::new(0.0, 0.0, 1.0))),
                _ => None,
            },
            _ => None,
        },
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
    if runaway(m) {
        return Err(KernelError::Unsupported("a screw of more than 64 turns"));
    }
    // A body of revolution with a torus or cone on it is left whole: cadrs cuts by a turned or
    // screwed one exactly, by its envelope (torus-screwed-through-rod: 4 faces in 0.1 s,
    // v-wheel-screwed-through-rod), and by a trimmed piece of one only as a mesh. A drum (a
    // cylinder) is trimmed, as cadrs's runner trims it: whole, cadrs meshes its screw
    // (drum-end-screwed-through-rod: 10.7 s to a mesh, MEASURED).
    let whole = k.get(tool).is_ok_and(|b| {
        use crate::native::surface::Surface as S;
        of_revolution(&b.faces) && b.faces.iter().any(|f| matches!(f.surf, S::Torus { .. } | S::Cone { .. } | S::Revolution { .. }))
    });
    let used = match *m {
        Move::Turn { point, axis, .. } | Move::Screw { point, axis, .. } | Move::Climb { point, axis, .. } if !whole => {
            near_axis(k, tool, target, point, axis)?
        }
        _ => tool,
    };
    // cadrs's cut by a moving tool is exact for a screw (fine-thread: 1276.703 against cadrs's
    // runner's 1276.703) but not for a turn (pin-swung-through-ring-sector: 53.931 against the
    // exact 53.843, MEASURED natively): there the sweep, exact for one turn, is cut instead.
    // A climb whose lead grows: cadrs's cut takes it half a turn at a time, exactly.
    let screwing = match *m {
        Move::Screw { .. } => true,
        Move::Climb { rise, .. } => rise.iter().skip(1).any(|&c| c != 0.0),
        _ => false,
    };
    let r = match *m {
        _ if screwing => k.cut_swept(target, used, &solid_motion(m)),
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
    if moves.is_empty() {
        return Err(KernelError::InvalidInput("a cut by no motion".into()));
    }
    let mut made_here: Vec<BodyId> = Vec::new();
    let r = (|| {
        let (mut cur, mut at) = (target, tool);
        let mut i = 0;
        loop {
            // A run of slides (a toolpath's pass) is cut at once; any other move on its own.
            let run = pass_len(&moves[i..]);
            let n = run.max(1);
            let step = if run >= 2 { slides_cut(k, cur, at, &moves[i..i + n])? } else { cut_swept_one(k, cur, at, &moves[i])? };
            i += n;
            if i >= moves.len() {
                return Ok(step);
            }
            made_here.push(step.0);
            cur = step.0;
            for m in &moves[i - n..i] {
                at = k.transform_motion(at, &crate::facade::motion::end_motion(m)?)?.first()?;
                made_here.push(at);
            }
        }
    })();
    for id in made_here {
        if r.as_ref().map_or(true, |(out, _): &Made| *out != id) {
            k.release(id);
        }
    }
    r
}

/// How many of `moves` from the first make one pass: slides each going on from the last within
/// [`PASS_TURN`] of its direction (a toolpath's row; the step over to the next row, a sharp turn,
/// starts another). cadrs carries a tool along a pass of a row's length at once quickly, along a
/// whole raster of a thousand moves only slowly.
fn pass_len(moves: &[Move]) -> usize {
    let mut last: Option<Vector3<f64>> = None;
    let mut n = 0;
    for m in moves {
        let Move::Slide { by } = *m else { break };
        let v = v3(by);
        if v.norm() == 0.0 {
            n += 1;
            continue;
        }
        if let Some(l) = last
            && (l.dot(&v) / (l.norm() * v.norm())).clamp(-1.0, 1.0).acos() > PASS_TURN
        {
            break;
        }
        last = Some(v);
        n += 1;
    }
    n
}

/// The sharpest turn between two slides of one pass ([`pass_len`]): 60 degrees.
const PASS_TURN: f64 = std::f64::consts::FRAC_PI_3;

/// `target` less everything `tool` passes through as it slides by each of `slides` in turn.
/// A body of revolution rides the whole polyline at once (cadrs's cut by a solid carried along
/// a path, its axis kept along the frame's up: turning about its own axis leaves it as it is),
/// as cadrs's runner cuts a mill's pass; else, or where that fails, each slide's sweep, all of
/// them taken away in one boolean (not one cut per move on an ever larger part).
fn slides_cut(k: &mut DefaultKernel, target: BodyId, tool: BodyId, slides: &[Move]) -> Result<Made> {
    let by: Vec<Vector3<f64>> = slides
        .iter()
        .filter_map(|m| match *m {
            Move::Slide { by } => Some(v3(by)),
            _ => None,
        })
        .filter(|v| v.norm() > 0.0)
        .collect();
    let axis = k.get(tool).ok().and_then(|b| revolution_axis(&b.faces));
    if let Some((o, a)) = axis
        && by.len() >= 2
    {
        let mut at = o;
        let mut path = Vec::with_capacity(by.len());
        for v in &by {
            path.push(PathCurve::Bezier3 { poles: vec![at, at + v], weights: Vec::new() });
            at += v;
        }
        let motion = SolidMotion::Path { path, up: nalgebra::Unit::new_normalize(a) };
        if let Ok(r) = k.cut_swept(target, tool, &motion) {
            let (out, history) = (r.first()?, r.history);
            if !faceted(k, out)? {
                return Ok((out, history));
            }
            k.release(out);
        }
    }
    let mut tools = Vec::with_capacity(by.len());
    let mut offset = Vector3::zeros();
    let r = (|| {
        for v in &by {
            let placed = k.transform_motion(tool, &crate::Motion { linear: nalgebra::Matrix3::identity(), translation: offset })?.first()?;
            let sw = swept(k, placed, &Move::Slide { by: [v.x, v.y, v.z] });
            k.release(placed);
            tools.push(sw?.0);
            offset += v;
        }
        made(k.boolean(crate::BoolOp::Subtract, target, &tools))
    })();
    for t in tools {
        k.release(t);
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
#[path = "yack_glue_tests.rs"]
mod tests;
