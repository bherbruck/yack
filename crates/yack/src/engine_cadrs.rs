// yack: routing to the temporary cadrs engine through yack-bridge (PLAN §3.2, §3.5, M1.10):
// sessions, conversions and error mapping only. No arithmetic here (gate 13).
//
// A cadrs body lives in a session (`DefaultKernel`). A solid cadrs built shares its operands'
// session: cadrs keeps per-session caches (exact swept tori, rigid-move records) its results may
// depend on, so a result can depend on its lineage's session. There is no global session. Every
// cadrs result is carried back to springrep through the bridge at once; queries and selection
// run on that springrep form, and its face order is the bridge's (it preserves order).
use std::collections::BTreeSet;
use std::sync::{Arc, Mutex, MutexGuard};

use yack_cadrs::yack_glue::{self, Made, Move};
use yack_cadrs::facade::names::{self as cnames, FaceRole, Wall};
use yack_cadrs::{BodyId, BoolOp, DefaultKernel, History, InputFace, Kernel, KernelError, Origin};
use yack_springrep::topo::FaceName;

use crate::engine::{self, Brep};
use crate::{Error, ErrorKind, FrameLaw, Motion, Result, Solid, Stage, SurfaceKind};

/// A body in a cadrs session.
#[derive(Clone)]
pub(crate) struct CadrsRef {
    session: Arc<Mutex<DefaultKernel>>,
    id: BodyId,
}

fn lock(s: &Mutex<DefaultKernel>) -> MutexGuard<'_, DefaultKernel> {
    s.lock().unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// cadrs's refusal as yack's error, from `op`.
pub(crate) fn error(op: &'static str, e: KernelError) -> Error {
    let detail = format!("cadrs: {e}");
    match e {
        // cadrs's "invalid" verdicts are taken as its own limits: whether an input breaks an op's
        // requirement is decided by springrep's error map and by the facade's validation (the
        // cadrs-only ops check their stated requirements before running), never by cadrs alone.
        KernelError::InvalidProfile(_)
        | KernelError::InvalidParameter(_)
        | KernelError::InvalidInput(_)
        | KernelError::UnknownBody(_) => Error { kind: ErrorKind::Failed, op, arg: None, stage: Some(Stage::Validate), detail },
        KernelError::Unsupported(_) => Error::new(ErrorKind::Unsupported, op, detail),
        KernelError::BudgetExhausted { .. } => Error { kind: ErrorKind::Budget, op, arg: None, stage: None, detail },
        KernelError::InvalidResult { .. } => Error::failed(op, Stage::Check, detail),
        _ => Error { kind: ErrorKind::Failed, op, arg: None, stage: None, detail },
    }
}

fn unsupported(op: &'static str, what: impl Into<String>) -> Error {
    Error::new(ErrorKind::Unsupported, op, what)
}

fn convert(b: &Brep) -> core::result::Result<CadrsRef, String> {
    let (body, _names) = yack_bridge::to_cadrs(b).map_err(|e| e.to_string())?;
    let mut k = DefaultKernel::new();
    let id = k.insert(body);
    Ok(CadrsRef { session: Arc::new(Mutex::new(k)), id })
}

/// `s` in cadrs, converted on first use; a solid the bridge cannot carry declines (Unsupported).
fn of(op: &'static str, s: &Solid) -> Result<CadrsRef> {
    s.cadrs_form(convert).cloned().map_err(|why| unsupported(op, format!("cadrs cannot take this solid: {why}")))
}

/// `b`'s body in `a`'s session (copied there when they differ).
fn into_session(op: &'static str, a: &CadrsRef, b: &CadrsRef) -> Result<BodyId> {
    if Arc::ptr_eq(&a.session, &b.session) {
        return Ok(b.id);
    }
    let body = (**lock(&b.session).get(b.id).map_err(|e| error(op, e))?).clone();
    Ok(lock(&a.session).insert(body))
}

/// How to name a cadrs result's faces from its history: the inputs (cadrs body, springrep form)
/// whose kept faces keep their names, and for a blend the call's edges and the name kind of the
/// face each one makes (`round` / `bevel`).
#[derive(Default)]
struct Naming<'a> {
    inputs: Vec<(BodyId, &'a Brep)>,
    edges: &'a [u64],
    /// The edges the blend actually ran along (each edge's tangent chain), with the index in
    /// `edges` of the edge whose chain it is on.
    seeds: Vec<(u64, usize)>,
    blend: &'a str,
    /// For a sweep of a profile: each segment's face part (`seg:<label>` / `seg:s<n>`), by the
    /// segment's index through the profile's loops (the cadrs curve's `source`).
    segments: Vec<String>,
    /// Per result face, cadrs's naming glue's roles (a shell's): these replace the history rules.
    roles: Option<Vec<Vec<FaceRole>>>,
}

fn kept(how: &Naming, from: &InputFace) -> Option<BTreeSet<FaceName>> {
    how.inputs.iter().find(|(id, _)| *id == from.body).and_then(|(_, b)| {
        let face = b.face_ids().nth(usize::try_from(from.face.0).ok()?)?;
        Some(b.face(face).names.clone())
    })
}

fn names(feature: &str, n: usize, history: &History, how: &Naming) -> Vec<BTreeSet<FaceName>> {
    let mut out = vec![BTreeSet::from([FaceName::new(feature, "")]); n];
    if let Some(roles) = &how.roles {
        for (slot, rs) in out.iter_mut().zip(roles) {
            let mut set = BTreeSet::new();
            for r in rs {
                match r {
                    FaceRole::Keep(from) => set.extend(kept(how, from).unwrap_or_default()),
                    FaceRole::Part(p) => {
                        set.insert(FaceName::new(feature, p.as_str()));
                    }
                }
            }
            if !set.is_empty() {
                *slot = set;
            }
        }
        return out;
    }
    for (f, from) in &history.modified {
        let kept = how.inputs.iter().find(|(id, _)| *id == from.body).and_then(|(_, b)| {
            let face = b.face_ids().nth(usize::try_from(from.face.0).ok()?)?;
            Some(b.face(face).names.clone())
        });
        if let (Some(slot), Some(kept)) = (usize::try_from(f.0).ok().and_then(|i| out.get_mut(i)), kept) {
            *slot = kept;
        }
    }
    for (f, origin) in &history.generated {
        let part = match origin {
            Origin::FromEdge { edge, .. } if !how.blend.is_empty() => {
                let seed = how.seeds.iter().find(|(e, _)| *e == edge.0).map(|&(_, n)| n);
                seed.or_else(|| how.edges.iter().position(|&e| e == edge.0)).map(|n| format!("{}_{n}", how.blend))
            }
            Origin::FromVertex { .. } => Some("corner".to_string()),
            Origin::ProfileCurve { curve, .. } => usize::try_from(*curve).ok().and_then(|i| how.segments.get(i)).cloned(),
            Origin::StartCap { .. } => Some("start".to_string()),
            Origin::EndCap { .. } => Some("end".to_string()),
            _ => None,
        };
        if let (Some(slot), Some(part)) = (usize::try_from(f.0).ok().and_then(|i| out.get_mut(i)), part) {
            *slot = BTreeSet::from([FaceName::new(feature, part)]);
        }
    }
    out
}

/// A cadrs result in `session` as a solid: carried back through the bridge (refused, as
/// Unsupported, when it cannot be), its faces named from its history (kept faces keep their
/// names; new ones carry `feature`).
fn result(op: &'static str, feature: &str, session: &Arc<Mutex<DefaultKernel>>, r: yack_cadrs::Result<Made>, how: &Naming) -> Result<Solid> {
    let (id, history) = match r {
        Err(KernelError::EmptyResult(_)) => return Ok(Solid::empty()),
        r => r.map_err(|e| error(op, e))?,
    };
    let body = lock(session).get(id).map_err(|e| error(op, e))?.clone();
    let names = names(feature, body.faces().len(), &history, how);
    // Edges cadrs left unused by any face are dropped (ids are springrep's, so nothing to remap).
    let (brep, _edges) = yack_bridge::to_springrep_mapped(&body, &names)
        .map_err(|e| unsupported(op, format!("the cadrs result cannot be carried back: {e}")))?;
    Ok(Solid::from_springrep(brep)
        .with_feature(feature)
        .with_cadrs(CadrsRef { session: session.clone(), id })
        .with_strategy("cadrs"))
}

/// Whether every face of `s` lies on a sphere (a ball, whatever its frame).
fn is_ball(s: &Solid) -> bool {
    let faces = s.faces();
    !faces.is_empty() && s.faces_where(|f| matches!(f.kind(), SurfaceKind::Sphere { .. })).len() == faces.len()
}

/// One leg of a motion in cadrs's terms; `ctrl` holds an `Along` path's control points.
fn leg<'a>(op: &'static str, s: &Solid, m: &'a Motion, ctrl: &'a [[f64; 3]]) -> Result<Move<'a>> {
    Ok(match m {
        Motion::Slide { by } => Move::Slide { by: by.to_array() },
        Motion::Turn { axis_point, axis, angle } => {
            Move::Turn { point: axis_point.to_array(), axis: axis.to_array(), angle: angle.radians() }
        }
        Motion::Screw { axis_point, axis, angle, rise_per_turn } => Move::Screw {
            point: axis_point.to_array(),
            axis: axis.to_array(),
            angle: angle.radians(),
            rise_per_turn: *rise_per_turn,
        },
        Motion::Along { path, law } => {
            let up = match law {
                FrameLaw::Fixed if is_ball(s) => [0.0, 0.0, 1.0],
                FrameLaw::Fixed => return Err(unsupported(op, "a fixed-frame path sweep of a solid other than a ball")),
                FrameLaw::RotationMinimising => yack_glue::plane_normal(ctrl, yack_tol::TANGENT)
                    .ok_or_else(|| unsupported(op, "a rotation-minimising sweep along a path that is not planar"))?,
                FrameLaw::Toward(_) => return Err(unsupported(op, "a path sweep facing a direction")),
            };
            let degree = usize::try_from(path.degree()).unwrap_or(usize::MAX);
            Move::Along { degree, knots: path.knots(), ctrl, up }
        }
        Motion::Then(_) => return Err(unsupported(op, "a sequence nested in a sequence")),
    })
}

fn control(m: &Motion) -> Vec<[f64; 3]> {
    match m {
        Motion::Along { path, .. } => path.ctrl().iter().map(|p| p.to_array()).collect(),
        _ => Vec::new(),
    }
}

/// Everything `s` passes through during `m`, by cadrs's sweep of a solid (a sequence: each leg
/// starting where the last left the solid, their sweeps united).
pub(crate) fn swept(s: &Solid, m: &Motion) -> Result<Solid> {
    let op = "swept";
    let legs: Vec<&Motion> = match m {
        Motion::Then(ms) => ms.iter().collect(),
        one => vec![one],
    };
    let ctrls: Vec<Vec<[f64; 3]>> = legs.iter().map(|m| control(m)).collect();
    let moves = legs.iter().zip(&ctrls).map(|(m, c)| leg(op, s, m, c)).collect::<Result<Vec<_>>>()?;
    let c = of(op, s)?;
    let feature = engine::feature();
    let r = match moves.as_slice() {
        [] => return Ok(s.clone()),
        [one] => yack_glue::swept(&mut lock(&c.session), c.id, one),
        many => yack_cadrs::facade::motion::swept_then(&mut lock(&c.session), c.id, many),
    };
    let how = Naming { inputs: s.brep().map(|b| vec![(c.id, b)]).unwrap_or_default(), ..Naming::default() };
    result(op, &feature, &c.session, r, &how)
}

/// `a` combined with `tools` by cadrs's boolean `how`.
pub(crate) fn boolean(op: &'static str, how: BoolOp, a: &Solid, tools: &[&Solid]) -> Result<Solid> {
    let out = boolean_raw(op, how, a, tools)?;
    if matches!(how, BoolOp::Intersect)
        && out.is_empty()
        && let (Some(x), Some(y)) = (a.brep(), tools.first().and_then(|t| t.brep()))
        && yack_springrep::facade::api::boxes_overlap(x, y)
    {
        return Err(Error::failed(op, Stage::Classify, "cadrs declines: an empty intersection of solids whose boxes overlap"));
    }
    Ok(out)
}

fn boolean_raw(op: &'static str, how: BoolOp, a: &Solid, tools: &[&Solid]) -> Result<Solid> {
    let ca = of(op, a)?;
    let ids = tools.iter().map(|t| into_session(op, &ca, &of(op, t)?)).collect::<Result<Vec<_>>>()?;
    let feature = engine::feature();
    let r = lock(&ca.session).boolean(how, ca.id, &ids).and_then(|r| Ok((r.first()?, r.history)));
    let mut inputs: Vec<(BodyId, &Brep)> = a.brep().map(|b| vec![(ca.id, b)]).unwrap_or_default();
    inputs.extend(ids.iter().zip(tools).filter_map(|(&id, t)| Some((id, t.brep()?))));
    result(op, &feature, &ca.session, r, &Naming { inputs, ..Naming::default() })
}

/// A cadrs op on `s`'s body, its result carried back.
pub(crate) fn op(name: &'static str, s: &Solid, f: impl FnOnce(&mut DefaultKernel, BodyId) -> yack_cadrs::Result<Made>) -> Result<Solid> {
    blend(name, s, &[], "", f)
}

/// A cadrs blend of `edges` on `s`'s body, the face each edge makes named `{kind}_{n}`.
pub(crate) fn blend(
    name: &'static str,
    s: &Solid,
    edges: &[u64],
    kind: &str,
    f: impl FnOnce(&mut DefaultKernel, BodyId) -> yack_cadrs::Result<Made>,
) -> Result<Solid> {
    let c = of(name, s)?;
    let feature = engine::feature();
    let seeds = if edges.is_empty() { Vec::new() } else { yack_glue::chained(&lock(&c.session), c.id, edges).unwrap_or_default() };
    let r = f(&mut lock(&c.session), c.id);
    let how = Naming { inputs: s.brep().map(|b| vec![(c.id, b)]).unwrap_or_default(), edges, seeds, blend: kind, ..Naming::default() };
    result(name, &feature, &c.session, r, &how)
}

/// A cadrs constructor in a fresh session.
pub(crate) fn make(name: &'static str, f: impl FnOnce(&mut DefaultKernel) -> yack_cadrs::Result<Made>) -> Result<Solid> {
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = f(&mut lock(&session));
    result(name, &feature, &session, r, &Naming::default())
}

/// `s` split by the plane through `origin` square to `normal`: (front, back), either empty.
pub(crate) fn split(s: &Solid, origin: [f64; 3], normal: [f64; 3]) -> Result<(Solid, Solid)> {
    let op = "split";
    let c = of(op, s)?;
    let (front, back) = yack_glue::split(&mut lock(&c.session), c.id, origin, normal).map_err(|e| error(op, e))?;
    let side = |m: Option<Made>| match m {
        None => Ok(Solid::empty()),
        Some(m) => result(op, &engine::feature(), &c.session, Ok(m), &Naming::default()),
    };
    Ok((side(front)?, side(back)?))
}

use yack_cadrs::facade::profile::{self as cp, Place, Seg};

fn place(f: &crate::Frame) -> Place {
    Place { origin: f.origin().to_array(), x: f.x().vec().to_array(), y: f.y().vec().to_array(), z: f.z().vec().to_array() }
}

/// A yack profile as cadrs segment data (loops outer first), and each segment's face part.
fn segs<'a>(op: &'static str, p: &'a crate::Profile, ctrl: &'a [Vec<[f64; 2]>]) -> Result<Vec<Vec<Seg<'a>>>> {
    use crate::Segment as S;
    let mut k = 0usize;
    let mut out = Vec::new();
    for l in core::iter::once(p.outer()).chain(p.holes()) {
        let mut segs = Vec::new();
        for s in l.segments() {
            let c = ctrl.get(k).map_or(&[][..], Vec::as_slice);
            k = k.saturating_add(1);
            segs.push(match s {
                S::Line { a, b } => Seg::Line { a: a.to_array(), b: b.to_array() },
                S::Arc { center, radius, start, sweep } => {
                    Seg::Arc { center: center.to_array(), radius: *radius, start: start.radians(), sweep: sweep.radians() }
                }
                S::Conic { a, apex, b, weight } => Seg::Conic { a: a.to_array(), apex: apex.to_array(), b: b.to_array(), weight: *weight },
                S::Nurbs { degree, knots, weights, .. } => Seg::Nurbs {
                    degree: usize::try_from(*degree).unwrap_or(usize::MAX),
                    knots,
                    ctrl: c,
                    weights: weights.as_deref(),
                },
                S::PeriodicSpline { degree, knots, .. } => {
                    Seg::Periodic { degree: usize::try_from(*degree).unwrap_or(usize::MAX), knots, ctrl: c }
                }
                S::Blend { .. } => return Err(unsupported(op, "a blend segment (cadrs has no such curve)")),
            });
        }
        out.push(segs);
    }
    Ok(out)
}

fn ctrl2(p: &crate::Profile) -> Vec<Vec<[f64; 2]>> {
    use crate::Segment as S;
    core::iter::once(p.outer())
        .chain(p.holes())
        .flat_map(|l| l.segments().iter())
        .map(|s| match s {
            S::Nurbs { ctrl, .. } | S::PeriodicSpline { ctrl, .. } => ctrl.iter().map(|c| c.to_array()).collect(),
            _ => Vec::new(),
        })
        .collect()
}

/// `profile` swept by cadrs (`revolve` false: extruded `amount` along z; true: revolved `amount`
/// radians about z, drawn in xz), its side faces named by segment, its caps start/end.
pub(crate) fn sweep_profile(op: &'static str, f: &crate::Frame, p: &crate::Profile, amount: f64, revolve: bool) -> Result<Solid> {
    let ctrl = ctrl2(p);
    let loops = segs(op, p, &ctrl)?;
    let refs: Vec<&[Seg]> = loops.iter().map(Vec::as_slice).collect();
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = if revolve { cp::revolve(&mut lock(&session), &place(f), &refs, amount) } else { cp::extrude(&mut lock(&session), &place(f), &refs, amount) };
    let how = Naming { segments: engine::segment_parts(p), ..Naming::default() };
    result(op, &feature, &session, r, &how)
}

/// A loft through profiles in their own frames along guide splines, by cadrs.
pub(crate) fn loft_guided(op: &'static str, sections: &[(crate::Frame, crate::Profile)], guides: &[crate::Spline3]) -> Result<Solid> {
    let ctrls: Vec<Vec<Vec<[f64; 2]>>> = sections.iter().map(|(_, p)| ctrl2(p)).collect();
    let loops: Vec<Vec<Vec<Seg>>> = sections.iter().zip(&ctrls).map(|((_, p), c)| segs(op, p, c)).collect::<Result<_>>()?;
    let refs: Vec<Vec<&[Seg]>> = loops.iter().map(|l| l.iter().map(Vec::as_slice).collect()).collect();
    let secs: Vec<(Place, &[&[Seg]])> = sections.iter().zip(&refs).map(|((f, _), r)| (place(f), r.as_slice())).collect();
    let gctrl: Vec<Vec<[f64; 3]>> = guides.iter().map(|g| g.ctrl().iter().map(|p| p.to_array()).collect()).collect();
    let gs: Vec<cp::Spline> = guides
        .iter()
        .zip(&gctrl)
        .map(|(g, c)| cp::Spline { degree: usize::try_from(g.degree()).unwrap_or(usize::MAX), knots: g.knots(), ctrl: c, weights: None })
        .collect();
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = cp::loft_guided(&mut lock(&session), &secs, &gs);
    result(op, &feature, &session, r, &Naming::default())
}

/// A solid bounded by a closed NURBS net, by cadrs.
pub(crate) fn nurbs_solid(
    op: &'static str,
    f: &crate::Frame,
    degree: [u32; 2],
    knots: [&[f64]; 2],
    ctrl: &[Vec<crate::Point3>],
    weights: Option<&[Vec<f64>]>,
) -> Result<Solid> {
    let net: Vec<Vec<[f64; 3]>> = ctrl.iter().map(|r| r.iter().map(|p| p.to_array()).collect()).collect();
    let d = degree.map(|x| usize::try_from(x).unwrap_or(usize::MAX));
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = cp::nurbs_solid(&mut lock(&session), &place(f), d, knots, &net, weights);
    result(op, &feature, &session, r, &Naming::default())
}

/// A primitive built by cadrs in frame `f`, its faces named as springrep's are (caps start/end,
/// block sides x_min..y_max, a round side `side`, a ball's `skin`).
pub(crate) fn primitive(op: &'static str, f: &crate::Frame, p: yack_glue::Prim) -> Result<Solid> {
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = yack_glue::primitive(&mut lock(&session), &place(f), p);
    let segments = ["seg:x_min", "seg:x_max", "seg:y_min", "seg:y_max", "seg:side", "skin"].map(String::from).to_vec();
    result(op, &feature, &session, r, &Naming { segments, ..Naming::default() })
}

/// `target` less every point `tool` passes through along `m`, cut directly by cadrs.
pub(crate) fn cut_swept(target: &Solid, tool: &Solid, m: &Motion) -> Result<Solid> {
    let op = "cut_swept";
    let legs: Vec<&Motion> = match m {
        Motion::Then(ms) => ms.iter().collect(),
        one => vec![one],
    };
    let ctrls: Vec<Vec<[f64; 3]>> = legs.iter().map(|m| control(m)).collect();
    let moves = legs.iter().zip(&ctrls).map(|(m, c)| leg(op, tool, m, c)).collect::<Result<Vec<_>>>()?;
    let ct = of(op, target)?;
    let tool_id = into_session(op, &ct, &of(op, tool)?)?;
    let feature = engine::feature();
    let r = yack_glue::cut_swept(&mut lock(&ct.session), ct.id, tool_id, &moves);
    let how = Naming { inputs: target.brep().map(|b| vec![(ct.id, b)]).unwrap_or_default(), ..Naming::default() };
    result(op, &feature, &ct.session, r, &how)
}

/// A cadrs shell of `s` (sharp or `rounded`, inward or `outward`), open at faces `open`, its faces
/// named by cadrs's shell naming: the input's faces keep their names, the cavity's are `inner`.
pub(crate) fn shell(name: &'static str, s: &Solid, open: &[u64], thickness: f64, outward: bool, rounded: bool) -> Result<Solid> {
    let c = of(name, s)?;
    let feature = engine::feature();
    let r = {
        let mut k = lock(&c.session);
        let r = if rounded {
            yack_glue::shell_rounded(&mut k, c.id, open, thickness, outward)
        } else {
            yack_glue::shell(&mut k, c.id, open, thickness, outward)
        };
        r.and_then(|(id, history)| {
            let how = cnames::Naming { shell: Some(if outward { Wall::Outward } else { Wall::Inward }), ..cnames::Naming::default() };
            let roles = cnames::face_roles(&k, Some(c.id), id, &history, &how)?;
            Ok(((id, history), roles))
        })
    };
    let (made, roles) = match r {
        Ok((m, roles)) => (Ok(m), Some(roles)),
        Err(e) => (Err(e), None),
    };
    let how = Naming { inputs: s.brep().map(|b| vec![(c.id, b)]).unwrap_or_default(), roles, ..Naming::default() };
    result(name, &feature, &c.session, made, &how)
}

