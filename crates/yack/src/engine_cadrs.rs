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
use yack_cadrs::{BodyId, BoolOp, DefaultKernel, History, Kernel, KernelError, Origin};
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
        KernelError::InvalidProfile(_)
        | KernelError::InvalidParameter(_)
        | KernelError::InvalidInput(_)
        | KernelError::UnknownBody(_) => Error { kind: ErrorKind::Invalid, op, arg: None, stage: Some(Stage::Validate), detail },
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
    blend: &'a str,
}

fn names(feature: &str, n: usize, history: &History, how: &Naming) -> Vec<BTreeSet<FaceName>> {
    let mut out = vec![BTreeSet::from([FaceName::new(feature, "")]); n];
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
                how.edges.iter().position(|&e| e == edge.0).map(|n| format!("{}_{n}", how.blend))
            }
            Origin::FromVertex { .. } => Some("corner".to_string()),
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
    let brep = yack_bridge::to_springrep(&body, &names)
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

/// Everything `s` passes through during `m`, by cadrs's sweep of a solid.
pub(crate) fn swept(s: &Solid, m: &Motion) -> Result<Solid> {
    let op = "swept";
    let ctrl: Vec<[f64; 3]>;
    let mv = match m {
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
            ctrl = path.ctrl().iter().map(|p| p.to_array()).collect();
            let up = match law {
                FrameLaw::Fixed if is_ball(s) => [0.0, 0.0, 1.0],
                FrameLaw::Fixed => return Err(unsupported(op, "a fixed-frame path sweep of a solid other than a ball")),
                FrameLaw::RotationMinimising => yack_glue::plane_normal(&ctrl, yack_tol::TANGENT)
                    .ok_or_else(|| unsupported(op, "a rotation-minimising sweep along a path that is not planar"))?,
                FrameLaw::Toward(_) => return Err(unsupported(op, "a path sweep facing a direction")),
            };
            let degree = usize::try_from(path.degree()).unwrap_or(usize::MAX);
            Move::Along { degree, knots: path.knots(), ctrl: &ctrl, up }
        }
        Motion::Then(_) => return Err(unsupported(op, "a sequence of motions")),
    };
    let c = of(op, s)?;
    let feature = engine::feature();
    let r = yack_glue::swept(&mut lock(&c.session), c.id, &mv);
    let how = Naming { inputs: s.brep().map(|b| vec![(c.id, b)]).unwrap_or_default(), ..Naming::default() };
    result(op, &feature, &c.session, r, &how)
}

/// `a` combined with `tools` by cadrs's boolean `how`.
pub(crate) fn boolean(op: &'static str, how: BoolOp, a: &Solid, tools: &[&Solid]) -> Result<Solid> {
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
    let r = f(&mut lock(&c.session), c.id);
    let how = Naming { inputs: s.brep().map(|b| vec![(c.id, b)]).unwrap_or_default(), edges, blend: kind };
    result(name, &feature, &c.session, r, &how)
}

/// A cadrs constructor in a fresh session.
pub(crate) fn make(name: &'static str, f: impl FnOnce(&mut DefaultKernel) -> yack_cadrs::Result<Made>) -> Result<Solid> {
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = f(&mut lock(&session));
    result(name, &feature, &session, r, &Naming::default())
}
