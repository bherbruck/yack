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

use yack_cadrs::facade::motion as cmotion;
use yack_cadrs::yack_glue::{self, Made, Move};
use yack_cadrs::facade::names::{self as cnames, FaceRole, Wall};
use yack_cadrs::{BodyId, BoolOp, DefaultKernel, History, InputFace, KernelError, Origin};
use yack_topo::FaceName;

use crate::engine::{self, Brep};
use crate::{Error, ErrorKind, FrameLaw, Motion, Result, Solid, Stage};

/// A cadrs answer held in cadrs form (the lazy bridge): its body, its faces' names (by cadrs face
/// index, as the bridge would number them), and why the bridge refused it. A `Solid` wraps it.
#[derive(Clone)]
pub(crate) struct Held {
    pub(crate) cadrs: CadrsRef,
    pub(crate) names: Vec<BTreeSet<FaceName>>,
    pub(crate) why: String,
}

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
    // the body and its session records (swept tori, pipes) carried over; one session locked at a
    // time, so two ops copying between the same two sessions both ways never wait on each other
    let carried = yack_glue::carry_out(&lock(&b.session), b.id).map_err(|e| error(op, e))?;
    yack_glue::carry_in(&mut lock(&a.session), carried).map_err(|e| error(op, e))
}

/// How to name a cadrs result's faces from its history: the inputs (cadrs body, springrep form)
/// whose kept faces keep their names, and for a blend the call's edges and the name kind of the
/// face each one makes (`round` / `bevel`).
#[derive(Default)]
struct Naming<'a> {
    /// Each input body and its faces' names, by face index.
    inputs: Vec<(BodyId, Vec<BTreeSet<FaceName>>)>,
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
    how.inputs.iter().find(|(id, _)| *id == from.body).and_then(|(_, names)| names.get(usize::try_from(from.face.0).ok()?).cloned())
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
        let kept = kept(how, from);
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
    // The carried body must hold cadrs's volume (ring-lying-flat: carried 217.13, cadrs and its
    // STEP 761.10), else it is not carried: a reported volume never disagrees with the file. Volume
    // only: cadrs's area quadrature is off by tens of mm² on some offset faces whose volumes agree
    // exactly (conformance closing-offsets), where the carried form's area is the right one.
    let mass = {
        use yack_cadrs::Kernel as _;
        lock(session).mass_properties(id).ok()
    };
    let carried = yack_bridge::to_springrep_mapped(&body, &names).map_err(|e| e.to_string()).and_then(|(b, edges)| {
        // Decided to the agreement alone (an exact volume took 81% of bench ring's 36 s); the area,
        // exact, only describes a refusal.
        match &mass {
            Some(m) if !yack_measure::volume_agrees(&b, m.volume, yack_bridge::CADRS_AGREE) => Err(format!(
                "the carried body's volume is not within {} of cadrs's {} (its area {}; cadrs's {})",
                yack_bridge::CADRS_AGREE,
                m.volume,
                yack_measure::area(&b),
                m.surface_area
            )),
            _ => Ok((b, edges)),
        }
    });
    let (brep, _edges) = match carried {
        Ok(carried) => carried,
        Err(e) => {
            // The lazy bridge: kept in cadrs form, only if cadrs's own full check passes it
            // (AGENTS rule 8); its springrep rungs and queries then decline.
            let issues = body.check_with(yack_cadrs::native::topo::CheckLevel::Full);
            if !body.is_solid() || !issues.is_empty() {
                let why = issues.first().map_or("it is not a closed solid", String::as_str);
                return Err(unsupported(op, format!("the cadrs result cannot be carried back: {e}; nor held in cadrs form: {why}")));
            }
            // held only if no face has the solid on both sides: a held solid is measured by cadrs,
            // whose area counts such faces (closing-offsets: a notch pushed shut kept both walls)
            match yack_glue::inner_faces(&lock(session), id) {
                Ok(inner) if inner.is_empty() => {}
                Ok(inner) => {
                    let why = format!("{} of its faces have the solid on both sides", inner.len());
                    return Err(unsupported(op, format!("the cadrs result cannot be carried back: {e}; nor held: {why}")));
                }
                Err(err) => return Err(unsupported(op, format!("the cadrs result cannot be carried back: {e}; nor held: {err}"))),
            }
            let held = Held { cadrs: CadrsRef { session: session.clone(), id }, names, why: format!("bridge: {e}") };
            return Ok(Solid::from_held(held).with_feature(feature).with_strategy("cadrs"));
        }
    };
    Ok(Solid::from_springrep(brep)
        .with_feature(feature)
        .with_cadrs(CadrsRef { session: session.clone(), id })
        .with_strategy("cadrs"))
}

/// A leg's Climb rise (polynomial in θ − `from`, mm), when the leg is a turn or screw that cadrs
/// takes as a climb: a turn from a nonzero angle (no rise), a constant lead from a nonzero angle
/// (`[c·from, c]`, c per radian), any polynomial lead (its coefficients). `None` otherwise.
fn climb_rise(m: &Motion) -> Option<Vec<f64>> {
    use crate::input::{LeadKind as L, MotionKind as K};
    let per_radian = |l: &L| match l {
        L::PerRadian(c) => Some(*c),
        L::PerTurn(p) => Some(yack_math::per_radian_of(*p)),
        L::Polynomial(_) => None,
    };
    match &m.0 {
        K::Turn { from, .. } if from.radians() != 0.0 => Some(Vec::new()),
        K::Screw { lead, .. } if matches!(lead.0, L::Polynomial(_)) => match &lead.0 {
            L::Polynomial(c) => Some(c.clone()),
            _ => None,
        },
        K::Screw { from, lead, .. } if from.radians() != 0.0 => {
            let c = per_radian(&lead.0)?;
            Some(vec![yack_math::scale_length(c, from.radians()), c])
        }
        _ => None,
    }
}

/// A constant lead's rise per whole turn (cadrs's unit); `None` for a polynomial lead.
fn per_turn(lead: &crate::input::Lead) -> Option<f64> {
    use crate::input::LeadKind as L;
    match &lead.0 {
        L::PerTurn(p) => Some(*p),
        L::PerRadian(c) => Some(yack_math::per_turn_of(*c)),
        L::Polynomial(_) => None,
    }
}

/// One leg of a motion in cadrs's terms, for `s` in session `c`; `ctrl` holds an along-path's
/// control points.
fn leg<'a>(op: &'static str, c: &CadrsRef, m: &'a Motion, ctrl: &'a [[f64; 3]], rise: Option<&'a [f64]>) -> Result<Move<'a>> {
    use crate::input::MotionKind as K;
    let starts_at_zero = |from: &crate::Angle| from.radians() == 0.0;
    if let (Some(rise), K::Turn { axis_point, axis, from, to } | K::Screw { axis_point, axis, from, to, .. }) = (rise, &m.0) {
        return Ok(Move::Climb { point: axis_point.to_array(), axis: axis.to_array(), from: from.radians(), to: to.radians(), rise });
    }
    Ok(match &m.0 {
        K::Slide { by } => Move::Slide { by: by.to_array() },
        K::Turn { axis_point, axis, from, to } if starts_at_zero(from) => {
            Move::Turn { point: axis_point.to_array(), axis: axis.to_array(), angle: to.radians() }
        }
        K::Screw { axis_point, axis, from, to, lead } if starts_at_zero(from) => Move::Screw {
            point: axis_point.to_array(),
            axis: axis.to_array(),
            angle: to.radians(),
            rise_per_turn: per_turn(lead).ok_or_else(|| unsupported(op, "a screw of growing lead (not built yet)"))?,
        },
        K::Turn { .. } | K::Screw { .. } => return Err(unsupported(op, "a turn cadrs cannot take")),
        K::Along { path, law } => {
            let fixed = match law {
                FrameLaw::Fixed => true,
                FrameLaw::RotationMinimising => false,
                FrameLaw::Toward(_) => return Err(unsupported(op, "a path sweep facing a direction")),
            };
            let up = yack_glue::along_up(&lock(&c.session), c.id, ctrl, fixed, yack_tol::TANGENT).map_err(|e| error(op, e))?;
            let degree = usize::try_from(path.degree()).unwrap_or(usize::MAX);
            Move::Along { degree, knots: path.knots(), ctrl, up }
        }
        K::Then(_) | K::Together(_) => return Err(unsupported(op, "a sequence nested in a sequence")),
    })
}

fn control(m: &Motion) -> Vec<[f64; 3]> {
    match &m.0 {
        crate::input::MotionKind::Along { path, .. } => path.ctrl().iter().map(|p| p.to_array()).collect(),
        _ => Vec::new(),
    }
}

/// One leg of a [`Motion::together`] as cadrs's simultaneous step.
fn step(op: &'static str, m: &Motion) -> Result<cmotion::Step> {
    use crate::input::MotionKind as K;
    Ok(match &m.0 {
        K::Slide { by } => cmotion::Step::Slide { from: [0.0; 3], to: by.to_array() },
        K::Turn { axis_point, axis, from, to } => {
            cmotion::Step::Turn { point: axis_point.to_array(), axis: axis.to_array(), from: from.radians(), to: to.radians() }
        }
        K::Screw { axis_point, axis, from, to, lead } => cmotion::Step::Screw {
            point: axis_point.to_array(),
            axis: axis.to_array(),
            from: from.radians(),
            to: to.radians(),
            rise_per_turn: per_turn(lead).ok_or_else(|| unsupported(op, "a screw of growing lead together with other motions"))?,
        },
        _ => return Err(unsupported(op, "a path or a sequence inside motions run together")),
    })
}

/// Everything `s` passes through during `m`, by cadrs's sweep of a solid (a sequence: each leg
/// starting where the last left the solid, their sweeps united; legs together: cadrs's rigid
/// motion, run exact-only: cadrs refuses where it would only mesh).
pub(crate) fn swept(s: &Solid, m: &Motion) -> Result<Solid> {
    use crate::input::MotionKind as K;
    let op = "swept";
    let c = of(op, s)?;
    let feature = engine::feature();
    let how = Naming { inputs: vec![(c.id, s.face_names())], ..Naming::default() };
    if let K::Together(legs) = &m.0 {
        let steps = legs.iter().map(|l| step(op, l)).collect::<Result<Vec<_>>>()?;
        // under exact_only: where cadrs would answer only by a mesh it refuses (a decline), so
        // what comes back is exact, whichever path built it (polyhedron, by faces, rigid)
        let t = yack_glue::exact(&mut lock(&c.session), |k| cmotion::together(k, c.id, &steps)).map_err(|e| error(op, e))?;
        return result(op, &feature, &c.session, Ok(t.made), &how);
    }
    let legs: Vec<&Motion> = match &m.0 {
        K::Then(ms) => ms.iter().collect(),
        _ => vec![m],
    };
    let ctrls: Vec<Vec<[f64; 3]>> = legs.iter().map(|m| control(m)).collect();
    let rises: Vec<Option<Vec<f64>>> = legs.iter().map(|m| climb_rise(m)).collect();
    let moves = legs
        .iter()
        .zip(&ctrls)
        .zip(&rises)
        .map(|((m, ct), r)| leg(op, &c, m, ct, r.as_deref()))
        .collect::<Result<Vec<_>>>()?;
    let r = match moves.as_slice() {
        [] => return Ok(s.clone()),
        [one] => yack_glue::swept(&mut lock(&c.session), c.id, one),
        many => cmotion::swept_then(&mut lock(&c.session), c.id, many),
    };
    result(op, &feature, &c.session, r, &how)
}

/// `a` combined with `tools` by cadrs's boolean `how`.
pub(crate) fn boolean(op: &'static str, how: BoolOp, a: &Solid, tools: &[&Solid]) -> Result<Solid> {
    let out = boolean_raw(op, how, a, tools)?;
    if matches!(how, BoolOp::Intersect)
        && out.is_empty()
        && let (Some(x), Some(y)) = (a.brep(), tools.first().and_then(|t| t.brep()))
        && yack_measure::guard::boxes_overlap(x, y)
    {
        return Err(Error::failed(op, Stage::Classify, "cadrs declines: an empty intersection of solids whose boxes overlap"));
    }
    Ok(out)
}

fn boolean_raw(op: &'static str, how: BoolOp, a: &Solid, tools: &[&Solid]) -> Result<Solid> {
    let ca = of(op, a)?;
    let ids = tools.iter().map(|t| into_session(op, &ca, &of(op, t)?)).collect::<Result<Vec<_>>>()?;
    let feature = engine::feature();
    let r = yack_glue::boolean(&mut lock(&ca.session), how, ca.id, &ids);
    let mut inputs = vec![(ca.id, a.face_names())];
    inputs.extend(ids.iter().zip(tools).map(|(&id, t)| (id, t.face_names())));
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
    let how = Naming { inputs: vec![(c.id, s.face_names())], edges, seeds, blend: kind, ..Naming::default() };
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
    let ct = of(op, target)?;
    let tool_id = into_session(op, &ct, &of(op, tool)?)?;
    if let crate::input::MotionKind::Together(together) = &m.0 {
        // cadrs's cut by the tool with every motion run at once, exact-only (as cadrs's runner)
        let steps = together.iter().map(|l| step(op, l)).collect::<Result<Vec<_>>>()?;
        let feature = engine::feature();
        let r = cmotion::cut_together(&mut lock(&ct.session), ct.id, tool_id, &steps);
        let how = Naming { inputs: vec![(ct.id, target.face_names())], ..Naming::default() };
        return result(op, &feature, &ct.session, r, &how);
    }
    let legs: Vec<&Motion> = match &m.0 {
        crate::input::MotionKind::Then(ms) => ms.iter().collect(),
        _ => vec![m],
    };
    let tool_ref = CadrsRef { session: ct.session.clone(), id: tool_id };
    let ctrls: Vec<Vec<[f64; 3]>> = legs.iter().map(|m| control(m)).collect();
    let rises: Vec<Option<Vec<f64>>> = legs.iter().map(|m| climb_rise(m)).collect();
    let moves = legs
        .iter()
        .zip(&ctrls)
        .zip(&rises)
        .map(|((m, c), r)| leg(op, &tool_ref, m, c, r.as_deref()))
        .collect::<Result<Vec<_>>>()?;
    let feature = engine::feature();
    let r = yack_glue::cut_swept(&mut lock(&ct.session), ct.id, tool_id, &moves);
    let how = Naming { inputs: vec![(ct.id, target.face_names())], ..Naming::default() };
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
    let how = Naming { inputs: vec![(c.id, s.face_names())], roles, ..Naming::default() };
    result(name, &feature, &c.session, made, &how)
}


/// A slab built by cadrs (protocol `slab`).
pub(crate) fn slab(op: &'static str, f: &crate::Frame, size: [f64; 2], ku: &[f64], kv: &[f64], heights: &[Vec<f64>]) -> Result<Solid> {
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = yack_glue::slab(&mut lock(&session), &place(f), size, ku, kv, heights);
    result(op, &feature, &session, r, &Naming::default())
}

/// `profile` swept by cadrs along `path` (lines and arcs) in frame `f`.
pub(crate) fn sweep(op: &'static str, f: &crate::Frame, p: &crate::Profile, path: &[crate::PathPiece]) -> Result<Solid> {
    let ctrl = ctrl2(p);
    let loops = segs(op, p, &ctrl)?;
    let refs: Vec<&[Seg]> = loops.iter().map(Vec::as_slice).collect();
    let pieces: Vec<cp::Piece> = path
        .iter()
        .map(|x| match *x {
            crate::PathPiece::Line { length } => cp::Piece::Line { length },
            crate::PathPiece::Arc { radius, angle } => cp::Piece::Arc { radius, angle: angle.radians() },
        })
        .collect();
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = cp::sweep(&mut lock(&session), &place(f), &refs, &pieces);
    result(op, &feature, &session, r, &Naming { segments: engine::segment_parts(p), ..Naming::default() })
}

/// `profile` swept by cadrs along the spline `path` (frame coordinates), twisting `twist` radians.
pub(crate) fn sweep_path(op: &'static str, f: &crate::Frame, p: &crate::Profile, path: &crate::Spline3, twist: f64) -> Result<Solid> {
    let ctrl = ctrl2(p);
    let loops = segs(op, p, &ctrl)?;
    let refs: Vec<&[Seg]> = loops.iter().map(Vec::as_slice).collect();
    let pc: Vec<[f64; 3]> = path.ctrl().iter().map(|c| c.to_array()).collect();
    let spline = cp::Spline { degree: usize::try_from(path.degree()).unwrap_or(usize::MAX), knots: path.knots(), ctrl: &pc, weights: None };
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = cp::sweep_path(&mut lock(&session), &place(f), &refs, &spline, twist);
    result(op, &feature, &session, r, &Naming { segments: engine::segment_parts(p), ..Naming::default() })
}

/// A two-section ruled loft by cadrs: `profile` at z = 0, `top` (already scaled) at `height`.
pub(crate) fn loft(op: &'static str, f: &crate::Frame, p: &crate::Profile, height: f64, top: &crate::Profile) -> Result<Solid> {
    let (c0, c1) = (ctrl2(p), ctrl2(top));
    let (l0, l1) = (segs(op, p, &c0)?, segs(op, top, &c1)?);
    let (r0, r1): (Vec<&[Seg]>, Vec<&[Seg]>) = (l0.iter().map(Vec::as_slice).collect(), l1.iter().map(Vec::as_slice).collect());
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = cp::loft(&mut lock(&session), &place(f), &r0, height, &r1);
    result(op, &feature, &session, r, &Naming::default())
}

/// `profile` (drawn as for revolve) screwed by cadrs about the frame's z by `angle` radians,
/// rising `rise` and moving out `taper` mm per radian.
pub(crate) fn screw_tapered(op: &'static str, f: &crate::Frame, p: &crate::Profile, rise: f64, angle: f64, taper: f64) -> Result<Solid> {
    let ctrl = ctrl2(p);
    let loops = segs(op, p, &ctrl)?;
    let refs: Vec<&[Seg]> = loops.iter().map(Vec::as_slice).collect();
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = cp::screw_tapered(&mut lock(&session), &place(f), &refs, rise, angle, taper);
    result(op, &feature, &session, r, &Naming { segments: engine::segment_parts(p), ..Naming::default() })
}

/// The solid `patches` close, sewn by cadrs.
pub(crate) fn sewn(op: &'static str, patches: &[crate::Patch]) -> Result<Solid> {
    if patches.iter().any(|p| matches!(p, crate::Patch::Bspline { .. })) {
        return Err(unsupported(op, "a B-spline patch in a sewn set (Solid::replace_face takes one)"));
    }
    let nets: Vec<Vec<Vec<[f64; 3]>>> = patches
        .iter()
        .map(|p| match p {
            crate::Patch::Bezier { ctrl } => ctrl.iter().map(|r| r.iter().map(|q| q.to_array()).collect()).collect(),
            _ => Vec::new(),
        })
        .collect();
    let curves: Vec<Vec<Vec<[f64; 3]>>> = patches
        .iter()
        .map(|p| match p {
            crate::Patch::PlanarFill { boundary } | crate::Patch::Fill { boundary } => {
                boundary.iter().map(|s| s.ctrl().iter().map(|q| q.to_array()).collect()).collect()
            }
            _ => Vec::new(),
        })
        .collect();
    let splines: Vec<Vec<cp::Spline>> = patches
        .iter()
        .zip(&curves)
        .map(|(p, cs)| match p {
            crate::Patch::PlanarFill { boundary } | crate::Patch::Fill { boundary } => boundary
                .iter()
                .zip(cs)
                .map(|(s, c)| cp::Spline { degree: usize::try_from(s.degree()).unwrap_or(usize::MAX), knots: s.knots(), ctrl: c, weights: None })
                .collect(),
            _ => Vec::new(),
        })
        .collect();
    let glue: Vec<cp::GluePatch> = patches
        .iter()
        .zip(nets.iter().zip(&splines))
        .map(|(p, (n, s))| match p {
            crate::Patch::Bezier { .. } => cp::GluePatch::Bezier { ctrl: n },
            crate::Patch::Fill { .. } => cp::GluePatch::Fill { boundary: s },
            _ => cp::GluePatch::PlanarFill { boundary: s },
        })
        .collect();
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = cp::sewn(&mut lock(&session), &glue);
    result(op, &feature, &session, r, &Naming::default())
}

/// `s` as an AP214 STEP file, by cadrs's writer.
pub(crate) fn write_step(op: &'static str, s: &Solid) -> Result<String> {
    use yack_cadrs::Kernel as _;
    let c = of(op, s)?;
    let bytes = lock(&c.session).export_step(&[c.id]).map_err(|e| error(op, e))?;
    String::from_utf8(bytes).map_err(|_| Error::failed(op, Stage::Check, "cadrs wrote a STEP file that is not UTF-8"))
}

/// `profile` (in the frame's xy) swept by cadrs along the polyline through `corners`, mitred at
/// each corner by its bisecting plane.
pub(crate) fn sweep_mitered(op: &'static str, f: &crate::Frame, p: &crate::Profile, corners: &[crate::Point3], closed: bool) -> Result<Solid> {
    let ctrl = ctrl2(p);
    let loops = segs(op, p, &ctrl)?;
    let refs: Vec<&[Seg]> = loops.iter().map(Vec::as_slice).collect();
    let points: Vec<[f64; 3]> = corners.iter().map(|c| c.to_array()).collect();
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    // closed: two open halves from mid-run united, as cadrs's runner sweeps a closed path
    let r = if closed {
        cp::sweep_mitered_closed(&mut lock(&session), &place(f), &refs, &points)
    } else {
        cp::sweep_mitered(&mut lock(&session), &place(f), &refs, &points)
    };
    result(op, &feature, &session, r, &Naming { segments: engine::segment_parts(p), ..Naming::default() })
}

// ---- a cadrs-held solid (the lazy bridge): its measures, topology and digest, read from cadrs.

/// The digest of a cadrs-held body: its STEP DATA section (the header carries the time of
/// writing), with its face names when `named`. Deterministic: the same body, the same bytes.
pub(crate) fn held_digest(h: &Held, named: bool) -> [u8; 32] {
    use yack_cadrs::Kernel as _;
    let text = lock(&h.cadrs.session).export_step(&[h.cadrs.id]).unwrap_or_default();
    let data = text.windows(5).position(|w| w == b"DATA;").and_then(|i| text.get(i..)).unwrap_or(&text);
    let mut bytes = b"cadrs\0".to_vec();
    bytes.extend_from_slice(data);
    if named {
        for set in &h.names {
            for n in set {
                bytes.extend_from_slice(format!("{}\0{}\0", n.feature, n.face).as_bytes());
            }
            bytes.push(b'\n');
        }
    }
    engine::digest(&bytes)
}

/// Volume, area, centroid and inertia about it.
pub(crate) type Mass = (f64, f64, [f64; 3], [[f64; 3]; 3]);

/// A cadrs-held body's mass properties.
pub(crate) fn held_mass(h: &Held) -> Option<Mass> {
    use yack_cadrs::Kernel as _;
    let m = lock(&h.cadrs.session).mass_properties(h.cadrs.id).ok()?;
    let c = m.center_of_mass;
    let g = |i: usize| m.inertia.get(i).copied().unwrap_or(0.0);
    Some((m.volume, m.surface_area, [c.x, c.y, c.z], [[g(0), g(3), g(6)], [g(1), g(4), g(7)], [g(2), g(5), g(8)]]))
}

/// A cadrs-held body's bounding box, least then greatest corner.
pub(crate) fn held_bbox(h: &Held) -> Option<([f64; 3], [f64; 3])> {
    use yack_cadrs::Kernel as _;
    let b = lock(&h.cadrs.session).bounding_box(h.cadrs.id).ok()?;
    Some(([b.min.x, b.min.y, b.min.z], [b.max.x, b.max.y, b.max.z]))
}

/// A cadrs-held body's face count, the indices of its edges that are not points, and its vertex
/// count: the ids its faces, edges and vertices take.
pub(crate) fn held_counts(h: &Held) -> (usize, Vec<usize>, usize) {
    let k = lock(&h.cadrs.session);
    let Ok(b) = k.get(h.cadrs.id) else { return (0, Vec::new(), 0) };
    let edges = b.edges().iter().enumerate().filter(|(_, e)| !e.degenerate).map(|(i, _)| i).collect();
    (b.faces().len(), edges, b.verts().len())
}

/// A cadrs-held body's validity problems by cadrs's full check (empty: it was held only so).
pub(crate) fn held_check(h: &Held) -> Vec<String> {
    let k = lock(&h.cadrs.session);
    match k.get(h.cadrs.id) {
        Ok(b) => b.check_with(yack_cadrs::native::topo::CheckLevel::Full),
        Err(e) => vec![e.to_string()],
    }
}

/// A cadrs-held body's dense p-curve scan (cadrs's `yack_glue::pcurve_scan`, measure-only; for the
/// bake-off), as one line: `coedges=N over_tol=N pcurve_off=E at=(f,e,t) tol=E edge_off=E outside=E`.
#[cfg(feature = "bakeoff")]
pub(crate) fn held_pcurve_scan(h: &Held) -> String {
    let k = lock(&h.cadrs.session);
    match yack_glue::pcurve_scan(&k, h.cadrs.id) {
        Ok(s) => format!(
            "coedges={} over_tol={} pcurve_off={:.3e} at=({},{},{:.3e}) tol={:.3e} edge_off={:.3e} outside={:.3e}",
            s.coedges, s.over_tol, s.pcurve_off, s.pcurve_at.0, s.pcurve_at.1, s.pcurve_at.2, s.tol_there, s.edge_off, s.outside
        ),
        Err(e) => format!("error: {e}"),
    }
}

/// A cadrs-held body's Betti numbers (pieces, tunnels, voids): its solids, each solid's genus
/// summed over its closed shells (χ = V − E + F less each face's holes; point edges are their
/// vertex), and its void shells.
pub(crate) fn held_betti(h: &Held) -> [u64; 3] {
    let k = lock(&h.cadrs.session);
    let Ok(b) = k.get(h.cadrs.id) else { return [0; 3] };
    let (mut tunnels, mut voids) = (0u64, 0u64);
    for solid in b.solids() {
        voids += u64::try_from(solid.shells.len().saturating_sub(1)).unwrap_or(0);
        for shell in solid.shells.iter().filter_map(|&s| b.shells().get(s)) {
            let mut edges = BTreeSet::new();
            let mut verts = BTreeSet::new();
            let mut holes = 0usize;
            for face in shell.faces.iter().filter_map(|&f| b.faces().get(f)) {
                holes += face.loops.len().saturating_sub(1);
                for ce in face.loops.iter().flatten() {
                    if let Some(e) = b.edges().get(ce.edge).filter(|e| !e.degenerate) {
                        edges.insert(ce.edge);
                        verts.extend(e.v);
                    }
                }
            }
            let chi = i64::try_from(verts.len() + shell.faces.len()).unwrap_or(0) - i64::try_from(edges.len() + holes).unwrap_or(0);
            tunnels += u64::try_from((2 - chi) / 2).unwrap_or(0);
        }
    }
    [u64::try_from(b.solids().len()).unwrap_or(0), tunnels, voids]
}

// ---- one-rung cadrs constructors (w-m17's glue, as cadrs's runner builds the bench cases)

/// The profile's loops in cadrs terms, then `f` on a fresh session; the result named by segment.
fn profiled(op: &'static str, p: &crate::Profile, f: impl FnOnce(&mut DefaultKernel, &[&[Seg]]) -> yack_cadrs::Result<Made>) -> Result<Solid> {
    let ctrl = ctrl2(p);
    let loops = segs(op, p, &ctrl)?;
    let refs: Vec<&[Seg]> = loops.iter().map(Vec::as_slice).collect();
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = f(&mut lock(&session), &refs);
    result(op, &feature, &session, r, &Naming { segments: engine::segment_parts(p), ..Naming::default() })
}

/// `profile` extruded `height` along the frame's z, its sides drafted by `draft` radians
/// (positive leans them in); with `both`, mirrored below the profile's plane too.
pub(crate) fn extrude_drafted(op: &'static str, f: &crate::Frame, p: &crate::Profile, height: f64, draft: f64, both: bool) -> Result<Solid> {
    profiled(op, p, |k, l| cp::extrude_drafted(k, &place(f), l, height, draft, both))
}

/// Straight rulings from the bottom section to the top one, each drawn in its own frame.
pub(crate) fn ruled(op: &'static str, fb: &crate::Frame, pb: &crate::Profile, ft: &crate::Frame, pt: &crate::Profile) -> Result<Solid> {
    let (cb, ct) = (ctrl2(pb), ctrl2(pt));
    let (lb, lt) = (segs(op, pb, &cb)?, segs(op, pt, &ct)?);
    let rb: Vec<&[Seg]> = lb.iter().map(Vec::as_slice).collect();
    let rt: Vec<&[Seg]> = lt.iter().map(Vec::as_slice).collect();
    let session = Arc::new(Mutex::new(DefaultKernel::new()));
    let feature = engine::feature();
    let r = cp::ruled(&mut lock(&session), &place(fb), &rb, &place(ft), &rt);
    result(op, &feature, &session, r, &Naming::default())
}

/// `profile` carried along the helix about `axis` through `axis_point` that starts at the
/// frame's origin, rising `pitch` a turn, for `turns` turns.
pub(crate) fn sweep_helix(op: &'static str, f: &crate::Frame, p: &crate::Profile, axis_point: [f64; 3], axis: [f64; 3], pitch: f64, turns: f64) -> Result<Solid> {
    profiled(op, p, |k, l| cp::sweep_helix(k, &place(f), l, axis_point, axis, pitch, turns))
}

/// make::sweep with the profile turning `twist` radians and scaling to `scale` by the end.
pub(crate) fn sweep_scaled(op: &'static str, f: &crate::Frame, p: &crate::Profile, path: &[crate::PathPiece], twist: f64, scale: f64) -> Result<Solid> {
    let pieces: Vec<cp::Piece> = path
        .iter()
        .map(|x| match *x {
            crate::PathPiece::Line { length } => cp::Piece::Line { length },
            crate::PathPiece::Arc { radius, angle } => cp::Piece::Arc { radius, angle: angle.radians() },
        })
        .collect();
    profiled(op, p, |k, l| cp::sweep_law(k, &place(f), l, &pieces, twist, scale))
}

/// `profile` lofted to the point `apex` along `guides`.
pub(crate) fn loft_to_point(op: &'static str, f: &crate::Frame, p: &crate::Profile, apex: [f64; 3], guides: &[crate::Spline3]) -> Result<Solid> {
    let gctrl: Vec<Vec<[f64; 3]>> = guides.iter().map(|g| g.ctrl().iter().map(|q| q.to_array()).collect()).collect();
    let gs: Vec<cp::Spline> = guides
        .iter()
        .zip(&gctrl)
        .map(|(g, c)| cp::Spline { degree: usize::try_from(g.degree()).unwrap_or(usize::MAX), knots: g.knots(), ctrl: c, weights: None })
        .collect();
    profiled(op, p, |k, l| cp::loft_guided_to(k, &[cp::Section::Profile(place(f), l), cp::Section::Point(apex)], &gs))
}

/// cadrs's conical helix as one clamped quintic: knots, control points.
#[allow(clippy::too_many_arguments)]
pub(crate) fn conical_helix(
    op: &'static str,
    origin: [f64; 3],
    axis: [f64; 3],
    x: [f64; 3],
    r0: f64,
    r1: f64,
    height: f64,
    start: f64,
    turns: f64,
) -> Result<(Vec<f64>, Vec<[f64; 3]>)> {
    cp::conical_helix(origin, axis, x, r0, r1, height, start, turns).map_err(|e| error(op, e))
}

/// `s` with face `face` replaced by cadrs's fill of the closed chain `boundary`.
pub(crate) fn replace_face_fill(op: &'static str, s: &Solid, face: u64, boundary: &[crate::Spline3]) -> Result<Solid> {
    let ctrl: Vec<Vec<[f64; 3]>> = boundary.iter().map(|g| g.ctrl().iter().map(|q| q.to_array()).collect()).collect();
    let curves: Vec<cp::Spline> = boundary
        .iter()
        .zip(&ctrl)
        .map(|(g, c)| cp::Spline { degree: usize::try_from(g.degree()).unwrap_or(usize::MAX), knots: g.knots(), ctrl: c, weights: None })
        .collect();
    self::op(op, s, |k, id| yack_glue::replace_face_fill(k, id, face, &curves))
}

// ---- face and edge selection on a held solid (w-m17's pick glue, read-only on cadrs's body)

use yack_cadrs::facade::pick as cpick;
pub(crate) use yack_cadrs::facade::pick::Picked;

/// A held face's facts, in yack's terms.
pub(crate) struct HeldFace {
    pub(crate) kind: crate::SurfaceKind,
    pub(crate) normal: Option<crate::Vec3>,
    pub(crate) area: f64,
    pub(crate) centroid: crate::Point3,
    pub(crate) bbox: crate::Aabb,
}

/// A held edge's facts, in yack's terms.
pub(crate) struct HeldEdge {
    pub(crate) kind: crate::CurveKind,
    pub(crate) ends: [crate::Point3; 2],
    pub(crate) convex: Option<bool>,
    pub(crate) length: f64,
    pub(crate) midpoint: crate::Point3,
    pub(crate) uses: usize,
}

/// The held face within `tol` of `p` (cadrs's pick: one, none, or ambiguous).
pub(crate) fn held_face_pick(op: &'static str, h: &Held, p: crate::Point3, tol: f64) -> Result<Picked> {
    cpick::face_pick(&lock(&h.cadrs.session), h.cadrs.id, p.to_array(), tol).map_err(|e| error(op, e))
}

/// The held edge within `tol` of `p`.
pub(crate) fn held_edge_pick(op: &'static str, h: &Held, p: crate::Point3, tol: f64) -> Result<Picked> {
    cpick::edge_pick(&lock(&h.cadrs.session), h.cadrs.id, p.to_array(), tol).map_err(|e| error(op, e))
}

/// Held face `face`'s kind, plane normal, area, centroid and box.
pub(crate) fn held_face_facts(op: &'static str, h: &Held, face: u64) -> Result<HeldFace> {
    use crate::SurfaceKind as K;
    let f = cpick::face_info(&lock(&h.cadrs.session), h.cadrs.id, face).map_err(|e| error(op, e))?;
    let v = crate::Vec3::from;
    let (kind, normal) = match f.kind {
        cpick::FaceKind::Plane { normal } => (K::Plane { normal: v(normal) }, Some(v(normal))),
        cpick::FaceKind::Cylinder { axis: (_, d), radius } => (K::Cylinder { axis: v(d), radius }, None),
        cpick::FaceKind::Cone { .. } => (K::Cone, None),
        cpick::FaceKind::Sphere { radius, .. } => (K::Sphere { radius }, None),
        cpick::FaceKind::Torus { .. } => (K::Torus, None),
        cpick::FaceKind::Swept => (K::Swept, None),
        cpick::FaceKind::Spline => (K::Spline, None),
    };
    let (lo, hi) = f.bbox;
    Ok(HeldFace {
        kind,
        normal,
        area: f.area,
        centroid: crate::Point3::from(f.centroid),
        bbox: crate::Aabb::new(crate::Point3::from(lo), crate::Point3::from(hi)),
    })
}

/// Held edge `edge`'s kind, ends, convexity, length, midpoint and face uses.
pub(crate) fn held_edge_facts(op: &'static str, h: &Held, edge: u64) -> Result<HeldEdge> {
    use crate::CurveKind as K;
    let e = cpick::edge_info(&lock(&h.cadrs.session), h.cadrs.id, edge).map_err(|e| error(op, e))?;
    let v = crate::Vec3::from;
    let kind = match e.kind {
        cpick::EdgeKind::Line { direction } => K::Line { direction: v(direction) },
        cpick::EdgeKind::Circle { radius, axis: (_, n) } => K::Circle { radius, axis: v(n) },
        cpick::EdgeKind::Ellipse => K::Ellipse,
        cpick::EdgeKind::Spline => K::Spline,
    };
    let (a, b) = e.ends;
    Ok(HeldEdge {
        kind,
        ends: [crate::Point3::from(a), crate::Point3::from(b)],
        convex: e.convex,
        length: e.length,
        midpoint: crate::Point3::from(e.mid),
        uses: e.uses,
    })
}

/// The outward normal of held face `face`'s own surface at the point of it nearest `p`.
pub(crate) fn held_face_normal_at(h: &Held, face: u64, p: crate::Point3) -> Option<crate::Vec3> {
    cpick::face_normal_at(&lock(&h.cadrs.session), h.cadrs.id, face, p.to_array()).ok().flatten().map(crate::Vec3::from)
}

/// The nearest point of held face `face` to `p`.
pub(crate) fn held_face_closest(h: &Held, face: u64, p: crate::Point3) -> Result<crate::Point3> {
    cpick::face_closest(&lock(&h.cadrs.session), h.cadrs.id, face, p.to_array())
        .map(crate::Point3::from)
        .map_err(|e| error("FaceRef::closest", e))
}

/// An even `nu` × `nv` grid over held face `face`'s parameter box, rows along u (off-face points kept).
pub(crate) fn held_face_grid(h: &Held, face: u64, nu: usize, nv: usize) -> Vec<Vec<crate::Point3>> {
    cpick::face_grid(&lock(&h.cadrs.session), h.cadrs.id, face, nu, nv)
        .map(|g| g.into_iter().map(|r| r.into_iter().map(crate::Point3::from).collect()).collect())
        .unwrap_or_default()
}

/// Held edge `edge`'s curve parameter range.
pub(crate) fn held_edge_range(h: &Held, edge: u64) -> [f64; 2] {
    cpick::edge_range(&lock(&h.cadrs.session), h.cadrs.id, edge).unwrap_or([0.0, 0.0])
}

/// The point of held edge `edge`'s curve at parameter `t`.
pub(crate) fn held_edge_point(h: &Held, edge: u64, t: f64) -> Option<crate::Point3> {
    cpick::edge_point(&lock(&h.cadrs.session), h.cadrs.id, edge, t).ok().map(crate::Point3::from)
}

/// `n` points even in held edge `edge`'s parameter, ends included.
pub(crate) fn held_edge_points(h: &Held, edge: u64, n: usize) -> Vec<crate::Point3> {
    cpick::edge_points(&lock(&h.cadrs.session), h.cadrs.id, edge, n)
        .map(|ps| ps.into_iter().map(crate::Point3::from).collect())
        .unwrap_or_default()
}

/// A held solid's closed, outward, welded mesh within `chord` mm of it (as springrep's).
pub(crate) fn held_mesh(op: &'static str, h: &Held, chord: f64) -> Result<(Vec<crate::Point3>, Vec<[u32; 3]>)> {
    let (positions, triangles) = yack_glue::mesh(&lock(&h.cadrs.session), h.cadrs.id, chord).map_err(|e| error(op, e))?;
    Ok((positions.into_iter().map(crate::Point3::from).collect(), triangles))
}

/// One face of a held solid's display mesh: positions, outward normals, triangles.
pub(crate) type HeldDisplayFace = (Vec<crate::Point3>, Vec<crate::Vec3>, Vec<[u32; 3]>);

/// A held solid as a viewer draws it, faces and edges within `chord` mm (as springrep's).
pub(crate) fn held_display(op: &'static str, h: &Held, chord: f64) -> Result<(Vec<HeldDisplayFace>, Vec<Vec<crate::Point3>>)> {
    let (faces, edges) = yack_glue::display(&lock(&h.cadrs.session), h.cadrs.id, chord, chord).map_err(|e| error(op, e))?;
    let p = |ps: Vec<[f64; 3]>| ps.into_iter().map(crate::Point3::from).collect::<Vec<_>>();
    Ok((
        faces.into_iter().map(|(ps, ns, ts)| (p(ps), ns.into_iter().map(crate::Vec3::from).collect(), ts)).collect(),
        edges.into_iter().map(p).collect(),
    ))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::{Frame, make};

    /// The cadrs-side Betti count against springrep's on solids that cross the bridge.
    #[test]
    fn held_betti_agrees_with_springrep() {
        let torus = make::torus(&Frame::WORLD, 5.0, 1.0).unwrap();
        let block = make::block(&Frame::WORLD, [10.0, 10.0, 10.0]).unwrap();
        let drilled = block.difference(&make::cylinder(&Frame::at([5.0, 5.0, -1.0]), 2.0, 12.0).unwrap()).unwrap();
        let hollow = make::sphere(&Frame::WORLD, 5.0).unwrap().difference(&make::sphere(&Frame::WORLD, 3.0).unwrap()).unwrap();
        for (s, want) in [(&torus, [1, 1, 0]), (&drilled, [1, 1, 0]), (&hollow, [1, 0, 1])] {
            assert_eq!(s.betti(), want);
            let held = Held { cadrs: of("test", s).unwrap(), names: Vec::new(), why: String::new() };
            assert_eq!(held_betti(&held), want, "cadrs count on {}", s.strategy());
        }
    }
}
