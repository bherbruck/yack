// yack: the M1 strategy ladder over the two temporary engines (PLAN §3.5, DECISIONS D9, M1.10).
//
// Rules: a fixed order per op; springrep's `Invalid` (and NotFound/Ambiguous/ForeignId, and a
// caught panic, `Internal`) is final, cadrs's `Invalid` falls through; Unsupported/Failed/Budget declines and the next rung runs; the first
// result wins (each engine's result has passed its own checker: springrep's ops end in its check,
// and a cadrs result is carried back only through the bridge, which checks it). When every rung
// declined: Unsupported if all were, else Failed with every rung's reason.
//
// Soft guards (a springrep result doubted on inputs it mostly gets right) only redirect: a doubted
// result is held and returned if every later rung fails or panics; they never turn a springrep
// answer into a refusal or a crash. Hard guards are only for input classes where brep-bench
// measured springrep wrong (named at each), since a wrong answer is never acceptable.
use crate::{Error, ErrorKind, Result, Solid};

/// One strategy: its name and how it builds the op's result.
pub(crate) struct Rung<'a> {
    pub(crate) name: &'static str,
    pub(crate) run: &'a dyn Fn() -> Result<Solid>,
}

/// Ops cadrs runs first until the bake-off (doc/donors.md) orders them; springrep first elsewhere.
/// (chamfer went back to springrep first: cadrs-first lost 77 chamfer-prism conformance cases.)
const CADRS_FIRST: &[&str] =
    &["swept", "delete_faces", "move_faces", "offset_faces", "shell", "shell_outward", "thicken", "draft", "fillet_variable"];

/// The springrep refusals that mean "this request is invalid" in the protocol's sense, by op
/// (`"*"`: any op) and a phrase of springrep's message. Only these are final; every other
/// springrep `Invalid` is springrep stating its own limit ("extrusion distance must be nonzero"
/// inside a delete, "an offset closes a torus to nothing", a draft that "turns inside out"), and
/// falls through so cadrs gets a turn (if cadrs fails too, the answer is still `Invalid`).
/// Derived from springrep's refusal texts in the conformance invalid-* families (runs/yack-m1-seed0..3).
const INVALID_PER_PROTOCOL: &[(&str, &str)] = &[
    ("*", "must be positive"),
    ("*", "takes a whole face"),
    ("*", "runs past the end of an edge"),
    ("*", "does not fit on its faces"),
    ("fillet", "the faces are parallel"),
    ("cone", "needs a positive height"),
    ("extrude", "must be nonzero"),
    ("loft_through", "do not come in order"),
    ("revolve", "axis crosses the profile"),
    ("screw", "coils met"),
    ("torus", "clear of its axis"),
    ("offset_faces", "turns inside out"),
    ("move_faces", "turns inside out"),
    ("shell", "turns inside out"),
];

/// Whether springrep's `Invalid` refusal `e` from `op` is the protocol's invalid (final).
fn invalid_per_protocol(op: &str, e: &Error) -> bool {
    INVALID_PER_PROTOCOL.iter().any(|&(o, phrase)| (o == "*" || o == op) && e.detail.contains(phrase))
}

/// Whether cadrs leads `op`'s ladder.
pub(crate) fn cadrs_first(op: &str) -> bool {
    CADRS_FIRST.contains(&op)
}

/// The springrep and cadrs rungs in `op`'s order. Validation comes first in every op (PLAN §4.2
/// rule 7), and springrep's is the reference: where cadrs leads and builds, springrep still runs,
/// and its `Invalid` vetoes the result (cadrs builds from some inputs the contract calls invalid:
/// a shell as thick as the part, a chamfer too big for its faces, a step raised past the next).
pub(crate) fn two<'a>(op: &'static str, springrep: &'a dyn Fn() -> Result<Solid>, cadrs: &'a dyn Fn() -> Result<Solid>) -> Result<Solid> {
    let (s, c) = (Rung { name: "springrep", run: springrep }, Rung { name: "cadrs", run: cadrs });
    if !cadrs_first(op) {
        return ladder(op, &[s, c]);
    }
    let built = ladder(op, &[c, s])?;
    if built.strategy() == "cadrs" {
        #[cfg(feature = "bakeoff")]
        if crate::bakeoff::forced() == crate::bakeoff::Engine::Cadrs {
            return Ok(built);
        }
        if let Err(e) = springrep()
            && is_final(op, "springrep", &e)
            && e.kind == ErrorKind::Invalid
        {
            return Err(e);
        }
    }
    Ok(built)
}

/// Whether a rung's refusal ends the ladder. springrep's validation is the reference: its
/// `Invalid` (and an id or pick refusal, which comes before any engine) is final; cadrs's
/// `Invalid` falls through, as cadrs refuses some inputs springrep builds. A caught panic is
/// always final.
fn is_final(op: &str, rung: &str, e: &Error) -> bool {
    match e.kind {
        ErrorKind::Internal | ErrorKind::NotFound | ErrorKind::Ambiguous | ErrorKind::ForeignId => true,
        ErrorKind::Invalid => rung == "springrep" && invalid_per_protocol(op, e),
        _ => false,
    }
}

/// Runs `rungs` in order (see the module comment). Each rung runs behind its own panic
/// firewall. A result a guard doubts ([`Solid::suspected`]) is held while the later rungs run:
/// the first clean result wins; if every later rung fails (or panics), the held result is the
/// answer. Guards redirect; they never turn a springrep answer into a refusal or a crash.
pub(crate) fn ladder(op: &'static str, rungs: &[Rung]) -> Result<Solid> {
    let mut declined: Vec<(&'static str, Error)> = Vec::new();
    let mut held: Option<Solid> = None;
    for rung in rungs {
        #[cfg(feature = "bakeoff")]
        if !crate::bakeoff::allows(rung.name) {
            declined.push((rung.name, Error::new(ErrorKind::Unsupported, op, format!("bakeoff: {} not forced", rung.name))));
            continue;
        }
        match crate::firewall::run(op, rung.run) {
            Ok(s) if s.suspected().is_some() && held.is_none() => held = Some(s.with_strategy(rung.name)),
            Ok(s) if s.suspected().is_some() => {}
            Ok(s) => return Ok(s.with_strategy(rung.name)),
            Err(e) if is_final(op, rung.name, &e) => {
                return match held {
                    Some(h) => Ok(h.trusted()),
                    None => Err(e),
                };
            }
            Err(e) => declined.push((rung.name, e)),
        }
    }
    if let Some(h) = held {
        return Ok(h.trusted());
    }
    if let [(_, only)] = declined.as_slice() {
        return Err(only.clone());
    }
    let detail = declined.iter().map(|(n, e)| format!("{n}: {e}")).collect::<Vec<_>>().join("; ");
    if let Some((_, invalid)) = declined.iter().find(|(_, e)| e.kind == ErrorKind::Invalid) {
        return Err(Error { detail, ..invalid.clone() });
    }
    if declined.iter().all(|(_, e)| e.kind == ErrorKind::Unsupported) {
        Err(Error::new(ErrorKind::Unsupported, op, detail))
    } else {
        let stage = declined.iter().find_map(|(_, e)| (e.kind != ErrorKind::Unsupported).then_some(e.stage)).flatten();
        Err(Error { kind: ErrorKind::Failed, op, arg: None, stage, detail })
    }
}

/// An op only springrep builds, still through the ladder (so `bakeoff::force(Cadrs)` refuses it
/// rather than letting springrep build behind the switch), behind the panic firewall.
pub(crate) fn springrep_only(op: &'static str, run: impl Fn() -> Result<Solid>) -> Result<Solid> {
    crate::firewall::run(op, || ladder(op, &[Rung { name: "springrep", run: &run }]))
}

/// Whether the engine `name` may run (always, unless the bake-off forced another): for ops whose
/// result is not one solid.
pub(crate) fn allowed(op: &'static str, name: &'static str) -> Result<()> {
    #[cfg(feature = "bakeoff")]
    if !crate::bakeoff::allows(name) {
        return Err(Error::new(ErrorKind::Unsupported, op, format!("bakeoff: {name} not forced")));
    }
    let _ = (op, name);
    Ok(())
}
