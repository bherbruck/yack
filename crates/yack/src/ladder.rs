// yack: the M1 strategy ladder over the two temporary engines (PLAN §3.5, DECISIONS D9, M1.10).
//
// Rules: a fixed order per op; `Invalid` (and NotFound/Ambiguous/ForeignId, and a caught panic,
// `Internal`) is final; Unsupported/Failed/Budget declines and the next rung runs; the first
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
            && e.kind == ErrorKind::Invalid
        {
            return Err(e);
        }
    }
    Ok(built)
}

/// Whether a rung's refusal ends the ladder: `Invalid` (the input breaks a stated requirement of
/// the op, as each engine's error map decides), an id or pick refusal, or a caught panic.
fn is_final(e: &Error) -> bool {
    matches!(e.kind, ErrorKind::Invalid | ErrorKind::Internal | ErrorKind::NotFound | ErrorKind::Ambiguous | ErrorKind::ForeignId)
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
            Err(e) if is_final(&e) => {
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
