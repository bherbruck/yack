// yack: the M1 strategy ladder over the two temporary engines (PLAN §3.5, DECISIONS D9, M1.10).
//
// Rules: a fixed order per op; springrep's `Invalid` (and NotFound/Ambiguous/ForeignId, and a
// caught panic, `Internal`) is final, cadrs's `Invalid` falls through; Unsupported/Failed/Budget declines and the next rung runs; the first
// result wins (each engine's result has passed its own checker: springrep's ops end in its check,
// and a cadrs result is carried back only through the bridge, which checks it). When every rung
// declined: Unsupported if all were, else Failed with every rung's reason.
use crate::{Error, ErrorKind, Result, Solid};

/// One strategy: its name and how it builds the op's result.
pub(crate) struct Rung<'a> {
    pub(crate) name: &'static str,
    pub(crate) run: &'a dyn Fn() -> Result<Solid>,
}

/// Ops cadrs runs first until the bake-off (doc/donors.md) orders them; springrep first elsewhere.
/// (chamfer went back to springrep first: cadrs-first lost 77 chamfer-prism conformance cases.)
const CADRS_FIRST: &[&str] =
    &["swept", "delete_faces", "move_faces", "offset_faces", "offset", "shell", "shell_outward", "thicken", "draft"];

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

/// Whether a rung's refusal ends the ladder. springrep's validation is the reference: its
/// `Invalid` (and an id or pick refusal, which comes before any engine) is final; cadrs's
/// `Invalid` falls through, as cadrs refuses some inputs springrep builds. A caught panic is
/// always final.
fn is_final(rung: &str, kind: ErrorKind) -> bool {
    match kind {
        ErrorKind::Internal | ErrorKind::NotFound | ErrorKind::Ambiguous | ErrorKind::ForeignId => true,
        ErrorKind::Invalid => rung == "springrep",
        _ => false,
    }
}

/// Runs `rungs` in order (see the module comment).
pub(crate) fn ladder(op: &'static str, rungs: &[Rung]) -> Result<Solid> {
    let mut declined: Vec<(&'static str, Error)> = Vec::new();
    for rung in rungs {
        #[cfg(feature = "bakeoff")]
        if !crate::bakeoff::allows(rung.name) {
            declined.push((rung.name, Error::new(ErrorKind::Unsupported, op, format!("bakeoff: {} not forced", rung.name))));
            continue;
        }
        match (rung.run)() {
            Ok(s) => return Ok(s.with_strategy(rung.name)),
            Err(e) if is_final(rung.name, e.kind) => return Err(e),
            Err(e) => declined.push((rung.name, e)),
        }
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
