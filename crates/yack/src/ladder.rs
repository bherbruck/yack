// yack: the M1 strategy ladder over the two temporary engines (PLAN §3.5, DECISIONS D9, M1.10).
//
// Rules: a fixed order per op; `Invalid` (and NotFound/Ambiguous/ForeignId, and a caught panic,
// `Internal`) is final; Unsupported/Failed/Budget declines and the next rung runs; the first
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
const CADRS_FIRST: &[&str] =
    &["swept", "delete_faces", "move_faces", "offset_faces", "offset", "shell", "shell_outward", "thicken", "draft", "chamfer"];

/// Whether cadrs leads `op`'s ladder.
pub(crate) fn cadrs_first(op: &str) -> bool {
    CADRS_FIRST.contains(&op)
}

/// The springrep and cadrs rungs in `op`'s order.
pub(crate) fn two<'a>(op: &'static str, springrep: &'a dyn Fn() -> Result<Solid>, cadrs: &'a dyn Fn() -> Result<Solid>) -> Result<Solid> {
    let (s, c) = (Rung { name: "springrep", run: springrep }, Rung { name: "cadrs", run: cadrs });
    if cadrs_first(op) { ladder(op, &[c, s]) } else { ladder(op, &[s, c]) }
}

fn is_final(kind: ErrorKind) -> bool {
    matches!(kind, ErrorKind::Invalid | ErrorKind::NotFound | ErrorKind::Ambiguous | ErrorKind::ForeignId | ErrorKind::Internal)
}

/// Runs `rungs` in order (see the module comment).
pub(crate) fn ladder(op: &'static str, rungs: &[Rung]) -> Result<Solid> {
    let mut declined: Vec<(&'static str, Error)> = Vec::new();
    for rung in rungs {
        match (rung.run)() {
            Ok(s) => return Ok(s.with_strategy(rung.name)),
            Err(e) if is_final(e.kind) => return Err(e),
            Err(e) => declined.push((rung.name, e)),
        }
    }
    if let [(_, only)] = declined.as_slice() {
        return Err(only.clone());
    }
    let detail = declined.iter().map(|(n, e)| format!("{n}: {e}")).collect::<Vec<_>>().join("; ");
    if declined.iter().all(|(_, e)| e.kind == ErrorKind::Unsupported) {
        Err(Error::new(ErrorKind::Unsupported, op, detail))
    } else {
        let stage = declined.iter().find_map(|(_, e)| (e.kind != ErrorKind::Unsupported).then_some(e.stage)).flatten();
        Err(Error { kind: ErrorKind::Failed, op, arg: None, stage, detail })
    }
}
