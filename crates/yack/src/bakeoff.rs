// yack: tool-only bake-off controls (feature `bakeoff`, PLAN §3.5, M1.10). Never on by default.
//! Bake-off controls for the benchmark runners: force a single engine for every op on this
//! thread, and report how a solid survives the bridge's round trip. Only with the `bakeoff`
//! feature, which tool crates enable and the published crate never does. The forced engine is
//! thread-local state: the one exception to "no hidden state", allowed only here (gate-exempt
//! under this feature).
use std::cell::{Cell, RefCell};

use crate::Solid;

/// Which engines the ladder may use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Engine {
    /// Every rung, in the ladder's order (the default).
    Both,
    /// springrep's rungs only.
    Springrep,
    /// cadrs's rungs only.
    Cadrs,
    /// yack's own rungs only (the `native` ones: yack-swept's, yack-blend's).
    Native,
}

thread_local! {
    static FORCED: Cell<Engine> = const { Cell::new(Engine::Both) };
    /// The one op the forced engine applies to, if [`force_op`] narrowed it.
    static ONLY_OP: RefCell<Option<String>> = const { RefCell::new(None) };
}

/// From now on, on this thread, run only `e`'s rungs (`Engine::Both` restores the ladder).
///
/// Thread-local, and that is enough: the ladder picks its rung in the facade, on the calling
/// thread, before an engine fans any work out to threads (springrep's `par_map`, cadrs's scoped
/// threads), so the forced engine holds for the whole op. Call it on the thread that runs the
/// case.
pub fn force(e: Engine) {
    FORCED.with(|f| f.set(e));
    if e == Engine::Both {
        ONLY_OP.with(|o| *o.borrow_mut() = None);
    }
}

/// From now on, on this thread, run only `e`'s rungs for the op named `op` (`"swept"`,
/// `"chamfer_angle"`), and every other op's whole ladder: a bake-off of one op whose operands
/// come from the default ladder, the same for each engine compared. `force(Engine::Both)` undoes it.
pub fn force_op(op: &str, e: Engine) {
    ONLY_OP.with(|o| *o.borrow_mut() = Some(op.to_owned()));
    FORCED.with(|f| f.set(e));
}

/// The engine forced on this thread.
pub fn forced() -> Engine {
    FORCED.with(Cell::get)
}

/// Whether the rung named `name` of op `op` may run.
pub(crate) fn allows(op: &str, name: &str) -> bool {
    if ONLY_OP.with(|o| o.borrow().as_deref().is_some_and(|only| only != op)) {
        return true;
    }
    match forced() {
        Engine::Both => true,
        Engine::Springrep => name == "springrep",
        Engine::Cadrs => name == "cadrs",
        Engine::Native => name == "native",
    }
}

/// The ladder rung (engine) that built `s`, for attributing a case to its builder; it may differ
/// from [`Solid::strategy`], the engine whose form answers `s`'s queries.
pub fn built_by(s: &Solid) -> &'static str {
    s.built_by()
}

/// How `s` survives springrep → cadrs → springrep through the bridge: empty when it comes back
/// alike, else one line per problem, each starting with a stable cause prefix (`to-cadrs:`,
/// `back:`, `order:`, `volume:`, `area:`, `face-area:`, `classify:`).
pub fn roundtrip(s: &Solid) -> Vec<String> {
    match s.brep() {
        None if let Some(h) = s.held() => vec![format!("back: held in cadrs form only ({})", h.why)],
        None => Vec::new(),
        Some(b) => yack_bridge::round_trip(b).problems,
    }
}
