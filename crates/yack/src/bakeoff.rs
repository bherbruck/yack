// yack: tool-only bake-off controls (feature `bakeoff`, PLAN §3.5, M1.10). Never on by default.
//! Bake-off controls for the benchmark runners: force a single engine for every op on this
//! thread, and report how a solid survives the bridge's round trip. Only with the `bakeoff`
//! feature, which tool crates enable and the published crate never does. The forced engine is
//! thread-local state: the one exception to "no hidden state", allowed only here (gate-exempt
//! under this feature).
use std::cell::Cell;

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
}

thread_local! {
    static FORCED: Cell<Engine> = const { Cell::new(Engine::Both) };
}

/// From now on, on this thread, run only `e`'s rungs (`Engine::Both` restores the ladder).
pub fn force(e: Engine) {
    FORCED.with(|f| f.set(e));
}

/// The engine forced on this thread.
pub fn forced() -> Engine {
    FORCED.with(Cell::get)
}

/// Whether the rung named `name` may run.
pub(crate) fn allows(name: &str) -> bool {
    match forced() {
        Engine::Both => true,
        Engine::Springrep => name == "springrep",
        Engine::Cadrs => name == "cadrs",
    }
}

/// How `s` survives springrep → cadrs → springrep through the bridge: empty when it comes back
/// alike, else one line per problem, each starting with a stable cause prefix (`to-cadrs:`,
/// `back:`, `order:`, `volume:`, `area:`, `face-area:`, `classify:`).
pub fn roundtrip(s: &Solid) -> Vec<String> {
    match s.brep() {
        None => Vec::new(),
        Some(b) => yack_bridge::round_trip(b).problems,
    }
}
