// yack: What an operation knows by construction about how the two arenas it hands a boolean meet, so
// Adapted from springrep@a03e8a1 src/topo/facts.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
//! What an operation knows by construction about how the two arenas it hands a boolean meet, so
//! the boolean takes it instead of computing it. The boolean checks every fact and refuses one that
//! does not hold.

use super::{EdgeId, FaceId};

/// Edge `edge` of input `side` (0 or 1) lies in the surface of face `face` of the other input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OnFace {
    pub side: usize,
    pub edge: EdgeId,
    pub face: FaceId,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Facts {
    pub on_face: Vec<OnFace>,
}
