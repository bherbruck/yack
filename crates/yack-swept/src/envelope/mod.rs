// yack: the envelope strategy for swept solids: what a body passes through, from its own surface (M4, w-h6).
//! The envelope strategy: the solid a moving body sweeps is the body at the start together with what
//! the part of its surface that the motion carries outward sweeps. That part is bounded by its
//! characteristic curves, where the velocity lies in the surface, and by the sharp edges between it
//! and the rest.
//!
//! Not yet routed: nothing in `STRATEGIES` names it (lead's order for step W1), so each module is
//! reachable only from its own tests until the strategy is wired in.
#![allow(dead_code)] // unrouted until the strategy lands (w-h6 W1)

pub(crate) mod motion;
pub(crate) mod grazing;
