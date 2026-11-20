// yack: curve–surface and surface–surface intersection (PLAN §3.4 T2).
//! Curve–surface and curve–curve intersection (`csi`, with its convex `hulls`), surface–surface
//! intersection (`ssi`), its marching (`march`) and contact tests (`contact`). Moved out of
//! `boolean/` in M2.1 (T2) so that `check`, `tweak`, `sweep` and `rays` need not reach into the
//! boolean.

pub(crate) mod contact;
pub(crate) mod csi;
mod hulls;
pub(crate) mod march;
pub(crate) mod ssi;
