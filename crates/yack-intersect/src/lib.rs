// yack: curve–surface and surface–surface intersection (PLAN §3.4 T2).
//! Curve–surface and curve–curve intersection (`csi`, with its convex `hulls`), surface–surface
//! intersection (`ssi`), its marching (`march`) and contact tests (`contact`). Moved out of
//! `boolean/` in M2.1 (T2) so that `check`, `tweak`, `sweep` and `rays` need not reach into the
//! boolean.

// springrep's foundation modules (D26), under the names intersect's code uses.
use yack_base::kernel::Error;
use yack_base::kernel::Result;
use yack_geom as geom;
use yack_math::kernel as math;
use yack_numeric as numeric;
use yack_tol as tol;
pub mod contact;
pub mod csi;
mod hulls;
pub(crate) mod march;
pub mod ssi;
