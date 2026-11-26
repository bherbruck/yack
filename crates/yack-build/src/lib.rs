// yack: building solids: primitives, sweeps and lofts, and the raw-to-arena builder with its healing
// (M3 W7; heal and adjust stay here until yack-heal takes them, D26).
//! Building solids: `primitive`, `sweep` and `loft` (PLAN §3's build), and `build`, the raw-to-arena
//! builder the STEP reader, the blends and the affine edit share, with its healing for now.


// springrep's foundation modules (D26), under the names the moved code uses.
use yack_base::kernel::Error;
use yack_base::kernel::Result;
use yack_check as check;
use yack_base::kernel as error;
use yack_geom as geom;
use yack_intersect as intersect;
use yack_math::kernel as math;
use yack_numeric as numeric;
use yack_profile as profile;
use yack_tol as tol;
use yack_topo as topo;
#[cfg(test)]
use yack_measure as measure;
pub mod build;
pub mod loft;
pub mod primitive;
pub mod ruled;
pub mod spindle;
pub mod drafted;
pub mod sweep;
