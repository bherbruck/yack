// yack: a solid seen from outside: hidden-line drawings, overlaps and sweeps between bodies, and its
// thin places (M3 W9d's crate root over springrep's draw, overlap and probe).
//! A solid seen from outside: [`draw`]'s hidden-line drawings, [`overlap`]'s overlaps, meetings and
//! swept contacts between bodies, and [`probe`]'s thin places.

// springrep's view code indexes by position; D26's hardening pass moves it onto checked access,
// as for yack-linalg until M5.
#![allow(clippy::indexing_slicing)]

// springrep's foundation modules (D26), under the names the moved code uses.
use yack_base::kernel::Result;
use yack_boolean as boolean;
use yack_locate::distance;
use yack_base::kernel as error;
use yack_geom as geom;
use yack_math::kernel as math;
use yack_measure as measure;
use yack_mesh as mesh;
use yack_numeric as numeric;
use yack_tol as tol;
use yack_topo as topo;
#[cfg(test)]
use yack_build::primitive;
#[cfg(test)]
use yack_profile as profile;
#[cfg(test)]
use yack_build::sweep;

pub mod draw;
pub mod overlap;
pub mod probe;
