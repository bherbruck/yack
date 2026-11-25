// yack: the swept solid as the start united with what each of its leading faces sweeps (M4.1 step 1).
// Adapted from cadrs@eb5899ef crates/cadrs_kernel/src/native/sweep_faces.rs (MIT OR Apache-2.0, (c) 2026 Rouan van der Ende).
// Changes: rewritten onto yack's crates (yack-topo faces, yack-boolean's join, yack-heal), not ported;
//   in this first commit only the strategy's place in the ladder, declining every motion.
//! What any solid passes through as it slides, turns or screws: the solid where it starts, with
//! what each of its *leading* faces sweeps (a face whose outward normal leads the motion), each
//! face split first along its contact so that every piece leads, trails or runs along itself.

use yack_base::kernel::{Error, Result};
use yack_topo::Brep;

use crate::Motion;

/// [`crate::swept`] by faces. Not built yet: declines every motion.
pub(crate) fn swept(_body: &Brep, _motion: &Motion) -> Result<Brep> {
    Err(Error::Unsupported("by faces: not built yet".into()))
}
