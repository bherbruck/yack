// yack: brep-bench case scale/balls-united-a-million-from-the-origin
//! Two balls united a million from the origin.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::at([600000.0, -800000.0, 0.0]), 5.0)?.union(&sphere(&Frame::at([600002.0, -799997.0, 6.0]), 4.0)?)
}
