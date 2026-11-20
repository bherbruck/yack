//! Two coaxial tori, their common part.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    torus(&Frame::at([0.0, 0.0, -1.0]), 5.0, 2.0)?.intersection(&torus(&Frame::at([0.0, 0.0, 1.0]), 5.0, 2.0)?)
}
