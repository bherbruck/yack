//! A lens of two unequal balls, one placed aslant.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    sphere(&Frame::WORLD, 5.511)?.intersection(&sphere(&Frame::at([4.231, 3.467, 2.653]), 3.317)?)
}
