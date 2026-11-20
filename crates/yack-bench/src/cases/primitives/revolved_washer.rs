//! A washer.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    revolve(&Frame::WORLD, &Profile::rectangle([6.0, 0.0], [10.0, 4.0])?, deg(360.0))
}
