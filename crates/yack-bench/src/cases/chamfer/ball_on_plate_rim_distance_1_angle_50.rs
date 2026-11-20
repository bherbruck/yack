//! A ball on a plate, the concave edge between them chamfered by a distance of 1 and an angle of
//! 50°.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    Err(Error::unsupported("chamfer/ball-on-plate-rim-distance-1-angle-50"))
}
