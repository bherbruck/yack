//! A frustum capped by a ball, its rim chamfered by a distance of 1 and an angle of 60°.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    Err(Error::unsupported("chamfer/capped-frustum-rim-distance-1-angle-60"))
}
