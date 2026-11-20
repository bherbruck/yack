//! Three bodies nested in each other's hollows.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let outer = sphere(&Frame::WORLD, 10.0)?.difference(&sphere(&Frame::at([1.0, 0.0, 0.0]), 8.0)?)?;
    let middle = sphere(&Frame::at([1.0, 1.0, 0.0]), 6.0)?.difference(&sphere(&Frame::at([1.0, 1.0, 1.0]), 4.0)?)?;
    outer.union(&middle)?.union(&sphere(&Frame::at([1.0, 1.5, 1.5]), 2.0)?)
}
