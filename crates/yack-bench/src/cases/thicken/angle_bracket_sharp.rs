// yack: brep-bench case thicken/angle-bracket-sharp
//! A creased sheet thickened outside its crease, the corner kept sharp.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let b = block(&Frame::WORLD, [10.0, 8.0, 10.0])?;
    b.thicken(&b.faces_at(&[[5.0, 4.0, 0.0], [0.0, 4.0, 5.0]])?, 1.0, Join::Sharp)
}
