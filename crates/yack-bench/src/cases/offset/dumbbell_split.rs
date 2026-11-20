//! A dumbbell shrunk until it splits into two bodies.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let outline = [[0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [14.0, 4.0], [14.0, 0.0], [24.0, 0.0]];
    let back = [[24.0, 10.0], [14.0, 10.0], [14.0, 6.0], [10.0, 6.0], [10.0, 10.0], [0.0, 10.0]];
    extrude(&Frame::WORLD, &Profile::polygon(&[outline, back].concat())?, 6.0)?.offset(-1.5)
}
