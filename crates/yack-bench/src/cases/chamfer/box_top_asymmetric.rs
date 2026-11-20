//! A box with its top edges chamfered unequally.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let rim = [[10.0, 10.0, 6.0], [20.0, 5.0, 6.0], [10.0, 0.0, 6.0], [0.0, 5.0, 6.0]];
    rim.iter().try_fold(block(&Frame::WORLD, [20.0, 10.0, 6.0])?, |s, &p| {
        s.chamfer_two(s.edge_at(p)?, s.face_at([10.0, 5.0, 6.0])?, 1.0, 2.0)
    })
}
