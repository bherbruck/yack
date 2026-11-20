//! A dumbbell plate shelled thicker than its neck: the inside walls cross and the neck fills.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let outline = [[0.0, 0.0], [10.0, 0.0], [10.0, 4.0], [14.0, 4.0], [14.0, 0.0], [24.0, 0.0], [24.0, 10.0]];
    let outline = [&outline[..], &[[14.0, 10.0], [14.0, 6.0], [10.0, 6.0], [10.0, 10.0], [0.0, 10.0]]].concat();
    let plate = extrude(&Frame::WORLD, &Profile::polygon(&outline)?, 6.0)?;
    plate.shell(&[plate.face_at([5.0, 5.0, 6.0])?], 1.5)
}
