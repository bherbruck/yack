// yack: brep-bench case chamfer/ball-in-conical-seat-distance-3-angle-15
//! A ball in a conical seat, the rim between them chamfered by a distance of 3 and an angle of 15°.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let plate = cylinder(&Frame::at([0.0, 0.0, -6.0]), 8.0, 11.0)?;
    let seat = plate.difference(&cone(&Frame::at([0.0, 0.0, 1.0]), 0.0, 8.0, 4.0)?)?;
    let s = seat.union(&sphere(&Frame::WORLD, 5.0)?)?;
    s.chamfer_angle(s.edge_at([0.0, 4.0, 3.0])?, s.face_at([0.0, 6.0, 4.0])?, 3.0, deg(15.0))
}
