// yack: brep-bench case chamfer/narrow-sector-rim-distance-2-angle-45
//! A narrow spherical sector's rim chamfered by a distance of 2 and an angle of 45°.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (r, z) = (3.0, 4.0);
    let side = Loop::starting_at([0.0, 0.0]).line_to([r, z]).arc_ccw_to([0.0, 5.0], [0.0, 0.0]).close()?;
    let s = revolve(&Frame::WORLD, &Profile::new(side, vec![])?, deg(360.0))?;
    s.chamfer_angle(s.edge_at([0.0, r, z])?, s.face_at([0.0, r / 2.0, z / 2.0])?, 2.0, deg(45.0))
}
