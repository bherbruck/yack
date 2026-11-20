// yack: brep-bench case swept/drum-swung-partway-into-ring-sector
//! A drum swung partway into a curved block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let start = Frame::at([10.0, 0.0, -0.5]).moved(&Rigid::rotation(o, z, deg(-140.0))?);
    sector.cut_swept(&cylinder(&start, 1.0, 1.0)?, &Motion::turn(o, z, rad(0.0), deg(170.0)))
}
