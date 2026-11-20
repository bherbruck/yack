// yack: brep-bench case swept/bull-nose-swung-partway-into-ring-sector
//! A bull-nose mill swung partway into a curved block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let mill = cylinder(&Frame::at([10.0, 0.0, -0.5]).moved(&Rigid::rotation(o, z, deg(-140.0))?), 1.0, 1.0)?;
    let rim = mill.edge_at(Rigid::rotation(o, z, deg(-140.0))?.point([10.0, 1.0, -0.5].into()))?;
    sector.cut_swept(&mill.fillet(&[rim], 0.3)?, &Motion::turn(o, z, rad(0.0), deg(170.0)))
}
