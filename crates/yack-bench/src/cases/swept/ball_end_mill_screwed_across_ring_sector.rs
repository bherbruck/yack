// yack: brep-bench case swept/ball-end-mill-screwed-across-ring-sector
//! A helical groove milled by a ball-end mill screwed across a block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -2.0], [12.0, 2.0])?, deg(60.0))?;
    let sector = sector.transform(&Rigid::rotation(o, z, deg(-30.0))?.into())?;
    let mill = sphere(&Frame::at([10.0, 0.0, 0.0]), 1.0)?.union(&cylinder(&Frame::at([10.0, 0.0, 0.0]), 1.0, 3.0)?)?;
    let mill = mill.transform(&Rigid::rotation(o, z, deg(-60.0))?.into())?;
    let mill = mill.transform(&Rigid::translation([0.0, 0.0, -0.5]).into())?;
    sector.difference(&mill.swept(&Motion::Screw { axis_point: o, axis: z, angle: deg(120.0), rise_per_turn: 3.0 })?)
}
