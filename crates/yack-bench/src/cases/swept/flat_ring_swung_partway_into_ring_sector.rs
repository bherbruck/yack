// yack: brep-bench case swept/flat-ring-swung-partway-into-ring-sector
//! A ring lying flat, swung partway into a curved block.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    let sector = revolve(&Frame::WORLD, &Profile::rectangle([8.0, -1.0], [12.0, 1.0])?, deg(60.0))?;
    let start = Frame::at([10.0, 0.0, 0.0]).moved(&Rigid::rotation(o, z, deg(-140.0))?);
    sector.difference(&torus(&start, 1.0, 0.4)?.swept(&Motion::Turn { axis_point: o, axis: z, angle: deg(170.0) })?)
}
