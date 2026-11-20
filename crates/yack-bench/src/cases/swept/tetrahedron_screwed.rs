// yack: brep-bench case swept/tetrahedron-screwed
//! A regular tetrahedron carried along a screw.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let corners = [[-1.0, -1.0, -1.0], [-1.0, 1.0, 1.0], [1.0, -1.0, 1.0], [1.0, 1.0, -1.0]];
    let cube = block(&Frame::at([3.0, -1.0, -1.0]), [2.0, 2.0, 2.0])?;
    let tet = corners.iter().try_fold(cube, |s, &[a, b, c]| {
        s.split(&Plane::new([4.0 + a / 3.0, b / 3.0, c / 3.0], [a, b, c])?).map(|p| p.back)
    })?;
    let (o, z) = (Point3::from([0.0; 3]), Vec3::from([0.0, 0.0, 1.0]));
    tet.swept(&Motion::Screw { axis_point: o, axis: z, angle: deg(540.0), rise_per_turn: 5.0 })
}
