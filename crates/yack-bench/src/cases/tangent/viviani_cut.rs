// yack: brep-bench case tangent/viviani-cut
//! A ball bored by a cylinder tangent inside it, pinched where they touch.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    // ball's poles on the y axis, seam through (−10, 0, 0); the cylinder's seam on its side at x = 0
    let ball = sphere(&Frame::along([0.0, 0.0, 0.0], [0.0, 1.0, 0.0])?.turned(deg(180.0)), 10.0)?;
    ball.difference(&cylinder(&Frame::at([5.0, 0.0, -12.0]).turned(deg(180.0)), 5.0, 24.0)?)
}
