// yack: brep-bench case feature/rib-revolved-to-a-stop
//! A rib turned about a hub until it meets a stop.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (a, b) = (120f64.to_radians(), 135f64.to_radians());
    let wedge = Loop::starting_at([0.0, 0.0]).line_to([25.0 * a.cos(), 25.0 * a.sin()]);
    let wedge = wedge.arc_ccw_to([25.0 * b.cos(), 25.0 * b.sin()], [0.0, 0.0]).close()?;
    let stop = extrude(&Frame::WORLD, &Profile::new(wedge, vec![])?, 10.0)?;
    let rib = revolve(&Frame::WORLD, &Profile::rectangle([5.0, 0.0], [20.0, 4.0])?, deg(120.0))?;
    cylinder(&Frame::WORLD, 10.0, 10.0)?.union(&stop)?.union(&rib)
}
