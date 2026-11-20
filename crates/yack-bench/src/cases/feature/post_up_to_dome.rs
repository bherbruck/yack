// yack: brep-bench case feature/post-up-to-dome
//! A post extruded up to the inside of a hollow dome.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let up = Plane::new([0.0; 3], [0.0, 0.0, 1.0])?;
    let shell = sphere(&Frame::WORLD, 20.0)?.difference(&sphere(&Frame::WORLD, 18.0)?)?.split(&up)?.front;
    let dome = shell.union(&cylinder(&Frame::at([0.0, 0.0, -2.0]), 20.0, 2.0)?)?;
    dome.union(&cylinder(&Frame::WORLD, 3.0, 20.0)?.intersection(&sphere(&Frame::WORLD, 18.0)?)?)
}
