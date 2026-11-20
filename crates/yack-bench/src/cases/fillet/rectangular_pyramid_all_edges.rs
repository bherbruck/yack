//! A pyramid on a rectangle with every edge rounded, its apex split.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let sides = [Plane::new([0.0, 6.0, 0.0], [0.0, 2.0, 1.0])?, Plane::new([0.0, -6.0, 0.0], [0.0, -2.0, 1.0])?];
    let ends = [Plane::new([10.0, 0.0, 0.0], [6.0, 0.0, 5.0])?, Plane::new([-10.0, 0.0, 0.0], [-6.0, 0.0, 5.0])?];
    let base = block(&Frame::at([-10.0, -6.0, 0.0]), [20.0, 12.0, 12.0])?;
    let pyramid = sides.iter().chain(&ends).try_fold(base, |s, p| s.split(p).map(|cut| cut.back))?;
    pyramid.fillet(&pyramid.edges(), 1.5)
}
