// yack: brep-bench case scale/cylinders-crossed-a-million-from-the-origin
//! Two cylinders crossed a million from the origin.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (c, u, v) = ([600000.0, -800000.0, 0.0], [2.0, 3.0, 6.0].map(|x| x / 7.0), [3.0, -6.0, 2.0].map(|x| x / 7.0));
    let first = cylinder(&Frame::along([c[0] - 10.0 * u[0], c[1] - 10.0 * u[1], c[2] - 10.0 * u[2]], u)?, 3.0, 20.0)?;
    let second = cylinder(&Frame::along([c[0] - 10.0 * v[0], c[1] - 10.0 * v[1], c[2] - 10.0 * v[2]], v)?, 3.0, 20.0)?;
    first.intersection(&second)
}
