//! A rounded puck shrunk past its rounds' radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    let (s, h) = (cylinder(&Frame::WORLD, 6.0, 10.0)?, 3.0 * 2f64.sqrt());
    s.fillet(&[s.edge_at([h, h, 0.0])?, s.edge_at([h, h, 10.0])?], 2.0)?.offset(-3.0)
}
