//! A ball swung about an axis that passes through it.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    Err(Error::unsupported("swept/ball-swung-across-its-own-center-line"))
}
