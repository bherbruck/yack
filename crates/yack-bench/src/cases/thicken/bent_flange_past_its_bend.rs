//! A bent sheet thickened inside its bend past the bend's radius.
use yack::prelude::*;
pub fn build() -> Result<Solid> {
    Err(Error::unsupported("thicken/bent-flange-past-its-bend"))
}
