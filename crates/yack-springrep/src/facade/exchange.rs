// yack: glue for yack's STEP and codec exchange over springrep's `step` and `codec`.
//! STEP read, write and sew, and the binary codec, with the protocol's argument shapes.

use crate::{Result, codec, step, topo::Brep};

/// The solid as an AP214 STEP file.
pub fn write_step(b: &Brep) -> Result<String> {
    step::write(b, "yack")
}

/// The solids in a STEP file's product structure.
pub fn read_step(text: &str) -> Result<Brep> {
    step::read_part(text.as_bytes(), "step")
}

/// The solid sewn from a STEP file's faces, closing gaps up to `tolerance`.
pub fn sew_step(text: &str, tolerance: f64) -> Result<Brep> {
    step::sew(text.as_bytes(), tolerance)
}

/// The solid as springrep's exact binary encoding.
pub fn encode(b: &Brep) -> Vec<u8> {
    codec::to_bytes(std::slice::from_ref(b))
}

/// The one solid encoded in `bytes`, or `None` when they are not exactly one encoded solid.
pub fn decode(bytes: &[u8]) -> Option<Brep> {
    let mut v = codec::from_bytes(bytes)?;
    if v.len() == 1 { v.pop() } else { None }
}
