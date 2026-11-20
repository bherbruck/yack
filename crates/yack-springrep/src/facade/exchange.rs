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

/// How far, relative, a written STEP file's re-read volume and area may be from the solid's.
/// springrep's reader heals what it reads, so it is a proxy good to about 1e-6: on bench
/// swept/ring-pushed-in-past-its-fold a correct file (exact to 1e-9 by the bench's own measurer)
/// reads back 3.9e-7 off in volume and 1.05e-6 off in area. The faults this guards against are
/// percent-sized (that case's misread face was 1.2%), so the bound sits a decade above the proxy's noise.
pub const WRITTEN_AGREES: f64 = 1e-5; // tol: springrep's healing reader as a measuring proxy

/// Whether `text`, a STEP file written for a solid of `volume` and `area`, reads back (by springrep's
/// reader) as that solid to `WRITTEN_AGREES`; the backstop of a STEP write, so that a writer's or a
/// bridge's loss becomes a refusal and never a wrong file. `Ok` with what it read, or why not.
pub fn check_written(text: &str, volume: f64, area: f64) -> std::result::Result<(f64, f64), String> {
    let back = read_step(text).map_err(|e| format!("the written file does not read back: {e}"))?;
    let (v, a) = (crate::measure::volume(&back), crate::measure::area(&back));
    let off = |x: f64, want: f64| (x - want).abs() / want.abs().max(1.0);
    if off(v, volume) <= WRITTEN_AGREES && off(a, area) <= WRITTEN_AGREES {
        Ok((v, a))
    } else {
        Err(format!("the written file reads back as volume {v}, area {a}; the solid is {volume}, {area}"))
    }
}
