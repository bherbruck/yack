// yack: `yack::step`, STEP exchange routed to the engine (M1.5, w-m11-springrep). Reading runs
// behind the panic firewall: a file is untrusted input.
//! STEP exchange (AP214, exact B-rep; never faceted).
use yack_springrep::facade::exchange as sr;

use crate::{Error, Result, Solid, engine, firewall};

/// The solid as an AP214 STEP file.
pub fn write(s: &Solid) -> Result<String> {
    let op = "step::write";
    let b = s.brep().ok_or_else(|| Error::invalid(op, "s", "the solid is empty"))?;
    firewall::run(op, || sr::write_step(b).map_err(|e| engine::error(op, e)))
}
/// The solid in a STEP file.
pub fn read(text: &str) -> Result<Solid> {
    let op = "step::read";
    crate::ladder::springrep_only(op, || engine::solid(op, sr::read_step(text)))
}
/// The solid sewn from the faces in a STEP file, closing gaps up to `tolerance` mm.
pub fn sew(text: &str, tolerance: f64) -> Result<Solid> {
    let op = "step::sew";
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(Error::invalid(op, "tolerance", format!("{tolerance} mm is not a finite length ≥ 0")));
    }
    crate::ladder::springrep_only(op, || engine::solid(op, sr::sew_step(text, tolerance)))
}
