// yack: the STEP-write backstop for a solid held in cadrs form: the written file read back by cadrs's
// own reader must hold the solid's volume and area, or the write is refused.
use yack_cadrs::{DefaultKernel, Kernel};

/// How far, relative, a cadrs re-read's volume and area may be from the solid's. cadrs's reader,
/// like springrep's, sews and fits what it reads, so it is a measuring proxy, not an exact one: the
/// same bound as springrep's backstop (`yack_springrep::facade::exchange::WRITTEN_AGREES`), a decade
/// above a healing reader's noise; the faults it guards against are percent-sized.
pub const CADRS_WRITTEN_AGREES: f64 = 1e-5; // tol: a healing reader as a measuring proxy

/// Whether `text`, a STEP file written for a solid of `volume` and `area`, reads back through cadrs's
/// reader as one solid of that volume and area to [`CADRS_WRITTEN_AGREES`]. `Ok` with what it read.
pub fn check_written_cadrs(text: &str, volume: f64, area: f64) -> std::result::Result<(f64, f64), String> {
    let mut k = DefaultKernel::new();
    let bodies = k.import_step(text.as_bytes()).map_err(|e| format!("the written file does not read back in cadrs: {e}"))?;
    let (mut v, mut a) = (0.0, 0.0);
    for b in &bodies {
        let m = k.mass_properties(*b).map_err(|e| format!("cadrs cannot measure the file's solid: {e}"))?;
        v += m.volume;
        a += m.surface_area;
    }
    if bodies.is_empty() {
        return Err("the written file holds no solid cadrs reads".into());
    }
    let off = |x: f64, want: f64| (x - want).abs() / want.abs().max(1.0);
    if off(v, volume) <= CADRS_WRITTEN_AGREES && off(a, area) <= CADRS_WRITTEN_AGREES {
        Ok((v, a))
    } else {
        Err(format!("the written file reads back in cadrs as volume {v}, area {a}; the solid is {volume}, {area}"))
    }
}
