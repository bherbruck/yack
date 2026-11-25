// yack: glue for yack's STEP and codec exchange over this crate and yack-codec (moved from springrep's facade/exchange.rs, M3 W10).
//! STEP read, write and sew, and the binary codec, with the protocol's argument shapes.

use crate::{Result, topo::Brep};

/// The solid as an AP214 STEP file.
pub fn write_step(b: &Brep) -> Result<String> {
    crate::write(b, "yack")
}

/// The solids in a STEP file's product structure.
pub fn read_step(text: &str) -> Result<Brep> {
    crate::read_part(text.as_bytes(), "step")
}

/// The solid sewn from a STEP file's faces, closing gaps up to `tolerance`.
pub fn sew_step(text: &str, tolerance: f64) -> Result<Brep> {
    crate::sew(text.as_bytes(), tolerance)
}

/// The solid as springrep's exact binary encoding.
pub fn encode(b: &Brep) -> Vec<u8> {
    yack_codec::to_bytes(std::slice::from_ref(b))
}

/// The one solid encoded in `bytes`, or `None` when they are not exactly one encoded solid.
pub fn decode(bytes: &[u8]) -> Option<Brep> {
    let mut v = yack_codec::from_bytes(bytes)?;
    if v.len() == 1 { v.pop() } else { None }
}

/// How far, relative, a written STEP file's re-read volume and area may be from the solid's.
/// springrep's reader heals what it reads, so it is a proxy good to about 1e-6: on bench
/// swept/ring-pushed-in-past-its-fold a correct file (exact to 1e-9 by the bench's own measurer)
/// reads back 3.9e-7 off in volume and 1.05e-6 off in area. The faults this guards against are
/// percent-sized (that case's misread face was 1.2%), so the bound sits a decade above the proxy's noise.
pub const WRITTEN_AGREES: f64 = 1e-5; // tol: springrep's healing reader as a measuring proxy

/// How much tighter than the agreement it checks a measure is integrated.
const MEASURE_MARGIN: f64 = 1e-3; // tol: a fraction of WRITTEN_AGREES, not a length

/// Whether `text`, a STEP file written for a solid of `volume` and `area`, reads back (by springrep's
/// reader) as that solid to `WRITTEN_AGREES`; the backstop of a STEP write, so that a writer's or a
/// bridge's loss becomes a refusal and never a wrong file. `Ok` with what it read, or why not.
pub fn check_written(text: &str, volume: f64, area: f64) -> std::result::Result<(f64, f64), String> {
    let back = read_step(text).map_err(|e| format!("the written file does not read back: {e}"))?;
    // The volume to a thousandth of the agreement asked, not to the last digits: a swept spline
    // body's exact integrals took most of a minute (bench swept/ring-carried-along-a-spline). The
    // area stays exact: `area_to` at that tolerance read conformance slabs-f7989a77c5e3f560's
    // area 2e-5 off (237.44601 for 237.45083) and refused a sound file.
    let rel = WRITTEN_AGREES * MEASURE_MARGIN;
    let (v, a) = (yack_measure::volume_to(&back, rel), yack_measure::area(&back));
    let off = |x: f64, want: f64| (x - want).abs() / want.abs().max(1.0);
    if off(v, volume) <= WRITTEN_AGREES && off(a, area) <= WRITTEN_AGREES {
        Ok((v, a))
    } else {
        Err(format!("the written file reads back as volume {v}, area {a}; the solid is {volume}, {area}"))
    }
}

/// Whether a STEP file's solid is a mesh rather than exact geometry: the file writes a faceted
/// B-rep or a polygon loop, or every face of the solid read is a flat triangle.
fn is_mesh(text: &str, b: &Brep) -> bool {
    if text.contains("FACETED_BREP") || text.contains("POLY_LOOP") {
        return true;
    }
    let mut faces = b.face_ids().peekable();
    faces.peek().is_some()
        && faces.all(|f| {
            let face = b.face(f);
            matches!(face.surface, crate::geom::Surface::Plane(_))
                && face.loops.len() == 1
                && face.loops.first().is_some_and(|&lp| b.lp(lp).coedges.len() == 3)
        })
}

/// [`read_step`], and whether the file's solid was a mesh (see `is_mesh`).
pub fn read_step_report(text: &str) -> Result<(Brep, bool)> {
    let b = read_step(text)?;
    let mesh = is_mesh(text, &b);
    Ok((b, mesh))
}

/// [`sew_step`], and whether the file's solid was a mesh (see `is_mesh`).
pub fn sew_step_report(text: &str, tolerance: f64) -> Result<(Brep, bool)> {
    let b = sew_step(text, tolerance)?;
    let mesh = is_mesh(text, &b);
    Ok((b, mesh))
}
