// yack: `yack::step`, STEP exchange routed to the engine (M1.5, w-m11-springrep). Reading runs
// behind the panic firewall: a file is untrusted input.
//! STEP exchange (AP214, exact B-rep; never faceted).
use yack_springrep::facade::exchange as sr;

use crate::{Error, Result, Solid, Stage, engine, engine_cadrs, firewall, ladder};

/// One STEP writer: the engine's name and how it writes the solid.
type Writer<'a> = (&'static str, &'a dyn Fn() -> Result<String>);

/// The solid as an AP214 STEP file. Written by the engine that built it, and kept only when the
/// file reads back as the solid (volume and area); otherwise by the other engine, checked the same
/// way; a file neither writes faithfully is refused (Failed), never returned.
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::WORLD, [10.0, 20.0, 30.0])?;
/// let text = yack::step::write(&b)?;
/// assert!(text.starts_with("ISO-10303-21;"));
/// # Ok::<(), yack::Error>(())
/// ```
pub fn write(s: &Solid) -> Result<String> {
    let op = "step::write";
    if s.is_empty() {
        return Err(Error::invalid(op, "s", "the solid is empty"));
    }
    if s.held().is_some() {
        // Held in cadrs form: cadrs writes it, and the file is kept when cadrs's reader reads it
        // back as the solid. It is legitimate STEP, but our own step::read (springrep's reader)
        // may refuse it: a solid is held exactly because springrep cannot carry its faces (a
        // swept face folded in (u, v), say). Graded on the held bench cases (lemon,
        // rational-spheroid, paraboloid tips, the tumbling family): no wrong answer (lead ruling).
        let (v, a) = (s.volume(), s.area());
        return firewall::run(op, || {
            ladder::allowed(op, "cadrs")?;
            let text = engine_cadrs::write_step(op, s)?;
            yack_bridge::check_written_cadrs(&text, v, a).map_err(|e| Error::failed(op, Stage::Check, format!("cadrs: {e}")))?;
            Ok(text)
        });
    }
    let b = s.need(op)?;
    let springrep = || sr::write_step(b).map_err(|e| engine::error(op, e));
    let cadrs = || engine_cadrs::write_step(op, s);
    let writers: [Writer; 2] =
        if s.built_by() == "cadrs" { [("cadrs", &cadrs), ("springrep", &springrep)] } else { [("springrep", &springrep), ("cadrs", &cadrs)] };
    firewall::run(op, || written(op, s.volume(), s.area(), &writers))
}

/// The first of `writers`' files that reads back as a solid of `volume` and `area`, else a
/// refusal naming every writer's reason.
fn written(op: &'static str, volume: f64, area: f64, writers: &[Writer]) -> Result<String> {
    let mut why = Vec::new();
    for &(name, w) in writers {
        // a cadrs file is read back by both engines' readers (springrep's alone can share the
        // bridge's reading of it); springrep's by its own
        let read = |text: &str| -> core::result::Result<(), String> {
            sr::check_written(text, volume, area)?;
            if name == "cadrs" {
                yack_bridge::check_written_cadrs(text, volume, area)?;
            }
            Ok(())
        };
        let checked = ladder::allowed(op, name)
            .and_then(|()| w())
            .and_then(|text| read(&text).map(|()| text).map_err(|e| Error::failed(op, Stage::Check, e)));
        match checked {
            Ok(text) => return Ok(text),
            Err(e) => why.push(format!("{name}: {e}")),
        }
    }
    Err(Error::failed(op, Stage::Check, why.join("; ")))
}
/// The solid in a STEP file.
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::WORLD, [10.0, 20.0, 30.0])?;
/// let back = yack::step::read(&yack::step::write(&b)?)?;
/// assert!((back.volume() - 6000.0).abs() < 1e-6);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn read(text: &str) -> Result<Solid> {
    let op = "step::read";
    crate::ladder::springrep_only(op, &[], || engine::solid(op, sr::read_step(text))).map(Solid::with_read_history)
}
/// The solid sewn from the faces in a STEP file, closing gaps up to `tolerance` mm.
///
/// ```
/// use yack::prelude::*;
/// let b = block(&Frame::WORLD, [10.0, 20.0, 30.0])?;
/// let sewn = yack::step::sew(&yack::step::write(&b)?, 1e-3)?;
/// assert!((sewn.volume() - 6000.0).abs() < 1e-6);
/// # Ok::<(), yack::Error>(())
/// ```
pub fn sew(text: &str, tolerance: f64) -> Result<Solid> {
    let op = "step::sew";
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(Error::invalid(op, "tolerance", format!("{tolerance} mm is not a finite length ≥ 0")));
    }
    crate::ladder::springrep_only(op, &[], || engine::solid(op, sr::sew_step(text, tolerance))).map(Solid::with_read_history)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    fn cube() -> Solid {
        crate::make::block(&crate::Frame::WORLD, [1.0, 2.0, 3.0]).unwrap()
    }

    #[test]
    fn a_rejected_first_file_falls_to_the_second_writer() {
        let s = cube();
        let good = || write(&s);
        let wrong = || write(&crate::make::block(&crate::Frame::WORLD, [1.0, 1.0, 1.0]).unwrap());
        let text = written("step::write", s.volume(), s.area(), &[("first", &wrong), ("second", &good)]).unwrap();
        assert_eq!(text, write(&s).unwrap());
    }

    #[test]
    fn two_rejected_files_refuse_with_both_reasons() {
        let s = cube();
        let junk = || Ok(String::from("ISO-10303-21; junk"));
        let fails = || Err(Error::new(crate::ErrorKind::Unsupported, "step::write", "no writer"));
        let e = written("step::write", s.volume(), s.area(), &[("first", &junk), ("second", &fails)]).unwrap_err();
        assert_eq!(e.kind, crate::ErrorKind::Failed);
        assert!(e.detail.contains("first: ") && e.detail.contains("second: ") && e.detail.contains("no writer"), "{e}");
    }
}
