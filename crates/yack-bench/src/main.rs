// yack: the brep-bench runner for yack (PLAN §5.2; brep-bench SPEC.md "A runner").
// Adapted from brep-bench@d522d62 kernels/cadrs/src/main.rs (MIT OR Apache-2.0, (c) the brep-bench
// contributors).
// Changes: yack's public API; one mesh rung ladder; no profiling switches; the warm-start pause by
// `kill -STOP` (no unsafe); binary STL only where single precision keeps the points apart.
//! ```text
//! yack-bench CASE OUT.stl [OUT.step]
//! yack-bench --version
//! ```
//!
//! Builds the case `CASE` (`boolean/napkin-ring`) with its program,
//! `src/cases/boolean/napkin_ring.rs`, through yack's public API, and prints one line of JSON.
//! Given `OUT.step`, the solid written there by `yack::step::write` is the whole answer:
//! `{"status":"ok"}`. Otherwise (or where the writer declines, said in `step_error`) it prints
//! yack's own measures of the exact solid and writes its triangles (`Solid::mesh`) to `OUT.stl`.
//! A case that refuses prints `{"status":"refused","reason":"<the error>"}`. An unknown case
//! exits non-zero having built nothing. The runner never reads the case's description.
// A tool crate: it reads brep-bench's BREP_BENCH_WARM switch (SPEC.md "A runner"), never a kernel.
#![allow(clippy::disallowed_methods)]
use std::collections::BTreeMap;
use std::process::ExitCode;
use yack::{Solid, SurfaceKind};

/// A case's program: the solid it builds, or yack's refusal.
type Build = fn() -> yack::Result<Solid>;

/// One module per case file, and `CASES`, written by `build.rs`.
mod cases {
    use super::Build;
    include!(concat!(env!("OUT_DIR"), "/cases.rs"));
}

/// A face's surface as the reply counts it.
fn kind(s: &SurfaceKind) -> &'static str {
    match s {
        SurfaceKind::Plane { .. } => "plane",
        SurfaceKind::Cylinder { .. } => "cylinder",
        SurfaceKind::Cone => "cone",
        SurfaceKind::Sphere { .. } => "sphere",
        SurfaceKind::Torus => "torus",
        SurfaceKind::Spline => "bspline",
        _ => "other",
    }
}

/// A JSON string.
fn quoted(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// A JSON number (JSON has no NaN or infinity).
fn number(x: f64) -> String {
    if x.is_finite() { format!("{x:e}") } else { "null".to_owned() }
}

fn refused(why: &str) -> String {
    format!("{{\"status\":\"refused\",\"reason\":{}}}", quoted(why))
}

/// A mesh as plain points and triangles.
struct Tris {
    points: Vec<[f64; 3]>,
    tris: Vec<[usize; 3]>,
}

impl Tris {
    fn corners(&self, t: &[usize; 3]) -> Option<[[f64; 3]; 3]> {
        Some([*self.points.get(t[0])?, *self.points.get(t[1])?, *self.points.get(t[2])?])
    }

    /// The volume the triangles bound (divergence theorem), each point first passed through `at`.
    fn volume(&self, at: impl Fn([f64; 3]) -> [f64; 3]) -> f64 {
        self.tris
            .iter()
            .filter_map(|t| self.corners(t))
            .map(|c| {
                let [a, b, c] = c.map(&at);
                (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0]))
                    / 6.0
            })
            .sum()
    }
}

fn single(p: [f64; 3]) -> [f64; 3] {
    p.map(|x| f64::from(x as f32))
}

fn normal([a, b, c]: [[f64; 3]; 3]) -> [f64; 3] {
    let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
    let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 0.0 { n.map(|x| x / len) } else { n }
}

/// The triangles as STL: binary (single precision) when that keeps every distinct point apart and
/// the bounded volume as it was, else ASCII with every coordinate in full (shortest round trip).
fn write_stl(path: &str, mesh: &Tris) -> std::io::Result<()> {
    let bits = |p: [f64; 3]| p.map(f64::to_bits);
    let mut exact = std::collections::BTreeSet::new();
    let mut rounded = std::collections::BTreeSet::new();
    let mut apart = true;
    for &p in &mesh.points {
        if exact.insert(bits(p)) {
            apart &= rounded.insert(bits(single(p)));
        }
    }
    let full = mesh.volume(|p| p);
    let facets = mesh.tris.iter().filter_map(|t| mesh.corners(t));
    if apart && (mesh.volume(single) - full).abs() <= 1e-4 * full.abs() {
        let mut out = Vec::with_capacity(84 + 50 * mesh.tris.len());
        out.extend_from_slice(&[0u8; 80]);
        out.extend_from_slice(&u32::try_from(mesh.tris.len()).unwrap_or(u32::MAX).to_le_bytes());
        for c in facets {
            for x in normal(c).iter().chain(c.iter().flatten()) {
                out.extend_from_slice(&(*x as f32).to_le_bytes());
            }
            out.extend_from_slice(&[0u8; 2]);
        }
        return std::fs::write(path, out);
    }
    use std::fmt::Write as _;
    let mut out = String::with_capacity(32 + 160 * mesh.tris.len());
    out.push_str("solid yack\n");
    for c in facets {
        let [x, y, z] = normal(c);
        let _ = writeln!(out, "facet normal {x:e} {y:e} {z:e}\nouter loop");
        for [x, y, z] in c {
            let _ = writeln!(out, "vertex {x:e} {y:e} {z:e}");
        }
        out.push_str("endloop\nendfacet\n");
    }
    out.push_str("endsolid yack\n");
    std::fs::write(path, out)
}

/// The solid's triangles: the coarsest of a few chords, from a thousandth of its size (or a
/// twentieth of its thickness, volume over area, where less), that bounds its exact volume to
/// 1/100 (the harness asks 1/50).
fn meshed(solid: &Solid, volume: f64, area: f64) -> yack::Result<Tris> {
    let size = solid.bounding_box().map_or(1.0, |b| {
        let (lo, hi) = (b.min, b.max);
        ((hi.x - lo.x).powi(2) + (hi.y - lo.y).powi(2) + (hi.z - lo.z).powi(2)).sqrt()
    });
    let thickness = if area > 0.0 && volume > 0.0 { volume / area } else { size };
    let mut chord = (1e-3 * size).min(5e-2 * thickness).max(1e-9);
    let mut rung = 0;
    loop {
        let mesh = solid.mesh(chord)?;
        let tris = Tris {
            points: mesh.positions.iter().map(|p| [p.x, p.y, p.z]).collect(),
            tris: mesh.triangles.iter().map(|t| t.map(|i| i as usize)).collect(),
        };
        let off = (tris.volume(|p| p) - volume).abs() / volume.abs().max(f64::MIN_POSITIVE);
        rung += 1;
        if off <= 1e-2 || rung >= 5 {
            return Ok(tris);
        }
        chord /= 4.0;
    }
}

/// The reply for a solid built. With `out_step`, the solid is written there and that is the whole
/// answer, unless the writer declines (`step_error`), when the runner goes on to measure.
fn reply(solid: &Solid, out: &str, out_step: Option<&str>) -> Result<String, String> {
    let step_error = match out_step.map(|path| (path, yack::step::write(solid))) {
        None => String::new(),
        Some((path, Ok(text))) => {
            std::fs::write(path, text).map_err(|e| format!("{path} not written: {e}"))?;
            return Ok("{\"status\":\"ok\"}".to_owned());
        }
        Some((_, Err(e))) => format!(",\"step_error\":{}", quoted(&e.to_string())),
    };
    let (volume, area) = (solid.volume(), solid.area());
    let valid = solid.check().is_empty();
    let mut faces = BTreeMap::new();
    for id in solid.faces() {
        let face = solid.face(id).map_err(|e| e.to_string())?;
        *faces.entry(kind(&face.kind())).or_insert(0u64) += 1;
    }
    let faces = faces.iter().map(|(k, n)| format!("\"{k}\":{n}")).collect::<Vec<_>>().join(",");
    let centroid = solid
        .centroid()
        .map_or(String::new(), |c| format!(",\"centroid\":[{},{},{}]", number(c.x), number(c.y), number(c.z)));
    let rows = solid.inertia().map(|r| format!("[{}]", r.map(number).join(",")));
    let mesh = meshed(solid, volume, area).map_err(|e| format!("not meshed: {e}"))?;
    write_stl(out, &mesh).map_err(|e| format!("{out} not written: {e}"))?;
    Ok(format!(
        "{{\"status\":\"ok\",\"volume\":{},\"area\":{}{centroid},\"inertia\":[{}],\"faces\":{{{faces}}},\
         \"valid\":{valid}{step_error}}}",
        number(volume),
        number(area),
        rows.join(","),
    ))
}

/// Under the harness (`BREP_BENCH_WARM` set; SPEC.md "A runner"), stops the runner until the
/// harness continues it: once it has found its case, and once it has answered.
fn pause() {
    if std::env::var_os("BREP_BENCH_WARM").is_some() {
        use std::io::Write as _;
        let _ = std::io::stdout().flush();
        let pid = std::process::id().to_string();
        let _ = std::process::Command::new("kill").args(["-STOP", &pid]).status();
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("--version") {
        println!("yack {} {}", yack::VERSION, yack::SOURCE);
        return ExitCode::SUCCESS;
    }
    let (case, out, out_step) = match &args[..] {
        [_, case, out] => (case, out, None),
        [_, case, out, step] => (case, out, Some(step.as_str())),
        _ => {
            eprintln!("usage: yack-bench CASE OUT.stl [OUT.step]");
            return ExitCode::from(2);
        }
    };
    let Some(&(_, build)) = cases::CASES.iter().find(|(id, _)| id == case) else {
        eprintln!("no implementation of `{case}`");
        return ExitCode::from(2);
    };
    pause();
    let answer = match build() {
        Ok(solid) => reply(&solid, out, out_step).unwrap_or_else(|why| refused(&why)),
        Err(e) => refused(&e.to_string()),
    };
    println!("{answer}");
    pause();
    ExitCode::SUCCESS
}
