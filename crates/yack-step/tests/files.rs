// yack: the STEP measurer on written files, against the volumes and areas the brep-bench grader reads.
#![cfg(feature = "wip")]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use yack_step::measure_file;

fn file(name: &str) -> String {
    std::fs::read_to_string(format!("{}/tests/data/{name}", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

fn off(x: f64, want: f64) -> f64 {
    (x - want).abs() / want.abs().max(1.0)
}

/// The bench grades volume and area to 1e-6 relative; the measurer must agree to that on sound files.
const AGREES: f64 = 1e-6;

#[test]
fn sound_files_measure_as_the_grader_reads_them() {
    // (file, grader's volume, grader's area), from brep-bench's `mass` tool on the same files.
    let cases = [
        ("ring-pushed-good.step", 1907.0021200819538, 1164.5686671333683),
        ("nurbs-lens.step", 108.90854524627824, 125.66370606435706),
        ("spline-prism.step", 3149.171928239837, 1307.5077317745977),
    ];
    let mut report = Vec::new();
    for (name, v, a) in cases {
        match measure_file(&file(name)) {
            Ok(m) => report.push((name, m.volume, m.area, off(m.volume, v), off(m.area, a))),
            Err(e) => panic!("{name}: {e}"),
        }
    }
    for r in &report {
        eprintln!("{} volume {} area {} (off {:e}, {:e})", r.0, r.1, r.2, r.3, r.4);
    }
    assert!(report.iter().all(|r| r.3 <= AGREES && r.4 <= AGREES), "{report:?}");
}

#[test]
fn the_ring_pushed_past_its_fold_file_is_refused() {
    // Written from a body of volume 1907.0021203 (in memory); the grader reads it as 1930.17. Its
    // edges' 3-D curves bound the right solid, but pcurves written in another frame of the torus
    // disagree with them, and a reader that follows the pcurves gets another solid.
    match measure_file(&file("ring-pushed-bad.step")) {
        Err(e) => eprintln!("bad ring refused: {e}"),
        Ok(m) => panic!("bad ring measured as volume {} area {}", m.volume, m.area),
    }
}
