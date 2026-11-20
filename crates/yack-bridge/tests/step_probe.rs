// yack: probe for brep-bench swept/ring-pushed-in-past-its-fold (WRONG from its STEP): the cadrs result
// written by cadrs's own STEP writer, and carried to springrep and written by springrep's (files in
// the scratchpad, measured by the harness's mass tool outside the test).
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_cadrs::native::P3;
use yack_cadrs::{BoolOp, DefaultKernel, Kernel, Primitive, yack_glue};

const OUT: &str = "/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad";

#[test]
fn ring_hole_step_both_writers() {
    let (s, c) = (65f64.to_radians().sin(), 65f64.to_radians().cos());
    let mut k = DefaultKernel::new();
    let ring = yack_glue::torus(&mut k, [0.0, 0.0, 4.0], [s, 0.0, c], 3.0, 1.0).unwrap().0;
    let hole = yack_glue::swept(&mut k, ring, &yack_glue::Move::Slide { by: [0.0, 0.0, -8.0] }).unwrap().0;
    let block = k.primitive(&Primitive::Box { min: P3::new(-8.0, -8.0, -8.0), max: P3::new(8.0, 8.0, 0.0) }).unwrap().first().unwrap();
    let cut = k.boolean(BoolOp::Subtract, block, &[hole]).unwrap().first().unwrap();
    let body = (**k.get(cut).unwrap()).clone();
    let m = k.mass_properties(cut).unwrap();
    std::fs::write(format!("{OUT}/ring-cadrs.step"), k.export_step(&[cut]).unwrap()).unwrap();
    let mut lines = vec![format!("cadrs in memory: volume {} area {}", m.volume, m.surface_area)];
    match yack_bridge::to_springrep(&body, &[]) {
        Ok(b) => {
            lines.push(format!(
                "springrep in memory: volume {} area {}",
                yack_springrep::measure::volume(&b),
                yack_springrep::measure::area(&b)
            ));
            std::fs::write(format!("{OUT}/ring-springrep.step"), yack_springrep::step::write(&b, "ring").unwrap()).unwrap();
            // And springrep reading its own file back.
            let text = std::fs::read(format!("{OUT}/ring-springrep.step")).unwrap();
            match yack_springrep::step::read_part(&text, "r") {
                Ok(r) => lines.push(format!("springrep re-read: volume {}", yack_springrep::measure::volume(&r))),
                Err(e) => lines.push(format!("springrep re-read: {e}")),
            }
        }
        Err(e) => lines.push(format!("to_springrep: {e}")),
    }
    std::fs::write(format!("{OUT}/ring-probe.txt"), lines.join("\n")).unwrap();
}

/// The ladder's path: springrep's torus and block carried to cadrs, cadrs's sweep and cut, the result
/// carried back and written by springrep.
#[test]
fn ring_hole_step_ladder_path() {
    use yack_springrep::math::{Frame, Vec3};
    let (s, c) = (65f64.to_radians().sin(), 65f64.to_radians().cos());
    let f = Frame::new(Vec3::new(0.0, 0.0, 4.0), Vec3::new(s, 0.0, c), Vec3::new(c, 0.0, -s)).unwrap();
    let torus = yack_springrep::primitive::torus(&f, 3.0, 1.0, "t").unwrap();
    let block = yack_springrep::primitive::block(
        &Frame::new(Vec3::new(-8.0, -8.0, -8.0), Vec3::Z, Vec3::X).unwrap(),
        Vec3::new(16.0, 16.0, 8.0),
        "b",
    )
    .unwrap();
    let mut k = DefaultKernel::new();
    let ring = k.insert(yack_bridge::to_cadrs(&torus).unwrap().0);
    let hole = yack_glue::swept(&mut k, ring, &yack_glue::Move::Slide { by: [0.0, 0.0, -8.0] }).unwrap().0;
    let blk = k.insert(yack_bridge::to_cadrs(&block).unwrap().0);
    let cut = k.boolean(BoolOp::Subtract, blk, &[hole]).unwrap().first().unwrap();
    let body = (**k.get(cut).unwrap()).clone();
    let m = k.mass_properties(cut).unwrap();
    std::fs::write(format!("{OUT}/ring2-cadrs.step"), k.export_step(&[cut]).unwrap()).unwrap();
    let mut lines = vec![format!("cadrs in memory: volume {} area {} edges {}", m.volume, m.surface_area, body.edges().len())];
    match yack_bridge::to_springrep(&body, &[]) {
        Ok(b) => {
            lines.push(format!("springrep in memory: volume {} edges {}", yack_springrep::measure::volume(&b), b.edge_ids().count()));
            std::fs::write(format!("{OUT}/ring2-springrep.step"), yack_springrep::step::write(&b, "ring").unwrap()).unwrap();
        }
        Err(e) => lines.push(format!("to_springrep: {e}")),
    }
    std::fs::write(format!("{OUT}/ring2-probe.txt"), lines.join("\n")).unwrap();
}

/// As `ring_hole_step_ladder_path`, with springrep's torus given cadrs's own frame x for its axis
/// (`Frame::from_z`): does the seam's place decide the STEP?
#[test]
fn ring_hole_step_ladder_path_native_seam() {
    use yack_springrep::math::{Frame, Vec3};
    let (s, c) = (65f64.to_radians().sin(), 65f64.to_radians().cos());
    let cf = yack_cadrs::native::Frame::from_z(P3::new(0.0, 0.0, 4.0), yack_cadrs::native::V3::new(s, 0.0, c));
    let f = Frame::new(Vec3::new(0.0, 0.0, 4.0), Vec3::new(s, 0.0, c), Vec3::new(cf.x.x, cf.x.y, cf.x.z)).unwrap();
    let torus = yack_springrep::primitive::torus(&f, 3.0, 1.0, "t").unwrap();
    let block = yack_springrep::primitive::block(
        &Frame::new(Vec3::new(-8.0, -8.0, -8.0), Vec3::Z, Vec3::X).unwrap(),
        Vec3::new(16.0, 16.0, 8.0),
        "b",
    )
    .unwrap();
    let mut k = DefaultKernel::new();
    let ring = k.insert(yack_bridge::to_cadrs(&torus).unwrap().0);
    let hole = yack_glue::swept(&mut k, ring, &yack_glue::Move::Slide { by: [0.0, 0.0, -8.0] }).unwrap().0;
    let blk = k.insert(yack_bridge::to_cadrs(&block).unwrap().0);
    let cut = k.boolean(BoolOp::Subtract, blk, &[hole]).unwrap().first().unwrap();
    std::fs::write(format!("{OUT}/ring3-cadrs.step"), k.export_step(&[cut]).unwrap()).unwrap();
    let b = (**k.get(cut).unwrap()).clone();
    std::fs::write(format!("{OUT}/ring3-probe.txt"), format!("edges {} x {:?}", b.edges().len(), cf.x)).unwrap();
}

/// Timing (scratchpad, never failing): the STEP backstop's re-read of bench fine-thread's file
/// (written by the ladder12 binary), by springrep's reader and by cadrs's.
#[test]
#[allow(clippy::disallowed_methods)] // a timing probe, test only
fn time_the_backstop_re_reads() {
    use std::time::Instant;
    let Ok(text) = std::fs::read_to_string(format!("{OUT}/stepperf/ft.step")) else { return };
    let mut lines = Vec::new();
    let t = Instant::now();
    let s = yack_springrep::facade::exchange::check_written(&text, 1.0, 1.0);
    lines.push(format!("springrep re-read: {:.1} s, {:?}", t.elapsed().as_secs_f64(), s.map_err(|e| e.chars().take(120).collect::<String>())));
    let t = Instant::now();
    let c = yack_bridge::check_written_cadrs(&text, 1.0, 1.0);
    lines.push(format!("cadrs re-read: {:.1} s, {:?}", t.elapsed().as_secs_f64(), c.map_err(|e| e.chars().take(120).collect::<String>())));
    std::fs::write(format!("{OUT}/stepperf/times.txt"), lines.join("\n")).unwrap();
}

/// The cheap backstop must still catch the ring-pushed fault: cadrs's re-read of the bad file the
/// ladder once wrote (the harness reads 1930.17 from it; the solid is 1907.0021203, 1164.5686686)
/// must refuse it, and its re-read of the good one must accept it. Timed.
#[test]
#[allow(clippy::disallowed_methods)] // a timing probe, test only
fn the_cadrs_re_read_catches_the_ring_pushed_fault() {
    use std::time::Instant;
    let (v, a) = (1907.0021203274207, 1164.568668645316);
    let mut lines = Vec::new();
    for name in ["ring.step", "ring2-springrep.step", "ring-springrep.step"] {
        let Ok(text) = std::fs::read_to_string(format!("{OUT}/{name}")) else { continue };
        let t = Instant::now();
        let r = yack_bridge::check_written_cadrs(&text, v, a);
        lines.push(format!("{name}: cadrs {:.2} s, {:?}", t.elapsed().as_secs_f64(), r.map_err(|e| e.chars().take(140).collect::<String>())));
        let t = Instant::now();
        let r = yack_springrep::facade::exchange::check_written(&text, v, a);
        lines.push(format!("{name}: springrep {:.2} s, {:?}", t.elapsed().as_secs_f64(), r.map_err(|e| e.chars().take(140).collect::<String>())));
    }
    std::fs::write(format!("{OUT}/stepperf/ring-catch.txt"), lines.join("\n")).unwrap();
}
