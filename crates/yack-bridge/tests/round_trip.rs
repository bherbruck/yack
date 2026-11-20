// yack: round trips springrep → cadrs → springrep on one solid of each surface and curve kind (MAPPING.md §5).
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_bridge::round_trip;
use yack_springrep::{
    boolean,
    math::{Frame, Vec3},
    primitive,
    topo::Brep,
};

fn check(name: &str, s: &Brep) {
    let r = round_trip(s);
    assert!(r.ok(), "\nFAILED {name}: {}", r.problems.join("\nFAILED   "));
}

fn at(x: f64, y: f64, z: f64) -> Frame {
    Frame::new(Vec3::new(x, y, z), Vec3::Z, Vec3::X).unwrap()
}

#[test]
fn primitives() {
    check("block", &primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 20.0, 30.0), "b").unwrap());
    check("cylinder", &primitive::cylinder(&at(1.0, 2.0, 3.0), 4.0, 9.0, "c").unwrap());
    check("sphere", &primitive::sphere(&at(0.0, 0.0, 0.0), 5.0, "s").unwrap());
    check("torus", &primitive::torus(&at(0.0, 0.0, 0.0), 6.0, 2.0, "t").unwrap());
    check("cone", &primitive::cone(&at(0.0, 0.0, 0.0), 4.0, 1.0, 6.0, "k").unwrap());
    check("apex", &primitive::cone(&at(0.0, 0.0, 0.0), 4.0, 0.0, 6.0, "k").unwrap());
}

#[test]
fn drilled_and_ball() {
    let block = primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
    let rod = primitive::cylinder(&at(5.0, 5.0, -1.0), 2.0, 12.0, "c").unwrap();
    check("drilled", &boolean::difference(&block, &rod).unwrap());
    let ball = primitive::sphere(&at(10.0, 10.0, 10.0), 4.0, "s").unwrap();
    check("corner ball", &boolean::union(&block, &ball).unwrap());
}

#[test]
fn cross_drilled() {
    let block = primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
    let rod = primitive::cylinder(&at(5.0, 5.0, -1.0), 2.0, 12.0, "c").unwrap();
    let side = primitive::cylinder(&Frame::new(Vec3::new(-1.0, 5.0, 5.0), Vec3::X, Vec3::Y).unwrap(), 3.0, 12.0, "x").unwrap();
    check("cross-drilled", &boolean::difference(&boolean::difference(&block, &rod).unwrap(), &side).unwrap());
}

/// Diagnostic: every coedge's uv samples, original vs round-tripped (printed, never failing).
#[test]
fn cross_drilled_pcurves_diagnostic() {
    use yack_springrep::topo::PCurve;
    let block = primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
    let rod = primitive::cylinder(&at(5.0, 5.0, -1.0), 2.0, 12.0, "c").unwrap();
    let side = primitive::cylinder(&Frame::new(Vec3::new(-1.0, 5.0, 5.0), Vec3::X, Vec3::Y).unwrap(), 3.0, 12.0, "x").unwrap();
    let s = boolean::difference(&boolean::difference(&block, &rod).unwrap(), &side).unwrap();
    let (body, names) = yack_bridge::to_cadrs(&s).unwrap();
    let back = yack_bridge::to_springrep(&body, &names).unwrap();
    let mut out = Vec::new();
    for (c, d) in s.coedge_ids().zip(back.coedge_ids()) {
        let e = s.edge(s.coedge(c).edge);
        let ts: Vec<f64> = (0..5).map(|i| e.range.0 + (e.range.1 - e.range.0) * i as f64 / 4.0).collect();
        let (a, b) = (s.coedge_uvs(c, &ts), back.coedge_uvs(d, &ts));
        let off = a.iter().zip(&b).map(|(p, q)| (p.x - q.x).abs().max((p.y - q.y).abs())).fold(0.0, f64::max);
        let kind = |p: &PCurve| if matches!(p, PCurve::Affine { .. }) { "affine" } else { "projected" };
        if off > 1e-9 || s.coedge(c).edge != back.coedge(d).edge || s.coedge(c).reversed != back.coedge(d).reversed {
            out.push(format!(
                "FAILED-DIAG coedge {} edge {}/{} rev {}/{} {}/{} face {} off {off:.3e} a {:?} b {:?}",
                c.index(), s.coedge(c).edge.index(), back.coedge(d).edge.index(),
                s.coedge(c).reversed, back.coedge(d).reversed,
                kind(&s.coedge(c).pcurve), kind(&back.coedge(d).pcurve),
                s.coedge_face(c).index(),
                a.iter().map(|p| (p.x, p.y)).collect::<Vec<_>>(), b.iter().map(|p| (p.x, p.y)).collect::<Vec<_>>()
            ));
        }
    }
    for (f, g) in s.face_ids().zip(back.face_ids()) {
        let (lf, lg) = (&s.face(f).loops, &back.face(g).loops);
        let cs = |b: &Brep, l: &[yack_springrep::topo::LoopId]| l.iter().map(|&x| b.lp(x).coedges.iter().map(|c| c.index()).collect::<Vec<_>>()).collect::<Vec<_>>();
        if cs(&s, lf) != cs(&back, lg) || s.face(f).reversed != back.face(g).reversed {
            out.push(format!("FAILED-DIAG face {} loops {:?} vs {:?} rev {}/{}", f.index(), cs(&s, lf), cs(&back, lg), s.face(f).reversed, back.face(g).reversed));
        }
    }
    println!("{}", out.join("\n"));
}

/// An edge no face uses, as cadrs leaves after some ops: `to_springrep` refuses it (indices would
/// move); `to_springrep_mapped` drops it and its map takes every other cadrs edge id to the
/// springrep edge with the same geometry.
#[test]
fn an_unused_cadrs_edge_is_dropped_and_mapped() {
    let block = primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 20.0, 30.0), "b").unwrap();
    let (mut body, names) = yack_bridge::to_cadrs(&block).unwrap();
    // Insert a stray edge at index 3, shifting every later edge's index up by one.
    let k = 3;
    let stray = body.edges[0].clone();
    body.edges.insert(k, stray);
    for co in body.faces.iter_mut().flat_map(|f| f.loops.iter_mut().flatten()) {
        if co.edge >= k {
            co.edge += 1;
        }
    }
    assert!(yack_bridge::to_springrep(&body, &names).is_err());
    let (back, map) = yack_bridge::to_springrep_mapped(&body, &names).unwrap();
    assert_eq!(map[k], None);
    assert!(yack_springrep::check::check(&back).is_empty());
    for (i, m) in map.iter().enumerate() {
        let Some(j) = *m else { continue };
        let ce = &body.edges[i];
        let se = back.edge(back.edge_ids().nth(j).unwrap());
        assert_eq!([se.range.0, se.range.1], ce.t, "edge {i} -> {j}");
        let (a, b) = (ce.curve.point(ce.t[0]), se.curve.as_ref().unwrap().eval(se.range.0));
        assert!((a.x - b.x).abs() + (a.y - b.y).abs() + (a.z - b.z).abs() < 1e-12, "edge {i} -> {j}");
    }
}

/// A cadrs-built box carried to springrep keeps its creases: no normal at any edge midpoint
/// (bench conformance "joints off" on cadrs-built solids).
#[test]
fn a_cadrs_box_keeps_its_creases() {
    use yack_cadrs::native::P3;
    use yack_cadrs::{DefaultKernel, Kernel, Primitive};
    let mut k = DefaultKernel::new();
    let id = k.primitive(&Primitive::Box { min: P3::new(0.0, 0.0, 0.0), max: P3::new(10.0, 20.0, 30.0) }).unwrap().first().unwrap();
    let body = (**k.get(id).unwrap()).clone();
    let back = yack_bridge::to_springrep(&body, &[]).unwrap();
    let outlines = yack_springrep::measure::Outlines::new(&back);
    let mut bad = Vec::new();
    for e in back.edge_ids() {
        let edge = back.edge(e);
        let Some(c) = &edge.curve else { continue };
        let p = c.eval(0.5 * (edge.range.0 + edge.range.1));
        if let Some(n) = outlines.normal_at(p, 1e-6) {
            bad.push(format!("edge {} at {p:?}: normal {n:?}", e.index()));
        }
    }
    assert!(bad.is_empty(), "\nFAILED creases lost: {}", bad.join("\nFAILED   "));
}

/// Regression (bench `fillet/box-all-edges`, cause BR): cadrs's box with every edge rounded has
/// sphere-corner faces whose loops cross u = 0; carried to springrep they must stay closed.
#[test]
fn a_cadrs_box_rounded_on_every_edge_carries_back() {
    use yack_cadrs::native::P3;
    use yack_cadrs::{DefaultKernel, Kernel, Primitive};
    let mut k = DefaultKernel::new();
    let id = k.primitive(&Primitive::Box { min: P3::new(0.0, 0.0, 0.0), max: P3::new(10.0, 10.0, 10.0) }).unwrap().first().unwrap();
    let edges: Vec<_> = k.edges(id).unwrap().into_iter().map(|e| e.id).collect();
    let r = k.fillet(id, &edges, 2.0).unwrap().first().unwrap();
    let body = (**k.get(r).unwrap()).clone();
    let cadrs_volume = k.mass_properties(r).unwrap().volume;
    let back = yack_bridge::to_springrep(&body, &[]).unwrap_or_else(|e| panic!("\nFAILED box-all-edges: {e}"));
    assert!(yack_springrep::check::check(&back).is_empty());
    let v = yack_springrep::measure::volume(&back);
    assert!((v - cadrs_volume).abs() <= 1e-6 * v, "\nFAILED box-all-edges volume {v} vs cadrs {cadrs_volume}");
    let r = yack_bridge::round_trip(&back);
    assert!(r.ok(), "\nFAILED box-all-edges round trip: {}", r.problems.join("\nFAILED   "));
}

/// The elliptic cylinder x²/25 + y²/9 ≤ 1, 0 ≤ z ≤ 10, from four conic arcs: its ellipse arcs go
/// to cadrs as rational B-splines (cadrs's form), and the round trip holds.
fn elliptic_cylinder() -> Brep {
    use yack_springrep::math::Vec2;
    use yack_springrep::profile::{NamedSegment, Profile, ProfileLoop, Segment};
    let w = 0.5f64.sqrt();
    let p = Vec2::new;
    let pts = [
        (p(5.0, 0.0), p(5.0, 3.0), p(0.0, 3.0)),
        (p(0.0, 3.0), p(-5.0, 3.0), p(-5.0, 0.0)),
        (p(-5.0, 0.0), p(-5.0, -3.0), p(0.0, -3.0)),
        (p(0.0, -3.0), p(5.0, -3.0), p(5.0, 0.0)),
    ];
    let segments = pts
        .iter()
        .enumerate()
        .map(|(i, &(a, apex, b))| NamedSegment::new(Segment::conic(a, apex, b, w).unwrap(), format!("s{i}")))
        .collect();
    yack_springrep::sweep::extrude(&Profile { loops: vec![ProfileLoop { segments }] }, &Frame::WORLD, 10.0, "e").unwrap()
}

#[test]
fn elliptic_cylinder_round_trips() {
    check("elliptic cylinder", &elliptic_cylinder());
}

/// offset/elliptic-cylinder-grown: cadrs offsets the bridged elliptic cylinder by 1 (it refused the
/// analytic-ellipse form); the bench's value is 896.45909906955167.
#[test]
fn bridged_elliptic_cylinder_offsets_on_cadrs() {
    use yack_cadrs::{DefaultKernel, Kernel, yack_glue};
    let mut k = DefaultKernel::new();
    let id = k.insert(yack_bridge::to_cadrs(&elliptic_cylinder()).unwrap().0);
    let (r, _) = yack_glue::offset(&mut k, id, 1.0).unwrap_or_else(|e| panic!("\nFAILED offset: {e}"));
    let v = k.mass_properties(r).unwrap().volume;
    // cadrs natively gives 896.4591175 too: 2e-8 relative from the bench value.
    assert!((v - 896.45909906955167).abs() < 1e-6 * v, "\nFAILED offset volume {v}");
}

/// A cadrs edge 1.9e-5 mm off its faces (as cadrs's blends on a spline dome leave one) is carried
/// with that tolerance; one 2e-4 mm off is refused.
#[test]
fn a_cadrs_edge_off_its_faces_is_carried_with_its_gap_or_refused() {
    use yack_cadrs::native::{Curve, P3};
    use yack_cadrs::{DefaultKernel, Kernel, Primitive};
    let mut k = DefaultKernel::new();
    let id = k.primitive(&Primitive::Box { min: P3::new(0.0, 0.0, 0.0), max: P3::new(10.0, 20.0, 30.0) }).unwrap().first().unwrap();
    let base = (**k.get(id).unwrap()).clone();
    let moved = |by: f64| {
        let mut b = base.clone();
        let e = b.edges.iter_mut().find(|e| matches!(e.curve, Curve::Line { .. })).unwrap();
        if let Curve::Line { o, d } = e.curve {
            // Off the edge, square to it.
            let n = if d.x.abs() < 0.9 { yack_cadrs::native::V3::new(1.0, 0.0, 0.0) } else { yack_cadrs::native::V3::new(0.0, 1.0, 0.0) };
            let n = (n - d * n.dot(&d)).normalize();
            e.curve = Curve::Line { o: o + n * by, d };
        }
        b
    };
    let near = yack_bridge::to_springrep(&moved(1.9e-5), &[]).unwrap_or_else(|e| panic!("\nFAILED 1.9e-5: {e}"));
    assert!(yack_springrep::check::check(&near).is_empty(), "\nFAILED 1.9e-5 check: {:?}", yack_springrep::check::check(&near));
    assert!(yack_bridge::to_springrep(&moved(2e-4), &[]).is_err(), "\nFAILED 2e-4 was carried");
}

/// The STEP-write backstop: a written block reads back as itself; told another volume, it refuses.
#[test]
fn a_written_step_is_checked_by_reading_it_back() {
    use yack_springrep::facade::exchange::{check_written, write_step};
    let b = primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 20.0, 30.0), "b").unwrap();
    let text = write_step(&b).unwrap();
    assert!(check_written(&text, 6000.0, 2200.0).is_ok());
    assert!(check_written(&text, 6100.0, 2200.0).is_err());
    assert!(check_written("not a step file", 6000.0, 2200.0).is_err());
}

/// Timing (printed to the scratchpad, never failing): the self-check's share of `to_springrep` on a
/// typical solid, the cross-drilled block, against `to_cadrs` and against the self-check alone.
#[test]
#[allow(clippy::disallowed_methods)] // a timing probe, test only
fn time_of_the_self_check() {
    use std::time::Instant;
    let block = primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0), "b").unwrap();
    let rod = primitive::cylinder(&at(5.0, 5.0, -1.0), 2.0, 12.0, "c").unwrap();
    let side = primitive::cylinder(&Frame::new(Vec3::new(-1.0, 5.0, 5.0), Vec3::X, Vec3::Y).unwrap(), 3.0, 12.0, "x").unwrap();
    let s = boolean::difference(&boolean::difference(&block, &rod).unwrap(), &side).unwrap();
    let n = 20;
    let t = Instant::now();
    let mut body = None;
    for _ in 0..n {
        body = Some(yack_bridge::to_cadrs(&s).unwrap().0);
    }
    let to_cadrs = t.elapsed().as_secs_f64() / n as f64;
    let body = body.unwrap();
    let t = Instant::now();
    for _ in 0..n {
        let _ = yack_bridge::to_springrep(&body, &[]).unwrap();
    }
    let to_springrep = t.elapsed().as_secs_f64() / n as f64;
    let line = format!("cross-drilled: to_cadrs {:.2} ms, to_springrep (with self-check) {:.2} ms\n", to_cadrs * 1e3, to_springrep * 1e3);
    let _ = std::fs::write("/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/selfcheck-time.txt", line);
}

/// The STEP readers' mesh report: an exact block is not a mesh; a file naming a faceted B-rep is.
#[test]
fn step_readers_report_a_mesh() {
    use yack_springrep::facade::exchange::{read_step_report, write_step};
    let b = primitive::block(&at(0.0, 0.0, 0.0), Vec3::new(10.0, 20.0, 30.0), "b").unwrap();
    let text = write_step(&b).unwrap();
    assert!(!read_step_report(&text).unwrap().1);
}

/// The cadrs-side STEP-write backstop: cadrs's block written by cadrs reads back as itself; told
/// another volume, it refuses.
#[test]
fn a_cadrs_written_step_is_checked_by_cadrs_reading_it_back() {
    use yack_cadrs::native::P3;
    use yack_cadrs::{DefaultKernel, Kernel, Primitive};
    let mut k = DefaultKernel::new();
    let id = k.primitive(&Primitive::Box { min: P3::new(0.0, 0.0, 0.0), max: P3::new(10.0, 20.0, 30.0) }).unwrap().first().unwrap();
    let text = String::from_utf8(k.export_step(&[id]).unwrap()).unwrap();
    assert!(yack_bridge::check_written_cadrs(&text, 6000.0, 2200.0).is_ok());
    assert!(yack_bridge::check_written_cadrs(&text, 6100.0, 2200.0).is_err());
}

/// bench swept/ring-lying-flat-turned-whole: cadrs's swing of a torus turned a whole turn about a
/// parallel axis beside it (761.0994599066700) crosses into springrep whole, or is refused; never
/// smaller (it once came over at 217.13).
#[test]
fn a_swung_ring_crosses_whole_or_not_at_all() {
    use yack_cadrs::{DefaultKernel, Kernel, yack_glue};
    let mut k = DefaultKernel::new();
    let ring = yack_glue::torus(&mut k, [8.0, 0.0, 0.0], [0.0, 0.0, 1.0], 3.0, 1.0).unwrap().0;
    let turn = yack_glue::Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: std::f64::consts::TAU };
    let (swept, _) = yack_glue::swept(&mut k, ring, &turn).unwrap();
    let want = k.mass_properties(swept).unwrap().volume;
    assert!((want - 761.09945990667).abs() < 1e-6 * want, "\nFAILED cadrs swing {want}");
    let body = (**k.get(swept).unwrap()).clone();
    match yack_bridge::to_springrep(&body, &[]) {
        Ok(b) => {
            let got = yack_springrep::measure::volume(&b);
            assert!((got - want).abs() < 1e-5 * want, "\nFAILED carried {got} of {want}");
        }
        Err(e) => {
            let _ = std::fs::write("/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/ring-probe.txt", e.to_string());
        }
    }
}

/// As `a_swung_ring_crosses_whole_or_not_at_all`, along the ladder's path: springrep's torus
/// carried into cadrs, swung there, and carried back.
#[test]
fn a_bridged_swung_ring_crosses_whole_or_not_at_all() {
    use yack_cadrs::{DefaultKernel, Kernel, yack_glue};
    let torus = primitive::torus(&at(8.0, 0.0, 0.0), 3.0, 1.0, "t").unwrap();
    let mut k = DefaultKernel::new();
    let ring = k.insert(yack_bridge::to_cadrs(&torus).unwrap().0);
    let turn = yack_glue::Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: std::f64::consts::TAU };
    let (swept, _) = yack_glue::swept(&mut k, ring, &turn).unwrap();
    let want = k.mass_properties(swept).unwrap().volume;
    let body = (**k.get(swept).unwrap()).clone();
    let r = yack_bridge::to_springrep(&body, &[]);
    let line = match &r {
        Ok(b) => format!("cadrs {want}, carried {}", yack_springrep::measure::volume(b)),
        Err(e) => format!("cadrs {want}, refused: {e}"),
    };
    let faces: Vec<String> = body.faces().iter().map(|f| format!("{:?}/{} loops", f.surf.kind(), f.loops.len())).collect();
    let _ = std::fs::write(
        "/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/ring-bridged.txt",
        format!("{line}\nfaces {}", faces.join(" ")),
    );
    assert!((want - 761.09945990667).abs() < 1e-6 * want, "\nFAILED cadrs swing {want}");
    if let Ok(b) = r {
        let got = yack_springrep::measure::volume(&b);
        assert!((got - want).abs() < 1e-5 * want, "\nFAILED carried {got} of {want}");
    }
}
