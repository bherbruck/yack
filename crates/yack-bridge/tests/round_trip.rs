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
