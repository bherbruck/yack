// yack: probes for the two ladder setups that go wrong on cadrs after the bridge (printed): a springrep
// solid carried to cadrs and given cadrs's op, against cadrs's own solid given the same op.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_cadrs::native::Body;
use yack_cadrs::facade::profile::{Place, Seg};
use yack_cadrs::{DefaultKernel, Kernel, yack_glue};
use yack_math::kernel::{Frame, Vec2, Vec3};
use yack_profile::{NamedSegment, Profile, ProfileLoop, Segment};

/// Where the probes write (outside the builder's trimmed log).
fn dump(lines: &[String]) {
    use std::io::Write;
    let path = "/tmp/claude-1000/-home-brenn-github-yack/dfbaa7ec-edce-4285-9cb7-59a45b5f05a4/scratchpad/probe.txt";
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
        let _ = writeln!(f, "{}", lines.join("\n"));
    }
    println!("{}", lines.join("\n"));
}

fn describe(b: &Body) -> String {
    use yack_cadrs::native::{Curve, Surface};
    let curve = |c: &Curve| match c {
        Curve::Line { .. } => "line".to_string(),
        Curve::Circle { .. } => "circle".to_string(),
        Curve::Ellipse { .. } => "ellipse".to_string(),
        Curve::Nurbs(n) => format!("nurbs(p{} k{} w{})", n.degree, n.knots.len(), n.is_rational()),
    };
    b.faces()
        .iter()
        .enumerate()
        .map(|(i, f)| {
            let s = match &f.surf {
                Surface::Extrusion { c, d } => format!("extrusion[{} d({:.2},{:.2},{:.2})]", curve(c), d.x, d.y, d.z),
                Surface::Nurbs(n) => format!("nurbs-surface[p{}x{} k{}x{}]", n.du, n.dv, n.ku.len(), n.kv.len()),
                other => format!("{:?}", other.kind()),
            };
            let edges: Vec<String> = f.loops.iter().flatten().map(|c| format!("{}{}", c.edge, if c.fwd { "+" } else { "-" })).collect();
            let ranges: Vec<String> = f.loops.iter().flatten().map(|c| { let e = &b.edges()[c.edge]; format!("{}:[{:.3},{:.3}]{}", c.edge, e.t[0], e.t[1], curve(&e.curve)) }).collect();
            format!("\n  {i}: {s} sense {} edges {} {}", f.sense, edges.join(","), ranges.join(" "))
        })
        .collect::<String>()
}

fn measure(k: &DefaultKernel, id: yack_cadrs::BodyId) -> String {
    let m = k.mass_properties(id).unwrap();
    format!("volume {} area {} faces {}", m.volume, m.surface_area, k.get(id).unwrap().faces.len())
}

#[test]
fn ring_slid_down() {
    let (s, c) = (65f64.to_radians().sin(), 65f64.to_radians().cos());
    let f = Frame::new(Vec3::new(0.0, 0.0, 4.0), Vec3::new(s, 0.0, c), Vec3::new(c, 0.0, -s)).unwrap();
    let sr = yack_build::primitive::torus(&f, 3.0, 1.0, "t").unwrap();
    let mut k = DefaultKernel::new();
    let bridged = k.insert(yack_bridge::to_cadrs(&sr).unwrap().0);
    let native = yack_glue::torus(&mut k, [0.0, 0.0, 4.0], [s, 0.0, c], 3.0, 1.0).unwrap().0;
    let mut out = Vec::new();
    for (name, id) in [("bridged", bridged), ("native", native)] {
        out.push(format!("FAILED-PROBE ring {name}: {} | {}", measure(&k, id), describe(k.get(id).unwrap())));
        match yack_glue::swept(&mut k, id, &yack_glue::Move::Slide { by: [0.0, 0.0, -8.0] }) {
            Ok((r, _)) => out.push(format!("FAILED-PROBE ring {name} swept: {}", measure(&k, r))),
            Err(e) => out.push(format!("FAILED-PROBE ring {name} swept: error {e}")),
        }
    }
    dump(&out);
}

/// The body with every circle or ellipse edge whose range starts below 0 moved on by 2π (the same
/// curve), its pcurves re-parametrised to match; with `u`, extrusion faces' u moved on by 2π too.
fn shift_conics(mut b: Body, u: bool) -> Body {
    use yack_cadrs::native::{Curve, Surface, V3, Affine};
    use std::f64::consts::TAU;
    let moved: Vec<bool> = b.edges.iter().map(|e| matches!(e.curve, Curve::Circle { .. } | Curve::Ellipse { .. }) && e.t[0] < 0.0).collect();
    for (e, m) in b.edges.iter_mut().zip(&moved) {
        if *m {
            e.t = [e.t[0] + TAU, e.t[1] + TAU];
        }
    }
    let reparam = |pc: &Curve| -> Curve {
        match pc {
            Curve::Line { o, d } => Curve::Line { o: o - d * TAU, d: *d },
            Curve::Nurbs(n) => {
                let mut n = (**n).clone();
                for k in &mut n.knots {
                    *k += TAU;
                }
                Curve::Nurbs(std::sync::Arc::new(n))
            }
            other => other.clone(),
        }
    };
    for f in &mut b.faces {
        let extrusion = matches!(f.surf, Surface::Extrusion { .. });
        for co in f.loops.iter_mut().flatten() {
            if moved[co.edge] {
                co.pc = reparam(&co.pc);
            }
            if u && extrusion {
                co.pc = co.pc.transformed(&Affine::translation(V3::new(TAU, 0.0, 0.0)));
            }
        }
    }
    b
}

#[test]
fn elliptic_cylinder_offset() {
    let w = 0.5f64.sqrt();
    let p = |x: f64, y: f64| Vec2::new(x, y);
    let pts = [(p(5.0, 0.0), p(5.0, 3.0), p(0.0, 3.0)), (p(0.0, 3.0), p(-5.0, 3.0), p(-5.0, 0.0)), (p(-5.0, 0.0), p(-5.0, -3.0), p(0.0, -3.0)), (p(0.0, -3.0), p(5.0, -3.0), p(5.0, 0.0))];
    let segments = pts.iter().enumerate().map(|(i, &(a, apex, b))| NamedSegment::new(Segment::conic(a, apex, b, w).unwrap(), format!("s{i}"))).collect();
    let profile = Profile { loops: vec![ProfileLoop { segments }] };
    let sr = yack_build::sweep::extrude(&profile, &Frame::WORLD, 10.0, "e").unwrap();
    let mut k = DefaultKernel::new();
    let body = yack_bridge::to_cadrs(&sr).unwrap().0;
    let bridged = k.insert(body.clone());
    let shifted_t = k.insert(shift_conics(body.clone(), false));
    let shifted_tu = k.insert(shift_conics(body, true));
    let segs: Vec<Seg> = pts.iter().map(|&(a, apex, b)| Seg::Conic { a: [a.x, a.y], apex: [apex.x, apex.y], b: [b.x, b.y], weight: w }).collect();
    let place = Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    let native = yack_cadrs::facade::profile::extrude(&mut k, &place, &[&segs], 10.0).unwrap().0;
    let mut out = Vec::new();
    for (name, id) in [("bridged", bridged), ("shifted-t", shifted_t), ("shifted-t-and-u", shifted_tu), ("native", native)] {
        out.push(format!("FAILED-PROBE ellipse {name}: {} | {}", measure(&k, id), describe(k.get(id).unwrap())));
        match yack_glue::offset(&mut k, id, 1.0) {
            Ok((r, _)) => out.push(format!("FAILED-PROBE ellipse {name} offset: {}", measure(&k, r))),
            Err(e) => out.push(format!("FAILED-PROBE ellipse {name} offset: error {e}")),
        }
    }
    dump(&out);
}
