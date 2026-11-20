// yack: the M1.9 exit check: one solid carried springrep → cadrs → springrep and compared (MAPPING.md §5).
use yack_cadrs::PointClass;
use yack_cadrs::native::{Body, CheckLevel, P3, classify, mass};
use yack_springrep::{
    check,
    math::Vec3,
    measure,
    rays::{Rays, Side},
    topo::Brep,
};

use crate::{to_cadrs, to_springrep};

/// How far volumes and areas may differ, relative, between the input and the solid brought back:
/// the same engine integrating the same geometry.
pub const AGREE: f64 = 1e-9; // tol: one integrator over one exact solid, twice

/// How far cadrs's own volume and areas may differ, relative, from springrep's on the carried body:
/// two engines' quadratures on spline and intersection faces agree to about 1e-7 (measured over
/// the bench, MAPPING.md §5).
pub const CADRS_AGREE: f64 = 1e-5; // tol: two engines' quadratures of one exact solid (lead ruling)

/// Sample points per axis of the classify grid.
const GRID: usize = 7;

/// What one round trip found. `problems` is empty when the solid crossed both ways intact.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RoundTrip {
    /// springrep's volume and area of the input.
    pub volume: f64,
    pub area: f64,
    /// The cause of each disagreement, grouped by a short stable prefix (`to-cadrs:`, `back:`,
    /// `order:`, `volume:`, `area:`, `face-area:`, `cadrs-measure:`, `crease:`, `classify:`, `cadrs-classify:`).
    pub problems: Vec<String>,
}

impl RoundTrip {
    /// Whether the solid crossed both ways intact.
    pub fn ok(&self) -> bool {
        self.problems.is_empty()
    }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1.0)
}

fn cadrs_area(b: &Body) -> f64 {
    (0..b.faces().len()).map(|f| mass::face_area(b, f).0).sum()
}

/// Points on a `GRID`³ lattice over the solid's box grown by 1 mm, nudged off symmetry planes.
fn samples(s: &Brep) -> Vec<Vec3> {
    let bb = measure::bbox(s);
    let grow = Vec3::new(1.0, 1.0, 1.0);
    let (lo, hi) = (bb.min - grow, bb.max + grow);
    let at = |n: usize, a: f64, b: f64| a + (b - a) * (n as f64 + 0.37) / GRID as f64;
    let mut out = Vec::with_capacity(GRID * GRID * GRID);
    for i in 0..GRID {
        for j in 0..GRID {
            for k in 0..GRID {
                out.push(Vec3::new(at(i, lo.x, hi.x), at(j, lo.y, hi.y), at(k, lo.z, hi.z)));
            }
        }
    }
    out
}

fn class_matches(s: Side, c: PointClass) -> bool {
    matches!(
        (s, c),
        (Side::Inside, PointClass::Inside) | (Side::Outside, PointClass::Outside) | (Side::On, PointClass::OnBoundary)
    )
}

/// Carries `s` to cadrs and back, and compares volume, area, each face's area, entity order, both
/// checkers and point classification on a grid, on every side.
pub fn round_trip(s: &Brep) -> RoundTrip {
    let (volume, area) = (measure::volume(s), measure::area(s));
    let mut r = RoundTrip { volume, area, problems: Vec::new() };
    let mut say = |p: String| r.problems.push(p);
    let (body, names) = match to_cadrs(s) {
        Ok(x) => x,
        Err(e) => {
            say(format!("to-cadrs: {e}"));
            return r;
        }
    };
    let issues = body.check_with(CheckLevel::Full);
    if !issues.is_empty() {
        say(format!("to-cadrs: cadrs check: {}", issues.join("; ")));
    }
    let back = match to_springrep(&body, &names) {
        Ok(b) => b,
        Err(e) => {
            say(format!("back: {e}"));
            return r;
        }
    };
    // Vertices and faces match one for one; cadrs may hold extra degenerate pole edges, all after
    // springrep's (MAPPING.md §4), and they come back as springrep degenerate edges.
    let counts = |b: &Brep| (b.vertex_ids().count(), b.face_ids().count());
    let c = (body.verts().len(), body.faces().len());
    let n_edges = s.edge_ids().count();
    let extra_ok = body.edges().len() >= n_edges
        && body.edges().iter().skip(n_edges).all(|e| e.degenerate)
        && back.edge_ids().count() == body.edges().len();
    if counts(s) != c || counts(&back) != c || !extra_ok {
        say(format!(
            "order: vertices/faces {:?} / {c:?} / {:?}, edges {} / {} / {}",
            counts(s),
            counts(&back),
            n_edges,
            body.edges().len(),
            back.edge_ids().count()
        ));
    }
    for (v, cv) in s.vertex_ids().zip(body.verts()) {
        let p = s.vertex(v).point;
        if [p.x, p.y, p.z] != [cv.p.x, cv.p.y, cv.p.z] {
            say(format!("order: vertex {} moved", v.index()));
        }
    }
    for (e, ce) in s.edge_ids().zip(body.edges()) {
        if [s.edge(e).range.0, s.edge(e).range.1] != ce.t {
            say(format!("order: edge {} range changed", e.index()));
        }
    }
    let (cv, bv) = (mass::body_volume(&body), measure::volume(&back));
    if rel(bv, volume) > AGREE {
        say(format!("volume: springrep {volume} back {bv}"));
    }
    if rel(cv, volume) > CADRS_AGREE {
        say(format!("cadrs-measure: volume springrep {volume} cadrs {cv}"));
    }
    let (ca, ba) = (cadrs_area(&body), measure::area(&back));
    if rel(ba, area) > AGREE {
        say(format!("area: springrep {area} back {ba}"));
    }
    if rel(ca, area) > CADRS_AGREE {
        say(format!("cadrs-measure: area springrep {area} cadrs {ca}"));
    }
    for ((f, g), i) in s.face_ids().zip(back.face_ids()).zip(0..) {
        let (a, c, b) = (measure::face_area(s, f), mass::face_area(&body, i).0, measure::face_area(&back, g));
        if rel(b, a) > AGREE {
            say(format!("face-area: face {i}: springrep {a} back {b}"));
        }
        if rel(c, a) > CADRS_AGREE {
            say(format!("cadrs-measure: face {i} area springrep {a} cadrs {c}"));
        }
    }
    let back_issues = check::check(&back);
    if !back_issues.is_empty() {
        let m: Vec<String> = back_issues.into_iter().map(|i| i.message).collect();
        say(format!("back: springrep check: {}", m.join("; ")));
    }
    // Creases: at each edge's midpoint, springrep's normal is None where the faces meet at an
    // angle; the body brought back must agree edge for edge.
    let (os, ob) = (measure::Outlines::new(s), measure::Outlines::new(&back));
    let within = 10.0 * yack_springrep::tol::LINEAR;
    for (e, g) in s.edge_ids().zip(back.edge_ids()) {
        let edge = s.edge(e);
        let Some(curve) = &edge.curve else { continue };
        let p = curve.eval(0.5 * (edge.range.0 + edge.range.1));
        let (a, b) = (os.normal_at(p, within), ob.normal_at(p, within));
        if a.is_none() != b.is_none() {
            say(format!(
                "crease: edge {} midpoint ({}, {}, {}): normal {} before, {} after (edge {})",
                e.index(),
                p.x,
                p.y,
                p.z,
                if a.is_some() { "some" } else { "none" },
                if b.is_some() { "some" } else { "none" },
                g.index()
            ));
        }
    }
    match (Rays::new(s), Rays::new(&back)) {
        (Ok(rs), Ok(rb)) => {
            let polys: Vec<_> = body.faces().iter().map(|f| classify::face_polygons(&body, f)).collect();
            for p in samples(s) {
                let want = rs.side(p);
                let got = rb.side(p);
                if got != want {
                    say(format!("classify: back {got:?} vs {want:?} at ({}, {}, {})", p.x, p.y, p.z));
                }
                let c = classify::classify_point(&body, &polys, &P3::new(p.x, p.y, p.z), yack_springrep::tol::LINEAR);
                if !class_matches(want, c) {
                    say(format!("cadrs-classify: cadrs {c:?} vs {want:?} at ({}, {}, {})", p.x, p.y, p.z));
                }
            }
        }
        _ => say("classify: no ray caster".to_string()),
    }
    r
}
