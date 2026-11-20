// yack: rolling-ball fillets round a rod, a slot, a D and a bitten boss standing through a sloping plane.
// Adapted from springrep@a03e8a1 tests/rolling_blends.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); edges named by face-name pairs are
// selected by role: springrep's `feature.face` is `faces_by(feature, role)` on a solid `named(feature)`,
// a profile's n-th side is `Role::Side("s{n}")` (a circle's `side`, the slot's `mid_v_min` and the
// round walls are each s0), and the loft's `side` is any `Role::Side` of its feature; springrep's centred
// `Profile::rect` is written as
// `Profile::rectangle(min, max)` and its `Profile::circle(c, diameter)` as `Profile::circle(c, radius)`;
// the ruled two-section loft `slope` (two rectangles of equal depth) is the same prism built by `extrude`
// of its trapezoid section along y; `unify` has no §4.3 counterpart and is dropped (it only preceded checks
// and measures); blend faces are the fillet's faces (`named` + `faces_tagged`), a NURBS blend is
// `SurfaceKind::Spline`; springrep's `s.eval` over `s.domain()` is `FaceRef::sample` (the same even grid,
// over the face's parameter box, ends included), an edge's 17 points are `EdgeRef::points(17)`, and a
// B-spline edge is `CurveKind::Spline`.
//! Fillets along edges no exact surface rounds, built round the rolling ball: the rim a rod, a
//! slot, a D and a bitten boss make on a sloping plane. Every point of each blend is held to its
//! radius from the curve the ball's centre runs on, worked by hand.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use yack::prelude::*;

/// How far `p` is from the circle ρ = 4 about the z axis lifted onto the plane z = 7.5 − x/4 +
/// `offset`, searched either side of `p`'s own angle.
fn from_lifted_circle(p: Point3, offset: f64) -> f64 {
    let centre = |phi: f64| {
        let (x, y) = (4.0 * phi.cos(), 4.0 * phi.sin());
        Point3::new(x, y, 7.5 - x / 4.0 + offset)
    };
    let (mut lo, mut hi) = (p.y.atan2(p.x) - 0.6, p.y.atan2(p.x) + 0.6);
    for _ in 0..200 {
        let (a, c) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
        if centre(a).distance(p) < centre(c).distance(p) {
            hi = c;
        } else {
            lo = a;
        }
    }
    centre(0.5 * (lo + hi)).distance(p)
}

/// The points of edge `e` at 17 even steps of its parameter.
fn along(b: &Solid, e: EdgeId) -> Vec<Point3> {
    b.edge(e).unwrap().points(17)
}

/// The blend faces `feature` made, each checked to be a NURBS (spline) surface.
fn blends(b: &Solid, feature: &str) -> Vec<FaceId> {
    let faces = b.faces_tagged(feature);
    for &f in &faces {
        assert!(matches!(b.face(f).unwrap().kind(), SurfaceKind::Spline), "the blend is a NURBS surface");
    }
    faces
}

fn line(a: (f64, f64), b: (f64, f64)) -> Segment {
    Segment::Line { a: [a.0, a.1].into(), b: [b.0, b.1].into() }
}

fn arc(center: (f64, f64), radius: f64, start: f64, sweep: f64) -> Segment {
    Segment::Arc { center: [center.0, center.1].into(), radius, start: rad(start), sweep: rad(sweep) }
}

fn outline(segments: Vec<Segment>) -> Profile {
    Profile::new(Loop::new(segments).unwrap(), vec![]).unwrap()
}

/// Every edge where `id`'s first side (s0) meets slope.side (any side face of the loft `slope`).
fn foot_edges(b: &Solid, id: &str) -> Vec<EdgeId> {
    let wall = b.faces_by(id, Role::Side("s0".into()));
    let slope = b.faces_where(|f| matches!(f.role("slope"), Some(Role::Side(_))));
    let mut out = Vec::new();
    for &x in &wall {
        for &y in &slope {
            out.extend(b.edges_between(x, y).unwrap());
        }
    }
    assert!(!out.is_empty(), "no edge between {id}.s0 and slope.side");
    out
}

/// A 20 × 20 block, its top cut to the plane z = 7.5 − x/4.
fn sloped_block() -> Solid {
    let block =
        extrude(&Frame::WORLD, &Profile::rectangle([-10.0, -10.0], [10.0, 10.0]).unwrap(), 10.0).unwrap().named("block");
    // The loft between (z 4, x 14–16) and (z 11, x −14–16), 30 deep: its section in the xz plane,
    // pushed from y = 15 to y = −15.
    let [(za, x0a, x1a), (zb, x0b, x1b)] = [(4.0, 14.0, 16.0), (11.0, -14.0, 16.0)];
    let xz = Frame::new([0.0, 15.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]).unwrap();
    let section = Profile::polygon(&[[x0a, za], [x1a, za], [x1b, zb], [x0b, zb]]).unwrap();
    block.difference(&extrude(&xz, &section, 30.0).unwrap().named("slope")).unwrap()
}

/// `profile` standing 14 high from the xy plane through the sloped block's top.
fn standing(profile: &Profile, id: &str) -> Solid {
    sloped_block().union(&extrude(&Frame::WORLD, profile, 14.0).unwrap().named(id)).unwrap()
}

/// The sloped block with a ⌀6 hole up z through the slope, or a ⌀6 boss 14 high standing through
/// it: the rim on the slope is an ellipse.
fn sloped_rod(boss: bool) -> Solid {
    let circle = Profile::circle([0.0, 0.0], 3.0).unwrap();
    if boss {
        standing(&circle, "rod")
    } else {
        let rod = extrude(&Frame::WORLD, &circle, 12.0).unwrap().named("rod");
        sloped_block().difference(&rod).unwrap()
    }
}

#[test]
fn a_rolling_ball_rounds_a_rod_through_a_slope() {
    // The ball rolls round the ellipse where the rod meets the plane z + x/4 = 7.5, so its centre
    // stays on the cylinder ρ = 3 + r, offset r from the plane: into the block round the hole,
    // above the slope round the boss. Every point of the blend lies r from that centre curve, and
    // each edge of the blend lies on its face.
    let r = 1.0;
    let lean = (1.0f64 + 1.0 / 16.0).sqrt();
    for boss in [false, true] {
        let s = sloped_rod(boss);
        // (rod.side, slope.side)
        let b = s.fillet(&foot_edges(&s, "rod"), r).unwrap().named("rim");
        assert!(b.check().is_empty());
        let plain = sloped_rod(boss).volume();
        let filled = b.volume();
        assert!(if boss { filled > plain } else { filled < plain });

        let offset = if boss { r * lean } else { -r * lean };
        let f = *blends(&b, "rim").first().expect("the blend face");
        // u at i/200 for i in 0..200, v at j/8 for j in 0..=8.
        let grid = b.face(f).unwrap().sample(201, 9);
        let on_plane = |p: Point3| ((p.z + p.x / 4.0 - 7.5) / lean).abs() < 1e-8;
        let on_rod = |p: Point3| (p.x.hypot(p.y) - 3.0).abs() < 1e-8;
        for row in grid.iter().take(200) {
            for &p in row {
                assert!((from_lifted_circle(p, offset) - r).abs() < 1e-8, "boss {boss}: {p:?}");
            }
            let (a, c) = (row[0], row[8]);
            assert!(on_plane(a) && on_rod(c) || on_rod(a) && on_plane(c), "boss {boss}: {a:?} {c:?}");
        }
    }
}

/// The sloped block with a slot 14 × 6 standing 14 high through it: flat sides y = ±3 for |x| ≤ 4
/// and round ends of radius 3 about (±4, 0).
fn sloped_slot() -> Solid {
    let slot = outline(vec![
        line((-4.0, -3.0), (4.0, -3.0)),          // s0: mid_v_min
        arc((4.0, 0.0), 3.0, -FRAC_PI_2, PI),     // s1: e2_side
        line((4.0, 3.0), (-4.0, 3.0)),            // s2: mid_v_max
        arc((-4.0, 0.0), 3.0, FRAC_PI_2, PI),     // s3: e1_side
    ]);
    standing(&slot, "slot")
}

#[test]
fn a_rolling_ball_rounds_a_slot_through_a_slope_across_its_tangent_faces() {
    // Round the slot's foot the ball touches the slope and, by turns, each flat side and each
    // round end, sliding from one onto the next where they meet tangentially. Its centre runs on
    // the slot's outline offset by r (half-width 4, ends of radius 4), lifted onto the plane r
    // above the slope; every point of the blend lies r from that curve.
    let r = 1.0;
    let s = sloped_slot();
    // (slot.mid_v_min, slope.side)
    let lean = (1.0f64 + 1.0 / 16.0).sqrt();
    let b = s.fillet(&foot_edges(&s, "slot"), r).unwrap().named("foot");
    assert!(b.check().is_empty());
    let plain = sloped_slot().volume();
    assert!(b.volume() > plain);

    let lift = |x: f64, y: f64| Point3::new(x, y, 7.5 - x / 4.0 + r * lean);
    let to_segment = |p: Point3, a: Point3, c: Point3| {
        let d = c - a;
        let t = ((p - a).dot(d) / d.dot(d)).clamp(0.0, 1.0);
        (a + d * t).distance(p)
    };
    let to_arc = |p: Point3, cx: f64, from: f64| {
        let at = |phi: f64| lift(cx + 4.0 * phi.cos(), 4.0 * phi.sin());
        let (mut lo, mut hi) = (from, from + PI);
        for _ in 0..200 {
            let (a, c) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
            if at(a).distance(p) < at(c).distance(p) {
                hi = c;
            } else {
                lo = a;
            }
        }
        at(0.5 * (lo + hi)).distance(p)
    };
    let from_centres = |p: Point3| {
        to_segment(p, lift(-4.0, 4.0), lift(4.0, 4.0))
            .min(to_segment(p, lift(-4.0, -4.0), lift(4.0, -4.0)))
            .min(to_arc(p, 4.0, -FRAC_PI_2))
            .min(to_arc(p, -4.0, FRAC_PI_2))
    };
    let on_slope = |p: Point3| ((p.z + p.x / 4.0 - 7.5) / lean).abs() < 1e-8;
    let on_slot = |p: Point3| {
        let wall = if p.x.abs() <= 4.0 { p.y.abs() - 3.0 } else { (p.x.abs() - 4.0).hypot(p.y) - 3.0 };
        wall.abs() < 1e-8
    };
    let blends = blends(&b, "foot");
    assert!(blends.len() >= 4, "{} blend faces", blends.len());
    for f in blends {
        // u at i/40 for i in 0..=40, v at j/8 for j in 0..=8.
        for row in b.face(f).unwrap().sample(41, 9) {
            for &p in &row {
                assert!((from_centres(p) - r).abs() < 1e-8, "{p:?}");
            }
            let (a, c) = (row[0], row[8]);
            assert!(on_slot(a) && on_slope(c) || on_slope(a) && on_slot(c), "{a:?} {c:?}");
        }
    }
}

/// The sloped block with a D 14 high standing through it: the ⌀6 circle less its half x > 0, so
/// its round wall meets a flat back on the plane x = 0.
fn sloped_d() -> Solid {
    let d = outline(vec![
        arc((0.0, 0.0), 3.0, FRAC_PI_2, PI), // s0: round_side
        line((0.0, -3.0), (0.0, 3.0)),       // s1: back_u_min
    ]);
    standing(&d, "d")
}

#[test]
fn a_rolling_ball_round_an_open_curved_edge_is_trimmed_by_the_face_at_each_end() {
    // The D's round wall meets the slope in half an ellipse, ending at y = ±3 on its flat back,
    // which it leaves at about 14°. Past each end the ball rolls on between the same two surfaces,
    // so every point of the blend still lies r from the circle ρ = 4 lifted r above the slope;
    // the flat back trims it, so its trim curves lie on x = 0.
    let r = 1.0;
    let s = sloped_d();
    // (d.round_side, slope.side)
    let lean = (1.0f64 + 1.0 / 16.0).sqrt();
    let b = s.fillet(&foot_edges(&s, "d"), r).unwrap().named("foot");
    assert!(b.check().is_empty());
    let plain = sloped_d().volume();
    assert!(b.volume() > plain);

    let blends = blends(&b, "foot");
    assert!(!blends.is_empty());
    let mut on_back = 0;
    for f in blends {
        // Sample the face itself: points of its surface inside its loops lie on the kept side.
        for row in b.face(f).unwrap().sample(41, 9) {
            for p in row {
                if p.x <= 0.0 {
                    assert!((from_lifted_circle(p, r * lean) - r).abs() < 1e-8, "{p:?}");
                }
            }
        }
        for e in b.edges_of(f).unwrap() {
            let points = along(&b, e);
            let on_plane = points.iter().all(|p| p.x.abs() < 1e-8);
            let at_back = points.iter().any(|p| p.x.abs() < 1e-3);
            let spline = matches!(b.edge(e).unwrap().kind(), CurveKind::Spline);
            if at_back && !spline {
                continue;
            }
            if on_plane {
                on_back += 1;
            }
        }
    }
    assert!(on_back >= 2, "{on_back} trim edges on the back");
}

/// The sloped block with a boss 14 high standing through it: the ⌀6 circle less the ⌀8 circle
/// about (5, 0), so its round wall meets a hollow back, the cylinder of radius 4 about x = 5, at
/// (1.8, ±2.4).
fn sloped_bitten() -> Solid {
    let (round, bite) = (2.4f64.atan2(1.8), 2.4f64.atan2(3.2));
    let boss = outline(vec![
        arc((0.0, 0.0), 3.0, round, TAU - 2.0 * round), // s0: round_side
        arc((5.0, 0.0), 4.0, bite - PI, -2.0 * bite),   // s1: bite_side
    ]);
    standing(&boss, "boss")
}

#[test]
fn a_rolling_ball_round_an_open_curved_edge_ends_on_a_cylinder() {
    // The boss's round wall meets the slope in an arc of ellipse ending on its hollow back. The
    // ball rolls round it as round a full rod, its centre on the circle ρ = 4 lifted r above the
    // slope, and the back's cylinder trims it at each end: the trim curves lie on that cylinder.
    let r = 1.0;
    let s = sloped_bitten();
    // (boss.round_side, slope.side)
    let lean = (1.0f64 + 1.0 / 16.0).sqrt();
    let b = s.fillet(&foot_edges(&s, "boss"), r).unwrap().named("foot");
    assert!(b.check().is_empty());
    let plain = sloped_bitten().volume();
    assert!(b.volume() > plain);

    let off_back = |p: Point3| (p.x - 5.0).hypot(p.y) - 4.0;
    let mut on_back = 0;
    for f in blends(&b, "foot") {
        for row in b.face(f).unwrap().sample(41, 9) {
            for p in row {
                if off_back(p) >= 0.0 {
                    assert!((from_lifted_circle(p, r * lean) - r).abs() < 1e-8, "{p:?}");
                }
            }
        }
        for e in b.edges_of(f).unwrap() {
            if !matches!(b.edge(e).unwrap().kind(), CurveKind::Spline) {
                continue;
            }
            if along(&b, e).iter().all(|&p| off_back(p).abs() < 1e-8) {
                on_back += 1;
            }
        }
    }
    assert!(on_back >= 2, "{on_back} trim edges on the back");
}
