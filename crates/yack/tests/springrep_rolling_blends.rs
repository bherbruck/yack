// yack: Fillets along edges no exact surface rounds, built round the rolling ball: the rim a rod, a
// Adapted from springrep@a03e8a1 tests/rolling_blends.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
//! Fillets along edges no exact surface rounds, built round the rolling ball: the rim a rod, a
//! slot, a D and a bitten boss make on a sloping plane. Every point of each blend is held to its
//! radius from the curve the ball's centre runs on, worked by hand.
// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the
// kernel crates directly and compute their expected values, so the facade's maths ban and the
// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.
// TODO(api-owner, hardening, D26): rewrite these tests to the lints, test by test, and drop this allow.
#![allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use yack_blend::fillet;
use yack_boolean::difference;
use yack_boolean::unify;
use yack_boolean::union;
use yack_check as check;
use yack_geom::Curve;
use yack_geom::Surface;
use yack_build::loft::Section;
use yack_build::loft::loft;
use yack_math::kernel::Frame;
use yack_math::kernel::Vec2;
use yack_math::kernel::Vec3;
use yack_measure as measure;
use yack_profile::NamedSegment;
use yack_profile::Profile;
use yack_build::sweep::extrude;
use yack_topo::Brep;
use yack_topo::EdgeId;

fn up(z: f64) -> Frame {
    Frame::new(Vec3::new(0.0, 0.0, z), Vec3::Z, Vec3::X).unwrap()
}

fn line(a: (f64, f64), b: (f64, f64), name: &str) -> NamedSegment {
    NamedSegment::line(Vec2::new(a.0, a.1), Vec2::new(b.0, b.1), name)
}

fn arc(center: (f64, f64), radius: f64, start: f64, sweep: f64, name: &str) -> NamedSegment {
    NamedSegment::arc(Vec2::new(center.0, center.1), radius, start, sweep, name)
}

/// A 20 × 20 block, its top cut to the plane z = 7.5 − x/4.
fn sloped_block() -> Brep {
    let block = extrude(
        &Profile::rect(Vec2::ZERO, 20.0, 20.0),
        &up(0.0),
        10.0,
        "block",
    )
    .unwrap();
    let sections =
        [(4.0, 14.0, 16.0), (11.0, -14.0, 16.0)].map(|(z, x0, x1): (f64, f64, f64)| Section {
            profile: Profile::rect(Vec2::new(0.5 * (x0 + x1), 0.0), x1 - x0, 30.0),
            frame: up(z),
        });
    difference(&block, &loft(&sections, true, "slope").unwrap()).unwrap()
}

/// `profile` standing 14 high from the xy plane through the sloped block's top.
fn standing(profile: &Profile, id: &str) -> Brep {
    union(
        &sloped_block(),
        &extrude(profile, &up(0.0), 14.0, id).unwrap(),
    )
    .unwrap()
}

/// The sloped block with a ⌀6 hole up z through the slope, or a ⌀6 boss 14 high standing through
/// it: the rim on the slope is an ellipse.
fn sloped_rod(boss: bool) -> Brep {
    let circle = Profile::circle(Vec2::ZERO, 6.0);
    if boss {
        standing(&circle, "rod")
    } else {
        let rod = extrude(&circle, &up(0.0), 12.0, "rod").unwrap();
        difference(&sloped_block(), &rod).unwrap()
    }
}

/// How far `p` is from the circle ρ = 4 about the z axis lifted onto the plane z = 7.5 − x/4 +
/// `offset`, searched either side of `p`'s own angle.
fn from_lifted_circle(p: Vec3, offset: f64) -> f64 {
    let centre = |phi: f64| {
        let (x, y) = (4.0 * phi.cos(), 4.0 * phi.sin());
        Vec3::new(x, y, 7.5 - x / 4.0 + offset)
    };
    let (mut lo, mut hi) = (p.y.atan2(p.x) - 0.6, p.y.atan2(p.x) + 0.6);
    for _ in 0..200 {
        let (a, c) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
        if centre(a).dist(p) < centre(c).dist(p) {
            hi = c;
        } else {
            lo = a;
        }
    }
    centre(0.5 * (lo + hi)).dist(p)
}

/// The points of edge `e` at 17 even steps of its parameter.
fn along(b: &Brep, e: EdgeId) -> Vec<Vec3> {
    let x = b.edge(e);
    let curve = x.curve.as_ref().unwrap();
    (0..=16)
        .map(|k| curve.eval(x.range.0 + (x.range.1 - x.range.0) * k as f64 / 16.0))
        .collect()
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
        let b = fillet(&sloped_rod(boss), &[("rod.side", "slope.side")], r, "rim").unwrap();
        let b = unify(&b).unwrap();
        assert!(check::check(&b).is_empty());
        let plain = measure::volume(&unify(&sloped_rod(boss)).unwrap());
        let filled = measure::volume(&b);
        assert!(if boss { filled > plain } else { filled < plain });

        let offset = if boss { r * lean } else { -r * lean };
        let f = b
            .face_ids()
            .find(|&f| b.face(f).names.iter().any(|n| n.feature == "rim"))
            .expect("the blend face");
        let Surface::Nurbs(s) = &b.face(f).surface else {
            panic!("the blend is a NURBS surface");
        };
        let ((u0, u1), (v0, v1)) = s.domain();
        let on_plane = |p: Vec3| ((p.z + p.x / 4.0 - 7.5) / lean).abs() < 1e-8;
        let on_rod = |p: Vec3| (p.x.hypot(p.y) - 3.0).abs() < 1e-8;
        for i in 0..200 {
            let u = u0 + (u1 - u0) * i as f64 / 200.0;
            for j in 0..=8 {
                let p = s.eval(u, v0 + (v1 - v0) * j as f64 / 8.0);
                assert!(
                    (from_lifted_circle(p, offset) - r).abs() < 1e-8,
                    "boss {boss}: {p:?}"
                );
            }
            let (a, c) = (s.eval(u, v0), s.eval(u, v1));
            assert!(
                on_plane(a) && on_rod(c) || on_rod(a) && on_plane(c),
                "boss {boss}: {a:?} {c:?}"
            );
        }
    }
}

/// The sloped block with a slot 14 × 6 standing 14 high through it: flat sides y = ±3 for |x| ≤ 4
/// and round ends of radius 3 about (±4, 0).
fn sloped_slot() -> Brep {
    let slot = Profile::outline(vec![
        line((-4.0, -3.0), (4.0, -3.0), "mid_v_min"),
        arc((4.0, 0.0), 3.0, -FRAC_PI_2, PI, "e2_side"),
        line((4.0, 3.0), (-4.0, 3.0), "mid_v_max"),
        arc((-4.0, 0.0), 3.0, FRAC_PI_2, PI, "e1_side"),
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
    let lean = (1.0f64 + 1.0 / 16.0).sqrt();
    let b = fillet(
        &sloped_slot(),
        &[("slot.mid_v_min", "slope.side")],
        r,
        "foot",
    )
    .unwrap();
    let b = unify(&b).unwrap();
    assert!(check::check(&b).is_empty());
    let plain = measure::volume(&unify(&sloped_slot()).unwrap());
    assert!(measure::volume(&b) > plain);

    let lift = |x: f64, y: f64| Vec3::new(x, y, 7.5 - x / 4.0 + r * lean);
    let to_segment = |p: Vec3, a: Vec3, c: Vec3| {
        let d = c - a;
        let t = ((p - a).dot(d) / d.dot(d)).clamp(0.0, 1.0);
        (a + d * t).dist(p)
    };
    let to_arc = |p: Vec3, cx: f64, from: f64| {
        let at = |phi: f64| lift(cx + 4.0 * phi.cos(), 4.0 * phi.sin());
        let (mut lo, mut hi) = (from, from + PI);
        for _ in 0..200 {
            let (a, c) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
            if at(a).dist(p) < at(c).dist(p) {
                hi = c;
            } else {
                lo = a;
            }
        }
        at(0.5 * (lo + hi)).dist(p)
    };
    let from_centres = |p: Vec3| {
        to_segment(p, lift(-4.0, 4.0), lift(4.0, 4.0))
            .min(to_segment(p, lift(-4.0, -4.0), lift(4.0, -4.0)))
            .min(to_arc(p, 4.0, -FRAC_PI_2))
            .min(to_arc(p, -4.0, FRAC_PI_2))
    };
    let on_slope = |p: Vec3| ((p.z + p.x / 4.0 - 7.5) / lean).abs() < 1e-8;
    let on_slot = |p: Vec3| {
        let wall = if p.x.abs() <= 4.0 {
            p.y.abs() - 3.0
        } else {
            (p.x.abs() - 4.0).hypot(p.y) - 3.0
        };
        wall.abs() < 1e-8
    };
    let blends: Vec<_> = b
        .face_ids()
        .filter(|&f| b.face(f).names.iter().any(|n| n.feature == "foot"))
        .collect();
    assert!(blends.len() >= 4, "{} blend faces", blends.len());
    for f in blends {
        let Surface::Nurbs(s) = &b.face(f).surface else {
            panic!("the blend is a NURBS surface");
        };
        let ((u0, u1), (v0, v1)) = s.domain();
        for i in 0..=40 {
            let u = u0 + (u1 - u0) * i as f64 / 40.0;
            for j in 0..=8 {
                let p = s.eval(u, v0 + (v1 - v0) * j as f64 / 8.0);
                assert!((from_centres(p) - r).abs() < 1e-8, "{p:?}");
            }
            let (a, c) = (s.eval(u, v0), s.eval(u, v1));
            assert!(
                on_slot(a) && on_slope(c) || on_slope(a) && on_slot(c),
                "{a:?} {c:?}"
            );
        }
    }
}

/// The sloped block with a D 14 high standing through it: the ⌀6 circle less its half x > 0, so
/// its round wall meets a flat back on the plane x = 0.
fn sloped_d() -> Brep {
    let d = Profile::outline(vec![
        arc((0.0, 0.0), 3.0, FRAC_PI_2, PI, "round_side"),
        line((0.0, -3.0), (0.0, 3.0), "back_u_min"),
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
    let lean = (1.0f64 + 1.0 / 16.0).sqrt();
    let b = fillet(&sloped_d(), &[("d.round_side", "slope.side")], r, "foot").unwrap();
    let b = unify(&b).unwrap();
    assert!(check::check(&b).is_empty());
    let plain = measure::volume(&unify(&sloped_d()).unwrap());
    assert!(measure::volume(&b) > plain);

    let blends: Vec<_> = b
        .face_ids()
        .filter(|&f| b.face(f).names.iter().any(|n| n.feature == "foot"))
        .collect();
    assert!(!blends.is_empty());
    let mut on_back = 0;
    for f in blends {
        let Surface::Nurbs(s) = &b.face(f).surface else {
            panic!("the blend is a NURBS surface");
        };
        // Sample the face itself: points of its surface inside its loops lie on the kept side.
        let ((u0, u1), (v0, v1)) = s.domain();
        for i in 0..=40 {
            let u = u0 + (u1 - u0) * i as f64 / 40.0;
            for j in 0..=8 {
                let p = s.eval(u, v0 + (v1 - v0) * j as f64 / 8.0);
                if p.x <= 0.0 {
                    assert!((from_lifted_circle(p, r * lean) - r).abs() < 1e-8, "{p:?}");
                }
            }
        }
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                let e = b.coedge(c).edge;
                let points = along(&b, e);
                let on_plane = points.iter().all(|p| p.x.abs() < 1e-8);
                let at_back = points.iter().any(|p| p.x.abs() < 1e-3);
                let spline = matches!(b.edge(e).curve, Some(Curve::BSpline(_)));
                if at_back && !spline {
                    continue;
                }
                if on_plane {
                    on_back += 1;
                }
            }
        }
    }
    assert!(on_back >= 2, "{on_back} trim edges on the back");
}

/// The sloped block with a boss 14 high standing through it: the ⌀6 circle less the ⌀8 circle
/// about (5, 0), so its round wall meets a hollow back, the cylinder of radius 4 about x = 5, at
/// (1.8, ±2.4).
fn sloped_bitten() -> Brep {
    let (round, bite) = (2.4f64.atan2(1.8), 2.4f64.atan2(3.2));
    let boss = Profile::outline(vec![
        arc((0.0, 0.0), 3.0, round, TAU - 2.0 * round, "round_side"),
        arc((5.0, 0.0), 4.0, bite - PI, -2.0 * bite, "bite_side"),
    ]);
    standing(&boss, "boss")
}

#[test]
fn a_rolling_ball_round_an_open_curved_edge_ends_on_a_cylinder() {
    // The boss's round wall meets the slope in an arc of ellipse ending on its hollow back. The
    // ball rolls round it as round a full rod, its centre on the circle ρ = 4 lifted r above the
    // slope, and the back's cylinder trims it at each end: the trim curves lie on that cylinder.
    let r = 1.0;
    let lean = (1.0f64 + 1.0 / 16.0).sqrt();
    let b = fillet(
        &sloped_bitten(),
        &[("boss.round_side", "slope.side")],
        r,
        "foot",
    )
    .unwrap();
    let b = unify(&b).unwrap();
    assert!(check::check(&b).is_empty());
    let plain = measure::volume(&unify(&sloped_bitten()).unwrap());
    assert!(measure::volume(&b) > plain);

    let off_back = |p: Vec3| (p.x - 5.0).hypot(p.y) - 4.0;
    let mut on_back = 0;
    for f in b.face_ids() {
        if !b.face(f).names.iter().any(|n| n.feature == "foot") {
            continue;
        }
        let Surface::Nurbs(s) = &b.face(f).surface else {
            panic!("the blend is a NURBS surface");
        };
        let ((u0, u1), (v0, v1)) = s.domain();
        for i in 0..=40 {
            let u = u0 + (u1 - u0) * i as f64 / 40.0;
            for j in 0..=8 {
                let p = s.eval(u, v0 + (v1 - v0) * j as f64 / 8.0);
                if off_back(p) >= 0.0 {
                    assert!((from_lifted_circle(p, r * lean) - r).abs() < 1e-8, "{p:?}");
                }
            }
        }
        for &lp in &b.face(f).loops {
            for &c in &b.lp(lp).coedges {
                let e = b.coedge(c).edge;
                if !matches!(b.edge(e).curve, Some(Curve::BSpline(_))) {
                    continue;
                }
                if along(&b, e).iter().all(|&p| off_back(p).abs() < 1e-8) {
                    on_back += 1;
                }
            }
        }
    }
    assert!(on_back >= 2, "{on_back} trim edges on the back");
}
