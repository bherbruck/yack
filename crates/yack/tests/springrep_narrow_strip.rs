// yack: A face a hundred times narrower than a tolerance of 1e-5 mm, but wider than this kernel's
// Adapted from springrep@a03e8a1 tests/narrow_strip.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
//! A face a hundred times narrower than a tolerance of 1e-5 mm, but wider than this kernel's
//! 1e-7 mm, is kept as a face: swept from an outline edge 90 tolerances long, measured, and left alone when faces are
//! unified.
// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the
// kernel crates directly and compute their expected values, so the facade's maths ban and the
// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.
// TODO(api-owner, hardening, D26): rewrite these tests to the lints, test by test, and drop this allow.
#![allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use yack_boolean::unify;
use yack_math::kernel::Frame;
use yack_math::kernel::Vec2;
use yack_measure as measure;
use yack_profile::NamedSegment;
use yack_profile::Profile;
use yack_profile::ProfileLoop;
use yack_profile::Segment;
use yack_build::sweep::extrude;
use yack_topo::Brep;

/// A 3 by 1 + `step` mm rect on a circle of radius 4 at the origin, extruded 6 mm: the rect's
/// top runs `step` past the circle's top point, so its left side leaves a strip `step` wide.
fn rect_past_a_circle(step: f64) -> Brep {
    let line = |a: (f64, f64), b: (f64, f64), name: &str| {
        NamedSegment::new(
            Segment::Line {
                a: Vec2::new(a.0, a.1),
                b: Vec2::new(b.0, b.1),
            },
            name,
        )
    };
    let r7 = 7f64.sqrt();
    let top = 4.0 + step;
    // The circle from its top point round to where it drops below the rect's foot, at (√7, 3).
    let segments = vec![
        NamedSegment::new(
            Segment::Arc {
                center: Vec2::ZERO,
                radius: 4.0,
                start: FRAC_PI_2,
                sweep: TAU - (FRAC_PI_2 - 3f64.atan2(r7)),
            },
            "floor_side",
        ),
        line((r7, 3.0), (3.0, 3.0), "body_v_min"),
        line((3.0, 3.0), (3.0, top), "body_u_max"),
        line((3.0, top), (0.0, top), "body_v_max"),
        line((0.0, top), (0.0, 4.0), "body_u_min"),
    ];
    let outline = Profile {
        loops: vec![ProfileLoop { segments }],
    };
    unify(&extrude(&outline, &Frame::WORLD, 6.0, "x").unwrap()).unwrap()
}

#[test]
fn a_strip_90_tolerances_wide_stays_a_face() {
    // The channels' corners lie 8.9e-6 mm outside their floor circles. At a tolerance of 1e-5
    // mm such a strip is narrower than the tolerance; at 1e-7 mm it is a face, and merging it
    // would move the neighbouring edge 90 tolerances. The area outside the circle is ∫ (4 + s −
    // √(16 − x²)) to x = √7, where the circle drops below the rect, plus the full (3 − √7) by (1 +
    // s) beyond.
    let s = 8.936e-6;
    let b = rect_past_a_circle(s);
    let r7 = 7f64.sqrt();
    let outside = r7 * (4.0 + s) - 1.5 * r7 - 8.0 * (r7 / 4.0).asin() + (3.0 - r7) * (1.0 + s);
    let want = 6.0 * (16.0 * PI + outside);
    assert!((measure::volume(&b) - want).abs() < 1e-9 * want);
    assert_eq!(b.face_count(), 7);
    let strip: Vec<_> = b
        .face_ids()
        .filter(|&f| {
            b.face(f)
                .names
                .iter()
                .any(|n| n.to_string() == "x.body_u_min")
        })
        .collect();
    assert_eq!(strip.len(), 1);
    assert!((measure::face_area(&b, strip[0]) - 6.0 * s).abs() < 1e-9);
    assert_eq!(unify(&b).unwrap().face_count(), 7);
}
