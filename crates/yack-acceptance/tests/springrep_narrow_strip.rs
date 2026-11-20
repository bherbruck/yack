// yack: a face 90 tolerances wide is kept as a face, measured, and has the area it should.
// Adapted from springrep@a03e8a1 tests/narrow_strip.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); the named outline is a `Loop` of
// `Segment`s; the strip face (springrep's "x.body_u_min") is picked as the one plane facing −x; `unify`
// has no §4.3 counterpart: the call after extrusion is dropped and the re-unify face count is an API gap.
//! A face a hundred times narrower than a tolerance of 1e-5 mm, but wider than this kernel's
//! 1e-7 mm, is kept as a face: swept from an outline edge 90 tolerances long, measured, and left alone when faces are
//! unified.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use yack::SurfaceKind;
use yack::prelude::*;

/// A 3 by 1 + `step` mm rect on a circle of radius 4 at the origin, extruded 6 mm: the rect's
/// top runs `step` past the circle's top point, so its left side leaves a strip `step` wide.
fn rect_past_a_circle(step: f64) -> Solid {
    let line = |a: (f64, f64), b: (f64, f64)| Segment::Line { a: [a.0, a.1].into(), b: [b.0, b.1].into() };
    let r7 = 7f64.sqrt();
    let top = 4.0 + step;
    // The circle from its top point round to where it drops below the rect's foot, at (√7, 3).
    let segments = vec![
        Segment::Arc {
            center: [0.0, 0.0].into(),
            radius: 4.0,
            start: rad(FRAC_PI_2),
            sweep: rad(TAU - (FRAC_PI_2 - 3f64.atan2(r7))),
        }, // floor_side
        line((r7, 3.0), (3.0, 3.0)),   // body_v_min
        line((3.0, 3.0), (3.0, top)),  // body_u_max
        line((3.0, top), (0.0, top)),  // body_v_max
        line((0.0, top), (0.0, 4.0)),  // body_u_min
    ];
    let outline = Profile::new(Loop::new(segments).unwrap(), vec![]).unwrap();
    extrude(&Frame::WORLD, &outline, 6.0).unwrap()
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
    assert!((b.volume() - want).abs() < 1e-9 * want);
    assert_eq!(b.faces().len(), 7);
    // body_u_min, the segment on x = 0, is the outline's only side facing −x.
    let strip = b.faces_where(|f| matches!(f.kind(), SurfaceKind::Plane { normal } if normal.x < -0.5));
    assert_eq!(strip.len(), 1);
    assert!((b.face(strip[0]).unwrap().area() - 6.0 * s).abs() < 1e-9);
    // API GAP: unify(&b).face_count() == 7 needs an op that merges faces lying on one surface.
}
