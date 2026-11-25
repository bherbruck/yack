// yack: tests of curve–surface and curve–curve intersection.
// Adapted from springrep@a03e8a1 src/boolean/csi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of csi.rs into its own file (M3 hardening); unchanged otherwise.

use std::f64::consts::TAU;

use super::{curves::sampled_curve_curve, *};
use crate::{
    geom::{Circle, Cylinder, Line, Plane, Torus},
    math::Frame,
    numeric,
};

fn line(o: [f64; 3], d: [f64; 3]) -> Curve {
    Curve::Line(Line { origin: Vec3::from_array(o), dir: Vec3::from_array(d).unit().unwrap() })
}

/// A closed cubic whose right side ripples across the plane x = 5 every hundredth of a
/// millimetre, extruded along z: the small features of a gear's root in outline.
fn rippled_prism() -> (crate::geom::BSpline, Surface) {
    let mut ctrl: Vec<Vec3> = Vec::new();
    // Up the right side in ripples, then back round the left in a wide arc.
    for k in 0..16 {
        let x = 5.0 + if k % 2 == 0 { 0.004 } else { -0.004 };
        ctrl.push(Vec3::new(x, -0.075 + 0.01 * k as f64, 0.0));
    }
    for k in 1..8 {
        let a = TAU / 4.0 + TAU / 2.0 * k as f64 / 8.0;
        ctrl.push(Vec3::new(5.0 + 4.0 * a.cos(), 4.0 * a.sin(), 0.0));
    }
    let n = ctrl.len();
    ctrl.extend_from_within(..3);
    let knots = (0..n + 7).map(|i| i as f64 - 3.0).collect();
    let curve = crate::geom::BSpline::periodic_cubic(knots, ctrl).unwrap();
    let surface = Surface::Extrusion(Box::new(crate::geom::Extrusion {
        curve: Curve::BSpline(curve.clone()),
        dir: Vec3::Z,
    }));
    (curve, surface)
}

#[test]
fn a_line_along_an_extrusions_ripples_meets_it_at_every_crossing() {
    let (curve, surface) = rippled_prism();
    let (u0, u1) = curve.domain();
    let steps = 400_000;
    let at = |i: usize| u0 + (u1 - u0) * i as f64 / steps as f64;
    for shift in [0.0, 0.001, -0.001] {
        // A line in the plane x = 5 + shift, climbing as it goes: it runs along the ripples
        // and crosses the profile at each.
        let o = Vec3::new(5.0 + shift, -20.0, -3.0);
        let d = Vec3::new(0.0, 1.0, 0.2).unit().unwrap();
        let l = Curve::Line(Line { origin: o, dir: d });
        // Every crossing, from the profile itself: where it passes through the line's plane.
        let side = |u: f64| curve.eval(u).x - (5.0 + shift);
        let mut want = Vec::new();
        for i in 0..steps {
            let (a, b) = (side(at(i)), side(at(i + 1)));
            if (a < 0.0) != (b < 0.0) {
                let u = numeric::bisect(&mut |u| side(u), at(i), at(i + 1));
                want.push((curve.eval(u) - o).y / d.y);
            }
        }
        want.sort_by(f64::total_cmp);
        assert!(want.len() >= 10, "shift {shift}: {} crossings", want.len());
        let got = curve_surface(&l, (-100.0, 100.0), &surface);
        assert!(!got.on_surface);
        let mut ts = got.ts.clone();
        ts.sort_by(f64::total_cmp);
        assert_eq!(ts.len(), want.len(), "shift {shift}: {ts:?} against {want:?}");
        for (t, w) in ts.iter().zip(&want) {
            assert!((t - w).abs() < 1e-7, "shift {shift}: {ts:?} against {want:?}");
        }
    }
}

/// Every crossing of a line with an extrusion along z of `profile`, as the profile's own
/// passings through the line's plane `x = at`, each as the line's parameter: the answer a
/// search along the line must reach.
fn passings(profile: &Curve, at: f64, o: Vec3, d: Vec3) -> Vec<f64> {
    let (u0, u1) = profile.domain();
    let steps = 400_000;
    let u = |i: usize| u0 + (u1 - u0) * i as f64 / steps as f64;
    let side = |t: f64| profile.eval(t).x - at;
    let mut out = Vec::new();
    for i in 0..steps {
        let (a, b) = (side(u(i)), side(u(i + 1)));
        if (a < 0.0) != (b < 0.0) {
            let t = numeric::bisect(&mut |t| side(t), u(i), u(i + 1));
            out.push((profile.eval(t) - o).y / d.y);
        }
    }
    out.sort_by(f64::total_cmp);
    out
}

#[test]
fn a_line_along_a_rational_extrusions_ripples_meets_it_at_every_crossing() {
    // An open rational cubic rippling across x = 5, its weights uneven, extruded along z.
    let ctrl: Vec<Vec3> = (0..20)
        .map(|k| {
            let x = 5.0 + if k % 2 == 0 { 0.004 } else { -0.004 };
            Vec3::new(x, -0.1 + 0.01 * k as f64, 0.0)
        })
        .collect();
    let weights = (0..20).map(|k| 1.0 + 0.7 * (k % 3) as f64).collect();
    let mut knots = vec![0.0; 4];
    knots.extend((1..17).map(|i| i as f64));
    knots.extend([17.0; 4]);
    let nurbs = crate::geom::NurbsCurve::new(3, knots, ctrl, Some(weights)).unwrap();
    let profile = Curve::Nurbs(Box::new(nurbs));
    let surface = Surface::Extrusion(Box::new(crate::geom::Extrusion {
        curve: profile.clone(),
        dir: Vec3::Z,
    }));
    for shift in [0.0, 0.0007, -0.0005] {
        let o = Vec3::new(5.0 + shift, -20.0, -3.0);
        let d = Vec3::new(0.0, 1.0, 0.2).unit().unwrap();
        let want = passings(&profile, 5.0 + shift, o, d);
        assert!(want.len() >= 4, "shift {shift}: {} crossings", want.len());
        let got =
            curve_surface(&Curve::Line(Line { origin: o, dir: d }), (-100.0, 100.0), &surface);
        let mut ts = got.ts.clone();
        ts.sort_by(f64::total_cmp);
        assert_eq!(ts.len(), want.len(), "shift {shift}: {ts:?} against {want:?}");
        for (t, w) in ts.iter().zip(&want) {
            assert!((t - w).abs() < 1e-7, "shift {shift}: {ts:?} against {want:?}");
        }
    }
}

#[test]
fn a_line_meets_an_extrusion_three_times_within_a_fraction_of_one_piece() {
    // One cubic piece, x = 5 + 300·(s − 0.505)(s − 0.515)(s − 0.525), y = 10·s: it passes
    // through the plane x = 5 three times within a fiftieth of its length. With weights in
    // geometric progression it is the same curve, its parameter stretched towards one end.
    let (a, b, c) = (0.505, 0.515, 0.525);
    let k = 300.0;
    let power = [-k * a * b * c, k * (a * b + b * c + c * a), -k * (a + b + c), k];
    let bezier = [
        power[0],
        power[0] + power[1] / 3.0,
        power[0] + 2.0 * power[1] / 3.0 + power[2] / 3.0,
        power[0] + power[1] + power[2] + power[3],
    ];
    let ctrl: Vec<Vec3> =
        (0..4).map(|i| Vec3::new(5.0 + bezier[i], 10.0 * i as f64 / 3.0, 0.0)).collect();
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    for weights in [None, Some(vec![1.0, 2.0, 4.0, 8.0])] {
        let nurbs = crate::geom::NurbsCurve::new(3, knots.clone(), ctrl.clone(), weights);
        let profile = Curve::Nurbs(Box::new(nurbs.unwrap()));
        let surface = Surface::Extrusion(Box::new(crate::geom::Extrusion {
            curve: profile.clone(),
            dir: Vec3::Z,
        }));
        let o = Vec3::new(5.0, -20.0, -3.0);
        let d = Vec3::new(0.0, 1.0, 0.2).unit().unwrap();
        let want = passings(&profile, 5.0, o, d);
        assert_eq!(want.len(), 3);
        let line = Curve::Line(Line { origin: o, dir: d });
        let mut ts = curve_surface(&line, (-100.0, 100.0), &surface).ts;
        ts.sort_by(f64::total_cmp);
        assert_eq!(ts.len(), 3, "{ts:?} against {want:?}");
        for (t, w) in ts.iter().zip(&want) {
            assert!((t - w).abs() < 1e-7, "{ts:?} against {want:?}");
        }
    }
}

/// An open cubic in the plane y = 0 rippling across the radius 5 as it climbs z.
fn rippled_meridian() -> Curve {
    let ctrl: Vec<Vec3> = (0..20)
        .map(|k| {
            let r = 5.0 + if k % 2 == 0 { 0.004 } else { -0.004 };
            Vec3::new(r, 0.0, -0.1 + 0.01 * k as f64)
        })
        .collect();
    let mut knots = vec![0.0; 4];
    knots.extend((1..17).map(|i| i as f64));
    knots.extend([17.0; 4]);
    Curve::Nurbs(Box::new(crate::geom::NurbsCurve::new(3, knots, ctrl, None).unwrap()))
}

/// Where the line meets the surface of revolution of `profile` about z, from the profile:
/// each of its points is a ring, and the line passes that ring's height at a known distance
/// from the axis. The answer a search along the line must reach, for a line that climbs.
fn ring_passings(profile: &Curve, o: Vec3, d: Vec3) -> Vec<f64> {
    let (u0, u1) = profile.domain();
    let steps = 400_000;
    let u = |i: usize| u0 + (u1 - u0) * i as f64 / steps as f64;
    let line_at = |z: f64| (z - o.z) / d.z;
    let side = |t: f64| {
        let q = profile.eval(t);
        let x = o + d * line_at(q.z);
        x.x.hypot(x.y) - q.x.hypot(q.y)
    };
    let mut out = Vec::new();
    for i in 0..steps {
        let (a, b) = (side(u(i)), side(u(i + 1)));
        if (a < 0.0) != (b < 0.0) {
            let t = numeric::bisect(&mut |t| side(t), u(i), u(i + 1));
            out.push(line_at(profile.eval(t).z));
        }
    }
    out.sort_by(f64::total_cmp);
    out
}

#[test]
fn a_line_along_a_revolved_surfaces_ripples_meets_it_at_every_crossing() {
    let profile = rippled_meridian();
    let surface = Surface::Revolution(Box::new(crate::geom::Revolution {
        curve: profile.clone(),
        origin: Vec3::ZERO,
        axis: Vec3::Z,
    }));
    // Up the side at radius 5, along the axis and then askew to it, from far below.
    for (o, d) in [
        (Vec3::new(5.0, 0.0, -30.0), Vec3::Z),
        (Vec3::new(3.0, 4.0005, -30.0), Vec3::Z),
        (Vec3::new(5.0 - 0.3, 0.4, -30.0), Vec3::new(0.01, -0.013, 1.0)),
    ] {
        let d = d.unit().unwrap();
        let want = ring_passings(&profile, o, d);
        assert!(want.len() >= 6, "{o:?}: {} crossings", want.len());
        let line = Curve::Line(Line { origin: o, dir: d });
        let mut ts = curve_surface(&line, (0.0, 100.0), &surface).ts;
        ts.sort_by(f64::total_cmp);
        assert_eq!(ts.len(), want.len(), "{o:?}: {ts:?} against {want:?}");
        for (t, w) in ts.iter().zip(&want) {
            assert!((t - w).abs() < 1e-7, "{o:?}: {ts:?} against {want:?}");
        }
    }
}

#[test]
fn a_line_square_to_a_revolved_surfaces_axis_meets_its_ring_twice() {
    let profile = rippled_meridian();
    let surface = Surface::Revolution(Box::new(crate::geom::Revolution {
        curve: profile.clone(),
        origin: Vec3::ZERO,
        axis: Vec3::Z,
    }));
    // At a height the profile passes once, the surface is one ring: its radius from the
    // profile, and the line's two crossings of a circle of that radius from a quadratic.
    let z = 0.0312;
    let (u0, u1) = profile.domain();
    let u = numeric::bisect(&mut |t| profile.eval(t).z - z, u0, u1);
    let ring = profile.eval(u).x;
    let (o, d) = (Vec3::new(-20.0, 1.5, z), Vec3::X);
    let half = (ring * ring - 1.5 * 1.5).sqrt();
    let line = Curve::Line(Line { origin: o, dir: d });
    let mut ts = curve_surface(&line, (0.0, 100.0), &surface).ts;
    ts.sort_by(f64::total_cmp);
    assert_eq!(ts.len(), 2, "{ts:?}");
    assert!((ts[0] - (20.0 - half)).abs() < 1e-7 && (ts[1] - (20.0 + half)).abs() < 1e-7);
}

#[test]
fn a_line_along_a_ruled_surfaces_ripples_meets_it_at_every_crossing() {
    // Rulings from the rippled curve to the same curve 3 mm along y: the surface is the
    // curve's extrusion along y, drawn as a ruled one, so a line in a plane of constant y
    // meets it where it meets the curve.
    let near = rippled_meridian();
    let far = match &near {
        Curve::Nurbs(n) => Curve::Nurbs(Box::new(
            n.transformed(&crate::math::Transform::translation(Vec3::new(0.0, 3.0, 0.0))),
        )),
        _ => unreachable!(),
    };
    let range = near.domain();
    let surface = Surface::Ruled(Box::new(crate::geom::Ruled {
        a: near.clone(),
        a_range: range,
        b: far,
        b_range: range,
    }));
    for shift in [0.0, 0.0007, -0.0005] {
        // Up through the ripples in the plane y = 1, leaning along y a little.
        let o = Vec3::new(5.0 + shift, 1.0, -30.0);
        let d = Vec3::new(0.0, 0.02, 1.0).unit().unwrap();
        // The curve passes the plane x = 5 + shift where the line, which stays in it, meets
        // the ruling through that point.
        let steps = 400_000;
        let u = |i: usize| range.0 + (range.1 - range.0) * i as f64 / steps as f64;
        let side = |t: f64| near.eval(t).x - (5.0 + shift);
        let mut want = Vec::new();
        for i in 0..steps {
            let (a, b) = (side(u(i)), side(u(i + 1)));
            if (a < 0.0) != (b < 0.0) {
                let t = numeric::bisect(&mut |t| side(t), u(i), u(i + 1));
                want.push((near.eval(t).z - o.z) / d.z);
            }
        }
        want.sort_by(f64::total_cmp);
        assert!(want.len() >= 6, "shift {shift}: {} crossings", want.len());
        let line = Curve::Line(Line { origin: o, dir: d });
        let mut ts = curve_surface(&line, (0.0, 100.0), &surface).ts;
        ts.sort_by(f64::total_cmp);
        assert_eq!(ts.len(), want.len(), "shift {shift}: {ts:?} against {want:?}");
        for (t, w) in ts.iter().zip(&want) {
            assert!((t - w).abs() < 1e-7, "shift {shift}: {ts:?} against {want:?}");
        }
    }
}

#[test]
fn a_line_through_a_torus_whose_tube_reaches_its_axis_meets_both_its_sheets() {
    // A tube of radius 2 about a circle of radius 1: it passes the axis, and the surface has
    // an outer sheet and an inner one. Along a diameter the line meets them at 3 and at 1
    // from the axis, either side.
    let torus = Surface::Torus(Torus { frame: Frame::WORLD, major: 1.0, minor: 2.0 });
    let line = |o: [f64; 3], d: [f64; 3]| line(o, d);
    let mut ts = curve_surface(&line([-10.0, 0.0, 0.0], [1.0, 0.0, 0.0]), (0.0, 100.0), &torus).ts;
    ts.sort_by(f64::total_cmp);
    assert_eq!(ts.len(), 4, "{ts:?}");
    for (t, w) in ts.iter().zip([7.0, 9.0, 11.0, 13.0]) {
        assert!((t - w).abs() < 1e-9, "{ts:?}");
    }
    // Askew: every crossing lies on one sheet or the other, and a fine walk along the line
    // finds as many.
    let (o, d) = (Vec3::new(-9.0, 0.3, -0.4), Vec3::new(1.0, 0.02, 0.05).unit().unwrap());
    let sheet = |t: f64, sign: f64| {
        let q = o + d * t;
        (q.x.hypot(q.y) + sign).hypot(q.z) - 2.0
    };
    let both = |t: f64| sheet(t, -1.0) * sheet(t, 1.0);
    let steps = 2_000_000;
    let changes = (0..steps)
        .filter(|&i| {
            let (a, b) = (20.0 * i as f64 / steps as f64, 20.0 * (i + 1) as f64 / steps as f64);
            (both(a) < 0.0) != (both(b) < 0.0)
        })
        .count();
    let ts = curve_surface(&Curve::Line(Line { origin: o, dir: d }), (0.0, 20.0), &torus).ts;
    assert_eq!(ts.len(), changes, "{ts:?}");
    for &t in &ts {
        assert!(sheet(t, -1.0).abs().min(sheet(t, 1.0).abs()) < 1e-9, "{t}");
    }
}

/// A cubic by cubic sheet over the square 0..0.24 by 0..0.24 whose height ripples along x
/// every hundredth of a millimetre: its x and y run evenly with its parameters, so the point
/// above (x, y) is the surface at (x/0.24, y/0.24), and a line's height over it is known at
/// every point of the line.
fn rippled_sheet() -> crate::geom::NurbsSurface {
    let n = 24;
    let mut knots = vec![0.0; 4];
    knots.extend((1..n - 3).map(|i| i as f64 / (n - 3) as f64));
    knots.extend([1.0; 4]);
    // Greville abscissae: control points placed there make a coordinate run evenly.
    let greville = |i: usize| (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3.0;
    let mut ctrl = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let z = if i % 2 == 0 { 0.004 } else { -0.004 };
            ctrl.push(Vec3::new(0.24 * greville(i), 0.24 * greville(j), z));
        }
    }
    crate::geom::NurbsSurface::new((3, 3), (knots.clone(), knots), (n, n), ctrl, None).unwrap()
}

#[test]
fn a_line_skimming_a_spline_sheets_ripples_meets_it_at_every_crossing() {
    let sheet = rippled_sheet();
    let surface = Surface::Nurbs(Box::new(sheet.clone()));
    for (o, d) in [
        // Level with the sheet's middle height, across the ripples and askew to them.
        (Vec3::new(-1.0, 0.1, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        (Vec3::new(-1.0, 0.05, 0.0003), Vec3::new(1.0, 0.05, 0.0)),
        // Falling through it at a shallow angle.
        (Vec3::new(-1.0, 0.2, 0.004), Vec3::new(1.0, -0.02, -0.0035)),
    ] {
        let d = d.unit().unwrap();
        let above = |t: f64| {
            let q = o + d * t;
            q.z - sheet.eval(q.x / 0.24, q.y / 0.24).z
        };
        let over = |t: f64| {
            let q = o + d * t;
            (0.0..=0.24).contains(&q.x) && (0.0..=0.24).contains(&q.y)
        };
        // A walk along the line in steps a hundredth of a ripple long.
        let steps = 1_000_000;
        let at = |i: usize| 8.0 * i as f64 / steps as f64;
        let mut want = Vec::new();
        for i in 0..steps {
            if over(at(i)) && over(at(i + 1)) && (above(at(i)) < 0.0) != (above(at(i + 1)) < 0.0) {
                want.push(numeric::bisect(&mut |t| above(t), at(i), at(i + 1)));
            }
        }
        assert!(want.len() >= 6, "{o:?}: {} crossings", want.len());
        let line = Curve::Line(Line { origin: o, dir: d });
        let mut ts = curve_surface(&line, (0.0, 8.0), &surface).ts;
        ts.sort_by(f64::total_cmp);
        assert_eq!(ts.len(), want.len(), "{o:?}: {} against {}", ts.len(), want.len());
        for (t, w) in ts.iter().zip(&want) {
            assert!((t - w).abs() < 1e-6, "{o:?}: {t} against {w}");
        }
    }
}

fn cylinder(r: f64) -> Surface {
    Surface::Cylinder(Cylinder { frame: Frame::WORLD, radius: r })
}

#[test]
fn a_line_touching_a_torus_within_rounding_meets_it_once_where_it_touches() {
    // The tube (major 5.5, minor 1.5 about z) reaches in to r = 4 at z = 0. A line along z at
    // r = 4 + 3e-11 dips 3e-11 into it, crossing at z = ±√(3·3e-11) ≈ ±9.5e-6: one touch.
    let torus = Surface::Torus(crate::geom::Torus { frame: Frame::WORLD, major: 5.5, minor: 1.5 });
    let hits = curve_surface(&line([4.0 + 3e-11, 0.0, 0.0], [0.0, 0.0, 1.0]), (-5.0, 5.0), &torus);
    assert_eq!(hits.ts.len(), 1, "{hits:?}");
    assert!(hits.ts[0].abs() < 1e-7, "{hits:?}");
}

#[test]
fn a_line_cutting_a_tube_between_two_samples_meets_it_twice() {
    // The tube about z, centred at z = 0.05, cut by a line along z at r = 4 + d where
    // (1.5 − d)² = 1.5² − 0.04²: it crosses at z = 0.05 ± 0.04, both between the samples at
    // z = 0 and z ≈ 0.104 of the 96 over [−5, 5].
    let torus = Surface::Torus(crate::geom::Torus {
        frame: Frame::WORLD.translated(Vec3::new(0.0, 0.0, 0.05)),
        major: 5.5,
        minor: 1.5,
    });
    let d = 1.5 - (1.5f64 * 1.5 - 0.04 * 0.04).sqrt();
    let hits = curve_surface(&line([4.0 + d, 0.0, 0.0], [0.0, 0.0, 1.0]), (-5.0, 5.0), &torus);
    assert_eq!(hits.ts.len(), 2, "{hits:?}");
    assert!((hits.ts[0] - 0.01).abs() < 1e-9, "{hits:?}");
    assert!((hits.ts[1] - 0.09).abs() < 1e-9, "{hits:?}");
}

#[test]
fn line_meets_cylinder_twice_once_or_never() {
    let s = cylinder(5.0);
    let two = curve_surface(&line([-10.0, 4.999999, 0.0], [1.0, 0.0, 0.0]), (0.0, 20.0), &s);
    assert_eq!(two.ts.len(), 2);
    let gap = (two.ts[1] - two.ts[0]).abs();
    assert!((gap - 2.0 * (5.0f64.powi(2) - 4.999999f64.powi(2)).sqrt()).abs() < 1e-12);
    let touch = curve_surface(&line([-10.0, 5.0, 0.0], [1.0, 0.0, 0.0]), (0.0, 20.0), &s);
    assert_eq!(touch.ts, vec![10.0]);
    let miss = curve_surface(&line([-10.0, 5.000001, 0.0], [1.0, 0.0, 0.0]), (0.0, 20.0), &s);
    assert!(miss.ts.is_empty() && !miss.on_surface);
    let along = curve_surface(&line([5.0, 0.0, 0.0], [0.0, 0.0, 1.0]), (0.0, 20.0), &s);
    assert!(along.on_surface);
}

#[test]
fn circle_meets_plane_in_closed_form() {
    let c = Curve::Circle(Circle { frame: Frame::WORLD, radius: 5.0 });
    let at = |x: f64| {
        Surface::Plane(Plane {
            frame: Frame::new(Vec3::new(x, 0.0, 0.0), Vec3::X, Vec3::Y).unwrap(),
        })
    };
    assert_eq!(curve_surface(&c, (0.0, TAU), &at(3.0)).ts.len(), 2);
    let touch = curve_surface(&c, (0.0, TAU), &at(-5.0)).ts;
    assert_eq!(touch.len(), 1);
    assert!((touch[0] - TAU / 2.0).abs() < 1e-12);
    assert!(
        curve_surface(&c, (0.0, TAU), &Surface::Plane(Plane { frame: Frame::WORLD })).on_surface
    );
}

#[test]
fn a_round_touching_its_pillar_meets_it_at_the_touching_point() {
    // A quarter round's meridian edge, off the origin and turned, touching its pillar; rounding
    // there made sign changes 3e-8 mm from the touching point.
    for k in 0..64 {
        let a = 0.1 * k as f64;
        let axis = Vec3::new(20.0 * a.sin(), 0.0, 20.0 * (1.3 * a).cos());
        let out = Vec3::new(a.cos(), 0.0, a.sin());
        let c = Curve::Circle(Circle {
            frame: Frame::new(axis + out * 4.5 - Vec3::Y * 16.0, out.cross(Vec3::Y), Vec3::Y)
                .unwrap(),
            radius: 3.0,
        });
        let pillar = Surface::Cylinder(Cylinder {
            frame: Frame::new(axis, -Vec3::Y, out).unwrap(),
            radius: 1.5,
        });
        let touch = axis + out * 1.5 - Vec3::Y * 16.0;
        let end =
            (0..4).map(|k| k as f64 * TAU / 4.0).find(|&t| c.eval(t).dist(touch) <= 1e-12).unwrap();
        // The range ends on the touching point, as the round's does, so the touch need not be
        // reported; a crossing reported beside it would be a second vertex.
        for t in curve_surface(&c, (end - TAU / 4.0, end), &pillar).ts {
            let d = c.eval(t).dist(touch);
            assert!(d <= 1e-9, "{k} {d}");
        }
    }
}

#[test]
fn a_ray_clipping_a_lobe_of_an_extruded_outline_meets_it_twice() {
    // The wall r = 30 + 3·cos 9θ extruded along z, and rays along y at x = 33 − δ, a hair in
    // from the lobe tip at (33, 0): near the tip x ≈ 33 − 0.127·y², so each ray enters and
    // leaves 2·√(δ / 0.127) apart, under 0.06 mm for δ = 1e-4, a fraction of the step its
    // 140 mm are sampled at. Each must meet the wall as often as the outline crosses its line,
    // counted on a dense sampling of the outline.
    let r = |a: f64| 30.0 + 3.0 * (9.0 * a).cos();
    let pts: Vec<Vec3> = (0..360)
        .map(|i| {
            let a = TAU * i as f64 / 360.0;
            Vec3::new(r(a) * a.cos(), r(a) * a.sin(), 0.0)
        })
        .collect();
    let curve = Curve::BSpline(crate::geom::BSpline::interpolate_periodic(&pts).unwrap());
    let wall =
        Surface::Extrusion(Box::new(crate::geom::Extrusion { curve: curve.clone(), dir: Vec3::Z }));
    let (t0, t1) = curve.domain();
    let dense: Vec<Vec3> =
        (0..=400_000).map(|k| curve.eval(t0 + (t1 - t0) * k as f64 / 400_000.0)).collect();
    let tip = dense.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
    for delta in [1e-4, 3e-4, 1e-3, 3e-3, 1e-2] {
        let x = tip - delta;
        let o = Vec3::new(x, -70.0, 1.0);
        let want = dense.windows(2).filter(|w| (w[0].x - x) * (w[1].x - x) < 0.0).count();
        assert!(want >= 2, "δ = {delta}");
        let hits = ray_surface(&wall, o, Vec3::Y, 140.0);
        assert_eq!(hits.len(), want, "δ = {delta}: {hits:?}");
        for t in hits {
            assert!(signed_distance(&wall, o + Vec3::Y * t).abs() < 1e-9);
        }
    }
}

#[test]
fn numeric_hits_find_a_circle_crossing_a_cylinder() {
    let c = Curve::Circle(Circle {
        frame: Frame::new(Vec3::new(4.0, 0.0, 0.0), Vec3::Z, Vec3::X).unwrap(),
        radius: 3.0,
    });
    let h = curve_surface(&c, (0.0, TAU), &cylinder(5.0));
    assert_eq!(h.ts.len(), 2);
    for t in h.ts {
        assert!(signed_distance(&cylinder(5.0), c.eval(t)).abs() < 1e-12);
    }
}

#[test]
fn coplanar_circles_and_lines_cross() {
    let a = Curve::Circle(Circle { frame: Frame::WORLD, radius: 5.0 });
    let b = Curve::Circle(Circle {
        frame: Frame::new(Vec3::new(6.0, 0.0, 0.0), Vec3::Z, Vec3::X).unwrap(),
        radius: 3.0,
    });
    let pts = curve_curve(&a, (0.0, TAU), &b, (0.0, TAU));
    assert_eq!(pts.len(), 2);
    let l = line([0.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    assert_eq!(curve_curve(&a, (0.0, TAU), &l, (0.0, 20.0)).len(), 2);
    let tangent = line([5.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    assert_eq!(curve_curve(&a, (0.0, TAU), &tangent, (0.0, 20.0)).len(), 1);
}

#[test]
fn rays_hit_quadrics() {
    let hits = ray_surface(&cylinder(5.0), Vec3::ZERO, Vec3::X, 100.0);
    assert_eq!(hits.len(), 2);
    assert!(hits.iter().any(|&t| (t - 5.0).abs() < 1e-12));
}

/// A cubic through `per_turn` points a turn of the helix of radius `r` rising 1 a turn about
/// z, from turn `a` to turn `b`, every half turn among them; mirrored in y = 0 when `mirror`.
fn helix(r: f64, per_turn: usize, (a, b): (f64, f64), mirror: bool) -> Curve {
    let n = ((b - a) * per_turn as f64).round() as usize;
    let pts: Vec<Vec3> = (0..=n)
        .map(|i| {
            let turn = a + (b - a) * i as f64 / n as f64;
            let th = TAU * turn;
            Vec3::new(r * th.cos(), r * th.sin(), turn)
        })
        .collect();
    let s = crate::geom::BSpline::interpolate(&pts).unwrap();
    if !mirror {
        return Curve::BSpline(s);
    }
    let flip: Vec<Vec3> = s.control_points().iter().map(|p| Vec3::new(p.x, -p.y, p.z)).collect();
    Curve::BSpline(crate::geom::BSpline::new(s.degree(), s.knots().to_vec(), flip).unwrap())
}

#[test]
fn a_helix_crosses_its_mirror_image_every_half_turn() {
    // A right-hand helix of radius 5 rising 1 a turn, from a quarter turn before 0 to a
    // quarter past 3, and its mirror in y = 0, a left-hand one: a point of both lies in the
    // mirror plane, where the right-hand one passes at each half turn k/2, at (±5, 0, k/2).
    // Each is a node of the fit, so the splines pass there exactly: seven crossings.
    let a = helix(5.0, 360, (-0.25, 3.25), false);
    let b = helix(5.0, 360, (-0.25, 3.25), true);
    let (ra, rb) = (a.domain(), b.domain());
    let mut got = curve_curve(&a, ra, &b, rb);
    got.sort_by(|p, q| p.z.total_cmp(&q.z));
    assert_eq!(got.len(), 7, "{got:?}");
    for (k, p) in got.iter().enumerate() {
        let want = Vec3::new(if k % 2 == 0 { 5.0 } else { -5.0 }, 0.0, 0.5 * k as f64);
        assert!(p.dist(want) <= 1e-9, "{k}: {p:?}");
    }
}

#[test]
fn pruning_by_hulls_keeps_every_meeting_of_spline_curves() {
    // The search sampled everywhere and the search sampled only near the other curve's hulls
    // find the same points, bit for bit: a helix crossing its mirror image (every half turn),
    // a helix and a coaxial one 0.15 wider (never), a helix and a spline along the line
    // x = 5, y = 0 (each whole turn, at (5, 0, k)), and a helix and its copy raised by half
    // the tolerance (running together, no point).
    let h = helix(5.0, 40, (-0.25, 2.25), false);
    let up = crate::math::Transform::translation(Vec3::Z * (0.5 * tol::LINEAR));
    let post = Curve::BSpline(
        crate::geom::BSpline::interpolate(&[
            Vec3::new(5.0, 0.0, -0.6),
            Vec3::new(5.0, 0.0, 0.7),
            Vec3::new(5.0, 0.0, 1.4),
            Vec3::new(5.0, 0.0, 2.6),
        ])
        .unwrap(),
    );
    let cases = [
        (h.clone(), helix(5.0, 40, (-0.25, 2.25), true), 5),
        (h.clone(), helix(5.15, 40, (-0.25, 2.25), false), 0),
        (h.clone(), post, 3),
        (h.clone(), h.transformed(&up), 0),
    ];
    for (i, (a, b, n)) in cases.iter().enumerate() {
        let (ra, rb) = (a.domain(), b.domain());
        let full = sampled_curve_curve(a, ra, b, rb, false);
        let pruned = sampled_curve_curve(a, ra, b, rb, true);
        assert_eq!(pruned, full, "case {i}");
        assert_eq!(pruned.len(), *n, "case {i}: {pruned:?}");
    }
}

#[test]
fn within_agrees_with_the_distance() {
    // Points at the helix's node (5, 0, 1) and moved out from the axis by 0.5, 1.5 and 3
    // times the tolerance, and one far off on the axis: within the tolerance exactly when
    // the projection says so.
    let h = helix(5.0, 40, (-0.25, 2.25), false);
    let r = h.domain();
    for (out, want) in [(0.0, true), (0.5, true), (1.5, false), (3.0, false)] {
        let p = Vec3::new(5.0 + out * tol::LINEAR, 0.0, 1.0);
        assert_eq!(within(&h, r, p, tol::LINEAR), want, "{out}");
        assert_eq!(curve_distance(&h, r, p).0 <= tol::LINEAR, want, "{out}");
    }
    assert!(!within(&h, r, Vec3::new(0.0, 0.0, 1.0), tol::LINEAR));
}
