// yack: tests of surface–surface intersection in closed form.
// Adapted from springrep@a03e8a1 src/boolean/ssi.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of ssi.rs into its own file (M3 hardening); unchanged otherwise.

use super::*;
use crate::geom::{Cone, Cylinder, Plane, Sphere, Torus};

/// A closed cubic in the plane z = 0, as a profile for swept surfaces.
fn wavy() -> Curve {
    let n = 8;
    let mut ctrl: Vec<Vec3> = (0..n)
        .map(|k| {
            let a = TAU * k as f64 / n as f64;
            let r = if k % 2 == 0 { 4.0 } else { 6.0 };
            Vec3::new(10.0 + r * a.cos(), r * a.sin(), 0.0)
        })
        .collect();
    ctrl.extend_from_within(..3);
    let knots = (0..n + 7).map(|i| i as f64 - 3.0).collect();
    Curve::BSpline(crate::geom::BSpline::periodic_cubic(knots, ctrl).unwrap())
}

#[test]
fn swept_surfaces_drawn_from_other_starts_are_the_same_surface() {
    use crate::geom::{Extrusion, Revolution};
    use crate::math::Transform;
    let profile = wavy();
    let extruded = |curve: Curve, dir: Vec3| Surface::Extrusion(Box::new(Extrusion { curve, dir }));
    let along = Transform::translation(Vec3::new(0.0, 0.0, 3.0));
    let aside = Transform::translation(Vec3::new(0.01, 0.0, 0.0));
    let a = extruded(profile.clone(), Vec3::Z);
    // The profile slid along the extrusion, and the extrusion run the other way: one surface.
    assert!(same_domain(&a, &extruded(profile.transformed(&along), Vec3::Z)));
    assert!(same_domain(&a, &extruded(profile.clone(), -Vec3::Z)));
    assert!(!same_domain(&a, &extruded(profile.transformed(&aside), Vec3::Z)));

    // The profile in the plane y = 0, turned about z: turned a way round first, one surface.
    let upright = profile.transformed(&Transform::rotation(Vec3::ZERO, Vec3::X, TAU / 4.0));
    let revolved = |curve: Curve| {
        Surface::Revolution(Box::new(Revolution { curve, origin: Vec3::ZERO, axis: Vec3::Z }))
    };
    let turned = Transform::rotation(Vec3::ZERO, Vec3::Z, 1.0);
    let b = revolved(upright.clone());
    assert!(same_domain(&b, &revolved(upright.transformed(&turned))));
    assert!(!same_domain(&b, &revolved(upright.transformed(&aside))));
    assert!(!same_domain(&a, &b));
}

fn region() -> Aabb {
    Aabb { min: Vec3::new(-50.0, -50.0, -50.0), max: Vec3::new(50.0, 50.0, 50.0) }
}

fn plane(o: [f64; 3], n: [f64; 3]) -> Surface {
    let n = Vec3::from_array(n);
    Surface::Plane(Plane { frame: Frame::from_axis(Vec3::from_array(o), n).unwrap() })
}

fn cyl(o: [f64; 3], z: [f64; 3], r: f64) -> Surface {
    Surface::Cylinder(Cylinder {
        frame: Frame::from_axis(Vec3::from_array(o), Vec3::from_array(z)).unwrap(),
        radius: r,
    })
}

fn curves(a: &Surface, b: &Surface) -> (Vec<SsiCurve>, Vec<Point3>) {
    match intersect(a, b, &region(), &mut Vec::new).unwrap() {
        Ssi::Curves { curves, points } => (curves, points),
        Ssi::Coincident => panic!("coincident"),
    }
}

/// Every curve lies on both surfaces.
fn on_both(a: &Surface, b: &Surface, cs: &[SsiCurve]) {
    for c in cs {
        for i in 0..=16 {
            let t = c.range.0 + (c.range.1 - c.range.0) * i as f64 / 16.0;
            let p = c.curve.eval(t);
            for s in [a, b] {
                let d = super::super::csi::signed_distance(s, p).abs();
                assert!(d < 1e-9, "{} off {} by {d}", c.curve.kind(), s.kind());
            }
        }
    }
}

#[test]
fn plane_pairs_have_closed_forms() {
    let p = plane([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]);
    let q = plane([0.0, 2.0, 0.0], [0.0, 1.0, 1.0]);
    let (cs, _) = curves(&p, &q);
    assert_eq!(cs.len(), 1);
    on_both(&p, &q, &cs);
    let c_perp = cyl([1.0, 1.0, -5.0], [0.0, 0.0, 1.0], 3.0);
    let (cs, _) = curves(&p, &c_perp);
    assert!(matches!(cs[0].curve, Curve::Circle(_)));
    on_both(&p, &c_perp, &cs);
    let c_oblique = cyl([0.0, 0.0, 0.0], [0.3, 0.2, 1.0], 3.0);
    let (cs, _) = curves(&p, &c_oblique);
    assert!(matches!(cs[0].curve, Curve::Ellipse(_)));
    on_both(&p, &c_oblique, &cs);
    let c_along = cyl([0.0, 0.0, 3.0], [1.0, 0.0, 0.0], 2.5);
    let (cs, _) = curves(&p, &c_along);
    assert_eq!(cs.len(), 2);
    on_both(&p, &c_along, &cs);
    let sphere = Surface::Sphere(Sphere {
        frame: Frame::new(Vec3::new(0.0, 0.0, 4.0), Vec3::Z, Vec3::X).unwrap(),
        radius: 5.0,
    });
    let (cs, _) = curves(&p, &sphere);
    assert!(matches!(&cs[0].curve, Curve::Circle(c) if (c.radius - 4.0).abs() < 1e-12));
    let cone = Surface::Cone(Cone { frame: Frame::WORLD, radius: 2.0, half_angle: 0.3 });
    let (cs, _) = curves(&p, &cone);
    assert_eq!(cs.len(), 1);
    on_both(&p, &cone, &cs);
    let torus = Surface::Torus(Torus { frame: Frame::WORLD, major: 6.0, minor: 2.0 });
    let (cs, _) = curves(&p, &torus);
    assert_eq!(cs.len(), 2);
    on_both(&p, &torus, &cs);
}

#[test]
fn tangent_plane_and_cylinder_touch_along_a_line() {
    let p = plane([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let c = cyl([5.0, 20.0, 0.0], [0.0, 0.0, 1.0], 5.0);
    let (cs, _) = curves(&p, &c);
    assert_eq!(cs.len(), 1);
    assert!(cs[0].tangent);
    let past = cyl([4.999999, 20.0, 0.0], [0.0, 0.0, 1.0], 5.0);
    let (cs, _) = curves(&p, &past);
    assert_eq!(cs.len(), 2);
    assert!(cs.iter().all(|c| !c.tangent));
    let short = cyl([5.000001, 20.0, 0.0], [0.0, 0.0, 1.0], 5.0);
    assert!(curves(&p, &short).0.is_empty());
}

#[test]
fn coaxial_and_parallel_pairs_meet_in_circles_and_lines() {
    let c = cyl([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 3.0);
    let sphere = Surface::Sphere(Sphere { frame: Frame::WORLD, radius: 5.0 });
    let (cs, _) = curves(&c, &sphere);
    assert_eq!(cs.len(), 2);
    on_both(&c, &sphere, &cs);
    let cone = Surface::Cone(Cone { frame: Frame::WORLD, radius: 1.0, half_angle: 0.4 });
    let (cs, _) = curves(&c, &cone);
    assert_eq!(cs.len(), 1);
    on_both(&c, &cone, &cs);
    let d = cyl([4.0, 0.0, 0.0], [0.0, 0.0, 1.0], 2.0);
    let (cs, _) = curves(&c, &d);
    assert_eq!(cs.len(), 2);
    on_both(&c, &d, &cs);
    let same = cyl([0.0, 0.0, 7.0], [0.0, 0.0, -1.0], 3.0);
    assert!(matches!(intersect(&c, &same, &region(), &mut Vec::new).unwrap(), Ssi::Coincident));
}

#[test]
fn equal_crossing_cylinders_meet_in_two_ellipses_that_touch() {
    let a = cyl([-20.0, 0.0, 0.0], [1.0, 0.0, 0.0], 5.0);
    let b = cyl([0.0, -20.0, 0.0], [0.0, 1.0, 0.0], 5.0);
    let (cs, pts) = curves(&a, &b);
    assert_eq!(cs.len(), 2);
    on_both(&a, &b, &cs);
    assert_eq!(pts.len(), 2);
    for p in pts {
        assert!((p.z.abs() - 5.0).abs() < 1e-12 && p.x.abs() < 1e-12);
    }
}

fn spline_extrusion() -> Surface {
    let pts: Vec<Vec3> = (0..12)
        .map(|i| {
            let t = TAU * i as f64 / 12.0;
            let r = if i % 2 == 0 { 10.0 } else { 9.0 };
            Vec3::new(r * t.cos(), r * t.sin(), 0.0)
        })
        .collect();
    Surface::Extrusion(Box::new(crate::geom::Extrusion {
        curve: Curve::BSpline(crate::geom::BSpline::interpolate_periodic(&pts).unwrap()),
        dir: Vec3::Z,
    }))
}

#[test]
fn extrusions_meet_planes_and_parallel_cylinders_in_closed_form() {
    let e = spline_extrusion();
    let across = plane([0.0, 0.0, 3.0], [0.0, 0.0, 1.0]);
    let (cs, _) = curves(&e, &across);
    assert_eq!(cs.len(), 1);
    assert!(cs[0].closed && matches!(cs[0].curve, Curve::BSpline(_)));
    on_both(&e, &across, &cs);
    let along = plane([2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let (cs, _) = curves(&e, &along);
    assert_eq!(cs.len(), 2);
    assert!(cs.iter().all(|c| matches!(c.curve, Curve::Line(_))));
    on_both(&e, &along, &cs);
    let rod = cyl([9.5, 0.0, 0.0], [0.0, 0.0, 1.0], 1.0);
    let (cs, _) = curves(&rod, &e);
    assert_eq!(cs.len(), 2);
    on_both(&rod, &e, &cs);
}

#[test]
fn revolutions_meet_coaxial_surfaces_in_circles() {
    let profile = Curve::BSpline(
        crate::geom::BSpline::interpolate(&[
            Vec3::new(4.0, 0.0, 0.0),
            Vec3::new(6.0, 0.0, 2.0),
            Vec3::new(5.0, 0.0, 4.0),
            Vec3::new(7.0, 0.0, 6.0),
        ])
        .unwrap(),
    );
    let r = Surface::Revolution(Box::new(crate::geom::Revolution {
        curve: profile,
        origin: Vec3::ZERO,
        axis: Vec3::Z,
    }));
    let across = plane([0.0, 0.0, 3.0], [0.0, 0.0, 1.0]);
    let (cs, _) = curves(&r, &across);
    assert_eq!(cs.len(), 1);
    on_both(&r, &across, &cs);
    let c = cyl([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 5.5);
    let (cs, _) = curves(&c, &r);
    assert!(cs.len() >= 2);
    assert!(cs.iter().all(|c| matches!(c.curve, Curve::Circle(_))));
    on_both(&c, &r, &cs);
}

#[test]
fn spheres_meet_in_a_circle_or_touch_at_a_point() {
    let a = Surface::Sphere(Sphere { frame: Frame::WORLD, radius: 5.0 });
    let b = Surface::Sphere(Sphere {
        frame: Frame::new(Vec3::new(4.0, 3.0, 0.0), Vec3::Z, Vec3::X).unwrap(),
        radius: 4.0,
    });
    let (cs, _) = curves(&a, &b);
    assert_eq!(cs.len(), 1);
    on_both(&a, &b, &cs);
    let touching = Surface::Sphere(Sphere {
        frame: Frame::new(Vec3::new(8.0, 0.0, 0.0), Vec3::Z, Vec3::X).unwrap(),
        radius: 3.0,
    });
    let (cs, pts) = curves(&a, &touching);
    assert!(cs.is_empty());
    assert!(pts.len() == 1 && pts[0].dist(Vec3::new(5.0, 0.0, 0.0)) < 1e-12);
    let tangent_plane = plane([0.0, 0.0, 5.0], [0.0, 0.0, 1.0]);
    let (cs, pts) = curves(&a, &tangent_plane);
    assert!(cs.is_empty());
    assert_eq!(pts.len(), 1);
}

#[test]
fn crossing_cylinders_of_unequal_radii_are_traced() {
    let a = cyl([-20.0, 0.0, 0.0], [1.0, 0.0, 0.0], 5.0);
    let b = cyl([0.0, 0.0, -20.0], [0.0, 0.0, 1.0], 3.0);
    let seeds = vec![Vec3::new(0.0, 3.0, 4.0), Vec3::new(0.0, 3.0, -4.0)];
    let region = Aabb { min: Vec3::new(-4.0, -6.0, -6.0), max: Vec3::new(4.0, 6.0, 6.0) };
    let Ssi::Curves { curves, .. } = intersect(&a, &b, &region, &mut || seeds.clone()).unwrap()
    else {
        panic!("coincident");
    };
    assert_eq!(curves.len(), 2);
    assert!(curves.iter().all(|c| c.closed));
    for c in &curves {
        for i in 0..=32 {
            let p = c.curve.eval(c.range.0 + (c.range.1 - c.range.0) * i as f64 / 32.0);
            for s in [&a, &b] {
                assert!(super::super::csi::signed_distance(s, p).abs() <= tol::LINEAR);
            }
        }
    }
}
