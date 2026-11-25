// yack: intersect's tests that build their operands with springrep modules above yack-intersect.
// Adapted from springrep@a03e8a1 src/intersect (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of the crate's tests in M3 (W4-W7); paths and imports only.
/// Moved from intersect/march.rs's tests (M3 W4): they trace across lofts, and yack-intersect sits below loft; they reach the trace through ssi::intersect.
#[cfg(test)]
mod intersect_march {
    use crate::geom::Curve;
    use crate::geom::Plane;
    use crate::geom::Surface;
    use crate::geom::distance_to;
    use crate::loft::Section;
    use crate::loft::loft;
    use crate::math::Aabb;
    use crate::math::Frame;
    use crate::math::Point3;
    use crate::math::Vec2;
    use crate::math::Vec3;
    use crate::profile::PointsInterp;
    use crate::profile::Profile;
    use crate::tol;

    /// `march::trace` through the public path: `ssi::intersect` traces a free-form pair from the seeds.
    macro_rules! ssi_trace {
        ($a:expr, $b:expr, $region:expr, $seeds:expr) => {
            match crate::intersect::ssi::intersect($a, $b, $region, &mut || $seeds.to_vec()) {
                Ok(crate::intersect::ssi::Ssi::Curves { curves, points }) => Ok::<_, crate::Error>((curves, points)),
                Ok(_) => panic!("a free-form pair has a closed form"),
                Err(e) => Err(e),
            }
        };
    }

    /// A gear's tooth space: two flanks from r 21.5 down to a round root at r 17.
    fn tooth_space() -> Vec<Vec2> {
        let flank = |i: usize, s: f64| {
            let t = i as f64 / 12.0;
            let (r, a) = (21.5 - 4.5 * t, s * (0.09 - 0.05 * t * t));
            Vec2::new(r * a.cos(), r * a.sin())
        };
        let mut pts: Vec<Vec2> = (0..=12).map(|i| flank(i, -1.0)).collect();
        pts.extend((1..12).map(|i| {
            let a = -0.04 + 0.08 * i as f64 / 12.0;
            let r = 17.0 - 0.3 * (std::f64::consts::PI * i as f64 / 12.0).sin();
            Vec2::new(r * a.cos(), r * a.sin())
        }));
        pts.extend((0..=12).rev().map(|i| flank(i, 1.0)));
        pts
    }

    #[test]
    fn branches_that_stall_in_one_shared_band_end_on_one_crossing() {
        let at_z = |z: f64| Frame::WORLD.translated(Vec3::Z * z);
        let lofted = loft(
            &[
                Section {
                    profile: Profile::circle(Vec2::ZERO, 20.0),
                    frame: at_z(0.0),
                },
                Section {
                    profile: Profile::rect(Vec2::ZERO, 20.0, 20.0),
                    frame: at_z(30.0),
                },
            ],
            false,
            "lf",
        )
        .unwrap();
        let side = lofted
            .face_ids()
            .map(|f| &lofted.face(f).surface)
            .filter(|s| !matches!(s, Surface::Plane(_)))
            .min_by(|s, t| {
                let p = Vec3::new(0.0, -10.0, 15.0);
                distance_to(s, p).total_cmp(&distance_to(t, p))
            })
            .unwrap();
        let moved = side.transformed(&crate::math::Transform::translation(Vec3::X * 1e-5));
        let region = Aabb {
            min: Vec3::new(-12.0, -11.0, -3.0),
            max: Vec3::new(12.0, -5.0, 33.0),
        };
        let seeds = [
            Vec3::new(0.0, -10.0, 15.0),
            Vec3::new(-5.0, -10.0, 30.0),
            Vec3::new(5.0, -10.0, 30.0),
        ];
        let (curves, crossings) = ssi_trace!(side, &moved, &region, &seeds).unwrap();
        assert_eq!(crossings.len(), 1, "{crossings:?}");
        let ends: Vec<Point3> = curves
            .iter()
            .flat_map(|c| [c.curve.eval(c.range.0), c.curve.eval(c.range.1)])
            .filter(|p| p.z > 29.0 && p.x.abs() < 1e-3)
            .collect();
        assert_eq!(ends.len(), 3, "{ends:?}");
        for p in ends {
            assert!(p.dist(crossings[0]) <= tol::LINEAR, "{p:?} {crossings:?}");
        }
    }

    #[test]
    fn two_crossings_nearer_than_a_step_are_both_traced() {
        let (r, d) = (5.0, 10.0 - 3e-4);
        let sections: Vec<Section> = [0.0, 5.0, 10.0, 15.0]
            .iter()
            .map(|&z| Section {
                profile: Profile::circle(Vec2::ZERO, 2.0 * r),
                frame: Frame::WORLD.translated(Vec3::Z * z),
            })
            .collect();
        let can = loft(&sections, false, "can").unwrap();
        let side = can
            .face_ids()
            .map(|f| &can.face(f).surface)
            .find(|s| matches!(s, Surface::Nurbs(_)))
            .unwrap();
        let moved = side.transformed(&crate::math::Transform::translation(Vec3::X * d));
        let y = (r * r - 0.25 * d * d).sqrt();
        let region = Aabb {
            min: Vec3::new(3.0, -2.0, 1.0),
            max: Vec3::new(7.0, 2.0, 14.0),
        };
        let seeds = [Vec3::new(0.5 * d, y, 7.5), Vec3::new(0.5 * d, -y, 7.5)];
        let (curves, _) = ssi_trace!(side, &moved, &region, &seeds).unwrap();
        assert_eq!(curves.len(), 2);
        for c in &curves {
            for k in 0..=4 {
                let p = c
                    .curve
                    .eval(c.range.0 + (c.range.1 - c.range.0) * k as f64 / 4.0);
                assert!((p.x - 0.5 * d).abs() <= 1e-6, "{p:?}");
                assert!((p.y.abs() - y).abs() <= 1e-6, "{p:?}");
            }
        }
        let ys: Vec<f64> = curves.iter().map(|c| c.curve.eval(c.range.0).y).collect();
        assert!(ys[0] * ys[1] < 0.0, "{ys:?}");
    }

    #[test]
    fn a_plane_across_a_steeply_twisted_loft_never_shuttles_between_points() {
        let twist = 0.3;
        let outline = Profile::points(
            &tooth_space(),
            PointsInterp::Smooth {
                corner: 30f64.to_radians(),
            },
            "space",
        )
        .unwrap();
        let sections: Vec<Section> = (0..=8)
            .map(|i| {
                let z = -0.5 + 11.0 * i as f64 / 8.0;
                let x = Vec3::X * (twist * z).cos() + Vec3::Y * (twist * z).sin();
                Section {
                    profile: outline.clone(),
                    frame: Frame::new(Vec3::Z * z, Vec3::Z, x).unwrap(),
                }
            })
            .collect();
        let tool = loft(&sections, false, "g").unwrap();
        let plane = Surface::Plane(Plane {
            frame: Frame::WORLD,
        });
        let region = Aabb {
            min: Vec3::new(-19.16, -5.37, -0.107),
            max: Vec3::new(19.16, 19.16, 0.107),
        };
        let seeds = [Vec3::new(16.98647297164612, -0.6798377134004823, 0.0)];
        // From this seed on the second side face, refinement used to land each step behind the
        // last, and the walk went back and forth between two points to its 20 000-step cap.
        let side = &tool.face(tool.face_ids().nth(1).unwrap()).surface;
        for c in ssi_trace!(&plane, side, &region, &seeds).unwrap().0 {
            let Curve::Intersection(fit) = &c.curve else {
                continue;
            };
            let n = fit.approx.control_points().len();
            assert!(n < 2000, "a branch of {n} points");
        }
    }

    #[test]
    fn a_crossing_no_curve_reaches_is_dropped_rather_than_failing_the_fit() {
        let at_z = |z: f64| Frame::WORLD.translated(Vec3::Z * z);
        let section = |profile, z| Section {
            profile,
            frame: at_z(z),
        };
        let a = loft(
            &[
                section(Profile::circle(Vec2::ZERO, 20.0), 0.0),
                section(Profile::rect(Vec2::ZERO, 20.0, 20.0), 30.0),
            ],
            false,
            "lf",
        )
        .unwrap();
        let mut b = a.clone();
        b.transform(&crate::math::Transform::translation(Vec3::new(
            3e-9, 0.0, 0.0,
        )));
        // The sides either side of the ruling through (10, 0, z) touch along it. Traced from the
        // loft's corners, the walk stalls 0.4 mm up and 0.045 mm across from a touching point.
        let side =
            |x: &crate::topo::Brep, i: usize| x.face(x.face_ids().nth(i).unwrap()).surface.clone();
        let (sa, sb) = (side(&a, 0), side(&b, 4));
        let region = Aabb {
            min: Vec3::new(-11.0, -11.0, -1.0),
            max: Vec3::new(11.0, 11.0, 31.0),
        };
        let seeds: Vec<Point3> = a.vertex_ids().map(|v| a.vertex(v).point).collect();
        let (curves, _) = ssi_trace!(&sa, &sb, &region, &seeds).unwrap();
        assert!(!curves.is_empty());
    }
}
