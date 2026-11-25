// yack: measure's tests that build their operands with springrep modules above yack-measure.
// Adapted from springrep@a03e8a1 src/measure (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of the crate's tests in M3 (W4-W7); paths and imports only. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
/// Moved from measure.rs's tests (M3 W5): they build with modules above yack-measure.
#[cfg(test)]
mod measure_moved {
    use yack_math::kernel::Aabb;
    use yack_math::kernel::Frame;
    use yack_math::kernel::Point3;
    use yack_math::kernel::Vec2;
    use yack_math::kernel::Vec3;
    use yack_measure::curvature;
    use yack_measure::normal_at;
    use yack_profile::NamedSegment;
    use yack_profile::Profile;
    use yack_profile::ProfileLoop;
    use yack_profile::Segment;
    use yack_build::sweep::extrude;
    use yack_topo::bbox::face_box;

    #[test]
    fn a_face_moved_keeps_its_box_moved() {
        // An assembly measures a part against another moved before each boolean: a slid part's
        // faces keep their boxes, slid, rather than finding each spline edge's extremes again.
        let rod = extrude(
            &Profile::circle(Vec2::ZERO, 20.0),
            &Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap(),
            20.0,
            "r",
        )
        .unwrap();
        let boxes: Vec<Aabb> = rod.face_ids().map(|f| face_box(&rod, f)).collect();
        let shift = Vec3::new(3.5, -120.0, 7.25);
        let mut slid = rod.clone();
        slid.transform(&yack_math::kernel::Transform::translation(shift));
        for (f, before) in slid.face_ids().zip(boxes) {
            let kept = slid.memo(f).loose_box.get().copied();
            assert_eq!(
                kept.map(|b| (b.min, b.max)),
                Some((before.min + shift, before.max + shift))
            );
        }
        // Turned, a box is found again: the turned box's own would be looser.
        let mut turned = rod.clone();
        turned.transform(&yack_math::kernel::Transform::rotation(Vec3::ZERO, Vec3::X, 0.3));
        assert!(
            turned
                .face_ids()
                .all(|f| turned.memo(f).loose_box.get().is_none())
        );
    }

    #[test]
    fn a_rods_end_just_inside_its_rim_is_flat() {
        // A rod of radius 10: its end's outline, drawn as chords, cuts up to 0.012 inside the
        // rim, and a point of the end between a chord and the rim is still a point of the end.
        let rod = extrude(
            &Profile::circle(Vec2::ZERO, 20.0),
            &Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap(),
            20.0,
            "r",
        )
        .unwrap();
        for k in 0..256 {
            let a = std::f64::consts::TAU * k as f64 / 256.0;
            let p = Point3::new(9.999 * a.cos(), 9.999 * a.sin(), 20.0);
            let n = normal_at(&rod, p, 1e-6).unwrap_or_else(|| panic!("no normal at {p:?}"));
            assert!((n - Vec3::Z).norm() < 1e-12, "{n:?} at {p:?}");
            let bend = curvature(&rod, p).unwrap_or_else(|| panic!("no curvature at {p:?}"));
            assert!(
                bend[0].abs() < 1e-9 && bend[1].abs() < 1e-9,
                "{bend:?} at {p:?}"
            );
        }
    }

    #[test]
    fn a_drilled_rods_side_bends_away_its_hole_toward_and_its_rim_is_a_crease() {
        // A rod of radius 10, 20 long, drilled through along its axis at radius 4.
        let circle = |radius: f64, ccw: bool, name: &str| ProfileLoop {
            segments: vec![NamedSegment::new(
                Segment::Circle {
                    center: Vec2::ZERO,
                    radius,
                    ccw,
                },
                name,
            )],
        };
        let p = Profile {
            loops: vec![circle(10.0, true, "out"), circle(4.0, false, "in")],
        };
        let rod = extrude(
            &p,
            &Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap(),
            20.0,
            "r",
        )
        .unwrap();
        let near =
            |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9;
        let side = curvature(&rod, Point3::new(11.0, 1.0, 7.0)).unwrap();
        assert!(near(side, [0.1, 0.0]), "{side:?}");
        let hole = curvature(&rod, Point3::new(0.5, 3.0, 13.0)).unwrap();
        assert!(near(hole, [0.0, -0.25]), "{hole:?}");
        let end = curvature(&rod, Point3::new(6.0, 2.0, 20.5)).unwrap();
        assert!(near(end, [0.0, 0.0]), "{end:?}");
        assert!(curvature(&rod, Point3::new(10.5, 0.0, 20.5)).is_none());
    }
}

/// Moved from measure/sample.rs's tests (M3 W5): they build with modules above yack-measure.
#[cfg(test)]
mod measure_sample_moved {
    use yack_boolean::difference;
    use yack_math::kernel::Frame;
    use yack_math::kernel::Vec2;
    use yack_math::kernel::Vec3;
    use yack_measure::edge_length;
    use yack_measure::extent;
    use yack_measure::face_extent;
    use yack_measure::face_samples;
    use yack_measure::face_uv_bounds;
    use yack_measure::solid_area;
    use yack_measure::solid_bbox;
    use yack_profile::Profile;
    use yack_build::sweep::extrude;
    use yack_topo::Brep;
    use yack_topo::FaceId;
    use yack_topo::SolidId;
    use std::f64::consts::PI;

    fn named(b: &Brep, name: &str) -> FaceId {
        let has = |f: &FaceId| b.face(*f).names.iter().any(|n| n.to_string() == name);
        b.face_ids().find(has).unwrap()
    }

    #[test]
    fn a_block_s_edges_and_a_rod_s_rims_have_their_lengths() {
        // A 20 × 10 × 5 block has four edges of each length.
        let block = extrude(
            &Profile::rect(Vec2::ZERO, 20.0, 10.0),
            &Frame::WORLD,
            5.0,
            "b",
        );
        let block = block.unwrap();
        let total: f64 = block.edge_ids().map(|e| edge_length(&block, e)).sum();
        assert!((total - 4.0 * 35.0).abs() < 1e-9, "{total}");
        // A Ø10 rod 7 long: two rims of 10π and the seam.
        let rod = extrude(&Profile::circle(Vec2::ZERO, 10.0), &Frame::WORLD, 7.0, "r").unwrap();
        let mut lengths: Vec<f64> = rod.edge_ids().map(|e| edge_length(&rod, e)).collect();
        lengths.sort_by(f64::total_cmp);
        assert_eq!(lengths.len(), 3);
        assert!((lengths[0] - 7.0).abs() < 1e-9, "{lengths:?}");
        assert!((lengths[1] - 10.0 * PI).abs() < 1e-9 && (lengths[2] - 10.0 * PI).abs() < 1e-9);
    }

    #[test]
    fn samples_lie_on_the_face_with_its_outward_normal_and_miss_a_hole() {
        // The top of a 20 × 20 × 5 block with a Ø10 hole through its middle: a 5 × 5 grid has
        // points 4 mm apart from −8 to 8, and the nine within 5 mm of the middle fall in the hole
        // but for the four at (±4, ±4), which are 5.66 mm out.
        let block = extrude(
            &Profile::rect(Vec2::ZERO, 20.0, 20.0),
            &Frame::WORLD,
            5.0,
            "b",
        );
        let below = Frame::WORLD.translated(Vec3::Z * -1.0);
        let hole = extrude(&Profile::circle(Vec2::ZERO, 10.0), &below, 7.0, "h").unwrap();
        let drilled = difference(&block.unwrap(), &hole).unwrap();
        let samples = face_samples(&drilled, named(&drilled, "b.end"), 5);
        assert_eq!(samples.len(), 25 - 5);
        for (p, n) in &samples {
            assert!((p.z - 5.0).abs() < 1e-12 && (*n - Vec3::Z).norm() < 1e-12);
            assert!(p.x.hypot(p.y) > 5.0);
        }
        // The hole's wall faces into the hole.
        for (p, n) in face_samples(&drilled, named(&drilled, "h.side"), 4) {
            assert!((p.x.hypot(p.y) - 5.0).abs() < 1e-9);
            assert!((n.x * p.x + n.y * p.y + 5.0).abs() < 1e-9, "{n:?} at {p:?}");
        }
    }

    #[test]
    fn a_rod_reaches_its_radius_across_and_its_length_along_whatever_the_direction() {
        // A Ø10 rod 7 long on the z axis: ±5 along any direction square to it, 0 to 7 along it,
        // and along the diagonal of x and z the rim's far point, (5 + 7) / √2.
        let rod = extrude(&Profile::circle(Vec2::ZERO, 10.0), &Frame::WORLD, 7.0, "r").unwrap();
        let near = |got: (f64, f64), want: (f64, f64)| {
            assert!(
                (got.0 - want.0).abs() < 1e-9 && (got.1 - want.1).abs() < 1e-9,
                "{got:?}"
            );
        };
        near(extent(&rod, Vec3::new(3.0, 4.0, 0.0)), (-5.0, 5.0));
        near(extent(&rod, Vec3::Z), (0.0, 7.0));
        let slant = 1.0 / 2f64.sqrt();
        near(
            extent(&rod, Vec3::new(1.0, 0.0, 1.0)),
            (-5.0 * slant, 12.0 * slant),
        );
        // Its wall alone runs the rod's length, and its parameters a whole turn and that length.
        let wall = named(&rod, "r.side");
        near(face_extent(&rod, wall, Vec3::Z), (0.0, 7.0));
        let (lo, hi) = face_uv_bounds(&rod, wall);
        assert!(
            (hi.x - lo.x - std::f64::consts::TAU).abs() < 1e-9,
            "{lo:?} {hi:?}"
        );
        assert!((hi.y - lo.y - 7.0).abs() < 1e-9, "{lo:?} {hi:?}");
    }

    #[test]
    fn a_ball_reaches_its_radius_past_any_edge_it_has() {
        // A ball of radius 3 centred 1 above the origin: its one face bulges to 4 and −2 along z
        // and ±3 along a slant through its centre, where no edge of it reaches.
        let centre = Frame::WORLD.translated(Vec3::Z);
        let ball = yack_build::primitive::sphere(&centre, 3.0, "b").unwrap();
        let (lo, hi) = extent(&ball, Vec3::Z);
        assert!(
            (lo + 2.0).abs() < 1e-9 && (hi - 4.0).abs() < 1e-9,
            "{lo} {hi}"
        );
        let (lo, hi) = extent(&ball, Vec3::new(1.0, 1.0, 0.0));
        assert!(
            (lo + 3.0).abs() < 1e-9 && (hi - 3.0).abs() < 1e-9,
            "{lo} {hi}"
        );
    }

    #[test]
    fn each_solid_of_an_arena_has_its_own_box_and_area() {
        // A 2 × 2 × 2 block at the origin and a 4 × 4 × 1 one 10 along x, in one arena.
        let small = extrude(
            &Profile::rect(Vec2::ZERO, 2.0, 2.0),
            &Frame::WORLD,
            2.0,
            "a",
        );
        let far = Frame::WORLD.translated(Vec3::X * 10.0);
        let wide = extrude(&Profile::rect(Vec2::ZERO, 4.0, 4.0), &far, 1.0, "b").unwrap();
        let mut both = small.unwrap();
        both.append(&wide);
        let solids: Vec<SolidId> = both.solid_ids().collect();
        assert_eq!(solids.len(), 2);
        let (first, second) = (solid_bbox(&both, solids[0]), solid_bbox(&both, solids[1]));
        assert!((first.max.x - 1.0).abs() < 1e-12 && (first.max.z - 2.0).abs() < 1e-12);
        assert!((second.min.x - 8.0).abs() < 1e-12 && (second.max.z - 1.0).abs() < 1e-12);
        assert!((solid_area(&both, solids[0]) - 24.0).abs() < 1e-9);
        assert!((solid_area(&both, solids[1]) - 48.0).abs() < 1e-9);
    }
}

/// Moved from measure/tolerant.rs's tests (M3 W5): they build with modules above yack-measure.
#[cfg(test)]
mod measure_tolerant_moved {
    use yack_math::kernel::Frame;
    use yack_math::kernel::Vec3;
    use yack_measure::volume_agrees;
    use yack_build::primitive::block;

    #[test]
    fn a_cube_and_a_thin_slab_agree_with_their_volumes_and_not_with_others() {
        let at = Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap();
        let cube = block(&at, Vec3::new(10.0, 10.0, 10.0), "c").unwrap();
        assert!(volume_agrees(&cube, 1000.0, 1e-5));
        assert!(!volume_agrees(&cube, 1000.1, 1e-5));
        // 100 × 100 × 0.001: a volume of 10 in a box 141 across, where a size³-relative
        // tolerance would be blind to it.
        let slab = block(&at, Vec3::new(100.0, 100.0, 0.001), "s").unwrap();
        assert!(volume_agrees(&slab, 10.0, 1e-5));
        assert!(!volume_agrees(&slab, 10.001, 1e-5));
        // On the boundary: decided exactly.
        assert!(volume_agrees(&slab, 10.0 * (1.0 + 0.95e-5), 1e-5));
        assert!(!volume_agrees(&slab, 10.0 * (1.0 + 1.05e-5), 1e-5));
        assert!(!volume_agrees(&slab, f64::NAN, 1e-5));
    }
}
