// yack: tweak's tests that build their operands with springrep modules above yack-tweak.
// Adapted from springrep@a03e8a1 src/tweak (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of the crate's tests in M3 (W4-W7); paths and imports only. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
/// Moved from tweak/tests.rs (M3 W7): they build with edit, sweep and primitive, above yack-tweak.
#[cfg(test)]
mod tweak_moved {
    use yack_base::kernel::Result;
    use yack_check as check;
    use yack_edit::draft;
    use yack_edit::offset_faces;
    use yack_math::kernel::Frame;
    use yack_math::kernel::Vec2;
    use yack_math::kernel::Vec3;
    use yack_measure as measure;
    use yack_profile::Profile;
    use yack_build::sweep;
    use yack_topo::Brep;
    use yack_topo::FaceId;
    use yack_topo::FaceName;
    use std::collections::BTreeMap;
    use std::f64::consts::TAU;

    fn face(b: &Brep, name: &str) -> FaceId {
        let n = FaceName::new("box", name);
        b.face_ids()
            .find(|&f| b.face(f).names.contains(&n))
            .unwrap()
    }

    #[test]
    fn a_spline_walled_prism_offset_inward_holds_steiners_area() {
        // A closed spline through 24 points of an ellipse 12 × 8, extruded 10, every face moved 1 mm
        // in. The spline's side has no exact offset of its own kind; its curve's offset is fitted and
        // extruded. By Steiner's formula an inward offset t of a smooth convex outline (t below its
        // least radius of curvature, here 8²/12 > 5) has area A − P·t + π·t², so the moved prism holds
        // (A − P + π)·8, A and P the outline's area and perimeter, read from the prism itself.
        let pts: Vec<Vec2> = (0..24)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 24.0;
                Vec2::new(12.0 * a.cos(), 8.0 * a.sin())
            })
            .collect();
        let p = Profile::points(&pts, yack_profile::PointsInterp::PeriodicSpline, "wall").unwrap();
        let b = sweep::extrude(&p, &Frame::WORLD, 10.0, "can").unwrap();
        let area = measure::volume(&b) / 10.0;
        let perimeter = (measure::area(&b) - 2.0 * area) / 10.0;
        let by: BTreeMap<FaceId, f64> = b.face_ids().map(|f| (f, -1.0)).collect();
        let moved = offset_faces(&b, &by).unwrap();
        assert!(check::check(&moved).is_empty());
        let want = (area - perimeter + std::f64::consts::PI) * 8.0;
        let got = measure::volume(&moved);
        assert!((got - want).abs() < 1e-8 * want, "{got} for {want}");
    }

    /// A box 4 wide in x, 6 deep and 4 tall, its walls in `walls` drafted about their feet by 45°:
    /// the walls lean in 1 per 1 of height.
    fn drafted_box(walls: &[&str]) -> Result<Brep> {
        let p = Profile::rect(Vec2::ZERO, 4.0, 6.0);
        let b = sweep::extrude(&p, &Frame::WORLD, 4.0, "box").unwrap();
        let faces: Vec<FaceId> = walls.iter().map(|w| face(&b, w)).collect();
        draft(&b, &faces, &Frame::WORLD, std::f64::consts::FRAC_PI_4)
    }

    #[test]
    fn walls_drafted_until_they_meet_close_over_the_top() {
        let out = drafted_box(&["u_min", "u_max"]).unwrap();
        assert!(check::check(&out).is_empty());
        assert!((measure::volume(&out) - 24.0).abs() < 1e-9);
        assert_eq!(out.face_count(), 5);
    }

    #[test]
    fn all_walls_drafted_until_two_meet_make_a_hipped_roof() {
        let out = drafted_box(&["u_min", "u_max", "v_min", "v_max"]).unwrap();
        assert!(check::check(&out).is_empty());
        assert!((measure::volume(&out) - 56.0 / 3.0).abs() < 1e-9);
        assert_eq!(out.face_count(), 5);
    }

    #[test]
    fn a_step_raised_level_with_the_top_closes_the_riser() {
        let pts = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 2.0),
            (4.0, 2.0),
            (4.0, 6.0),
            (0.0, 6.0),
        ]
        .map(|(x, y)| Vec2::new(x, y));
        let names = ["e0", "e1", "e2", "e3", "e4", "e5"];
        let b = sweep::extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, 3.0, "box").unwrap();
        let out = offset_faces(&b, &BTreeMap::from([(face(&b, "e2"), 4.0)])).unwrap();
        assert!(check::check(&out).is_empty());
        assert!((measure::volume(&out) - 180.0).abs() < 1e-9);
        assert_eq!(out.face_count(), 6);
    }

    #[test]
    fn a_wall_moved_across_its_slot_closes_it() {
        let pts = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 6.0),
            (7.0, 6.0),
            (7.0, 2.0),
            (3.0, 2.0),
            (3.0, 6.0),
            (0.0, 6.0),
        ]
        .map(|(x, y)| Vec2::new(x, y));
        let names = ["e0", "e1", "e2", "e3", "e4", "e5", "e6", "e7"];
        let b = sweep::extrude(&Profile::polygon(&pts, &names), &Frame::WORLD, 3.0, "box").unwrap();
        let out = offset_faces(&b, &BTreeMap::from([(face(&b, "e5"), 4.0)])).unwrap();
        assert!(check::check(&out).is_empty());
        assert!((measure::volume(&out) - 180.0).abs() < 1e-9);
        assert_eq!(out.face_count(), 6);
    }

    /// A ball of radius `r` about `f.origin`, its axis `f.z`, revolved as a sketch's half disc is.
    fn ball(f: &Frame, r: f64) -> Brep {
        use yack_profile::NamedSegment;
        use yack_profile::ProfileLoop;
        use yack_profile::Segment;
        let seg = |segment, name: &str| NamedSegment::new(segment, name);
        let skin = Segment::Arc {
            center: Vec2::ZERO,
            radius: r,
            start: -std::f64::consts::FRAC_PI_2,
            sweep: std::f64::consts::PI,
        };
        let axis = Segment::Line {
            a: Vec2::new(0.0, r),
            b: Vec2::new(0.0, -r),
        };
        let half = Profile {
            loops: vec![ProfileLoop {
                segments: vec![seg(skin, "skin"), seg(axis, "axis")],
            }],
        };
        let meridian = Frame::new(f.origin, -f.y, f.x).unwrap();
        sweep::revolve(&half, &meridian, f.origin, f.z, TAU, "sph").unwrap()
    }

    /// Frames turned every which way, as moved copies are.
    fn turned_frames() -> Vec<Frame> {
        (0..200)
            .map(|k| {
                let a = k as f64 * 0.731;
                let z = Vec3::new(
                    a.sin() * (1.7 * a).cos(),
                    a.sin() * (1.7 * a).sin(),
                    a.cos(),
                );
                let x = Vec3::new((2.3 * a).cos(), (2.3 * a).sin(), 0.4);
                Frame::new(Vec3::new(1.5, -2.0, 3.0), z, x).unwrap()
            })
            .collect()
    }

    #[test]
    fn a_ball_offset_keeps_its_seam_apart_in_any_frame() {
        let (r, d): (f64, f64) = (5.358245536894633, 0.8586557324583324);
        let want = 4.0 / 3.0 * std::f64::consts::PI * (r + d).powi(3);
        for (k, f) in turned_frames().iter().enumerate() {
            let b = ball(f, r);
            let by: BTreeMap<FaceId, f64> = b.face_ids().map(|f| (f, d)).collect();
            let grown = offset_faces(&b, &by).unwrap_or_else(|e| panic!("frame {k}: {e}"));
            let got = measure::volume(&grown);
            assert!(
                (got - want).abs() < 1e-6 * want,
                "frame {k}: {got} against {want}"
            );
        }
    }
}
