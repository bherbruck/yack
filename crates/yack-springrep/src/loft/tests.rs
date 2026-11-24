// yack: springrep module src/loft/tests.rs
// Adapted from springrep@a03e8a1 src/loft/tests.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: none (verbatim); this header added.
use std::f64::consts::PI;

use super::*;
use crate::{measure, profile::PointsInterp};

fn at_z(z: f64) -> Frame {
    Frame::WORLD.translated(Vec3::Z * z)
}

fn sec(profile: Profile, frame: Frame) -> Section {
    Section { profile, frame }
}

fn volume(b: &Brep) -> f64 {
    measure::volume(b)
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

#[test]
fn ruled_rectangles_make_a_planar_frustum() {
    let b = loft(
        &[
            sec(Profile::rect(Vec2::ZERO, 20.0, 10.0), at_z(0.0)),
            sec(Profile::rect(Vec2::ZERO, 10.0, 6.0), at_z(4.0)),
        ],
        true,
        "lf",
    )
    .unwrap();
    assert_eq!(b.face_count(), 6);
    assert!(b.face_ids().all(|f| b.face(f).surface.kind() == "plane"));
    // Prismoid: h/6 (A0 + 4 Am + A1).
    let want = 4.0 / 6.0 * (200.0 + 4.0 * 15.0 * 8.0 + 60.0);
    assert!(rel(volume(&b), want) < 1e-12, "{}", volume(&b));
    let names = b.name_counts();
    assert_eq!(names["lf.side"], 4);
    assert_eq!(names["lf.start"], 1);
    assert_eq!(names["lf.end"], 1);
}

fn named_area(b: &Brep, name: &str) -> Vec<f64> {
    b.face_ids()
        .filter(|&f| b.face(f).names.iter().any(|n| n.to_string() == name))
        .map(|f| measure::face_area(b, f))
        .collect()
}

#[test]
fn a_side_carries_the_name_of_the_first_sections_edge() {
    let b = loft(
        &[
            sec(Profile::rect(Vec2::ZERO, 20.0, 10.0), at_z(0.0)),
            sec(Profile::rect(Vec2::ZERO, 10.0, 6.0), at_z(4.0)),
        ],
        true,
        "lf",
    )
    .unwrap();
    // Each side is a trapezoid: the two edges it joins, and a slant of the 4 mm rise and half the
    // change in the other dimension. The long sides lean 2 mm, the short ones 5 mm.
    let long = 0.5 * (20.0 + 10.0) * 20f64.sqrt();
    let short = 0.5 * (10.0 + 6.0) * 41f64.sqrt();
    for (name, want) in [
        ("lf.v_min", long),
        ("lf.v_max", long),
        ("lf.u_min", short),
        ("lf.u_max", short),
    ] {
        let got = named_area(&b, name);
        assert_eq!(got.len(), 1, "{name}");
        assert!(rel(got[0], want) < 1e-12, "{name}: {}", got[0]);
    }
}

#[test]
fn a_split_edge_gives_its_name_to_each_part() {
    let b = loft(
        &[
            sec(Profile::rect(Vec2::ZERO, 20.0, 20.0), at_z(0.0)),
            sec(Profile::circle(Vec2::ZERO, 20.0), at_z(30.0)),
        ],
        false,
        "lf",
    )
    .unwrap();
    // The circle starts on +x, which splits the square's +x edge at its middle.
    let names = b.name_counts();
    assert_eq!(names["lf.side"], 5);
    assert_eq!(names["lf.u_max"], 2);
    for whole in ["lf.u_min", "lf.v_min", "lf.v_max"] {
        assert_eq!(names[whole], 1, "{whole}");
    }
    let halves = named_area(&b, "lf.u_max");
    assert!(rel(halves[0], halves[1]) < 1e-9, "{halves:?}");
}

#[test]
fn parallel_sections_out_of_order_are_refused() {
    let circle = |r| Profile::circle(Vec2::ZERO, r);
    for zs in [[0.0, 6.0, 3.0], [0.0, 3.0, 3.0]] {
        let sections = zs.map(|z| sec(circle(10.0), at_z(z)));
        assert!(loft(&sections, false, "lf").is_err(), "{zs:?}");
    }
    let falling = [6.0, 3.0, 0.0].map(|z| sec(circle(10.0), at_z(z)));
    assert!(loft(&falling, false, "lf").is_ok());
}

#[test]
fn circles_loft_to_a_frustum() {
    let b = loft(
        &[
            sec(Profile::circle(Vec2::ZERO, 20.0), at_z(0.0)),
            sec(Profile::circle(Vec2::ZERO, 10.0), at_z(6.0)),
        ],
        false,
        "lf",
    )
    .unwrap();
    assert_eq!(b.face_count(), 3);
    let want = PI * 6.0 / 3.0 * (100.0 + 50.0 + 25.0);
    assert!(rel(volume(&b), want) < 1e-9, "{}", volume(&b));
}

#[test]
fn circle_to_square_splits_at_every_vertex_angle() {
    let b = loft(
        &[
            sec(Profile::circle(Vec2::ZERO, 20.0), at_z(0.0)),
            sec(Profile::rect(Vec2::ZERO, 20.0, 20.0), at_z(30.0)),
        ],
        false,
        "lf",
    )
    .unwrap();
    // The circle's start and the square's four corners: five sides. The circle's one edge is
    // `side`, so the sides take no second name.
    assert_eq!(b.face_count(), 7);
    let names: Vec<_> = b.name_counts().into_iter().collect();
    let want = [("lf.end", 1), ("lf.side", 5), ("lf.start", 1)].map(|(n, c)| (n.to_string(), c));
    assert_eq!(names, want);
    assert!(rel(volume(&b), 10667.102709516572) < 0.02, "{}", volume(&b));
    let bb = measure::bbox(&b);
    assert!((bb.min.x + 10.0).abs() < 1e-6 && (bb.max.z - 30.0).abs() < 1e-6);
}

/// An ellipse turned steadily along z: every z section is the same outline, so the volume is the
/// straight prism's to within the skinning's fit.
#[test]
fn twisted_sections_skin_smoothly() {
    let pts: Vec<Vec2> = (0..12)
        .map(|i| {
            let a = TAU * i as f64 / 12.0;
            Vec2::new(5.0 * a.cos() + 2.0, 3.0 * a.sin())
        })
        .collect();
    let p = Profile::points(&pts, PointsInterp::PeriodicSpline, "side").unwrap();
    let prism = volume(&crate::sweep::extrude(&p, &at_z(0.0), 10.0, "x").unwrap());
    let sections: Vec<Section> = (0..=4)
        .map(|i| {
            let z = 10.0 * i as f64 / 4.0;
            let th = 0.06 * z;
            let x = Vec3::new(th.cos(), th.sin(), 0.0);
            sec(p.clone(), Frame::new(Vec3::Z * z, Vec3::Z, x).unwrap())
        })
        .collect();
    let b = loft(&sections, false, "tw").unwrap();
    assert_eq!(b.face_count(), 3);
    assert!(rel(volume(&b), prism) < 1e-6, "{} {prism}", volume(&b));
}

/// A helical gear's tooth space, 1.4 mm across at its root, lofted through 9 sections from
/// z = -0.5 over `length`, turned `twist` rad/mm.
fn tooth_space(twist: f64, length: f64) -> Brep {
    let flank = |i: usize, s: f64| {
        let t = i as f64 / 12.0;
        let (r, a) = (21.5 - 4.5 * t, s * (0.09 - 0.05 * t * t));
        Vec2::new(r * a.cos(), r * a.sin())
    };
    let mut pts: Vec<Vec2> = (0..=12).map(|i| flank(i, -1.0)).collect();
    pts.extend((1..12).map(|i| {
        let a = -0.04 + 0.08 * i as f64 / 12.0;
        let r = 17.0 - 0.3 * (PI * i as f64 / 12.0).sin();
        Vec2::new(r * a.cos(), r * a.sin())
    }));
    pts.extend((0..=12).rev().map(|i| flank(i, 1.0)));
    let corner = 30f64.to_radians();
    let p = Profile::points(&pts, PointsInterp::Smooth { corner }, "space").unwrap();
    let sections: Vec<Section> = (0..=8)
        .map(|i| {
            let z = -0.5 + length * i as f64 / 8.0;
            let x = Vec3::new((twist * z).cos(), (twist * z).sin(), 0.0);
            sec(p.clone(), Frame::new(Vec3::Z * z, Vec3::Z, x).unwrap())
        })
        .collect();
    loft(&sections, false, "g").unwrap()
}

/// Points `off` from each face, on an `n`×`m` grid of its parameters, project back `off` away.
fn assert_projects_nearest(b: &Brep, off: f64, n: usize, m: usize) {
    for f in b.face_ids() {
        let s = &b.face(f).surface;
        for i in 0..=n {
            for j in 0..=m {
                let (u, v) = (i as f64 / n as f64, j as f64 / m as f64);
                let q = s.eval(u, v) + s.normal(u, v) * off;
                let uv = s.project(q);
                let d = s.eval(uv.x, uv.y).dist(q);
                assert!(d <= off + 1e-9, "face {f:?} at ({u}, {v}): {d}");
            }
        }
    }
}

#[test]
fn projection_finds_the_nearest_point_on_a_narrow_twisted_side() {
    // The root winds 1.4 rad round the axis, so seeds spaced along its length alone sit in other
    // points' basins.
    assert_projects_nearest(&tooth_space(0.2, 7.0), 0.01, 12, 12);
}

// A tooth space's wedge, 8.5 to 11.5 mm from the axis and 0.1 to 0.3 rad across, lofted through 9
// sections over 15.5 mm and turned 5 rad about the axis, as a helical gear's is: each section moves
// 6 mm, twice its width, from the one before. Matched by where their vertices lie in space, the
// sections paired each vertex with the next one round, and the skin came out inside out. Its
// volume is the wedge's area times the length, to the Bézier skin's departure from the helix
// between sections.
#[test]
fn sections_turned_further_than_their_width_keep_their_edges_matched() {
    let polar = [
        (8.5, 0.05),
        (8.5, -0.05),
        (9.5, -0.09),
        (10.5, -0.12),
        (11.5, -0.15),
        (11.5, 0.15),
        (10.5, 0.12),
        (9.5, 0.09),
    ];
    let pts: Vec<Vec2> = polar
        .iter()
        .map(|&(r, a): &(f64, f64)| Vec2::new(r * a.cos(), r * a.sin()))
        .collect();
    let area = 0.5 * (0..8).map(|i| pts[i].cross(pts[(i + 1) % 8])).sum::<f64>();
    let names = ["a", "b", "c", "d", "e", "f", "g", "h"];
    let p = Profile::polygon(&pts, &names);
    let sections: Vec<Section> = (0..=8)
        .map(|i| {
            let z = 15.5 * i as f64 / 8.0;
            let th = 5.0 * z / 15.5;
            let x = Vec3::new(th.cos(), th.sin(), 0.0);
            sec(p.clone(), Frame::new(Vec3::Z * z, Vec3::Z, x).unwrap())
        })
        .collect();
    let b = loft(&sections, false, "g").unwrap();
    assert_eq!(b.face_count(), 10);
    let want = area * 15.5;
    assert!(rel(volume(&b), want) < 1e-4, "{}", rel(volume(&b), want));
}

// The same 20 x 10 rectangle in space, the second drawn 10 x 20 on a plane whose x is world y, or
// on one facing down the loft: matched in each section's own frame, each corner paired with the
// next one round and the 2000 mm³ prism came out a twisted 1333.33.
#[test]
fn sections_on_differently_turned_planes_match_where_they_lie() {
    for z in [Vec3::Z, -Vec3::Z] {
        for ruled in [true, false] {
            let b = loft(
                &[
                    sec(Profile::rect(Vec2::ZERO, 20.0, 10.0), at_z(0.0)),
                    sec(
                        Profile::rect(Vec2::ZERO, 10.0, 20.0),
                        Frame::new(Vec3::Z * 10.0, z, Vec3::Y).unwrap(),
                    ),
                ],
                ruled,
                "lf",
            )
            .unwrap();
            assert_eq!(b.face_count(), 6);
            assert!(
                rel(volume(&b), 2000.0) < 1e-12,
                "{z:?} {ruled} {}",
                volume(&b)
            );
        }
    }
}

#[test]
fn projection_finds_the_nearest_point_on_a_side_wound_past_a_turn() {
    // One span along the sections wound 6.6 rad: the root is about 110 mm long, so seeds a few
    // millimetres apart along it miss its basin.
    assert_projects_nearest(&tooth_space(0.6, 11.0), 1e-3, 4, 24);
}

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

/// Moved from measure.rs's tests (M3 W5): they build with modules above yack-measure.
#[cfg(test)]
mod measure_moved {
    use crate::math::Aabb;
    use crate::math::Frame;
    use crate::math::Point3;
    use crate::math::Vec2;
    use crate::math::Vec3;
    use crate::measure::curvature;
    use crate::measure::normal_at;
    use crate::profile::NamedSegment;
    use crate::profile::Profile;
    use crate::profile::ProfileLoop;
    use crate::profile::Segment;
    use crate::sweep::extrude;
    use crate::topo::bbox::face_box;

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
        slid.transform(&crate::math::Transform::translation(shift));
        for (f, before) in slid.face_ids().zip(boxes) {
            let kept = slid.memo(f).loose_box.get().copied();
            assert_eq!(
                kept.map(|b| (b.min, b.max)),
                Some((before.min + shift, before.max + shift))
            );
        }
        // Turned, a box is found again: the turned box's own would be looser.
        let mut turned = rod.clone();
        turned.transform(&crate::math::Transform::rotation(Vec3::ZERO, Vec3::X, 0.3));
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
    use crate::boolean::difference;
    use crate::math::Frame;
    use crate::math::Vec2;
    use crate::math::Vec3;
    use crate::measure::edge_length;
    use crate::measure::extent;
    use crate::measure::face_extent;
    use crate::measure::face_samples;
    use crate::measure::face_uv_bounds;
    use crate::measure::solid_area;
    use crate::measure::solid_bbox;
    use crate::profile::Profile;
    use crate::sweep::extrude;
    use crate::topo::Brep;
    use crate::topo::FaceId;
    use crate::topo::SolidId;
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
        let ball = crate::primitive::sphere(&centre, 3.0, "b").unwrap();
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
    use crate::math::Frame;
    use crate::math::Vec3;
    use crate::measure::volume_agrees;
    use crate::primitive::block;

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

/// Moved from rays.rs's tests (M3 W5): they build with modules above yack-locate.
#[cfg(test)]
mod rays_moved {
    use crate::boolean;
    use crate::math::Frame;
    use crate::math::Point3;
    use crate::math::Vec2;
    use crate::math::Vec3;
    use crate::profile::NamedSegment;
    use crate::profile::Profile;
    use crate::profile::ProfileLoop;
    use crate::profile::Segment;
    use crate::rays::Side;
    use crate::sweep::Helix;
    use crate::sweep::extrude;
    use crate::sweep::helix;
    use std::f64::consts::PI;
    use std::f64::consts::TAU;

    /// A cone under a non-uniform scale and shear (conformance stretches-369ef83ce9d5f331): every
    /// point of a grid about it is placed as its preimage lies in the cone, worked out exactly.
    /// Its narrow end passes near its wide rim, where four seeds of the global projection once all
    /// lay, and a ray's exact crossing of the side there was thrown out as off the face.
    #[test]
    fn a_sheared_cone_places_points_as_their_preimages_lie() {
        let o = Vec3::new(-0.1369187722614813, -0.1927317277546228, 9.496120663976027);
        let x = Vec3::new(0.9862312463263244, 0.0011868763560915257, 0.16536783270678776);
        let z = Vec3::new(0.1653720918704991, -0.007078186740360178, -0.9862058459078631);
        let (r0, r1, h) = (3.753950727109955, 0.9541105874096755, 14.435434108066273);
        let cone = crate::primitive::cone(&Frame::new(o, z, x).unwrap(), r0, r1, h, "c").unwrap();
        let m = [
            [-0.4538971246554957, 0.15252576458329087, 0.9555666727243001, 41.6656262880843],
            [-0.26697402297825473, -0.5673293657946105, -1.4703977293152846, 16.072466163157856],
            [0.05877765464764511, -1.3990214079245642, 0.7004527186325231, -15.994874937232893],
        ];
        let b = crate::facade::edit::affine(&cone, m).unwrap();
        let rays = crate::mesh::rays(&b).unwrap();
        let map = |p: Vec3| {
            let row = |r: [f64; 4]| r[0] * p.x + r[1] * p.y + r[2] * p.z + r[3];
            Vec3::new(row(m[0]), row(m[1]), row(m[2]))
        };
        let y = z.cross(x);
        // How far inside the cone a preimage lies: along the axis within [0, h], within the radius.
        let margin = |p: Vec3| {
            let q = p - o;
            let (a, b, c) = (q.dot(x), q.dot(y), q.dot(z));
            (r0 + (r1 - r0) * c / h - (a * a + b * b).sqrt()).min(c).min(h - c)
        };
        // The point that was placed outside, its preimage 0.099 inside.
        assert_eq!(rays.side(Vec3::new(36.53894920997076, 22.626132672803138, -19.18988598536717)), Side::Inside);
        let n = 12;
        let f = |t: usize| (t as f64 + 0.5) / n as f64;
        let mut wrong = Vec::new();
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    let p = o + x * ((f(i) - 0.5) * 2.4 * r0) + y * ((f(j) - 0.5) * 2.4 * r0) + z * ((f(k) * 1.2 - 0.1) * h);
                    let want = if margin(p) > 0.0 { Side::Inside } else { Side::Outside };
                    let got = rays.side(map(p));
                    if got != want && margin(p).abs() > 1e-6 { // tol: a preimage on the surface may go either way
                        wrong.push((map(p), margin(p), got));
                    }
                }
            }
        }
        assert!(wrong.is_empty(), "{} of {} placed wrong: {:?}", wrong.len(), n * n * n, &wrong[..wrong.len().min(5)]);
    }

    #[test]
    fn a_ray_up_a_threaded_bore_meets_its_wall() {
        // A hex nut 7.6 tall, bored 4 in radius, a 60° thread ridge of pitch 1.2 joined inside it
        // for three turns from z = 2: the bore's wall left between the turns is a strip that winds
        // three times round, and a ray from inside the bore must cross it or a flank to leave.
        let seg = |segment, name: &str| NamedSegment::new(segment, name);
        let d: f64 = 6.8;
        let rc = 2.0 * d / 3f64.sqrt();
        let hex: Vec<Vec2> = (0..6)
            .map(|i| {
                let a = PI / 3.0 * i as f64;
                Vec2::new(rc * a.cos(), rc * a.sin())
            })
            .collect();
        let mut nut_profile = Profile::polygon(&hex, &["a", "b", "c", "d", "e", "f"]);
        nut_profile.loops.push(ProfileLoop {
            segments: vec![seg(
                Segment::Circle {
                    center: Vec2::ZERO,
                    radius: 4.0,
                    ccw: false,
                },
                "bore",
            )],
        });
        let nut = extrude(&nut_profile, &Frame::WORLD, 7.6, "nut").unwrap();
        let (pitch, major) = (1.2, 4.0);
        let minor = major - 0.6134 * pitch;
        let root = major + 0.1 * pitch;
        let half = |x: f64| (x - minor) * (PI / 6.0).tan();
        let tri = [
            Vec2::new(root, 2.0 - half(root)),
            Vec2::new(root, 2.0 + half(root)),
            Vec2::new(minor, 2.0),
        ];
        let ridge_profile = Profile {
            loops: vec![ProfileLoop {
                segments: (0..3)
                    .map(|i| {
                        seg(
                            Segment::Line {
                                a: tri[i],
                                b: tri[(i + 1) % 3],
                            },
                            "s",
                        )
                    })
                    .collect(),
            }],
        };
        let motion = Helix {
            origin: Point3::ZERO,
            axis: Vec3::Z,
            rise: pitch / TAU,
            sign: 1.0,
        };
        let meridian = Frame::new(Point3::ZERO, -Vec3::Y, Vec3::X).unwrap();
        let ridge = helix(&ridge_profile, &meridian, &motion, 3.0 * TAU, "hel").unwrap();
        let threaded = boolean::union(&nut, &ridge).unwrap();
        let rays = crate::mesh::rays(&threaded).unwrap();
        let a = 72.7f64.to_radians();
        let p = Point3::new(2.031 * a.cos(), 2.031 * a.sin(), 4.655);
        let dir = Vec3::new(0.3713, 0.5347, 0.7591);
        let hits = rays.cast(p, dir, (0.0, 1e6)).unwrap();
        let first = hits
            .first()
            .expect("the ray leaves the bore through something");
        assert!(
            first.normal.dot(dir) < 0.0,
            "the bore's first crossing enters the nut"
        );
    }
}

/// Moved from codec/tests.rs (M3 W5): they encode built solids (sweep, boolean, blend), above yack-codec; they reach the codec through to_bytes/from_bytes.
#[cfg(test)]
mod codec_moved {
    use crate::blend::fillet;
    use crate::boolean::difference;
    use crate::math::Frame;
    use crate::math::Point3;
    use crate::math::Vec2;
    use crate::math::Vec3;
    use crate::measure;
    use crate::profile::Profile;
    use crate::sweep::extrude;
    use crate::topo::Brep;

    /// The solids' bytes through the codec's public path (yack-codec keeps `Codec` crate-private).
    fn bytes<T: Solids>(x: &T) -> Vec<u8> {
        crate::codec::to_bytes(&x.solids())
    }

    fn back<T: Solids>(b: &[u8]) -> Option<T> {
        crate::codec::from_bytes(b).and_then(T::from_solids)
    }

    /// A solid, or a list of solids, as `to_bytes`/`from_bytes` carry them.
    trait Solids: Sized {
        fn solids(&self) -> Vec<Brep>;
        fn from_solids(v: Vec<Brep>) -> Option<Self>;
    }

    impl Solids for Brep {
        fn solids(&self) -> Vec<Brep> {
            vec![self.clone()]
        }
        fn from_solids(mut v: Vec<Brep>) -> Option<Brep> {
            (v.len() == 1).then(|| v.remove(0))
        }
    }

    impl Solids for Vec<Brep> {
        fn solids(&self) -> Vec<Brep> {
            self.clone()
        }
        fn from_solids(v: Vec<Brep>) -> Option<Vec<Brep>> {
            Some(v)
        }
    }

    /// `x` read back from its bytes equals it, and writes the same bytes again: every number kept to
    /// its last bit, signed zeros included.
    fn round_trips<T: Solids + PartialEq + std::fmt::Debug>(x: &T) {
        let b = bytes(x);
        let y: T = back(&b).expect("reads back");
        assert_eq!(&y, x);
        assert_eq!(bytes(&y), b);
    }

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Vec3::new(x, y, z)
    }

    /// A 20 × 20 × 10 block with a Ø6 hole through it and its top's two edges along y rounded R1.
    fn drilled_and_rounded() -> Brep {
        let block = extrude(
            &Profile::rect(Vec2::ZERO, 20.0, 20.0),
            &at_z(0.0),
            10.0,
            "block",
        )
        .unwrap();
        let hole = extrude(&Profile::circle(Vec2::ZERO, 6.0), &at_z(0.0), 10.0, "hole").unwrap();
        let drilled = difference(&block, &hole).unwrap();
        let edges = [("block.end", "block.u_min"), ("block.end", "block.u_max")];
        fillet(&drilled, &edges, 1.0, "round").unwrap()
    }

    fn at_z(z: f64) -> Frame {
        Frame::new(p(0.0, 0.0, z), p(0.0, 0.0, 1.0), p(1.0, 0.0, 0.0)).unwrap()
    }

    #[test]
    fn a_built_solid_reads_back_with_its_ids_and_names() {
        let b = drilled_and_rounded();
        let volume = measure::volume(&b);
        let back: Brep = back(&bytes(&b)).unwrap();
        assert_eq!(back, b);
        assert_eq!(bytes(&back), bytes(&b));
        // Measured again from its read-back geometry, it has the same volume to the last bit.
        assert_eq!(measure::volume(&back).to_bits(), volume.to_bits());
    }

    #[test]
    fn the_bytes_do_not_depend_on_what_was_asked_of_the_solid() {
        // Queries fill a face's memo (its volume share, its box, the trims a boolean reads); none of
        // that is written, so a solid encodes the same before and after them.
        let b = drilled_and_rounded();
        let before = bytes(&b);
        measure::volume(&b);
        measure::area(&b);
        let rays = crate::mesh::rays(&b).unwrap();
        rays.side(p(5.0, 5.0, 5.0));
        let square = Profile::rect(Vec2::new(10.0, 10.0), 4.0, 4.0);
        let post = extrude(&square, &at_z(-5.0), 20.0, "post").unwrap();
        difference(&b, &post).unwrap();
        assert_eq!(bytes(&b), before);
        let pair = vec![b.clone(), b];
        let back: Vec<Brep> = back(&bytes(&pair)).unwrap();
        assert_eq!(bytes(&back), bytes(&pair));
    }

    #[test]
    fn local_tolerances_read_back() {
        let mut b = drilled_and_rounded();
        let v = b.vertex_ids().next().unwrap();
        let e = b.edge_ids().last().unwrap();
        b.set_vertex_tolerance(v, 2e-4);
        b.set_edge_tolerance(e, 1e-4);
        round_trips(&b);
    }
}
