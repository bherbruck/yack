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
