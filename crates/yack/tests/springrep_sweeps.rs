// yack: Swept solids against closed-form volume, area and box, each checked valid and watertight.
// Adapted from springrep@a03e8a1 tests/sweeps.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
//! Swept solids against closed-form volume, area and box, each checked valid and watertight.
// W11: springrep's own kernel tests, moved here when springrep went (ruling (b)). They build with the
// kernel crates directly and compute their expected values, so the facade's maths ban and the
// workspace lints' test-only exemptions (which reach only #[test] fns) do not fit them.
// TODO(api-owner, hardening, D26): rewrite these tests to the lints, test by test, and drop this allow.
#![allow(clippy::disallowed_methods, clippy::disallowed_types, clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)]

use std::f64::consts::PI;

use yack_base::kernel::Error;
use yack_check as check;
use yack_math::kernel::Frame;
use yack_math::kernel::Vec2;
use yack_math::kernel::Vec3;
use yack_measure as measure;
use yack_mesh as mesh;
use yack_mesh::MeshOptions;
use yack_profile::NamedSegment;
use yack_profile::PointsInterp;
use yack_profile::Profile;
use yack_profile::ProfileLoop;
use yack_profile::Segment;
use yack_build::sweep::extrude;
use yack_build::sweep::revolve;
use yack_topo::Brep;

const COARSE: MeshOptions = MeshOptions {
    linear: 0.05,
    angular: 0.3,
};

fn assert_solid(b: &Brep, volume: f64, area: f64, lo: [f64; 3], hi: [f64; 3], rel: f64) {
    let issues = check::check(b);
    assert!(issues.is_empty(), "{issues:?}");
    let v = measure::volume(b);
    let a = measure::area(b);
    assert!((v - volume).abs() <= rel * volume, "volume {v} vs {volume}");
    assert!((a - area).abs() <= rel * area, "area {a} vs {area}");
    let bb = measure::bbox(b);
    let d = (bb.min - Vec3::from_array(lo))
        .max_abs()
        .max((bb.max - Vec3::from_array(hi)).max_abs());
    assert!(d < 1e-6, "bbox {bb:?}");
    let m = mesh::tessellate(b, &COARSE).unwrap();
    for s in b.solid_ids() {
        let r = mesh::report(b, &m, s);
        assert!(r.watertight(), "{r:?}");
        assert!(
            (r.volume - volume).abs() < 0.02 * volume,
            "mesh volume {}",
            r.volume
        );
    }
}

fn xz_frame() -> Frame {
    Frame::new(Vec3::ZERO, Vec3::new(0.0, -1.0, 0.0), Vec3::X).unwrap()
}

#[test]
fn box_extrusion_is_exact_and_named() {
    let b = extrude(
        &Profile::rect(Vec2::new(5.0, 2.0), 10.0, 4.0),
        &Frame::WORLD,
        3.0,
        "blk",
    )
    .unwrap();
    assert_solid(
        &b,
        120.0,
        2.0 * (40.0 + 30.0 + 12.0),
        [0.0, 0.0, 0.0],
        [10.0, 4.0, 3.0],
        1e-12,
    );
    let names = b.name_counts();
    for n in [
        "blk.start",
        "blk.end",
        "blk.v_min",
        "blk.u_max",
        "blk.v_max",
        "blk.u_min",
    ] {
        assert_eq!(names.get(n), Some(&1), "{n}");
    }
}

#[test]
fn plate_with_a_hole_either_way_along_its_normal() {
    let hole = Profile::circle(Vec2::new(2.0, 0.0), 2.0).loops.remove(0);
    let p = Profile::rect(Vec2::ZERO, 10.0, 6.0).with_holes([hole]);
    let frame = Frame::new(Vec3::ZERO, Vec3::new(0.0, 0.0, -1.0), Vec3::X).unwrap();
    let b = extrude(&p, &frame, 2.0, "p").unwrap();
    let v = 2.0 * (60.0 - PI);
    let a = 2.0 * (60.0 - PI) + 2.0 * 32.0 + 2.0 * PI * 2.0;
    assert_solid(&b, v, a, [-5.0, -3.0, -2.0], [5.0, 3.0, 0.0], 1e-12);
    let negative = extrude(&p, &Frame::WORLD, -2.0, "p").unwrap();
    assert_solid(&negative, v, a, [-5.0, -3.0, -2.0], [5.0, 3.0, 0.0], 1e-12);
    assert_eq!(negative.name_counts().get("p.start"), Some(&1));
}

#[test]
fn arcs_in_a_profile_sweep_cylinders_both_ways() {
    // A 10 × 10 square with a half-disc bump on the right and a half-disc bite on the left.
    let seg = |segment: Segment, name: &str| NamedSegment::new(segment, name);
    let lp = ProfileLoop {
        segments: vec![
            seg(
                Segment::Line {
                    a: Vec2::new(0.0, 0.0),
                    b: Vec2::new(10.0, 0.0),
                },
                "bottom",
            ),
            seg(
                Segment::Arc {
                    center: Vec2::new(10.0, 5.0),
                    radius: 5.0,
                    start: -PI / 2.0,
                    sweep: PI,
                },
                "bump",
            ),
            seg(
                Segment::Line {
                    a: Vec2::new(10.0, 10.0),
                    b: Vec2::new(0.0, 10.0),
                },
                "top",
            ),
            seg(
                Segment::Arc {
                    center: Vec2::new(0.0, 5.0),
                    radius: 3.0,
                    start: PI / 2.0,
                    sweep: -PI,
                },
                "bite_a",
            ),
        ],
    };
    // The bite's ends must meet the square's: the left side runs (0,10)→(0,8), arc, (0,2)→(0,0).
    let mut segments = lp.segments;
    segments.insert(
        3,
        seg(
            Segment::Line {
                a: Vec2::new(0.0, 10.0),
                b: Vec2::new(0.0, 8.0),
            },
            "l1",
        ),
    );
    segments[4] = seg(
        Segment::Arc {
            center: Vec2::new(0.0, 5.0),
            radius: 3.0,
            start: PI / 2.0,
            sweep: -PI,
        },
        "bite",
    );
    segments.push(seg(
        Segment::Line {
            a: Vec2::new(0.0, 2.0),
            b: Vec2::new(0.0, 0.0),
        },
        "l2",
    ));
    let p = Profile {
        loops: vec![ProfileLoop { segments }],
    };
    let b = extrude(&p, &Frame::WORLD, 4.0, "f").unwrap();
    let area2d = 100.0 + PI * 12.5 - PI * 4.5;
    let perim = 10.0 + PI * 5.0 + 10.0 + 2.0 + PI * 3.0 + 2.0;
    assert_solid(
        &b,
        4.0 * area2d,
        2.0 * area2d + 4.0 * perim,
        [0.0, 0.0, 0.0],
        [15.0, 10.0, 4.0],
        1e-12,
    );
}

#[test]
fn periodic_spline_disc_matches_its_ellipse() {
    let pts: Vec<Vec2> = (0..72)
        .map(|i| {
            let a = 2.0 * PI * i as f64 / 72.0;
            Vec2::new(30.0 * a.cos(), 20.0 * a.sin())
        })
        .collect();
    let p = Profile::points(&pts, PointsInterp::PeriodicSpline, "side").unwrap();
    let b = extrude(&p, &Frame::WORLD, 10.0, "disc").unwrap();
    assert!(check::check(&b).is_empty());
    let v = measure::volume(&b);
    assert!((v / (PI * 6000.0) - 1.0).abs() < 2e-5, "{v}");
    assert_eq!(b.face_count(), 3);
    let bb = measure::bbox(&b);
    assert!((bb.max.x - 30.0).abs() < 1e-3 && (bb.min.y + 20.0).abs() < 1e-3);
}

#[test]
fn partial_revolve_of_a_square() {
    let p = Profile::polygon(
        &[
            Vec2::new(10.0, 0.0),
            Vec2::new(20.0, 0.0),
            Vec2::new(20.0, 10.0),
            Vec2::new(10.0, 10.0),
        ],
        &["bottom", "outer", "top", "inner"],
    );
    let b = revolve(&p, &xz_frame(), Vec3::ZERO, Vec3::Z, PI / 2.0, "rv").unwrap();
    let v = PI * 300.0 * 10.0 / 4.0;
    let a = 200.0 + 2.0 * PI * 300.0 / 4.0 + 2.0 * PI * 200.0 / 4.0 + 2.0 * PI * 100.0 / 4.0;
    assert_solid(&b, v, a, [0.0, 0.0, 0.0], [20.0, 20.0, 10.0], 1e-12);
    assert_eq!(b.name_counts().len(), 6);
}

#[test]
fn partial_revolve_touching_the_axis_has_a_pole() {
    // A right triangle on the axis turned 120°: a cone sector closed by two caps.
    let p = Profile::polygon(
        &[
            Vec2::new(0.0, 0.0),
            Vec2::new(6.0, 0.0),
            Vec2::new(0.0, 8.0),
        ],
        &["base", "slant", "axis"],
    );
    let b = revolve(&p, &xz_frame(), Vec3::ZERO, Vec3::Z, 2.0 * PI / 3.0, "c").unwrap();
    let v = PI * 36.0 * 8.0 / 3.0 / 3.0;
    let a = (PI * 36.0 + PI * 6.0 * 10.0) / 3.0 + 2.0 * 24.0;
    assert_solid(&b, v, a, [-3.0, 0.0, 0.0], [6.0, 6.0, 8.0], 1e-12);
}

#[test]
fn full_revolve_of_a_half_disc_is_a_sphere() {
    let seg = |segment: Segment, name: &str| NamedSegment::new(segment, name);
    let p = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                seg(
                    Segment::Arc {
                        center: Vec2::ZERO,
                        radius: 5.0,
                        start: -PI / 2.0,
                        sweep: PI,
                    },
                    "ball",
                ),
                seg(
                    Segment::Line {
                        a: Vec2::new(0.0, 5.0),
                        b: Vec2::new(0.0, -5.0),
                    },
                    "axis",
                ),
            ],
        }],
    };
    let b = revolve(&p, &xz_frame(), Vec3::ZERO, Vec3::Z, 2.0 * PI, "s").unwrap();
    assert_solid(
        &b,
        4.0 / 3.0 * PI * 125.0,
        4.0 * PI * 25.0,
        [-5.0; 3],
        [5.0; 3],
        1e-11,
    );
}

#[test]
fn full_revolve_of_an_off_axis_circle_is_a_torus() {
    let p = Profile::circle(Vec2::new(10.0, 0.0), 4.0);
    let b = revolve(&p, &xz_frame(), Vec3::ZERO, Vec3::Z, 2.0 * PI, "tor").unwrap();
    let v = 2.0 * PI * PI * 10.0 * 4.0;
    let a = 4.0 * PI * PI * 10.0 * 2.0;
    assert_solid(&b, v, a, [-12.0, -12.0, -2.0], [12.0, 12.0, 2.0], 1e-11);
}

#[test]
fn turned_stack_with_a_cone() {
    let pts = [
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 10.0),
        (10.0, 10.0),
        (10.0, 28.0),
        (8.0, 30.0),
        (0.0, 30.0),
    ];
    let pts: Vec<Vec2> = pts.iter().map(|&(x, y)| Vec2::new(x, y)).collect();
    let p = Profile::polygon(&pts, &["a", "b", "c", "d", "e", "f", "axis"]);
    let b = revolve(&p, &xz_frame(), Vec3::ZERO, Vec3::Z, 2.0 * PI, "t").unwrap();
    let frustum = PI * 2.0 / 3.0 * (100.0 + 80.0 + 64.0);
    let v = PI * 400.0 * 10.0 + PI * 100.0 * 18.0 + frustum;
    let a = 400.0 * PI
        + 400.0 * PI
        + 300.0 * PI
        + 360.0 * PI
        + 64.0 * PI
        + PI * 18.0 * 2.0 * 2f64.sqrt();
    assert_solid(&b, v, a, [-20.0, -20.0, 0.0], [20.0, 20.0, 30.0], 1e-12);
    assert_eq!(b.face_count(), 6);
}

#[test]
fn revolving_a_spline_makes_a_surface_of_revolution() {
    // A spline bulge on the outside of a tube section: the volume by Pappus would need the
    // spline's centroid, so check against the tessellation instead, and validity exactly.
    let seg = |segment: Segment, name: &str| NamedSegment::new(segment, name);
    let spline = yack_geom::BSpline::interpolate(&[
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(12.0, 3.0, 0.0),
        Vec3::new(11.0, 7.0, 0.0),
        Vec3::new(10.0, 10.0, 0.0),
    ])
    .unwrap();
    let p = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                seg(
                    Segment::Line {
                        a: Vec2::new(5.0, 0.0),
                        b: Vec2::new(10.0, 0.0),
                    },
                    "bottom",
                ),
                seg(
                    Segment::Spline {
                        curve: spline,
                        reversed: false,
                    },
                    "bulge",
                ),
                seg(
                    Segment::Line {
                        a: Vec2::new(10.0, 10.0),
                        b: Vec2::new(5.0, 10.0),
                    },
                    "top",
                ),
                seg(
                    Segment::Line {
                        a: Vec2::new(5.0, 10.0),
                        b: Vec2::new(5.0, 0.0),
                    },
                    "bore",
                ),
            ],
        }],
    };
    let b = revolve(&p, &xz_frame(), Vec3::ZERO, Vec3::Z, 2.0 * PI, "r").unwrap();
    assert!(check::check(&b).is_empty());
    let v = measure::volume(&b);
    let m = mesh::tessellate(&b, &COARSE).unwrap();
    let r = mesh::report(&b, &m, b.solid_ids().next().unwrap());
    assert!(r.watertight(), "{r:?}");
    assert!((r.volume - v).abs() < 5e-3 * v, "{} vs {v}", r.volume);
    let bb = measure::bbox(&b);
    assert!(bb.max.x > 12.0 && bb.max.x < 12.5);
}

#[test]
fn bad_dimensions_are_invalid_input() {
    let rect = |w: f64| Profile::rect(Vec2::ZERO, w, 1.0);
    let bad = [
        extrude(&rect(f64::NAN), &Frame::WORLD, 1.0, "e"),
        extrude(&rect(1e-9), &Frame::WORLD, 1.0, "e"),
        extrude(&rect(1.0), &Frame::WORLD, f64::INFINITY, "e"),
        extrude(&rect(1.0), &Frame::WORLD, f64::NAN, "e"),
    ];
    for r in bad {
        assert!(matches!(r, Err(Error::InvalidInput(_))), "{r:?}");
    }
}
