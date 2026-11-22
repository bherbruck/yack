// yack: Solids mirrored and scaled uniformly: a similarity, a uniform scale times a rotation or a
// Adapted from springrep@a03e8a1 tests/transforms.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added.
//! Solids mirrored and scaled uniformly: a similarity, a uniform scale times a rotation or a
//! reflection, then a translation. Each result is checked valid and measured against the original
//! moved by hand: volume times s³, area times s², the centre moved, and the inertia turned by the
//! rotation and scaled by s⁵.

use std::f64::consts::{PI, TAU};

use yack_springrep::{
    Error,
    boolean::{difference, intersection, union},
    check,
    math::{Frame, Transform, Vec2, Vec3},
    build, measure, primitive,
    profile::{NamedSegment, PointsInterp, Profile, ProfileLoop, Segment},
    sweep::{extrude, revolve},
    topo::Brep,
};

/// A `2n`-sided mesh of a cylinder about z, its upper ring half a side round from its lower, as
/// a tool that turns each ring on its own writes one: every side triangle a hair off the axis.
fn twisted_prism(n: u32, r: f64, h: f64) -> Brep {
    let mut points = Vec::new();
    for k in 0..n {
        let a = TAU * f64::from(k) / f64::from(n);
        points.push(Vec3::new(r * a.cos(), r * a.sin(), 0.0));
        let a = a + PI / f64::from(n);
        points.push(Vec3::new(r * a.cos(), r * a.sin(), h));
    }
    points.extend([Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, h)]);
    let (low, high) = (2 * n, 2 * n + 1);
    let mut triangles = Vec::new();
    for k in 0..n {
        let j = (k + 1) % n;
        let (a, b, c, d) = (2 * k, 2 * j, 2 * j + 1, 2 * k + 1);
        triangles.extend([[a, b, d], [b, c, d], [low, b, a], [high, d, c]]);
    }
    build::solid_from_mesh(&points, &triangles).unwrap()
}

/// A turn about z then a quarter turn about x, as another program writes it: to nine digits,
/// which leaves it 3e-10 from a rotation.
const NINE_DIGITS: [[f64; 3]; 3] = [
    [0.963144594, 0.268984183, 0.0],
    [0.0, 0.0, 1.0],
    [0.268984183, -0.963144594, 0.0],
];

#[test]
fn a_matrix_written_to_nine_digits_moves_a_solid_as_the_rotation_it_stands_for() {
    // The prism's axis lands on y; a cylinder along y, 1 mm aside, shares a lens with it. Moved
    // by the matrix as written, the prism's 90 side planes had normals 1.7e-10 from unit and the
    // intersection came out with open edges.
    let mut prism = twisted_prism(45, 2.0, 18.0);
    prism.transform_similar(NINE_DIGITS, Vec3::ZERO).unwrap();
    assert!(check::check(&prism).is_empty());
    let frame = Frame::new(Vec3::new(1.0, 5.0, 0.0), Vec3::Y, Vec3::X).unwrap();
    let tool = primitive::cylinder(&frame, 2.0, 5.0, "tool").unwrap();
    let shared = intersection(&tool, &prism).unwrap();
    assert!(check::check(&shared).is_empty());
    // Two r = 2 circles 1 mm apart share 2r²·acos(d/2r) − (d/2)·√(4r² − d²) = 8.608 mm², over
    // 5 mm. The prism lies inside its circle, by at most r·(1 − cos 4°) = 4.9 µm at its rings,
    // along the 2r·acos(d/2r) = 5.3 mm of the lens's edge that is the prism's: under 0.13 mm³.
    let lens = 8.0 * (0.25_f64).acos() - 0.5 * 15.0_f64.sqrt();
    let v = measure::volume(&shared);
    assert!((5.0 * lens - 0.13..5.0 * lens).contains(&v), "{v}");
}

#[test]
fn a_rigid_motion_from_outside_is_the_nearest_rotation() {
    let t = Transform::rigid(NINE_DIGITS, Vec3::new(1.0, 2.0, 3.0)).unwrap();
    for (i, (row, written)) in t.r.iter().zip(&NINE_DIGITS).enumerate() {
        for (j, other) in t.r.iter().enumerate() {
            let dot: f64 = row.iter().zip(other).map(|(a, b)| a * b).sum();
            let want = if i == j { 1.0 } else { 0.0 };
            assert!((dot - want).abs() < 4.0 * f64::EPSILON, "{i}{j}: {dot}");
        }
        assert!(row.iter().zip(written).all(|(a, b)| (a - b).abs() < 1e-9));
    }
    assert_eq!(t.t, Vec3::new(1.0, 2.0, 3.0));
    // A mirror in z, and a stretch of a thousandth along x, are not rigid motions.
    let mirror = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]];
    let stretch = [[1.001, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert!(matches!(
        Transform::rigid(mirror, Vec3::ZERO),
        Err(Error::InvalidInput(_))
    ));
    assert!(matches!(
        Transform::rigid(stretch, Vec3::ZERO),
        Err(Error::InvalidInput(_))
    ));
}

fn block(id: &str, lo: [f64; 3], hi: [f64; 3]) -> Brep {
    let c = Vec2::new(0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]));
    let frame = Frame::new(Vec3::new(0.0, 0.0, lo[2]), Vec3::Z, Vec3::X).unwrap();
    extrude(
        &Profile::rect(c, hi[0] - lo[0], hi[1] - lo[1]),
        &frame,
        hi[2] - lo[2],
        id,
    )
    .unwrap()
}

fn rod(id: &str, base: [f64; 3], axis: [f64; 3], r: f64, len: f64) -> Brep {
    let frame = Frame::from_axis(Vec3::from_array(base), Vec3::from_array(axis)).unwrap();
    extrude(&Profile::circle(Vec2::ZERO, 2.0 * r), &frame, len, id).unwrap()
}

fn seg(segment: Segment, name: &str) -> NamedSegment {
    NamedSegment::new(segment, name)
}

fn ball(id: &str, centre: [f64; 3], r: f64) -> Brep {
    let p = Profile {
        loops: vec![ProfileLoop {
            segments: vec![
                seg(
                    Segment::Arc {
                        center: Vec2::ZERO,
                        radius: r,
                        start: -PI / 2.0,
                        sweep: PI,
                    },
                    "skin",
                ),
                seg(
                    Segment::Line {
                        a: Vec2::new(0.0, r),
                        b: Vec2::new(0.0, -r),
                    },
                    "axis",
                ),
            ],
        }],
    };
    let c = Vec3::from_array(centre);
    let frame = Frame::new(c, Vec3::new(0.0, -1.0, 0.0), Vec3::X).unwrap();
    revolve(&p, &frame, c, Vec3::Z, TAU, id).unwrap()
}

/// A block with a ball on it and a rod bored through both: planes, a sphere, a cylinder, and the
/// curves where they cross. Off every axis, so a mirror or turn moves its centre.
fn part() -> Brep {
    let base = union(
        &block("a", [1.0, 2.0, 0.0], [9.0, 7.0, 4.0]),
        &ball("b", [4.0, 4.0, 4.0], 2.5),
    )
    .unwrap();
    difference(
        &base,
        &rod("c", [6.0, 4.5, -1.0], [0.0, 0.0, 1.0], 1.2, 9.0),
    )
    .unwrap()
}

/// A torus of major radius 5 and minor 1.5 about (2, 1, 0), and a prism on a closed spline.
fn curved() -> Brep {
    let tube = Profile::circle(Vec2::new(5.0, 0.0), 3.0);
    let frame = Frame::new(Vec3::new(2.0, 1.0, 0.0), Vec3::new(0.0, -1.0, 0.0), Vec3::X).unwrap();
    let torus = revolve(&tube, &frame, Vec3::new(2.0, 1.0, 0.0), Vec3::Z, TAU, "t").unwrap();
    let pts = [(0.0, 0.0), (4.0, -1.0), (6.0, 2.0), (3.0, 5.0), (-1.0, 3.0)]
        .map(|(x, y)| Vec2::new(x + 12.0, y));
    let outline = Profile::points(&pts, PointsInterp::PeriodicSpline, "s").unwrap();
    let prism = extrude(&outline, &Frame::WORLD, 3.0, "p").unwrap();
    union(&torus, &prism).unwrap()
}

/// `p` moved by the linear map `m` and then `t`.
fn moved(m: [[f64; 3]; 3], t: Vec3, p: Vec3) -> Vec3 {
    Vec3::new(
        m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z,
        m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z,
        m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z,
    ) + t
}

/// `b` moved by the similarity `m`, `t`, of uniform scale `s`, checked against its own measures
/// moved by hand.
fn assert_moved(b: &Brep, m: [[f64; 3]; 3], t: Vec3, s: f64) -> Brep {
    let mut out = b.clone();
    out.transform_similar(m, t).unwrap();
    let issues = check::check(&out);
    assert!(issues.is_empty(), "{issues:?}");
    let (before, after) = (measure::mass_properties(b), measure::mass_properties(&out));
    let rel = |got: f64, want: f64| (got - want).abs() <= 1e-9 * want.abs().max(1.0);
    assert!(
        rel(after.volume, before.volume * s.powi(3)),
        "volume {} vs {}",
        after.volume,
        before.volume * s.powi(3)
    );
    assert!(rel(measure::area(&out), measure::area(b) * s * s));
    let centre = moved(m, t, before.centre);
    assert!(
        after.centre.dist(centre) <= 1e-9 * (1.0 + centre.norm()),
        "{:?} vs {centre:?}",
        after.centre
    );
    // The inertia about the centre turns with the rotation part, q = m / s, and grows by s⁵.
    let q = m.map(|row| row.map(|x| x / s));
    for i in 0..3 {
        for j in 0..3 {
            let want: f64 = (0..3)
                .flat_map(|k| (0..3).map(move |l| (k, l)))
                .map(|(k, l)| q[i][k] * before.inertia[k][l] * q[j][l])
                .sum::<f64>()
                * s.powi(5);
            assert!(
                rel(after.inertia[i][j], want),
                "inertia {i}{j}: {} vs {want}",
                after.inertia[i][j]
            );
        }
    }
    out
}

const MIRROR_X: [[f64; 3]; 3] = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

#[test]
fn a_part_mirrored_across_a_plane_is_its_mirror_image() {
    assert_moved(&part(), MIRROR_X, Vec3::ZERO, 1.0);
    // Across the plane x + y = 3: its normal (1, 1, 0)/√2, so I − 2nnᵀ and a shift of 2·3·n/√2.
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let m = [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
    assert_moved(&part(), m, Vec3::new(3.0, 3.0, 0.0) * (2.0 * h * h), 1.0);
}

#[test]
fn a_torus_and_a_spline_prism_mirror_and_scale() {
    assert_moved(&curved(), MIRROR_X, Vec3::new(1.0, 0.0, 0.0), 1.0);
    let s = 1.7;
    let m = MIRROR_X.map(|row| row.map(|x| x * s));
    assert_moved(&curved(), m, Vec3::new(0.0, -2.0, 3.0), s);
}

#[test]
fn a_part_scaled_turned_and_mirrored_at_once() {
    // A quarter turn about z, then a mirror across z = 0, all scaled by 1.5 and moved.
    let s = 1.5;
    let m = [[0.0, -s, 0.0], [s, 0.0, 0.0], [0.0, 0.0, -s]];
    assert_moved(&part(), m, Vec3::new(4.0, -1.0, 2.0), s);
    // Scaled down, and a turn of a third about the diagonal, no mirror.
    let s = 0.4;
    let m = [[0.0, 0.0, s], [s, 0.0, 0.0], [0.0, s, 0.0]];
    assert_moved(&part(), m, Vec3::ZERO, s);
}

#[test]
fn a_part_mirrored_and_joined_to_itself() {
    // The part across x = 2 overlaps it over x 1..3: the union and intersection add up to both.
    let a = part();
    let mut b = a.clone();
    b.transform_similar(MIRROR_X, Vec3::new(4.0, 0.0, 0.0))
        .unwrap();
    let (u, i) = (union(&a, &b).unwrap(), intersection(&a, &b).unwrap());
    for x in [&u, &i] {
        assert!(check::check(x).is_empty());
    }
    let v = measure::volume(&a);
    let sum = measure::volume(&u) + measure::volume(&i);
    assert!(
        (sum - 2.0 * v).abs() <= 1e-7 * measure::area(&a),
        "{sum} vs {}",
        2.0 * v
    );
}

#[test]
fn a_stretch_or_shear_is_refused() {
    let mut b = part();
    let stretch = [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert!(matches!(
        b.transform_similar(stretch, Vec3::ZERO),
        Err(Error::InvalidInput(_))
    ));
    let shear = [[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert!(matches!(
        b.transform_similar(shear, Vec3::ZERO),
        Err(Error::InvalidInput(_))
    ));
}
