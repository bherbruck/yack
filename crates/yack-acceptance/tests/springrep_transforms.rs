// yack: solids mirrored and scaled uniformly, measured against the original moved by hand.
// Adapted from springrep@a03e8a1 tests/transforms.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); transform_similar(m, t) becomes
// Solid::transform(&Similarity::from_rows(..)) and a refused map is refused by Similarity::from_rows
// / Rigid::from_rows as ErrorKind::Invalid; the block, ball, rod and torus are made with
// make::block / sphere / cylinder / torus on the same frames (springrep circles take a diameter);
// mass_properties becomes volume / centroid / inertia (inertia about the centroid in both); the
// spline prism's outline is Loop::through; the mesh-built prism and reading back a Rigid's rotation
// are API gaps.
//
// Each result is checked valid and measured against the original moved by hand: volume times s³,
// area times s², the centre moved, and the inertia turned by the rotation and scaled by s⁵.

use yack::prelude::*;

/// A turn about z then a quarter turn about x, as another program writes it: to nine digits,
/// which leaves it 3e-10 from a rotation.
const NINE_DIGITS: [[f64; 3]; 3] = [[0.963144594, 0.268984183, 0.0], [0.0, 0.0, 1.0], [0.268984183, -0.963144594, 0.0]];

/// The linear map `m` and shift `t` as the rows yack's transforms take.
fn rows(m: [[f64; 3]; 3], t: [f64; 3]) -> [[f64; 4]; 3] {
    [0, 1, 2].map(|i| [m[i][0], m[i][1], m[i][2], t[i]])
}

// API GAP (test a_matrix_written_to_nine_digits_moves_a_solid_as_the_rotation_it_stands_for omitted):
// its solid is a 90-sided prism built from a triangle list (springrep mesh::solid); §4.3 has no way
// to make a solid from triangles (only step::read/sew of STEP text), and hand-building it is barred.

#[test]
fn a_rigid_motion_from_outside_is_the_nearest_rotation() {
    // The matrix written to nine digits is taken as a rigid motion.
    let t = Rigid::from_rows(rows(NINE_DIGITS, [1.0, 2.0, 3.0])).unwrap();
    // API GAP: Rigid has no accessor for its rows in §4.3, so the checks that the stored rotation
    // is orthonormal to 4·EPSILON, within 1e-9 of the written rows, and that the shift is (1, 2, 3)
    // are dropped.
    let _ = t;
    // A mirror in z, and a stretch of a thousandth along x, are not rigid motions.
    let mirror = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]];
    let stretch = [[1.001, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert!(matches!(Rigid::from_rows(rows(mirror, [0.0; 3])), Err(e) if e.kind == ErrorKind::Invalid));
    assert!(matches!(Rigid::from_rows(rows(stretch, [0.0; 3])), Err(e) if e.kind == ErrorKind::Invalid));
}

/// A block with a ball on it and a rod bored through both: planes, a sphere, a cylinder, and the
/// curves where they cross. Off every axis, so a mirror or turn moves its centre.
fn part() -> Solid {
    let base = block(&Frame::at([1.0, 2.0, 0.0]), [8.0, 5.0, 4.0])
        .unwrap()
        .union(&sphere(&Frame::at([4.0, 4.0, 4.0]), 2.5).unwrap())
        .unwrap();
    base.difference(&cylinder(&Frame::at([6.0, 4.5, -1.0]), 1.2, 9.0).unwrap()).unwrap()
}

/// A torus of major radius 5 and minor 1.5 about (2, 1, 0), and a prism on a closed spline.
fn curved() -> Solid {
    let ring = torus(&Frame::at([2.0, 1.0, 0.0]), 5.0, 3.0 / 2.0).unwrap();
    let pts = [[0.0, 0.0], [4.0, -1.0], [6.0, 2.0], [3.0, 5.0], [-1.0, 3.0]].map(|[x, y]: [f64; 2]| [x + 12.0, y]);
    let outline = Profile::new(Loop::through(&pts).unwrap(), vec![]).unwrap();
    let prism = extrude(&Frame::WORLD, &outline, 3.0).unwrap();
    ring.union(&prism).unwrap()
}

/// `p` moved by the linear map `m` and then `t`.
fn moved(m: [[f64; 3]; 3], t: [f64; 3], p: Point3) -> Point3 {
    Point3::new(
        m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + t[0],
        m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + t[1],
        m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + t[2],
    )
}

/// `b` moved by the similarity `m`, `t`, of uniform scale `s`, checked against its own measures
/// moved by hand.
fn assert_moved(b: &Solid, m: [[f64; 3]; 3], t: [f64; 3], s: f64) -> Solid {
    let out = b.transform(&Similarity::from_rows(rows(m, t)).unwrap()).unwrap();
    let issues = out.check();
    assert!(issues.is_empty(), "{issues:?}");
    let rel = |got: f64, want: f64| (got - want).abs() <= 1e-9 * want.abs().max(1.0);
    let (before, after) = (b.volume(), out.volume());
    assert!(rel(after, before * s.powi(3)), "volume {after} vs {}", before * s.powi(3));
    assert!(rel(out.area(), b.area() * s * s));
    let centre = moved(m, t, b.centroid().unwrap());
    let got = out.centroid().unwrap();
    assert!(
        got.distance(centre) <= 1e-9 * (1.0 + centre.to_vec().length()),
        "{got:?} vs {centre:?}"
    );
    // The inertia about the centre turns with the rotation part, q = m / s, and grows by s⁵.
    let (inertia_before, inertia_after) = (b.inertia(), out.inertia());
    let q = m.map(|row| row.map(|x| x / s));
    for i in 0..3 {
        for j in 0..3 {
            let want: f64 = (0..3)
                .flat_map(|k| (0..3).map(move |l| (k, l)))
                .map(|(k, l)| q[i][k] * inertia_before[k][l] * q[j][l])
                .sum::<f64>()
                * s.powi(5);
            assert!(rel(inertia_after[i][j], want), "inertia {i}{j}: {} vs {want}", inertia_after[i][j]);
        }
    }
    out
}

const MIRROR_X: [[f64; 3]; 3] = [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

#[test]
fn a_part_mirrored_across_a_plane_is_its_mirror_image() {
    assert_moved(&part(), MIRROR_X, [0.0; 3], 1.0);
    // Across the plane x + y = 3: its normal (1, 1, 0)/√2, so I − 2nnᵀ and a shift of 2·3·n/√2.
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let m = [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
    let k = 2.0 * h * h;
    assert_moved(&part(), m, [3.0 * k, 3.0 * k, 0.0], 1.0);
}

#[test]
fn a_torus_and_a_spline_prism_mirror_and_scale() {
    assert_moved(&curved(), MIRROR_X, [1.0, 0.0, 0.0], 1.0);
    let s = 1.7;
    let m = MIRROR_X.map(|row| row.map(|x| x * s));
    assert_moved(&curved(), m, [0.0, -2.0, 3.0], s);
}

#[test]
fn a_part_scaled_turned_and_mirrored_at_once() {
    // A quarter turn about z, then a mirror across z = 0, all scaled by 1.5 and moved.
    let s = 1.5;
    let m = [[0.0, -s, 0.0], [s, 0.0, 0.0], [0.0, 0.0, -s]];
    assert_moved(&part(), m, [4.0, -1.0, 2.0], s);
    // Scaled down, and a turn of a third about the diagonal, no mirror.
    let s = 0.4;
    let m = [[0.0, 0.0, s], [s, 0.0, 0.0], [0.0, s, 0.0]];
    assert_moved(&part(), m, [0.0; 3], s);
}

#[test]
fn a_part_mirrored_and_joined_to_itself() {
    // The part across x = 2 overlaps it over x 1..3: the union and intersection add up to both.
    let a = part();
    let b = a.transform(&Similarity::from_rows(rows(MIRROR_X, [4.0, 0.0, 0.0])).unwrap()).unwrap();
    let (u, i) = (a.union(&b).unwrap(), a.intersection(&b).unwrap());
    for x in [&u, &i] {
        assert!(x.check().is_empty());
    }
    let v = a.volume();
    let sum = u.volume() + i.volume();
    assert!((sum - 2.0 * v).abs() <= 1e-7 * a.area(), "{sum} vs {}", 2.0 * v);
}

#[test]
fn a_stretch_or_shear_is_refused() {
    let stretch = [[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert!(matches!(Similarity::from_rows(rows(stretch, [0.0; 3])), Err(e) if e.kind == ErrorKind::Invalid));
    let shear = [[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert!(matches!(Similarity::from_rows(rows(shear, [0.0; 3])), Err(e) if e.kind == ErrorKind::Invalid));
}
