// yack: unit tests of the shim's facades.

use super::prelude::*;
use super::*;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs()))
}

#[test]
fn vectors_and_points() {
    let a = Vector3::new(1.0, 2.0, 3.0);
    let b = Vector3::new(-2.0, 0.5, 4.0);
    assert_eq!(a.x, 1.0);
    assert_eq!(a.dot(&b), -2.0 + 1.0 + 12.0);
    let c = a.cross(&b);
    assert_eq!(c, Vector3::new(2.0 * 4.0 - 3.0 * 0.5, 3.0 * -2.0 - 1.0 * 4.0, 1.0 * 0.5 - 2.0 * -2.0));
    assert!(close(a.normalize().norm(), 1.0, 1e-15));
    assert_eq!(Vector3::<f64>::zeros().try_normalize(0.0), None);
    let p = Point3::new(1.0, 1.0, 1.0);
    let q = p + a;
    assert_eq!(q - p, a);
    assert_eq!(q.y, 3.0);
    assert_eq!(center(&p, &q), Point3::new(1.5, 2.0, 2.5));
    let mut r = p;
    r.x = 7.0;
    r += Vector3::x();
    assert_eq!(r.coords, Vector3::new(8.0, 1.0, 1.0));
    let u = Unit::new_normalize(Vector3::new(0.0, 3.0, 4.0));
    assert!(close(u.y, 0.6, 1e-15));
    assert!(Unit::try_new(Vector3::new(1e-9, 0.0, 0.0), 1e-6).is_none());
    assert_eq!(a.amax(), 3.0);
    assert_eq!(b.imin(), 0);
    let outer = a * b.transpose();
    assert_eq!(outer[(2, 0)], 3.0 * -2.0);
}

#[test]
fn matrices() {
    let m = Matrix3::new(2.0, 1.0, 0.0, 1.0, 3.0, 1.0, 0.0, 1.0, 4.0);
    assert_eq!(m[(0, 1)], 1.0);
    assert_eq!(m.column(2), Vector3::new(0.0, 1.0, 4.0));
    let inv = m.try_inverse().unwrap_or_else(Matrix3::zeros);
    let id = m * inv;
    assert!(id.is_identity(1e-14));
    assert!(close(m.determinant(), 2.0 * 11.0 - 1.0 * 4.0, 1e-14));
    let b = Vector3::new(1.0, 2.0, 3.0);
    let x = m.lu().solve(&b).unwrap_or_else(Vector3::zeros);
    assert!(((m * x) - b).norm() < 1e-14);
    let singular = Matrix3::new(1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 0.0, 0.0, 1.0);
    assert!(singular.try_inverse().is_none());
    assert!(singular.lu().solve(&b).is_none());
    // 4 × 4 and 6 × 6 through LU.
    let m4 = Matrix4::new(
        4.0, 1.0, 0.0, 0.5, 1.0, 5.0, 1.0, 0.0, 0.0, 1.0, 6.0, 1.0, 0.5, 0.0, 1.0, 7.0,
    );
    let inv4 = m4.try_inverse().unwrap_or_else(Matrix4::zeros);
    assert!((m4 * inv4).is_identity(1e-13));
    let m6 = SMatrix::<f64, 6, 6>::from_fn(|r, c| if r == c { 10.0 } else { 1.0 / (1 + r + c) as f64 });
    let rhs = SVector::<f64, 6>::from_fn(|r, _| r as f64);
    let sol = m6.lu().solve(&rhs).unwrap_or_else(SVector::<f64, 6>::zeros);
    assert!((m6 * sol - rhs).norm() < 1e-12);
    let m32 = Matrix3x2::from_columns(&[Vector3::new(1.0, 0.0, 1.0), Vector3::new(0.0, 1.0, 1.0)]);
    let normal = m32.transpose() * m32;
    assert_eq!(normal, Matrix2::new(2.0, 1.0, 1.0, 2.0));
}

#[test]
fn eigen_and_svd() {
    let m = Matrix3::new(4.0, 1.0, 0.5, 1.0, 3.0, 0.2, 0.5, 0.2, 1.0);
    let e = m.symmetric_eigen();
    for k in 0..3 {
        let v = e.eigenvectors.column(k);
        assert!((m * v - v * e.eigenvalues[k]).norm() < 1e-12);
        assert!(close(v.norm(), 1.0, 1e-12));
    }
    assert!((e.recompose() - m).norm() < 1e-12);
    let a = DMatrix::from_fn(5, 3, |i, j| ((i + 1) * (j + 2)) as f64 + if i == j { 1.0 } else { 0.0 });
    let b = DVector::from_fn(5, |i, _| i as f64 - 1.0);
    let svd = a.clone().svd(true, true);
    let s = &svd.singular_values;
    assert!(s[0] >= s[1] && s[1] >= s[2]);
    let x = svd.solve(&b, 1e-12).unwrap_or_else(|_| DVector::zeros(3));
    // The normal equations hold at the least-squares solution.
    let r = &a * &x - &b;
    assert!((a.transpose() * r).norm() < 1e-10);
    let wide = a.transpose();
    let svd2 = wide.clone().svd(true, true);
    let back = svd2.recompose().unwrap_or_else(|_| DMatrix::zeros(3, 5));
    assert!((back - wide).norm() < 1e-11);
    let d = DMatrix::from_fn(4, 4, |r, c| if r == c { 3.0 } else { 0.5 });
    let de = d.clone().symmetric_eigen();
    assert!(de.eigenvalues.iter().all(|&l| l > 0.0));
    let lu = d.clone().lu();
    let y = lu.solve(&DVector::from_column_slice(&[1.0, 2.0, 3.0, 4.0])).unwrap_or_else(|| DVector::zeros(4));
    assert!((&d * &y - DVector::from_column_slice(&[1.0, 2.0, 3.0, 4.0])).norm() < 1e-13);
}

#[test]
fn rotations() {
    let axis = Unit::new_normalize(Vector3::new(1.0, 2.0, 2.0));
    let r = Rotation3::from_axis_angle(&axis, 0.7);
    assert!(close(r.angle(), 0.7, 1e-14));
    let q = UnitQuaternion::from_rotation_matrix(&r);
    let v = Vector3::new(0.3, -1.0, 2.0);
    assert!((q * v - r * v).norm() < 1e-14);
    assert!((q.to_rotation_matrix().into_inner() - r.into_inner()).norm() < 1e-14);
    let q2 = UnitQuaternion::from_axis_angle(&axis, 0.7);
    assert!((q2 * v - r * v).norm() < 1e-14);
    let half = UnitQuaternion::identity().slerp(&q2, 0.5);
    assert!(close(half.angle(), 0.35, 1e-14));
    let iso = Isometry3::from_parts(Translation3::new(1.0, 2.0, 3.0), q);
    let p = Point3::new(4.0, 5.0, 6.0);
    assert!((iso.inverse() * (iso * p) - p).norm() < 1e-13);
    let between = Rotation3::rotation_between(&Vector3::x(), &Vector3::y());
    assert!(between.is_some_and(|b| (b * Vector3::x() - Vector3::y()).norm() < 1e-15));
    assert!(Rotation3::rotation_between(&Vector3::x(), &-Vector3::x()).is_none());
    let r2 = Rotation2::new(0.5);
    assert!(close(r2.angle(), 0.5, 1e-15));
}

#[test]
fn parallel_steps_keep_order() {
    let v: Vec<u64> = (0..1000).collect();
    let doubled: Vec<u64> = v.par_iter().map(|x| x * 2).collect();
    assert_eq!(doubled, (0..1000).map(|x| x * 2).collect::<Vec<_>>());
    let sum: u64 = (0..10_000u64).into_par_iter().map(|x| x * 2).sum();
    assert_eq!(sum, 99_990_000);
    let firsts = v.par_iter().find_map_any(|&x| (x % 97 == 96).then_some(x));
    assert_eq!(firsts, Some(96));
    assert!(v.par_iter().all(|&x| x < 1000));
    assert!(!v.par_iter().any(|&x| x > 1000));
    let flat: Vec<u64> = v.par_chunks(10).flat_map_iter(|c| c.iter().copied()).collect();
    assert_eq!(flat, v);
    let (a, b) = join(|| 1, || 2);
    assert_eq!((a, b), (1, 2));
    let nested: Vec<Vec<u64>> = (0..8u64).into_par_iter().map(|i| (0..i).into_par_iter().collect()).collect();
    assert_eq!(nested[5], vec![0, 1, 2, 3, 4]);
    let folded: u64 = v.par_iter().fold(|| 0, |s, &x| s + x).sum();
    assert_eq!(folded, 499_500);
}

#[test]
fn uuids_and_clocks() {
    let id = Uuid::from_u128(0x0123_4567_89ab_cdef_0011_2233_4455_6677);
    assert_eq!(id.to_string(), "01234567-89ab-cdef-0011-223344556677");
    assert!(Uuid::from_u128(1) < Uuid::from_u128(2));
    assert_eq!(id.as_u128(), 0x0123_4567_89ab_cdef_0011_2233_4455_6677);
    let t = Instant::now();
    assert_eq!(t.elapsed(), std::time::Duration::ZERO);
    assert!(SystemTime::now().duration_since(UNIX_EPOCH).is_ok());
    let mut m: FxHashMap<u32, u32> = FxHashMap::default();
    m.insert(1, 2);
    assert_eq!(m.get(&1), Some(&2));
}
