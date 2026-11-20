// yack: points against solids with closed forms: which the solid holds, at edges, corners, seams and poles.
// Adapted from springrep@a03e8a1 tests/classify.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); Rays::holds(p) == Some(b)
// becomes Solid::classify == In / Out and Rays::side becomes Solid::classify (Inside/Outside/On are
// In/Out/On), points classified in one call per solid; edit::affine is
// Solid::transform_affine(&Affine::from_rows(..)) with the map's point images computed from its rows;
// Brep::default() is make::empty(); the skin's Surface::project then eval is FaceRef::closest on
// the solid's one face (the face is the whole surface).

use yack::prelude::*;

/// A fixed sequence of numbers in [0, 1): the same points every run.
struct Seq(u64);

impl Seq {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// A point of the box from `lo` to `hi`.
    fn within(&mut self, lo: [f64; 3], hi: [f64; 3]) -> Vec3 {
        Vec3::new(
            lo[0] + (hi[0] - lo[0]) * self.next(),
            lo[1] + (hi[1] - lo[1]) * self.next(),
            lo[2] + (hi[2] - lo[2]) * self.next(),
        )
    }
}

/// A micrometre: a point nearer the boundary than this may be placed either way, or not at all.
const NEAR: f64 = 1e-3;

fn held(inside: bool) -> Side {
    if inside { Side::In } else { Side::Out }
}

/// Classifies `cases` (point, whether it is held) in one call and checks each.
fn assert_holds(b: &Solid, cases: &[(Point3, bool)]) {
    let pts: Vec<Point3> = cases.iter().map(|c| c.0).collect();
    let got = b.classify(&pts).unwrap();
    assert_eq!(got.len(), cases.len());
    for (&(p, inside), side) in cases.iter().zip(got) {
        assert_eq!(side, held(inside), "{p:?}");
    }
}

#[test]
fn points_about_a_cylinder_are_held_where_the_closed_form_says() {
    let (r, h) = (2.0, 10.0);
    let b = cylinder(&Frame::WORLD, r, h).unwrap();
    let mut seq = Seq(11);
    let mut cases = Vec::new();
    for _ in 0..20_000 {
        let p = Point3::ORIGIN + seq.within([-3.0, -3.0, -2.0], [3.0, 3.0, 12.0]);
        let rho = p.x.hypot(p.y);
        if (rho - r).abs() < NEAR || p.z.abs() < NEAR || (p.z - h).abs() < NEAR {
            continue;
        }
        let inside = rho < r && p.z > 0.0 && p.z < h;
        cases.push((p, inside));
    }
    assert_holds(&b, &cases);
}

#[test]
fn points_about_a_block_are_held_where_its_slabs_say() {
    let size = [4.0, 3.0, 2.0];
    let b = block(&Frame::WORLD, size).unwrap();
    let mut seq = Seq(5);
    let mut cases = Vec::new();
    for _ in 0..20_000 {
        let p = Point3::ORIGIN + seq.within([-1.0; 3], [5.0, 4.0, 3.0]);
        let q = p.to_array();
        if (0..3).any(|k| q[k].abs() < NEAR || (q[k] - size[k]).abs() < NEAR) {
            continue;
        }
        let inside = (0..3).all(|k| q[k] > 0.0 && q[k] < size[k]);
        cases.push((p, inside));
    }
    assert_holds(&b, &cases);
}

#[test]
fn a_torus_holds_its_tube_and_neither_its_hole_nor_a_void_cut_in_a_block() {
    let (major, minor) = (3.0, 1.0);
    let ring = torus(&Frame::WORLD, major, minor).unwrap();
    let mut seq = Seq(3);
    let mut cases = Vec::new();
    for _ in 0..5_000 {
        let p = Point3::ORIGIN + seq.within([-4.5, -4.5, -1.5], [4.5, 4.5, 1.5]);
        let from_tube = (p.x.hypot(p.y) - major).hypot(p.z);
        if (from_tube - minor).abs() < NEAR {
            continue;
        }
        cases.push((p, from_tube < minor));
    }
    assert_holds(&ring, &cases);

    let outer = block(&Frame::WORLD, [6.0, 6.0, 6.0]).unwrap();
    let c = Point3::new(3.0, 3.0, 3.0);
    let hollow = outer.difference(&sphere(&Frame::at(c), 1.5).unwrap()).unwrap();
    let mut cases = Vec::new();
    for _ in 0..5_000 {
        let p = Point3::ORIGIN + seq.within([-1.0; 3], [7.0; 3]);
        let q = p.to_array();
        let from_void = (p - c).length();
        let on_wall = (0..3).any(|k| q[k].abs() < NEAR || (q[k] - 6.0).abs() < NEAR);
        if (from_void - 1.5).abs() < NEAR || on_wall {
            continue;
        }
        let in_block = (0..3).all(|k| q[k] > 0.0 && q[k] < 6.0);
        cases.push((p, in_block && from_void > 1.5));
    }
    assert_holds(&hollow, &cases);
}

#[test]
fn an_arena_with_no_solid_holds_no_point() {
    assert_eq!(empty().classify(&[[0.0, 0.0, 0.0]]).unwrap(), vec![Side::Out]);
}

/// A ball stretched and sheared, as the conformance case that found this: its skin is a rational
/// spline surface with a pole at each end, a whole row of control points at one point. Returns the
/// solid, the map's point image, and the ball's centre and radius.
fn stretched_ball() -> (Solid, impl Fn(Point3) -> Point3, Point3, f64) {
    let c = Point3::new(-2.919, -1.302, -1.162);
    let r = 1.815;
    let m = [[0.1028, -0.9280, 0.5227], [-1.0971, 0.5789, 0.4802], [1.2920, 0.5654, 0.3662]];
    let t = [20.147, -35.624, 7.546];
    let map = Affine::from_rows([0, 1, 2].map(|i| [m[i][0], m[i][1], m[i][2], t[i]])).unwrap();
    let image = move |p: Point3| {
        let [x, y, z] = [0, 1, 2].map(|i| m[i][0] * p.x + m[i][1] * p.y + m[i][2] * p.z + t[i]);
        Point3::new(x, y, z)
    };
    (sphere(&Frame::at(c), r).unwrap().transform_affine(&map).unwrap(), image, c, r)
}

#[test]
fn a_point_near_a_stretched_balls_pole_projects_onto_its_skin_there() {
    let (b, map, c, r) = stretched_ball();
    let face = b.faces()[0];
    let s = b.face(face).unwrap();
    for lat in [80.0f64, 85.0, 88.0, 89.5, 89.99, -85.0, -89.5] {
        for lon in [10.0f64, 100.0, 200.0, 300.0] {
            let (la, lo) = (lat.to_radians(), lon.to_radians());
            let local = Vec3::new(la.cos() * lo.cos(), la.cos() * lo.sin(), la.sin());
            // On the skin, and a way off it along the ball's radius, inside and out.
            for scale in [1.0, 0.97, 1.03] {
                let p = map(c + local * (r * scale));
                let foot = s.closest(p).unwrap();
                if scale == 1.0 {
                    let off = foot.distance(p);
                    assert!(off < 1e-9, "lat {lat} lon {lon}: {off} off");
                }
                // The foot is no farther than the point of the skin on the same radius.
                let radial = map(c + local * r);
                assert!(
                    foot.distance(p) <= radial.distance(p) + 1e-9,
                    "lat {lat} lon {lon} at {scale}: {} against {}",
                    foot.distance(p),
                    radial.distance(p)
                );
            }
        }
    }
}

#[test]
fn a_stretched_ball_holds_the_points_its_ball_did() {
    let (b, map, c, r) = stretched_ball();
    let mut seq = Seq(7);
    let mut cases = Vec::new();
    for _ in 0..20_000 {
        let l = seq.within([-1.2 * r; 3], [1.2 * r; 3]);
        if (l.length() - r).abs() < NEAR {
            continue;
        }
        cases.push((map(c + l), l.length() < r));
    }
    assert_holds(&b, &cases);
}

#[test]
fn a_point_of_the_boundary_is_on_it_and_a_step_off_is_in_or_out() {
    let (r, h) = (2.0, 10.0);
    let b = cylinder(&Frame::WORLD, r, h).unwrap();
    // On the side, on each end, on a rim, and where the side's seam meets a rim.
    let on = [
        Point3::new(r * 0.6, r * 0.8, 4.0),
        Point3::new(0.5, -0.3, 0.0),
        Point3::new(-1.1, 0.2, h),
        Point3::new(0.0, r, h),
        Point3::new(r, 0.0, 0.0),
        Point3::new(r, 0.0, 3.0),
    ];
    for (p, side) in on.iter().zip(b.classify(&on).unwrap()) {
        assert_eq!(side, Side::On, "{p:?}");
    }
    let off = [
        [r * 0.6 - 1e-3, r * 0.8, 4.0],
        [r * 0.6 + 1e-3, r * 0.8, 4.0],
        [0.5, -0.3, 1e-3],
        [0.5, -0.3, -1e-3],
    ];
    assert_eq!(b.classify(&off).unwrap(), vec![Side::In, Side::Out, Side::In, Side::Out]);

    let b = block(&Frame::WORLD, [4.0, 3.0, 2.0]).unwrap();
    // On a face, along an edge and at a corner.
    let on = [
        Point3::new(1.0, 1.0, 2.0),
        Point3::new(4.0, 1.0, 2.0),
        Point3::new(4.0, 3.0, 2.0),
        Point3::new(0.0, 0.0, 0.0),
    ];
    for (p, side) in on.iter().zip(b.classify(&on).unwrap()) {
        assert_eq!(side, Side::On, "{p:?}");
    }
    assert_eq!(b.classify(&[[3.999, 2.999, 1.999], [4.001, 3.001, 2.001]]).unwrap(), vec![Side::In, Side::Out]);
}
