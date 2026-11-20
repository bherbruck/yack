// yack: Points against solids with closed forms: which the solid holds, wherever the classifier's
// Adapted from springrep@a03e8a1 tests/classify.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added.
//! Points against solids with closed forms: which the solid holds, wherever the classifier's
//! rays meet its edges, corners, seams and poles.

use yack_springrep::{
    boolean::difference,
    edit::{Affine, affine},
    math::{Frame, Vec3},
    primitive::{block, cylinder, sphere, torus},
    rays::{Rays, Side},
    topo::Brep,
};

/// A fixed sequence of numbers in [0, 1): the same points every run.
struct Seq(u64);

impl Seq {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
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

#[test]
fn points_about_a_cylinder_are_held_where_the_closed_form_says() {
    let (r, h) = (2.0, 10.0);
    let b = cylinder(&Frame::WORLD, r, h, "cyl").unwrap();
    let holds = Rays::new(&b).unwrap();
    let mut seq = Seq(11);
    for _ in 0..20_000 {
        let p = seq.within([-3.0, -3.0, -2.0], [3.0, 3.0, 12.0]);
        let rho = p.x.hypot(p.y);
        if (rho - r).abs() < NEAR || p.z.abs() < NEAR || (p.z - h).abs() < NEAR {
            continue;
        }
        let inside = rho < r && p.z > 0.0 && p.z < h;
        assert_eq!(holds.holds(p), Some(inside), "{p:?}");
    }
}

#[test]
fn points_about_a_block_are_held_where_its_slabs_say() {
    let b = block(&Frame::WORLD, Vec3::new(4.0, 3.0, 2.0), "blk").unwrap();
    let holds = Rays::new(&b).unwrap();
    let size = [4.0, 3.0, 2.0];
    let mut seq = Seq(5);
    for _ in 0..20_000 {
        let p = seq.within([-1.0; 3], [5.0, 4.0, 3.0]);
        let q = p.to_array();
        if (0..3).any(|k| q[k].abs() < NEAR || (q[k] - size[k]).abs() < NEAR) {
            continue;
        }
        let inside = (0..3).all(|k| q[k] > 0.0 && q[k] < size[k]);
        assert_eq!(holds.holds(p), Some(inside), "{p:?}");
    }
}

#[test]
fn a_torus_holds_its_tube_and_neither_its_hole_nor_a_void_cut_in_a_block() {
    let (major, minor) = (3.0, 1.0);
    let ring = torus(&Frame::WORLD, major, minor, "tor").unwrap();
    let holds = Rays::new(&ring).unwrap();
    let mut seq = Seq(3);
    for _ in 0..5_000 {
        let p = seq.within([-4.5, -4.5, -1.5], [4.5, 4.5, 1.5]);
        let from_tube = (p.x.hypot(p.y) - major).hypot(p.z);
        if (from_tube - minor).abs() < NEAR {
            continue;
        }
        assert_eq!(holds.holds(p), Some(from_tube < minor), "{p:?}");
    }

    let outer = block(&Frame::WORLD, Vec3::new(6.0, 6.0, 6.0), "blk").unwrap();
    let c = Vec3::new(3.0, 3.0, 3.0);
    let centre = Frame::new(c, Vec3::Z, Vec3::X).unwrap();
    let hollow = difference(&outer, &sphere(&centre, 1.5, "sph").unwrap()).unwrap();
    let holds = Rays::new(&hollow).unwrap();
    for _ in 0..5_000 {
        let p = seq.within([-1.0; 3], [7.0; 3]);
        let q = p.to_array();
        let from_void = (p - c).norm();
        let on_wall = (0..3).any(|k| q[k].abs() < NEAR || (q[k] - 6.0).abs() < NEAR);
        if (from_void - 1.5).abs() < NEAR || on_wall {
            continue;
        }
        let in_block = (0..3).all(|k| q[k] > 0.0 && q[k] < 6.0);
        assert_eq!(holds.holds(p), Some(in_block && from_void > 1.5), "{p:?}");
    }
}

#[test]
fn an_arena_with_no_solid_holds_no_point() {
    let empty = Brep::default();
    assert_eq!(Rays::new(&empty).unwrap().holds(Vec3::ZERO), Some(false));
}

/// A ball stretched and sheared, as the conformance case that found this: its skin is a rational
/// spline surface with a pole at each end, a whole row of control points at one point.
fn stretched_ball() -> (Brep, Affine, Vec3, f64) {
    let c = Vec3::new(-2.919, -1.302, -1.162);
    let r = 1.815;
    let map = Affine {
        m: [
            [0.1028, -0.9280, 0.5227],
            [-1.0971, 0.5789, 0.4802],
            [1.2920, 0.5654, 0.3662],
        ],
        t: Vec3::new(20.147, -35.624, 7.546),
    };
    let f = Frame::new(c, Vec3::Z, Vec3::X).unwrap();
    (
        affine(&sphere(&f, r, "sph").unwrap(), &map).unwrap(),
        map,
        c,
        r,
    )
}

#[test]
fn a_point_near_a_stretched_balls_pole_projects_onto_its_skin_there() {
    let (b, map, c, r) = stretched_ball();
    let face = b.face_ids().next().unwrap();
    let s = &b.face(face).surface;
    for lat in [80.0f64, 85.0, 88.0, 89.5, 89.99, -85.0, -89.5] {
        for lon in [10.0f64, 100.0, 200.0, 300.0] {
            let (la, lo) = (lat.to_radians(), lon.to_radians());
            let local = Vec3::new(la.cos() * lo.cos(), la.cos() * lo.sin(), la.sin());
            // On the skin, and a way off it along the ball's radius, inside and out.
            for scale in [1.0, 0.97, 1.03] {
                let p = map.point(c + local * (r * scale));
                let uv = s.project(p);
                let foot = s.eval(uv.x, uv.y);
                if scale == 1.0 {
                    let off = foot.dist(p);
                    assert!(off < 1e-9, "lat {lat} lon {lon}: {off} off");
                }
                // The foot is no farther than the point of the skin on the same radius.
                let radial = map.point(c + local * r);
                assert!(
                    foot.dist(p) <= radial.dist(p) + 1e-9,
                    "lat {lat} lon {lon} at {scale}: {} against {}",
                    foot.dist(p),
                    radial.dist(p)
                );
            }
        }
    }
}

#[test]
fn a_stretched_ball_holds_the_points_its_ball_did() {
    let (b, map, c, r) = stretched_ball();
    let holds = Rays::new(&b).unwrap();
    let mut seq = Seq(7);
    for _ in 0..20_000 {
        let l = seq.within([-1.2 * r; 3], [1.2 * r; 3]);
        if (l.norm() - r).abs() < NEAR {
            continue;
        }
        let p = map.point(c + l);
        assert_eq!(holds.holds(p), Some(l.norm() < r), "{p:?}");
    }
}

#[test]
fn a_point_of_the_boundary_is_on_it_and_a_step_off_is_in_or_out() {
    let (r, h) = (2.0, 10.0);
    let b = cylinder(&Frame::WORLD, r, h, "cyl").unwrap();
    let rays = Rays::new(&b).unwrap();
    // On the side, on each end, on a rim, and where the side's seam meets a rim.
    for p in [
        Vec3::new(r * 0.6, r * 0.8, 4.0),
        Vec3::new(0.5, -0.3, 0.0),
        Vec3::new(-1.1, 0.2, h),
        Vec3::new(0.0, r, h),
        Vec3::new(r, 0.0, 0.0),
        Vec3::new(r, 0.0, 3.0),
    ] {
        assert_eq!(rays.side(p), Side::On, "{p:?}");
    }
    assert_eq!(
        rays.side(Vec3::new(r * 0.6 - 1e-3, r * 0.8, 4.0)),
        Side::Inside
    );
    assert_eq!(
        rays.side(Vec3::new(r * 0.6 + 1e-3, r * 0.8, 4.0)),
        Side::Outside
    );
    assert_eq!(rays.side(Vec3::new(0.5, -0.3, 1e-3)), Side::Inside);
    assert_eq!(rays.side(Vec3::new(0.5, -0.3, -1e-3)), Side::Outside);

    let b = block(&Frame::WORLD, Vec3::new(4.0, 3.0, 2.0), "blk").unwrap();
    let rays = Rays::new(&b).unwrap();
    // On a face, along an edge and at a corner.
    for p in [
        Vec3::new(1.0, 1.0, 2.0),
        Vec3::new(4.0, 1.0, 2.0),
        Vec3::new(4.0, 3.0, 2.0),
        Vec3::new(0.0, 0.0, 0.0),
    ] {
        assert_eq!(rays.side(p), Side::On, "{p:?}");
    }
    assert_eq!(rays.side(Vec3::new(3.999, 2.999, 1.999)), Side::Inside);
    assert_eq!(rays.side(Vec3::new(4.001, 3.001, 2.001)), Side::Outside);
}
