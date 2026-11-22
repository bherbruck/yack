// yack: The point of a solid's boundary nearest a point, against closed forms: on a face, on an edge,
// Adapted from springrep@a03e8a1 tests/nearest.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: crate paths springrep:: -> yack_springrep::; this header added.
//! The point of a solid's boundary nearest a point, against closed forms: on a face, on an edge,
//! at a corner, and from inside the solid as from outside.

use yack_springrep::{
    edit::{Affine, affine},
    math::{Frame, Vec3},
    primitive::{block, cone, cylinder, sphere, torus},
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

    fn within(&mut self, lo: [f64; 3], hi: [f64; 3]) -> Vec3 {
        Vec3::new(
            lo[0] + (hi[0] - lo[0]) * self.next(),
            lo[1] + (hi[1] - lo[1]) * self.next(),
            lo[2] + (hi[2] - lo[2]) * self.next(),
        )
    }
}

fn assert_near(got: f64, want: f64, what: &str) {
    assert!(
        (got - want).abs() <= 1e-9 * (1.0 + want),
        "{what}: {got} against {want}"
    );
}

#[test]
fn the_nearest_point_of_a_ball_lies_along_the_radius() {
    let c = Vec3::new(1.0, -2.0, 0.5);
    let b = sphere(&Frame::new(c, Vec3::Z, Vec3::X).unwrap(), 2.0, "sph").unwrap();
    let boundary = yack_springrep::mesh::nearest_boundary(&b).unwrap();
    let mut seq = Seq(1);
    for _ in 0..300 {
        let p = c + seq.within([-4.0; 3], [4.0; 3]);
        let from_centre = (p - c).norm();
        if from_centre < 1e-3 {
            continue;
        }
        let got = boundary.to(p).unwrap();
        assert_near(got.distance, (from_centre - 2.0).abs(), "distance");
        let foot = c + (p - c) * (2.0 / from_centre);
        assert!(got.on_b.dist(foot) < 1e-7, "{p:?}: {:?}", got.on_b);
    }
    // Along the ball's own axis, through its poles.
    for z in [3.0, 2.5, 1.0, -1.0, -3.5] {
        let got = boundary.to(c + Vec3::Z * z).unwrap();
        assert_near(got.distance, (z.abs() - 2.0).abs(), "on the axis");
    }
}

#[test]
fn the_nearest_point_of_a_block_is_on_a_face_an_edge_or_a_corner() {
    let size = [4.0, 3.0, 2.0];
    let b = block(&Frame::WORLD, Vec3::from_array(size), "blk").unwrap();
    let boundary = yack_springrep::mesh::nearest_boundary(&b).unwrap();
    let mut seq = Seq(2);
    for _ in 0..300 {
        let p = seq.within([-2.0; 3], [6.0, 5.0, 4.0]);
        let q = p.to_array();
        let inside = (0..3).all(|k| q[k] > 0.0 && q[k] < size[k]);
        let want = if inside {
            // The nearest wall.
            (0..3)
                .map(|k| q[k].min(size[k] - q[k]))
                .fold(f64::INFINITY, f64::min)
        } else {
            // Clamped onto the block: a face, an edge or a corner, as the point lies.
            let clamped = [0, 1, 2].map(|k| q[k].clamp(0.0, size[k]));
            p.dist(Vec3::from_array(clamped))
        };
        let got = boundary.to(p).unwrap();
        assert_near(got.distance, want, "distance");
        assert_near(got.on_b.dist(p), want, "the point given");
    }
}

#[test]
fn the_nearest_point_of_a_cylinder_is_on_its_side_an_end_or_a_rim() {
    let (r, h) = (1.5, 4.0);
    let b = cylinder(&Frame::WORLD, r, h, "cyl").unwrap();
    let boundary = yack_springrep::mesh::nearest_boundary(&b).unwrap();
    let mut seq = Seq(3);
    for _ in 0..300 {
        let p = seq.within([-3.0, -3.0, -2.0], [3.0, 3.0, 6.0]);
        let rho = p.x.hypot(p.y);
        let (dr, dz) = (rho - r, (-p.z).max(p.z - h));
        let want = if dr <= 0.0 && dz <= 0.0 {
            (-dr).min(-dz)
        } else {
            dr.max(0.0).hypot(dz.max(0.0))
        };
        assert_near(boundary.to(p).unwrap().distance, want, "distance");
    }
}

#[test]
fn the_nearest_point_of_a_torus_is_on_its_tube() {
    let (major, minor) = (3.0, 1.0);
    let b = torus(&Frame::WORLD, major, minor, "tor").unwrap();
    let boundary = yack_springrep::mesh::nearest_boundary(&b).unwrap();
    let mut seq = Seq(4);
    for _ in 0..300 {
        let p = seq.within([-5.0, -5.0, -2.0], [5.0, 5.0, 2.0]);
        let from_tube = (p.x.hypot(p.y) - major).hypot(p.z);
        let got = boundary.to(p).unwrap();
        assert_near(got.distance, (from_tube - minor).abs(), "distance");
    }
}

#[test]
fn the_nearest_point_of_a_cone_is_on_its_side_its_base_or_its_rim() {
    let (r, h) = (2.0, 3.0);
    let b = cone(&Frame::WORLD, r, 0.0, h, "con").unwrap();
    let boundary = yack_springrep::mesh::nearest_boundary(&b).unwrap();
    // In the meridian plane the cone is the triangle (0, 0), (r, 0), (0, h) turned about the
    // axis: the distance to its boundary is the distance to that triangle's two outer sides.
    let to_segment = |x: f64, y: f64, a: (f64, f64), b: (f64, f64)| {
        let (ux, uy) = (b.0 - a.0, b.1 - a.1);
        let t = (((x - a.0) * ux + (y - a.1) * uy) / (ux * ux + uy * uy)).clamp(0.0, 1.0);
        (x - a.0 - t * ux).hypot(y - a.1 - t * uy)
    };
    let mut seq = Seq(5);
    for _ in 0..300 {
        let p = seq.within([-3.0, -3.0, -1.0], [3.0, 3.0, 4.0]);
        let rho = p.x.hypot(p.y);
        let want = to_segment(rho, p.z, (0.0, 0.0), (r, 0.0)).min(to_segment(
            rho,
            p.z,
            (r, 0.0),
            (0.0, h),
        ));
        assert_near(boundary.to(p).unwrap().distance, want, "distance");
    }
}

#[test]
fn the_nearest_point_of_a_stretched_ball_is_found_at_its_poles_as_elsewhere() {
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
    let b = affine(&sphere(&f, r, "sph").unwrap(), &map).unwrap();
    let boundary = yack_springrep::mesh::nearest_boundary(&b).unwrap();
    // A point of the skin is its own nearest point, at the poles of the ball it came from too.
    for lat in [0.0f64, 60.0, 85.0, 89.0, 90.0, -89.0, -90.0] {
        for lon in [10.0f64, 130.0, 250.0] {
            let (la, lo) = (lat.to_radians(), lon.to_radians());
            let local = Vec3::new(la.cos() * lo.cos(), la.cos() * lo.sin(), la.sin());
            let on = map.point(c + local * r);
            let got = boundary.to(on).unwrap();
            assert!(got.distance < 1e-9, "lat {lat} lon {lon}: {}", got.distance);
        }
    }
}

#[test]
fn the_nearest_point_of_a_solid_to_a_point_it_holds_is_the_point() {
    let b = block(&Frame::WORLD, Vec3::new(4.0, 3.0, 2.0), "blk").unwrap();
    let solid = yack_springrep::mesh::nearest_solid(&b).unwrap();
    let inside = Vec3::new(1.0, 2.0, 0.5);
    let got = solid.to(inside).unwrap();
    assert!(got.distance == 0.0 && got.on_b.dist(inside) < 1e-12);
    // Outside, the solid's nearest point is its boundary's.
    let out = Vec3::new(6.0, 1.0, 1.0);
    let got = solid.to(out).unwrap();
    assert_near(got.distance, 2.0, "outside");
    assert!(got.on_b.dist(Vec3::new(4.0, 1.0, 1.0)) < 1e-9);
    // The boundary alone is half a millimetre from the point inside.
    let got = yack_springrep::mesh::nearest_boundary(&b).unwrap().to(inside).unwrap();
    assert_near(got.distance, 0.5, "to the boundary");
}

#[test]
fn the_nearest_point_of_a_rim_is_found_either_side_of_where_the_rim_closes() {
    // A rim is one edge round a whole circle, its two ends one point. Points above and outside
    // the cylinder a hair to either side of that point are nearest the rim just past it.
    let (r, h) = (2.0, 5.0);
    let b = cylinder(&Frame::WORLD, r, h, "cyl").unwrap();
    for nearest in [yack_springrep::mesh::nearest_boundary(&b).unwrap(), yack_springrep::mesh::nearest_solid(&b).unwrap()] {
        for y in [0.021, -0.021, 0.3, -0.3, 1e-4, -1e-4, 0.0] {
            let q = Vec3::new(3.2, y, 7.2);
            let rho = q.x.hypot(q.y);
            let got = nearest.to(q).unwrap();
            assert_near(got.distance, (rho - r).hypot(q.z - h), "distance");
            let foot = Vec3::new(r * q.x / rho, r * q.y / rho, h);
            assert!(got.on_b.dist(foot) < 1e-7, "y {y}: {:?}", got.on_b);
        }
    }
}

#[test]
fn the_nearest_point_of_one_face_ignores_the_rest_of_the_solid() {
    // A 20 × 20 × 10 block with a corner at the origin. From (25, 10, 12), 5 past its side and 2
    // above it,
    // the top face's nearest point is on its edge, √29 away; the bottom face's is √(25 + 144).
    use yack_springrep::distance::Selection;
    let b = block(&Frame::WORLD, Vec3::new(20.0, 20.0, 10.0), "b").unwrap();
    let face = |z: f64| {
        let at = |f: &yack_springrep::topo::FaceId| {
            (yack_springrep::measure::face_centroid(&b, *f).z - z).abs() < 1e-9
        };
        b.face_ids().find(at).unwrap()
    };
    let p = yack_springrep::math::Point3::new(25.0, 10.0, 12.0);
    for (z, want) in [(10.0, 29f64.sqrt()), (0.0, 169f64.sqrt())] {
        let nearest = yack_springrep::mesh::nearest_of(&b, &Selection::face(face(z))).unwrap();
        let found = nearest.to(p).unwrap();
        assert_near(found.distance, want, "one face");
        assert_near(found.on_b.z, z, "on that face");
    }
}
