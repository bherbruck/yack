// yack: the point of a solid's boundary nearest a point, against closed forms.
// Adapted from springrep@a03e8a1 tests/nearest.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); Nearest::boundary(..).to(p) is
// Solid::nearest_face(p) (the distance) then FaceRef::closest(p) on that face (the point);
// Nearest::solid(..).to(p) is Solid::closest(&[p]) (insides counted), its distance read as
// |closest − p|; Nearest::of(Selection::face(f)) is FaceRef::closest; edit::affine is
// Solid::transform_affine(&Affine::from_rows(..)) and the map's own point images are computed from
// its rows.

use yack::prelude::*;

/// A fixed sequence of numbers in [0, 1): the same points every run.
struct Seq(u64);

impl Seq {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
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
    assert!((got - want).abs() <= 1e-9 * (1.0 + want), "{what}: {got} against {want}");
}

/// The boundary of `b` nearest `p`: `(distance, point)`.
fn boundary(b: &Solid, p: Point3) -> (f64, Point3) {
    let (face, distance) = b.nearest_face(p).unwrap();
    (distance, b.face(face).unwrap().closest(p).unwrap())
}

/// The solid `b`, insides counted, nearest `p`: `(distance, point)`.
fn solid(b: &Solid, p: Point3) -> (f64, Point3) {
    let on = b.closest(&[p]).unwrap()[0];
    (on.distance(p), on)
}

#[test]
fn the_nearest_point_of_a_ball_lies_along_the_radius() {
    let c = Point3::new(1.0, -2.0, 0.5);
    let b = sphere(&Frame::at(c), 2.0).unwrap();
    let mut seq = Seq(1);
    for _ in 0..300 {
        let p = c + seq.within([-4.0; 3], [4.0; 3]);
        let from_centre = (p - c).length();
        if from_centre < 1e-3 {
            continue;
        }
        let (distance, on_b) = boundary(&b, p);
        assert_near(distance, (from_centre - 2.0).abs(), "distance");
        let foot = c + (p - c) * (2.0 / from_centre);
        assert!(on_b.distance(foot) < 1e-7, "{p:?}: {on_b:?}");
    }
    // Along the ball's own axis, through its poles.
    for z in [3.0, 2.5, 1.0, -1.0, -3.5] {
        let p = c + Vec3::new(0.0, 0.0, 1.0) * z;
        assert_near(boundary(&b, p).0, (z.abs() - 2.0).abs(), "on the axis");
    }
}

#[test]
fn the_nearest_point_of_a_block_is_on_a_face_an_edge_or_a_corner() {
    let size = [4.0, 3.0, 2.0];
    let b = block(&Frame::WORLD, size).unwrap();
    let mut seq = Seq(2);
    for _ in 0..300 {
        let p = Point3::ORIGIN + seq.within([-2.0; 3], [6.0, 5.0, 4.0]);
        let q = p.to_array();
        let inside = (0..3).all(|k| q[k] > 0.0 && q[k] < size[k]);
        let want = if inside {
            // The nearest wall.
            (0..3).map(|k| q[k].min(size[k] - q[k])).fold(f64::INFINITY, f64::min)
        } else {
            // Clamped onto the block: a face, an edge or a corner, as the point lies.
            let clamped = [0, 1, 2].map(|k| q[k].clamp(0.0, size[k]));
            p.distance(Point3::from(clamped))
        };
        let (distance, on_b) = boundary(&b, p);
        assert_near(distance, want, "distance");
        assert_near(on_b.distance(p), want, "the point given");
    }
}

#[test]
fn the_nearest_point_of_a_cylinder_is_on_its_side_an_end_or_a_rim() {
    let (r, h) = (1.5, 4.0);
    let b = cylinder(&Frame::WORLD, r, h).unwrap();
    let mut seq = Seq(3);
    for _ in 0..300 {
        let p = Point3::ORIGIN + seq.within([-3.0, -3.0, -2.0], [3.0, 3.0, 6.0]);
        let rho = p.x.hypot(p.y);
        let (dr, dz) = (rho - r, (-p.z).max(p.z - h));
        let want = if dr <= 0.0 && dz <= 0.0 { (-dr).min(-dz) } else { dr.max(0.0).hypot(dz.max(0.0)) };
        assert_near(boundary(&b, p).0, want, "distance");
    }
}

#[test]
fn the_nearest_point_of_a_torus_is_on_its_tube() {
    let (major, minor) = (3.0, 1.0);
    let b = torus(&Frame::WORLD, major, minor).unwrap();
    let mut seq = Seq(4);
    for _ in 0..300 {
        let p = Point3::ORIGIN + seq.within([-5.0, -5.0, -2.0], [5.0, 5.0, 2.0]);
        let from_tube = (p.x.hypot(p.y) - major).hypot(p.z);
        assert_near(boundary(&b, p).0, (from_tube - minor).abs(), "distance");
    }
}

#[test]
fn the_nearest_point_of_a_cone_is_on_its_side_its_base_or_its_rim() {
    let (r, h) = (2.0, 3.0);
    let b = cone(&Frame::WORLD, r, 0.0, h).unwrap();
    // In the meridian plane the cone is the triangle (0, 0), (r, 0), (0, h) turned about the
    // axis: the distance to its boundary is the distance to that triangle's two outer sides.
    let to_segment = |x: f64, y: f64, a: (f64, f64), b: (f64, f64)| {
        let (ux, uy) = (b.0 - a.0, b.1 - a.1);
        let t = (((x - a.0) * ux + (y - a.1) * uy) / (ux * ux + uy * uy)).clamp(0.0, 1.0);
        (x - a.0 - t * ux).hypot(y - a.1 - t * uy)
    };
    let mut seq = Seq(5);
    for _ in 0..300 {
        let p = Point3::ORIGIN + seq.within([-3.0, -3.0, -1.0], [3.0, 3.0, 4.0]);
        let rho = p.x.hypot(p.y);
        let want = to_segment(rho, p.z, (0.0, 0.0), (r, 0.0)).min(to_segment(rho, p.z, (r, 0.0), (0.0, h)));
        assert_near(boundary(&b, p).0, want, "distance");
    }
}

#[test]
fn the_nearest_point_of_a_stretched_ball_is_found_at_its_poles_as_elsewhere() {
    let c = Point3::new(-2.919, -1.302, -1.162);
    let r = 1.815;
    let m = [[0.1028, -0.9280, 0.5227], [-1.0971, 0.5789, 0.4802], [1.2920, 0.5654, 0.3662]];
    let t = [20.147, -35.624, 7.546];
    let map = Affine::from_rows([0, 1, 2].map(|i| [m[i][0], m[i][1], m[i][2], t[i]])).unwrap();
    let image = |p: Point3| {
        let [x, y, z] = [0, 1, 2].map(|i| m[i][0] * p.x + m[i][1] * p.y + m[i][2] * p.z + t[i]);
        Point3::new(x, y, z)
    };
    let b = sphere(&Frame::at(c), r).unwrap().transform_affine(&map).unwrap();
    // A point of the skin is its own nearest point, at the poles of the ball it came from too.
    for lat in [0.0f64, 60.0, 85.0, 89.0, 90.0, -89.0, -90.0] {
        for lon in [10.0f64, 130.0, 250.0] {
            let (la, lo) = (lat.to_radians(), lon.to_radians());
            let local = Vec3::new(la.cos() * lo.cos(), la.cos() * lo.sin(), la.sin());
            let on = image(c + local * r);
            let d = boundary(&b, on).0;
            assert!(d < 1e-9, "lat {lat} lon {lon}: {d}");
        }
    }
}

#[test]
fn the_nearest_point_of_a_solid_to_a_point_it_holds_is_the_point() {
    let b = block(&Frame::WORLD, [4.0, 3.0, 2.0]).unwrap();
    let inside = Point3::new(1.0, 2.0, 0.5);
    let (distance, on_b) = solid(&b, inside);
    assert!(distance == 0.0 && on_b.distance(inside) < 1e-12);
    // Outside, the solid's nearest point is its boundary's.
    let out = Point3::new(6.0, 1.0, 1.0);
    let (distance, on_b) = solid(&b, out);
    assert_near(distance, 2.0, "outside");
    assert!(on_b.distance(Point3::new(4.0, 1.0, 1.0)) < 1e-9);
    // The boundary alone is half a millimetre from the point inside.
    assert_near(boundary(&b, inside).0, 0.5, "to the boundary");
}

#[test]
fn the_nearest_point_of_a_rim_is_found_either_side_of_where_the_rim_closes() {
    // A rim is one edge round a whole circle, its two ends one point. Points above and outside
    // the cylinder a hair to either side of that point are nearest the rim just past it.
    let (r, h) = (2.0, 5.0);
    let b = cylinder(&Frame::WORLD, r, h).unwrap();
    for nearest in [boundary as fn(&Solid, Point3) -> (f64, Point3), solid] {
        for y in [0.021, -0.021, 0.3, -0.3, 1e-4, -1e-4, 0.0] {
            let q = Point3::new(3.2, y, 7.2);
            let rho = q.x.hypot(q.y);
            let (distance, on_b) = nearest(&b, q);
            assert_near(distance, (rho - r).hypot(q.z - h), "distance");
            let foot = Point3::new(r * q.x / rho, r * q.y / rho, h);
            assert!(on_b.distance(foot) < 1e-7, "y {y}: {on_b:?}");
        }
    }
}

#[test]
fn the_nearest_point_of_one_face_ignores_the_rest_of_the_solid() {
    // A 20 × 20 × 10 block with a corner at the origin. From (25, 10, 12), 5 past its side and 2
    // above it,
    // the top face's nearest point is on its edge, √29 away; the bottom face's is √(25 + 144).
    let b = block(&Frame::WORLD, [20.0, 20.0, 10.0]).unwrap();
    let face = |z: f64| b.faces_where(|f| (f.centroid().z - z).abs() < 1e-9).unwrap()[0];
    let p = Point3::new(25.0, 10.0, 12.0);
    for (z, want) in [(10.0, 29f64.sqrt()), (0.0, 169f64.sqrt())] {
        let on_b = b.face(face(z)).unwrap().closest(p).unwrap();
        assert_near(on_b.distance(p), want, "one face");
        assert_near(on_b.z, z, "on that face");
    }
}
