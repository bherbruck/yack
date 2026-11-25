// yack: rays's tests that build their operands with springrep modules above yack-locate.
// Adapted from springrep@a03e8a1 src/rays (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved out of the crate's tests in M3 (W4-W7); paths and imports only. Moved to crates/yack/tests at W11 (springrep deleted): paths to the crates the items live in.
/// Moved from rays.rs's tests (M3 W5): they build with modules above yack-locate.
#[cfg(test)]
mod rays_moved {
    use yack_boolean as boolean;
    use yack_math::kernel::Frame;
    use yack_math::kernel::Point3;
    use yack_math::kernel::Vec2;
    use yack_math::kernel::Vec3;
    use yack_profile::NamedSegment;
    use yack_profile::Profile;
    use yack_profile::ProfileLoop;
    use yack_profile::Segment;
    use yack_locate::rays::Side;
    use yack_build::sweep::Helix;
    use yack_build::sweep::extrude;
    use yack_build::sweep::helix;
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
        let cone = yack_build::primitive::cone(&Frame::new(o, z, x).unwrap(), r0, r1, h, "c").unwrap();
        let m = [
            [-0.4538971246554957, 0.15252576458329087, 0.9555666727243001, 41.6656262880843],
            [-0.26697402297825473, -0.5673293657946105, -1.4703977293152846, 16.072466163157856],
            [0.05877765464764511, -1.3990214079245642, 0.7004527186325231, -15.994874937232893],
        ];
        let b = yack_edit::glue::affine(&cone, m).unwrap();
        let rays = yack_mesh::rays(&b).unwrap();
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
        let rays = yack_mesh::rays(&threaded).unwrap();
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
