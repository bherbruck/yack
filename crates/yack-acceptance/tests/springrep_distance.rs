// yack: closest distances between solids and from points, against closed forms.
// Adapted from springrep@a03e8a1 tests/distance.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); the revolved half-disc ball,
// the rect/circle extrusions and the revolved torus are made with make::sphere / block / cylinder /
// torus on the same frames (springrep circles take a diameter, so radii are halved); Selection /
// Nearest::boundary become Solid::closest_pair, Solid::nearest_face and FaceRef::closest (a vertex
// to a face is the vertex's point to that face's closest point); the outline spline through 24
// points is Loop::through, its curve evaluated with EdgeRef::range / point; measure::face_at is
// Solid::nearest_face; face-to-face distance is an API gap.

use std::f64::consts::TAU;

use yack::prelude::*;
use yack::VertexId;

fn ball(c: [f64; 3], r: f64) -> Solid {
    sphere(&Frame::at(c), r).unwrap()
}

fn cuboid(lo: [f64; 3], hi: [f64; 3]) -> Solid {
    block(&Frame::at(lo), [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]).unwrap()
}

/// `(distance, on a, on b)`.
fn bodies(a: &Solid, b: &Solid) -> (f64, Point3, Point3) {
    let c = a.closest_pair(b).unwrap();
    (c.distance, c.on_self, c.on_other)
}

/// The distance, to rounding, and that both points lie that far apart.
fn assert_distance(c: (f64, Point3, Point3), want: f64) {
    let (distance, on_a, on_b) = c;
    assert!((distance - want).abs() <= 1e-9 * (1.0 + want), "{c:?} vs {want}");
    assert!((on_a.distance(on_b) - distance).abs() <= 1e-12 * (1.0 + want), "{c:?}");
}

/// Balls a few micrometres across that overlap, centres 3.4 µm apart and radii 4.1 and 2.7 µm: they
/// share points, so they are no distance apart, to rounding, however small the parts.
#[test]
fn overlapping_balls_micrometres_across_are_no_distance_apart() {
    let a = ball([0.0; 3], 0.004054250852155359);
    let b = ball([0.00338806114604008, 0.0, 0.0], 0.0026689525243786373);
    let d = bodies(&a, &b).0;
    assert!(d <= 1e-15, "{d}");
}

#[test]
fn two_balls_are_their_centres_apart_less_both_radii() {
    let (c1, c2) = (Point3::new(1.0, -2.0, 0.5), Point3::new(9.0, 3.0, -1.5));
    let c = bodies(&ball(c1.to_array(), 3.0), &ball(c2.to_array(), 2.0));
    assert_distance(c, c1.distance(c2) - 5.0);
    // Each point lies on its ball, on the line through the centres.
    let (_, on_a, on_b) = c;
    assert!((on_a.distance(c1) - 3.0).abs() < 1e-9 && (on_b.distance(c2) - 2.0).abs() < 1e-9);
}

#[test]
fn a_ball_by_a_box_meets_its_face_edge_or_corner() {
    // The box is x 0..10, y 0..6, z 0..3; a unit ball off each kind of place.
    let b = cuboid([0.0; 3], [10.0, 6.0, 3.0]);
    // Off the face x = 10.
    assert_distance(bodies(&ball([13.0, 3.0, 1.5], 1.0), &b), 2.0);
    // Past the edge x = 10, y = 6, where the face planes' nearest points lie off their faces.
    let edge = (2.0f64 * 2.0 + 2.0 * 2.0).sqrt() - 1.0;
    assert_distance(bodies(&ball([12.0, 8.0, 1.5], 1.0), &b), edge);
    // Past the corner (10, 6, 3).
    let corner = 12.0f64.sqrt() - 1.0;
    assert_distance(bodies(&ball([12.0, 8.0, 5.0], 1.0), &b), corner);
}

#[test]
fn boxes_set_corner_to_corner_meet_edge_to_edge() {
    // Unit cubes at (0..1)² and (2..3)² in plan, the same height: their vertical edges at (1, 1)
    // and (2, 2) are √2 apart.
    let c = bodies(&cuboid([0.0; 3], [1.0; 3]), &cuboid([2.0, 2.0, 0.0], [3.0, 3.0, 1.0]));
    assert_distance(c, 2.0f64.sqrt());
}

#[test]
fn crossed_rods_are_their_axes_apart_less_both_radii() {
    // A rod of radius 1 along z, and one of radius 1.5 along x whose axis passes 4 from it.
    let a = cylinder(&Frame::at([0.0, 0.0, -5.0]), 2.0 / 2.0, 10.0).unwrap();
    // springrep Frame::new(origin, z = X, x = Y): y = z × x = Z.
    let along_x = Frame::new([-5.0, 4.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]).unwrap();
    let b = cylinder(&along_x, 3.0 / 2.0, 10.0).unwrap();
    assert_distance(bodies(&a, &b), 4.0 - 1.0 - 1.5);
}

#[test]
fn a_ball_on_a_torus_axis_is_nearest_along_a_whole_ring() {
    // Tube radius 2 at R = 10 round z; a unit ball at height 6 on the axis is nearest every
    // point of the ring where the tube faces it: √(R² + h²) − a − r.
    let t = torus(&Frame::WORLD, 10.0, 4.0 / 2.0).unwrap();
    let c = bodies(&ball([0.0, 0.0, 6.0], 1.0), &t);
    assert_distance(c, (100.0f64 + 36.0).sqrt() - 3.0);
}

#[test]
fn a_ball_inside_a_box_or_through_it_is_no_distance_away() {
    let b = cuboid([0.0; 3], [10.0, 6.0, 3.0]);
    // Inside, touching nothing: its boundary is a whole unit from the box's.
    assert_eq!(bodies(&ball([5.0, 3.0, 1.5], 1.0), &b).0, 0.0);
    // Through the face x = 10.
    assert_distance(bodies(&ball([10.5, 3.0, 1.5], 1.0), &b), 0.0);
}

#[test]
fn a_rod_inside_one_a_hair_wider_is_no_distance_away() {
    // The inner rod's side is 3.3 µm inside the outer's, far closer than the meshes' chords, so
    // most of its boundary's points sit within a chord of the outer mesh; its ends do not.
    let outer = cylinder(&Frame::at([0.0, 0.0, 1.28]), 6.6391 / 2.0, 9.17).unwrap();
    let inner = cylinder(&Frame::at([0.0, 0.0, 1.38]), 6.6390934 / 2.0, 5.64).unwrap();
    assert_distance(bodies(&outer, &inner), 0.0);
    assert_distance(bodies(&inner, &outer), 0.0);
}

#[test]
fn faces_edges_and_vertices_measure_as_themselves() {
    let b = cuboid([0.0; 3], [10.0, 6.0, 3.0]);
    let top = b.face_at([5.0, 3.0, 3.0]).unwrap();
    // API GAP: no face-to-face distance (springrep Selection::face on both sides), so "the box's
    // floor and roof are its height apart, though they bound one solid" (3.0) is dropped.
    // A corner of a second box set 1 above and 2 beyond, to the first's roof: its edge (10, 6) is
    // the nearest part of the roof.
    let other = cuboid([12.0, 7.0, 4.0], [14.0, 9.0, 5.0]);
    let at = Point3::new(12.0, 7.0, 4.0);
    let d = |v: VertexId| other.vertex(v).unwrap().point().distance(at);
    let v = other.vertices().into_iter().min_by(|&x, &y| d(x).total_cmp(&d(y))).unwrap();
    let corner = other.vertex(v).unwrap().point();
    let on_roof = b.face(top).unwrap().closest(corner).unwrap();
    assert_distance((corner.distance(on_roof), corner, on_roof), (4.0f64 + 1.0 + 1.0).sqrt());
}

#[test]
fn a_ball_beside_a_spline_extrusion_is_the_profile_curves_distance_away() {
    // A disc whose outline is a periodic spline, 10 high, and a unit ball at mid-height beside it:
    // the nearest point lies on the side at that height, so the distance is the plan distance
    // from the ball's centre to the outline less the radius. The outline's own distance comes
    // from sampling its curve densely and refining the nearest sample by golden sections, which
    // shares nothing with the search.
    let pts: Vec<[f64; 2]> = (0..24)
        .map(|k| {
            let a = TAU * k as f64 / 24.0;
            [30.0 * a.cos(), 20.0 * a.sin() + 3.0 * (3.0 * a).sin()]
        })
        .collect();
    let p = Profile::new(Loop::through(&pts).unwrap(), vec![]).unwrap();
    let disc = extrude(&Frame::WORLD, &p, 10.0).unwrap();
    let centre = Point3::new(33.0, 9.0, 5.0);
    let outline =
        disc.edges_where(|e| e.midpoint().z.abs() < 1e-9 && !matches!(e.kind(), CurveKind::Line { .. })).unwrap()[0];
    let c = disc.edge(outline).unwrap();
    let [t0, t1] = c.range();
    let plan = |t: f64| {
        let q = c.point(t);
        (q.x - centre.x).hypot(q.y - centre.y)
    };
    let n = 200_000;
    let at = |i: usize| t0 + (t1 - t0) * i as f64 / n as f64;
    let k = (0..=n).min_by(|&i, &j| plan(at(i)).total_cmp(&plan(at(j)))).unwrap();
    let (mut lo, mut hi) = (at(k.saturating_sub(1)), at((k + 1).min(n)));
    let g = 0.5 * (5.0f64.sqrt() - 1.0);
    for _ in 0..200 {
        let (x1, x2) = (hi - g * (hi - lo), lo + g * (hi - lo));
        if plan(x1) < plan(x2) {
            hi = x2;
        } else {
            lo = x1;
        }
    }
    let want = plan(0.5 * (lo + hi)) - 1.0;
    assert_distance(bodies(&ball(centre.to_array(), 1.0), &disc), want);
}

/// A plate 30 square and 2 thick with a hole of radius 10 through its middle.
fn drilled_plate() -> Solid {
    let plate = block(&Frame::WORLD, [30.0, 30.0, 2.0]).unwrap();
    plate.difference(&cylinder(&Frame::at([15.0, 15.0, -1.0]), 10.0, 4.0).unwrap()).unwrap()
}

#[test]
fn a_point_just_inside_a_holes_rim_is_as_far_as_the_rim_not_on_the_face() {
    // In the plane of the plate's top, a few micrometres off the rim either way: inside the hole
    // the top face is not there and the rim is the nearest thing, and on the face the distance is
    // nothing. A chord of the rim's polygon cuts 12 µm inside the circle at this radius.
    let plate = drilled_plate();
    for (off, want) in [(-0.005, 0.005), (-0.001, 0.001), (0.003, 0.0)] {
        for turn in [0.1f64, 1.3, 2.9, 4.4] {
            let r = 10.0 + off;
            let p = Point3::new(15.0 + r * turn.cos(), 15.0 + r * turn.sin(), 2.0);
            let got = plate.nearest_face(p).unwrap().distance;
            assert!((got - want).abs() < 1e-7, "{off} at {turn}: {got} against {want}");
        }
    }
}

#[test]
fn a_dowel_in_a_hole_is_as_far_from_the_plate_as_its_clearance() {
    // A dowel through the hole with micrometres to spare all round, less than the chords of the
    // hole's mesh stand off its wall: the two solids share no point, and the gap is the clearance.
    let plate = drilled_plate();
    let at = Frame::at([15.0, 15.0, -3.0]);
    for clearance in [0.002, 0.02, 0.2] {
        let dowel = cylinder(&at, 10.0 - clearance, 8.0).unwrap();
        let d = plate.distance(&dowel).unwrap();
        assert!((d - clearance).abs() < 1e-7, "clearance {clearance}: {d}");
    }
}

#[test]
fn the_face_at_a_point_just_inside_a_holes_rim_is_the_holes_wall() {
    // In the plane of the plate's top, 5 µm inside the hole: the top face is not there, and the
    // face whose own region holds its nearest point is the hole's wall, 5 µm away.
    let plate = drilled_plate();
    for turn in [0.1f64, 1.3, 2.9, 4.4] {
        let r = 10.0 - 0.005;
        let p = Point3::new(15.0 + r * turn.cos(), 15.0 + r * turn.sin(), 2.0);
        let yack::NearestFace { face, distance: d } = plate.nearest_face(p).unwrap();
        assert!(matches!(plate.face(face).unwrap().kind(), SurfaceKind::Cylinder { .. }), "at {turn}");
        assert!((d - 0.005).abs() < 1e-9, "at {turn}: {d}");
        // And 5 µm outside it, on the plate: the top, at no distance.
        let r = 10.0 + 0.005;
        let p = Point3::new(15.0 + r * turn.cos(), 15.0 + r * turn.sin(), 2.0);
        let yack::NearestFace { face, distance: d } = plate.nearest_face(p).unwrap();
        assert!(matches!(plate.face(face).unwrap().kind(), SurfaceKind::Plane { .. }), "at {turn}");
        assert!(d < 1e-9, "at {turn}: {d}");
    }
}
