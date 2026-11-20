// yack: results whose pieces touch, or that touch themselves, at points or along curves: valid, closed-form, right Betti numbers.
// Adapted from springrep@a03e8a1 tests/touching.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); springrep `Frame::from_axis` written as
// yack `Frame::new(o, x, y, z)` with the same axes; the revolved ball and cone helpers are yack's `sphere` and
// `cone`; named faces and edges are picked by point (`face_at`/`edge_at`); a solid count is `betti()[0]`.

use std::f64::consts::PI;

use yack::prelude::*;
use yack::EdgeId;

/// The original's `MeshOptions { linear: 0.2, .. }`.
const COARSE: f64 = 0.2;

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// springrep's `Frame::from_axis(base, z)`: x is z × (X, or Y when z is near X), y is z × x.
fn axis_frame(base: [f64; 3], z: [f64; 3]) -> Frame {
    let hint = if z[0].abs() < 0.6 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let x = cross(z, hint);
    let n = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
    let x = [x[0] / n, x[1] / n, x[2] / n];
    Frame::new(base, x, cross(z, x), z).unwrap()
}

/// The box `lo`–`hi`.
fn bx(lo: [f64; 3], hi: [f64; 3]) -> Solid {
    block(&Frame::at(lo), [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]).unwrap()
}

/// A cylinder of radius `r` along `axis` from `base`, `len` long.
fn rod(base: [f64; 3], axis: [f64; 3], r: f64, len: f64) -> Solid {
    cylinder(&axis_frame(base, axis), r, len).unwrap()
}

fn ball(centre: [f64; 3], r: f64) -> Solid {
    sphere(&Frame::at(centre), r).unwrap()
}

/// A cone of base radius `r` and height `h` standing on its apex at `apex`, opening along `up`
/// (+1 or −1 along z).
fn cone_on_apex(apex: [f64; 3], up: f64, r: f64, h: f64) -> Solid {
    let frame = if up > 0.0 {
        Frame::at(apex)
    } else {
        Frame::new(apex, [1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]).unwrap()
    };
    cone(&frame, 0.0, r, h).unwrap()
}

fn cone_measures(r: f64, h: f64) -> (f64, f64) {
    (PI * r * r * h / 3.0, PI * r * r + PI * r * (r * r + h * h).sqrt())
}

/// One valid solid of the volume, area and Betti numbers given.
fn assert_touching(b: &Solid, volume: f64, area: f64, betti: [u64; 3]) {
    let issues = b.check();
    assert!(issues.is_empty(), "{issues:?}");
    assert_eq!(b.betti()[0], 1, "touching pieces are one solid");
    let (v, a) = (b.volume(), b.area());
    assert!((v - volume).abs() <= 1e-9 * volume, "volume {v} vs {volume}");
    assert!((a - area).abs() <= 1e-9 * area, "area {a} vs {area}");
    assert_eq!(b.betti(), betti);
    let m = b.mesh(COARSE).unwrap();
    // API GAP: springrep reports watertightness per solid of the set; yack's Mesh is one mesh of the
    // whole set, so it is checked as one.
    assert!(m.is_watertight());
}

/// The edges four faces use: where pieces meet along a curve.
fn four_face_edges(b: &Solid) -> Vec<EdgeId> {
    b.edges().into_iter().filter(|&e| b.edge(e).unwrap().uses() == 4).collect()
}

#[test]
fn blocks_touching_along_an_edge_unite_round_it() {
    // Two 10 mm cubes sharing one vertical edge: one piece, both cubes, the edge used by all four
    // of their faces that meet there.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([10.0, 10.0, 0.0], [20.0, 20.0, 10.0]);
    let u = a.union(&b).unwrap();
    assert_touching(&u, 2000.0, 1200.0, [1, 0, 0]);
    assert_eq!(four_face_edges(&u).len(), 1);
}

#[test]
fn blocks_touching_at_a_corner_unite_there() {
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([10.0, 10.0, 10.0], [20.0, 20.0, 20.0]);
    let u = a.union(&b).unwrap();
    assert_touching(&u, 2000.0, 1200.0, [1, 0, 0]);
    assert_eq!(
        u.vertices()
            .into_iter()
            .filter(|&v| {
                let p = u.vertex(v).unwrap().point();
                [p.x, p.y, p.z] == [10.0, 10.0, 10.0]
            })
            .count(),
        1
    );
}

#[test]
fn a_rod_lying_on_a_box_face_unites_along_the_line() {
    // The rod's side touches the face z = 10 along x = 5..15 at y = 5: both stay whole, and the
    // line is an edge.
    let a = bx([0.0, 0.0, 0.0], [20.0, 10.0, 10.0]);
    let c = rod([5.0, 5.0, 12.0], [1.0, 0.0, 0.0], 2.0, 10.0);
    let u = a.union(&c).unwrap();
    let rod_area = 2.0 * PI * 4.0 + 2.0 * PI * 2.0 * 10.0;
    assert_touching(&u, 2000.0 + PI * 4.0 * 10.0, 1000.0 + rod_area, [1, 0, 0]);
    assert_eq!(four_face_edges(&u).len(), 1);
}

#[test]
fn a_box_less_an_inscribed_cylinder_is_four_corners_in_a_ring() {
    // A 10 mm square prism less the cylinder touching all four sides: four corner pieces, each
    // meeting the next along a line, round the bore — one tunnel.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 8.0]);
    let c = rod([5.0, 5.0, -1.0], [0.0, 0.0, 1.0], 5.0, 10.0);
    let d = a.difference(&c).unwrap();
    let area = 2.0 * (100.0 - PI * 25.0) + 4.0 * 80.0 + 2.0 * PI * 5.0 * 8.0;
    assert_touching(&d, 800.0 - PI * 25.0 * 8.0, area, [1, 1, 0]);
    assert_eq!(four_face_edges(&d).len(), 4);
}

#[test]
fn a_block_less_a_bore_flush_to_its_top_touches_itself_along_a_line() {
    // A bore of radius 3 along x whose top line lies on the top face z = 8: one piece round the
    // bore, its material thinning to nothing along that line, still a tunnel.
    let a = bx([0.0, 0.0, 0.0], [12.0, 10.0, 8.0]);
    let c = rod([-1.0, 5.0, 5.0], [1.0, 0.0, 0.0], 3.0, 14.0);
    let d = a.difference(&c).unwrap();
    let area = 2.0 * (120.0 + 96.0 + 80.0) - 2.0 * PI * 9.0 + 2.0 * PI * 3.0 * 12.0;
    assert_touching(&d, 960.0 - PI * 9.0 * 12.0, area, [1, 1, 0]);
    assert_eq!(four_face_edges(&d).len(), 1);
}

#[test]
fn a_rod_inside_a_box_against_a_wall_leaves_a_void_touching_it() {
    // The rod lies inside, its side on the wall x = 0 along a line: the cavity is still closed off,
    // as the line is the solid's.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let c = rod([2.0, 5.0, 2.0], [0.0, 0.0, 1.0], 2.0, 6.0);
    let d = a.difference(&c).unwrap();
    let area = 600.0 + 2.0 * PI * 4.0 + 2.0 * PI * 2.0 * 6.0;
    assert_touching(&d, 1000.0 - PI * 4.0 * 6.0, area, [1, 0, 1]);
    assert_eq!(four_face_edges(&d).len(), 1);
}

#[test]
fn a_cube_less_a_ball_touching_its_six_faces_keeps_the_void() {
    // The ball touches each face at its centre: one piece whose cavity meets the outside at six
    // points, which are the solid's, so the cavity is still a void.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let s = ball([5.0, 5.0, 5.0], 5.0);
    let d = a.difference(&s).unwrap();
    assert_touching(&d, 1000.0 - 4.0 / 3.0 * PI * 125.0, 600.0 + 4.0 * PI * 25.0, [1, 0, 1]);
}

#[test]
fn a_touching_result_combined_with_itself_keeps_its_contact() {
    // Combining a solid with itself copies it: the copy's edge four faces use is paired again
    // from the faces round it.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([10.0, 10.0, 0.0], [20.0, 20.0, 10.0]);
    let u = a.union(&b).unwrap();
    for x in [u.intersection(&u).unwrap(), u.union(&u).unwrap()] {
        assert_touching(&x, 2000.0, 1200.0, [1, 0, 0]);
        assert_eq!(four_face_edges(&x).len(), 1);
    }
}

#[test]
fn balls_touching_at_a_point_unite_there() {
    // Each ball's surface meets the other only at (10, 0, 0), inside a face of each: that point is
    // a vertex both fans share.
    let a = ball([5.0, 0.0, 0.0], 5.0);
    let b = ball([15.0, 0.0, 0.0], 5.0);
    let u = a.union(&b).unwrap();
    let (v, s) = (4.0 / 3.0 * PI * 125.0, 4.0 * PI * 25.0);
    assert_touching(&u, 2.0 * v, 2.0 * s, [1, 0, 0]);
    let at = [10.0, 0.0, 0.0];
    assert_eq!(
        u.vertices()
            .into_iter()
            .filter(|&v| {
                let p = u.vertex(v).unwrap().point();
                ((p.x - at[0]).powi(2) + (p.y - at[1]).powi(2) + (p.z - at[2]).powi(2)).sqrt() < 1e-9
            })
            .count(),
        1
    );
}

#[test]
fn a_ball_on_a_box_face_unites_at_a_point() {
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let s = ball([5.0, 5.0, 13.0], 3.0);
    let u = a.union(&s).unwrap();
    let (v, area) = (4.0 / 3.0 * PI * 27.0, 4.0 * PI * 9.0);
    assert_touching(&u, 1000.0 + v, 600.0 + area, [1, 0, 0]);
}

#[test]
fn balls_touching_at_a_point_combine_as_inputs() {
    let a = ball([5.0, 0.0, 0.0], 5.0);
    let b = ball([15.0, 0.0, 0.0], 5.0);
    let u = a.union(&b).unwrap();
    let (v, s) = (4.0 / 3.0 * PI * 125.0, 4.0 * PI * 25.0);
    // Combined with itself, the contact is kept.
    for x in [u.intersection(&u).unwrap(), u.union(&u).unwrap()] {
        assert_touching(&x, 2.0 * v, 2.0 * s, [1, 0, 0]);
    }
    // A third ball touching the second at (20, 0, 0) joins the chain at a second point.
    let c = ball([25.0, 0.0, 0.0], 5.0);
    assert_touching(&u.union(&c).unwrap(), 3.0 * v, 3.0 * s, [1, 0, 0]);
    // A slab through the contact takes a cap of height 1 off each ball, parting them; a cap's
    // volume is π h² (3r − h) / 3.
    let slab = bx([9.0, -6.0, -6.0], [11.0, 6.0, 6.0]);
    let d = u.difference(&slab).unwrap();
    assert!(d.check().is_empty());
    assert_eq!(d.betti()[0], 2);
    assert_eq!(d.betti(), [2, 0, 0]);
    let cap = PI * (3.0 * 5.0 - 1.0) / 3.0;
    assert!((d.volume() - 2.0 * (v - cap)).abs() < 1e-9 * v);
}

#[test]
fn a_cone_standing_on_its_apex_unites_with_the_face_under_it() {
    let a = bx([-10.0, -10.0, -10.0], [10.0, 10.0, 0.0]);
    let c = cone_on_apex([0.0, 0.0, 0.0], 1.0, 3.0, 6.0);
    let u = a.union(&c).unwrap();
    let (v, s) = cone_measures(3.0, 6.0);
    assert_touching(&u, 4000.0 + v, 1600.0 + s, [1, 0, 0]);
}

#[test]
fn cones_touching_apex_to_apex_unite_there() {
    let a = cone_on_apex([0.0, 0.0, 0.0], 1.0, 3.0, 6.0);
    let b = cone_on_apex([0.0, 0.0, 0.0], -1.0, 2.0, 5.0);
    let u = a.union(&b).unwrap();
    let (va, sa) = cone_measures(3.0, 6.0);
    let (vb, sb) = cone_measures(2.0, 5.0);
    assert_touching(&u, va + vb, sa + sb, [1, 0, 0]);
}

#[test]
fn a_cone_on_a_ball_unites_at_its_apex() {
    let a = ball([0.0, 0.0, -4.0], 4.0);
    let c = cone_on_apex([0.0, 0.0, 0.0], 1.0, 3.0, 6.0);
    let u = a.union(&c).unwrap();
    let (v, s) = cone_measures(3.0, 6.0);
    assert_touching(&u, 4.0 / 3.0 * PI * 64.0 + v, 4.0 * PI * 16.0 + s, [1, 0, 0]);
}

#[test]
fn filling_the_corner_between_touching_blocks_makes_one_manifold_piece() {
    // The cubes share the edge x = y = 10; a block over x 5..15, y 5..15 fills round it, so the
    // contact is inside the union and no edge is used by four faces.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([10.0, 10.0, 0.0], [20.0, 20.0, 10.0]);
    let u = a.union(&b).unwrap();
    let c = bx([5.0, 5.0, 0.0], [15.0, 15.0, 10.0]);
    let filled = u.union(&c).unwrap();
    // Two 5 x 5 x 10 quarters of c are new; the plan is a stepped outline of 32 unit sides.
    let area = 2.0 * 250.0 + 10.0 * 80.0;
    assert_touching(&filled, 2500.0, area, [1, 0, 0]);
    assert!(four_face_edges(&filled).is_empty());
}

#[test]
fn cutting_the_contact_away_parts_the_pieces() {
    // A square bar along z round the shared edge removes it: two solids, each a cube less a
    // 2 x 2 corner.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([10.0, 10.0, 0.0], [20.0, 20.0, 10.0]);
    let u = a.union(&b).unwrap();
    let c = bx([8.0, 8.0, -1.0], [12.0, 12.0, 11.0]);
    let d = u.difference(&c).unwrap();
    assert!(d.check().is_empty());
    assert_eq!(d.betti()[0], 2);
    assert!((d.volume() - 2.0 * 960.0).abs() < 1e-9);
    assert!(four_face_edges(&d).is_empty());
}

#[test]
fn cutting_across_the_contact_keeps_it_where_it_is_left() {
    // Cutting the top half off both cubes keeps the shared edge below z = 5.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([10.0, 10.0, 0.0], [20.0, 20.0, 10.0]);
    let u = a.union(&b).unwrap();
    let c = bx([-1.0, -1.0, 5.0], [21.0, 21.0, 11.0]);
    let d = u.difference(&c).unwrap();
    assert_touching(&d, 1000.0, 2.0 * (200.0 + 200.0), [1, 0, 0]);
    assert_eq!(four_face_edges(&d).len(), 1);
}

#[test]
fn a_touching_result_with_its_contact_cut_into() {
    // A ring of four corners touching along four lines, less a slot through one contact: the ring
    // is broken there, so one piece with no tunnel, three contacts left.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 8.0]);
    let c = rod([5.0, 5.0, -1.0], [0.0, 0.0, 1.0], 5.0, 10.0);
    let ring = a.difference(&c).unwrap();
    let slot = bx([9.0, 4.0, -1.0], [11.0, 6.0, 9.0]);
    let d = ring.difference(&slot).unwrap();
    assert!(d.check().is_empty());
    assert_eq!(d.betti()[0], 1);
    assert_eq!(d.betti(), [1, 0, 0]);
    assert_eq!(four_face_edges(&d).len(), 3);
}

fn refused_as_touching(r: Result<Solid>) {
    match r {
        Err(e) if e.kind == ErrorKind::Unsupported => assert!(e.to_string().contains("touch"), "{e}"),
        r => panic!("expected a refusal, got {:?}", r.map(|b| b.volume())),
    }
}

#[test]
fn a_blend_along_a_contact_is_refused() {
    // The edge the cubes share is the contact: a round there would cut into the other cube.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([10.0, 10.0, 0.0], [20.0, 20.0, 10.0]);
    let u = a.union(&b).unwrap();
    // springrep's edge between "a.u_max" (x = 10) and "a.v_max" (y = 10).
    let contact = u.edge_at([10.0, 10.0, 5.0]).unwrap();
    refused_as_touching(u.fillet(&[contact], 1.0));
    refused_as_touching(u.chamfer(&[contact], 1.0));
    // An edge away from the contact still blends: "a.u_min" (x = 0) against "a.v_min" (y = 0).
    let away = u.edge_at([0.0, 0.0, 5.0]).unwrap();
    let f = u.fillet(&[away], 1.0).unwrap();
    assert!(f.check().is_empty());
}

#[test]
fn moving_a_face_at_a_contact_is_refused() {
    // a.u_max is the face x = 10, which ends at the contact: moving it opens or overlaps it.
    let a = bx([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    let b = bx([10.0, 10.0, 0.0], [20.0, 20.0, 10.0]);
    let u = a.union(&b).unwrap();
    let u_max = u.face_at([10.0, 5.0, 5.0]).unwrap();
    let u_min = u.face_at([0.0, 5.0, 5.0]).unwrap();
    refused_as_touching(u.offset_faces(&[u_max], 1.0));
    let moved = u.offset_faces(&[u_min], 1.0).unwrap();
    assert!(moved.check().is_empty());
    assert!((moved.volume() - 2100.0).abs() < 1e-9);
}

#[test]
fn a_face_a_rod_touches_through_a_hole_splits_round_the_hole() {
    // A 40 × 20 × 20 block with a hole r 2 through it along y at x = 20, z = 10, and a rod r 3
    // standing on its front face from z = 5 up: the line of contact runs from the top edge down
    // to the hole, and on below it to where the rod ends. The front face is one face, whose loop
    // runs down the contact, round the hole, down to the rod's end and back.
    let holed = bx([0.0, 0.0, 0.0], [40.0, 20.0, 20.0]).difference(&rod([20.0, -1.0, 10.0], [0.0, 1.0, 0.0], 2.0, 22.0)).unwrap();
    let standing = rod([20.0, -3.0, 5.0], [0.0, 0.0, 1.0], 3.0, 15.0);
    let touched = holed.union(&standing).unwrap();
    assert!(touched.check().is_empty());
    // Cut at x = 30, so the face is split away from the contact: where the loop reaches the
    // hole down the contact it goes on round the hole, not straight back up.
    let left = bx([-1.0, -10.0, -1.0], [30.0, 30.0, 21.0]);
    let kept = touched.intersection(&left).unwrap();
    assert!(kept.check().is_empty());
    assert_eq!(kept.betti()[0], 1);
    // 30 · 20 · 20 of block, less the hole π · 2² · 20, and the rod π · 3² · 15.
    let want = 12_000.0 - 80.0 * PI + 135.0 * PI;
    let got = kept.volume();
    assert!((got - want).abs() < 1e-6 * want, "{got}");
}
