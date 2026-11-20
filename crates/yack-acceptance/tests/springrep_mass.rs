// yack: mass properties against closed forms: volume, centre and the inertia tensor about the centre.
// Adapted from springrep@a03e8a1 tests/mass.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: springrep API replaced by the yack public API (PLAN §4.3); mass_properties becomes
// volume / centroid / inertia (both about the centroid); springrep circles take a diameter, yack's a
// radius; revolve profiles are drawn in the frame's xz plane and turned about its z (springrep's
// xz frame and z axis); the loft's cutter is a block on a frame moved by the same rotation and
// shift; the representation checks and the other volume integrators are API gaps.

use std::f64::consts::{PI, TAU};

use yack::prelude::*;
use yack::{SurfaceKind, codec};

/// The diagonal tensor with these moments about the axes.
fn diagonal(i: [f64; 3]) -> [[f64; 3]; 3] {
    [[i[0], 0.0, 0.0], [0.0, i[1], 0.0], [0.0, 0.0, i[2]]]
}

fn assert_mass(b: &Solid, volume: f64, centre: [f64; 3], inertia: [[f64; 3]; 3]) {
    let (v, c, i) = (b.volume(), b.centroid().unwrap(), b.inertia());
    assert!((v - volume).abs() <= 1e-12 * volume, "volume {v} vs {volume}");
    let size = volume.cbrt();
    let d = [c.x - centre[0], c.y - centre[1], c.z - centre[2]].iter().fold(0.0f64, |m, x| m.max(x.abs()));
    assert!(d <= 1e-12 * size, "centre {c:?} vs {centre:?}");
    let scale = inertia.iter().flatten().fold(0.0f64, |m, x| m.max(x.abs()));
    for r in 0..3 {
        for k in 0..3 {
            assert!((i[r][k] - inertia[r][k]).abs() <= 1e-11 * scale, "inertia {i:?} vs {inertia:?}");
        }
    }
}

#[test]
fn a_box_off_the_origin_has_the_box_formulas() {
    // x 1..11, y 2..8, z 3..6: a 10 × 6 × 3 box of 180 about (6, 5, 4.5), each axis's moment
    // V(b² + c²)/12 in the other two sides, no products.
    let b = extrude(&Frame::at([0.0, 0.0, 3.0]), &Profile::rectangle([1.0, 2.0], [11.0, 8.0]).unwrap(), 3.0).unwrap();
    let v = 180.0;
    let i = [v * 45.0 / 12.0, v * 109.0 / 12.0, v * 136.0 / 12.0];
    assert_mass(&b, v, [6.0, 5.0, 4.5], diagonal(i));
}

#[test]
fn a_cylinder_off_the_axis_has_the_cylinder_formulas() {
    // Radius 4 about (2, −1), height 5: V = 80π, across the axis V(3r² + h²)/12, along it Vr²/2.
    let b = extrude(&Frame::WORLD, &Profile::circle([2.0, -1.0], 8.0 / 2.0).unwrap(), 5.0).unwrap();
    let v = 80.0 * PI;
    let across = v * (48.0 + 25.0) / 12.0;
    assert_mass(&b, v, [2.0, -1.0, 2.5], diagonal([across, across, v * 8.0]));
}

#[test]
fn a_ball_has_two_fifths_v_r_squared_about_every_axis() {
    let r = 3.0;
    let half_disc = Loop::new(vec![
        Segment::Arc { center: [0.0, 0.0].into(), radius: r, start: rad(-PI / 2.0), sweep: rad(PI) },
        Segment::Line { a: [0.0, r].into(), b: [0.0, -r].into() },
    ])
    .unwrap();
    let p = Profile::new(half_disc, vec![]).unwrap();
    let b = revolve(&Frame::WORLD, &p, rad(TAU)).unwrap();
    let v = 4.0 / 3.0 * PI * r * r * r;
    let i = 0.4 * v * r * r;
    assert_mass(&b, v, [0.0; 3], diagonal([i, i, i]));
}

#[test]
fn a_torus_has_the_torus_formulas() {
    // Tube radius a = 2 at R = 10 round z: V = 2π²Ra², along the axis V(R² + 3a²/4), across it
    // V(R²/2 + 5a²/8).
    let (big, a) = (10.0, 2.0);
    let p = Profile::circle([big, 0.0], a).unwrap();
    let b = revolve(&Frame::WORLD, &p, rad(TAU)).unwrap();
    let v = 2.0 * PI * PI * big * a * a;
    let across = v * (big * big / 2.0 + 5.0 * a * a / 8.0);
    let along = v * (big * big + 0.75 * a * a);
    assert_mass(&b, v, [0.0; 3], diagonal([across, across, along]));
}

#[test]
fn an_l_shaped_plate_has_products_of_inertia() {
    // The L is two 1-thick boxes: A, x 0..4 by y 0..2, and B, x 0..2 by y 2..6, each 8 about
    // (2, 1, ½) and (1, 4, ½). Their sum is 16 about (1.5, 2.5, ½); each box adds its own moments
    // V(b² + c²)/12 and, by the parallel-axis rule, V(d² − d dᵀ) for its offset d.
    let pts = [[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [2.0, 2.0], [2.0, 6.0], [0.0, 6.0]];
    let b = extrude(&Frame::WORLD, &Profile::polygon(&pts).unwrap(), 1.0).unwrap();
    let centre = [1.5, 2.5, 0.5];
    let mut want = [[0.0; 3]; 3];
    for (size, at) in [([4.0, 2.0, 1.0], [2.0, 1.0, 0.5]), ([2.0, 4.0, 1.0], [1.0, 4.0, 0.5])] {
        let v: f64 = size.iter().product();
        let s2 = size.map(|x: f64| x * x);
        let own = [v * (s2[1] + s2[2]) / 12.0, v * (s2[0] + s2[2]) / 12.0, v * (s2[0] + s2[1]) / 12.0];
        let d = [at[0] - centre[0], at[1] - centre[1], at[2] - centre[2]];
        let dd: f64 = d.iter().map(|x| x * x).sum();
        for r in 0..3 {
            for k in 0..3 {
                let shift = v * (if r == k { dd } else { 0.0 } - d[r] * d[k]);
                want[r][k] += shift + if r == k { own[r] } else { 0.0 };
            }
        }
    }
    assert!(want[0][1] != 0.0);
    assert_mass(&b, 16.0, centre, want);
}

#[test]
fn a_tilted_cylinder_turns_its_tensor_with_it() {
    // A cylinder along a unit axis z has I = I⊥·1 + (I∥ − I⊥)·z zᵀ: every entry is nonzero when
    // z leans away from all three axes.
    let z = Vec3::new(1.0, 2.0, 2.0) * (1.0 / 3.0);
    let x = Vec3::new(2.0, -2.0, 1.0) * (1.0 / 3.0);
    let o = Point3::new(1.0, -3.0, 2.0);
    // springrep Frame::new(o, z, x): y = z × x.
    let frame = Frame::new(o, x, z.cross(x), z).unwrap();
    let (r, h) = (1.5, 6.0);
    let b = extrude(&frame, &Profile::circle([0.0, 0.0], r).unwrap(), h).unwrap();
    let v = PI * r * r * h;
    let (across, along) = (v * (3.0 * r * r + h * h) / 12.0, v * r * r / 2.0);
    let zs = [z.x, z.y, z.z];
    let mut want = [[0.0; 3]; 3];
    for i in 0..3 {
        for k in 0..3 {
            want[i][k] = if i == k { across } else { 0.0 } + (along - across) * zs[i] * zs[k];
        }
    }
    let c = o + z * (h / 2.0);
    assert_mass(&b, v, [c.x, c.y, c.z], want);
}

/// Each way a solid's volume is measured, on a fresh copy so no face memo carries from one to the
/// next, against `want`, to `rel` of it.
fn assert_volume_every_way(b: &Solid, want: f64, rel: f64) {
    let fresh = || codec::decode(&codec::encode(b)).unwrap();
    // API GAP: §4.3 has one volume integral; springrep's "rough sign" (solid_volume_sign),
    // "to 1e-10" (volume_to) and "exact" (solid_volume) measures are dropped.
    for (what, v) in [("moments", fresh().volume())] {
        assert!((v - want).abs() <= rel * want, "{what}: {v} vs {want}");
    }
}

#[test]
fn a_lofted_cylinder_cut_on_a_slant_measures_its_closed_form_every_way() {
    // A smooth loft through equal circles is a cylinder on a rational spline surface. Below the
    // plane z = h + x·tan θ it holds πr²h, the slant cancelling across the axis, and its area is
    // the disc, the ellipse πr²/cos θ and the side 2πrh. The plane meets the spline side along a
    // fitted curve, so the boundary integrals there run through projections.
    let (r, h, theta) = (5.0, 7.0, 0.3);
    let sections: Vec<(f64, Profile)> =
        [0.0, 5.0, 10.0, 15.0].iter().map(|&z| (z, Profile::circle([0.0, 0.0], r).unwrap())).collect();
    let can = loft_through(&Frame::WORLD, &sections).unwrap();
    assert!(!can.faces_where(|f| matches!(f.kind(), SurfaceKind::Spline)).is_empty());
    // The 100-cube from (−50, −50, −100) to (50, 50, 0), turned −θ about y through the origin and
    // then raised h.
    let turn = Rigid::rotation([0.0; 3], [0.0, 1.0, 0.0], rad(-theta)).unwrap();
    let raise = Rigid::translation([0.0, 0.0, h]);
    let below = Frame::at([-50.0, -50.0, -100.0]).moved(&turn).moved(&raise);
    let cut = block(&below, [100.0, 100.0, 100.0]).unwrap();
    let b = can.intersection(&cut).unwrap();
    // API GAP: the check that some spline face is bounded by an intersection curve with a projected
    // pcurve needs face-loop / coedge / pcurve introspection, which §4.3 does not expose.
    assert_volume_every_way(&b, PI * r * r * h, 1e-9);
    let area = PI * r * r * (1.0 + 1.0 / theta.cos()) + TAU * r * h;
    let got = b.area();
    assert!((got - area).abs() <= 1e-9 * area, "area {got} vs {area}");
}
