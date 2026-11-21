// yack: tests of the glue the yack facade calls (yack_glue.rs).
use super::*;
use crate::Pick;
use nalgebra::Point3;

fn cube(k: &mut DefaultKernel) -> BodyId {
    k.primitive(&Primitive::Box { min: Point3::new(0.0, 0.0, 0.0), max: Point3::new(10.0, 10.0, 10.0) })
        .and_then(|r| r.first())
        .unwrap()
}

/// A plate drilled through with a hole of radius `rad` at `c`, and its top rim's edges.
fn drilled(k: &mut DefaultKernel, size: [f64; 3], c: [f64; 2], rad: f64) -> (BodyId, Vec<u64>) {
    let plate = k.primitive(&Primitive::Box { min: Point3::new(0.0, 0.0, 0.0), max: Point3::new(size[0], size[1], size[2]) }).and_then(|r| r.first()).unwrap();
    let hole = k
        .primitive(&Primitive::Cylinder {
            base: Point3::new(c[0], c[1], -1.0),
            axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
            radius: rad,
            height: size[2] + 2.0,
        })
        .and_then(|r| r.first())
        .unwrap();
    let (d, _) = boolean(k, crate::BoolOp::Subtract, plate, &[hole]).unwrap();
    let rim = k
        .edges(d)
        .unwrap()
        .iter()
        .filter(|e| (e.mid.z - size[2]).abs() < 1e-9 && ((e.mid.x - c[0]).hypot(e.mid.y - c[1]) - rad).abs() < 1e-6)
        .map(|e| e.id.0)
        .collect();
    (d, rim)
}

#[test]
fn a_rim_round_that_runs_off_the_plate_is_declined() {
    // brep-conformance seed 2 chain-rim-after-cut-2ee7ef: the round's outer edge (radius 4.75
    // about (4.73, 4.32)) crosses the plate's sides x = 0 and y = 0.
    let mut k = DefaultKernel::new();
    let size = [10.875072829244813, 10.544234942431189, 5.669455797435571];
    let (d, rim) = drilled(&mut k, size, [4.726518256981529, 4.317994990440217], 3.428784132901775);
    assert!(!rim.is_empty());
    assert!(fillet(&mut k, d, &rim[..1], 1.3217145850745369).is_err());
    assert!(chamfer(&mut k, d, &rim[..1], 1.3217145850745369).is_err());
    // One that stays on the top: Pappus (section (1 - pi/4) r^2, its centroid at
    // rad + r (10 - 3 pi) / (12 - 3 pi) from the axis).
    let (r, rad) = (0.5, 3.428784132901775);
    let (f, _) = fillet(&mut k, d, &rim[..1], r).unwrap();
    let v0 = k.mass_properties(d).unwrap().volume;
    let pi = std::f64::consts::PI;
    let cut = 2.0 * pi * (rad + r * (10.0 - 3.0 * pi) / (12.0 - 3.0 * pi)) * (1.0 - pi / 4.0) * r * r;
    assert!((k.mass_properties(f).unwrap().volume - (v0 - cut)).abs() < 1e-6);
    assert!(chamfer(&mut k, d, &rim[..1], r).is_ok());
}

#[test]
fn edge_rounds_meeting_their_end_faces_fit() {
    let mut k = DefaultKernel::new();
    let b = cube(&mut k);
    assert!(fillet(&mut k, b, &[0], 2.0).is_ok());
    let all: Vec<u64> = (0..k.edges(b).unwrap().len() as u64).collect();
    assert!(fillet(&mut k, b, &all, 2.0).is_ok());
    assert!(chamfer(&mut k, b, &[0], 2.0).is_ok());
}

#[test]
fn a_ball_screwed_three_turns_is_a_round_ended_pipe() {
    // brep-bench sweep/helical-pipe-round-ends: pi r^2 L + 4/3 pi r^3 with L the helix's length.
    let mut k = DefaultKernel::new();
    let ball = k.primitive(&Primitive::Sphere { center: Point3::new(10.0, 0.0, 0.0), radius: 1.0, axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)), seam: nalgebra::Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)) }).and_then(|r| r.first()).unwrap();
    let tau = std::f64::consts::TAU;
    let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 3.0 * tau, rise_per_turn: 5.0 };
    let (s, _) = swept(&mut k, ball, &m).unwrap();
    let pi = std::f64::consts::PI;
    let len = 3.0 * (tau * 10.0).hypot(5.0);
    let want = pi * len + 4.0 / 3.0 * pi;
    let v = k.mass_properties(s).unwrap().volume;
    assert!((v - want).abs() < 1e-6 * want, "{v} against {want}");
}

#[test]
fn a_screwed_torus_cuts_whole_and_exactly() {
    // brep-bench swept/torus-screwed-through-rod: cadrs's runner 1460.6300023677913 (4 faces).
    let mut k = DefaultKernel::new();
    let rod = k
        .primitive(&Primitive::Cylinder {
            base: Point3::new(0.0, 0.0, 0.0),
            axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
            radius: 5.0,
            height: 20.0,
        })
        .and_then(|r| r.first())
        .unwrap();
    let h = (std::f64::consts::TAU * 5.5 / 30.0).atan();
    let (s, c) = (-108f64).to_radians().sin_cos();
    let rot = |v: [f64; 3]| [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
    let centre = rot([11.5, 0.0, 0.0]);
    let (wheel, _) = torus(&mut k, [centre[0], centre[1], -9.0], rot([0.0, h.cos(), -h.sin()]), 6.0, 2.0).unwrap();
    let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 456f64.to_radians(), rise_per_turn: 30.0 };
    let (out, _) = cut_swept(&mut k, rod, wheel, &[m]).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 1460.6300023677913).abs() < 1e-6 * v, "{v}");
    assert!(k.get(out).unwrap().faces.len() <= 10);
}

#[test]
fn a_v_wheel_screwed_through_a_rod_cuts_exactly() {
    // brep-bench swept/v-wheel-screwed-through-rod: cadrs's runner 1554.3809661193923.
    let mut k = DefaultKernel::new();
    let rod = k
        .primitive(&Primitive::Cylinder {
            base: Point3::new(0.0, 0.0, 0.0),
            axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)),
            radius: 5.0,
            height: 20.0,
        })
        .and_then(|r| r.first())
        .unwrap();
    let h = (std::f64::consts::TAU * 4.5 / 30.0).atan();
    let along = Vector3::new(0.0, h.cos(), -h.sin());
    let centre = Point3::new(14.0, 0.0, 0.0);
    let x = Vector3::new(-1.0, 0.0, 0.0);
    let on = crate::Plane::new(centre, x, x.cross(&along));
    let w = 2.5 * 30f64.to_radians().tan();
    let section = crate::Profile::of(on, vec![crate::Loop::polygon(&[[10.0, 0.0], [7.5, w], [7.5, -w]])]);
    let wheel = k.revolve(&section, Axis::new(centre, along), std::f64::consts::TAU).and_then(|r| r.first()).unwrap();
    // cadrs's runner turns it from -144 degrees: placed there first.
    let start = crate::facade::motion::end_motion(&Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: (-144f64).to_radians(), rise_per_turn: 30.0 }).unwrap();
    let wheel = k.transform_motion(wheel, &start).and_then(|r| r.first()).unwrap();
    let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 528f64.to_radians(), rise_per_turn: 30.0 };
    let (out, _) = cut_swept(&mut k, rod, wheel, &[m]).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 1554.3809661193923).abs() < 1e-6 * v, "{v}");
}

#[test]
fn a_key_carried_along_an_s_shaped_spline() {
    // brep-bench swept/key-carried-along-a-spline: cadrs's runner 10652.879989210425. The
    // S-shaped control polygon encloses no area, yet the path is planar.
    let mut k = DefaultKernel::new();
    let ctrl = [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 20.0, 0.0], [30.0, 20.0, 0.0]];
    assert_eq!(plane_normal(&ctrl, 1e-9), Some([0.0, 0.0, 1.0]));
    let block = |k: &mut DefaultKernel, a: [f64; 3], b: [f64; 3]| {
        k.primitive(&Primitive::Box { min: p3(a), max: p3(b) }).and_then(|r| r.first()).unwrap()
    };
    let plate = block(&mut k, [-6.0, -6.0, -8.0], [36.0, 26.0, 0.0]);
    let key = block(&mut k, [-1.5, -0.8, -1.5], [1.5, 0.8, 2.5]);
    assert!(along_up(&k, key, &ctrl, true, 1e-9).is_err());
    let up = along_up(&k, key, &ctrl, false, 1e-9).unwrap();
    let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let m = Move::Along { degree: 3, knots: &knots, ctrl: &ctrl, up };
    let (out, _) = cut_swept(&mut k, plate, key, &[m]).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 10652.879989210425).abs() < 1e-6 * v, "{v}");
    // A post on its own axis at the path's start: a fixed frame is the same motion.
    let post = k
        .primitive(&Primitive::Cylinder { base: Point3::new(0.0, 0.0, -1.5), axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)), radius: 1.0, height: 3.0 })
        .and_then(|r| r.first())
        .unwrap();
    assert_eq!(along_up(&k, post, &ctrl, true, 1e-9).unwrap(), [0.0, 0.0, 1.0]);
}

#[test]
fn a_square_thread_at_a_growing_lead() {
    // brep-bench swept/square-thread-at-a-growing-lead: cadrs's runner 1324.4782675300098,
    // its rise 6/(2 pi) θ + 0.3/(4 pi^2) θ^2 in the absolute angle, from -90 to 1260 degrees.
    let mut k = DefaultKernel::new();
    let rod = k
        .primitive(&Primitive::Cylinder { base: Point3::new(0.0, 0.0, 0.0), axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)), radius: 5.0, height: 20.0 })
        .and_then(|r| r.first())
        .unwrap();
    let bar = k.primitive(&Primitive::Box { min: Point3::new(4.0, -8.0, -1.0), max: Point3::new(12.0, 8.0, 1.0) }).and_then(|r| r.first()).unwrap();
    let pi = std::f64::consts::PI;
    let (a1, a2, f) = (6.0 / (2.0 * pi), 0.3 / (4.0 * pi * pi), (-90f64).to_radians());
    // The same rise in θ - from.
    let rise = [a1 * f + a2 * f * f, a1 + 2.0 * a2 * f, a2];
    assert!((absolute_rise(&rise, f)[1] - a1).abs() < 1e-12 && (absolute_rise(&rise, f)[2] - a2).abs() < 1e-12);
    assert!(absolute_rise(&rise, f)[0].abs() < 1e-12);
    let m = Move::Climb { point: [0.0; 3], axis: [0.0, 0.0, 1.0], from: f, to: 1260f64.to_radians(), rise: &rise };
    let (out, _) = cut_swept(&mut k, rod, bar, &[m]).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 1324.4782675300098).abs() < 1e-6 * v, "{v}");
    // A constant lead from a nonzero angle is a screw: the same cut as Move::Screw placed there.
    let screw = |k: &mut DefaultKernel, m: &Move| {
        let bar = k.primitive(&Primitive::Box { min: Point3::new(4.0, -1.0, -1.0), max: Point3::new(12.0, 1.0, 1.0) }).and_then(|r| r.first()).unwrap();
        let (o, _) = cut_swept(k, rod, bar, std::slice::from_ref(m)).unwrap();
        k.mass_properties(o).unwrap().volume
    };
    let steady = [a1 * f, a1];
    let v1 = screw(&mut k, &Move::Climb { point: [0.0; 3], axis: [0.0, 0.0, 1.0], from: f, to: 2.0 * pi, rise: &steady });
    assert!(v1 > 0.0 && v1 < 500.0 * pi);
}

#[test]
fn a_ball_end_mill_raster_is_cut_a_row_at_a_time() {
    // brep-bench cnc/ball-end-surface-raster: 944 slides; cadrs's runner 119999.9438120307.
    use crate::facade::profile::{revolve, Place, Seg};
    let mut k = DefaultKernel::new();
    let pi = std::f64::consts::PI;
    let f = |x: f64, y: f64| -5.0 + 4.0 * (pi * x / 50.0).sin() * (pi * y / 40.0).sin();
    let mut points = vec![[-5.0, 0.0, 5.0]];
    for j in 0..=40 {
        let y = 2.0 * f64::from(j);
        let along: Vec<i32> = if j % 2 == 0 { (0..=22).collect() } else { (0..=22).rev().collect() };
        points.extend(along.into_iter().map(|i| [-5.0 + 5.0 * f64::from(i), y, f(-5.0 + 5.0 * f64::from(i), y)]));
    }
    points.push([-5.0, 80.0, 5.0]);
    let moves: Vec<Move> = points.windows(2).map(|w| Move::Slide { by: [w[1][0] - w[0][0], w[1][1] - w[0][1], w[1][2] - w[0][2]] }).collect();
    let side = [
        Seg::Arc { center: [0.0, 3.0], radius: 3.0, start: -pi / 2.0, sweep: pi / 2.0 },
        Seg::Line { a: [3.0, 3.0], b: [3.0, 30.0] },
        Seg::Line { a: [3.0, 30.0], b: [0.0, 30.0] },
        Seg::Line { a: [0.0, 30.0], b: [0.0, 0.0] },
    ];
    let at = Place { origin: [-5.0, 0.0, 5.0], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    let (mill, _) = revolve(&mut k, &at, &[&side], 2.0 * pi).unwrap();
    let stock = k.primitive(&Primitive::Box { min: Point3::new(0.0, 0.0, -20.0), max: Point3::new(100.0, 80.0, 0.0) }).and_then(|r| r.first()).unwrap();
    let (out, _) = cut_swept(&mut k, stock, mill, &moves).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 119999.9438120307).abs() < 1e-6 * v, "{v}");
}

/// The L-block of brep-bench's `*/l-block-inside-consumed` (its inside corner edge along x at
/// (·, 3, 3)) and that edge.
fn l_block(k: &mut DefaultKernel) -> (BodyId, u64) {
    use crate::facade::profile::{extrude, Place, Seg};
    let p = [[0.0, 0.0], [10.0, 0.0], [10.0, 3.0], [3.0, 3.0], [3.0, 10.0], [0.0, 10.0]];
    let segs: Vec<Seg> = (0..6).map(|i| Seg::Line { a: p[i], b: p[(i + 1) % 6] }).collect();
    let at = Place { origin: [0.0; 3], x: [0.0, 0.0, 1.0], y: [1.0, 0.0, 0.0], z: [0.0, 1.0, 0.0] };
    let (b, _) = extrude(k, &at, &[&segs], 10.0).unwrap();
    let e = k.edge_at(b, Point3::new(3.0, 5.0, 3.0)).unwrap();
    (b, e.0)
}

#[test]
fn a_blend_that_swallows_a_face_fits() {
    // brep-bench chamfer/ and fillet/l-block-inside-consumed: the 7 mm blend of the inside
    // corner swallows the 3 mm walls beside it and meets the faces past them (cadrs's 755 and
    // 615.1549, as bakeoff7).
    let mut k = DefaultKernel::new();
    let (b, e) = l_block(&mut k);
    let (c, _) = chamfer(&mut k, b, &[e], 7.0).unwrap();
    assert!((k.mass_properties(c).unwrap().volume - 755.0).abs() < 1e-6);
    let (f, _) = fillet(&mut k, b, &[e], 7.0).unwrap();
    assert!((k.mass_properties(f).unwrap().volume - 615.1549).abs() < 1e-3);
}

#[test]
fn an_octahedron_chamfered_edge_by_edge_fits() {
    // brep-bench chamfer/octahedron-all-edges-mitered: each bevel's ends run past the short
    // edges a truncated corner leaves, onto the faces there (cadrs's 981.33, as bakeoff7).
    use crate::facade::profile::{extrude, Place, Seg};
    let mut k = DefaultKernel::new();
    let d = [[10.0, 0.0], [0.0, 10.0], [-10.0, 0.0], [0.0, -10.0]];
    let segs: Vec<Seg> = (0..4).map(|i| Seg::Line { a: d[i], b: d[(i + 1) % 4] }).collect();
    let at = Place { origin: [0.0, 0.0, -10.0], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    let (mut b, _) = extrude(&mut k, &at, &[&segs], 20.0).unwrap();
    for i in 0..8 {
        let n = [1, 2, 4].map(|bit| if i & bit == 0 { 1.0 } else { -1.0 });
        b = split(&mut k, b, n.map(|c| c * 10.0 / 3.0), n).unwrap().1.unwrap().0;
    }
    let mids: Vec<Point3<f64>> = k.edges(b).unwrap().iter().map(|e| e.mid).collect();
    for p in mids {
        let e = k.edge_at(b, p).unwrap();
        b = chamfer(&mut k, b, &[e.0], 6f64.sqrt()).unwrap().0;
    }
    let v = k.mass_properties(b).unwrap().volume;
    assert!((v - 981.33).abs() < 5e-3, "{v}");
}

#[test]
fn a_spline_bar_screwed_through_a_rod_cuts_exactly() {
    // brep-bench swept/spline-bar-screwed-through-rod (yack's case, its loop starting at the
    // point nearest the screw's axis): cadrs's runner 1335.3195911687133. The loop's seam is
    // put at its farthest point, which the trim takes away.
    use crate::facade::profile::{extrude, Place, Seg};
    let mut k = DefaultKernel::new();
    let rod = k
        .primitive(&Primitive::Cylinder { base: Point3::new(0.0, 0.0, 0.0), axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0)), radius: 5.0, height: 20.0 })
        .and_then(|r| r.first())
        .unwrap();
    let c = [[3.5, 0.0], [4.5, -1.5], [8.0, -1.5], [12.0, -1.0], [12.0, 1.0], [8.0, 1.5], [4.5, 1.5]];
    let ctrl: Vec<[f64; 2]> = c.iter().chain(c.iter().take(3)).copied().collect();
    let knots: Vec<f64> = (0..14).map(f64::from).collect();
    let seg = [Seg::Periodic { degree: 3, knots: &knots, ctrl: &ctrl }];
    let at = Place { origin: [8.0, 0.0, -1.5], x: [0.0, -1.0, 0.0], y: [0.0, 0.0, 1.0], z: [-1.0, 0.0, 0.0] };
    let (bar, _) = extrude(&mut k, &at, &[&seg], 16.0).unwrap();
    let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 1440f64.to_radians(), rise_per_turn: 6.0 };
    let (out, _) = cut_swept(&mut k, rod, bar, &[m]).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 1335.3195911687133).abs() < 1e-6 * v, "{v}");
}

#[test]
fn a_motion_cadrs_only_meshes_is_refused_under_exact_only() {
    // A cube climbing (a screw whose lead grows): cadrs sweeps it only as a mesh.
    let mut k = DefaultKernel::new();
    let b = k.primitive(&Primitive::Box { min: Point3::new(2.0, -0.5, 0.0), max: Point3::new(3.0, 0.5, 1.0) }).and_then(|r| r.first()).unwrap();
    let climb = SolidMotion::Climb { axis: Axis::new(Point3::origin(), Vector3::new(0.0, 0.0, 1.0)), from: 0.0, to: 1.0, rise: vec![0.0, 0.5, 0.5] };
    // Unset (cadrs's default): the mesh.
    assert!(!k.tolerances().exact_only);
    let meshed = k.sweep_solid(b, &climb).and_then(|r| r.first()).unwrap();
    assert!(k.get(meshed).unwrap().faces.iter().all(|f| matches!(f.surf, crate::native::surface::Surface::Plane { .. })));
    // Set: refused, and the session's tolerances as they were after.
    let r = exact(&mut k, |k| k.sweep_solid(b, &climb));
    assert!(matches!(r, Err(KernelError::Unsupported(_))), "{r:?}");
    assert!(!k.tolerances().exact_only);
}

fn rod(k: &mut DefaultKernel) -> BodyId {
    k.primitive(&Primitive::Cylinder { base: Point3::origin(), axis: nalgebra::Unit::new_normalize(Vector3::z()), radius: 5.0, height: 20.0 })
        .and_then(|r| r.first())
        .unwrap()
}

#[test]
fn a_cone_point_screwed_through_a_rod_is_trimmed_first() {
    // brep-bench swept/cone-point-screwed-through-rod: cadrs's runner 1513.245602658365 (its cone
    // trimmed to the rod's reach; whole, cadrs meshes it).
    let mut k = DefaultKernel::new();
    let r = rod(&mut k);
    // The cone's base at (12, 0, 0) after the -90 degree turn and 1.5 down: at (0, -12, -1.5),
    // its point toward the axis.
    let h = 8.0 * 30f64.to_radians().tan();
    let bit = k
        .primitive(&Primitive::Cone { base: Point3::new(0.0, -12.0, -1.5), axis: nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 0.0)), base_radius: h, top_radius: 0.0, height: 8.0 })
        .and_then(|r| r.first())
        .unwrap();
    let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 4.0 * std::f64::consts::TAU, rise_per_turn: 6.0 };
    let (out, _) = cut_swept(&mut k, r, bit, &[m]).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 1513.245602658365).abs() < 1e-6 * v, "{v}");
}

#[test]
fn a_cone_spun_about_a_line_through_it() {
    // brep-bench swept/cone-spun-about-a-line-through-it: cadrs's runner 288.495056512097. The
    // glue's pointed cone is drawn from its point, which cadrs sweeps exactly.
    use crate::facade::profile::Place;
    let mut k = DefaultKernel::new();
    let (s, c) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
    let base = Place { origin: [5.0 * c, 0.0, 5.0 * s], x: [0.0, 1.0, 0.0], y: [s, 0.0, -c], z: [-c, 0.0, -s] };
    let (spike, h) = primitive(&mut k, &base, Prim::Cone { bottom: 7.0 * 25f64.to_radians().tan(), top: 0.0, height: 7.0 }).unwrap();
    // Its base still named the start cap.
    assert!(h.generated.iter().any(|(_, o)| matches!(o, Origin::StartCap { .. })));
    let m = Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 150f64.to_radians() };
    let (out, _) = swept(&mut k, spike, &m).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 288.495056512097).abs() < 1e-6 * v, "{v}");
}

#[test]
fn a_v_wheel_turned_once_round_a_rod() {
    // brep-bench swept/v-wheel-turned-once-round-rod: cadrs's runner 1536.4810559042348, by its
    // endless turn.
    use crate::facade::profile::{revolve, Place, Seg};
    let mut k = DefaultKernel::new();
    let (s, c, t) = (20f64.to_radians().sin(), 20f64.to_radians().cos(), 2.5 * 30f64.to_radians().tan());
    let hub = Place { origin: [14.0, 0.0, 10.0], x: [-1.0, 0.0, 0.0], y: [0.0, -c, s], z: [0.0, s, c] };
    let p = [[7.5, -t], [10.0, 0.0], [7.5, t]];
    let segs: Vec<Seg> = (0..3).map(|i| Seg::Line { a: p[i], b: p[(i + 1) % 3] }).collect();
    let (wheel, _) = revolve(&mut k, &hub, &[&segs], std::f64::consts::TAU).unwrap();
    let r = rod(&mut k);
    let m = Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: std::f64::consts::TAU };
    let (out, _) = cut_swept(&mut k, r, wheel, &[m]).unwrap();
    let v = k.mass_properties(out).unwrap().volume;
    assert!((v - 1536.4810559042348).abs() < 1e-6 * v, "{v}");
}

#[test]
fn faces_thicken_joined_sharp_or_round() {
    // brep-bench thicken/angle-bracket-{sharp,rounded} (a block's floor and wall, 1 outward:
    // 168, 160 + 2π) and open-box-outward-{sharp,rounded} (a cube's floor and four walls: 584,
    // 564.926248174189).
    let mut k = DefaultKernel::new();
    let block = k.primitive(&Primitive::Box { min: Point3::origin(), max: Point3::new(10.0, 8.0, 10.0) }).and_then(|r| r.first()).unwrap();
    let f = |k: &DefaultKernel, b: BodyId, p: [f64; 3]| k.face_at(b, Point3::new(p[0], p[1], p[2])).unwrap().0;
    let ix = [f(&k, block, [5.0, 4.0, 0.0]), f(&k, block, [0.0, 4.0, 5.0])];
    let v = |k: &mut DefaultKernel, b: BodyId, ix: &[u64], sharp: bool| {
        let (t, _) = thicken_joined(k, b, ix, 1.0, sharp).unwrap();
        k.mass_properties(t).unwrap().volume
    };
    assert!((v(&mut k, block, &ix, true) - 168.0).abs() < 1e-6);
    assert!((v(&mut k, block, &ix, false) - (160.0 + 2.0 * std::f64::consts::PI)).abs() < 1e-6);
    let cube = cube(&mut k);
    let tray: Vec<u64> = [[5.0, 5.0, 0.0], [0.0, 5.0, 5.0], [10.0, 5.0, 5.0], [5.0, 0.0, 5.0], [5.0, 10.0, 5.0]].iter().map(|&p| f(&k, cube, p)).collect();
    assert!((v(&mut k, cube, &tray, true) - 584.0).abs() < 1e-6);
    assert!((v(&mut k, cube, &tray, false) - 564.926248174189).abs() < 1e-6);
}

#[test]
fn faces_replaced_by_b_spline_nets() {
    // brep-bench nurbs/bspline-dome: a block 20 x 20 x 10 under a free bicubic net (cadrs's
    // runner 3081.0055555555546), and nurbs/flat-bspline-slabs-united: two blocks whose
    // touching faces are flat B-spline patches, united (699.9999999999997).
    let mut k = DefaultKernel::new();
    const X: [f64; 6] = [0.0, 3.0, 7.0, 11.0, 16.0, 20.0];
    const Y: [f64; 5] = [0.0, 5.0, 10.0, 15.0, 20.0];
    const Z: [[f64; 5]; 6] = [
        [3.0, 5.0, 6.0, 5.0, 4.0],
        [5.0, 9.0, 11.0, 8.0, 5.0],
        [6.0, 12.0, 15.0, 10.0, 6.0],
        [4.0, 10.0, 18.0, 9.0, 5.0],
        [5.0, 8.0, 9.0, 7.0, 4.0],
        [2.0, 4.0, 5.0, 4.0, 3.0],
    ];
    let net: Vec<Vec<[f64; 3]>> = (0..6).map(|i| (0..5).map(|j| [X[i], Y[j], Z[i][j]]).collect()).collect();
    let ku = [0.0, 0.0, 0.0, 0.0, 0.4, 0.4, 1.0, 1.0, 1.0, 1.0];
    let kv = [0.0, 0.0, 0.0, 0.0, 1.0 / 3.0, 1.0, 1.0, 1.0, 1.0];
    let b = k.primitive(&Primitive::Box { min: Point3::origin(), max: Point3::new(20.0, 20.0, 10.0) }).and_then(|r| r.first()).unwrap();
    let top = k.face_at(b, Point3::new(10.0, 10.0, 10.0)).unwrap().0;
    let (d, _) = replace_face(&mut k, b, top, [3, 3], &ku, &kv, &net, None).unwrap();
    let v = k.mass_properties(d).unwrap().volume;
    assert!((v - 3081.0055555555546).abs() < 1e-6 * v, "{v}");

    let flat = |xs: &[f64], ys: &[f64]| -> Vec<Vec<[f64; 3]>> { xs.iter().map(|&x| ys.iter().map(|&y| [x, y, 4.0]).collect()).collect() };
    let bezier = |n: usize| [vec![0.0; n], vec![1.0; n]].concat();
    let lower = k.primitive(&Primitive::Box { min: Point3::origin(), max: Point3::new(10.0, 10.0, 4.0) }).and_then(|r| r.first()).unwrap();
    let f = k.face_at(lower, Point3::new(5.0, 5.0, 4.0)).unwrap().0;
    let (lower, _) = replace_face(&mut k, lower, f, [3, 3], &bezier(4), &bezier(4), &flat(&[0.0, 2.0, 7.0, 10.0], &[0.0, 4.0, 5.0, 10.0]), None).unwrap();
    let upper = k.primitive(&Primitive::Box { min: Point3::new(5.0, 3.0, 4.0), max: Point3::new(15.0, 13.0, 7.0) }).and_then(|r| r.first()).unwrap();
    let f = k.face_at(upper, Point3::new(10.0, 8.0, 4.0)).unwrap().0;
    let (upper, _) = replace_face(&mut k, upper, f, [2, 2], &bezier(3), &bezier(3), &flat(&[5.0, 12.0, 15.0], &[3.0, 4.0, 13.0]), None).unwrap();
    let (u, _) = boolean(&mut k, crate::BoolOp::Union, lower, &[upper]).unwrap();
    let v = k.mass_properties(u).unwrap().volume;
    assert!((v - 699.9999999999997).abs() < 1e-6 * v, "{v}");
}

#[test]
fn a_ring_lying_flat_turned_whole() {
    // brep-bench swept/ring-lying-flat-turned-whole: the torus (8, 0) R 3 r 1, axis upright,
    // turned a whole turn about z: 2πD(4rR + πr²) = 761.0994599066700 (Pappus). Its axis is
    // parallel to the turn's: swung, not cadrs's endless turn (which answers 217.13 there).
    let mut k = DefaultKernel::new();
    let (ring, _) = torus(&mut k, [8.0, 0.0, 0.0], [0.0, 0.0, 1.0], 3.0, 1.0).unwrap();
    let m = Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: std::f64::consts::TAU };
    let (s, _) = swept(&mut k, ring, &m).unwrap();
    let v = k.mass_properties(s).unwrap().volume;
    assert!((v - 761.09945990667004).abs() < 1e-6 * v, "{v}");
}

#[test]
fn exact_puts_the_tolerances_back_on_every_way_out() {
    let mut k = DefaultKernel::new();
    let r: Result<()> = exact(&mut k, |_| Err(KernelError::Unsupported("no")));
    assert!(r.is_err() && !k.tolerances().exact_only);
}

#[test]
fn a_block_top_replaced_by_a_fill_of_four_parabolas() {
    // fill/coons-patch-of-four-parabolas as block(10 × 10 × 10).replace_face(top, Fill { the 4
    // arcs }): 1000 (cadrs's runner).
    use crate::facade::profile::Spline;
    let mut k = DefaultKernel::new();
    let c = cube(&mut k);
    let top = k.face_at(c, Point3::new(5.0, 5.0, 10.0)).unwrap().0;
    let q = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let arcs = [
        [[0.0, 0.0, 10.0], [5.0, 0.0, 14.0], [10.0, 0.0, 10.0]],
        [[10.0, 0.0, 10.0], [10.0, 5.0, 6.0], [10.0, 10.0, 10.0]],
        [[10.0, 10.0, 10.0], [5.0, 10.0, 6.0], [0.0, 10.0, 10.0]],
        [[0.0, 10.0, 10.0], [0.0, 5.0, 14.0], [0.0, 0.0, 10.0]],
    ];
    let b: Vec<Spline> = arcs.iter().map(|a| Spline { degree: 2, knots: &q, ctrl: a, weights: None }).collect();
    let (r, _) = replace_face_fill(&mut k, c, top, &b).unwrap();
    let v = k.mass_properties(r).unwrap().volume;
    assert!((v - 1000.0).abs() < 1e-6 * v, "{v}");
}

#[test]
fn a_notch_pushed_shut_is_the_box() {
    // conformance closing-offsets-3c8da3c4: a U-block's notch wall pushed 2.77 onto the facing
    // wall: the box, 6 faces, area 262.83918347690224 (cadrs's offset keeps both walls back to
    // back inside: 10 faces, area 310.06).
    use crate::facade::profile::{extrude, Place, Seg};
    let mut k = DefaultKernel::new();
    let (w, h, d) = (8.894168270224732, 5.248374306338988, 5.9918269330986);
    let p = [[0.0, 0.0], [w, 0.0], [w, h], [5.514168303630225, h], [5.514168303630225, 1.307900772125746], [2.7449948436746756, 1.307900772125746], [2.7449948436746756, h], [0.0, h]];
    let segs: Vec<Seg> = (0..8).map(|i| Seg::Line { a: p[i], b: p[(i + 1) % 8] }).collect();
    let at = Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 0.0, 1.0], z: [0.0, -1.0, 0.0] };
    let (u, _) = extrude(&mut k, &at, &[&segs], d).unwrap();
    assert!(inner_faces(&k, u).unwrap().is_empty());
    let f = k.face_at(u, Point3::new(2.7449948436746756, -2.9959134665493, 3.2781375392323673)).unwrap().0;
    // cadrs's own offset: the two walls back to back inside, both found.
    let raw = k.offset(u, &OffsetSpec { distance: 0.0, faces: vec![(FaceId(f), 2.769173459955549)], sharp: true }).and_then(|r| r.first()).unwrap();
    assert_eq!(inner_faces(&k, raw).unwrap().len(), 2);
    let (b, _) = offset_faces(&mut k, u, &[f], 2.769173459955549).unwrap();
    let m = k.mass_properties(b).unwrap();
    assert_eq!(k.get(b).unwrap().faces.len(), 6);
    assert!((m.surface_area - 2.0 * (w * h + h * d + d * w)).abs() < 1e-9, "{}", m.surface_area);
    assert!((m.volume - w * h * d).abs() < 1e-9, "{}", m.volume);
}
#[test]
fn curved_solids_have_no_inner_faces() {
    // A torus, a ball, a cone, a tumbling torus's sweep: no face with the solid on both sides
    // (a tessellation triangle's centroid sags inside a convex face by up to its deflection).
    let mut k = DefaultKernel::new();
    let (t, _) = torus(&mut k, [0.0; 3], [0.0, 0.0, 1.0], 5.0, 1.0).unwrap();
    let ball = k.primitive(&Primitive::Sphere { center: Point3::origin(), radius: 3.0, axis: nalgebra::Unit::new_normalize(Vector3::z()), seam: nalgebra::Unit::new_normalize(Vector3::x()) }).and_then(|r| r.first()).unwrap();
    let cone = k.primitive(&Primitive::Cone { base: Point3::origin(), axis: nalgebra::Unit::new_normalize(Vector3::z()), base_radius: 3.0, top_radius: 1.0, height: 4.0 }).and_then(|r| r.first()).unwrap();
    // crates/yack/tests/held.rs's tumbling torus: the ring (3, 0, 2) R 1 r 0.5 rolled ±18.75° about
    // the upright through (2, 0, 0) while tumbled 75° to −75° about x through (0, 0.5, 0.5).
    let (ring, _) = torus(&mut k, [3.0, 0.0, 2.0], [0.0, 0.0, 1.0], 1.0, 0.5).unwrap();
    let r = 18.75f64.to_radians();
    let t75 = 75f64.to_radians();
    let tumble = [
        crate::facade::motion::Step::Turn { point: [2.0, 0.0, 0.0], axis: [0.0, 0.0, 1.0], from: -r, to: r },
        crate::facade::motion::Step::Turn { point: [0.0, 0.5, 0.5], axis: [1.0, 0.0, 0.0], from: t75, to: -t75 },
    ];
    let tumbled = crate::facade::motion::together(&mut k, ring, &tumble).unwrap().made.0;
    // held.rs's paraboloid tip: the paraboloid z = x²/8 to 8, cut by the slab below 1e-4 (a
    // sliver 0.03 across).
    use crate::facade::profile::{revolve, Place, Seg};
    let side = [
        Seg::Conic { a: [0.0, 0.0], apex: [4.0, 0.0], b: [8.0, 8.0], weight: 1.0 },
        Seg::Line { a: [8.0, 8.0], b: [0.0, 8.0] },
        Seg::Line { a: [0.0, 8.0], b: [0.0, 0.0] },
    ];
    let world = Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    let (para, _) = revolve(&mut k, &world, &[&side], std::f64::consts::TAU).unwrap();
    let slab = k.primitive(&Primitive::Box { min: Point3::new(-9.0, -9.0, -1.0), max: Point3::new(9.0, 9.0, 1e-4) }).and_then(|r| r.first()).unwrap();
    let (tip, _) = boolean(&mut k, crate::BoolOp::Intersect, para, &[slab]).unwrap();
    assert!(k.get(tumbled).unwrap().faces.len() > 1);
    for b in [t, ball, cone, tumbled, tip] {
        assert!(inner_faces(&k, b).unwrap().is_empty(), "{:?}", inner_faces(&k, b));
    }
}

#[test]
fn an_l_step_pushed_level_is_the_box() {
    // conformance closing-offsets-f9a2a750: an L-block's low top pushed 1.40065 up level with
    // its high top: the box (cadrs's offset leaves an edge of no length there).
    use crate::facade::profile::{extrude, Place, Seg};
    let mut k = DefaultKernel::new();
    let (w, h, d) = (8.270117232320677, 5.3617773736568015, 4.286181010494695);
    let p = [[0.0, 0.0], [w, 0.0], [w, 3.9611320861080412], [3.6147345061240137, 3.9611320861080412], [3.6147345061240137, h], [0.0, h]];
    let segs: Vec<Seg> = (0..6).map(|i| Seg::Line { a: p[i], b: p[(i + 1) % 6] }).collect();
    let at = Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 0.0, 1.0], z: [0.0, -1.0, 0.0] };
    let (l, _) = extrude(&mut k, &at, &[&segs], d).unwrap();
    let f = k.face_at(l, Point3::new(5.9424, -2.1431, 3.9611320861080412)).unwrap().0;
    let raw = k.offset(l, &OffsetSpec { distance: 0.0, faces: vec![(FaceId(f), 1.4006452875487603)], sharp: true }).and_then(|r| r.first()).unwrap();
    assert!(has_null_edge(&k, raw).unwrap());
    let (b, _) = offset_faces(&mut k, l, &[f], 1.4006452875487603).unwrap();
    let m = k.mass_properties(b).unwrap();
    assert!(!has_null_edge(&k, b).unwrap());
    assert_eq!(k.get(b).unwrap().faces.len(), 6);
    assert!((m.volume - w * h * d).abs() < 1e-9 && (m.surface_area - 2.0 * (w * h + h * d + d * w)).abs() < 1e-9, "{} {}", m.volume, m.surface_area);
}

#[test]
fn a_hypoid_gear_cut_by_its_lofted_pinion_tooth() {
    // brep-bench swept/hypoid-gear-cut-by-its-pinion: the tooth lofted between the inner and
    // outer cone planes (its sides planes, not B-splines) cut by the pinion's and the gear's
    // turns at once: cadrs's runner 11792.246457926205.
    use crate::facade::motion::{cut_together, end_motion, Step};
    use crate::facade::profile::{loft, revolve, Place, Seg};
    let mut k = DefaultKernel::new();
    let s5 = 5f64.sqrt();
    let tan20 = 20f64.to_radians().tan();
    let (g, d) = ([2.0 / s5, 0.0, 1.0 / s5], [-1.0 / s5, 0.0, 2.0 / s5]);
    let ae = 15.0 * s5;
    let w = |u: f64| std::f64::consts::FRAC_PI_2 - u * tan20;
    let trap = |kk: f64| -> Vec<[f64; 2]> { [[2.5, -w(2.5)], [2.5, w(2.5)], [-2.5, w(-2.5)], [-2.5, -w(-2.5)]].iter().map(|[u, v]| [u * kk, v * kk]).collect() };
    let seg = |p: &[[f64; 2]]| -> Vec<Seg<'static>> { (0..p.len()).map(|i| Seg::Line { a: p[i], b: p[(i + 1) % p.len()] }).collect() };
    let (inner, outer) = (ae - 12.0, ae + 2.0);
    let foot = Place { origin: g.map(|c| c * inner), x: d, y: [0.0, -1.0, 0.0], z: g };
    let (sl, sh) = (seg(&trap(inner / ae)), seg(&trap(outer / ae)));
    let (t, _) = loft(&mut k, &foot, &[&sl], outer - inner, &[&sh]).unwrap();
    assert!(k.get(t).unwrap().faces.iter().all(|f| matches!(f.surf, crate::native::surface::Surface::Plane { .. })));
    let at = |sv: f64, u: f64| [sv * g[0] + u * d[0], sv * g[2] + u * d[2]];
    let ai = ae - 10.0;
    let q = seg(&[at(ae, -2.0), at(ae, 6.0), at(ai, 6.0), at(ai, -2.0 * ai / ae)]);
    let world = Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    let (blank, _) = revolve(&mut k, &world, &[&q], std::f64::consts::TAU).unwrap();
    let phi = (24.0 / (75.0 - 5.0 * s5)).asin();
    let m = end_motion(&Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: phi }).unwrap();
    let tooth = k.transform_motion(t, &m).and_then(|r| r.first()).unwrap();
    let (r70, gz) = (70f64.to_radians(), (98.0f64 / 3.0).to_radians());
    let steps = [Step::Turn { point: [0.0, 12.0, 0.0], axis: [1.0, 0.0, 0.0], from: -r70, to: r70 }, Step::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], from: -gz, to: gz }];
    let (cut, _) = cut_together(&mut k, blank, tooth, &steps).unwrap();
    let v = k.mass_properties(cut).unwrap().volume;
    assert!((v - 11792.246457926205).abs() < 1e-6 * v, "{v}");
}

#[test]
fn a_split_sorts_its_sides() {
    let mut k = DefaultKernel::new();
    let b = cube(&mut k);
    let (front, back) = split(&mut k, b, [0.0, 0.0, 4.0], [0.0, 0.0, 1.0]).unwrap();
    let v = |k: &DefaultKernel, m: Option<Made>| k.mass_properties(m.unwrap().0).unwrap().volume;
    assert!((v(&k, front) - 600.0).abs() < 1e-9);
    assert!((v(&k, back) - 400.0).abs() < 1e-9);
    let (front, back) = split(&mut k, b, [0.0, 0.0, 20.0], [0.0, 0.0, 1.0]).unwrap_or((None, None));
    assert!(front.is_none() || back.is_none());
}

#[test]
fn a_chamfer_of_every_edge_cuts_its_corners_flat() {
    let mut k = DefaultKernel::new();
    let b = cube(&mut k);
    let all: Vec<u64> = (0..k.edges(b).unwrap().len() as u64).collect();
    let (c, _) = chamfer(&mut k, b, &all, 1.0).unwrap();
    // 6 faces, 12 bevels, 8 flat corner triangles (as cadrs's runner's Flat corners).
    assert_eq!(k.get(c).unwrap().faces.len(), 26);
}

#[test]
fn a_shell_is_sharp_and_a_rounded_shell_rounds() {
    let mut k = DefaultKernel::new();
    let b = cube(&mut k);
    let top = k.face_at(b, Point3::new(5.0, 5.0, 10.0)).unwrap().0;
    let (s, _) = shell(&mut k, b, &[top], 1.0, true).unwrap();
    // Sharp outward: the 12 × 12 × 11 box less the 10 × 10 × 10 cavity, open at the top.
    let v = k.mass_properties(s).unwrap().volume;
    assert!((v - (12.0 * 12.0 * 11.0 - 1000.0)).abs() < 1e-6, "{v}");
    let (r, _) = shell_rounded(&mut k, b, &[top], 1.0, true).unwrap();
    assert!(k.mass_properties(r).unwrap().volume < v);
}

#[test]
fn a_fillet_takes_the_whole_tangent_chain() {
    let mut k = DefaultKernel::new();
    let b = k
        .primitive(&Primitive::cylinder(Point3::origin(), nalgebra::Vector3::z(), 5.0, 10.0))
        .and_then(|r| r.first())
        .unwrap();
    let rim = k.edge_at(b, Point3::new(5.0, 0.0, 10.0)).unwrap().0;
    let chain = chained(&k, b, &[rim]).unwrap();
    assert!(chain.iter().all(|&(_, n)| n == 0) && chain.iter().any(|&(e, _)| e == rim));
    assert!(fillet(&mut k, b, &[rim], 1.0).is_ok());
}

#[test]
fn primitives_land_in_their_frame_with_caps_named() {
    let mut k = DefaultKernel::new();
    // A frame at (10, 0, 0) turned a quarter about z: its x is world y.
    let frame = &crate::facade::profile::Place { origin: [10.0, 0.0, 0.0], x: [0.0, 1.0, 0.0], y: [-1.0, 0.0, 0.0], z: [0.0, 0.0, 1.0] };
    let (b, h) = primitive(&mut k, frame, Prim::Block { size: [4.0, 2.0, 3.0] }).unwrap();
    let m = k.mass_properties(b).unwrap();
    assert!((m.volume - 24.0).abs() < 1e-9);
    // Centre: origin + 2 x + 1 y + 1.5 z = (10 - 1, 2, 1.5).
    assert!((m.center_of_mass - Point3::new(9.0, 2.0, 1.5)).norm() < 1e-9, "{:?}", m.center_of_mass);
    let caps = |h: &History| {
        (
            h.generated.iter().filter(|(_, o)| matches!(o, Origin::StartCap { .. })).count(),
            h.generated.iter().filter(|(_, o)| matches!(o, Origin::EndCap { .. })).count(),
        )
    };
    assert_eq!(caps(&h), (1, 1));
    assert_eq!(h.generated.len(), 6);
    let (c, h) = primitive(&mut k, frame, Prim::Cone { bottom: 3.0, top: 1.0, height: 2.0 }).unwrap();
    let v = k.mass_properties(c).unwrap().volume;
    assert!((v - std::f64::consts::PI * 2.0 / 3.0 * (9.0 + 3.0 + 1.0)).abs() < 1e-9, "{v}");
    assert_eq!(caps(&h), (1, 1));
    let (s, h) = primitive(&mut k, frame, Prim::Sphere { radius: 2.0 }).unwrap();
    assert!((k.mass_properties(s).unwrap().center_of_mass - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-9);
    assert!(h.generated.iter().all(|(_, o)| matches!(o, Origin::ProfileCurve { curve: 5, .. })));
    let (cy, h) = primitive(&mut k, frame, Prim::Cylinder { radius: 1.0, height: 5.0 }).unwrap();
    assert!((k.mass_properties(cy).unwrap().volume - 5.0 * std::f64::consts::PI).abs() < 1e-9);
    assert_eq!(caps(&h), (1, 1));
}

#[test]
fn a_fine_thread_is_cut_in_time() {
    let mut k = DefaultKernel::new();
    let rod = k
        .primitive(&Primitive::cylinder(Point3::origin(), nalgebra::Vector3::z(), 5.0, 20.0))
        .and_then(|r| r.first())
        .unwrap();
    let bar = k
        .primitive(&Primitive::Box { min: Point3::new(4.5, -8.0, -0.45), max: Point3::new(12.0, 8.0, 0.45) })
        .and_then(|r| r.first())
        .unwrap();
    // Three turns (the bench case's 21 take seconds; this test stays quick).
    let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 3.0 * std::f64::consts::TAU, rise_per_turn: 1.0 };
    let (cut, _) = cut_swept(&mut k, rod, bar, &[m]).unwrap();
    let v = k.mass_properties(cut).unwrap().volume;
    assert!(v > 0.0 && v < 500.0 * std::f64::consts::PI, "{v}");
}

#[test]
fn a_pin_swung_through_a_ring_sector_cuts_exactly() {
    let mut k = DefaultKernel::new();
    let spec = crate::CylindricalBox {
        frame: crate::Plane::new(Point3::new(0.0, 0.0, -1.0), nalgebra::Vector3::x(), nalgebra::Vector3::z()),
        length: 2.0,
        inner_radius: 8.0,
        outer_radius: 12.0,
        start_angle: 0.0,
        end_angle: 60f64.to_radians(),
    };
    let block = k.cylindrical_box(&spec).and_then(|r| r.first()).unwrap();
    let pin = k
        .primitive(&Primitive::cylinder(Point3::new(6.0, 0.0, 0.0), nalgebra::Vector3::x(), 0.5, 5.0))
        .and_then(|r| r.first())
        .unwrap();
    let start = k.transform_motion(pin, &Motion::rotation(&Axis::new(Point3::origin(), nalgebra::Vector3::z()), (-140f64).to_radians())).and_then(|r| r.first()).unwrap();
    let m = Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 280f64.to_radians() };
    let (cut, _) = cut_swept(&mut k, block, start, &[m]).unwrap();
    // cadrs's runner's (sweep, then cut) volume; cadrs's own cut_swept gives 53.931 here.
    let v = k.mass_properties(cut).unwrap().volume;
    assert!((v - 53.843407424025).abs() < 1e-6, "{v}");
}

#[test]
fn a_hairline_saw_cut_is_kept() {
    let mut k = DefaultKernel::new();
    let cube = cube(&mut k);
    let saw = k
        .primitive(&Primitive::Box { min: Point3::new(5.0, -1.0, 5.0), max: Point3::new(5.000001, 12.0, 12.0) })
        .and_then(|r| r.first())
        .unwrap();
    let (cut, _) = boolean(&mut k, crate::BoolOp::Subtract, cube, &[saw]).unwrap();
    // The cube's 600 and the slot's two walls, 5 × 10 each... less the top strip.
    let area = k.mass_properties(cut).unwrap().surface_area;
    assert!((area - 700.0).abs() < 1e-3, "{area}");
    // The session's own tolerances are back.
    assert_eq!(k.tolerances(), crate::native::tol::Tolerances::default());
}

#[test]
fn a_lens_thinner_than_the_tolerance_is_refused_not_empty() {
    let mut k = DefaultKernel::new();
    let ball = |k: &mut DefaultKernel, x: f64| {
        k.primitive(&Primitive::Sphere {
            center: Point3::new(x, 0.0, 0.0),
            radius: 5.0,
            axis: nalgebra::Unit::new_normalize(nalgebra::Vector3::z()),
            seam: nalgebra::Unit::new_normalize(nalgebra::Vector3::x()),
        })
        .and_then(|r| r.first())
        .unwrap()
    };
    let (a, b) = (ball(&mut k, 0.0), ball(&mut k, 9.999999));
    match boolean(&mut k, crate::BoolOp::Intersect, a, &[b]) {
        Ok((lens, _)) => {
            let v = k.mass_properties(lens).unwrap().volume;
            assert!(v > 0.0 && v < 1e-9, "{v}");
        }
        Err(KernelError::EmptyResult(_)) => panic!("an empty answer for solids that overlap"),
        Err(_) => {}
    }
}

#[test]
fn a_flat_slab_is_a_block_and_an_affine_map_stretches() {
    let mut k = DefaultKernel::new();
    let world = crate::facade::profile::Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    // Every coefficient 2: a flat top at z = 2 over a 4 × 3 rectangle.
    let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let heights = vec![vec![2.0; 3]; 3];
    let (s, _) = slab(&mut k, &world, [4.0, 3.0], &knots, &knots, &heights).unwrap();
    let v = k.mass_properties(s).unwrap().volume;
    assert!((v - 24.0).abs() < 1e-6, "{v}");
    let b = cube(&mut k);
    let (t, _) = transform(&mut k, b, [[2.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).unwrap();
    assert!((k.mass_properties(t).unwrap().volume - 2000.0).abs() < 1e-6);
    let (r, _) = transform(&mut k, b, [[0.0, -1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).unwrap();
    assert!((k.mass_properties(r).unwrap().center_of_mass - Point3::new(-5.0, 5.0, 5.0)).norm() < 1e-9);
    // A mirror stays a valid solid of the same volume; a uniform scale cubes the volume.
    let (m, _) = transform(&mut k, b, [[-1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]).unwrap();
    assert!((k.mass_properties(m).unwrap().volume - 1000.0).abs() < 1e-6);
    assert!(k.get(m).unwrap().check().is_empty());
    let (u, _) = transform(&mut k, b, [[2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0]]).unwrap();
    assert!((k.mass_properties(u).unwrap().volume - 8000.0).abs() < 1e-6);
}

#[test]
fn a_conic_blend_of_a_square_edge() {
    let mut k = DefaultKernel::new();
    let b = k
        .primitive(&Primitive::Box { min: Point3::origin(), max: Point3::new(20.0, 10.0, 10.0) })
        .and_then(|r| r.first())
        .unwrap();
    let e = {
        use crate::Pick;
        k.edge_at(b, Point3::new(20.0, 5.0, 10.0)).unwrap().0
    };
    let (c, _) = fillet_conic(&mut k, b, &[e], 3.0, 0.5).unwrap();
    // A parabola tangent to both faces at the contacts cuts a third of the corner's triangle
    // (3 × 3 / 2) off the corner: 1.5 per unit length, 10 long.
    let removed = 2000.0 - k.mass_properties(c).unwrap().volume;
    assert!((removed - 15.0).abs() < 1e-6, "{removed}");
}

#[test]
fn a_conic_of_shoulder_root_two_less_one_is_a_quarter_circle() {
    let mut k = DefaultKernel::new();
    let b = k
        .primitive(&Primitive::Box { min: Point3::origin(), max: Point3::new(20.0, 10.0, 10.0) })
        .and_then(|r| r.first())
        .unwrap();
    let e = {
        use crate::Pick;
        k.edge_at(b, Point3::new(20.0, 5.0, 10.0)).unwrap().0
    };
    let (c, _) = fillet_conic(&mut k, b, &[e], 3.0, std::f64::consts::SQRT_2 - 1.0).unwrap();
    let removed = 2000.0 - k.mass_properties(c).unwrap().volume;
    let want = 9.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 10.0;
    assert!((removed - want).abs() < 1e-6, "{removed} {want}");
}

#[test]
fn held_meshes_close_and_lie_within_their_chord() {
    let mut k = DefaultKernel::new();
    let world = crate::facade::profile::Place { origin: [0.0; 3], x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    let (s, _) = primitive(&mut k, &world, Prim::Sphere { radius: 2.0 }).unwrap();
    let (cy, _) = primitive(&mut k, &world, Prim::Cylinder { radius: 1.0, height: 3.0 }).unwrap();
    for (b, chord) in [(s, 1e-3), (cy, 1e-2)] {
        let (p, t) = mesh(&k, b, chord).unwrap();
        // Closed and welded: every side twice, once each way; Euler characteristic 2.
        let mut sides = std::collections::BTreeMap::new();
        for tri in &t {
            for i in 0..3 {
                *sides.entry((tri[i], tri[(i + 1) % 3])).or_insert(0) += 1;
            }
        }
        assert!(sides.iter().all(|(&(a, b), &n)| n == 1 && sides.get(&(b, a)) == Some(&1)));
        assert_eq!(p.len() as i64 - sides.len() as i64 / 2 + t.len() as i64, 2);
        // Outward: the signed volume is the solid's, near enough.
        let v: f64 = t.iter().map(|&[a, b, c]| {
            let (a, b, c) = (p[a as usize], p[b as usize], p[c as usize]);
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        }).sum();
        let exact = k.mass_properties(b).unwrap().volume;
        assert!((v - exact).abs() < 0.01 * exact, "{v} {exact}");
    }
    // The sphere's points on it, its triangles' middles within the chord.
    let (p, t) = mesh(&k, s, 1e-3).unwrap();
    assert!(p.iter().all(|q| ((q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt() - 2.0).abs() < 1e-9));
    assert!(t.iter().all(|&[a, b, c]| {
        let m: Vec<f64> = (0..3).map(|i| (p[a as usize][i] + p[b as usize][i] + p[c as usize][i]) / 3.0).collect();
        2.0 - (m[0] * m[0] + m[1] * m[1] + m[2] * m[2]).sqrt() <= 1e-3
    }));
    assert!(mesh(&k, s, 0.0).is_err() && mesh(&k, s, f64::NAN).is_err());
    let (faces, edges) = display(&k, cy, 1e-2, 1e-3).unwrap();
    assert_eq!(faces.len(), k.get(cy).unwrap().faces.len());
    assert!(faces.iter().all(|(p, n, t)| p.len() == n.len() && !t.is_empty() && n.iter().all(|v| ((v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt() - 1.0).abs() < 1e-9)));
    assert!(edges.len() >= 2 && edges.iter().all(|e| e.len() >= 2));
}

#[test]
fn a_crossed_boolean_of_swept_tori_records_its_gaps_as_tolerances() {
    // brep-bench swept/tumbling-torus-crossed-intersection: the two sweeps' intersection
    // p-curves lie up to 8.1e-5 off their edges, within the boolean's 1e-4 merge distance.
    use crate::facade::motion::{together, Step};
    use crate::native::topo::CheckLevel;
    let (l, a, h) = (45f64.to_radians(), 20f64.to_radians(), 0.7071);
    let q = [0.0, h * l.cos(), h * l.sin()];
    let u = [a.cos(), a.sin() * l.cos(), a.sin() * l.sin()];
    let mut k = DefaultKernel::new();
    let (ring, _) = torus(&mut k, [3.0, 0.0, 2.0], [0.0, 0.0, 1.0], 1.0, 0.22).unwrap();
    let mut sweep = |s: f64| {
        let steps = [
            Step::Turn { point: [2.0, 0.0, 0.0], axis: [0.0, 0.0, 1.0], from: -70f64.to_radians(), to: 70f64.to_radians() },
            Step::Turn { point: q, axis: u, from: (-100.0 * s).to_radians(), to: (100.0 * s).to_radians() },
        ];
        together(&mut k, ring, &steps).unwrap().made.0
    };
    let (first, second) = (sweep(-1.0), sweep(1.0));
    let (r, _) = boolean(&mut k, crate::BoolOp::Intersect, first, &[second]).unwrap();
    let b = k.get(r).unwrap();
    assert!(b.check_with(CheckLevel::Full).is_empty(), "{:?}", b.check_with(CheckLevel::Full));
    let most = b.edges.iter().map(|e| e.tol).fold(0.0, f64::max);
    assert!(most > 6e-5 && most < 1e-4, "{most}");
    assert!((k.mass_properties(r).unwrap().volume - 4.616577580202968).abs() < 1e-6);
    // A boolean that passes as it is keeps its tolerances.
    let at = |o: [f64; 3]| crate::facade::profile::Place { origin: o, x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0] };
    let (x, _) = primitive(&mut k, &at([0.0; 3]), Prim::Block { size: [2.0; 3] }).unwrap();
    let (y, _) = primitive(&mut k, &at([1.0; 3]), Prim::Block { size: [2.0; 3] }).unwrap();
    let (z, _) = boolean(&mut k, crate::BoolOp::Union, x, &[y]).unwrap();
    assert!(k.get(z).unwrap().edges.iter().all(|e| e.tol <= crate::native::tol::POINT));
    // The rules: a raised tolerance under a tenth of its edge, under half the shortest edge, a
    // vertex's within the merge distance; each refusal names its rule.
    let (thin, _) = primitive(&mut k, &at([0.0; 3]), Prim::Block { size: [2.0, 2.0, 0.01] }).unwrap();
    let was = (**k.get(thin).unwrap()).clone();
    let long = was.edges.iter().position(|e| super::edge_length(e) > 1.0).unwrap();
    let short = was.edges.iter().position(|e| super::edge_length(e) < 0.02).unwrap();
    let raised = |e: usize, tol: f64| { let mut b = was.clone(); b.edges[e].tol = tol; b };
    assert_eq!(super::blocked(&raised(long, 4e-3), &was, 1e-4), None);
    let why = super::blocked(&raised(short, 2e-3), &was, 1e-4).unwrap();
    assert!(why.contains("a tenth of its length"), "{why}");
    let why = super::blocked(&raised(long, 6e-3), &was, 1e-4).unwrap();
    assert!(why.contains("half its shortest edge"), "{why}");
    let mut v = was.clone();
    v.verts[0].tol = 2e-4;
    let why = super::blocked(&v, &was, 1e-4).unwrap();
    assert!(why.contains("beyond the merge distance"), "{why}");
    assert_eq!(super::blocked(&v, &was, 1e-3), None);
}
