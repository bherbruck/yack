// yack: springrep module src/edit/tweak/tests.rs
// Adapted from springrep@a03e8a1 src/edit/tweak/tests.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from edit/tweak/tests.rs to tweak/tests.rs (M2.4, T6); paths only; then to yack-tweak (M3 W7), paths and visibility only.
use super::*;
use crate::offset::{OFFSET_FIT, offset_fitted};
use crate::planes::move_planes;
use crate::math::Vec3;
use crate::topo::FaceName;
use yack_measure as measure;

/// The octant of the sphere of radius `r` about the origin, as the rational biquadratic of
/// Piegl and Tiller. A copy of yack-geom's test helper, whose cfg(test) items another crate's
/// tests cannot see (M3 W3).
fn octant(r: f64) -> crate::geom::Surface {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let equator = [(r, 0.0, 1.0), (r, r, w), (0.0, r, 1.0)];
    let mut ctrl = Vec::new();
    let mut weights = Vec::new();
    for &(x, y, wu) in &equator {
        for (j, wv) in [1.0, w, 1.0].into_iter().enumerate() {
            ctrl.push(match j {
                0 => crate::math::Vec3::new(x, y, 0.0),
                1 => crate::math::Vec3::new(x, y, r),
                _ => crate::math::Vec3::new(0.0, 0.0, r),
            });
            weights.push(wu * wv);
        }
    }
    let k = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let n = crate::geom::NurbsSurface::new((2, 2), (k.clone(), k), (3, 3), ctrl, Some(weights)).unwrap();
    crate::geom::Surface::Nurbs(Box::new(n))
}

fn cube() -> Brep {
    // cube10 fixture: sweep::extrude's 10 mm cube, proven by w-m04-fix-1.
    fixture("cube10")
}

/// A solid sweep::extrude built, read from its fixture (extrude sits beside yack-tweak, M3 W7): the
/// same solid bit for bit, as the printer that wrote each one proved.
fn fixture(name: &str) -> Brep {
    let bytes: &[u8] = match name {
        "cube10" => include_bytes!("../tests/data/cube10.yack"),
        "box20" => include_bytes!("../tests/data/box20.yack"),
        "rod10" => include_bytes!("../tests/data/rod10.yack"),
        "tip" => include_bytes!("../tests/data/tip.yack"),
        _ => panic!("no fixture {name}"),
    };
    yack_codec::from_bytes(bytes).and_then(|mut v| v.pop()).unwrap()
}

fn face(b: &Brep, name: &str) -> FaceId {
    let n = FaceName::new("box", name);
    b.face_ids()
        .find(|&f| b.face(f).names.contains(&n))
        .unwrap()
}

fn frame(b: &Brep, f: FaceId) -> Frame {
    plane_of(&b.face(f).surface).unwrap()
}

#[test]
#[allow(clippy::disallowed_types)] fn raising_the_top_stretches_the_sides() { // D26: BTree* in hardening, proven by digests
    let b = cube();
    let top = face(&b, "end");
    let fr = frame(&b, top).translated(Vec3::Z * 5.0);
    let out = move_planes(&b, &HashMap::from([(top, fr)])).unwrap();
    assert!((measure::volume(&out) - 1500.0).abs() < 1e-9);
    assert_eq!(out.face_count(), 6);
}

#[test]
#[allow(clippy::disallowed_types)] fn tilting_a_side_about_its_foot_takes_off_a_wedge() { // D26: BTree* in hardening, proven by digests
    let b = cube();
    let side = face(&b, "u_max");
    let old = frame(&b, side);
    let n = old.z;
    let tilted = (n * 10.0 + Vec3::Z * 2.0).unit().unwrap();
    let foot = old.origin - Vec3::Z * old.origin.z;
    let fr = Frame::new(foot, tilted, old.x).unwrap();
    let out = move_planes(&b, &HashMap::from([(side, fr)])).unwrap();
    assert!((measure::volume(&out) - 900.0).abs() < 1e-9);
}

#[test]
#[allow(clippy::disallowed_types)] fn a_draft_past_the_apex_is_refused() { // D26: BTree* in hardening, proven by digests
    // box20 fixture: sweep::extrude's 20 x 20 x 10 box, proven by w-m04-fix-2.
    let b = fixture("box20");
    let drafted = |a: f64| {
        let mut moved = HashMap::new();
        for name in ["u_min", "u_max", "v_min", "v_max"] {
            let f = face(&b, name);
            let fr = frame(&b, f);
            let n = if b.face(f).reversed { -fr.z } else { fr.z };
            let tilted = n * a.cos() + Vec3::Z * a.sin();
            let z = if b.face(f).reversed { -tilted } else { tilted };
            let foot = fr.origin - Vec3::Z * fr.origin.z;
            moved.insert(f, Frame::new(foot, z, fr.x).unwrap());
        }
        move_planes(&b, &moved)
    };
    let top = 20.0 - 20.0 * 0.5f64.tan();
    let want = 10.0 / 3.0 * (400.0 + top * top + 20.0 * top);
    assert!((measure::volume(&drafted(0.5).unwrap()) - want).abs() < 1e-9);
    for a in [std::f64::consts::FRAC_PI_4 + 1e-6, 0.79, 0.9] {
        assert!(matches!(drafted(a), Err(Error::InvalidResult(_))), "{a}");
    }
}

#[test]
#[allow(clippy::disallowed_types)] fn a_curved_face_at_a_moved_corner_is_unsupported() { // D26: BTree* in hardening, proven by digests
    // rod10 fixture: sweep::extrude's rod of radius 10, 10 long, proven by w-m04-fix-2.
    let b = fixture("rod10");
    let top = face(&b, "end");
    let fr = frame(&b, top).translated(Vec3::Z * 5.0);
    assert!(matches!(
        move_planes(&b, &HashMap::from([(top, fr)])),
        Err(Error::Unsupported(_))
    ));
}

#[test]
#[allow(clippy::disallowed_types)] fn a_tip_moved_through_the_other_tip_is_refused() { // D26: BTree* in hardening, proven by digests
    // tip fixture: sweep::extrude's 14-sided prism, 5 tall, proven by w-m04-fix-2.
    let b = fixture("tip");
    let tip = face(&b, "e2");
    let raised = |dy: f64| {
        let fr = frame(&b, tip).translated(Vec3::Y * dy);
        move_planes(&b, &HashMap::from([(tip, fr)]))
    };
    let v0 = measure::volume(&b);
    assert!((measure::volume(&raised(5.0).unwrap()) - v0 - 200.0).abs() < 1e-9);
    assert!(matches!(raised(12.0), Err(Error::InvalidResult(_))));
}

#[test]
fn a_spline_sphere_octant_offset_is_the_octant_of_the_grown_sphere() {
    // The octant of the R5 sphere as a rational B-spline, offset 0.5 along its normal: the fitted
    // bicubic must lie on the R5.5 sphere about the same centre wherever it is sampled, not only at
    // the points it was checked at.
    let s = octant(5.0);
    let crate::geom::Surface::Nurbs(n) = &s else {
        panic!("the octant is a spline")
    };
    let ((u0, u1), (v0, v1)) = n.domain();
    let off = offset_fitted(&s, 0.5, ((u0, u1), (v0, v1))).unwrap();
    let mut worst = 0.0f64;
    for i in 0..=37 {
        for j in 0..=37 {
            let (u, v) = (
                u0 + (u1 - u0) * i as f64 / 37.0,
                v0 + (v1 - v0) * j as f64 / 37.0,
            );
            worst = worst.max((off.eval(u, v).norm() - 5.5).abs());
        }
    }
    assert!(worst <= 2.0 * OFFSET_FIT, "{worst:e}");
}

#[test]
#[allow(clippy::disallowed_types)] fn two_faces_refused_two_ways_refuse_by_the_lower_face_id_every_run() { // D26: BTree* in hardening, proven by digests
    // rod10: its side is a cylinder, which no plane move takes (Unsupported); its end turned over
    // faces the wrong way (InvalidInput). Each new HashMap iterates in its own order.
    let b = fixture("rod10");
    let end = face(&b, "end");
    let side = b.face_ids().find(|&f| plane_of(&b.face(f).surface).is_none()).unwrap();
    let fr = frame(&b, end);
    let over = Frame::new(fr.origin, -fr.z, fr.x).unwrap();
    for _ in 0..16 {
        let moved: HashMap<FaceId, Frame> = [(side, fr), (end, over)].into_iter().collect();
        let got = move_planes(&b, &moved);
        if side < end {
            assert!(matches!(got, Err(Error::Unsupported(_))), "{:?}", got.err());
        } else {
            assert!(matches!(got, Err(Error::InvalidInput(_))), "{:?}", got.err());
        }
    }
}
