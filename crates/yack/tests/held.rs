// yack: a cadrs answer the bridge cannot carry, held in cadrs form (the lazy bridge).
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use yack::prelude::*;

/// swept/tumbling-torus: cadrs sweeps it exactly; its swept face folds in (u, v), which springrep
/// cannot carry. Held in cadrs form, it is measured, listed and written by cadrs, and a springrep
/// query on it refuses (Unsupported), never answers empty.
fn tumbling_torus() -> Solid {
    tumbling_torus_swept().unwrap()
}

fn tumbling_torus_swept() -> Result<Solid> {
    let ring = torus(&Frame::at([3.0, 0.0, 2.0]), 1.0, 0.5).unwrap();
    let roll = Motion::turn(Point3::new(2.0, 0.0, 0.0), Dir3::Z, deg(-18.75), deg(18.75));
    let tumble = Motion::turn(Point3::new(0.0, 0.5, 0.5), Dir3::X, deg(75.0), deg(-75.0));
    ring.swept(&Motion::together(vec![roll, tumble]))
}

#[test]
fn a_held_solid_is_measured_listed_and_refuses_springrep_queries() {
    let s = tumbling_torus();
    assert_eq!(s.strategy(), "cadrs");
    assert!(s.volume() > 0.0 && s.area() > 0.0, "{} {}", s.volume(), s.area());
    assert!(!s.faces().is_empty() && !s.edges().is_empty());
    assert!(s.check().is_empty(), "{:?}", s.check());
    // one piece; the tumble sweeps the ring through its own hole, closing it; no void
    assert_eq!(s.betti(), [1, 0, 0]);
    let e = s.classify(&[[3.0, 0.0, 2.0]]).unwrap_err();
    assert_eq!(e.kind, ErrorKind::Unsupported, "{e}");
    assert_eq!(s.normal_at(&[[3.0, 0.0, 2.0]]).unwrap_err().kind, ErrorKind::Unsupported);
    assert_eq!(s.curvature_at(&[[3.0, 0.0, 2.0]]).unwrap_err().kind, ErrorKind::Unsupported);
    assert_eq!(yack::codec::encode(&s).unwrap_err().kind, ErrorKind::Unsupported);
}

#[test]
fn a_held_solid_digests_alike_and_writes_cadrs_step() {
    let (a, b) = (tumbling_torus(), tumbling_torus());
    assert_eq!(a.digest(), b.digest());
    assert_ne!(a.digest(), [0; 32]);
    // cadrs's file, kept because cadrs's reader reads it back as the solid; our own step::read
    // (springrep's reader) may refuse it, since springrep cannot carry the folded swept face
    let text = yack::step::write(&a).unwrap();
    assert!(text.starts_with("ISO-10303-21;"));
}

#[test]
fn a_held_solid_takes_a_cadrs_boolean() {
    let s = tumbling_torus();
    let cut = s.difference(&block(&Frame::at([3.0, -10.0, -10.0]), [20.0, 20.0, 30.0]).unwrap()).unwrap();
    assert!(cut.volume() > 0.0 && cut.volume() < s.volume(), "{} of {}", cut.volume(), s.volume());
}

/// tangent/paraboloid-tip-a-ten-thousandth-high (case header: volume 1.2566370614359173e-7): the
/// slab's sliver of a paraboloid's tip. springrep's intersection says empty (tentative), the
/// bridge refuses cadrs's sliver (held): the held sliver is the answer.
#[test]
fn a_paraboloid_tip_is_not_lost() {
    let side = Loop::starting_at([0.0, 0.0]).conic_to([8.0, 8.0], [4.0, 0.0], 1.0).line_to([0.0, 8.0]).close().unwrap();
    let paraboloid = revolve(&Frame::WORLD, &Profile::new(side, vec![]).unwrap(), deg(360.0)).unwrap();
    let tip = paraboloid.intersection(&block(&Frame::at([-9.0, -9.0, -1.0]), [18.0, 18.0, 1.0001]).unwrap()).unwrap();
    assert!(!tip.is_empty());
    let want = 1.2566370614359173e-7;
    assert!((tip.volume() - want).abs() < 1e-5 * want, "{} ({})", tip.volume(), tip.strategy());
}

/// conformance closing-offsets-12002a6de0a0c47b (modify/push-faces-until-they-merge): a U-block's
/// notch pushed shut is the box 10.4856 × 3.0050 × 5.6476. Whichever engine answers, the area is
/// the box's or the op refuses: never cadrs's quadrature off by tens of mm² (it read 234.92).
#[test]
fn a_notch_pushed_shut_reports_the_boxs_area_or_refuses() {
    let (w, h, d) = (10.485590877076831, 5.647614152176731, 3.0049592791799498);
    let u = Profile::polygon(&[[0.0, 0.0], [w, 0.0], [w, h], [6.426407687457797, h], [6.426407687457797, 2.399759974379497], [2.1996469896794357, 2.399759974379497], [2.1996469896794357, h], [0.0, h]]).unwrap();
    let f = Frame::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]).unwrap();
    let block = extrude(&f, &u, d).unwrap();
    let wall = block.face_at([2.1996469896794357, -1.5024796395899749, 4.023687063278114]).unwrap();
    match block.offset_faces(&[wall], 4.226760697778361) {
        Ok(s) => {
            let area = 2.0 * (w * d + d * h + h * w);
            assert!((s.volume() - w * h * d).abs() < 1e-6 * w * h * d, "{}", s.volume());
            assert!((s.area() - area).abs() < 1e-6 * area, "area {} against the box's {area} ({})", s.area(), s.strategy());
        }
        Err(e) => assert_eq!(e.kind, ErrorKind::Unsupported, "{e}"),
    }
}


/// Face and edge selection on a held solid come from cadrs's own topology, and never answer
/// "nothing" for "can't look": every face and edge view has real geometry.
#[test]
fn a_held_solid_selects_faces_and_edges_through_cadrs() {
    let s = tumbling_torus();
    let faces = s.faces_where(|_| true).unwrap();
    assert_eq!(faces.len(), s.faces().len());
    for &id in &faces {
        let f = s.face(id).unwrap();
        assert!(f.area() > 0.0, "{f:?}");
        let grid = f.sample(3, 3);
        assert_eq!(grid.len(), 3);
        let on = f.closest(f.centroid()).unwrap();
        // a point of the face picks it, or an edge-near point is ambiguous: never another face
        assert!(s.face_at(on).map_or(true, |hit| hit == id), "{id} at {on:?}");
        let n = f.normal_at(on);
        assert!(n.is_some_and(|n| (n.x * n.x + n.y * n.y + n.z * n.z - 1.0).abs() < 1e-6), "normal {n:?}");
    }
    let edges = s.edges_where(|_| true).unwrap();
    assert!(!edges.is_empty());
    for &id in &edges {
        let e = s.edge(id).unwrap();
        let [t0, t1] = e.range();
        assert!(t0 < t1, "{e:?} {t0} {t1}");
        assert!(e.length() > 0.0 && e.points(3).len() == 3, "{e:?}");
    }
}
