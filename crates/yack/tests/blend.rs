// yack: T1 checks of fillets and chamfers through the public API, against closed forms.
#![allow(clippy::unwrap_used, clippy::disallowed_methods)] // test helpers; the facade rules are for src/
use std::f64::consts::PI;

use yack::prelude::*;

fn close(got: f64, want: f64) {
    assert!((got - want).abs() <= 1e-6 * want.abs().max(1.0), "{got} vs {want}");
}

fn ten() -> Solid {
    block(&Frame::WORLD, [10.0, 10.0, 10.0]).unwrap()
}

#[test]
fn one_rounded_edge_loses_the_corner_sliver() {
    let b = ten();
    let e = b.edge_at([5.0, 0.0, 10.0]).unwrap();
    let r = b.fillet(&[e], 2.0).unwrap();
    assert!(r.check().is_empty());
    close(r.volume(), 1000.0 - (4.0 - PI) * 10.0);
}

#[test]
fn one_chamfered_edge_loses_a_prism() {
    let b = ten();
    let e = b.edge_at([5.0, 0.0, 10.0]).unwrap();
    let c = b.chamfer(&[e], 2.0).unwrap();
    assert!(c.check().is_empty());
    close(c.volume(), 1000.0 - 2.0 * 10.0);
    let top = b.face_at([5.0, 5.0, 10.0]).unwrap();
    let c2 = b.chamfer_two(e, top, 1.0, 3.0).unwrap();
    close(c2.volume(), 1000.0 - 1.5 * 10.0);
}

#[test]
fn foreign_ids_are_refused() {
    let (a, b) = (ten(), ten());
    let e = a.edge_at([5.0, 0.0, 10.0]).unwrap();
    assert_eq!(b.fillet(&[e], 1.0).unwrap_err().kind, ErrorKind::ForeignId);
    assert_eq!(a.edge_at([5.0, 5.0, 5.0]).unwrap_err().kind, ErrorKind::NotFound);
}

#[test]
fn faces_meet_at_their_edge() {
    let b = ten();
    let top = b.face_at([5.0, 5.0, 10.0]).unwrap();
    let front = b.face_at([5.0, 0.0, 5.0]).unwrap();
    let between = b.edges_between(top, front).unwrap();
    assert_eq!(between, vec![b.edge_at([5.0, 0.0, 10.0]).unwrap()]);
    let mut of = b.faces_of(between[0]).unwrap();
    of.sort();
    let mut want = vec![top, front];
    want.sort();
    assert_eq!(of, want);
}
