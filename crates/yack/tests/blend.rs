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

#[test]
fn a_dead_solids_ids_stay_foreign() {
    let stale = {
        let b = ten();
        b.edge_at([5.0, 0.0, 10.0]).unwrap()
    };
    for _ in 0..50 {
        let fresh = ten();
        assert_eq!(fresh.fillet(&[stale], 1.0).unwrap_err().kind, ErrorKind::ForeignId);
        assert_eq!(fresh.edge(stale).unwrap_err().kind, ErrorKind::ForeignId);
    }
}

#[test]
fn picks_on_shared_boundaries_are_ambiguous() {
    let b = ten();
    assert_eq!(b.edge_at([0.0, 0.0, 10.0]).unwrap_err().kind, ErrorKind::Ambiguous);
    assert_eq!(b.face_at([5.0, 0.0, 10.0]).unwrap_err().kind, ErrorKind::Ambiguous);
}

#[test]
fn convexity_tells_convex_concave_and_tangent() {
    let b = ten();
    assert!(b.edges_where(|e| e.convex() == Some(true)).len() == 12);
    let l = b.union(&block(&Frame::WORLD, [20.0, 10.0, 5.0]).unwrap()).unwrap();
    let concave = l.edges_where(|e| e.convex() == Some(false));
    assert_eq!(concave.len(), 1, "{concave:?}");
    let e = b.edge_at([5.0, 0.0, 10.0]).unwrap();
    let r = b.fillet(&[e], 2.0).unwrap();
    assert_eq!(r.edges_where(|e| e.convex().is_none()).len(), 2);
}

#[test]
fn face_refs_carry_tags_normals_and_samples() {
    let b = ten();
    let r = b.fillet(&[b.edge_at([5.0, 0.0, 10.0]).unwrap()], 2.0).unwrap().named("round");
    let round = r.faces_tagged("round")[0];
    let f = r.face(round).unwrap();
    assert_eq!(f.tags(), &[yack::Tag::from("round")]);
    let grid = f.sample(5, 3);
    assert_eq!((grid.len(), grid[0].len()), (5, 3));
    let top = r.face(r.face_at([5.0, 5.0, 10.0]).unwrap()).unwrap();
    let n = top.normal_at([5.0, 5.0, 10.0]).unwrap();
    assert!((n.z - 1.0).abs() < 1e-12);
    let e = r.edge(r.edges_of(round).unwrap()[0]).unwrap();
    assert_eq!(e.points(9).len(), 9);
}
