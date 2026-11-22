// yack: the protocol's must-refuse inputs (brep-conformance doc/api.md, "Refusals") are refused as
// invalid or failed, whichever engine decides: never unsupported, never a solid.
#![allow(clippy::unwrap_used, clippy::disallowed_methods, clippy::panic)] // test helpers; the facade rules are for src/
use yack::prelude::*;

fn refused(what: &str, r: Result<Solid>) {
    let ok = match &r {
        Ok(s) => {
            eprintln!("MUSTREFUSE {what}: built a solid (strategy {})", s.strategy());
            false
        }
        Err(e) => {
            eprintln!("MUSTREFUSE {what}: {:?}: {e}", e.kind);
            matches!(e.kind, ErrorKind::Invalid | ErrorKind::Failed)
        }
    };
    assert!(ok, "{what}");
}

fn cube() -> Solid {
    block(&Frame::WORLD, [4.0, 5.0, 6.0]).unwrap()
}

#[test]
fn constructors() {
    refused("box side of zero", block(&Frame::WORLD, [1.0, 0.0, 1.0]));
    refused("cylinder radius below zero", cylinder(&Frame::WORLD, -1.0, 2.0));
    refused("sphere radius of zero", sphere(&Frame::WORLD, 0.0));
    refused("tube wider than its ring", torus(&Frame::WORLD, 1.0, 2.0));
    refused("cone with no radius", cone(&Frame::WORLD, 0.0, 0.0, 2.0));
    let sq = Profile::rectangle([1.0, 0.0], [2.0, 1.0]).unwrap();
    refused("extrude of height zero", extrude(&Frame::WORLD, &sq, 0.0));
    let across = Profile::rectangle([-1.0, 0.0], [1.0, 1.0]).unwrap();
    refused("revolve across the axis", revolve(&Frame::WORLD, &across, deg(180.0)));
    refused("screw turns that meet", screw(&Frame::WORLD, &sq, Lead::per_radian(0.01), deg(720.0)));
    refused("loft heights that do not rise", loft_through(&Frame::WORLD, &[(1.0, sq.clone()), (0.0, sq)]));
}

#[test]
fn blends_and_edits() {
    let c = cube();
    let e = c.edge_at([2.0, 0.0, 6.0]).unwrap();
    refused("fillet too big for its faces", c.fillet(&[e], 10.0));
    refused("fillet radius of zero", c.fillet(&[e], 0.0));
    refused("chamfer too big for its faces", c.chamfer(&[e], 10.0));
    refused("chamfer distance below zero", c.chamfer(&[e], -1.0));
    refused("one of several radii too big", c.fillet_radii(&[(e, 1.0), (c.edge_at([4.0, 2.5, 6.0]).unwrap(), 10.0)]));
    let top = c.face_at([2.0, 2.5, 6.0]).unwrap();
    refused("shell as thick as the part", c.shell(&[top], 3.0));
    refused("face pushed through the part", c.offset_faces(&[top], -7.0));
}

/// An empty edge or face list for a blend or face edit, or a face the edge does not border, is
/// `Invalid` before any engine runs (a springrep refusal that is not `Invalid` would otherwise fall through to cadrs, which
/// built from such input).
#[test]
fn empty_lists_and_a_foreign_face_are_invalid() {
    let b = cube();
    let invalid = |what: &str, r: Result<Solid>| {
        assert!(matches!(&r, Err(e) if e.kind == ErrorKind::Invalid), "{what}: {:?}", r.map(|s| s.volume()));
    };
    invalid("fillet of no edges", b.fillet(&[], 1.0));
    invalid("chamfer of no edges", b.chamfer(&[], 1.0));
    let edge = b.edge_at([4.0, 2.5, 6.0]).unwrap();
    invalid("chamfer_two on a face the edge does not border", b.chamfer_two(edge, b.face_at([2.0, 2.5, 0.0]).unwrap(), 1.0, 0.5));
    invalid("offset_faces of no faces", b.offset_faces(&[], 1.0));
    invalid("move_faces of no faces", b.move_faces(&[], &Rigid::translation([0.0, 0.0, 1.0])));
    invalid("thicken of no faces", b.thicken(&[], 1.0, Join::Round));
    invalid("draft of no faces", b.draft(&[], [0.0; 3], [0.0, 0.0, 1.0], deg(5.0)));
    invalid("sweep along no path", sweep(&Frame::WORLD, &Profile::rectangle([0.0, 0.0], [1.0, 1.0]).unwrap(), &[]));
    // a boolean's empty cases are identities, not refusals: no tools leaves the solid
    assert!((b.difference_all(&[]).unwrap().volume() - 120.0).abs() < 1e-9);
}

fn invalid_arg<T>(what: &str, r: Result<T>, arg: &str) {
    match r {
        Err(e) => assert!(e.kind == ErrorKind::Invalid && e.arg == Some(arg), "{what}: {e:?}"),
        Ok(_) => panic!("{what}: accepted"),
    }
}

/// Rule 7, landing 1: inputs api.md refuses that yack built, or spun on, before the facade checked.
#[test]
fn landing_one_requirements_are_invalid_with_their_arg() {
    let b = cube();
    let (top, side) = (b.face_at([2.0, 2.5, 6.0]).unwrap(), b.face_at([4.0, 2.5, 3.0]).unwrap());
    let sq = Profile::rectangle([2.0, 0.0], [3.0, 1.0]).unwrap();
    let f = &Frame::WORLD;
    invalid_arg("project at a negative tolerance (api.md:1524)", b.project([0.0, 0.0, 1.0], -0.1), "tolerance");
    invalid_arg("section at tolerance zero (api.md:1498)", b.section(&Plane::new([0.0, 0.0, 3.0], [0.0, 0.0, 1.0]).unwrap(), 0.0), "tolerance");
    invalid_arg("distance to an empty solid (api.md:1428)", b.distance(&empty()), "other");
    invalid_arg("offset of zero (api.md:949)", b.offset(0.0), "distance");
    invalid_arg("offset_faces of zero (api.md:971)", b.offset_faces(&[top], 0.0), "distance");
    invalid_arg("thicken of zero (api.md:1028)", b.thicken(&[top], 0.0, Join::Round), "thickness");
    invalid_arg("draft of 0° (api.md:1069)", b.draft(&[side], [0.0; 3], [0.0, 0.0, 1.0], deg(0.0)), "angle");
    invalid_arg("draft of −5° (api.md:1069)", b.draft(&[side], [0.0; 3], [0.0, 0.0, 1.0], deg(-5.0)), "angle");
    invalid_arg("draft of 90° (api.md:1069)", b.draft(&[side], [0.0; 3], [0.0, 0.0, 1.0], deg(90.0)), "angle");
    invalid_arg("revolve past a turn (api.md:356)", revolve(f, &sq, deg(400.0)), "angle");
    invalid_arg("revolve by 0° (api.md:356)", revolve(f, &sq, deg(0.0)), "angle");
    invalid_arg("sweep along a negative line (api.md:466)", sweep(f, &sq, &[PathPiece::Line { length: -2.0 }]), "path");
    invalid_arg("sweep round a negative radius (api.md:466)", sweep(f, &sq, &[PathPiece::Arc { radius: -10.0, angle: deg(90.0) }]), "path");
    invalid_arg("sweep round no angle (api.md:466)", sweep(f, &sq, &[PathPiece::Arc { radius: 10.0, angle: deg(0.0) }]), "path");
    // the edges of each range are accepted: a full turn, a negative offset (yack's offset is signed)
    assert!(revolve(f, &sq, deg(360.0)).is_ok());
    assert!(b.offset(-0.5).is_ok());
}
