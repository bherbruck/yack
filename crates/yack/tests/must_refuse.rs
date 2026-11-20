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
    refused("screw turns that meet", screw(&Frame::WORLD, &sq, 0.01, deg(720.0)));
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
