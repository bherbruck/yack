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

/// Rule 7, landing 2: refusals that came back Failed or Unsupported, now Invalid with their arg.
#[test]
fn landing_two_requirements_are_invalid_with_their_arg() {
    let b = cube();
    let e = b.edge_at([4.0, 2.5, 6.0]).unwrap();
    let (top, side, bottom, front) = (
        b.face_at([2.0, 2.5, 6.0]).unwrap(),
        b.face_at([4.0, 2.5, 3.0]).unwrap(),
        b.face_at([2.0, 2.5, 0.0]).unwrap(),
        b.face_at([2.0, 0.0, 3.0]).unwrap(),
    );
    // the top and the bottom share no edge
    let sq = Profile::rectangle([2.0, 0.0], [3.0, 1.0]).unwrap();
    let f = &Frame::WORLD;
    invalid_arg("block with a NaN side (api.md:246)", block(f, [1.0, f64::NAN, 1.0]), "size");
    invalid_arg("screw by 0° (api.md:378)", screw(f, &sq, Lead::per_turn(3.0), deg(0.0)), "angle");
    invalid_arg("loft of no height (api.md:403)", loft(f, &sq, 0.5, 0.0), "height");
    invalid_arg("fillet_conic with no setback (api.md:748)", b.fillet_conic(&[e], 0.0, 0.5), "setback");
    invalid_arg("fillet_conic with rho 1 (api.md:749)", b.fillet_conic(&[e], 1.0, 1.0), "rho");
    invalid_arg("fillet_setback inside its radius (api.md:789)", b.fillet_setback(&[e], 1.0, 0.5), "setback");
    invalid_arg("fillet_setback_g2 inside its radius (api.md:813)", b.fillet_setback_g2(&[e], 1.0, 0.5), "setback");
    invalid_arg("fillet_faces of no radius (api.md:840)", b.fillet_faces(top, side, 0.0), "radius");
    invalid_arg("full_round naming a face twice (api.md:859)", b.full_round(side, top, side), "middle");
    invalid_arg("full_round whose middle misses a side (api.md:860)", b.full_round(front, top, bottom), "side_b");
    invalid_arg("shell of no thickness (api.md:1051)", b.shell(&[top], 0.0), "thickness");
    invalid_arg("shell_outward of no thickness (api.md:1051)", b.shell_outward(&[top], -1.0), "thickness");
    // a curved face is drafted on purpose (yack extends api.md:1068, as springrep's tests do)
}

/// Rule 7, landing 3: refusals the engine already made as Invalid, now naming their arg.
#[test]
fn landing_three_requirements_name_their_arg() {
    let b = cube();
    let e = b.edge_at([4.0, 2.5, 6.0]).unwrap();
    let e2 = b.edge_at([2.0, 0.0, 6.0]).unwrap();
    let top = b.face_at([2.0, 2.5, 6.0]).unwrap();
    let f = &Frame::WORLD;
    let sq = Profile::rectangle([2.0, 0.0], [3.0, 1.0]).unwrap();
    invalid_arg("cylinder of no height (api.md:263)", cylinder(f, 1.0, 0.0), "height");
    invalid_arg("sphere of negative radius (api.md:280)", sphere(f, -1.0), "radius");
    invalid_arg("torus with no tube (api.md:297)", torus(f, 5.0, 0.0), "minor");
    invalid_arg("cone of no height (api.md:315)", cone(f, 1.0, 0.5, 0.0), "height");
    invalid_arg("cone of negative bottom (api.md:315)", cone(f, -1.0, 0.5, 2.0), "bottom");
    invalid_arg("cone with no radius (api.md:315)", cone(f, 0.0, 0.0, 2.0), "bottom");
    invalid_arg("extrude of NaN height (api.md:334)", extrude(f, &sq, f64::NAN), "height");
    invalid_arg("slab with a negative height (api.md:520)", slab(f, [4.0, 4.0], &[0.0, 0.0, 1.0, 1.0], &[0.0, 0.0, 1.0, 1.0], &[vec![1.0, 1.0], vec![1.0, -1.0]]), "heights");
    invalid_arg("fillet of negative radius (api.md:659)", b.fillet(&[e], -1.0), "radius");
    invalid_arg("fillet_radii with one negative (api.md:703)", b.fillet_radii(&[(e, 1.0), (e2, -1.0)]), "edges");
    invalid_arg("fillet_variable ending at zero (api.md:723)", b.fillet_variable(e, [4.0, 0.0, 6.0], 1.0, 0.0), "r_to");
    invalid_arg("fillet_g2 of negative offset (api.md:773)", b.fillet_g2(&[e], -1.0, 1.4), "offset");
    invalid_arg("fillet_faces naming one face twice (api.md:841)", b.fillet_faces(top, top, 1.0), "b");
    invalid_arg("chamfer of NaN (api.md:876)", b.chamfer(&[e], f64::NAN), "distance");
    invalid_arg("chamfer_two of no setback (api.md:897)", b.chamfer_two(e, top, 1.0, 0.0), "along_other");
    invalid_arg("chamfer_distances with one negative (api.md:913)", b.chamfer_distances(&[(e, 1.0), (e2, -1.0)]), "edges");
    invalid_arg("mesh at tolerance zero (api.md:1577)", b.mesh(0.0), "tolerance");
    invalid_arg("display at NaN tolerance (api.md:1577)", b.display(f64::NAN), "tolerance");
    // yack's own extensions stay accepted: a negative extrude height
    assert!(extrude(f, &sq, -1.0).is_ok());
}
