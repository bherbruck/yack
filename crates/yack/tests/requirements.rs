// yack: a probe of how each brep-conformance requirement (doc/api.md) is refused today. It prints one
// line per case (`REQ <api line> <case>: <outcome>`) and asserts nothing; run with `--ignored
// --nocapture`. The table it feeds is swarm/done/w-m03-rule7-table.md; the asserts that come out of
// it live in must_refuse.rs.
#![allow(clippy::unwrap_used, clippy::disallowed_methods, clippy::panic, clippy::print_stderr)] // test helpers
use yack::prelude::*;

fn report<T>(line: u32, what: &str, r: Result<T>) {
    let outcome = match &r {
        Ok(_) => "BUILT".to_string(),
        Err(e) => format!("{:?} arg={:?} op={}", e.kind, e.arg, e.op),
    };
    eprintln!("REQ {line} {what}: {outcome}");
}

fn cube() -> Solid {
    block(&Frame::WORLD, [4.0, 5.0, 6.0]).unwrap()
}

fn sq() -> Profile {
    Profile::rectangle([2.0, 0.0], [3.0, 1.0]).unwrap()
}

#[test]
#[ignore = "a probe: run with --ignored --nocapture"]
fn probe_core_constructors() {
    let f = &Frame::WORLD;
    report(246, "box side negative", block(f, [1.0, -1.0, 1.0]));
    report(246, "box side NaN", block(f, [1.0, f64::NAN, 1.0]));
    report(263, "cylinder height zero", cylinder(f, 1.0, 0.0));
    report(280, "sphere radius negative", sphere(f, -1.0));
    report(297, "torus minor zero", torus(f, 5.0, 0.0));
    report(315, "cone height zero", cone(f, 1.0, 0.5, 0.0));
    report(315, "cone bottom negative", cone(f, -1.0, 0.5, 2.0));
    report(334, "extrude height NaN", extrude(f, &sq(), f64::NAN));
    report(356, "revolve angle zero", revolve(f, &sq(), deg(0.0)));
    report(356, "revolve angle past a turn", revolve(f, &sq(), deg(400.0)));
    report(378, "screw angle zero", screw(f, &sq(), Lead::per_turn(3.0), deg(0.0)));
    report(403, "loft height zero", loft(f, &sq(), 0.5, 0.0));
    // a square with a square hole (holes run clockwise)
    let holed = Profile::new(Loop::polygon(&[[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]).unwrap(), vec![Loop::polygon(&[[1.0, 1.0], [1.0, 2.0], [2.0, 2.0], [2.0, 1.0]]).unwrap()]);
    match holed {
        Ok(h) => report(403, "loft two loops", loft(f, &h, 0.5, 2.0)),
        Err(e) => eprintln!("REQ 403 loft two loops: SETUP {e}"),
    }
    report(430, "loft_through one profile", loft_through(f, &[(0.0, sq())]));
    report(466, "sweep line length negative", sweep(f, &sq(), &[PathPiece::Line { length: -2.0 }]));
    report(466, "sweep arc angle zero", sweep(f, &sq(), &[PathPiece::Arc { radius: 10.0, angle: deg(0.0) }]));
    report(466, "sweep arc radius negative", sweep(f, &sq(), &[PathPiece::Arc { radius: -10.0, angle: deg(90.0) }]));
    report(520, "slab height negative", slab(f, [4.0, 4.0], &[0.0, 0.0, 1.0, 1.0], &[0.0, 0.0, 1.0, 1.0], &[vec![1.0, 1.0], vec![1.0, -1.0]]));
}

#[test]
#[ignore = "a probe: run with --ignored --nocapture"]
fn probe_core_blends_and_edits() {
    let f = &Frame::WORLD;
    let b = cube();
    let e = b.edge_at([4.0, 2.5, 6.0]).unwrap();
    let e2 = b.edge_at([2.0, 0.0, 6.0]).unwrap();
    let (top, side, bottom, front) = (
        b.face_at([2.0, 2.5, 6.0]).unwrap(),
        b.face_at([4.0, 2.5, 3.0]).unwrap(),
        b.face_at([2.0, 2.5, 0.0]).unwrap(),
        b.face_at([2.0, 0.0, 3.0]).unwrap(),
    );
    report(659, "fillet radius negative", b.fillet(&[e], -1.0));
    report(659, "fillet radius NaN", b.fillet(&[e], f64::NAN));
    report(703, "fillet_radii one radius negative", b.fillet_radii(&[(e, 1.0), (e2, -1.0)]));
    report(723, "fillet_variable r_to zero", b.fillet_variable(e, [4.0, 0.0, 6.0], 1.0, 0.0));
    report(748, "fillet_conic setback zero", b.fillet_conic(&[e], 0.0, 0.5));
    report(749, "fillet_conic rho one", b.fillet_conic(&[e], 1.0, 1.0));
    report(749, "fillet_conic rho zero", b.fillet_conic(&[e], 1.0, 0.0));
    report(773, "fillet_g2 offset negative", b.fillet_g2(&[e], -1.0, 1.4));
    report(789, "fillet_setback setback below radius", b.fillet_setback(&[e], 1.0, 0.5));
    report(840, "fillet_faces radius zero", b.fillet_faces(top, side, 0.0));
    report(841, "fillet_faces one face twice", b.fillet_faces(top, top, 1.0));
    report(859, "full_round a face twice", b.full_round(side, top, side));
    report(860, "full_round middle not beside a side", b.full_round(front, bottom, side));
    report(876, "chamfer distance NaN", b.chamfer(&[e], f64::NAN));
    report(897, "chamfer_two a distance zero", b.chamfer_two(e, top, 1.0, 0.0));
    report(913, "chamfer_distances one negative", b.chamfer_distances(&[(e, 1.0), (e2, -1.0)]));
    // edits
    report(949, "offset distance zero", b.offset(0.0));
    report(949, "offset distance negative", b.offset(-1.0));
    report(971, "offset_faces distance zero", b.offset_faces(&[top], 0.0));
    report(1028, "thicken thickness zero", b.thicken(&[top], 0.0, Join::Round));
    report(1051, "shell thickness zero", b.shell(&[top], 0.0));
    report(1051, "shell thickness negative", b.shell(&[top], -1.0));
    report(1051, "shell_outward thickness zero", b.shell_outward(&[top], 0.0));
    report(1068, "draft a curved face", cylinder(f, 2.0, 4.0).unwrap().draft(&[cylinder(f, 2.0, 4.0).unwrap().face_at([2.0, 0.0, 2.0]).unwrap()], [0.0; 3], [0.0, 0.0, 1.0], deg(5.0)));
    report(1069, "draft angle zero", b.draft(&[side], [0.0; 3], [0.0, 0.0, 1.0], deg(0.0)));
    report(1069, "draft angle right", b.draft(&[side], [0.0; 3], [0.0, 0.0, 1.0], deg(90.0)));
    report(1069, "draft angle negative", b.draft(&[side], [0.0; 3], [0.0, 0.0, 1.0], deg(-5.0)));
    // queries
    report(1428, "distance to empty", b.distance(&empty()));
}

// each tolerance case alone: one of them may never finish
#[test]
#[ignore = "a probe: run with --ignored --nocapture"]
fn probe_tol_section_zero() {
    report(1498, "section tolerance zero", cube().section(&Plane::new([0.0, 0.0, 3.0], [0.0, 0.0, 1.0]).unwrap(), 0.0));
}

#[test]
#[ignore = "a probe: run with --ignored --nocapture"]
fn probe_tol_project_negative() {
    report(1524, "project tolerance negative", cube().project([0.0, 0.0, 1.0], -0.1));
}

#[test]
#[ignore = "a probe: run with --ignored --nocapture"]
fn probe_tol_mesh_zero() {
    report(1577, "mesh tolerance zero", cube().mesh(0.0));
}

#[test]
#[ignore = "a probe: run with --ignored --nocapture"]
fn probe_tol_mesh_nan() {
    report(1577, "mesh tolerance NaN", cube().mesh(f64::NAN));
}

#[test]
#[ignore = "a probe: run with --ignored --nocapture"]
fn probe_tol_display_zero() {
    report(1577, "display tolerance zero", cube().display(0.0));
}
