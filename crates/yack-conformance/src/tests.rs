// yack: the adapter's own tests: `supports()` is exactly the functions that answer (PLAN §5.1).
use super::*;
use brep_protocol::{FUNCTIONS, Kind, model::Frame, path::PathCurve, profile::Seg};

const ID: [[f64; 4]; 3] = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]];

fn square() -> profile::Profile {
    square_at(0.0)
}

/// The 4 × 4 square from (x0, 0).
fn square_at(x0: f64) -> profile::Profile {
    let c = [[x0, 0.0], [x0 + 4.0, 0.0], [x0 + 4.0, 4.0], [x0, 4.0]];
    let seg = |i: usize| Seg::Line { a: c[i], b: c[(i + 1) % 4] };
    profile::Profile { loops: vec![(0..4).map(seg).collect()] }
}

/// A 10 × 20 × 30 box at the origin: an edge through (5, 0, 0), a face through (5, 10, 0).
fn block(k: &Yack) -> Res<Solid> {
    k.make(&Primitive::Box { frame: Frame::at([0.0; 3]), size: [10.0, 20.0, 30.0] })
}

/// One small call of each protocol function, by name, on the box above.
fn smoke(k: &Yack, name: &str) -> Res<()> {
    let at = Frame::at([0.0; 3]);
    let (e, f, o, z) = ([5.0, 0.0, 0.0], [5.0, 10.0, 0.0], [5.0, 10.0, 15.0], [0.0, 0.0, 1.0]);
    let make = |p: Primitive| k.make(&p).map(drop);
    let s = || block(k);
    let ball = || k.make(&Primitive::Sphere { frame: Frame::at([30.0, 0.0, 0.0]), radius: 5.0 });
    let sq = square;
    // The three edges meeting at the origin corner.
    let corner = [e, [0.0, 10.0, 0.0], [0.0, 0.0, 15.0]];
    match name {
        "box" => s().map(drop),
        "cylinder" => make(Primitive::Cylinder { frame: at, radius: 2.0, height: 5.0 }),
        "sphere" => ball().map(drop),
        "torus" => make(Primitive::Torus { frame: at, major: 5.0, minor: 1.0 }),
        "cone" => make(Primitive::Cone { frame: at, bottom: 3.0, top: 1.0, height: 4.0 }),
        "extrude" => make(Primitive::Extrude { frame: at, profile: sq(), height: 2.0 }),
        "revolve" => make(Primitive::Revolve { frame: at, profile: sq(), angle: 1.0 }),
        "screw" => make(Primitive::Screw { frame: at, profile: square_at(1.0), rise: 1.0, angle: 1.0 }),
        "loft" => {
            make(Primitive::Loft { frame: at, profile: sq(), scale: 1.0, height: 3.0, top: sq() })
        }
        "loft_through" => k.loft_through(&at, &[sq(), sq()], &[0.0, 3.0]).map(drop),
        "sweep" => make(Primitive::Sweep {
            frame: at,
            profile: sq(),
            path: vec![model::PathSeg::Line { length: 5.0 }],
        }),
        "sweep_path" => make(Primitive::SweepPath {
            frame: at,
            profile: sq(),
            path: PathCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![[0.0; 3], [0.0, 0.0, 5.0]]),
            twist: 0.0,
        }),
        "slab" => make(Primitive::Slab {
            frame: at,
            size: [4.0, 4.0],
            knots_u: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            knots_v: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            heights: vec![vec![1.0; 4]; 4],
        }),
        "union" => k.union(&s()?, &ball()?).map(drop),
        "intersection" => k.intersection(&s()?, &ball()?).map(drop),
        "difference" => k.difference(&s()?, &ball()?).map(drop),
        "union_all" => k.union_all(&[&s()?, &ball()?]).map(drop),
        "difference_all" => k.difference_all(&s()?, &[&ball()?]).map(drop),
        "fillet" => k.fillet(&s()?, &[e], 1.0).map(drop),
        "fillet_radii" => k.fillet_radii(&s()?, &[e], &[1.0]).map(drop),
        "fillet_variable" => k.fillet_variable(&s()?, e, [0.0; 3], 1.0, 2.0).map(drop),
        "fillet_conic" => k.fillet_conic(&s()?, &[e], 1.0, 0.5).map(drop),
        "fillet_g2" => k.fillet_g2(&s()?, &[e], 1.0, 1.0).map(drop),
        "fillet_setback" => k.fillet_setback(&s()?, &corner, 1.0, 2.0).map(drop),
        "fillet_setback_g2" => k.fillet_setback_g2(&s()?, &corner, 1.0, 2.0).map(drop),
        "fillet_faces" => k.fillet_faces(&s()?, f, [5.0, 0.0, 15.0], 1.0).map(drop),
        "full_round" => {
            k.full_round(&s()?, [0.0, 10.0, 15.0], [5.0, 10.0, 30.0], [10.0, 10.0, 15.0]).map(drop)
        }
        "chamfer" => k.chamfer(&s()?, &[e], 1.0).map(drop),
        "chamfer_two" => k.chamfer_two(&s()?, e, f, 1.0, 2.0).map(drop),
        "chamfer_distances" => k.chamfer_distances(&s()?, &[e], &[1.0]).map(drop),
        "offset" => k.offset(&s()?, 1.0).map(drop),
        "offset_faces" => k.offset_faces(&s()?, &[f], 1.0).map(drop),
        "delete_faces" => {
            k.delete_faces(&k.fillet(&s()?, &[e], 1.0)?, &[[5.0, 0.3, 0.3]]).map(drop)
        }
        "move_faces" => k.move_faces(&s()?, &[f], ID).map(drop),
        "thicken" => k.thicken(&s()?, &[f], 1.0).map(drop),
        "shell" => k.shell(&s()?, &[f], 1.0).map(drop),
        "draft" => k.draft(&s()?, &[[0.0, 10.0, 15.0]], [0.0; 3], z, 0.05).map(drop),
        "write_step" => k.write_step(&s()?).map(drop),
        "read_step" => k.read_step(&k.write_step(&s()?)?).map(drop),
        "sew" => k.sew(&k.write_step(&s()?)?, 1e-3).map(drop),
        "volume" => k.volume(&s()?).map(drop),
        "area" => k.area(&s()?).map(drop),
        "classify" => k.classify(&s()?, &[o]).map(drop),
        "closest" => k.closest(&s()?, &[[50.0; 3]]).map(drop),
        "normal" => k.normal(&s()?, &[f]).map(drop),
        "curvature" => k.curvature(&s()?, &[f]).map(drop),
        "check" => k.check(&s()?).map(drop),
        "centroid" => k.centroid(&s()?).map(drop),
        "inertia" => k.inertia(&s()?).map(drop),
        "distance" => k.distance(&s()?, &ball()?).map(drop),
        "interference" => k.interference(&s()?, &ball()?).map(drop),
        "betti" => k.betti(&s()?).map(drop),
        "section_area" => k.section_area(&s()?, o, z).map(drop),
        "section" => k.section(&s()?, o, z, 0.01).map(drop),
        "project" => k.project(&s()?, z, 0.01).map(drop),
        "bounding_box" => k.bounding_box(&s()?).map(drop),
        "split" => k.split(&s()?, o, z).map(drop),
        "mesh" => k.mesh(&s()?, 0.01).map(drop),
        "display" => k.display(&s()?, 0.01).map(drop),
        "raycast" => k.raycast(&s()?, [5.0, 10.0, -5.0], z).map(drop),
        "transform" => k.transform(&s()?, ID).map(drop),
        "affine" => k.affine(&s()?, ID).map(drop),
        _ => Err(Refusal::failed(format!("no smoke case for `{name}`"))),
    }
}

#[test]
fn supports_names_only_protocol_functions() {
    for name in Yack.supports() {
        assert!(FUNCTIONS.contains(&name.as_str()), "`{name}` is not a protocol function");
    }
}

/// Each listed function passes its smoke case; each unlisted one refuses as unsupported, so
/// `supports()` (yack::SUPPORTS) is exactly the functions that are not stubs.
#[test]
fn supports_is_what_answers() {
    let listed = Yack.supports();
    let mut wrong = Vec::new();
    for &name in FUNCTIONS {
        let got = smoke(&Yack, name);
        if listed.iter().any(|l| l == name) {
            if let Err(e) = got {
                wrong.push(format!("`{name}` is listed but its smoke case refused: {e}"));
            }
        } else if got.as_ref().err().map(|e| e.kind) != Some(Kind::Unsupported) {
            wrong.push(format!("`{name}` is unlisted but gave {got:?}"));
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

#[test]
fn queries_on_the_empty_solid_are_answered() {
    let e = make::empty();
    assert_eq!(Yack.volume(&e).ok(), Some(0.0));
    assert_eq!(Yack.area(&e).ok(), Some(0.0));
    assert_eq!(Yack.centroid(&e).ok(), Some(None));
}
