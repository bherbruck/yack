// yack: temporary probes of the glue against cadrs's own runner calls (w-m17; deleted after).
use crate::{DefaultKernel, Kernel, Pick, Primitive, ShellSpec};
use nalgebra::{Point3, Vector3};

#[test]
fn frustum_shells() {
    for (t, at) in [(2.7, [1.0, 1.0, 4.0])] {
        let mut k = DefaultKernel::new();
        let b = k.primitive(&Primitive::cone(Point3::origin(), Vector3::z(), 6.0, 3.0, 4.0)).unwrap().first().unwrap();
        let remove = k.faces_at(b, &[Point3::new(at[0], at[1], at[2])]).unwrap();
        eprintln!("faces {}, remove {remove:?}", k.get(b).unwrap().faces.len());
        for spec in [ShellSpec::inward(remove.clone(), t), ShellSpec::inward(remove.clone(), t).mitred()] {
            match k.shell_with(b, &spec).and_then(|r| r.first()) {
                Ok(s) => {
                    let m = k.mass_properties(s).unwrap();
                    eprintln!("PROBE sharp={} volume {} area {} faces {}", spec.sharp, m.volume, m.surface_area, k.get(s).unwrap().faces.len());
                }
                Err(e) => eprintln!("PROBE sharp={} error {e}", spec.sharp),
            }
        }
    }
}

#[test]
fn screw_timing() {
    use crate::yack_glue::{Move, swept};
    let mut k = DefaultKernel::new();
    let bar = k
        .primitive(&Primitive::Box { min: Point3::new(4.5, -8.0, -0.45), max: Point3::new(12.0, 8.0, 0.45) })
        .unwrap()
        .first()
        .unwrap();
    for turns in [0.5f64, 21.0] {
        let t = std::time::Instant::now();
        let m = Move::Screw { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: turns * std::f64::consts::TAU, rise_per_turn: 1.0 };
        let r = swept(&mut k, bar, &m);
        let faces = r.as_ref().ok().and_then(|(b, _)| k.get(*b).ok()).map(|b| b.faces.len());
        eprintln!("PROBE screw {turns} turns: {:?} faces {faces:?} ok {}", t.elapsed(), r.is_ok());
    }
}

#[test]
fn glue_split_and_chamfers() {
    use crate::yack_glue::{chamfer, split};
    let mut k = DefaultKernel::new();
    let cube = k
        .primitive(&Primitive::Box { min: Point3::new(0.0, 0.0, 0.0), max: Point3::new(10.0, 10.0, 10.0) })
        .unwrap()
        .first()
        .unwrap();
    let (front, back) = split(&mut k, cube, [0.0, 0.0, 4.0], [0.0, 0.0, 1.0]).unwrap();
    let vf = k.mass_properties(front.unwrap().0).unwrap().volume;
    let vb = k.mass_properties(back.unwrap().0).unwrap().volume;
    assert!((vf - 600.0).abs() < 1e-9 && (vb - 400.0).abs() < 1e-9, "{vf} {vb}");
    let all: Vec<u64> = (0..k.edges(cube).unwrap().len() as u64).collect();
    let (c, _) = chamfer(&mut k, cube, &all, 1.0).unwrap();
    let m = k.mass_properties(c).unwrap();
    eprintln!("PROBE cube-all-edges chamfer 1: volume {} faces {}", m.volume, k.get(c).unwrap().faces.len());
}
