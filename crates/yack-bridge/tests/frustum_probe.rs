// yack: probe for bench-ladder-diff row 7: the springrep frustum carried to cadrs and shelled, against
// cadrs's own frustum shelled the same way (printed; it asserts only that both shells build).
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_cadrs::{DefaultKernel, Kernel, Pick, Primitive, ShellSpec};
use yack_cadrs::native::{Body, P3, V3};
use yack_math::kernel::{Frame, Vec3};

fn describe(b: &Body) -> String {
    b.faces()
        .iter()
        .enumerate()
        .map(|(i, f)| format!("{i}:{:?}/sense {}/{} loops", f.surf.kind(), f.sense, f.loops.len()))
        .collect::<Vec<_>>()
        .join(" ")
}

#[test]
fn frustum_shell_bridged_vs_native() {
    let sr = yack_build::primitive::cone(&Frame::new(Vec3::ZERO, Vec3::Z, Vec3::X).unwrap(), 6.0, 3.0, 4.0, "k").unwrap();
    let top_sr = yack_measure::face_at(&sr, Vec3::new(1.0, 1.0, 4.0)).unwrap().0.index();
    let (body, _) = yack_bridge::to_cadrs(&sr).unwrap();
    let mut k = DefaultKernel::new();
    let bridged = k.insert(body.clone());
    let native = k.primitive(&Primitive::cone(P3::origin(), V3::z(), 6.0, 3.0, 4.0)).unwrap().first().unwrap();
    let mut out = vec![format!("FAILED-PROBE springrep top face index {top_sr}")];
    for (name, id) in [("bridged", bridged), ("native", native)] {
        let at = k.faces_at(id, &[P3::new(1.0, 1.0, 4.0)]).unwrap();
        out.push(format!("FAILED-PROBE {name}: faces_at top {at:?}; {}", describe(k.get(id).unwrap())));
        for spec in [ShellSpec::inward(at.clone(), 2.7), ShellSpec::inward(at.clone(), 2.7).mitred()] {
            let r = k.shell_with(id, &spec).and_then(|r| r.first());
            match r {
                Ok(s) => {
                    let m = k.mass_properties(s).unwrap();
                    out.push(format!(
                        "FAILED-PROBE {name} sharp={} volume {} area {} faces {}",
                        spec.sharp, m.volume, m.surface_area, k.get(s).unwrap().faces.len()
                    ));
                }
                Err(e) => out.push(format!("FAILED-PROBE {name} sharp={} error {e}", spec.sharp)),
            }
        }
    }
    println!("{}", out.join("\n"));
}
