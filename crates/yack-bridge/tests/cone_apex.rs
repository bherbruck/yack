// yack: bench swept/cone-spun-about-a-line-through-it through the bridge: springrep's pointed cone,
// drawn base-first, must reach cadrs drawn from its apex, which cadrs sweeps exactly.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
use yack_cadrs::native::Body;
use yack_cadrs::{DefaultKernel, Kernel, yack_glue};
use yack_math::kernel::{Frame, Vec3};

#[test]
#[ignore = "open: cadrs still sweeps the bridged pointed cone only as a mesh (w-m11, cone apex-first work)"]
fn a_bridged_pointed_cone_sweeps_as_cadrs_own() {
    let (s, c) = (30f64.to_radians().sin(), 30f64.to_radians().cos());
    let base = Frame::new(Vec3::new(5.0 * c, 0.0, 5.0 * s), Vec3::new(-c, 0.0, -s), Vec3::new(0.0, 1.0, 0.0)).unwrap();
    let spike = yack_build::primitive::cone(&base, 7.0 * 25f64.to_radians().tan(), 0.0, 7.0, "k").unwrap();
    let mut k = DefaultKernel::new();
    let body = yack_bridge::to_cadrs(&spike).unwrap().0;
    dump("bridged", &body);
    // cadrs's own pointed cone in the same place (yack_glue draws it from its point).
    let place = yack_cadrs::facade::profile::Place {
        origin: [5.0 * c, 0.0, 5.0 * s],
        x: [0.0, 1.0, 0.0],
        y: [s, 0.0, -c],
        z: [-c, 0.0, -s],
    };
    let own = yack_glue::primitive(&mut k, &place, yack_glue::Prim::Cone { bottom: 7.0 * 25f64.to_radians().tan(), top: 0.0, height: 7.0 })
        .unwrap()
        .0;
    dump("cadrs's own", &k.get(own).unwrap());
    let id = k.insert(body);
    let m = k.mass_properties(id).unwrap();
    assert!((m.volume - yack_measure::volume(&spike)).abs() < 1e-9 * m.volume, "{} vs springrep's", m.volume);
    let turn = yack_glue::Move::Turn { point: [0.0; 3], axis: [0.0, 0.0, 1.0], angle: 150f64.to_radians() };
    let swept = yack_glue::swept(&mut k, id, &turn).unwrap().0;
    let v = k.mass_properties(swept).unwrap().volume;
    // The bench's value (cad-kernel and cadrs agree to 2e-9).
    assert!((v - 288.495056967).abs() < 1e-6 * 288.5, "swept volume {v}");
}

fn dump(what: &str, b: &Body) {
    eprintln!("== {what}");
    for (i, f) in b.faces().iter().enumerate() {
        let surf = format!("{:?}", f.surf);
        eprintln!("face {i}: sense {} {}", f.sense, &surf[..surf.len().min(300)]);
        for lp in &f.loops {
            for co in lp {
                let e = &b.edges()[co.edge];
                let pc = format!("{:?}", co.pc);
                eprintln!(
                    "  edge {} fwd {} degenerate {} t {:?} v {:?} pc {}",
                    co.edge, co.fwd, e.degenerate, e.t, e.v, &pc[..pc.len().min(160)]
                );
            }
        }
    }
}
