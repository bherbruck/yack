// yack: probe for conformance B38 projections-eb712599cd54d823 (hidden lines of a box behind a box,
// seen 0.000994 too long, hidden as much too short): which stretch of the drawing's seen lines is
// hidden by an independent ray test.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing, clippy::print_stderr)]
use yack_springrep::{
    boolean, draw,
    math::{Frame, Vec3},
    primitive,
};

#[test]
#[ignore = "probe: conformance projections-eb712599cd54d823 (B38, box behind a box), under diagnosis by w-m11"]
fn box_behind_a_box_seen_lines() {
    let at = |o: [f64; 3]| Frame::new(Vec3::new(o[0], o[1], o[2]), Vec3::Z, Vec3::X).unwrap();
    let a = primitive::block(&at([1.3359254904417064, 2.9917121568493954, -3.5357264479528503]), Vec3::new(3.5687381798099533, 2.555585327934775, 3.2599654704937153), "a").unwrap();
    let b = primitive::block(&at([0.3325093840725135, 6.557799273837184, 1.8125629786453756]), Vec3::new(2.949756189677025, 1.9633581851472757, 3.276016077235808), "b").unwrap();
    let u = boolean::union(&a, &b).unwrap();
    let d = Vec3::new(0.3074569148383761, -0.4625450798448814, -0.8315781951382322).unit().unwrap();
    let dr = draw::hidden_lines(&u, d, 0.01).unwrap();
    let rays = yack_springrep::mesh::rays(&u).unwrap();
    // Hidden where the ray toward the viewer from just in front of the point meets material.
    let hidden = |p: Vec3| {
        let hits = rays.cast(p - d * 1e-6, -d, (0.0, 1e3)).unwrap();
        hits.iter().any(|h| h.t > 1e-6)
    };
    let len = |l: &Vec<Vec3>| l.windows(2).map(|w| (w[1] - w[0]).norm()).sum::<f64>();
    eprintln!("seen {} lines, length {:.6}; hidden {} lines, length {:.6}", dr.visible.len(), dr.visible.iter().map(len).sum::<f64>(), dr.hidden.len(), dr.hidden.iter().map(len).sum::<f64>());
    for (i, l) in dr.visible.iter().enumerate() {
        for w in l.windows(2) {
            let n = 4000;
            let mut run: Option<(usize, usize)> = None;
            for k in 1..n {
                let p = w[0] + (w[1] - w[0]) * (k as f64 / n as f64);
                if hidden(p) {
                    run = Some(run.map_or((k, k), |r| (r.0, k)));
                }
            }
            if let Some((k0, k1)) = run {
                let seg = (w[1] - w[0]).norm();
                eprintln!("  seen line {i}: chord {:?} -> {:?} hidden from {:.6} to {:.6} of it ({:.3e} mm)", w[0], w[1], k0 as f64 / n as f64, k1 as f64 / n as f64, seg * (k1 - k0) as f64 / n as f64);
            }
        }
    }
}
