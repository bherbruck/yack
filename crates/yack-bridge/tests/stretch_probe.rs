// yack: probe for conformance B32 stretches-369ef83ce9d5f331 (a cone under a non-uniform scale and
// shear, 5 of 1024 points placed differently): springrep's classification of mapped points against
// the exact classification of their preimages in the cone.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing, clippy::print_stderr)]
use yack_springrep::{
    facade::edit,
    math::{Frame, Vec3},
    primitive,
    rays::{Rays, Side},
};

#[test]
#[ignore = "probe: conformance stretches-369ef83ce9d5f331 (B32, sheared cone), under diagnosis by w-m11"]
fn sheared_cone_classification() {
    let o = Vec3::new(-0.1369187722614813, -0.1927317277546228, 9.496120663976027);
    let x = Vec3::new(0.9862312463263244, 0.0011868763560915257, 0.16536783270678776);
    let z = Vec3::new(0.1653720918704991, -0.007078186740360178, -0.9862058459078631);
    let (r0, r1, h) = (3.753950727109955, 0.9541105874096755, 14.435434108066273);
    let cone = primitive::cone(&Frame::new(o, z, x).unwrap(), r0, r1, h, "c").unwrap();
    let m = [
        [-0.4538971246554957, 0.15252576458329087, 0.9555666727243001, 41.6656262880843],
        [-0.26697402297825473, -0.5673293657946105, -1.4703977293152846, 16.072466163157856],
        [0.05877765464764511, -1.3990214079245642, 0.7004527186325231, -15.994874937232893],
    ];
    let mapped = edit::affine(&cone, m).unwrap();
    let map = |p: Vec3| Vec3::new(
        m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3],
        m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
        m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3],
    );
    let y = z.cross(x);
    // Exactly in the cone: along its axis within [0, h], within the radius there; and how far in.
    let inside = |p: Vec3| {
        let q = p - o;
        let (a, b, c) = (q.dot(x), q.dot(y), q.dot(z));
        let rad = r0 + (r1 - r0) * c / h;
        let margin = (rad - (a * a + b * b).sqrt()).min(c).min(h - c);
        (margin > 0.0, margin)
    };
    let rays = Rays::new(&mapped).unwrap();
    let n = 16;
    let (mut wrong, mut total) = (0, 0);
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let f = |t: usize| (t as f64 + 0.5) / n as f64;
                let p = o + x * ((f(i) - 0.5) * 2.4 * r0) + y * ((f(j) - 0.5) * 2.4 * r0) + z * ((f(k) * 1.2 - 0.1) * h);
                let (want, margin) = inside(p);
                let got = rays.side(map(p));
                total += 1;
                let ok = match got {
                    Side::Inside => want,
                    Side::Outside => !want,
                    Side::On => margin.abs() < 1e-6,
                };
                if !ok {
                    wrong += 1;
                    eprintln!("  wrong: preimage margin {margin:.3e} want inside {want} got {got:?} at {:?}", map(p));
                }
            }
        }
    }
    eprintln!("{wrong} of {total} placed differently");
}

#[test]
#[ignore = "probe: conformance stretches-369ef83ce9d5f331, the crossings behind one misplaced point (w-m11)"]
fn sheared_cone_one_point() {
    let o = Vec3::new(-0.1369187722614813, -0.1927317277546228, 9.496120663976027);
    let x = Vec3::new(0.9862312463263244, 0.0011868763560915257, 0.16536783270678776);
    let z = Vec3::new(0.1653720918704991, -0.007078186740360178, -0.9862058459078631);
    let cone = primitive::cone(&Frame::new(o, z, x).unwrap(), 3.753950727109955, 0.9541105874096755, 14.435434108066273, "c").unwrap();
    let m = [
        [-0.4538971246554957, 0.15252576458329087, 0.9555666727243001, 41.6656262880843],
        [-0.26697402297825473, -0.5673293657946105, -1.4703977293152846, 16.072466163157856],
        [0.05877765464764511, -1.3990214079245642, 0.7004527186325231, -15.994874937232893],
    ];
    let mapped = edit::affine(&cone, m).unwrap();
    eprintln!("mapped: {} faces", mapped.face_ids().count());
    for f in mapped.face_ids() {
        eprintln!("  face {} {}", f.index(), mapped.face(f).surface.kind());
    }
    let rays = Rays::new(&mapped).unwrap();
    // Exactly inside (preimage margin +0.099), placed Outside.
    let p = Vec3::new(36.53894920997076, 22.626132672803138, -19.18988598536717);
    eprintln!("side {:?}", rays.side(p));
    for d in [Vec3::X, Vec3::Y, Vec3::Z, -Vec3::X, -Vec3::Y, -Vec3::Z, Vec3::new(0.48, 0.6, 0.64)] {
        let hits = rays.cast(p, d, (0.0, 1e3)).unwrap();
        eprintln!("  dir {:?}: {} hits", d, hits.len());
        for h in &hits {
            eprintln!("    t {:.9} face {} on_edge {} normal·d {:.4}", h.t, h.face.index(), h.on_edge, h.normal.dot(d.unit().unwrap()));
        }
    }
}

#[test]
#[ignore = "probe: conformance stretches-369ef83ce9d5f331, mesh and exact crossing behind one misplaced point (w-m11)"]
fn sheared_cone_mesh_and_exact_crossing() {
    use yack_springrep::{check, measure, mesh};
    let o = Vec3::new(-0.1369187722614813, -0.1927317277546228, 9.496120663976027);
    let x = Vec3::new(0.9862312463263244, 0.0011868763560915257, 0.16536783270678776);
    let z = Vec3::new(0.1653720918704991, -0.007078186740360178, -0.9862058459078631);
    let y = z.cross(x);
    let (r0, r1, h) = (3.753950727109955, 0.9541105874096755, 14.435434108066273);
    let cone = primitive::cone(&Frame::new(o, z, x).unwrap(), r0, r1, h, "c").unwrap();
    let m = [
        [-0.4538971246554957, 0.15252576458329087, 0.9555666727243001, 41.6656262880843],
        [-0.26697402297825473, -0.5673293657946105, -1.4703977293152846, 16.072466163157856],
        [0.05877765464764511, -1.3990214079245642, 0.7004527186325231, -15.994874937232893],
    ];
    let mapped = edit::affine(&cone, m).unwrap();
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    eprintln!("check issues: {:?}", check::check(&mapped));
    eprintln!("volume {} against det·V {}", measure::volume(&mapped), det.abs() * measure::volume(&cone));
    let me = mesh::tessellate(&mapped, &mesh::MeshOptions { linear: 0.01, angular: 0.2 }).unwrap();
    let sid = mapped.solid_ids().next().unwrap();
    let rep = mesh::report(&mapped, &me, sid);
    eprintln!("mesh: {} triangles, watertight {}, volume {}, boundary {} nonmanifold {}", rep.triangles, rep.watertight(), rep.volume, rep.boundary_edges, rep.nonmanifold_edges);
    // The exact crossing along +x from the point, found in the cone's own space.
    let p = Vec3::new(36.53894920997076, 22.626132672803138, -19.18988598536717);
    let inv = |v: Vec3| {
        // M⁻¹ v by Cramer (rows of M).
        let a = [[m[0][0], m[0][1], m[0][2]], [m[1][0], m[1][1], m[1][2]], [m[2][0], m[2][1], m[2][2]]];
        let col = |i: usize, w: Vec3| {
            let mut b = a;
            for (r, val) in [w.x, w.y, w.z].iter().enumerate() {
                b[r][i] = *val;
            }
            b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) - b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) + b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0])
        };
        Vec3::new(col(0, v) / det, col(1, v) / det, col(2, v) / det)
    };
    let pp = inv(p - Vec3::new(m[0][3], m[1][3], m[2][3]));
    let dd = inv(Vec3::X);
    let q = pp - o;
    let (qa, qb, qc) = (q.dot(x), q.dot(y), q.dot(z));
    let (da, db, dc) = (dd.dot(x), dd.dot(y), dd.dot(z));
    let k = (r1 - r0) / h;
    // (qa + t da)² + (qb + t db)² = (r0 + k (qc + t dc))²
    let (aa, bb, cc) = (da * da + db * db - k * k * dc * dc, 2.0 * (qa * da + qb * db) - 2.0 * k * dc * (r0 + k * qc), qa * qa + qb * qb - (r0 + k * qc).powi(2));
    let disc = bb * bb - 4.0 * aa * cc;
    eprintln!("preimage point: radial {} at height {} (radius there {})", (qa * qa + qb * qb).sqrt(), qc, r0 + k * qc);
    for t in [(-bb - disc.sqrt()) / (2.0 * aa), (-bb + disc.sqrt()) / (2.0 * aa)] {
        let c = qc + t * dc;
        eprintln!("  side crossing t {t:.9} at height {c:.6} (within 0..{h}: {})", c >= 0.0 && c <= h);
        if t > 0.0 && c >= 0.0 && c <= h {
            let hit = p + Vec3::X * t;
            let face1 = mapped.face_ids().nth(1).unwrap();
            let near = me
                .faces
                .iter()
                .filter(|(f, _)| *f == face1)
                .flat_map(|(_, r)| me.triangles[r.clone()].iter())
                .flat_map(|t| t.iter().map(|&i| me.positions[i as usize].dist(hit)))
                .fold(f64::INFINITY, f64::min);
            eprintln!("  nearest face-1 mesh vertex to the exact hit: {near:.4e}; the surface there: {:?}", hit);
        }
    }
}
