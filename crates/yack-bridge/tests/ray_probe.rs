// yack: probe for conformance B26 "4 crossings against 2" (circle-contact-fcb3423424f28443,
// booleans/touching/sphere-in-a-torus-hole): the failing ray cast on each boolean, every hit listed.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing, clippy::print_stderr)]
use yack_boolean as boolean;
use yack_math::kernel::{Frame, Vec3};
use yack_build::primitive;

#[test]
#[ignore = "evidence: conformance circle-contact-fcb3423424f28443 expects 2 crossings; the bodies are 1.3e-5 mm apart, so 4 (swarm/done/w-m11-conformance-reference-issues.md)"]
fn sphere_in_a_torus_hole_ray() {
    let x = Vec3::new(-0.7148566939609067, -0.3270574454210159, 0.6180722728726482);
    let z = Vec3::new(0.06459752030432298, 0.8492149734521551, 0.5240811857290699);
    let ft = Frame::new(Vec3::new(-2.4536287888669417, -0.4188831209451287, -2.3224440845692773), z, x).unwrap();
    let fs = Frame::new(Vec3::new(-2.3263451224619023, 1.254419428097376, -1.2897887365120186), z, x).unwrap();
    let a = primitive::torus(&ft, 1.6819245919605297, 0.5900110918968451, "t").unwrap();
    let b = primitive::sphere(&fs, 2.000610796586716, "s").unwrap();
    let o = Vec3::new(7.219212934459197, -0.6668055044892296, 10.405358061114654);
    let d = Vec3::new(-0.5497548415799469, 0.07066902496427863, -0.8323313661457231);
    for (name, r) in [
        ("a", Ok(a.clone())),
        ("b", Ok(b.clone())),
        ("a ∪ b", boolean::union(&a, &b)),
        ("a ∩ b", boolean::intersection(&a, &b)),
        ("a ∖ b", boolean::difference(&a, &b)),
        ("b ∖ a", boolean::difference(&b, &a)),
    ] {
        match r {
            Ok(s) => {
                let hits = yack_mesh::rays(&s).unwrap().cast(o, d, (0.0, 1e9)).unwrap();
                eprintln!("{name}: {} solids, {} faces, {} hits", s.solid_ids().count(), s.face_ids().count(), hits.len());
                for h in &hits {
                    eprintln!("   t {:.12} face {} on_edge {} normal·d {:.3e}", h.t, h.face.index(), h.on_edge, h.normal.dot(d));
                }
            }
            Err(e) => eprintln!("{name}: {e}"),
        }
    }
}
