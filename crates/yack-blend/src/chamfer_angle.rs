// yack: chamfer_angle, a bevel set back along one face at an angle to it: yack's own rungs (M4.8).
//! A bevel of an edge set back a distance along one of its two faces, at an angle to that face:
//! yack's own rungs for `Solid::chamfer_angle`, behind cadrs's bridged one until a bake-off puts
//! them ahead.
//!
//! Each way of building it is a private strategy (PLAN §3.5, rule 7). It answers, or declines with
//! why (an [`Error::Unsupported`]); an [`Error::InvalidInput`] ends the ladder. The first strategy
//! that answers wins, and its solid goes through [`finish`]: a solid the checker finds any issue
//! in is a decline, never an answer (rule 8).

mod revolved;

use std::f64::consts::FRAC_PI_2;

use yack_base::kernel::{Error, Result};
use yack_topo::{Brep, EdgeId, FaceId};

/// A way of building the bevel: its name, and its run, which answers or declines.
struct Strategy {
    name: &'static str,
    run: fn(&Brep, EdgeId, FaceId, f64, f64, &str) -> Result<Brep>,
}

/// The strategies in the order they are tried (doc/donors.md orders them once there are two).
const STRATEGIES: &[Strategy] = &[Strategy { name: "revolved", run: revolved::chamfer }];

/// `body` with `edge` bevelled: set back `distance` along `face`, one of the edge's two faces, the
/// bevel at `angle` (radians, strictly between 0 and a right angle) to that face. The bevel face is
/// `<feature>.bevel`; every other face keeps its names. Declines (an [`Error::Unsupported`] naming
/// each strategy's reason) where no strategy answers.
pub fn chamfer_angle(
    body: &Brep,
    edge: EdgeId,
    face: FaceId,
    distance: f64,
    angle: f64,
    feature: &str,
) -> Result<Brep> {
    if !(distance.is_finite() && distance > 0.0) {
        return Err(Error::InvalidInput(format!("chamfer distance {distance} is not positive")));
    }
    if !(angle > 0.0 && angle < FRAC_PI_2) {
        return Err(Error::InvalidInput(
            "the bevel's angle to the face must be between 0 and 90 degrees".into(),
        ));
    }
    let mut why = Vec::new();
    for s in STRATEGIES {
        match (s.run)(body, edge, face, distance, angle, feature).and_then(finish) {
            Ok(b) => return Ok(b),
            Err(e @ Error::InvalidInput(_)) => return Err(e),
            Err(e) => why.push(format!("{}: {e}", s.name)),
        }
    }
    Err(Error::Unsupported(format!("no chamfer_angle strategy answers: {}", why.join("; "))))
}

/// `b`, where the checker finds no issue in it; else a decline listing the issues.
fn finish(b: Brep) -> Result<Brep> {
    let issues = yack_check::check(&b);
    if issues.is_empty() {
        Ok(b)
    } else {
        Err(Error::InvalidResult(issues.into_iter().map(|i| i.to_string()).collect()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::{FRAC_PI_4, FRAC_PI_6, PI};

    use yack_geom::{Curve, Surface};
    use yack_math::kernel::{Frame, Vec2, Vec3};
    use yack_profile::{NamedSegment, Profile};
    use yack_topo::FaceName;

    /// The whole circle edge of `radius` about the z axis at height `z`.
    fn rim(b: &Brep, radius: f64, z: f64) -> EdgeId {
        b.edge_ids()
            .find(|&e| {
                matches!(&b.edge(e).curve, Some(Curve::Circle(k))
                    if (k.radius - radius).abs() < 1e-9 && (k.frame.origin.z - z).abs() < 1e-9) // tol: picks a test's own rim
            })
            .expect("the rim")
    }

    /// Of `e`'s two faces, the plane one when `plane`, else the other.
    fn beside(b: &Brep, e: EdgeId, plane: bool) -> FaceId {
        b.edge_uses(e)
            .iter()
            .map(|&c| b.coedge_face(c))
            .find(|&f| matches!(b.face(f).surface, Surface::Plane(_)) == plane)
            .expect("the face")
    }

    fn near(got: &Brep, want: f64) {
        let v = yack_measure::volume(got);
        assert!((v - want).abs() <= 1e-9 * want, "volume {v} against {want}"); // tol: closed forms and cadrs's volumes
    }

    fn cylinder() -> Brep {
        yack_build::primitive::cylinder(&Frame::WORLD, 5.0, 10.0, "c").expect("cylinder")
    }

    /// The half ball of radius 5 on the plane z = 0.
    fn hemisphere() -> Brep {
        let o = Vec2::new(0.0, 0.0);
        let section = Profile::outline(vec![
            NamedSegment::line(o, Vec2::new(5.0, 0.0), "base"),
            NamedSegment::arc(o, 5.0, 0.0, FRAC_PI_2, "dome"),
            NamedSegment::line(Vec2::new(0.0, 5.0), o, "axis"),
        ]);
        let sketch = Frame { origin: Vec3::ZERO, x: Vec3::X, y: Vec3::Z, z: -Vec3::Y };
        yack_build::sweep::revolve(&section, &sketch, Vec3::ZERO, Vec3::Z, 2.0 * PI, "h")
            .expect("hemisphere")
    }

    #[test]
    fn a_cylinder_rim_set_back_along_its_top_loses_a_triangle_ring() {
        // The ring's section is the right triangle with legs d across the top and d·tan a down the
        // side; its centroid is d/3 in from the side, so it turns at r − d/3 (Pappus).
        let c = cylinder();
        let e = rim(&c, 5.0, 10.0);
        let top = beside(&c, e, true);
        for (a, drop) in [(FRAC_PI_4, 1.0), (FRAC_PI_6, FRAC_PI_6.tan())] {
            let out = chamfer_angle(&c, e, top, 1.0, a, "k").expect("bevel");
            near(&out, 250.0 * PI - 2.0 * PI * (5.0 - 1.0 / 3.0) * drop / 2.0);
            let bevels = out
                .face_ids()
                .filter(|&f| out.face(f).names.contains(&FaceName::new("k", "bevel")))
                .count();
            assert_eq!(bevels, 1);
        }
    }

    #[test]
    fn a_cylinder_rim_set_back_down_its_side_loses_the_other_triangle_ring() {
        // Legs 1 down the side and tan 30° across the top; the centroid is tan 30°/3 in.
        let c = cylinder();
        let e = rim(&c, 5.0, 10.0);
        let side = beside(&c, e, false);
        let t = FRAC_PI_6.tan();
        let out = chamfer_angle(&c, e, side, 1.0, FRAC_PI_6, "k").expect("bevel");
        near(&out, 250.0 * PI - 2.0 * PI * (5.0 - t / 3.0) * t / 2.0);
    }

    #[test]
    fn a_hemisphere_rim_matches_cadrs() {
        // bench chamfer/hemisphere-rim-distance-{1,2}-angle-45, cadrs's volumes.
        let h = hemisphere();
        let e = rim(&h, 5.0, 0.0);
        let base = beside(&h, e, true);
        near(&chamfer_angle(&h, e, base, 1.0, FRAC_PI_4, "k").expect("d 1"), 248.05368192979518);
        near(&chamfer_angle(&h, e, base, 2.0, FRAC_PI_4, "k").expect("d 2"), 213.87538998678968);
    }

    #[test]
    fn a_ball_on_a_plate_gains_a_ring_in_its_concave_rim() {
        // bench chamfer/ball-on-plate-rim-distance-2-angle-45, cadrs's volume.
        let plate = Frame { origin: Vec3::new(0.0, 0.0, -6.0), ..Frame::WORLD };
        let plate = yack_build::primitive::cylinder(&plate, 8.0, 6.0, "p").expect("plate");
        let ball = yack_build::primitive::sphere(&Frame::WORLD, 5.0, "b").expect("ball");
        let body = yack_boolean::union(&plate, &ball).expect("ball on plate");
        let e = rim(&body, 5.0, 0.0);
        let top = beside(&body, e, true);
        near(&chamfer_angle(&body, e, top, 2.0, FRAC_PI_4, "k").expect("bevel"), 1552.9939684245546);
    }

    #[test]
    fn a_straight_edge_or_a_setback_past_the_face_declines() {
        let b = yack_build::primitive::block(&Frame::WORLD, Vec3::new(10.0, 10.0, 10.0), "b")
            .expect("block");
        let e = b.edge_ids().find(|&e| matches!(b.edge(e).curve, Some(Curve::Line(_)))).expect("edge");
        let f = b.coedge_face(b.edge_uses(e)[0]);
        assert!(matches!(chamfer_angle(&b, e, f, 1.0, FRAC_PI_4, "k"), Err(Error::Unsupported(_))));
        let c = cylinder();
        let e = rim(&c, 5.0, 10.0);
        let top = beside(&c, e, true);
        assert!(matches!(chamfer_angle(&c, e, top, 6.0, FRAC_PI_4, "k"), Err(Error::Unsupported(_))));
        assert!(matches!(chamfer_angle(&c, e, top, 1.0, FRAC_PI_2, "k"), Err(Error::InvalidInput(_))));
    }
}
