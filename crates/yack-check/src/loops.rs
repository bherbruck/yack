// yack: a loop's closure in its face's parameters, and inner loops outside their face.
// Adapted from springrep@a03e8a1 src/check.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/check.rs to yack-check (M3 W6); split out of lib.rs into its
// own module (M3 hardening); paths and visibility only.
//! A loop's closure in its face's parameters, and inner loops outside their face.

use crate::{
    geom::Surface,
    math::Vec2,
    tol,
    topo::{Brep, CoedgeId, FaceId, face_domain},
};

pub(crate) fn uv_gap(b: &Brep, face: FaceId, p: Vec2, q: Vec2) -> f64 {
    let (_, su, sv) = b.face(face).surface.d1(p.x, p.y);
    // Each parameter at its own speed, so a gap is measured alike at any scale; where one stands
    // still beside the other, at a pole, a gap in it still opens the loop in the parameter plane
    // and is measured at the other's.
    let (a, c) = (su.norm(), sv.norm());
    let wu = if a <= 1e-9 * c { c } else { a };
    let wv = if c <= 1e-9 * a { a } else { c };
    (p.x - q.x).abs() * wu + (p.y - q.y).abs() * wv
}

/// Whether the two images `p` and `q` of the tolerant vertex that coedge `c` ends at are apart by
/// no more than its diameter, as the surface carries the step between them. Each parameter's own
/// speed overstates that step where the parameters run askew, as on a blend's spline: there the
/// two speeds partly cancel.
pub(crate) fn within_vertex(b: &Brep, face: FaceId, c: CoedgeId, p: Vec2, q: Vec2) -> bool {
    let tol = b.vertex_tolerance(b.coedge_end(c));
    if tol <= tol::LINEAR {
        return false;
    }
    carried(&b.face(face).surface, p, q) <= 2.0 * (tol - tol::LINEAR) + 10.0 * tol::LINEAR
}

/// The length of the step from `p` to `q` in parameters as the surface carries it at `p`.
fn carried(s: &Surface, p: Vec2, q: Vec2) -> f64 {
    let (_, su, sv) = s.d1(p.x, p.y);
    (su * (q.x - p.x) + sv * (q.y - p.y)).norm()
}

/// The first inner loop of face `f` that the region bounded by its other loops does not hold,
/// tested at a point halfway along the loop's first coedge. A loop that winds once round a period,
/// as each rim of a band round a cylinder, bounds nothing in the parameters, so such a face has no
/// region to test.
pub(crate) fn stray_loop(b: &Brep, f: FaceId) -> Option<usize> {
    let d = face_domain(b, f);
    let periods = [d.surface.u_period(), d.surface.v_period()];
    let wraps = d.samples().iter().any(|l| {
        let mut run = [0.0f64; 2];
        for pl in l {
            let (a, z) = (pl[0], pl[pl.len() - 1]);
            run[0] += z.x - a.x;
            run[1] += z.y - a.y;
        }
        (0..2).any(|k| periods[k].is_some_and(|p| run[k].abs() > 0.5 * p))
    });
    if wraps {
        return None;
    }
    (1..d.loops.len()).find(|&li| {
        let pl = &d.samples()[li][0];
        !d.inside_without(pl[pl.len() / 2], li)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    #[test]
    fn a_step_on_a_surface_whose_parameters_run_askew_is_measured_as_it_lies_in_space() {
        // A plane whose parameters run nearly opposite ways, as a blend's spline can near a
        // corner: a step along both moves the point little, though each alone moves it far.
        let (su, sv) = (Vec3::new(10.0, 0.0, 0.0), Vec3::new(-9.9, 0.5, 0.0));
        let net = vec![Vec3::ZERO, sv, su, su + sv];
        let knots = vec![0.0, 0.0, 1.0, 1.0];
        let s = Surface::Nurbs(Box::new(
            crate::geom::NurbsSurface::new((1, 1), (knots.clone(), knots), (2, 2), net, None)
                .unwrap(),
        ));
        let (p, q) = (Vec2::new(0.2, 0.3), Vec2::new(0.21, 0.31));
        let apart = s.eval(p.x, p.y).dist(s.eval(q.x, q.y));
        assert!((carried(&s, p, q) - apart).abs() < 1e-12, "{apart}");
        // Each speed alone, as the strict test of a vertex weighs a gap, overstates it twentyfold.
        let each = (q.x - p.x).abs() * su.norm() + (q.y - p.y).abs() * sv.norm();
        assert!(each > 20.0 * apart);
    }
}
