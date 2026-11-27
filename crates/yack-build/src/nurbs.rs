// yack: a solid bounded by one rational B-spline surface that closes on itself: periodic in u, each end row in v collapsed to a pole.
//! [`nurbs_solid`]: the solid one NURBS net bounds by itself, as a ball's is: the net closes
//! round `u` (its first and last rows one curve) and pinches each end row of `v` to a point, a
//! pole. Its one face is `side`, its edge the seam along `u = u₀` from pole to pole, used once on
//! each side; the face is turned so its normal points out.

use crate::{
    Error, Result, check,
    build::{PartFace, Parts, solid_of},
    geom::{Curve, NurbsSurface, Surface},
    math::{Frame, Point3},
    tol,
    topo::{Brep, FaceName},
};

/// The solid the net `ctrl` (rows along `u`, each running along `v`) bounds, of degrees `degree`
/// and knot vectors `knots`, with optional `weights`, placed by `frame`, named under `feature`.
/// Declines (`Unsupported`) a net that does not close round `u` with a pole at each end of `v`
/// (a torus-like or open net); refuses (`InvalidInput`) a malformed net, and an end row that
/// does not pinch to a point within the tolerance.
pub fn nurbs_solid(frame: &Frame, degree: [u32; 2], knots: [&[f64]; 2], ctrl: &[Vec<Point3>], weights: Option<&[Vec<f64>]>, feature: &str) -> Result<Brep> {
    let nu = ctrl.len();
    let nv = ctrl.first().map_or(0, Vec::len);
    if nu < 2 || nv < 2 || ctrl.iter().any(|row| row.len() != nv) {
        return Err(Error::InvalidInput("a NURBS solid's net must be rectangular, two rows and columns at least".into()));
    }
    if let Some(w) = weights
        && (w.len() != nu || w.iter().any(|row| row.len() != nv))
    {
        return Err(Error::InvalidInput("a NURBS solid's weights must match its net".into()));
    }
    let (du, dv) = (usize::try_from(degree[0]).unwrap_or(usize::MAX), usize::try_from(degree[1]).unwrap_or(usize::MAX));
    let place = |p: &Point3| frame.to_world(*p);
    let flat: Vec<Point3> = ctrl.iter().flat_map(|row| row.iter().map(place)).collect();
    let flat_w = weights.map(|w| w.iter().flatten().copied().collect::<Vec<f64>>());
    let s = NurbsSurface::new((du, dv), (knots[0].to_vec(), knots[1].to_vec()), (nu, nv), flat, flat_w)?;
    // Closed round u, by the surface's own test; each end column of the net one point.
    if s.u_period().is_none() || s.v_period().is_some() {
        return Err(Error::Unsupported("a NURBS solid whose net does not close round u alone".into()));
    }
    let pinched = |j: usize| {
        let pts: Vec<Point3> = ctrl.iter().filter_map(|row| row.get(j)).map(place).collect();
        pts.first().is_some_and(|&p| pts.iter().all(|q| q.dist(p) <= tol::LINEAR))
    };
    if !(pinched(0) && pinched(nv - 1)) {
        return Err(Error::InvalidInput("a NURBS solid's end rows in v must each pinch to a point".into()));
    }
    let ((u0, _), (v0, v1)) = s.domain();
    let (bottom, top) = (s.eval(u0, v0), s.eval(u0, v1));
    if bottom.dist(top) <= tol::LINEAR {
        return Err(Error::Unsupported("a NURBS solid whose two poles meet".into()));
    }
    let seam = Curve::Nurbs(Box::new(s.iso_curve(true, u0)?));
    let names = std::iter::once(FaceName::new(feature, FaceName::SIDE)).collect();
    let solid = |reversed: bool, names: std::collections::BTreeSet<FaceName>| {
        solid_of(&Parts {
            points: vec![bottom, top],
            edges: vec![(seam.clone(), 0, 1)],
            faces: vec![PartFace { surface: Surface::Nurbs(Box::new(s.clone())), reversed, loops: vec![vec![(0, false), (0, true)]], names, shell: 0 }],
        })
    };
    let b = solid(false, names)?;
    // Turned so the face's normal points out of the solid it bounds.
    let b = match b.solid_ids().next() {
        Some(sid) if yack_measure::solid_volume_sign(&b, sid) < 0.0 => solid(true, std::iter::once(FaceName::new(feature, FaceName::SIDE)).collect())?,
        _ => b,
    };
    let issues = check::check(&b);
    if issues.is_empty() { Ok(b) } else { Err(Error::InvalidResult(issues.into_iter().map(|i| i.to_string()).collect())) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;
    use std::f64::consts::PI;

    fn ball(r: f64) -> (Vec<Vec<Point3>>, Vec<Vec<f64>>) {
        let c = [[1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [-1.0, 1.0], [-1.0, 0.0], [-1.0, -1.0], [0.0, -1.0], [1.0, -1.0], [1.0, 0.0]];
        let m = [[0.0, -r], [r, -r], [r, 0.0], [r, r], [0.0, r]];
        let net = c.iter().map(|&[x, y]| m.iter().map(|&[a, z]| Point3::new(a * x, a * y, z)).collect()).collect();
        let w = (0..9).map(|i: i32| (0..5).map(|j: i32| 0.5f64.sqrt().powi(i % 2 + j % 2)).collect()).collect();
        (net, w)
    }

    fn world() -> Frame {
        Frame::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).unwrap()
    }

    const KU: [f64; 12] = [0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0];
    const KV: [f64; 8] = [0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];

    #[test]
    fn the_rational_sphere_net_bounds_a_ball() {
        let (net, w) = ball(5.0);
        let b = nurbs_solid(&world(), [2, 2], [&KU, &KV], &net, Some(&w), "n").unwrap();
        let (v, a) = (yack_measure::volume(&b), yack_measure::area(&b));
        assert!((v - 500.0 / 3.0 * PI).abs() <= 1e-9 * v, "{v}");
        assert!((a - 100.0 * PI).abs() <= 1e-9 * a, "{a}");
        assert!(b.face_ids().all(|f| b.face(f).names.iter().any(|n| n.face == "side")));
    }

    #[test]
    fn the_same_net_run_the_other_way_still_points_out() {
        let (mut net, mut w) = ball(2.0);
        net.reverse();
        w.reverse();
        let b = nurbs_solid(&world(), [2, 2], [&KU, &KV], &net, Some(&w), "n").unwrap();
        let v = yack_measure::volume(&b);
        assert!((v - 32.0 / 3.0 * PI).abs() <= 1e-9 * v, "{v}");
    }

    #[test]
    fn an_end_row_that_does_not_pinch_is_refused() {
        let (mut net, w) = ball(5.0);
        if let Some(p) = net.get_mut(2).and_then(|row| row.get_mut(0)) {
            *p = Point3::new(1.0, 1.0, -5.0);
        }
        assert!(nurbs_solid(&world(), [2, 2], [&KU, &KV], &net, Some(&w), "n").is_err());
    }
}
