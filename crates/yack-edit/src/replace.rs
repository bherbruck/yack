// yack: a solid with one face replaced by a patch: the face's surface becomes the patch, its corners move to the patch's, its edges become the patch's sides, each lying on the neighbour it borders.
//! [`replace_face`]: `b` with face `f` on the patch's surface instead. The patch's four sides (a
//! side whose points all coincide being a pole, not an edge) take the place of `f`'s edges, each
//! matched to the edge whose other face it lies on; the corners move to the patch's corners, and
//! every other edge through a moved corner must still pass through it. The neighbours keep their
//! surfaces: a side that leaves its neighbour's surface is refused, never fitted. The face keeps
//! its names and is also named `patch`.

use crate::{
    build::{parts_of, solid_of, param_of},
    error::{Error, Result},
    geom::{Curve, NurbsSurface, Surface},
    math::Point3,
    tol,
    topo::{Brep, FaceId, FaceName},
};

/// A patch to put in a face's place.
#[derive(Clone, Debug)]
pub enum Patch {
    /// A Bézier patch on the control net `ctrl` (rows along u).
    Bezier { ctrl: Vec<Vec<Point3>> },
    /// A B-spline patch: degrees, knot vectors, net (rows along u) and optional weights.
    Bspline { degree: [u32; 2], knots: [Vec<f64>; 2], ctrl: Vec<Vec<Point3>>, weights: Option<Vec<Vec<f64>>> },
}

fn decline<T>(why: &str) -> Result<T> {
    Err(Error::Unsupported(why.into()))
}

/// The patch's surface.
fn surface_of(p: &Patch) -> Result<NurbsSurface> {
    let net = |ctrl: &[Vec<Point3>]| -> Result<(usize, usize, Vec<Point3>)> {
        let nu = ctrl.len();
        let nv = ctrl.first().map_or(0, Vec::len);
        if nu < 2 || nv < 2 || ctrl.iter().any(|r| r.len() != nv) {
            return Err(Error::InvalidInput("a patch's net must be rectangular, two rows and columns at least".into()));
        }
        Ok((nu, nv, ctrl.iter().flatten().copied().collect()))
    };
    match p {
        Patch::Bezier { ctrl } => {
            let (nu, nv, flat) = net(ctrl)?;
            let clamped = |n: usize| -> Vec<f64> { std::iter::repeat_n(0.0, n).chain(std::iter::repeat_n(1.0, n)).collect() };
            NurbsSurface::new((nu - 1, nv - 1), (clamped(nu), clamped(nv)), (nu, nv), flat, None)
        }
        Patch::Bspline { degree, knots, ctrl, weights } => {
            let (nu, nv, flat) = net(ctrl)?;
            if let Some(w) = weights
                && (w.len() != nu || w.iter().any(|r| r.len() != nv))
            {
                return Err(Error::InvalidInput("a patch's weights must match its net".into()));
            }
            let d = |x: u32| usize::try_from(x).unwrap_or(usize::MAX);
            let [ku, kv] = knots;
            NurbsSurface::new((d(degree[0]), d(degree[1])), (ku.clone(), kv.clone()), (nu, nv), flat, weights.as_ref().map(|w| w.iter().flatten().copied().collect()))
        }
    }
}

/// How far `c` strays from surface `s` over 17 samples of its domain.
fn off_surface(c: &Curve, s: &Surface) -> f64 {
    let (t0, t1) = c.domain();
    (0..=16)
        .map(|k| {
            let p = c.eval(t0 + (t1 - t0) * k as f64 / 16.0);
            let uv = s.project(p);
            s.eval(uv.x, uv.y).dist(p)
        })
        .fold(0.0, f64::max)
}

/// `b` with face `f` replaced by `patch`, named under `feature`. Declines (`Unsupported`) a face
/// with holes, a patch with another number of sides than the face has edges, a side that lies on
/// the surface of no neighbour across the face's edges, and a moved corner another edge does not
/// pass through.
pub fn replace_face(b: &Brep, f: FaceId, patch: &Patch, feature: &str) -> Result<Brep> {
    let surface = surface_of(patch)?;
    let mut parts = parts_of(b)?;
    // The part face of `f`: parts_of lists faces shell by shell in each shell's order.
    let order: Vec<FaceId> = b.solid_ids().flat_map(|s| b.solid(s).shells.clone()).flat_map(|sh| b.shell(sh).faces.clone()).collect();
    let Some(k) = order.iter().position(|&g| g == f) else { return decline("a face outside the solid's shells") };
    let Some(target) = parts.faces.get(k).cloned() else { return decline("a face the parts do not hold") };
    let [lp] = target.loops.as_slice() else { return decline("replacing a face with holes") };
    // The patch's sides along its boundary, a pole side left out.
    let ((u0, u1), (v0, v1)) = surface.domain();
    let mut sides: Vec<Curve> = Vec::new();
    for (along_v, at) in [(false, v0), (true, u1), (false, v1), (true, u0)] {
        let c = surface.iso_curve(along_v, at)?;
        let (t0, t1) = c.domain();
        let first = c.eval(t0);
        if [c.eval(0.5 * (t0 + t1)), c.eval(t1)].iter().all(|p| p.dist(first) <= tol::LINEAR) {
            continue;
        }
        sides.push(Curve::Nurbs(Box::new(c)));
    }
    if sides.len() != lp.len() {
        return decline("a patch with another number of sides than the face has edges");
    }
    // Each edge of the face: the neighbour across it and the side that lies on that neighbour.
    let mut taken = vec![false; sides.len()];
    let mut moves: Vec<(usize, Point3)> = Vec::new();
    let mut flips: Vec<usize> = Vec::new();
    for &(e, _) in lp {
        let neighbour = parts.faces.iter().enumerate().find(|(i, g)| *i != k && g.loops.iter().flatten().any(|&(x, _)| x == e)).map(|(_, g)| g.surface.clone());
        let Some(ns) = neighbour else { return decline("an edge of the face with no neighbour") };
        let Some(si) = sides.iter().zip(&taken).position(|(c, &used)| !used && off_surface(c, &ns) <= tol::LINEAR) else {
            return decline("a patch side that leaves the surface of the face it borders");
        };
        if let Some(t) = taken.get_mut(si) {
            *t = true;
        }
        let Some((curve, s, t)) = parts.edges.get_mut(e) else { return decline("an edge the parts do not hold") };
        let Some(side) = sides.get(si).cloned() else { return decline("a patch side the patch does not have") };
        let (a, z) = { let (t0, t1) = side.domain(); (side.eval(t0), side.eval(t1)) };
        // The side runs the edge's way or the other: its ends go to the edge's ends nearest them.
        let old = (parts.points.get(*s).copied(), parts.points.get(*t).copied());
        let ps = match old {
            (Some(p), Some(q)) if p.dist(a) + q.dist(z) <= p.dist(z) + q.dist(a) => a,
            (Some(_), Some(_)) => z,
            _ => return decline("an edge whose ends the parts do not hold"),
        };
        // The edge now runs along the side: from the end the side starts at.
        let forward = ps.dist(a) <= tol::LINEAR;
        *curve = side;
        if !forward {
            std::mem::swap(s, t);
            flips.push(e);
        }
        moves.push((*s, a));
        moves.push((*t, z));
    }
    // An edge turned to run along its side is used the other way by every face.
    for g in &mut parts.faces {
        for lp in &mut g.loops {
            for u in lp.iter_mut() {
                if flips.contains(&u.0) {
                    u.1 = !u.1;
                }
            }
        }
    }
    for (i, p) in moves {
        if let Some(q) = parts.points.get_mut(i) {
            *q = p;
        }
    }
    // Every edge still meets its moved ends.
    for (c, s, t) in &parts.edges {
        for v in [*s, *t] {
            let Some(&p) = parts.points.get(v) else { return decline("an edge whose ends the parts do not hold") };
            if c.eval(param_of(c, p)).dist(p) > tol::LINEAR {
                return decline("an edge that no longer reaches a corner the patch moved");
            }
        }
    }
    // The face on the patch, its sense kept: the patch's normal at its middle against the old one.
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let new_s = Surface::Nurbs(Box::new(surface.clone()));
    let p_mid = new_s.eval(um, vm);
    let new_n = new_s.normal(um, vm);
    let uv = target.surface.project(p_mid);
    let old_n = target.surface.normal(uv.x, uv.y) * if target.reversed { -1.0 } else { 1.0 };
    let reversed_new = new_n.dot(old_n) < 0.0;
    if let Some(face) = parts.faces.get_mut(k) {
        face.surface = new_s;
        face.reversed = reversed_new;
        face.names.insert(FaceName::new(feature, "patch"));
    }
    solid_of(&parts)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Frame, Vec3};

    fn top_of(b: &Brep, z: f64) -> FaceId {
        b.face_ids().find(|&g| (crate::measure::face_bbox(b, g).min.z - z).abs() < 1e-9 && (crate::measure::face_bbox(b, g).max.z - z).abs() < 1e-9).unwrap()
    }

    fn block(x: f64, y: f64, z: f64) -> Brep {
        crate::primitive::block(&Frame::WORLD, Vec3::new(x, y, z), "b").unwrap()
    }

    #[test]
    fn a_flat_patch_in_the_tops_place_keeps_the_box() {
        // The top of a 10 × 10 × 4 box as a bicubic Bézier net lying in z = 4: still 400.
        let b = block(10.0, 10.0, 4.0);
        let net = |u: &[f64], v: &[f64]| u.iter().map(|&x| v.iter().map(|&y| Point3::new(x, y, 4.0)).collect()).collect();
        let r = replace_face(&b, top_of(&b, 4.0), &Patch::Bezier { ctrl: net(&[0.0, 2.0, 7.0, 10.0], &[0.0, 4.0, 5.0, 10.0]) }, "p").unwrap();
        let v = crate::measure::volume(&r);
        assert!((v - 400.0).abs() <= 1e-9 * 400.0, "{v}");
        assert!(crate::check::check(&r).is_empty(), "{:?}", crate::check::check(&r));
        assert!(r.face_ids().any(|g| r.face(g).names.iter().any(|n| n.face == "patch")));
    }

    #[test]
    fn a_tilted_patch_moves_the_corners_up_the_side_edges() {
        // A 20 × 20 × 10 box whose top becomes the plane through z = 10 along y = 0 and z = 14
        // along y = 20 (a bilinear patch, its sides lines in the side planes): 20·20·12.
        let b = block(20.0, 20.0, 10.0);
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        let ctrl = vec![vec![p(0.0, 0.0, 10.0), p(0.0, 20.0, 14.0)], vec![p(20.0, 0.0, 10.0), p(20.0, 20.0, 14.0)]];
        let r = replace_face(&b, top_of(&b, 10.0), &Patch::Bezier { ctrl }, "p").unwrap();
        let v = crate::measure::volume(&r);
        assert!((v - 4800.0).abs() <= 1e-9 * 4800.0, "{v}");
        assert!(crate::check::check(&r).is_empty(), "{:?}", crate::check::check(&r));
    }

    #[test]
    fn a_side_that_leaves_its_neighbour_is_refused() {
        // The same patch, its u = 0 side bowed out of the plane x = 0.
        let b = block(20.0, 20.0, 10.0);
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        let ctrl = vec![
            vec![p(0.0, 0.0, 10.0), p(-3.0, 10.0, 10.0), p(0.0, 20.0, 10.0)],
            vec![p(20.0, 0.0, 10.0), p(20.0, 10.0, 10.0), p(20.0, 20.0, 10.0)],
        ];
        let r = replace_face(&b, top_of(&b, 10.0), &Patch::Bezier { ctrl }, "p");
        assert!(matches!(r, Err(Error::Unsupported(_))), "{r:?}");
    }
}
