// yack: a solid sewn from patches: Bézier patches on their own surfaces and flat fills of closed chains of curves, joined where their edges coincide.
//! [`sewn`]: one face per patch, welded along the edges two patches share exactly (the same
//! control points, either way round), a side whose points all coincide being a pole of its patch
//! rather than an edge. The faces must close into one shell; the solid is then turned to point out.
//! Face `k` is `patch_<k>`.

use crate::{
    Error, Result, check,
    build::{PartFace, Parts, solid_of},
    geom::{BSpline, Curve, NurbsSurface, Plane, Surface},
    math::{Frame, Point3, Vec3},
    tol,
    topo::{Brep, FaceName},
};

/// One patch of a sewn set.
#[derive(Clone, Debug)]
pub enum Patch {
    /// A Bézier patch on the control net `ctrl`: rows along u, degree rows − 1 by columns − 1.
    Bezier { ctrl: Vec<Vec<Point3>> },
    /// The flat face filling a closed chain of curves laid end to end in one plane.
    PlanarFill { boundary: Vec<BSpline> },
    /// A fill spanning a closed chain of curves that need not be flat.
    Fill { boundary: Vec<BSpline> },
}

fn decline<T>(why: &str) -> Result<T> {
    Err(Error::Unsupported(why.into()))
}

/// The solid `patches` close up, named under `feature`. Declines (`Unsupported`) a fill that is
/// not flat, a set that does not close into one shell, and an edge two patches share only to
/// within something larger than the tolerance (a gap it would have to close).
pub fn sewn(patches: &[Patch], feature: &str) -> Result<Brep> {
    if patches.is_empty() {
        return Err(Error::InvalidInput("a sewn solid needs patches".into()));
    }
    let mut parts = Parts { points: Vec::new(), edges: Vec::new(), faces: Vec::new() };
    // Each edge's control points, to find a patch's side again when another patch shares it.
    let mut edge_ctrl: Vec<Vec<Point3>> = Vec::new();
    for (k, p) in patches.iter().enumerate() {
        let names = std::iter::once(FaceName::new(feature, format!("patch_{k}"))).collect();
        let face = match p {
            Patch::Bezier { ctrl } => bezier_face(ctrl, &mut parts, &mut edge_ctrl, names)?,
            Patch::PlanarFill { boundary } => planar_face(boundary, &mut parts, &mut edge_ctrl, names)?,
            Patch::Fill { .. } => return decline("a sewn solid with a fill that is not flat"),
        };
        parts.faces.push(face);
    }
    // Every edge used exactly twice, once each way: the faces close into a shell.
    let mut uses = vec![(0usize, 0usize); parts.edges.len()];
    for f in &parts.faces {
        for lp in &f.loops {
            for &(e, rev) in lp {
                if let Some(u) = uses.get_mut(e) {
                    if rev { u.1 += 1 } else { u.0 += 1 }
                }
            }
        }
    }
    if uses.iter().any(|&(a, b)| a != 1 || b != 1) {
        return decline("a sewn set that does not close into one shell");
    }
    let b = solid_of(&parts)?;
    // Turned so the faces point out.
    let b = match b.solid_ids().next() {
        Some(sid) if yack_measure::solid_volume_sign(&b, sid) < 0.0 => {
            for f in &mut parts.faces {
                f.reversed = !f.reversed;
                for lp in &mut f.loops {
                    lp.reverse();
                    for u in lp.iter_mut() {
                        u.1 = !u.1;
                    }
                }
            }
            solid_of(&parts)?
        }
        _ => b,
    };
    let issues = check::check(&b);
    if issues.is_empty() { Ok(b) } else { Err(Error::InvalidResult(issues.into_iter().map(|i| i.to_string()).collect())) }
}

/// The point index of `p`, the one already there within the tolerance or a new one.
fn point(parts: &mut Parts, p: Point3) -> usize {
    if let Some(i) = parts.points.iter().position(|q| q.dist(p) <= tol::LINEAR) {
        return i;
    }
    parts.points.push(p);
    parts.points.len() - 1
}

/// A use of the edge on the curve through `ctrl` (a Bézier, in this order): the edge already
/// there with the same control points either way round, or a new one; `None` for a side whose
/// points all coincide (a pole: no edge). Control points that match only to within more than the
/// tolerance but less than a hundred times it are a gap this does not close, and decline.
fn edge(ctrl: &[Point3], parts: &mut Parts, edge_ctrl: &mut Vec<Vec<Point3>>) -> Result<Option<(usize, bool)>> {
    let (Some(&a), Some(&z)) = (ctrl.first(), ctrl.last()) else { return decline("a patch side with no points") };
    if ctrl.iter().all(|q| q.dist(a) <= tol::LINEAR) {
        return Ok(None);
    }
    let same = |x: &[Point3], y: &mut dyn Iterator<Item = &Point3>| x.iter().zip(y).map(|(p, q)| p.dist(*q)).fold(0.0f64, f64::max);
    for (i, known) in edge_ctrl.iter().enumerate() {
        if known.len() != ctrl.len() {
            continue;
        }
        let fwd = same(known, &mut ctrl.iter());
        let back = same(known, &mut ctrl.iter().rev());
        if fwd <= tol::LINEAR {
            return Ok(Some((i, false)));
        }
        if back <= tol::LINEAR {
            return Ok(Some((i, true)));
        }
        if fwd.min(back) <= 100.0 * tol::LINEAR {
            return decline("two patch sides that meet only to within a gap");
        }
    }
    let n = ctrl.len();
    let knots: Vec<f64> = std::iter::repeat_n(0.0, n).chain(std::iter::repeat_n(1.0, n)).collect();
    let curve = Curve::BSpline(BSpline::new(n - 1, knots, ctrl.to_vec())?);
    let (s, e) = (point(parts, a), point(parts, z));
    parts.edges.push((curve, s, e));
    edge_ctrl.push(ctrl.to_vec());
    Ok(Some((parts.edges.len() - 1, false)))
}

/// A Bézier patch's face: its tensor surface, its loop the four sides counter-clockwise in
/// (u, v), a pole side left out.
fn bezier_face(ctrl: &[Vec<Point3>], parts: &mut Parts, edge_ctrl: &mut Vec<Vec<Point3>>, names: std::collections::BTreeSet<FaceName>) -> Result<PartFace> {
    let nu = ctrl.len();
    let nv = ctrl.first().map_or(0, Vec::len);
    if nu < 2 || nv < 2 || ctrl.iter().any(|r| r.len() != nv) {
        return Err(Error::InvalidInput("a Bézier patch's net must be rectangular, two rows and columns at least".into()));
    }
    let clamped = |n: usize| -> Vec<f64> { std::iter::repeat_n(0.0, n).chain(std::iter::repeat_n(1.0, n)).collect() };
    let flat: Vec<Point3> = ctrl.iter().flatten().copied().collect();
    let surface = NurbsSurface::new((nu - 1, nv - 1), (clamped(nu), clamped(nv)), (nu, nv), flat, None)?;
    let column = |j: usize| -> Vec<Point3> { ctrl.iter().filter_map(|r| r.get(j).copied()).collect() };
    let (first, last) = (ctrl.first().cloned().unwrap_or_default(), ctrl.last().cloned().unwrap_or_default());
    // v = 0 along u, u = 1 along v, v = 1 back along u, u = 0 back along v.
    let mut v1 = column(nv - 1);
    v1.reverse();
    let mut u0 = first;
    u0.reverse();
    let sides = [column(0), last, v1, u0];
    let mut lp = Vec::new();
    for side in &sides {
        if let Some(u) = edge(side, parts, edge_ctrl)? {
            lp.push(u);
        }
    }
    Ok(PartFace { surface: Surface::Nurbs(Box::new(surface)), reversed: false, loops: vec![lp], names, shell: 0 })
}

/// A flat fill's face: the plane through its chain, the chain's curves joined end to end into one
/// loop, run so that each shared edge goes the other way from its first use; declines a chain
/// that is not flat or does not close.
fn planar_face(boundary: &[BSpline], parts: &mut Parts, edge_ctrl: &mut Vec<Vec<Point3>>, names: std::collections::BTreeSet<FaceName>) -> Result<PartFace> {
    if boundary.is_empty() {
        return decline("a flat fill with no boundary");
    }
    // The chain in order, each curve's control points turned to run on from the last.
    let mut left: Vec<Vec<Point3>> = boundary.iter().map(|c| c.control_points().to_vec()).collect();
    let mut chain: Vec<Vec<Point3>> = vec![left.remove(0)];
    while !left.is_empty() {
        let end = chain.last().and_then(|c| c.last().copied()).unwrap_or_default();
        let Some(i) = left.iter().position(|c| c.first().is_some_and(|p| p.dist(end) <= tol::LINEAR) || c.last().is_some_and(|p| p.dist(end) <= tol::LINEAR)) else {
            return decline("a flat fill whose curves do not join end to end");
        };
        let mut c = left.remove(i);
        if c.first().is_some_and(|p| p.dist(end) > tol::LINEAR) {
            c.reverse();
        }
        chain.push(c);
    }
    let (Some(start), Some(end)) = (chain.first().and_then(|c| c.first().copied()), chain.last().and_then(|c| c.last().copied())) else {
        return decline("a flat fill with no boundary");
    };
    if start.dist(end) > tol::LINEAR {
        return decline("a flat fill whose chain does not close");
    }
    // The plane: the chain's points' centroid and the normal of its polygon (Newell's), every
    // curve sample on it within the tolerance.
    let samples: Vec<Point3> = chain.iter().zip(boundary.iter().cycle()).flat_map(|(c, _)| c.iter().copied()).collect();
    let n = samples.len().max(1) as f64;
    let centre = samples.iter().fold(Vec3::new(0.0, 0.0, 0.0), |s, &p| s + p) * (1.0 / n);
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    for (p, q) in samples.iter().zip(samples.iter().cycle().skip(1)) {
        normal += (*p - centre).cross(*q - centre);
    }
    let Some(z) = normal.unit() else { return decline("a flat fill whose chain encloses no area") };
    let on_plane = boundary.iter().all(|c| {
        let (t0, t1) = c.domain();
        (0..=16).all(|k| (c.eval(t0 + (t1 - t0) * k as f64 / 16.0) - centre).dot(z).abs() <= tol::LINEAR)
    });
    if !on_plane {
        return decline("a flat fill whose curves do not lie in one plane");
    }
    let mut lp = Vec::new();
    for c in &chain {
        if let Some(u) = edge(c, parts, edge_ctrl)? {
            lp.push(u);
        }
    }
    // Run the loop the other way from its edges' other uses: those already given by patches.
    let opposed = lp.first().is_some_and(|&(e, rev)| {
        parts.faces.iter().any(|f| f.loops.iter().flatten().any(|&(e2, rev2)| e2 == e && rev2 == rev))
    });
    if opposed {
        lp.reverse();
        for u in &mut lp {
            u.1 = !u.1;
        }
    }
    // The plane's frame turned so the loop runs counter-clockwise about its normal.
    let x_hint = chain.first().and_then(|c| c.get(1).copied()).map_or(Vec3::new(1.0, 0.0, 0.0), |p| p - centre);
    let along = if opposed { -z } else { z };
    let frame = Frame::new(centre, along, x_hint)?;
    Ok(PartFace { surface: Surface::Plane(Plane { frame }), reversed: false, loops: vec![lp], names, shell: 0 })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad(a: Point3, b: Point3, c: Point3, d: Point3) -> Vec<Vec<Point3>> {
        // The bilinear patch a, b along u at v = 0, d, c at v = 1.
        vec![vec![a, d], vec![b, c]]
    }

    #[test]
    fn six_flat_patches_sew_into_a_cube() {
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        let (o, x, y, xy) = (p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0), p(0.0, 2.0, 0.0), p(2.0, 2.0, 0.0));
        let up = |q: Point3| q + Vec3::new(0.0, 0.0, 2.0);
        let patches = vec![
            Patch::Bezier { ctrl: quad(o, y, xy, x) },
            Patch::Bezier { ctrl: quad(up(o), up(x), up(xy), up(y)) },
            Patch::Bezier { ctrl: quad(o, x, up(x), up(o)) },
            Patch::Bezier { ctrl: quad(x, xy, up(xy), up(x)) },
            Patch::Bezier { ctrl: quad(xy, y, up(y), up(xy)) },
            Patch::Bezier { ctrl: quad(y, o, up(o), up(y)) },
        ];
        let b = sewn(&patches, "s").unwrap();
        let v = yack_measure::volume(&b);
        assert!((v - 8.0).abs() <= 1e-9, "{v}");
        let named = b.face_ids().filter(|&f| b.face(f).names.iter().any(|n| n.face.starts_with("patch_"))).count();
        assert_eq!(named, 6);
    }

    #[test]
    fn an_open_set_is_declined() {
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        let r = sewn(&[Patch::Bezier { ctrl: quad(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(1.0, 1.0, 0.0), p(0.0, 1.0, 0.0)) }], "s");
        assert!(matches!(r, Err(Error::Unsupported(_))), "{r:?}");
    }
}
