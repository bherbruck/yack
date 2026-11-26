// yack: faces offset along their normals, onto exact or fitted offset surfaces (from lib.rs, hardening T1).
// Adapted from springrep@a03e8a1 src/edit/tweak.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from edit/tweak.rs to tweak/mod.rs (M2.4, T6) and to yack-tweak (M3 W7); split out of
// lib.rs by what it does (hardening T1); the fits read their nodes by checked access (hardening T2);
// the faces offset in face-id order (hardening T3).
//! Faces offset along their normals: onto the same kind of surface where one exists, and otherwise onto
//! a spline fitted to the true offset within [`OFFSET_FIT`].

use std::collections::BTreeMap;
use crate::{Error, Result, entry, geom::Cone, geom::Curve, geom::Cylinder, geom::Plane, geom::Sphere, geom::Surface, geom::Torus, math::Point3, math::Vec2, math::Vec3, topo::Brep, topo::FaceId, unsupported, plane_of, near_uv, followed, Moved, through, tol, planes::planes_or_faces, surfaces::move_faces};

/// `b` with each face of `by` moved that far along its outward normal, onto its offset surface.
pub fn offset_faces(b: &Brep, by: &BTreeMap<FaceId, f64>) -> Result<Moved> {
    // Where a face closes on the way, a round offset past its radius or a riser between two faces
    // brought level, the offset is followed there.
    followed(offset_kept(b, by), || {
        let mut paths: BTreeMap<FaceId, through::Path> = BTreeMap::new();
        for (&f, &d) in by {
            let face = b.face(f);
            let natural = if face.reversed { -d } else { d };
            let (surface, bounds) = (face.surface.clone(), face_box(b, f));
            paths.insert(
                f,
                Box::new(move |s| match offset_surface(&surface, s * natural) {
                    Err(Error::Unsupported(_)) => offset_fitted(&surface, s * natural, bounds),
                    r => r,
                }),
            );
        }
        through::through(b, &paths)
    })
}

/// [`offset_faces`] with the solid's topology kept.
fn offset_kept(b: &Brep, by: &BTreeMap<FaceId, f64>) -> Result<Brep> {
    let mut planes = BTreeMap::new();
    let mut surfaces = BTreeMap::new();
    // In face-id order (the map's), so that of two faces whose offsets fail the same one speaks.
    for (&f, &d) in by {
        let face = b.face(f);
        let natural = if face.reversed { -d } else { d };
        if let Some(fr) = plane_of(&face.surface) {
            planes.insert(f, fr.translated(fr.z * natural));
        }
        let moved = match offset_surface(&face.surface, natural) {
            Err(Error::Unsupported(_)) => offset_fitted(&face.surface, natural, face_box(b, f))?,
            r => r?,
        };
        surfaces.insert(f, moved);
    }
    if planes.len() == surfaces.len() {
        return planes_or_faces(b, &planes);
    }
    move_faces(b, &surfaces)
}

/// How far a fitted offset may stray from the true one at any point it is checked: a tenth of the
/// linear tolerance, so the edges traced on it keep their own budget.
pub(crate) const OFFSET_FIT: f64 = 0.1 * tol::LINEAR;

/// The most samples along one parameter a fitted offset may take before it is refused.
const OFFSET_SAMPLES: usize = 1 << 13;

/// The derivative of `f` at `t`, by fourth-order differences of step `h`, one-sided where `t` lies
/// within two steps of an end of `(t0, t1)`.
fn derivative(f: &dyn Fn(f64) -> Point3, t: f64, h: f64, (t0, t1): (f64, f64)) -> Vec3 {
    if t - 2.0 * h >= t0 && t + 2.0 * h <= t1 {
        return ((f(t + h) - f(t - h)) * 8.0 - (f(t + 2.0 * h) - f(t - 2.0 * h))) / (12.0 * h);
    }
    let s = if t - 2.0 * h < t0 { h } else { -h };
    let at = |k: f64| f(t + s * k);
    (at(0.0) * -25.0 + at(1.0) * 48.0 - at(2.0) * 36.0 + at(3.0) * 16.0 - at(4.0) * 3.0)
        / (12.0 * s)
}

/// The C¹ cubic through `true_at` with its derivative at every node: the nodes are `breaks` (where
/// the curve itself may lose smoothness, so the offset may too) with each piece between them cut
/// evenly, and the pieces doubled until the cubic lies within [`OFFSET_FIT`] of `true_at` at every
/// piece's middle. Each node's knot is double, so the cubic keeps the offset's own continuity there.
fn fit_offset_curve(
    true_at: &dyn Fn(f64) -> Point3,
    (t0, t1): (f64, f64),
    period: Option<f64>,
    breaks: &[f64],
) -> Result<crate::geom::BSpline> {
    // A closed curve's offset closes too, and is fitted round its whole period.
    let closed = period.filter(|&p| t1 - t0 >= p - 1e-12);
    let (t0, t1) = match closed {
        Some(p) => (t0, t0 + p),
        None => (t0, t1),
    };
    let mut cuts: Vec<f64> = vec![t0];
    cuts.extend(
        breaks
            .iter()
            .copied()
            .filter(|&b| b > t0 + 1e-12 && b < t1 - 1e-12),
    );
    cuts.push(t1);
    let h = 1e-4 * (t1 - t0) / cuts.len() as f64;
    let mut per = 2;
    while (cuts.len() - 1) * per <= OFFSET_SAMPLES {
        let mut nodes: Vec<f64> = Vec::new();
        for w in cuts.windows(2) {
            let &[a, z] = w else { continue };
            for k in 0..per {
                nodes.push(a + (z - a) * k as f64 / per as f64);
            }
        }
        nodes.push(t1);
        let pts: Vec<Point3> = nodes.iter().map(|&t| true_at(t)).collect();
        let dirs: Vec<Vec3> = nodes
            .iter()
            .map(|&t| {
                derivative(
                    true_at,
                    t,
                    h,
                    if closed.is_some() {
                        (f64::MIN, f64::MAX)
                    } else {
                        (t0, t1)
                    },
                )
            })
            .collect();
        let m = nodes.len() - 1;
        // The span from node j to node j + 1, j < m.
        let hs: Vec<f64> = nodes.windows(2).filter_map(|w| match *w { [a, z] => Some(z - a), _ => None }).collect();
        let node = |j: usize| -> Result<(Point3, Vec3, f64)> { Ok((entry(&pts, j)?, entry(&dirs, j)?, entry(&hs, j)?)) };
        let fit = if closed.is_some() {
            // Nodes 0 … m − 1, node m being node 0 a period on: each node's two neighbouring control
            // points lie a third of the span before and after it along its derivative.
            let minus = |j: usize| -> Result<Point3> {
                let (p, d, _) = node(j)?;
                Ok(p - d * (entry(&hs, (j + m - 1) % m)? / 3.0))
            };
            let plus = |j: usize| -> Result<Point3> {
                let (p, d, h) = node(j)?;
                Ok(p + d * (h / 3.0))
            };
            let mut ctrl = vec![plus(m - 1)?];
            for j in 0..m {
                ctrl.push(minus(j)?);
                if j + 1 < m {
                    ctrl.push(plus(j)?);
                }
            }
            // Periodic: the first three control points again at the end.
            let wrap: Vec<Point3> = ctrl.iter().take(3).copied().collect();
            ctrl.extend(wrap);
            let n = 2 * m;
            let period = t1 - t0;
            let s = |k: isize| -> Result<f64> {
                let q = k.rem_euclid(n as isize) as usize;
                Ok(entry(&nodes, q / 2)? + period * k.div_euclid(n as isize) as f64)
            };
            let knots = (-3..=(n as isize + 3)).map(s).collect::<Result<Vec<f64>>>()?;
            crate::geom::BSpline::periodic_cubic(knots, ctrl)?
        } else {
            // Bézier pieces joined at double knots: an inner node is no control point, the two
            // either side of it carry it.
            let mut full = vec![entry(&pts, 0)?];
            for j in 0..m {
                let (p, d, h) = node(j)?;
                let (q, e) = (entry(&pts, j + 1)?, entry(&dirs, j + 1)?);
                full.push(p + d * (h / 3.0));
                full.push(q - e * (h / 3.0));
            }
            full.push(entry(&pts, m)?);
            let mut knots = vec![t0; 4];
            for &t in nodes.iter().skip(1).take(m.saturating_sub(1)) {
                knots.push(t);
                knots.push(t);
            }
            knots.extend([t1; 4]);
            crate::geom::BSpline::new(3, knots, full)?
        };
        let worst = nodes
            .windows(2)
            .filter_map(|w| match *w {
                [a, z] => {
                    let t = 0.5 * (a + z);
                    Some(fit.eval(t).dist(true_at(t)))
                }
                _ => None,
            })
            .fold(0.0f64, f64::max);
        if worst <= OFFSET_FIT {
            return Ok(fit);
        }
        per *= 2;
    }
    unsupported("offsetting a face whose offset no spline holds to the tolerance")
}

/// The offset of a surface with no exact offset of its own kind, `d` along its natural normal,
/// over `(u, v)` (a face's parameter box, with room): an extrusion's is the extrusion of its
/// curve's offset and a revolution's the revolution of its meridian's, so only that curve is
/// fitted; any other surface is fitted as a bicubic through a grid. Every fit is checked against
/// the true offset `p + d·n` at the middle of each span and refined until within [`OFFSET_FIT`].
pub(crate) fn offset_fitted(s: &Surface, d: f64, (u, v): ((f64, f64), (f64, f64))) -> Result<Surface> {
    let at = |a: f64, b: f64| s.eval(a, b) + s.normal(a, b) * d;
    match s {
        Surface::Extrusion(e) => {
            let breaks = e.curve.breakpoints(u.0, u.1);
            let fit = fit_offset_curve(&|t| at(t, 0.0), u, e.curve.period(), &breaks)?;
            Ok(Surface::Extrusion(Box::new(crate::geom::Extrusion {
                curve: Curve::BSpline(fit),
                dir: e.dir,
            })))
        }
        Surface::Revolution(r) => {
            let breaks = r.curve.breakpoints(v.0, v.1);
            let fit = fit_offset_curve(&|t| at(0.0, t), v, r.curve.period(), &breaks)?;
            Ok(Surface::Revolution(Box::new(crate::geom::Revolution {
                curve: Curve::BSpline(fit),
                origin: r.origin,
                axis: r.axis,
            })))
        }
        _ => {
            let mut n = 9;
            while n <= 257 {
                let us: Vec<f64> = (0..n)
                    .map(|k| u.0 + (u.1 - u.0) * k as f64 / (n - 1) as f64)
                    .collect();
                let vs: Vec<f64> = (0..n)
                    .map(|k| v.0 + (v.1 - v.0) * k as f64 / (n - 1) as f64)
                    .collect();
                // Rows through the grid, then columns through the rows' control points: the
                // tensor-product cubic through every grid point.
                let mut rows = Vec::with_capacity(n);
                for &b in &vs {
                    let pts: Vec<Point3> = us.iter().map(|&a| at(a, b)).collect();
                    rows.push(crate::geom::BSpline::interpolate_at(&pts, &us)?);
                }
                let (Some(row), true) = (rows.first(), rows.len() == n) else {
                    return Err(Error::InvalidResult(vec!["an offset fit has no rows".into()]));
                };
                let ku = row.knots().to_vec();
                let mut cols = Vec::with_capacity(n);
                for i in 0..n {
                    let pts = rows.iter().map(|r| entry(r.control_points(), i)).collect::<Result<Vec<Point3>>>()?;
                    cols.push(crate::geom::BSpline::interpolate_at(&pts, &vs)?);
                }
                let Some(col) = cols.first() else {
                    return Err(Error::InvalidResult(vec!["an offset fit has no columns".into()]));
                };
                let kv = col.knots().to_vec();
                let (pu, pv) = (row.degree(), col.degree());
                let ctrl: Vec<Point3> = cols
                    .iter()
                    .flat_map(|c| c.control_points().iter().copied())
                    .collect();
                let fit = crate::geom::NurbsSurface::new((pu, pv), (ku, kv), (n, n), ctrl, None)?;
                let mut worst = 0.0f64;
                for w in us.windows(2) {
                    let &[u0, u1] = w else { continue };
                    for z in vs.windows(2) {
                        let &[v0, v1] = z else { continue };
                        for (a, b) in [
                            (0.5 * (u0 + u1), 0.5 * (v0 + v1)),
                            (0.5 * (u0 + u1), v0),
                            (u0, 0.5 * (v0 + v1)),
                        ] {
                            worst = worst.max(fit.eval(a, b).dist(at(a, b)));
                        }
                    }
                }
                if worst <= OFFSET_FIT {
                    return Ok(Surface::Nurbs(Box::new(fit)));
                }
                n = 2 * n - 1;
            }
            unsupported("offsetting a face whose offset no spline holds to the tolerance")
        }
    }
}

/// The box of `f`'s boundary in its surface's parameters, each side grown by a tenth of its span
/// so the moved edges stay inside, and held to the surface's own domain where it has one.
fn face_box(b: &Brep, f: FaceId) -> ((f64, f64), (f64, f64)) {
    let face = b.face(f);
    let s = &face.surface;
    let (mut lo, mut hi) = (
        Vec2::new(f64::INFINITY, f64::INFINITY),
        Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for &lp in &face.loops {
        let mut last = None;
        for &c in &b.lp(lp).coedges {
            let x = b.edge(b.coedge(c).edge);
            let Some(cv) = &x.curve else { continue };
            for k in 0..=16 {
                let p = cv.eval(x.range.0 + (x.range.1 - x.range.0) * k as f64 / 16.0);
                let uv = near_uv(s, p, last);
                last = Some(uv);
                lo = Vec2::new(lo.x.min(uv.x), lo.y.min(uv.y));
                hi = Vec2::new(hi.x.max(uv.x), hi.y.max(uv.y));
            }
        }
    }
    let grow = |a: f64, z: f64| {
        let m = 0.1 * (z - a).max(1e-6);
        (a - m, z + m)
    };
    let (mut u, mut v) = (grow(lo.x, hi.x), grow(lo.y, hi.y));
    let clamp = |r: (f64, f64), dom: (f64, f64)| (r.0.max(dom.0), r.1.min(dom.1));
    match s {
        Surface::Extrusion(e) => u = clamp(u, e.curve.domain()),
        Surface::Revolution(r) => v = clamp(v, r.curve.domain()),
        Surface::Nurbs(n) => {
            let (du, dv) = n.domain();
            u = clamp(u, du);
            v = clamp(v, dv);
        }
        _ => {}
    }
    (u, v)
}

/// `s` moved `d` along its natural normal: for planes and the analytic surfaces of revolution the
/// same kind of surface, exactly. A plane moves along its normal; a cylinder's, sphere's or
/// torus's (minor) radius grows by `d`; a cone keeps its angle, its radius at the frame's origin
/// growing by `d·cos a` as the origin moves `−d·sin a` along the axis.
pub fn offset_surface(s: &Surface, d: f64) -> Result<Surface> {
    let positive = |r: f64, what: &str| {
        if r > tol::LINEAR {
            Ok(r)
        } else {
            Err(Error::InvalidInput(format!(
                "an offset of {d} mm closes a {what} to nothing"
            )))
        }
    };
    Ok(match s {
        Surface::Plane(p) => Surface::Plane(Plane {
            frame: p.frame.translated(p.frame.z * d),
        }),
        Surface::Cylinder(c) => Surface::Cylinder(Cylinder {
            frame: c.frame,
            radius: positive(c.radius + d, "cylinder")?,
        }),
        Surface::Cone(c) => {
            let (sa, ca) = c.half_angle.sin_cos();
            Surface::Cone(Cone {
                frame: c.frame.translated(c.frame.z * (-d * sa)),
                radius: c.radius + d * ca,
                half_angle: c.half_angle,
            })
        }
        Surface::Sphere(sp) => Surface::Sphere(Sphere {
            frame: sp.frame,
            radius: positive(sp.radius + d, "sphere")?,
        }),
        Surface::Torus(t) => Surface::Torus(Torus {
            frame: t.frame,
            major: t.major,
            minor: positive(t.minor + d, "torus")?,
        }),
        other => return unsupported(&format!("offsetting a {} face", other.kind())),
    })
}
