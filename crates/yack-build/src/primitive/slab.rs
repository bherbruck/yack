// yack: A slab: the solid between a frame's xy plane and a clamped B-spline height field over a
// Adapted from springrep@a03e8a1 src/primitive/slab.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/primitive/slab.rs to yack-build (M3 W7); paths and visibility only.
//! A slab: the solid between a frame's xy plane and a clamped B-spline height field over a
//! rectangle. Faces are `top`, `bottom`, and the sides `front` (y low), `right`, `back`, `left`.

use crate::{
    Error, Result, check,
    error::invalid,
    geom::{Curve, Line, NurbsCurve, NurbsSurface, Plane, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    topo::{Brep, CoedgeSpec, FaceName, PCurve},
};

const SIDES: [&str; 4] = ["front", "right", "back", "left"];

/// The degree of a clamped knot vector over `n` coefficients, and its domain.
fn clamped(knots: &[f64], n: usize) -> Result<(usize, (f64, f64))> {
    let Some(p) = knots.len().checked_sub(n + 1).filter(|&p| p >= 1) else {
        return invalid("a slab needs more knots than coefficients plus one in each direction");
    };
    let (lo, hi) = (knots[p], knots[n]);
    if knots[..=p].iter().any(|&k| k != lo) || knots[n..].iter().any(|&k| k != hi) {
        return invalid("a slab's knots must be clamped at both ends");
    }
    Ok((p, (lo, hi)))
}

/// The Greville abscissae: where each coefficient sits so the spline reproduces a linear map.
fn greville(knots: &[f64], p: usize, n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| knots[i + 1..=i + p].iter().sum::<f64>() / p as f64)
        .collect()
}

fn affine(origin: Vec2, dir: Vec2) -> PCurve {
    PCurve::Affine { origin, dir }
}

fn coedge(edge: crate::topo::EdgeId, reversed: bool, pcurve: PCurve) -> CoedgeSpec {
    CoedgeSpec {
        edge,
        reversed,
        pcurve,
    }
}

/// The solid under `z = f(x / size[0], y / size[1])` in `frame`, where `f` is the B-spline on
/// `knots` with coefficients `heights[i][j]` placed at the knots' Greville abscissae, so x and y
/// run linearly with the parameters. Every coefficient must be above the floor, which keeps the
/// whole field above it.
pub fn slab(
    frame: &Frame,
    size: [f64; 2],
    knots: (&[f64], &[f64]),
    heights: &[Vec<f64>],
    feature: &str,
) -> Result<Brep> {
    let nu = heights.len();
    let nv = heights.first().map_or(0, Vec::len);
    if heights.iter().any(|r| r.len() != nv) {
        return invalid("a slab's heights must form a grid");
    }
    if !(size[0] > 0.0 && size[1] > 0.0) {
        return invalid("a slab's size must be positive");
    }
    if heights
        .iter()
        .flatten()
        .any(|&h| !(h > 0.0 && h.is_finite()))
    {
        return invalid("a slab's heights must all lie above its floor");
    }
    let (pu, (u0, u1)) = clamped(knots.0, nu)?;
    let (pv, (v0, v1)) = clamped(knots.1, nv)?;
    let (gu, gv) = (greville(knots.0, pu, nu), greville(knots.1, pv, nv));
    let [a, b] = size;
    let at = |x: f64, y: f64, z: f64| frame.to_world(Vec3::new(x, y, z));
    let net: Vec<Point3> = (0..nu)
        .flat_map(|i| (0..nv).map(move |j| (i, j)))
        .map(|(i, j)| at(a * gu[i], b * gv[j], heights[i][j]))
        .collect();
    let top_surface = NurbsSurface::new(
        (pu, pv),
        (knots.0.to_vec(), knots.1.to_vec()),
        (nu, nv),
        net.clone(),
        None,
    )?;
    let row = |j: usize| (0..nu).map(|i| net[i * nv + j]).collect::<Vec<_>>();
    let column = |i: usize| net[i * nv..(i + 1) * nv].to_vec();
    let along_u = |ctrl| NurbsCurve::new(pu, knots.0.to_vec(), ctrl, None);
    let along_v = |ctrl| NurbsCurve::new(pv, knots.1.to_vec(), ctrl, None);

    // Corners counterclockwise from (u0, v0), and each side's top edge: its curve, its range, its
    // pcurve on the top, and whether it runs from corner i to corner i + 1.
    let corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)];
    let ends = [(0, 0), (nu - 1, 0), (nu - 1, nv - 1), (0, nv - 1)];
    let tops = [
        (
            along_u(row(0))?,
            (u0, u1),
            affine(Vec2::new(0.0, v0), Vec2::new(1.0, 0.0)),
            true,
        ),
        (
            along_v(column(nu - 1))?,
            (v0, v1),
            affine(Vec2::new(u1, 0.0), Vec2::new(0.0, 1.0)),
            true,
        ),
        (
            along_u(row(nv - 1))?,
            (u0, u1),
            affine(Vec2::new(0.0, v1), Vec2::new(1.0, 0.0)),
            false,
        ),
        (
            along_v(column(0))?,
            (v0, v1),
            affine(Vec2::new(u0, 0.0), Vec2::new(0.0, 1.0)),
            false,
        ),
    ];

    let mut s = Brep::new();
    let floor: [Point3; 4] = corners.map(|(u, v)| at(a * u, b * v, 0.0));
    let rise: [f64; 4] = ends.map(|(i, j)| heights[i][j]);
    let low = floor.map(|p| s.add_vertex(p));
    let high: Vec<_> = (0..4)
        .map(|k| s.add_vertex(floor[k] + frame.z * rise[k]))
        .collect();
    let dirs: Vec<Vec3> = (0..4)
        .map(|k| {
            (floor[(k + 1) % 4] - floor[k])
                .unit()
                .ok_or_else(|| Error::Numerical("slab side".into()))
        })
        .collect::<Result<_>>()?;
    let lens: Vec<f64> = (0..4).map(|k| floor[k].dist(floor[(k + 1) % 4])).collect();
    let bottom_e: Vec<_> = (0..4)
        .map(|k| {
            let line = Curve::Line(Line {
                origin: floor[k],
                dir: dirs[k],
            });
            s.add_edge(line, (0.0, lens[k]), low[k], low[(k + 1) % 4])
        })
        .collect();
    let rise_e: Vec<_> = (0..4)
        .map(|k| {
            let line = Curve::Line(Line {
                origin: floor[k],
                dir: frame.z,
            });
            s.add_edge(line, (0.0, rise[k]), low[k], high[k])
        })
        .collect();
    let top_e: Vec<_> = tops
        .iter()
        .enumerate()
        .map(|(k, (curve, range, _, forward))| {
            let (from, to) = (high[k], high[(k + 1) % 4]);
            let (start, end) = if *forward { (from, to) } else { (to, from) };
            s.add_edge(Curve::Nurbs(Box::new(curve.clone())), *range, start, end)
        })
        .collect();

    let name = |n: &str| [FaceName::new(feature, n)];
    let top = s.add_face(Surface::Nurbs(Box::new(top_surface)), false, name("top"));
    let bottom = s.add_face(
        Surface::Plane(Plane { frame: *frame }),
        true,
        name("bottom"),
    );
    let mut faces = vec![top, bottom];
    s.add_loop(
        top,
        (0..4)
            .map(|k| coedge(top_e[k], !tops[k].3, tops[k].2.clone()))
            .collect(),
    );
    let local = |d: Vec3| Vec2::new(d.dot(frame.x), d.dot(frame.y));
    s.add_loop(
        bottom,
        (0..4)
            .rev()
            .map(|k| {
                let o = frame.to_local(floor[k]);
                coedge(
                    bottom_e[k],
                    true,
                    affine(Vec2::new(o.x, o.y), local(dirs[k])),
                )
            })
            .collect(),
    );
    for k in 0..4 {
        let next = (k + 1) % 4;
        let plane = Frame {
            origin: floor[k],
            x: dirs[k],
            y: frame.z,
            z: dirs[k].cross(frame.z),
        };
        let face = s.add_face(
            Surface::Plane(Plane { frame: plane }),
            false,
            name(SIDES[k]),
        );
        let up = Vec2::new(0.0, 1.0);
        s.add_loop(
            face,
            vec![
                coedge(bottom_e[k], false, affine(Vec2::ZERO, Vec2::new(1.0, 0.0))),
                coedge(rise_e[next], false, affine(Vec2::new(lens[k], 0.0), up)),
                coedge(
                    top_e[k],
                    tops[k].3,
                    PCurve::Projected {
                        seed: Vec2::new(lens[k], rise[next]),
                    },
                ),
                coedge(rise_e[k], true, affine(Vec2::ZERO, up)),
            ],
        );
        faces.push(face);
    }
    let shell = s.add_shell(faces);
    s.add_solid(vec![shell]);
    let issues = check::check(&s);
    if issues.is_empty() {
        Ok(s)
    } else {
        Err(Error::InvalidResult(
            issues.into_iter().map(|i| i.to_string()).collect(),
        ))
    }
}
