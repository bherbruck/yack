// yack: A slab: the solid between a frame's xy plane and a clamped B-spline height field over a
// Adapted from springrep@a03e8a1 src/primitive/slab.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/primitive/slab.rs to yack-build (M3 W7); paths and visibility only;
// M3 hardening (w-h3): the four corners and walls are walked round by iterators, each paired with
// the next, instead of indexed modulo four; the same entities are added in the same order.
//! A slab: the solid between a frame's xy plane and a clamped B-spline height field over a
//! rectangle. Faces are `top`, `bottom`, and the sides `front` (y low), `right`, `back`, `left`.
#![warn(clippy::indexing_slicing)]

use crate::{
    Error, Result, check,
    error::invalid,
    geom::{Curve, Line, NurbsCurve, NurbsSurface, Plane, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    topo::{Brep, CoedgeSpec, EdgeId, FaceName, PCurve},
};

const SIDES: [&str; 4] = ["front", "right", "back", "left"];

/// The degree of a clamped knot vector over `n` coefficients, and its domain.
fn clamped(knots: &[f64], n: usize) -> Result<(usize, (f64, f64))> {
    let Some(p) = knots.len().checked_sub(n + 1).filter(|&p| p >= 1) else {
        return invalid("a slab needs more knots than coefficients plus one in each direction");
    };
    // `p >= 1` and `n + p + 1` knots, so both ends' runs are there.
    let (Some(start), Some(end)) = (knots.get(..=p), knots.get(n..)) else {
        return invalid("a slab needs more knots than coefficients plus one in each direction");
    };
    let (Some(&lo), Some(&hi)) = (start.last(), end.first()) else {
        return invalid("a slab needs more knots than coefficients plus one in each direction");
    };
    if start.iter().any(|&k| k != lo) || end.iter().any(|&k| k != hi) {
        return invalid("a slab's knots must be clamped at both ends");
    }
    Ok((p, (lo, hi)))
}

/// The Greville abscissae: where each coefficient sits so the spline reproduces a linear map.
fn greville(knots: &[f64], p: usize, n: usize) -> Vec<f64> {
    knots
        .windows(p)
        .skip(1)
        .take(n)
        .map(|run| run.iter().sum::<f64>() / p as f64)
        .collect()
}

fn affine(origin: Vec2, dir: Vec2) -> PCurve {
    PCurve::Affine { origin, dir }
}

fn coedge(edge: EdgeId, reversed: bool, pcurve: PCurve) -> CoedgeSpec {
    CoedgeSpec { edge, reversed, pcurve }
}

/// Each item with the one after it round the loop, the last with the first.
fn round<T>(items: &[T]) -> impl Iterator<Item = (&T, &T)> {
    items.iter().zip(items.iter().cycle().skip(1))
}

/// The item after each one round the loop.
fn following<T>(items: &[T]) -> impl Iterator<Item = &T> {
    round(items).map(|(_, next)| next)
}

/// The heights at the grid's four corners `ends`.
fn corner_heights(heights: &[Vec<f64>], ends: [(usize, usize); 4]) -> Result<[f64; 4]> {
    let at = |(i, j): (usize, usize)| {
        let height = heights.get(i).and_then(|row| row.get(j)).copied();
        height.ok_or_else(|| Error::InvalidInput("a slab's heights must form a grid".into()))
    };
    let [a, b, c, d] = ends;
    Ok([at(a)?, at(b)?, at(c)?, at(d)?])
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
    let [a, b] = size;
    let nu = heights.len();
    let nv = heights.first().map_or(0, Vec::len);
    if heights.iter().any(|r| r.len() != nv) {
        return invalid("a slab's heights must form a grid");
    }
    if !(a > 0.0 && b > 0.0) {
        return invalid("a slab's size must be positive");
    }
    if heights.iter().flatten().any(|&h| !(h > 0.0 && h.is_finite())) {
        return invalid("a slab's heights must all lie above its floor");
    }
    let (pu, (u0, u1)) = clamped(knots.0, nu)?;
    let (pv, (v0, v1)) = clamped(knots.1, nv)?;
    let (gu, gv) = (greville(knots.0, pu, nu), greville(knots.1, pv, nv));
    let at = |x: f64, y: f64, z: f64| frame.to_world(Vec3::new(x, y, z));
    // The control net, row by row in `u`.
    let net: Vec<Point3> = heights
        .iter()
        .zip(&gu)
        .flat_map(|(row, &x)| row.iter().zip(&gv).map(move |(&h, &y)| at(a * x, b * y, h)))
        .collect();
    let top_surface = NurbsSurface::new(
        (pu, pv),
        (knots.0.to_vec(), knots.1.to_vec()),
        (nu, nv),
        net.clone(),
        None,
    )?;
    let stride = nv.max(1);
    let row = |j: usize| net.iter().skip(j).step_by(stride).copied().collect::<Vec<_>>();
    let column =
        |i: usize| net.get(i * nv..(i + 1) * nv).map(<[Point3]>::to_vec).unwrap_or_default();
    let along_u = |ctrl| NurbsCurve::new(pu, knots.0.to_vec(), ctrl, None);
    let along_v = |ctrl| NurbsCurve::new(pv, knots.1.to_vec(), ctrl, None);

    // Corners counterclockwise from (u0, v0), and each side's top edge: its curve, its range, its
    // pcurve on the top, and whether it runs from corner i to corner i + 1.
    let corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)];
    let ends = [(0, 0), (nu - 1, 0), (nu - 1, nv - 1), (0, nv - 1)];
    let tops = [
        (along_u(row(0))?, (u0, u1), affine(Vec2::new(0.0, v0), Vec2::new(1.0, 0.0)), true),
        (along_v(column(nu - 1))?, (v0, v1), affine(Vec2::new(u1, 0.0), Vec2::new(0.0, 1.0)), true),
        (along_u(row(nv - 1))?, (u0, u1), affine(Vec2::new(0.0, v1), Vec2::new(1.0, 0.0)), false),
        (along_v(column(0))?, (v0, v1), affine(Vec2::new(u0, 0.0), Vec2::new(0.0, 1.0)), false),
    ];

    let mut s = Brep::new();
    let floor: [Point3; 4] = corners.map(|(u, v)| at(a * u, b * v, 0.0));
    let rise = corner_heights(heights, ends)?;
    let low = floor.map(|p| s.add_vertex(p));
    let high: Vec<_> =
        floor.iter().zip(rise).map(|(&p, h)| s.add_vertex(p + frame.z * h)).collect();
    let dirs: Vec<Vec3> = round(&floor)
        .map(|(&p, &q)| (q - p).unit().ok_or_else(|| Error::Numerical("slab side".into())))
        .collect::<Result<_>>()?;
    let lens: Vec<f64> = round(&floor).map(|(p, &q)| p.dist(q)).collect();
    let bottom_e: Vec<EdgeId> = floor
        .iter()
        .zip(&dirs)
        .zip(&lens)
        .zip(round(&low))
        .map(|(((&origin, &dir), &len), (&from, &to))| {
            s.add_edge(Curve::Line(Line { origin, dir }), (0.0, len), from, to)
        })
        .collect();
    let rise_e: Vec<EdgeId> = floor
        .iter()
        .zip(rise)
        .zip(low.iter().zip(&high))
        .map(|((&origin, h), (&from, &to))| {
            s.add_edge(Curve::Line(Line { origin, dir: frame.z }), (0.0, h), from, to)
        })
        .collect();
    let top_e: Vec<EdgeId> = tops
        .iter()
        .zip(round(&high))
        .map(|((curve, range, _, forward), (&from, &to))| {
            let (start, end) = if *forward { (from, to) } else { (to, from) };
            s.add_edge(Curve::Nurbs(Box::new(curve.clone())), *range, start, end)
        })
        .collect();

    let name = |n: &str| [FaceName::new(feature, n)];
    let top = s.add_face(Surface::Nurbs(Box::new(top_surface)), false, name("top"));
    let bottom = s.add_face(Surface::Plane(Plane { frame: *frame }), true, name("bottom"));
    let mut faces = vec![top, bottom];
    s.add_loop(
        top,
        top_e
            .iter()
            .zip(&tops)
            .map(|(&edge, (_, _, pcurve, forward))| coedge(edge, !forward, pcurve.clone()))
            .collect(),
    );
    let local = |d: Vec3| Vec2::new(d.dot(frame.x), d.dot(frame.y));
    s.add_loop(
        bottom,
        floor
            .iter()
            .zip(&bottom_e)
            .zip(&dirs)
            .rev()
            .map(|((&corner, &edge), &dir)| {
                let o = frame.to_local(corner);
                coedge(edge, true, affine(Vec2::new(o.x, o.y), local(dir)))
            })
            .collect(),
    );
    // Each wall: its floor corner, direction and length; its bottom and top edges, the top's
    // pcurve and sense; the edges rising at its two ends, and the height at its far end.
    let ground = floor.iter().zip(&dirs).zip(&lens);
    let rims = bottom_e.iter().zip(&top_e).zip(&tops);
    let risers = rise_e.iter().zip(following(&rise_e)).zip(following(&rise));
    let walls = SIDES.iter().zip(ground).zip(rims).zip(risers);
    for (((side, ((&origin, &dir), &len)), ((&bottom_edge, &top_edge), rim)), risers) in walls {
        let ((&rise_edge, &next_rise_edge), &next_height) = risers;
        let plane = Frame { origin, x: dir, y: frame.z, z: dir.cross(frame.z) };
        let face = s.add_face(Surface::Plane(Plane { frame: plane }), false, name(side));
        let up = Vec2::new(0.0, 1.0);
        let top_forward = rim.3;
        s.add_loop(
            face,
            vec![
                coedge(bottom_edge, false, affine(Vec2::ZERO, Vec2::new(1.0, 0.0))),
                coedge(next_rise_edge, false, affine(Vec2::new(len, 0.0), up)),
                coedge(
                    top_edge,
                    top_forward,
                    PCurve::Projected { seed: Vec2::new(len, next_height) },
                ),
                coedge(rise_edge, true, affine(Vec2::ZERO, up)),
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
        Err(Error::InvalidResult(issues.into_iter().map(|i| i.to_string()).collect()))
    }
}
