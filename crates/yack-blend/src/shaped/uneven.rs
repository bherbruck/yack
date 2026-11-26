// yack: The corner where three fillets of unequal radii meet, closed without a crease.
// Adapted from springrep@a03e8a1 src/blend/shaped/uneven.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/blend/shaped/uneven.rs to yack-blend (M3 W9a); paths and visibility only;
// hardened (M3 L5): checked access in place of indexing (a corner's threes through `of3`), its small
// literals named.
//! The corner where three fillets of unequal radii meet, closed without a crease.
//!
//! A ball of radius `ρ` touching two of the corner's planes has its centre `ρ` in from both, and
//! touching all three, at the one point `ρ` in from each. The corner is closed by the ball of the
//! largest radius `R` touching all three. Along each edge whose fillet is smaller the ball rolls
//! on from there touching that edge's two planes and shrinking evenly as it goes, down to the
//! fillet's radius over a travel of `R`, and from there on it is the fillet's own ball. Its centre
//! runs on a line while it shrinks, so what it sweeps is a cone, tangent to both planes along
//! lines and to its first and last ball along circles. An edge whose fillet is the largest has
//! the ball roll the same travel unchanged, sweeping a cylinder. Everything the ball touches is a
//! plane, a cylinder, a cone or a sphere, each meeting the next tangentially: the corner ball
//! between the three sweeps' circles, each sweep between its two circles, and the cap of a
//! cone's last ball between the cone's circle and the fillet's end.

use std::collections::BTreeSet;

use crate::boolean::half_space;
use crate::{
    boolean,
    error::{Error, Result},
    geom::{Circle, Cone, Curve, Cylinder, Line, Sphere, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    measure,
    profile::Profile,
    build::{PartFace, Parts, parts_of, solid_of},
    sweep::extrude,
    tol,
    topo::{Brep, EdgeId, FaceName, VertexId},
};

/// Entry `j` of a corner's three (its faces, edges, radii), counted round the corner: `j` past 2
/// wraps.
fn of3<T>(xs: &[T; 3], j: usize) -> &T {
    let [a, b, c] = xs;
    match j % 3 {
        0 => a,
        1 => b,
        _ => c,
    }
}

/// [`of3`], to change.
fn of3_mut<T>(xs: &mut [T; 3], j: usize) -> &mut T {
    let [a, b, c] = xs;
    match j % 3 {
        0 => a,
        1 => b,
        _ => c,
    }
}

/// Unit normals whose dot product is within this of 1 are one; an edge whose direction's dot with
/// a face normal is within this of 0 lies in that face.
const SAME_COS: f64 = 1e-9; // tol: a cosine, not a length

/// A corner of three planes whose three edges are rounded at radii not all equal. Edge `j` lies
/// between faces `j + 1` and `j + 2`.
struct Corner {
    vertex: VertexId,
    at: Point3,
    /// Each face's unit normal into the solid.
    a: [Vec3; 3],
    /// Each edge's unit direction away from the corner, and its length and radius.
    e: [Vec3; 3],
    len: [f64; 3],
    r: [f64; 3],
    edges: [EdgeId; 3],
}

impl Corner {
    fn largest(&self) -> f64 {
        self.r.iter().copied().fold(0.0, f64::max)
    }

    /// Where a ball touching edge `j`'s two faces has its centre, for each unit of its radius:
    /// square to the edge, one in from both.
    fn in_from(&self, j: usize) -> Vec3 {
        let (p, q) = (*of3(&self.a, j + 1), *of3(&self.a, j + 2));
        (p + q) / (1.0 + p.dot(q))
    }

    /// How far along edge `j` the ball touching all three faces sits, for each unit of its
    /// radius.
    fn along(&self, j: usize) -> f64 {
        (1.0 - self.in_from(j).dot(*of3(&self.a, j))) / of3(&self.e, j).dot(*of3(&self.a, j))
    }

    /// The centre of the ball of the largest radius touching all three faces.
    fn centre(&self) -> Point3 {
        self.at + (self.in_from(0) + self.e[0] * self.along(0)) * self.largest()
    }

    fn shrinks(&self, j: usize) -> bool {
        self.largest() - *of3(&self.r, j) > tol::LINEAR
    }

    /// How far along edge `j` its own fillet starts: where the corner's ball sits, and its
    /// radius on.
    fn start(&self, j: usize) -> f64 {
        self.largest() * (self.along(j) + 1.0)
    }

    /// The centre of edge `j`'s fillet's first ball.
    fn first(&self, j: usize) -> Point3 {
        self.at + *of3(&self.e, j) * self.start(j) + self.in_from(j) * *of3(&self.r, j)
    }

    /// What the ball sweeps from the corner along edge `j`: its unit axis toward the corner's
    /// ball, and its half angle, nought where the ball keeps its radius.
    fn cone(&self, j: usize) -> (Vec3, f64) {
        let run = self.centre() - self.first(j);
        (
            run.unit_or_zero(),
            ((self.largest() - *of3(&self.r, j)) / run.norm()).asin(),
        )
    }
}

/// The corners of `s` where three of `edges`, straight between planes and convex, meet at radii
/// not all equal, each edge long enough for its ball to reach its own radius.
fn corners(s: &Brep, edges: &[(EdgeId, f64)]) -> Vec<Corner> {
    let mut out = Vec::new();
    for v in s.vertex_ids() {
        let ends = |e: EdgeId| [s.edge(e).start, s.edge(e).end];
        let here: Vec<(EdgeId, f64)> = edges
            .iter()
            .copied()
            .filter(|&(e, _)| ends(e).contains(&v))
            .collect();
        let all = s
            .edge_ids()
            .filter(|&e| !s.edge(e).is_degenerate() && ends(e).contains(&v))
            .count();
        let (lo, hi) = here.iter().fold((f64::INFINITY, 0.0f64), |(a, b), x| {
            (a.min(x.1), b.max(x.1))
        });
        if here.len() != 3 || all != 3 || hi - lo <= tol::LINEAR {
            continue;
        }
        let at = s.vertex(v).point;
        let mut normals: Vec<Vec3> = Vec::new();
        let mut flat = true;
        for co in s.coedge_ids() {
            if !here.iter().any(|x| x.0 == s.coedge(co).edge) {
                continue;
            }
            let face = s.face(s.coedge_face(co));
            let Surface::Plane(p) = &face.surface else {
                flat = false;
                break;
            };
            let n = if face.reversed { -p.frame.z } else { p.frame.z };
            if !normals.iter().any(|m| m.dot(n) > 1.0 - SAME_COS) {
                normals.push(n);
            }
        }
        let straight = here
            .iter()
            .all(|x| matches!(s.edge(x.0).curve, Some(Curve::Line(_))));
        let (true, true, &[n0, n1, n2], Some(&(first_edge, _))) = (flat, straight, normals.as_slice(), here.first()) else {
            continue;
        };
        let far = |e: EdgeId| {
            let [p, q] = ends(e);
            s.vertex(if p == v { q } else { p }).point
        };
        // Edge j is the one that lies in neither of the faces after it: it leaves face j.
        let mut c = Corner {
            vertex: v,
            at,
            a: [-n0, -n1, -n2],
            e: [Vec3::ZERO; 3],
            len: [0.0; 3],
            r: [0.0; 3],
            edges: [first_edge; 3],
        };
        let mut placed = 0;
        for &(e, r) in &here {
            let run = far(e) - at;
            let Some(d) = run.unit() else { continue };
            let leaves: Vec<usize> = (0..3).filter(|&j| d.dot(*of3(&c.a, j)).abs() > SAME_COS).collect();
            if let [j] = leaves[..]
                && d.dot(*of3(&c.a, j)) > 0.0
            {
                *of3_mut(&mut c.e, j) = d;
                *of3_mut(&mut c.len, j) = run.norm();
                *of3_mut(&mut c.r, j) = r;
                *of3_mut(&mut c.edges, j) = e;
                placed |= 1 << j;
            }
        }
        if placed == 7 && (0..3).all(|j| *of3(&c.len, j) > c.start(j) + *of3(&c.r, j)) {
            out.push(c);
        }
    }
    out
}

/// The arc of the circle about `centre` from `from` round to `to`, the short way.
fn arc(centre: Point3, from: Point3, to: Point3) -> Result<(Curve, Point3)> {
    let (p, q) = (from - centre, to - centre);
    let frame = Frame::new(centre, p.cross(q), p)?;
    let mid = centre + (p + q).unit_or_zero() * p.norm();
    Ok((
        Curve::Circle(Circle {
            frame,
            radius: p.norm(),
        }),
        mid,
    ))
}

/// One side of a new face: its edge, the vertices it runs between, and a point midway.
#[derive(Clone, Copy)]
struct Side {
    edge: usize,
    from: usize,
    to: usize,
    mid: Point3,
}

/// The parts being put together, and the corner's side of them.
struct Build {
    parts: Parts,
    shell: usize,
    /// A direction every new face's outward normal leans along: toward the corner from inside.
    out: Vec3,
}

impl Build {
    fn found(&self, p: Point3) -> Option<usize> {
        self.parts
            .points
            .iter()
            .position(|q| q.dist(p) <= 10.0 * tol::LINEAR)
    }

    fn vertex(&mut self, p: Point3) -> usize {
        self.found(p).unwrap_or_else(|| {
            self.parts.points.push(p);
            self.parts.points.len() - 1
        })
    }

    fn between(&self, v: usize, w: usize) -> Option<usize> {
        self.parts
            .edges
            .iter()
            .position(|&(_, p, q)| (p, q) == (v, w) || (p, q) == (w, v))
    }

    fn side(&mut self, curve: Curve, from: usize, to: usize, mid: Point3) -> Side {
        self.parts.edges.push((curve, from, to));
        Side {
            edge: self.parts.edges.len() - 1,
            from,
            to,
            mid,
        }
    }

    fn point(&self, i: usize) -> Result<Point3> {
        self.parts.points.get(i).copied().ok_or_else(|| Error::Numerical("a corner's point that is not there".into()))
    }

    fn line(&mut self, from: usize, to: usize) -> Result<Side> {
        let (p, q) = (self.point(from)?, self.point(to)?);
        let dir = (q - p)
            .unit()
            .ok_or_else(|| Error::InvalidInput("a corner's line of no length".into()))?;
        Ok(self.side(
            Curve::Line(Line { origin: p, dir }),
            from,
            to,
            p.lerp(q, 0.5),
        ))
    }

    fn arc(&mut self, centre: Point3, from: usize, to: usize) -> Result<Side> {
        let (curve, mid) = arc(centre, self.point(from)?, self.point(to)?)?;
        Ok(self.side(curve, from, to, mid))
    }

    /// A face on `surface` bounded by `sides`, each running on from the last, either way round.
    fn face(&mut self, surface: Surface, name: String, mut sides: Vec<Side>) -> Result<()> {
        let mut ring = Vec::new();
        for s in &sides {
            ring.push(self.point(s.from)?);
            ring.push(s.mid);
        }
        let middle = ring.iter().fold(Vec3::ZERO, |sum, &p| sum + p) / ring.len() as f64;
        // each point with the next, the last with the first
        let turn = ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .fold(Vec3::ZERO, |sum, (&p, &q)| sum + (p - middle).cross(q - middle));
        if turn.dot(self.out) < 0.0 {
            sides.reverse();
            for s in &mut sides {
                (s.from, s.to) = (s.to, s.from);
            }
        }
        let first = sides.first().ok_or_else(|| Error::Numerical("a corner face with no sides".into()))?;
        let uv = surface.project(first.mid);
        let reversed = surface.normal(uv.x, uv.y).dot(self.out) < 0.0;
        let uses = sides
            .iter()
            .map(|s| Some((s.edge, self.parts.edges.get(s.edge)?.1 != s.from)))
            .collect::<Option<_>>()
            .ok_or_else(|| Error::Numerical("a corner side that is not there".into()))?;
        self.parts.faces.push(PartFace {
            surface,
            reversed,
            loops: vec![uses],
            names: BTreeSet::from([FaceName::new("\u{1}fil", name)]),
            shell: self.shell,
        });
        Ok(())
    }
}

/// `parts`, a solid whose corner `c` was cut away as far as each edge's fillet starts, with that
/// corner closed; `None` where the fillets do not end there as fillets of their radii do.
fn closed(parts: Parts, c: &Corner, at: usize) -> Result<Option<Parts>> {
    let big = c.largest();
    let centre = c.centre();
    let foot = |ball: Point3, rho: f64, face: usize| ball - *of3(&c.a, face) * rho;
    let mut b = Build {
        parts,
        shell: 0,
        out: (c.at - centre).unit_or_zero(),
    };
    let (next, prev) = (|j: usize| (j + 1) % 3, |j: usize| (j + 2) % 3);
    // Where each edge's fillet ends on its two faces, and where the cut's planes cross on each.
    let mut ends = [[0; 2]; 3];
    let mut cross = [0; 3];
    let mut round = [0; 3];
    for j in 0..3 {
        let first = c.first(j);
        let rj = *of3(&c.r, j);
        let (Some(p), Some(q)) = (b.found(foot(first, rj, next(j))), b.found(foot(first, rj, prev(j)))) else {
            return Ok(None);
        };
        *of3_mut(&mut ends, j) = [p, q];
        let Some(e) = b.between(p, q) else {
            return Ok(None);
        };
        *of3_mut(&mut round, j) = e;
        // On face j, its edges' cuts cross where each is as far along as its fillet starts.
        let (u, v) = (*of3(&c.e, next(j)), *of3(&c.e, prev(j)));
        let (s, t) = (c.start(next(j)), c.start(prev(j)));
        let k = u.dot(v);
        let p = c.at + u * ((s - k * t) / (1.0 - k * k)) + v * ((t - k * s) / (1.0 - k * k));
        let Some(x) = b.found(p) else {
            return Ok(None);
        };
        *of3_mut(&mut cross, j) = x;
    }
    // The cut's faces go: each the flat one on a fillet's end.
    let mut cut = Vec::new();
    for e in round {
        let uses = |f: &PartFace| f.loops.iter().flatten().any(|u| u.0 == e);
        let flat = |f: &PartFace| matches!(f.surface, Surface::Plane(_));
        let Some(f) = b.parts.faces.iter().position(|f| uses(f) && flat(f)) else {
            return Ok(None);
        };
        cut.push(f);
    }
    b.shell = cut.first().and_then(|&f| b.parts.faces.get(f)).map_or(0, |f| f.shell);
    cut.sort_unstable();
    cut.dedup();
    for &f in cut.iter().rev() {
        b.parts.faces.remove(f);
    }
    let feet = [0, 1, 2].map(|i| b.vertex(foot(centre, big, i)));
    // Each edge's side of the corner ball's patch, from the foot on the face after it to the
    // foot on the face before: its sweep's circle on the ball.
    let mut sides = Vec::new();
    // The lines each sweep touches its two faces along, from its fillet's end to the ball's
    // foot.
    let mut touch = Vec::new();
    for j in 0..3 {
        let ([end_from, end_to], rj) = (*of3(&ends, j), *of3(&c.r, j));
        let (p, q) = (b.point(end_from)?, b.point(end_to)?);
        let own = Side {
            edge: *of3(&round, j),
            from: end_from,
            to: end_to,
            mid: c.first(j) + ((p + q) * 0.5 - c.first(j)).unit_or_zero() * rj,
        };
        let (axis, half) = c.cone(j);
        let first = c.first(j);
        let small = match c.shrinks(j) {
            true => b.arc(first - axis * (rj * half.sin()), end_from, end_to)?,
            false => own,
        };
        let (foot_next, foot_prev) = (*of3(&feet, next(j)), *of3(&feet, prev(j)));
        let large = b.arc(centre - axis * (big * half.sin()), foot_next, foot_prev)?;
        let lines = [b.line(end_from, foot_next)?, b.line(end_to, foot_prev)?];
        let back = |s: Side| Side {
            from: s.to,
            to: s.from,
            ..s
        };
        let away = c.in_from(j);
        let sweep = if c.shrinks(j) {
            b.face(
                Surface::Sphere(Sphere {
                    frame: Frame::new(first, *of3(&c.e, j), away)?,
                    radius: rj,
                }),
                format!("corner{at}_cap{j}"),
                vec![own, back(small)],
            )?;
            Surface::Cone(Cone {
                frame: Frame::new(first - axis * (rj * half.sin()), axis, away)?,
                radius: rj * half.cos(),
                half_angle: half,
            })
        } else {
            Surface::Cylinder(Cylinder {
                frame: Frame::new(first, axis, away)?,
                radius: big,
            })
        };
        b.face(
            sweep,
            format!("corner{at}_sweep{j}"),
            vec![small, lines[1], back(large), back(lines[0])],
        )?;
        sides.push(large);
        touch.push(lines);
    }
    let inward = centre - c.at;
    let &[side0, side1, side2] = sides.as_slice() else {
        return Ok(None);
    };
    b.face(
        Surface::Sphere(Sphere {
            frame: Frame::new(centre, inward.cross(c.e[0]), inward)?,
            radius: big,
        }),
        format!("corner{at}_ball"),
        vec![side2, side0, side1],
    )?;
    // Each face's corner the cut took comes back as far as the lines its two cones touch it
    // along: on face i those of the edge before it, then the edge after it.
    for i in 0..3 {
        let x = *of3(&cross, i);
        let (Some(l1), Some(l2)) = (b.between(of3(&ends, prev(i))[0], x), b.between(of3(&ends, next(i))[1], x)) else {
            return Ok(None);
        };
        let Some(f) = b.parts.faces.iter().position(|f| {
            let has = |e: usize| f.loops.iter().flatten().any(|u| u.0 == e);
            has(l1) && has(l2)
        }) else {
            return Ok(None);
        };
        let (Some(&[a, _]), Some(&[_, z])) = (touch.get(prev(i)), touch.get(next(i))) else {
            return Ok(None);
        };
        let edges = &b.parts.edges;
        let Some(face) = b.parts.faces.get_mut(f) else {
            return Ok(None);
        };
        for lp in &mut face.loops {
            let (Some(p1), Some(p2)) = (
                lp.iter().position(|u| u.0 == l1),
                lp.iter().position(|u| u.0 == l2),
            ) else {
                continue;
            };
            // From the first fillet's end to the foot and on to the second's, or back.
            let forward = (p1 + 1) % lp.len() == p2;
            let back = |s: Side| Side {
                from: s.to,
                to: s.from,
                ..s
            };
            let run = if forward { [a, back(z)] } else { [z, back(a)] };
            let Some(uses) = run.iter().map(|s| Some((s.edge, edges.get(s.edge)?.1 != s.from))).collect::<Option<Vec<(usize, bool)>>>()
            else {
                return Ok(None);
            };
            let (lo, hi) = (p1.min(p2), p1.max(p2));
            if (lo, hi) == (0, lp.len() - 1) {
                lp.pop();
                lp.remove(0);
                lp.extend(uses);
            } else {
                lp.splice(lo..=hi, uses);
            }
        }
    }
    Ok(Some(b.parts))
}

/// `s` with each of `edges` rounded at its own radius and every corner where three of them meet
/// at unequal radii closed without a crease; `None` where it has no such corner, or one does not
/// come out as this module builds it.
pub(super) fn rounded(s: &Brep, edges: &[(EdgeId, f64)]) -> Result<Option<Brep>> {
    let corners = corners(s, edges);
    if corners.is_empty() {
        return Ok(None);
    }
    // Each edge is found again, once the corners are cut away, by the middle of what is left.
    let mut marks = Vec::new();
    for &(e, r) in edges {
        let x = s.edge(e);
        let (p, q) = (s.vertex(x.start).point, s.vertex(x.end).point);
        let taken = |v: VertexId| {
            corners
                .iter()
                .filter(|c| c.vertex == v)
                .filter_map(|c| (0..3).find(|&j| *of3(&c.edges, j) == e).map(|j| c.start(j)))
                .sum::<f64>()
        };
        let (from, to) = (taken(x.start), p.dist(q) - taken(x.end));
        if to - from <= r {
            return Ok(None);
        }
        let dir = (q - p).unit_or_zero();
        marks.push((p + dir * (0.5 * (from + to)), r));
    }
    let mut cut = s.clone();
    for c in &corners {
        let reach = 4.0 * (0..3).map(|j| c.start(j)).fold(0.0, f64::max);
        let frame = Frame::new(c.at - Vec3::Z * reach, Vec3::Z, Vec3::X)?;
        let mut cell = extrude(
            &Profile::rect(Vec2::ZERO, 2.0 * reach, 2.0 * reach),
            &frame,
            2.0 * reach,
            "\u{1}corner",
        )?;
        for j in 0..3 {
            let side = half_space(s, c.at + *of3(&c.e, j) * c.start(j), -*of3(&c.e, j))?;
            cell = boolean::intersection(&cell, &side)?;
        }
        cut = boolean::difference(&cut, &cell)?;
    }
    let mut radii: Vec<f64> = edges.iter().map(|e| e.1).collect();
    radii.sort_by(|a, b| b.total_cmp(a));
    radii.dedup();
    let mut out: Option<Brep> = None;
    for r in radii {
        let mut these = Vec::new();
        for &(p, _) in marks.iter().filter(|m| m.1 == r) {
            let Some((e, d)) = measure::edge_at(&cut, p) else {
                return Ok(None);
            };
            if d > 10.0 * tol::LINEAR {
                return Ok(None);
            }
            these.push(e);
        }
        let round = crate::fillet_edges(&cut, &these, r, "\u{1}fil")?;
        out = Some(match out {
            None => round,
            Some(o) => boolean::intersection(&o, &round)?,
        });
    }
    let Some(out) = out else { return Ok(None) };
    let mut parts = parts_of(&out)?;
    for (at, c) in corners.iter().enumerate() {
        let Some(next) = closed(parts, c, at)? else {
            return Ok(None);
        };
        parts = next;
    }
    solid_of(&parts).map(Some)
}

#[cfg(test)]
mod tests {
    use super::super::fillet_radii_raw as fillet_radii;
    use super::*;
    use crate::check::check;
    use std::f64::consts::PI;

    /// A cube of side 12 with the three edges at its corner (12, 12, 12) rounded at `radii`,
    /// the edge along x first.
    fn corner(radii: [f64; 3]) -> Brep {
        let b = extrude(
            &Profile::rect(Vec2::new(6.0, 6.0), 12.0, 12.0),
            &Frame::WORLD,
            12.0,
            "p",
        )
        .unwrap();
        let at = |x, y, z| measure::edge_at(&b, Point3::new(x, y, z)).unwrap().0;
        let edges = [
            (at(6.0, 12.0, 12.0), radii[0]),
            (at(12.0, 6.0, 12.0), radii[1]),
            (at(12.0, 12.0, 6.0), radii[2]),
        ];
        fillet_radii(&b, &edges).unwrap()
    }

    #[test]
    fn rounds_of_three_radii_at_a_corner_close_without_a_crease() {
        // Rounds of 2, 1.5 and 1, then two alike and one smaller, then one larger. Within 5 of
        // the corner every edge the rounds made lies between faces that share their tangent
        // plane all along it; outside the cube of side 5 there each round takes (1 − π/4)r² a
        // unit of its edge; and the whole lies between the cube rounded at the largest radius
        // and at the smallest, a ball closing each of those.
        for radii in [[2.0, 1.5, 1.0], [2.0, 2.0, 1.2], [1.0, 1.0, 1.8]] {
            let out = corner(radii);
            assert!(check(&out).is_empty(), "{radii:?}: {:?}", check(&out));
            let tip = Point3::new(12.0, 12.0, 12.0);
            let mut smooth = 0;
            for e in out.edge_ids() {
                let x = out.edge(e);
                let Some(c) = &x.curve else { continue };
                let faces: Vec<_> = out
                    .edge_uses(e)
                    .iter()
                    .map(|&u| out.coedge_face(u))
                    .collect();
                let made = |f: &crate::topo::FaceId| {
                    out.face(*f).names.iter().any(|n| n.feature == "\u{1}fil")
                };
                if !faces.iter().any(made) {
                    continue;
                }
                for k in 1..8 {
                    let p = c.eval(x.range.0 + (x.range.1 - x.range.0) * k as f64 / 8.0);
                    if p.dist(tip) > 5.0 {
                        continue;
                    }
                    let n: Vec<Vec3> = faces
                        .iter()
                        .map(|&f| {
                            let face = out.face(f);
                            let uv = face.surface.project(p);
                            let n = face.surface.normal(uv.x, uv.y);
                            if face.reversed { -n } else { n }
                        })
                        .collect();
                    assert!((n[0] - n[1]).norm() < 1e-9, "{radii:?}: a crease at {p:?}");
                    smooth += 1;
                }
            }
            assert!(smooth > 20, "{radii:?}: {smooth} points looked at");
            let frame = Frame::new(Point3::new(9.5, 9.5, 7.0), Vec3::Z, Vec3::X).unwrap();
            let zone = extrude(&Profile::rect(Vec2::ZERO, 5.0, 5.0), &frame, 6.0, "k").unwrap();
            let k = 1.0 - PI / 4.0;
            let outside = measure::volume(&boolean::difference(&out, &zone).unwrap());
            let want = 1728.0 - 125.0 - radii.iter().map(|r| k * r * r * 7.0).sum::<f64>();
            assert!(
                (outside - want).abs() < 1e-7 * want,
                "{radii:?}: {outside} against {want}"
            );
            let at = |r: f64| 1728.0 - 3.0 * k * r * r * (12.0 - r) - r.powi(3) * (1.0 - PI / 6.0);
            let (lo, hi) = radii
                .iter()
                .fold((f64::INFINITY, 0.0f64), |(a, b), &x| (a.min(x), b.max(x)));
            let v = measure::volume(&out);
            assert!(
                v > at(hi) && v < at(lo),
                "{radii:?}: {v} in {} to {}",
                at(hi),
                at(lo)
            );
        }
    }
}
