// yack: The corner where three fillets of unequal radii meet, closed without a crease.
// Adapted from springrep@a03e8a1 src/blend/shaped/uneven.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/blend/shaped/uneven.rs to yack-blend (M3 W9a); paths and visibility only.
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
        let (p, q) = (self.a[(j + 1) % 3], self.a[(j + 2) % 3]);
        (p + q) / (1.0 + p.dot(q))
    }

    /// How far along edge `j` the ball touching all three faces sits, for each unit of its
    /// radius.
    fn along(&self, j: usize) -> f64 {
        (1.0 - self.in_from(j).dot(self.a[j])) / self.e[j].dot(self.a[j])
    }

    /// The centre of the ball of the largest radius touching all three faces.
    fn centre(&self) -> Point3 {
        self.at + (self.in_from(0) + self.e[0] * self.along(0)) * self.largest()
    }

    fn shrinks(&self, j: usize) -> bool {
        self.largest() - self.r[j] > tol::LINEAR
    }

    /// How far along edge `j` its own fillet starts: where the corner's ball sits, and its
    /// radius on.
    fn start(&self, j: usize) -> f64 {
        self.largest() * (self.along(j) + 1.0)
    }

    /// The centre of edge `j`'s fillet's first ball.
    fn first(&self, j: usize) -> Point3 {
        self.at + self.e[j] * self.start(j) + self.in_from(j) * self.r[j]
    }

    /// What the ball sweeps from the corner along edge `j`: its unit axis toward the corner's
    /// ball, and its half angle, nought where the ball keeps its radius.
    fn cone(&self, j: usize) -> (Vec3, f64) {
        let run = self.centre() - self.first(j);
        (
            run.unit_or_zero(),
            ((self.largest() - self.r[j]) / run.norm()).asin(),
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
            if !normals.iter().any(|m| m.dot(n) > 1.0 - 1e-9) {
                normals.push(n);
            }
        }
        let straight = here
            .iter()
            .all(|x| matches!(s.edge(x.0).curve, Some(Curve::Line(_))));
        if !flat || !straight || normals.len() != 3 {
            continue;
        }
        let far = |e: EdgeId| {
            let [p, q] = ends(e);
            s.vertex(if p == v { q } else { p }).point
        };
        // Edge j is the one that lies in neither of the faces after it: it leaves face j.
        let mut c = Corner {
            vertex: v,
            at,
            a: [-normals[0], -normals[1], -normals[2]],
            e: [Vec3::ZERO; 3],
            len: [0.0; 3],
            r: [0.0; 3],
            edges: [here[0].0; 3],
        };
        let mut placed = 0;
        for &(e, r) in &here {
            let run = far(e) - at;
            let Some(d) = run.unit() else { continue };
            let leaves: Vec<usize> = (0..3).filter(|&j| d.dot(c.a[j]).abs() > 1e-9).collect();
            if let [j] = leaves[..]
                && d.dot(c.a[j]) > 0.0
            {
                (c.e[j], c.len[j], c.r[j], c.edges[j]) = (d, run.norm(), r, e);
                placed |= 1 << j;
            }
        }
        if placed == 7 && (0..3).all(|j| c.len[j] > c.start(j) + c.r[j]) {
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

    fn line(&mut self, from: usize, to: usize) -> Result<Side> {
        let (p, q) = (self.parts.points[from], self.parts.points[to]);
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
        let (curve, mid) = arc(centre, self.parts.points[from], self.parts.points[to])?;
        Ok(self.side(curve, from, to, mid))
    }

    /// A face on `surface` bounded by `sides`, each running on from the last, either way round.
    fn face(&mut self, surface: Surface, name: String, mut sides: Vec<Side>) {
        let mut ring = Vec::new();
        for s in &sides {
            ring.push(self.parts.points[s.from]);
            ring.push(s.mid);
        }
        let middle = ring.iter().fold(Vec3::ZERO, |sum, &p| sum + p) / ring.len() as f64;
        let turn = (0..ring.len()).fold(Vec3::ZERO, |sum, k| {
            sum + (ring[k] - middle).cross(ring[(k + 1) % ring.len()] - middle)
        });
        if turn.dot(self.out) < 0.0 {
            sides.reverse();
            for s in &mut sides {
                (s.from, s.to) = (s.to, s.from);
            }
        }
        let uv = surface.project(sides[0].mid);
        let reversed = surface.normal(uv.x, uv.y).dot(self.out) < 0.0;
        let uses = sides
            .iter()
            .map(|s| (s.edge, self.parts.edges[s.edge].1 != s.from))
            .collect();
        self.parts.faces.push(PartFace {
            surface,
            reversed,
            loops: vec![uses],
            names: BTreeSet::from([FaceName::new("\u{1}fil", name)]),
            shell: self.shell,
        });
    }
}

/// `parts`, a solid whose corner `c` was cut away as far as each edge's fillet starts, with that
/// corner closed; `None` where the fillets do not end there as fillets of their radii do.
fn closed(parts: Parts, c: &Corner, at: usize) -> Result<Option<Parts>> {
    let big = c.largest();
    let centre = c.centre();
    let foot = |ball: Point3, rho: f64, face: usize| ball - c.a[face] * rho;
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
        let (Some(p), Some(q)) = (
            b.found(foot(first, c.r[j], next(j))),
            b.found(foot(first, c.r[j], prev(j))),
        ) else {
            return Ok(None);
        };
        ends[j] = [p, q];
        let Some(e) = b.between(p, q) else {
            return Ok(None);
        };
        round[j] = e;
        // On face j, its edges' cuts cross where each is as far along as its fillet starts.
        let (u, v) = (c.e[next(j)], c.e[prev(j)]);
        let (s, t) = (c.start(next(j)), c.start(prev(j)));
        let k = u.dot(v);
        let p = c.at + u * ((s - k * t) / (1.0 - k * k)) + v * ((t - k * s) / (1.0 - k * k));
        let Some(x) = b.found(p) else {
            return Ok(None);
        };
        cross[j] = x;
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
    b.shell = b.parts.faces[cut[0]].shell;
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
        let (p, q) = (b.parts.points[ends[j][0]], b.parts.points[ends[j][1]]);
        let own = Side {
            edge: round[j],
            from: ends[j][0],
            to: ends[j][1],
            mid: c.first(j) + ((p + q) * 0.5 - c.first(j)).unit_or_zero() * c.r[j],
        };
        let (axis, half) = c.cone(j);
        let first = c.first(j);
        let small = match c.shrinks(j) {
            true => b.arc(first - axis * (c.r[j] * half.sin()), ends[j][0], ends[j][1])?,
            false => own,
        };
        let large = b.arc(
            centre - axis * (big * half.sin()),
            feet[next(j)],
            feet[prev(j)],
        )?;
        let lines = [
            b.line(ends[j][0], feet[next(j)])?,
            b.line(ends[j][1], feet[prev(j)])?,
        ];
        let back = |s: Side| Side {
            from: s.to,
            to: s.from,
            ..s
        };
        let away = c.in_from(j);
        let sweep = if c.shrinks(j) {
            b.face(
                Surface::Sphere(Sphere {
                    frame: Frame::new(first, c.e[j], away)?,
                    radius: c.r[j],
                }),
                format!("corner{at}_cap{j}"),
                vec![own, back(small)],
            );
            Surface::Cone(Cone {
                frame: Frame::new(first - axis * (c.r[j] * half.sin()), axis, away)?,
                radius: c.r[j] * half.cos(),
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
        );
        sides.push(large);
        touch.push(lines);
    }
    let inward = centre - c.at;
    b.face(
        Surface::Sphere(Sphere {
            frame: Frame::new(centre, inward.cross(c.e[0]), inward)?,
            radius: big,
        }),
        format!("corner{at}_ball"),
        vec![sides[2], sides[0], sides[1]],
    );
    // Each face's corner the cut took comes back as far as the lines its two cones touch it
    // along: on face i those of the edge before it, then the edge after it.
    for i in 0..3 {
        let (Some(l1), Some(l2)) = (
            b.between(ends[prev(i)][0], cross[i]),
            b.between(ends[next(i)][1], cross[i]),
        ) else {
            return Ok(None);
        };
        let Some(f) = b.parts.faces.iter().position(|f| {
            let has = |e: usize| f.loops.iter().flatten().any(|u| u.0 == e);
            has(l1) && has(l2)
        }) else {
            return Ok(None);
        };
        let (a, z) = (touch[prev(i)][0], touch[next(i)][1]);
        let edges = &b.parts.edges;
        for lp in &mut b.parts.faces[f].loops {
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
            let uses: Vec<(usize, bool)> = run
                .iter()
                .map(|s| (s.edge, edges[s.edge].1 != s.from))
                .collect();
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
                .filter_map(|c| (0..3).find(|&j| c.edges[j] == e).map(|j| c.start(j)))
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
            let side = half_space(s, c.at + c.e[j] * c.start(j), -c.e[j])?;
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
