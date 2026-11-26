// yack: Lofts: a solid through a sequence of planar sections, each side face a B-spline surface skinned
// Adapted from springrep@a03e8a1 src/loft/mod.rs (MIT, (c) 2026 Springcraft, Inc.).
// Changes: moved from springrep's src/loft/mod.rs to yack-build (M3 W7); paths and visibility only;
// M3 hardening (w-h3): the loft walks its sections and columns by iterators and checked look-ups
// (`rail_at`, `ring_at`, `row_edges`) instead of grid indexing, each loop's pieces with the next;
// pieces that do not match are an error, not a panic. Every entity is added in the same order.
//! Lofts: a solid through a sequence of planar sections, each side face a B-spline surface skinned
//! across them (a plane where a ruled side is flat). Faces are `start` and `end` on the first and
//! last sections and `side` for the rest.
//!
//! Sections are matched by one rule: outer
//! loops turned to run the same way about the loft, edge `i` to edge `i` when every section has as
//! many, otherwise every section split at the polar angles of every other's vertices. Each column
//! of matched edges is brought to one degree and knot vector, and its control points are carried
//! across the sections: linearly when ruled or with two sections, otherwise by the lowest-degree
//! Bézier through the first and last sections that fits the others within the linear tolerance.

pub mod rail;
#[cfg(test)]
mod tests;

use std::f64::consts::TAU;

use rail::{Across, Hom, Rail, point};

use crate::{
    Error, Result, check,
    error::invalid,
    geom::{Curve, Line, NurbsCurve, NurbsSurface, Plane, Surface},
    math::{Frame, Point3, Vec2, Vec3},
    profile::{Names, Profile, ProfileLoop},
    tol,
    topo::{Brep, CoedgeSpec, EdgeId, FaceName, PCurve, VertexId},
};

/// The highest degree a smooth loft takes across its sections.
const MAX_ACROSS_DEGREE: usize = 8;

/// One section: a profile on a plane. Only its outer loop is lofted.
#[derive(Clone, Debug)]
pub struct Section {
    pub profile: Profile,
    pub frame: Frame,
}

/// A stretch of a 3-D curve, from parameter `t0` to `t1` in either order.
#[derive(Clone, Debug)]
struct Piece {
    curve: Curve,
    t0: f64,
    t1: f64,
    straight: bool,
    /// The names of the profile edge this is, or is a part of.
    names: Names,
}

impl Piece {
    fn param(&self, s: f64) -> f64 {
        self.t0 + (self.t1 - self.t0) * s
    }

    fn at(&self, s: f64) -> Point3 {
        self.curve.eval(self.param(s))
    }

    fn split(&self, s: f64) -> (Piece, Piece) {
        let tm = self.param(s);
        (
            Piece {
                t1: tm,
                ..self.clone()
            },
            Piece {
                t0: tm,
                ..self.clone()
            },
        )
    }
}

/// A section's outer loop in 3-D, running counter-clockwise about the loft direction.
struct Ring {
    pieces: Vec<Piece>,
    /// The section plane, `z` along the loft.
    frame: Frame,
    center: Point3,
}

fn centroid(lp: &ProfileLoop) -> Vec2 {
    let p = lp.polygon();
    let (mut a, mut cx, mut cy) = (0.0, 0.0, 0.0);
    for (&q, &r) in round(&p) {
        let c = q.cross(r);
        a += c;
        cx += (q.x + r.x) * c;
        cy += (q.y + r.y) * c;
    }
    Vec2::new(cx / (3.0 * a), cy / (3.0 * a))
}

fn outer(s: &Section) -> Result<ProfileLoop> {
    let p = s.profile.normalized()?;
    // A normalized profile has a loop.
    let no_loop = || Error::InvalidInput("a loft section with no loop".into());
    p.loops.into_iter().next().ok_or_else(no_loop)
}

fn ring(s: &Section, lp: &ProfileLoop, along: Vec3) -> Result<Ring> {
    let mut lp = lp.clone();
    let c = centroid(&lp);
    let mut frame = s.frame;
    if frame.z.dot(along) < 0.0 {
        lp.reverse();
        frame = Frame::new(frame.origin, -frame.z, frame.x)?;
    }
    let pieces = lp
        .segments
        .iter()
        .map(|seg| {
            let placed = seg.segment.place(&s.frame);
            let (t0, t1) = if placed.forward {
                placed.range
            } else {
                (placed.range.1, placed.range.0)
            };
            Piece {
                straight: matches!(placed.curve, Curve::Line(_)),
                names: seg.names.clone(),
                curve: placed.curve,
                t0,
                t1,
            }
        })
        .collect();
    Ok(Ring {
        pieces,
        frame,
        center: s.frame.at(c.x, c.y),
    })
}

/// A vertex angle closer than this to another is the same angle.
const SAME_ANGLE: f64 = 1e-9;

fn angle_dist(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(TAU);
    d.min(TAU - d)
}

impl Ring {
    /// The polar angle of `p` about the section's centre, from `x0` projected into its plane.
    fn angle(&self, p: Point3, x0: Vec3) -> f64 {
        let z = self.frame.z;
        let x = (x0 - z * x0.dot(z)).unit().unwrap_or(self.frame.x);
        let y = z.cross(x);
        let d = p - self.center;
        d.dot(y).atan2(d.dot(x)).rem_euclid(TAU)
    }

    /// Splits the pieces at each of `angles` that is not already a vertex. Each piece must turn
    /// steadily forward about the centre.
    fn split_at(&mut self, angles: &[f64], x0: Vec3) -> Result<()> {
        let mut out = Vec::new();
        for piece in &self.pieces {
            let a0 = self.angle(piece.at(0.0), x0);
            let samples = 64;
            let mut prev = 0.0;
            let mut turn = Vec::with_capacity(samples + 1);
            turn.push(0.0);
            for i in 1..=samples {
                let a = self.angle(piece.at(i as f64 / samples as f64), x0);
                let mut d = (a - a0).rem_euclid(TAU);
                if i == samples && d < SAME_ANGLE {
                    d = TAU;
                }
                if d + SAME_ANGLE < prev {
                    return Err(Error::Unsupported(
                        "lofting sections with different edge counts that are not star-shaped \
                         about their centres"
                            .into(),
                    ));
                }
                prev = d;
                turn.push(d);
            }
            let total = prev;
            let mut cuts: Vec<f64> = angles
                .iter()
                .map(|&a| (a - a0).rem_euclid(TAU))
                .filter(|&d| d > SAME_ANGLE && d < total - SAME_ANGLE)
                .collect();
            cuts.sort_by(f64::total_cmp);
            let mut rest = piece.clone();
            let mut done = 0.0;
            for d in cuts {
                let i = turn.partition_point(|&t| t < d).clamp(1, samples);
                let (mut lo, mut hi) = ((i - 1) as f64 / samples as f64, i as f64 / samples as f64);
                for _ in 0..80 {
                    let mid = 0.5 * (lo + hi);
                    let dm = (self.angle(piece.at(mid), x0) - a0).rem_euclid(TAU);
                    if dm < d { lo = mid } else { hi = mid }
                }
                let s = 0.5 * (lo + hi);
                let local = (s - done) / (1.0 - done);
                let (a, b) = rest.split(local);
                out.push(a);
                rest = b;
                done = s;
            }
            out.push(rest);
        }
        self.pieces = out;
        Ok(())
    }

    fn starts(&self) -> Vec<Point3> {
        self.pieces.iter().map(|p| p.at(0.0)).collect()
    }
}

/// Matches every ring's pieces one to one, in order.
fn match_rings(rings: &mut [Ring]) -> Result<()> {
    let Some(first_ring) = rings.first() else { return Ok(()) };
    let k = first_ring.pieces.len();
    if rings.iter().all(|r| r.pieces.len() == k) {
        // Vertices about their section's centre, read two ways, and the shift that fits either
        // best wins: along world axes (one outline drawn on planes whose axes differ, or on a
        // plane facing back down the loft), or along the section's own axes (a section carried
        // with its frame, as a helical gear's tooth space turned about the gear's axis, moves
        // further between sections than it is wide). On a tie the world reading wins, as it turns
        // the loft least.
        let offsets = |r: &Ring| -> (Vec<Vec3>, Vec<Vec3>) {
            r.starts()
                .into_iter()
                .map(|p| {
                    let d = p - r.center;
                    (d, Vec3::new(d.dot(r.frame.x), d.dot(r.frame.y), 0.0))
                })
                .unzip()
        };
        for i in 1..rings.len() {
            // Each ring is matched against the one before as that one was turned.
            let (Some(before), Some(this)) = (rings.get(i - 1), rings.get(i)) else { break };
            let (prev_w, prev_l) = offsets(before);
            let (cur_w, cur_l) = offsets(this);
            let mut costs = Vec::with_capacity(2 * k);
            for (prev, cur) in [(&prev_w, &cur_w), (&prev_l, &cur_l)] {
                for shift in 0..k {
                    let shifted = cur.iter().cycle().skip(shift);
                    let c: f64 = prev.iter().zip(shifted).map(|(&p, &q)| (p - q).norm()).sum();
                    costs.push(c);
                }
            }
            let least = costs.iter().copied().fold(f64::INFINITY, f64::min);
            let best = costs
                .iter()
                .position(|&c| c <= least + k as f64 * tol::LINEAR)
                .map_or(0, |at| at % k);
            if let Some(this) = rings.get_mut(i) {
                this.pieces.rotate_left(best);
            }
        }
        return Ok(());
    }
    let x0 = first_ring.frame.x;
    let mut all: Vec<f64> = rings
        .iter()
        .flat_map(|r| r.starts().into_iter().map(|p| r.angle(p, x0)))
        .collect();
    all.sort_by(f64::total_cmp);
    let mut angles: Vec<f64> = Vec::new();
    for a in all {
        if angles.iter().all(|&b| angle_dist(a, b) > SAME_ANGLE) {
            angles.push(a);
        }
    }
    let first = match (rings.first(), rings.first().and_then(|r| r.pieces.first())) {
        (Some(r), Some(piece)) => r.angle(piece.at(0.0), x0),
        _ => return Err(unmatched()),
    };
    for r in rings.iter_mut() {
        r.split_at(&angles, x0)?;
        if r.pieces.len() != angles.len() {
            return Err(Error::Numerical(
                "loft sections did not split into matching edges".into(),
            ));
        }
        let at = r
            .starts()
            .iter()
            .position(|&p| angle_dist(r.angle(p, x0), first) <= SAME_ANGLE)
            .ok_or_else(|| Error::Numerical("loft sections lost a matching vertex".into()))?;
        r.pieces.rotate_left(at);
    }
    Ok(())
}

/// The section parameters by chord length over every control point.
fn chord_params(columns: &[Vec<Rail>], n: usize) -> Vec<f64> {
    let mut acc = vec![0.0];
    for s in 1..n {
        let d: f64 = columns
            .iter()
            .map(|col| match (col.get(s), col.get(s - 1)) {
                (Some(here), Some(before)) => here
                    .pw
                    .iter()
                    .zip(&before.pw)
                    .map(|(&a, &b)| point(a).dist(point(b)))
                    .sum::<f64>(),
                _ => 0.0,
            })
            .sum();
        let so_far = acc.last().copied().unwrap_or_default();
        acc.push(so_far + d);
    }
    let total = acc.last().copied().unwrap_or_default();
    if total > 0.0 {
        acc.iter().map(|a| a / total).collect()
    } else {
        (0..n).map(|s| s as f64 / (n - 1) as f64).collect()
    }
}

/// How the control points run across the sections of a smooth loft: the lowest-degree Bézier
/// that passes within the linear tolerance of every section, or a cubic through them all.
fn across(columns: &[Vec<Rail>], n: usize) -> Result<Across> {
    let params = chord_params(columns, n);
    if n == 2 {
        return rail::bezier_fit(&params, 1);
    }
    for p in 2..=MAX_ACROSS_DEGREE.min(n - 1) {
        let fit = rail::bezier_fit(&params, p)?;
        let mut worst: f64 = 0.0;
        for col in columns {
            // A column's rails are compatible: each has its first's number of control points.
            let controls = col.first().map_or(0, |r| r.pw.len());
            for m in 0..controls {
                let data: Vec<Hom> = col.iter().filter_map(|r| r.pw.get(m).copied()).collect();
                let ctrl = fit.apply(&data);
                for (&v, &d) in params.iter().zip(&data) {
                    worst = worst.max(point(fit.eval(&ctrl, v)).dist(point(d)));
                }
            }
        }
        if worst <= tol::LINEAR {
            return Ok(fit);
        }
    }
    rail::interpolation(&params)
}

fn affine(origin: Vec2, dir: Vec2) -> PCurve {
    PCurve::Affine { origin, dir }
}

fn projected(frame: &Frame, p: Point3) -> PCurve {
    let l = frame.to_local(p);
    PCurve::Projected {
        seed: Vec2::new(l.x, l.y),
    }
}

fn coedge(edge: EdgeId, reversed: bool, pcurve: PCurve) -> CoedgeSpec {
    CoedgeSpec {
        edge,
        reversed,
        pcurve,
    }
}

/// How an edge's parameter maps onto a side face's `u` or `v`: `scale · t`.
#[derive(Clone, Copy)]
struct EdgeParam {
    id: EdgeId,
    scale: f64,
}

/// The plane through two matched straight edges, `x` along the loop and `z` outward, when they
/// are coplanar.
fn flat_side(a0: Point3, b0: Point3, a1: Point3, b1: Point3) -> Option<Frame> {
    let x = (b0 - a0).unit()?;
    let w = a1 - a0;
    let y = (w - x * w.dot(x)).unit()?;
    let z = x.cross(y);
    let off = (b1 - a0).dot(z).abs().max((b1 - a1).dot(z).abs());
    (off <= tol::LINEAR).then_some(Frame {
        origin: a0,
        x,
        y,
        z,
    })
}

/// A side face's names: `side`, and the names of the first section's edge it grows from.
fn side_names(feature: &str, edge: &Names) -> Vec<FaceName> {
    let mut names = vec![FaceName::new(feature, FaceName::SIDE)];
    names.extend(FaceName::all(
        feature,
        edge.iter().filter(|n| !n.is_empty()),
    ));
    names
}

/// Lofts through `sections` in order, ruled (straight between consecutive sections) or smooth. The
/// first section's outline is the `start` face and the last one's the `end` face. Every face
/// between is `side`, and also carries the name of the first section's edge it grows from; where
/// sections with different edge counts split that edge, each part carries its name.
pub fn loft(sections: &[Section], ruled: bool, feature: &str) -> Result<Brep> {
    let n = sections.len();
    let (Some(first_section), true) = (sections.first(), n >= 2) else {
        return invalid("a loft needs at least two sections");
    };
    let loops: Vec<ProfileLoop> = sections.iter().map(outer).collect::<Result<_>>()?;
    let centers: Vec<Point3> = sections
        .iter()
        .zip(&loops)
        .map(|(s, l)| {
            let c = centroid(l);
            s.frame.at(c.x, c.y)
        })
        .collect();
    let (Some(&first_center), Some(&last_center)) = (centers.first(), centers.last()) else {
        return invalid("a loft needs at least two sections");
    };
    let Some(along) = (last_center - first_center).unit() else {
        return invalid("a loft's first and last sections share a centre");
    };
    if sections.iter().any(|s| s.frame.z.dot(along).abs() <= tol::ANGULAR) {
        return invalid("a loft section's plane contains the loft direction");
    }
    // Sections on parallel planes must come in order across them: one that steps back, or sits on
    // its neighbour's plane, folds the sides over and covers part of the solid twice.
    let normal = first_section.frame.z;
    if sections.iter().all(|s| s.frame.z.cross(normal).norm() <= tol::ANGULAR) {
        let offsets: Vec<f64> = sections
            .iter()
            .map(|s| (s.frame.origin - first_section.frame.origin).dot(normal))
            .collect();
        let steps = || offsets.iter().zip(offsets.iter().skip(1));
        let rises = steps().all(|(a, b)| b - a > tol::LINEAR);
        let falls = steps().all(|(a, b)| a - b > tol::LINEAR);
        if !rises && !falls {
            return invalid("a loft's parallel sections do not come in order across their planes");
        }
    }
    let mut rings: Vec<Ring> = sections
        .iter()
        .zip(&loops)
        .map(|(s, l)| ring(s, l, along))
        .collect::<Result<_>>()?;
    match_rings(&mut rings)?;
    let k = rings.first().map_or(0, |r| r.pieces.len());

    // Each column of matched pieces, one per section, at one degree on one knot vector.
    let mut columns: Vec<Vec<Rail>> = Vec::with_capacity(k);
    for j in 0..k {
        let mut col: Vec<Rail> = rings
            .iter()
            .map(|r| {
                let p = r.pieces.get(j).ok_or_else(unmatched)?;
                Rail::of_curve(&p.curve, p.t0, p.t1)
            })
            .collect::<Result<_>>()?;
        rail::make_compatible(&mut col);
        columns.push(col);
    }
    // Section `s`'s ring.
    let ring_at = |s: usize| rings.get(s).ok_or_else(unmatched);

    // The sections that bound bands of side faces, and how each band runs across its sections.
    let bands: Vec<(usize, usize, Across)> = if ruled || n == 2 {
        let lin = rail::bezier_fit(&[0.0, 1.0], 1)?;
        (0..n - 1).map(|s| (s, s + 1, lin.clone())).collect()
    } else {
        vec![(0, n - 1, across(&columns, n)?)]
    };
    let rows: Vec<usize> = if ruled || n == 2 { (0..n).collect() } else { vec![0, n - 1] };

    let mut b = Brep::new();
    // The vertices of each section that bounds a band.
    let mut verts: Vec<Vec<VertexId>> = vec![Vec::new(); n];
    for &r in &rows {
        let starts = ring_at(r)?.starts();
        if let Some(row) = verts.get_mut(r) {
            *row = starts.into_iter().map(|p| b.add_vertex(p)).collect();
        }
    }
    // Section edges, each with how its parameter maps onto its column's `u`.
    let mut sec_edges: Vec<Vec<EdgeParam>> = vec![Vec::new(); n];
    for &r in &rows {
        let row_verts = verts.get(r).ok_or_else(unmatched)?;
        let pieces = &ring_at(r)?.pieces;
        let mut row = Vec::with_capacity(k);
        for ((col, piece), (&va, &vb)) in columns.iter().zip(pieces).zip(round(row_verts)) {
            let rl = rail_at(col, r)?;
            let ep = if piece.straight {
                let (line, len) = Line::through(rl.start(), rl.end()).ok_or_else(|| {
                    Error::InvalidInput("a loft section has a zero-length edge".into())
                })?;
                EdgeParam {
                    id: b.add_edge(Curve::Line(line), (0.0, len), va, vb),
                    scale: 1.0 / len,
                }
            } else {
                let c = nurbs_curve(rl.degree, rl.knots.clone(), &rl.pw)?;
                let id = b.add_edge(Curve::Nurbs(Box::new(c)), (0.0, 1.0), va, vb);
                EdgeParam { id, scale: 1.0 }
            };
            row.push(ep);
        }
        if let Some(slot) = sec_edges.get_mut(r) {
            *slot = row;
        }
    }
    let row_edges = |s: usize| sec_edges.get(s).filter(|row| row.len() == k).ok_or_else(unmatched);
    let first_pieces = &ring_at(0)?.pieces;

    let mut faces = Vec::new();
    for (lo, hi, acr) in &bands {
        let (lo, hi) = (*lo, *hi);
        let picked: Vec<usize> = (lo..=hi).collect();
        let linear = acr.degree == 1;
        let (lo_verts, hi_verts) =
            (verts.get(lo).ok_or_else(unmatched)?, verts.get(hi).ok_or_else(unmatched)?);
        // Side edges at each vertex, running from section `lo` to section `hi`.
        let mut sides: Vec<EdgeParam> = Vec::with_capacity(k);
        for (col, (&va, &vb)) in columns.iter().zip(lo_verts.iter().zip(hi_verts)) {
            let ep = if linear {
                let (p0, p1) = (rail_at(col, lo)?.start(), rail_at(col, hi)?.start());
                let (line, len) = Line::through(p0, p1)
                    .ok_or_else(|| Error::InvalidInput("loft sections share a vertex".into()))?;
                EdgeParam {
                    id: b.add_edge(Curve::Line(line), (0.0, len), va, vb),
                    scale: 1.0 / len,
                }
            } else {
                let data: Vec<Hom> = picked
                    .iter()
                    .map(|&s| rail_at(col, s)?.pw.first().copied().ok_or_else(unmatched))
                    .collect::<Result<_>>()?;
                let c = nurbs_curve(acr.degree, acr.knots.clone(), &acr.apply(&data))?;
                let id = b.add_edge(Curve::Nurbs(Box::new(c)), (0.0, 1.0), va, vb);
                EdgeParam { id, scale: 1.0 }
            };
            sides.push(ep);
        }
        let (lo_pieces, hi_pieces) = (&ring_at(lo)?.pieces, &ring_at(hi)?.pieces);
        let rims = row_edges(lo)?.iter().zip(row_edges(hi)?);
        let straight = lo_pieces.iter().zip(hi_pieces).map(|(a, b)| a.straight && b.straight);
        let columns_here = columns.iter().zip(first_pieces).zip(straight);
        for (((col, first_piece), straight), ((&bottom, &top), (&left, &right))) in
            columns_here.zip(rims.zip(round(&sides)))
        {
            let (rl, rh) = (rail_at(col, lo)?, rail_at(col, hi)?);
            let flat = (linear && straight)
                .then(|| flat_side(rl.start(), rl.end(), rh.start(), rh.end()))
                .flatten();
            let names = side_names(feature, &first_piece.names);
            let face = if let Some(fr) = flat {
                let face = b.add_face(Surface::Plane(Plane { frame: fr }), false, names);
                b.add_loop(
                    face,
                    vec![
                        coedge(bottom.id, false, projected(&fr, rl.start())),
                        coedge(right.id, false, projected(&fr, rl.end())),
                        coedge(top.id, true, projected(&fr, rh.end())),
                        coedge(left.id, true, projected(&fr, rh.start())),
                    ],
                );
                face
            } else {
                let surface = skin(col, &picked, acr)?;
                let face = b.add_face(surface, false, names);
                b.add_loop(
                    face,
                    vec![
                        coedge(bottom.id, false, affine(Vec2::ZERO, Vec2::new(bottom.scale, 0.0))),
                        coedge(
                            right.id,
                            false,
                            affine(Vec2::new(1.0, 0.0), Vec2::new(0.0, right.scale)),
                        ),
                        coedge(
                            top.id,
                            true,
                            affine(Vec2::new(0.0, 1.0), Vec2::new(top.scale, 0.0)),
                        ),
                        coedge(left.id, true, affine(Vec2::ZERO, Vec2::new(0.0, left.scale))),
                    ],
                );
                face
            };
            faces.push(face);
        }
    }

    for (r, name, reversed) in [(0, FaceName::START, true), (n - 1, FaceName::END, false)] {
        let fr = ring_at(r)?.frame;
        let face = b.add_face(
            Surface::Plane(Plane { frame: fr }),
            reversed,
            [FaceName::new(feature, name)],
        );
        let mut lp = columns
            .iter()
            .zip(row_edges(r)?)
            .map(|(col, e)| {
                let rl = rail_at(col, r)?;
                let at = if reversed { rl.end() } else { rl.start() };
                Ok(coedge(e.id, reversed, projected(&fr, at)))
            })
            .collect::<Result<Vec<_>>>()?;
        if reversed {
            lp.reverse();
        }
        b.add_loop(face, lp);
        faces.push(face);
    }
    let shell = b.add_shell(faces);
    b.add_solid(vec![shell]);
    let issues = check::check(&b);
    if issues.is_empty() {
        Ok(b)
    } else {
        Err(Error::InvalidResult(issues.into_iter().map(|i| i.to_string()).collect()))
    }
}

/// A column's rail at section `s`.
fn rail_at(col: &[Rail], s: usize) -> Result<&Rail> {
    col.get(s).ok_or_else(unmatched)
}

/// The error of sections whose pieces do not match one to one, which `match_rings` rules out.
fn unmatched() -> Error {
    Error::Numerical("loft sections did not split into matching edges".into())
}

/// Each item with the one after it round the loop, the last with the first.
fn round<T>(items: &[T]) -> impl Iterator<Item = (&T, &T)> {
    items.iter().zip(items.iter().cycle().skip(1))
}

fn split_hom(pw: &[Hom]) -> (Vec<Point3>, Option<Vec<f64>>) {
    let pts = pw.iter().map(|&h| point(h)).collect();
    let w: Vec<f64> = pw.iter().map(|h| h[3]).collect();
    let rational = w.iter().any(|&x| (x - 1.0).abs() > 1e-15);
    (pts, rational.then_some(w))
}

fn nurbs_curve(degree: usize, knots: Vec<f64>, pw: &[Hom]) -> Result<NurbsCurve> {
    if pw.iter().any(|h| h[3].is_nan() || h[3] <= 0.0) {
        return Err(Error::Numerical(
            "a loft edge came out with a weight that is not positive".into(),
        ));
    }
    let (pts, w) = split_hom(pw);
    NurbsCurve::new(degree, knots, pts, w)
}

/// How far a loft face's surface runs on past each of its edges, as a fraction of its parameter
/// range. Intersections that end on an edge are then traced and fitted across it, as on a swept
/// surface, rather than stopping where the surface does.
const RUN_ON: f64 = 0.1;

/// The surface through one column's rails at the `picked` sections, `u` along the rails and `v`
/// across the sections, carried on past every edge that is not a seam.
fn skin(col: &[Rail], picked: &[usize], acr: &Across) -> Result<Surface> {
    let first = col.first().ok_or_else(unmatched)?;
    let closed = first.start().dist(first.end()) <= tol::LINEAR;
    let rails: Vec<(Vec<f64>, Vec<Hom>)> = picked
        .iter()
        .map(|&s| {
            let r = col.get(s).ok_or_else(unmatched)?;
            Ok(if closed {
                (r.knots.clone(), r.pw.clone())
            } else {
                rail::extended(r.degree, &r.knots, &r.pw, RUN_ON)
            })
        })
        .collect::<Result<_>>()?;
    let (ku, nu) = rails.first().map(|(k, pw)| (k.clone(), pw.len())).ok_or_else(unmatched)?;
    let mut kv = Vec::new();
    let mut net: Vec<Hom> = Vec::new();
    for m in 0..nu {
        let data: Vec<Hom> = rails.iter().filter_map(|r| r.1.get(m).copied()).collect();
        let (k, ctrl) = rail::extended(acr.degree, &acr.knots, &acr.apply(&data), RUN_ON);
        kv = k;
        net.extend(ctrl);
    }
    let nv = net.len() / nu;
    if net.iter().any(|h| h[3].is_nan() || h[3] <= 0.0) {
        return Err(Error::Numerical(
            "a loft surface came out with a weight that is not positive".into(),
        ));
    }
    let (pts, w) = split_hom(&net);
    let s = NurbsSurface::new((first.degree, acr.degree), (ku, kv), (nu, nv), pts, w)?;
    Ok(Surface::Nurbs(Box::new(s)))
}
